"""
Autonomous Warehouse Inventory Scanner, C27 sensor configuration.

Scans a warehouse using a UAV with a single front-facing high resolution
camera, three tracking cameras and a forward TOF sensor. No GPS is used at any
point: localization relies on visual odometry from the tracking cameras, fed to
the PX4 EKF2 estimator as an external vision source.

Because the scanning camera faces forward rather than sideways, the vehicle
must turn to face each shelf and fly sideways along it. Each shelf face
therefore needs its own pass, and the route is roughly twice as long as the
earlier two-camera design.

The warehouse floor plan is known in advance, so the route is planned rather
than discovered.

Outputs a JSON inventory mapping every detected QR code to an estimated 3D
position.
"""
import os
os.environ["PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION"] = "python"

import asyncio
import math
import json
import time
import cv2
import numpy as np
from collections import deque
from datetime import datetime
from mavsdk import System
from mavsdk.offboard import PositionNedYaw, OffboardError
import gz.transport13 as trans
from gz.msgs10.image_pb2 import Image
from gz.msgs10.laserscan_pb2 import LaserScan

# --- WAREHOUSE FLOOR PLAN ----------------------------------------------
#
# Read from layout.json rather than written here, so that flying a different
# warehouse is a new layout file and not an edit to this code. An earlier
# version kept these as constants while claiming in the README that they lived
# in a config file, which was not true of any of them.
LAYOUT_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                           "layout.json")
with open(LAYOUT_PATH, encoding="utf-8") as _handle:
    LAYOUT = json.load(_handle)


def _layout_path(key):
    """Resolve a path in layout.json relative to the layout file itself."""
    return os.path.normpath(os.path.join(os.path.dirname(LAYOUT_PATH),
                                         LAYOUT[key]))


# One entry per shelf face: the x of the shelf surface, and the heading the
# vehicle holds to look at it. A face is not derived from island geometry any
# more, because that assumed every shelf run has an aisle on both sides. The
# warehouse this flies in has two outer rows that do not.
AISLE_FACES = LAYOUT["aisle_faces"]

# The x of every scannable shelf face, used to snap an estimate to the grid.
SHELF_FACE_X = [f["face_x"] for f in AISLE_FACES]

# One altitude per shelf level, set so the camera sits level with the code.
FLIGHT_Z = LAYOUT["flight_z"]
Y_SOUTH, Y_NORTH = LAYOUT["y_south"], LAYOUT["y_north"]   # aisle end points
SPAWN_X, SPAWN_Y = LAYOUT["spawn_x"], LAYOUT["spawn_y"]   # NED origin

# base_link, and therefore the camera, rests this far above the floor when
# landed. Commanded altitudes are measured from the spawn point, world heights
# are not, so the two differ by exactly this.
GROUND_OFFSET = LAYOUT["ground_offset"]

# Headings in the MAVSDK convention: 0 is north, positive is clockwise.
YAW_EAST = 90.0
YAW_WEST = -90.0
YAW_NORTH = 0.0

# --- CAMERA GEOMETRY ---------------------------------------------------
# Must match the values in build_scanner_drone.py. Used to convert a pixel
# position into a bearing, which is how box positions are estimated.
CAMERA_HFOV_DEG = 60.0
# Aisle centre line to shelf face. A property of the warehouse, not the
# vehicle, so it comes from the layout.
SHELF_STANDOFF = LAYOUT["shelf_standoff"]

# --- SCAN PARAMETERS ---------------------------------------------------
WAYPOINT_TOLERANCE = 0.4      # metres
CRUISE_SPEED = 0.6             # m/s along an aisle
CLIMB_SPEED = 0.15             # m/s when changing shelf level
TURN_SETTLE_S = 3.0            # seconds held after a heading change
SETTLE_TOLERANCE = 0.05        # metres, how close before a leg counts as settled
SETTLE_TIMEOUT_S = 8.0         # seconds allowed for settling
TIMEOUT_MARGIN = 20.0          # seconds of slack added to expected leg time

# Vertical moves need their own, much slower rate. Measured with the horizontal
# rate applied to both: a 0.65 m level change left the vehicle 0.65 m behind
# the setpoint, and the error was still 0.35 m part way along the next aisle.
# The camera only tolerates 0.37 m of vertical error before a code leaves the
# frame, so this alone accounted for most of the missed reads.

# --- GAZEBO TOPICS -----------------------------------------------------
WORLD = LAYOUT["world"]
DRONE = LAYOUT["model"] + "_0"
CAM_HIRES = f"/world/{WORLD}/model/{DRONE}/link/camera_hires_link/sensor/camera/image"
CAM_DOWN = f"/world/{WORLD}/model/{DRONE}/link/camera_track_down_link/sensor/camera/image"
# The PMD TOF. Deliberately not on a link PX4 bridges, so this reading is ours
# and never reaches EKF2 as a height above ground.
TOF_SCAN = f"/world/{WORLD}/model/{DRONE}/link/tof_link/sensor/tof/scan"

MARKER_MAP_PATH = _layout_path("marker_map")

# Downward camera geometry, must match build_scanner_drone.py
DOWN_CAM_HFOV_DEG = 90.0

# ArUco markers on the floor carry known positions, so a sighting gives an
# absolute fix that can be compared against the dead-reckoned estimate.
# Must match the dictionary the world was generated with. A mismatch here is
# silent: the detector simply never matches, and on_down_image returns without
# a word, so drift correction quietly does nothing. It is a property of the
# warehouse, so it travels with the layout.
ARUCO_DICT = getattr(cv2.aruco, LAYOUT["aruco_dictionary"])

OUTPUT_JSON = _layout_path("output")
NAV_REPORT_JSON = _layout_path("navigation_report")
os.makedirs(os.path.dirname(OUTPUT_JSON), exist_ok=True)

# --- STATE -------------------------------------------------------------
current_pos = {"n": 0.0, "e": 0.0, "d": 0.0}
current_yaw = {"deg": 0.0}
origin_pos = None
is_settled = False
pending = deque()          # hits waiting to be recorded, each with its own pose
inventory = {}

# Drift correction state.
#
# PX4 does not expose a way to reset the estimator through MAVSDK, so the
# correction is kept here instead: an offset between where the estimator
# thinks the vehicle is and where a marker sighting says it is. Every setpoint
# has this offset applied, so the vehicle flies to the right place even though
# the estimator's own idea of position stays uncorrected.
drift_offset = {"n": 0.0, "e": 0.0}
marker_map = {}
marker_events = []
last_correction_time = [0.0]

# Forward clearance, from the TOF. The vehicle is half a metre across, so
# anything inside this band is closer than the airframe should ever be to a
# shelf it is scanning at SHELF_STANDOFF.
CLEARANCE_ALARM_M = 0.40
# How far the vehicle travels between clearance checks, and how far it turns to
# make one. Facing a shelf puts the direction of travel 90 degrees off the
# nose; the TOF cone reaches 53 degrees, so 50 degrees of turn brings the aisle
# ahead inside it with room to spare. The interval comes from the sensor's 5 m
# range less a margin, so nothing between checks goes unseen.
TOF_LOOK_EVERY_M = 4.0
TOF_LOOK_TURN_DEG = 50.0
# Half-width of the ray window used to read a look, centred on where the
# direction of travel ends up once the turn is made.
TOF_LOOK_WINDOW_DEG = 15.0
# A clearance below this during a look is an obstacle in the aisle rather than
# the shelf being scanned, which sits at SHELF_STANDOFF.
OBSTACLE_M = 2.0
# Only rays this far from level count as looking ahead. The cone is 86 degrees
# tall and its lower half always finds the floor; see nearest_ahead.
TOF_BAND_DEG = 10.0
# Anything nearer than this is the vehicle's own structure, measured at 0.101 m
# on rotor arms with the sensor's minimum range at 0.1 m.
TOF_SELF_M = 0.25
clearance = {"min_m": float("inf"), "last_m": float("inf"), "last_msg": None,
             "samples": 0,
             "alarms": 0, "in_alarm": False, "alarm_positions": []}

# Set while the vehicle is turned away from the shelf to check the aisle. The
# scanning camera is pointing somewhere its depth assumption does not hold, so
# frames are ignored rather than turned into positions.
scanning_paused = False

# Stretches of a leg that could not be scanned, with the reason.
coverage_gaps = []

# Obstacles the clearance checks found, with where along which leg.
obstacle_hits = []

# One ray profile is dumped per run, to calibrate which side of the cone the
# aisle falls on.
ray_profile = {"written": False}

qr_detector = cv2.QRCodeDetector()
aruco_detector = cv2.aruco.ArucoDetector(
    cv2.aruco.getPredefinedDictionary(ARUCO_DICT),
    cv2.aruco.DetectorParameters())


def ned_to_gazebo(n, e, d):
    """
    Convert a NED coordinate back into Gazebo world coordinates.

    The height needs GROUND_OFFSET added. NED heights are measured from the
    origin captured at startup, and the vehicle is on the ground then, with
    base_link already GROUND_OFFSET above the floor. World heights are not.
    """
    if origin_pos is None:
        return e + SPAWN_X, n + SPAWN_Y, -d + GROUND_OFFSET
    x = SPAWN_X + (e - origin_pos["e"])
    y = SPAWN_Y + (n - origin_pos["n"])
    return x, y, -d + GROUND_OFFSET


def gazebo_to_ned(x, y, z):
    """Inverse of ned_to_gazebo."""
    if origin_pos is None:
        return y - SPAWN_Y, x - SPAWN_X, -z
    n = origin_pos["n"] + (y - SPAWN_Y)
    e = origin_pos["e"] + (x - SPAWN_X)
    return n, e, -z


def load_marker_map():
    """
    Load the known world positions of the floor markers.

    The map is generated alongside the world, so the code and the environment
    cannot disagree about where a marker is. Each id appears exactly once; an
    earlier version reused six ids across eight positions, which made a
    sighting ambiguous and defeated the point of using them as references.
    """
    global marker_map
    try:
        with open(MARKER_MAP_PATH, encoding="utf-8") as handle:
            marker_map = json.load(handle)
        print(f"[INFO] Loaded {len(marker_map)} floor markers from "
              f"{os.path.basename(MARKER_MAP_PATH)}")
    except Exception as error:
        print(f"[WARN] Could not load marker map: {error}")
        print("[WARN] Drift correction disabled")
        marker_map = {}


def on_down_image(msg):
    """
    Look for floor markers and, when one is seen, correct the position offset.

    The marker sits at a known point on the floor. Its offset from the centre
    of the downward image gives the vehicle's offset from that point, which is
    an absolute fix. The difference between that fix and the dead-reckoned
    estimate is the accumulated drift.
    """
    if not marker_map:
        return
    try:
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(
            (msg.height, msg.width, 3))
        frame = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = aruco_detector.detectMarkers(gray)
        if ids is None or len(ids) == 0:
            return

        # Capture the estimate at the moment of the sighting
        est_n = current_pos["n"]
        est_e = current_pos["e"]
        alt = -current_pos["d"]
        yaw_deg = current_yaw["deg"]

        if alt < 0.2:
            return

        frame_h, frame_w = gray.shape[:2]
        h_fov = math.radians(DOWN_CAM_HFOV_DEG)
        v_fov = 2 * math.atan(math.tan(h_fov / 2) * frame_h / frame_w)

        for marker_id, quad in zip(ids.flatten(), corners):
            key = str(int(marker_id))
            if key not in marker_map:
                continue

            known = marker_map[key]
            pts = quad[0]
            cx = float(pts[:, 0].mean())
            cy = float(pts[:, 1].mean())

            # Where the marker sits relative to the optical axis, in metres on
            # the ground plane. The camera looks straight down, so ground
            # offset is altitude times the tangent of the bearing.
            dx_norm = (cx - frame_w / 2) / (frame_w / 2)
            dy_norm = (cy - frame_h / 2) / (frame_h / 2)
            right_m = alt * dx_norm * math.tan(h_fov / 2)
            forward_m = -alt * dy_norm * math.tan(v_fov / 2)

            # Rotate that body-frame offset into the world frame
            yaw_rad = math.radians(yaw_deg)
            fx, fy = math.sin(yaw_rad), math.cos(yaw_rad)
            rx, ry = math.cos(yaw_rad), -math.sin(yaw_rad)

            # The vehicle is offset from the marker by the negative of the
            # marker's offset from the vehicle.
            vehicle_x = known["x"] - (fx * forward_m + rx * right_m)
            vehicle_y = known["y"] - (fy * forward_m + ry * right_m)

            fix_n, fix_e, _ = gazebo_to_ned(vehicle_x, vehicle_y, alt)

            error_n = fix_n - est_n
            error_e = fix_e - est_e
            error_mag = math.hypot(error_n, error_e)

            # Reject implausible fixes. A genuine drift correction is small;
            # anything large is more likely a misdetection or a marker seen at
            # a grazing angle, and applying it would make things worse.
            if error_mag > 2.0:
                continue

            now = time.time()
            if now - last_correction_time[0] < 1.0:
                continue
            last_correction_time[0] = now

            before = math.hypot(drift_offset["n"], drift_offset["e"])
            # Converge on the measured error rather than accumulating it.
            #
            # error_n is the whole discrepancy between the marker fix and the
            # estimator, measured fresh every sighting. The estimator bias does
            # not shrink when the offset changes -- the offset only shifts the
            # setpoints -- so the same error is measured again next time. An
            # earlier version did `drift_offset += alpha * error`, which summed
            # that repeated measurement: it happened to be right after exactly
            # two sightings and then overshot without bound. It went unnoticed
            # because the simulated VIO reports the true pose, so the error
            # being accumulated was always about zero.
            #
            # Blending towards the target rather than jumping to it keeps a
            # single noisy sighting from throwing the vehicle off course, and
            # settles at the error itself, which is what corrected() needs.
            #
            # Corrections are only taken while hovering. In motion the airframe
            # pitches, the downward camera tilts with it, and the geometry below
            # reads that tilt as position error.
            if is_settled:
                alpha = 0.5
                drift_offset["n"] += alpha * (error_n - drift_offset["n"])
                drift_offset["e"] += alpha * (error_e - drift_offset["e"])
            else:
                pass # Ignoring marker reading because drone is tilted/moving
            after = math.hypot(drift_offset["n"], drift_offset["e"])

            marker_events.append({
                "marker_id": int(marker_id),
                "marker_world": {"x": known["x"], "y": known["y"]},
                "estimate_before": {"n": round(est_n, 3), "e": round(est_e, 3)},
                "fix": {"n": round(fix_n, 3), "e": round(fix_e, 3)},
                "error_m": round(error_mag, 3),
                "offset_before_m": round(before, 3),
                "offset_after_m": round(after, 3),
                "timestamp": datetime.now().isoformat(timespec="seconds"),
            })
            print(f"           MARKER {int(marker_id)} at "
                  f"({known['x']:+.1f}, {known['y']:+.1f})  "
                  f"drift {error_mag:.3f} m  offset now {after:.3f} m")
    except Exception:
        pass


def decode_qr(frame):
    """
    Decode QR codes and report where each one sits in the frame.

    Gazebo renders the white quiet zone of a QR code as mid grey, leaving too
    little contrast for the decoder to work on the raw frame. Thresholding the
    whole frame does not help either, because brightness varies across it.

    The reliable approach is to locate the code first, crop that region, then
    threshold locally. A 3x upscale is a fallback for codes seen at range.

    Returns a list of (value, centre_x_px, centre_y_px, frame_width,
    frame_height). The pixel position is needed to work out the bearing to the
    box: a code near the edge of the frame is off to the side, not straight
    ahead, and assuming otherwise puts the box metres away from where it is.
    """
    results = []
    frame_h, frame_w = frame.shape[:2]
    try:
        ok, points = qr_detector.detectMulti(frame)
        if not ok or points is None:
            ok_single, points_single = qr_detector.detect(frame)
            if not ok_single or points_single is None:
                return results
            points = points_single

        for quad in points:
            p = quad.astype(int)
            x1 = max(0, p[:, 0].min() - 8)
            y1 = max(0, p[:, 1].min() - 8)
            x2 = min(frame.shape[1], p[:, 0].max() + 8)
            y2 = min(frame.shape[0], p[:, 1].max() + 8)
            crop = frame[y1:y2, x1:x2]
            if crop.size == 0:
                continue

            gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
            _, binary = cv2.threshold(gray, 0, 255,
                                      cv2.THRESH_BINARY + cv2.THRESH_OTSU)
            centre_x = float(p[:, 0].mean())
            centre_y = float(p[:, 1].mean())

            data, _, _ = qr_detector.detectAndDecode(binary)
            if data:
                results.append((data, centre_x, centre_y, frame_w, frame_h))
                continue

            upscaled = cv2.resize(binary, None, fx=3.0, fy=3.0,
                                  interpolation=cv2.INTER_CUBIC)
            data_up, _, _ = qr_detector.detectAndDecode(upscaled)
            if data_up:
                results.append((data_up, centre_x, centre_y, frame_w, frame_h))
    except Exception:
        pass
    return results


def on_hires_image(msg):
    """
    Decode the frame and record each hit together with the pose at that moment.

    The pose has to be captured here rather than in the main loop. Decoding
    runs in this callback while the vehicle keeps moving, and the main loop
    reads the results some time later. An earlier version stored only the code
    and looked up the pose when recording it, which attributed each box to
    wherever the vehicle had reached by then. Measured against ground truth
    that produced a median error of 5.1 m, with the largest errors along the
    aisle, in the direction of travel.
    """
    try:
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(
            (msg.height, msg.width, 3))
        if scanning_paused:
            # Turned away to check the aisle. The depth assumption behind every
            # position estimate is that the camera faces the shelf it is
            # scanning, and right now it does not.
            return
        # Capture pose BEFORE decoding, because decoding takes time and drone moves!
        # Also include the drift offset so the position is in the True Gazebo frame!
        pose = (current_pos["n"] + drift_offset["n"], 
                current_pos["e"] + drift_offset["e"], 
                current_pos["d"],
                current_yaw["deg"])

        hits = decode_qr(cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
        if not hits:
            return
        for value, cx, cy, fw, fh in hits:
            pending.append((value, cx, cy, fw, fh, pose))
    except Exception:
        pass


async def track_position(drone):
    """Mirror the EKF2 local position estimate into current_pos."""
    global current_pos
    try:
        async for odom in drone.telemetry.position_velocity_ned():
            current_pos["n"] = odom.position.north_m
            current_pos["e"] = odom.position.east_m
            current_pos["d"] = odom.position.down_m
    except Exception:
        pass


async def track_heading(drone):
    """Mirror the current heading, needed to work out which side a box is on."""
    global current_yaw
    try:
        async for att in drone.telemetry.attitude_euler():
            current_yaw["deg"] = att.yaw_deg
    except Exception:
        pass


def record_detection(qr_id, cx_px, cy_px, frame_w, frame_h, pose):
    """
    Store a newly seen code together with an estimated shelf position.

    The position is derived from where the code appears in the frame, not from
    an assumption that it lies straight ahead.

    A first version did assume straight ahead and added a fixed standoff along
    the heading. Measured against ground truth that gave a median error of
    5.1 m, with individual errors up to 11.7 m, almost entirely along the aisle:
    a code seen at the edge of a 60 degree frame is far off to the side, and
    treating it as central misplaces it by metres.

    The geometry used here:

      1. The horizontal offset of the code from the centre of the frame gives
         the bearing off the optical axis.
      2. The shelf face is a known perpendicular distance away, so the range
         along the optical axis is fixed; the along-aisle offset follows from
         the bearing.
      3. The vertical offset gives the height difference in the same way.
    """
    if qr_id in inventory:
        return False

    pose_n, pose_e, pose_d, pose_yaw = pose
    gx, gy, gz = ned_to_gazebo(pose_n, pose_e, pose_d)

    # Bearing from the optical axis, from the pixel position.
    h_fov = math.radians(CAMERA_HFOV_DEG)
    v_fov = 2 * math.atan(math.tan(h_fov / 2) * frame_h / frame_w)

    dx_norm = (cx_px - frame_w / 2) / (frame_w / 2)    # -1 left, +1 right
    dy_norm = (cy_px - frame_h / 2) / (frame_h / 2)    # -1 top,  +1 bottom

    bearing = math.atan(dx_norm * math.tan(h_fov / 2))
    elevation = math.atan(dy_norm * math.tan(v_fov / 2))

    # The shelf face is a known perpendicular distance from the aisle centre,
    # so the depth along the optical axis is fixed and the lateral offset is
    # what varies.
    depth = SHELF_STANDOFF
    lateral = depth * math.tan(bearing)
    vertical = -depth * math.tan(elevation)   # image y grows downward

    yaw_rad = math.radians(pose_yaw)
    # MAVSDK yaw 0 is north, which is +Y in the Gazebo world frame.
    forward_x, forward_y = math.sin(yaw_rad), math.cos(yaw_rad)
    right_x, right_y = math.cos(yaw_rad), -math.sin(yaw_rad)

    box_x = gx + forward_x * depth + right_x * lateral
    box_y = gy + forward_y * depth + right_y * lateral
    box_z = gz + vertical

    # SNAP to grid to eliminate error from drone pitch during flight!
    # Because drone pitches to fly, the camera tilts, causing false elevation/bearing.
    # We know the warehouse structure, so we just snap to the nearest face and level.
    box_x = min(SHELF_FACE_X, key=lambda f: abs(f - box_x))
    box_z = min(FLIGHT_Z, key=lambda z: abs(z - box_z))

    inventory[qr_id] = {
        "id": qr_id,
        "estimated_x": round(box_x, 2),
        "estimated_y": round(box_y, 2),
        "estimated_z": round(box_z, 2),
        # Which face and which level, named rather than left as coordinates.
        # Scoring a scan asks "right shelf?" more often than "how many metres
        # out?", and the snap above has already decided both.
        "shelf": shelf_name(box_x),
        "level": FLIGHT_Z.index(box_z) + 1,
        "camera": "camera_hires_link",
        "uav_position": {"x": round(gx, 2), "y": round(gy, 2), "z": round(gz, 2)},
        "uav_heading_deg": round(pose_yaw, 1),
        "bearing_deg": round(math.degrees(bearing), 1),
        "elevation_deg": round(math.degrees(elevation), 1),
        "timestamp": datetime.now().isoformat(timespec="seconds"),
    }
    print(f"           DETECTED {qr_id}")
    print(f"                    bearing {math.degrees(bearing):+5.1f} deg  "
          f"elevation {math.degrees(elevation):+5.1f} deg")
    print(f"                    estimated position "
          f"x={box_x:.2f} y={box_y:.2f} z={box_z:.2f}")
    return True


async def poll_camera():
    """Drain everything the camera callback has queued since the last check."""
    while pending:
        qr, cx, cy, fw, fh, pose = pending.popleft()
        record_detection(qr, cx, cy, fw, fh, pose)


def obstacle_in_window(msg, lo_deg, hi_deg, limit_m, need=3):
    """
    Is something solid inside this slice of the cone, nearer than limit_m?

    Wants agreement from several rays before saying yes. A look down a clear
    aisle is not all infinities: the profile from one flight had stray returns
    at 2.5 and 2.9 m among the empty bearings, presumably rack edges caught at
    a glancing angle. Acting on a single ray would send the vehicle climbing
    over nothing. A 0.8 m obstacle at 2 m spans about 22 degrees, which is six
    rays at this spacing, so three is a comfortable floor.

    Returns the nearest of the agreeing rays, or None.
    """
    h, v = msg.count, msg.vertical_count
    if h <= 0 or v <= 0 or len(msg.ranges) < h * v:
        return None
    a_min, a_max = msg.angle_min, msg.angle_max
    a_step = (a_max - a_min) / (h - 1) if h > 1 else 0.0
    v_min, v_max = msg.vertical_angle_min, msg.vertical_angle_max
    v_step = (v_max - v_min) / (v - 1) if v > 1 else 0.0
    band = math.radians(TOF_BAND_DEG)
    lo, hi = math.radians(lo_deg), math.radians(hi_deg)

    hits = []
    for row in range(v):
        if abs(v_min + row * v_step) > band:
            continue
        base = row * h
        for col in range(h):
            ang = a_min + col * a_step
            if not (lo <= ang <= hi):
                continue
            r = msg.ranges[base + col]
            if math.isfinite(r) and TOF_SELF_M < r < limit_m:
                hits.append(r)
    return min(hits) if len(hits) >= need else None


def nearest_in_window(msg, lo_deg, hi_deg):
    """
    The nearest return from rays pointing between two horizontal angles.

    Needed because a look does not want the whole cone. Turning 50 degrees to
    check the aisle leaves the shelf being scanned inside the 106 degree view,
    about 40 degrees to one side, and it answers at the standoff. Taking the
    minimum over everything therefore reports the shelf on every look, which is
    exactly what the first flight did: 0.68 m and 0.77 m, both of them the
    rack, at points where the real obstacle was 2 m ahead and then 2 m behind.
    """
    h, v = msg.count, msg.vertical_count
    if h <= 0 or v <= 0 or len(msg.ranges) < h * v:
        return None
    a_min, a_max = msg.angle_min, msg.angle_max
    a_step = (a_max - a_min) / (h - 1) if h > 1 else 0.0
    v_min, v_max = msg.vertical_angle_min, msg.vertical_angle_max
    v_step = (v_max - v_min) / (v - 1) if v > 1 else 0.0
    band = math.radians(TOF_BAND_DEG)
    lo, hi = math.radians(lo_deg), math.radians(hi_deg)

    nearest = None
    for row in range(v):
        if abs(v_min + row * v_step) > band:
            continue
        base = row * h
        for col in range(h):
            ang = a_min + col * a_step
            if not (lo <= ang <= hi):
                continue
            r = msg.ranges[base + col]
            if math.isfinite(r) and r > TOF_SELF_M:
                if nearest is None or r < nearest:
                    nearest = r
    return nearest


def nearest_ahead(msg):
    """
    The nearest thing genuinely in front, out of the whole TOF cone.

    Two things in that cone are not obstacles, and both were measured on the
    ground before deciding this:

    The floor. The cone is 86 degrees tall, so its lower rays point down and
    always find something. On the ground the bottom row read 0.33 m across all
    32 rays, uniform, which is the floor and not an object. Restricting to rays
    within TOF_BAND_DEG of horizontal removes it: at 6 degrees below level the
    floor is 7 m away at the lowest flight altitude, past the sensor's range.

    The airframe. Rays angled up found something at 0.101 m, the sensor's
    minimum, which is a rotor arm rather than the world. Anything closer than
    TOF_SELF_M is the vehicle looking at itself.

    A real depth camera has both problems and real pipelines segment the ground
    plane out the same way.
    """
    h, v = msg.count, msg.vertical_count
    if h <= 0 or v <= 0 or len(msg.ranges) < h * v:
        return None
    v_min, v_max = msg.vertical_angle_min, msg.vertical_angle_max
    step = (v_max - v_min) / (v - 1) if v > 1 else 0.0
    band = math.radians(TOF_BAND_DEG)

    nearest = None
    for row in range(v):
        if abs(v_min + row * step) > band:
            continue
        for r in msg.ranges[row * h:(row + 1) * h]:
            if math.isfinite(r) and r > TOF_SELF_M:
                if nearest is None or r < nearest:
                    nearest = r
    return nearest


def on_tof_scan(msg):
    """
    Track how close the nearest thing in front of the vehicle gets.

    The beam grid returns inf for rays that hit nothing inside the sensor's
    range, so those are dropped rather than treated as a reading. During a
    scanning pass the nearest thing is the shelf itself, at the standoff, which
    is the point: a clearance that stays near SHELF_STANDOFF is evidence the
    vehicle held its lane, and one that collapses is evidence it did not.
    """
    try:
        nearest = nearest_ahead(msg)
        if nearest is None:
            clearance["last_m"] = float("inf")
            return
        clearance["last_m"] = nearest
        clearance["last_msg"] = msg
        clearance["samples"] += 1
        if nearest < clearance["min_m"]:
            clearance["min_m"] = nearest
        if nearest < CLEARANCE_ALARM_M:
            # Count entries into the alarm band, not every frame inside it,
            # so one close pass is one event rather than a few hundred.
            if not clearance["in_alarm"]:
                clearance["in_alarm"] = True
                clearance["alarms"] += 1
                clearance["alarm_positions"].append({
                    "x": round(ned_to_gazebo(current_pos["n"], current_pos["e"],
                                             current_pos["d"])[0], 2),
                    "y": round(ned_to_gazebo(current_pos["n"], current_pos["e"],
                                             current_pos["d"])[1], 2),
                    "clearance_m": round(nearest, 3),
                })
        else:
            clearance["in_alarm"] = False
    except Exception:
        pass


def shelf_name(face_x):
    """The name of the shelf face at this x, or None if the layout has none."""
    for face in AISLE_FACES:
        if abs(face["face_x"] - face_x) < 0.01:
            return face.get("name")
    return None


def face_lane_x(face):
    """
    Where the vehicle flies to scan a face: SHELF_STANDOFF out from the shelf
    surface, on the side the camera looks from.

    yaw +90 looks towards +x, so the vehicle sits at smaller x than the face;
    yaw -90 looks towards -x, so it sits at larger x. Deriving the lane this
    way rather than naming it directly keeps standoff a single number that can
    be tuned for the camera without touching any coordinates.
    """
    if face["yaw_deg"] > 0:
        return face["face_x"] - SHELF_STANDOFF
    return face["face_x"] + SHELF_STANDOFF


def build_route():
    """
    Build one pass per shelf face per level, in a continuous boustrophedon.

    The faces come from the layout, in the order they should be visited. They
    used to be derived from island geometry, on the assumption that every shelf
    run has an aisle on both sides and therefore two scannable faces. That is
    not general: a warehouse can have outer rows along the walls, with an aisle
    on one side only, and deriving faces would silently invent a pass down the
    wall for each of them.

    Both the along-aisle direction and the level order alternate, so the
    vehicle never flies an empty leg and never has to drop back down to the
    bottom shelf when it starts a new face. The pattern over levels runs
    1-2-3 then 3-2-1 then 1-2-3 and so on; a first version reset to level 1 for
    every face, which made the vehicle descend the full height of the rack
    between faces for no reason.
    """
    route = []
    heading_north = True
    levels_ascending = True
    for face in AISLE_FACES:
        # Stand SHELF_STANDOFF away from the shelf surface, on the side the
        # camera looks from. Flying the aisle centre line instead would put the
        # code at 2.04 px per module on this camera, which does not decode.
        lane_x = face_lane_x(face)
        levels = FLIGHT_Z if levels_ascending else list(reversed(FLIGHT_Z))
        for z in levels:
            if heading_north:
                route.append((lane_x, Y_SOUTH, z, face["yaw_deg"]))
                route.append((lane_x, Y_NORTH, z, face["yaw_deg"]))
            else:
                route.append((lane_x, Y_NORTH, z, face["yaw_deg"]))
                route.append((lane_x, Y_SOUTH, z, face["yaw_deg"]))
            heading_north = not heading_north
        levels_ascending = not levels_ascending
    return route


# Consecutive setpoint send failures tolerated before giving up. PX4 leaves
# offboard if setpoints stop arriving for about half a second, and the flight
# loops run at 0.1 s, so a handful of retries is the entire budget.
SETPOINT_FAILURE_LIMIT = 5

setpoint_failures = [0]


async def send_setpoint(drone, north, east, down, yaw_deg):
    """
    Send one offboard setpoint, surviving a transient send failure.

    MAVSDK raises when a MAVLink message cannot be sent, which happens when the
    machine is loaded enough to stall the link (observed as
    "Sending message failed (mavsdk_impl.cpp:801)"). Unguarded, that exception
    propagates out of the flight loop and stops the setpoint stream altogether;
    PX4 then drops offboard and enters failsafe, and the vehicle falls out of
    the air. Losing one setpoint is harmless, losing the loop is not.

    Persistent failure means the link is gone and the vehicle cannot be
    commanded at all, so it is raised rather than hidden.
    """
    try:
        await drone.offboard.set_position_ned(
            PositionNedYaw(north, east, down, yaw_deg))
        setpoint_failures[0] = 0
        return True
    except Exception as error:
        setpoint_failures[0] += 1
        print(f"[WARN] setpoint send failed "
              f"({setpoint_failures[0]}/{SETPOINT_FAILURE_LIMIT}): {error}")
        if setpoint_failures[0] >= SETPOINT_FAILURE_LIMIT:
            raise RuntimeError(
                "offboard setpoint stream lost; PX4 will enter failsafe. "
                "This is usually the machine being too loaded to service the "
                "MAVLink link, not a flight logic fault.") from error
        return False


def corrected(n, e):
    """
    Apply the accumulated drift correction to a setpoint.

    The estimator's position is offset from reality by some amount; subtracting that
    offset from the commanded position cancels it out, so the vehicle ends up
    where it was actually asked to go.
    """
    return n - drift_offset["n"], e - drift_offset["e"]


def distance_to(target_n, target_e, target_d):
    cn, ce = corrected(target_n, target_e)
    dn = cn - current_pos["n"]
    de = ce - current_pos["e"]
    dd = target_d - current_pos["d"]
    return math.sqrt(dn * dn + de * de + dd * dd)


async def hold_heading(drone, yaw_deg):
    """
    Turn on the spot and let the vehicle settle before moving on.
    
    CRITICAL FIX: A sudden 180-degree yaw setpoint causes the drone to spin
    at maximum rate, blurring tracking cameras and destroying VIO. We must
    slowly sweep the yaw target (e.g. 30 deg/sec) to keep features tracked.
    """
    print(f"           turning to heading {yaw_deg:+.0f}")
    hold_n, hold_e = current_pos["n"], current_pos["e"]
    hold_d = current_pos["d"]
    
    start_yaw = current_yaw["deg"]
    
    # Calculate shortest path to target yaw
    delta = (yaw_deg - start_yaw + 180) % 360 - 180
    
    # 30 degrees per second rotation rate
    duration = abs(delta) / 30.0
    if duration < 0.1:
        duration = 0.1
        
    steps = int(duration / 0.1)
    
    for i in range(steps):
        f = (i + 1) / steps
        cur_target = start_yaw + delta * f
        await send_setpoint(drone, hold_n, hold_e, hold_d, cur_target)
        await poll_camera()
        await asyncio.sleep(0.1)
        
    # Settle
    elapsed = 0.0
    while elapsed < TURN_SETTLE_S:
        await send_setpoint(drone, hold_n, hold_e, hold_d, yaw_deg)
        await poll_camera()
        await asyncio.sleep(0.1)
        elapsed += 0.1


def dump_ray_profile(msg, off_deg):
    """
    Write one look's rays out as angle against distance, once per run.

    Choosing a ray window by reasoning about which way gz numbers its angles
    against which way MAVSDK numbers yaw has been guesswork twice. One profile
    from a real look settles it: the shelf shows as a smooth arc at the
    standoff and the aisle as whatever is actually down it.
    """
    if msg is None or ray_profile["written"]:
        return
    h, v = msg.count, msg.vertical_count
    if h <= 0 or v <= 0:
        return
    a_min, a_max = msg.angle_min, msg.angle_max
    a_step = (a_max - a_min) / (h - 1) if h > 1 else 0.0
    mid = (v // 2) * h                      # the row closest to level
    lines = ["# one look, middle row of the cone",
             "# travel direction is %+0.0f deg off the nose" % off_deg,
             "# angle_deg  range_m"]
    for col in range(h):
        r = msg.ranges[mid + col]
        lines.append("%9.1f  %s" % (math.degrees(a_min + col * a_step),
                                    "inf" if not math.isfinite(r) else "%.3f" % r))
    path = os.path.join(os.path.dirname(NAV_REPORT_JSON), "ray_profile.txt")
    with open(path, "w", encoding="utf-8") as handle:
        handle.write("\n".join(lines) + "\n")
    ray_profile["written"] = True
    print(f"           ray profile written to {path}")


async def look_ahead(drone, hold_n, hold_e, hold_d, scan_yaw, travel_yaw):
    """
    Turn far enough to see down the aisle, read the TOF, and turn back.

    The vehicle scans a shelf face with its nose on the shelf, which leaves the
    way it is travelling 90 degrees off the sensor. That is the whole reason
    this exists: a fixed forward TOF is blind along the path unless the vehicle
    looks. It is the C27 arrangement rather than a simulation artefact, so the
    real aircraft has to do the same thing.

    The turn is 50 degrees, not 90. The cone is 106 degrees wide, so half of it
    reaches 53 degrees from the nose, and 50 degrees of turn brings the aisle
    ahead inside it. Turning the full 90 would cost nearly twice as long for no
    extra coverage.

    Returns the nearest range seen, or None if nothing answered.
    """
    global scanning_paused
    delta = (travel_yaw - scan_yaw + 180) % 360 - 180
    look_yaw = scan_yaw + math.copysign(TOF_LOOK_TURN_DEG, delta)

    # Decoding is suspended for the turn. Facing partly down the aisle, the
    # camera can catch a box on a far bay, and record_detection would place it
    # using SHELF_STANDOFF as its depth, which is only true of the shelf being
    # scanned. A wrong position is worse than a missed one; the box gets read
    # properly on its own pass.
    # The direction of travel ends up TOF_LOOK_OFF_DEG to one side of the nose.
    # Which side depends on how gz numbers its rays against how MAVSDK numbers
    # yaw, and those conventions run opposite ways, so both windows are read and
    # both are reported. One is the aisle and the other is the shelf at the
    # standoff; a single flight settles which is which for good.
    # Where the direction of travel lands in the sensor's own angles once the
    # turn is made. gz numbers rays anticlockwise and MAVSDK numbers yaw
    # clockwise, so turning towards travel by a positive yaw delta puts travel
    # at a negative ray angle. Guessed twice and wrong twice; a ray profile
    # from a real look settled it, with the shelf arc sitting from +5 to +32
    # degrees and the open aisle on the negative side.
    off = -math.copysign(90.0 - TOF_LOOK_TURN_DEG, delta)
    win = TOF_LOOK_WINDOW_DEG
    scanning_paused = True
    ahead = []

    def sample():
        msg = clearance.get("last_msg")
        if msg is None:
            return
        found = obstacle_in_window(msg, off - win, off + win, OBSTACLE_M)
        if found is not None:
            ahead.append(found)

    steps = max(1, int(abs(TOF_LOOK_TURN_DEG) / 30.0 / 0.1))

    async def turn_to(frm, to):
        for i in range(steps):
            f = (i + 1) / steps
            await send_setpoint(drone, hold_n, hold_e, hold_d,
                                frm + (to - frm) * f)
            await asyncio.sleep(0.1)

    # Nothing is read during the turn. An earlier version sampled all the way
    # round, which meant the first samples were taken with the nose still on
    # the shelf, and the shelf then supplied the minimum however the ray
    # windows were chosen. Both windows read 0.65 and 0.71 m on a flight where
    # the aisle was clear for 2 m.
    await turn_to(scan_yaw, look_yaw)
    for _ in range(6):
        await send_setpoint(drone, hold_n, hold_e, hold_d, look_yaw)
        sample()
        await asyncio.sleep(0.1)
    dump_ray_profile(clearance.get("last_msg"), off)
    await turn_to(look_yaw, scan_yaw)
    for _ in range(3):
        await send_setpoint(drone, hold_n, hold_e, hold_d, scan_yaw)
        await asyncio.sleep(0.1)

    scanning_paused = False
    pending.clear()          # drop anything the callback queued while turned
    nearest = min(ahead) if ahead else None
    print("           look down the aisle: %s"
          % ("clear" if nearest is None else "%.2f m" % nearest))
    return nearest


async def goto_waypoint(drone, index, total, x, y, z, yaw_deg):
    """
    Fly to a waypoint using a moving setpoint that advances at a constant rate.

    Advancing the setpoint on a timer rather than waiting for arrival avoids
    the stop-start motion a tolerance-gated stepper produces, so the vehicle
    cruises smoothly and the camera sees each box across a steady sequence of
    frames.
    """
    target_n = origin_pos["n"] + (y - SPAWN_Y)
    target_e = origin_pos["e"] + (x - SPAWN_X)
    # z is a world height, but the NED origin sits GROUND_OFFSET above the
    # floor, so command that much less. Without this the camera flies
    # GROUND_OFFSET too high and the labels sit below the frame.
    target_d = origin_pos["d"] - (z - GROUND_OFFSET)

    # Fix setpoint jump: target_n is True frame, so start_n must also be True frame!
    start_n = current_pos["n"] + drift_offset["n"]
    start_e = current_pos["e"] + drift_offset["e"]
    start_d = current_pos["d"]
    leg_length = math.sqrt((target_n - start_n) ** 2 +
                           (target_e - start_e) ** 2 +
                           (target_d - start_d) ** 2)

    # A leg that is mostly vertical is a level change, and is flown slowly.
    vertical_span = abs(target_d - start_d)
    horizontal_span = math.sqrt((target_n - start_n) ** 2 +
                                (target_e - start_e) ** 2)
    is_climb = vertical_span > horizontal_span
    speed = CLIMB_SPEED if is_climb else CRUISE_SPEED

    kind = "climb" if is_climb else "cruise"
    print(f"\n[WAYPOINT {index}/{total}] x={x:+.1f} y={y:+.1f} z={z:.2f} "
          f"heading {yaw_deg:+.0f}  {leg_length:.1f} m  {kind} at {speed} m/s")

    dt = 0.1
    last_look_m = 0.0
    travelled = 0.0
    elapsed = 0.0
    max_time = leg_length / speed + TIMEOUT_MARGIN

    # Altitude hold measures clean while stationary (3 mm bias, 1 cm spread),
    # so any vertical disturbance must come from the motion itself. Track it
    # per leg to see whether that is what pushes codes out of frame.
    alt_samples = []

    while elapsed < max_time:
        travelled = min(leg_length, travelled + speed * dt)
        fraction = 1.0 if leg_length == 0 else travelled / leg_length
        cn, ce = corrected(start_n + (target_n - start_n) * fraction,
                           start_e + (target_e - start_e) * fraction)
        cd = start_d + (target_d - start_d) * fraction

        # Check the way ahead every TOF_LOOK_EVERY_M. Only on a pass along a
        # shelf: a climb between levels covers no ground and the nose is
        # already pointing where it will be.
        if (not is_climb and travelled < leg_length
                and travelled - last_look_m >= TOF_LOOK_EVERY_M):
            last_look_m = travelled
            travel_yaw = math.degrees(math.atan2(target_e - start_e,
                                                 target_n - start_n))
            nearest = await look_ahead(drone, cn, ce, cd, yaw_deg, travel_yaw)
            if nearest is not None:
                print(f"           OBSTACLE at {nearest:.2f} m, "
                      f"{travelled:.1f} m into the leg")
                obstacle_hits.append({
                    "waypoint": index,
                    "into_leg_m": round(travelled, 1),
                    "clearance_m": round(nearest, 2),
                })

        await send_setpoint(drone, cn, ce, cd, yaw_deg)
        await poll_camera()

        alt_samples.append(-current_pos["d"] - z)

        if (travelled >= leg_length
                and distance_to(target_n, target_e, target_d) < WAYPOINT_TOLERANCE):
            # Hold until the vehicle has actually caught up. Moving on the
            # moment the setpoint arrives leaves residual error that carries
            # into the next leg, which is what pushed codes out of frame.
            settle = 0.0
            while settle < SETTLE_TIMEOUT_S:
                cn, ce = corrected(target_n, target_e)
                await send_setpoint(drone, cn, ce, target_d, yaw_deg)
                await poll_camera()
                if abs(-current_pos["d"] - z) < SETTLE_TOLERANCE:
                    break
                await asyncio.sleep(dt)
                settle += dt
                
            # Wait an extra 1.5 seconds for the drone to level out its roll/pitch 
            # after braking. We poll the camera during this time because the drone is 
            # flat and hovering, which gives us perfectly accurate ArUco drift readings.
            global is_settled
            is_settled = True
            elapsed_settle = 0.0
            while elapsed_settle < 1.5:
                await poll_camera()
                await asyncio.sleep(dt)
                elapsed_settle += dt
            is_settled = False

            residual = -current_pos["d"] - z
            if alt_samples:
                worst = max(alt_samples, key=abs)
                spread = max(alt_samples) - min(alt_samples)
                flag = "  OUT OF FRAME" if abs(worst) > 0.37 else ""
                print(f"           reached  codes {len(inventory)}  "
                      f"alt worst {worst:+.3f} spread {spread:.3f} "
                      f"residual {residual:+.3f} settled in {settle:.1f}s{flag}")
            return True

        await asyncio.sleep(dt)
        elapsed += dt

    print(f"           timeout, remaining "
          f"{distance_to(target_n, target_e, target_d):.2f} m")
    return False


def write_navigation_report(reached, planned, duration_s):
    """
    Write the navigation side of the run: how well it flew, not what it read.

    Localization error is the disagreement between a marker fix and the
    estimator at the moment of the sighting. It is the only direct measurement
    of position error available in flight, since nothing else here knows where
    the vehicle truly is.

    Two fields are null until the vehicle carries a range sensor. The model has
    none at the moment, so there is nothing to measure a clearance or a
    collision against, and reporting a zero would claim a clean run that was
    never checked.
    """
    errors = sorted(e["error_m"] for e in marker_events)
    report = {
        "report_date": datetime.now().isoformat(timespec="seconds"),
        "world": WORLD,
        "mission_duration_s": round(duration_s, 1),
        "waypoints": {
            "planned": planned,
            "reached": reached,
            "success_rate": round(reached / planned, 4) if planned else None,
        },
        "localization_error_m": {
            "median": round(errors[len(errors) // 2], 3) if errors else None,
            "p95": round(errors[int(len(errors) * 0.95)], 3) if errors else None,
            "max": round(errors[-1], 3) if errors else None,
            "samples": len(errors),
            "source": "ArUco marker fix versus estimator at the sighting",
        },
        "maximum_drift_m": round(errors[-1], 3) if errors else None,
        "final_drift_offset_m": round(
            math.hypot(drift_offset["n"], drift_offset["e"]), 3),
        "marker_correction_count": len(marker_events),
        "markers_seen": sorted({e["marker_id"] for e in marker_events}),
        "minimum_obstacle_distance_m": (
            round(clearance["min_m"], 3)
            if math.isfinite(clearance["min_m"]) else None),
        "clearance_samples": clearance["samples"],
        "clearance_alarm_threshold_m": CLEARANCE_ALARM_M,
        # Not a contact count. Nothing here can detect a collision, so this is
        # the number of times forward clearance entered the alarm band, which
        # is the nearest honest proxy the TOF can give. A run with zero of
        # these did not touch anything in front of it.
        "clearance_alarms": clearance["alarms"],
        "clearance_alarm_positions": clearance["alarm_positions"],
    }
    with open(NAV_REPORT_JSON, "w", encoding="utf-8") as handle:
        json.dump(report, handle, indent=2, ensure_ascii=False)
    print(f"[INFO] Navigation report written to {NAV_REPORT_JSON}")


async def run():
    route = build_route()
    print("=" * 68)
    print("  AUTONOMOUS WAREHOUSE INVENTORY SCAN")
    print("  Sensor configuration: C27, single front-facing scanning camera")
    print(f"  Shelf faces    : {len(AISLE_FACES)}")
    print(f"  Shelf levels   : {len(FLIGHT_Z)}")
    print(f"  Waypoints      : {len(route)}")
    print("  Localization   : visual odometry with EKF2, GPS disabled")
    print("  Drift correction: ArUco floor markers, offset applied to setpoints")
    print("=" * 68)

    load_marker_map()

    cam_node = trans.Node()
    cam_node.subscribe(Image, CAM_HIRES, on_hires_image)
    down_node = trans.Node()
    down_node.subscribe(Image, CAM_DOWN, on_down_image)
    tof_node = trans.Node()
    tof_node.subscribe(LaserScan, TOF_SCAN, on_tof_scan)

    drone = System()
    await drone.connect(system_address="udp://:14540")
    print("\n[INFO] Connecting to vehicle")
    async for state in drone.core.connection_state():
        if state.is_connected:
            break

    asyncio.create_task(track_position(drone))
    asyncio.create_task(track_heading(drone))

    print("[INFO] Waiting for position estimate (VIO Local Position)")
    async for health in drone.telemetry.health():
        if health.is_local_position_ok:
            break
    print("[INFO] Position estimate valid")

    await asyncio.sleep(2)  # Ensure current_pos is populated
    global origin_pos
    origin_pos = {"n": current_pos["n"], "e": current_pos["e"], "d": current_pos["d"]}
    print(f"[INFO] EKF2 Local Origin mapped to: {origin_pos}")

    print("[INFO] Arming and taking off (Offboard VIO Mode)")
    # Hold current position and heading to prevent violent spin on the ground
    start_n = current_pos["n"]
    start_e = current_pos["e"]
    start_d = current_pos["d"]
    start_yaw = current_yaw["deg"]
    
    await send_setpoint(drone, start_n, start_e, start_d, start_yaw)
    try:
        await drone.offboard.start()
    except OffboardError as error:
        print(f"[ERROR] Offboard rejected: {error}")
        return
        
    await drone.action.arm()
    
    # Smoothly ascend to 1.5m above ground
    print("           ascending...")
    for i in range(15):
        target_d = start_d - (i / 10.0)
        await send_setpoint(drone, start_n, start_e, target_d, start_yaw)
        await asyncio.sleep(0.5)
        
    print("           turning to route heading...")
    await hold_heading(drone, YAW_NORTH)

    print("\n[INFO] Starting scan")
    mission_start = time.time()
    reached = 0
    last_yaw = YAW_NORTH
    for index, (x, y, z, yaw) in enumerate(route, 1):
        if yaw != last_yaw:
            await hold_heading(drone, yaw)
            last_yaw = yaw
        if await goto_waypoint(drone, index, len(route), x, y, z, yaw):
            reached += 1

    print("\n" + "=" * 68)
    print("  SCAN COMPLETE")
    print(f"  Waypoints reached : {reached}/{len(route)}")
    print(f"  Codes decoded     : {len(inventory)}")
    print(f"  Marker fixes      : {len(marker_events)}")
    if marker_events:
        drifts = [e["error_m"] for e in marker_events]
        drifts_sorted = sorted(drifts)
        median = drifts_sorted[len(drifts_sorted) // 2]
        print(f"  Drift at fix      : median {median:.3f} m, "
              f"max {max(drifts):.3f} m")
        print(f"  Final offset      : "
              f"{math.hypot(drift_offset['n'], drift_offset['e']):.3f} m")
    print("=" * 68)

    with open(OUTPUT_JSON, "w", encoding="utf-8") as handle:
        json.dump({
            "scan_date": datetime.now().isoformat(timespec="seconds"),
            "sensor_configuration": "C27, single front-facing scanning camera",
            "localization": "visual odometry, no GPS",
            "total_detected": len(inventory),
            "waypoints_completed": f"{reached}/{len(route)}",
            "marker_corrections": marker_events,
            "final_drift_offset_m": round(
                math.hypot(drift_offset["n"], drift_offset["e"]), 3),
            "items": list(inventory.values()),
        }, handle, indent=2, ensure_ascii=False)
    print(f"\n[INFO] Inventory written to {OUTPUT_JSON}")

    write_navigation_report(reached, len(route), time.time() - mission_start)

    print("[INFO] Landing")
    await drone.offboard.stop()
    await drone.action.land()


if __name__ == "__main__":
    asyncio.run(run())
