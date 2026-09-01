"""
Autonomous Warehouse Inventory Scanner, C27 sensor configuration.

Scans a warehouse using a UAV with a single front-facing high resolution
camera, three tracking cameras and a forward TOF sensor. No GPS is used at any
point: localization relies on visual odometry from the tracking cameras, fed to
the PX4 EKF2 estimator as an external vision source.

Because the scanning camera faces forward rather than sideways, the vehicle
must turn to face the shelves and fly sideways along them. Flying the centre
of an aisle puts both of its faces at the same distance, so the forward hires
camera reads the face ahead while the rear tracking camera reads the one
behind, and one pass covers both.

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
# The AR0144 tracking cameras. Wider than the hires, so a code at the same
# pixel offset sits at a different bearing.
TRACKING_HFOV_DEG = 90.0
# Lane to shelf face, measured to base_link, which is what the setpoint
# commands. A property of the warehouse, not the vehicle, so it comes from the
# layout.
SHELF_STANDOFF = LAYOUT["shelf_standoff"]

# Where each camera sits along the body, from build_c27_drone.py. The bearing
# to a code is turned into a position using the distance from the camera to
# the shelf, and the camera is not at base_link: the hires looks forward from
# the front face and the rear camera looks back from the rear one, so each is
# nearer its own shelf than base_link is. Leaving these out scaled every
# lateral offset by eight per cent.
HIRES_MOUNT_X = 0.10
REAR_MOUNT_X = -0.08

# --- SCAN PARAMETERS ---------------------------------------------------
WAYPOINT_TOLERANCE = 0.4      # metres
CRUISE_SPEED = 0.6             # m/s along an aisle
CLIMB_SPEED = 0.15             # m/s when changing shelf level
TURN_SETTLE_S = 3.0            # seconds held after a heading change
SETTLE_TOLERANCE = 0.05        # metres of altitude error before a leg counts as settled
# Metres of lateral error allowed before a leg starts. Tighter than
# WAYPOINT_TOLERANCE by a long way, because this one decides how far the
# cameras are from their shelves for the whole of the leg that follows, and
# the rear camera has only 0.05 m of margin at the distance it flies.
SETTLE_LATERAL_M = 0.08
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
CAM_REAR = f"/world/{WORLD}/model/{DRONE}/link/camera_track_rear_link/sensor/camera/image"
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

# Forward clearance, from the TOF. The simulated airframe is PX4's x500, half
# a metre across, so anything inside this band is closer than it should ever be
# to a shelf it is scanning.
#
# The real vehicle is a ModalAI Starling 2, 283 mm across at the propeller
# tips, and on that airframe this would be 0.20. See build_starling2.py.
CLEARANCE_ALARM_M = 0.40
# Only rays this far from level count as looking ahead. The cone is 86 degrees
# tall and its lower half always finds the floor; see nearest_ahead.
TOF_BAND_DEG = 10.0
# Anything nearer than this is the vehicle's own structure, measured at 0.101 m
# on rotor arms with the sensor's minimum range at 0.1 m.
TOF_SELF_M = 0.25
clearance = {"min_m": float("inf"), "samples": 0, "alarms": 0,
             "in_alarm": False, "alarm_positions": []}

# The simulation clock, from PX4's odometry. It arrives over UDP at about
# 26 Hz, so the camera queue cannot delay it, and its timestamps sit on the
# same clock as the image headers: measured 4 ms apart, 14 ms spread, once the
# opening burst is past. Zero means nothing has arrived yet.
#
# It was taken from the TOF before, which shares gz transport with the
# cameras. When frames backed up the TOF backed up with them, so a stale frame
# was measured against a stale clock and the difference stayed near zero. The
# check reported nothing wrong because it could not see anything.
sim_now = {"s": 0.0}

# Where the vehicle has been, stamped in simulation time, oldest first. A
# frame is placed against its own entry here rather than against wherever the
# vehicle has reached by the time the frame is decoded.
#
# Long enough to cover a queue far deeper than should ever form: Ibrahim's
# failing run reached 122 s of lag, and at 26 Hz five minutes of history is
# eight thousand tuples.
POSE_HISTORY_S = 300.0
pose_history = []

# How old a frame may be before it is dropped. This no longer protects the
# position, which the history now does; it sheds load so that a decoder slower
# than the camera cannot build a queue that grows for the whole flight and
# costs the codes at the end of it. Generous, because dropping a frame that
# could have been placed correctly is pure loss.
MAX_FRAME_AGE_S = 0.50
frames_stale = {"n": 0}
frame_ages = []


def pose_at(when):
    """
    Where the vehicle was at a given simulation time.

    Linear between the two samples either side, which at 26 Hz are 39 ms and
    about 23 mm of travel apart. None if the history does not reach back that
    far, which means the frame is older than anything we can place it against.
    """
    if len(pose_history) < 2 or when < pose_history[0][0]:
        return None
    if when >= pose_history[-1][0]:
        return pose_history[-1][1:]

    lo, hi = 0, len(pose_history) - 1
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if pose_history[mid][0] <= when:
            lo = mid
        else:
            hi = mid

    t0, n0, e0, d0, y0 = pose_history[lo]
    t1, n1, e1, d1, y1 = pose_history[hi]
    span = t1 - t0
    f = 0.0 if span <= 0 else (when - t0) / span
    # Yaw wraps, so interpolate along the shorter way round.
    dy = (y1 - y0 + 180.0) % 360.0 - 180.0
    return (n0 + (n1 - n0) * f,
            e0 + (e1 - e0) * f,
            d0 + (d1 - d0) * f,
            y0 + dy * f)


def stamp_seconds(msg):
    """The simulation time a message was produced, in seconds."""
    return msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9

class CameraDecoder:
    """
    One camera, with its own detector and its own idea of where it points.

    The detector is not shared. cv2.wechat_qrcode_WeChatQRCode holds model
    state and is not safe to call from two threads, and sharing one between
    the hires and rear callbacks also serialises them: each waits for the
    other, both fall behind the simulator, and a callback then runs long after
    the frame it is holding was taken. Since the pose is read when the
    callback runs, the code is recorded wherever the vehicle has reached by
    then.

    The first two-camera run measured exactly that. Codes came out up to
    5.8 m along the aisle from where they are, on both cameras, worst at the
    start and draining away as the run caught up: bay 6 was out by 5.3 m, bay
    4 by 3.3 m. Nothing landed on the wrong face or the wrong level, because
    those are snapped to the layout; only the along-aisle estimate is not, and
    that is the one that moved.

    A frame arriving while the previous one is still being decoded is dropped
    rather than queued, which is what keeps the pose fresh. Coverage can
    afford it: a code stays in the hires frame for about 2 m of travel and the
    boxes are 0.65 m apart at their closest.
    """

    def __init__(self, name, hfov_deg, yaw_offset_deg):
        self.name = name
        self.hfov_deg = hfov_deg
        # The rear camera is mounted looking backwards, so its frames are
        # interpreted with the heading turned through 180 degrees and the
        # existing geometry puts the code on the far side of the aisle.
        self.yaw_offset_deg = yaw_offset_deg
        # Perpendicular distance to the face this camera reads. The hires
        # holds the layout's standoff for the whole flight; the rear camera's
        # changes with the aisle, so the route sets it on entry to each lane.
        # None means this camera has no face to read on the current lane.
        self.depth = None
        self.detector = cv2.wechat_qrcode_WeChatQRCode()
        # The second pass detector, for a code WeChat locates but cannot
        # read. Per camera for the same reason as the first.
        self.fallback = cv2.QRCodeDetector()
        self.busy = False
        self.frames = 0
        self.dropped = 0
        self.stale = 0


HIRES = CameraDecoder("camera_hires_link", CAMERA_HFOV_DEG, 0.0)
REAR = CameraDecoder("camera_track_rear_link", TRACKING_HFOV_DEG, 180.0)

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


def decode_qr(frame, cam):
    """
    Decode QR codes and report where each one sits in the frame.

    WeChat's detector does the finding. The live view told us where the
    pipeline was losing codes: it draws orange for a code that was located but
    would not decode, and green for one that read, and there was never any
    orange. Missed codes were not being located at all.

    That is not a shortage of pixels. Given the same synthetic label at the
    size the real 1024x768 stream sees it, cv2.QRCodeDetector failed to find it
    at every distance from 0.6 m to 1.5 m, including 4.14 px per module, half
    again over the usual threshold. WeChat read it at all of them, down to
    1.66 px per module, and QRCodeDetectorAruco managed all but the furthest.

    Reading at the aisle centre depends on this. At 1.20 m from both faces
    there are 2.07 px per module, which the old detector never found and this
    one reads.

    The crop, threshold and upscale that follow are kept as a second pass for
    anything WeChat locates but cannot read: Gazebo renders the white quiet
    zone as mid grey, and a local threshold recovers the contrast.

    Returns a list of (value, centre_x_px, centre_y_px, frame_width,
    frame_height). The pixel position is needed to work out the bearing to the
    box: a code near the edge of the frame is off to the side, not straight
    ahead, and assuming otherwise puts the box metres away from where it is.
    """
    results = []
    frame_h, frame_w = frame.shape[:2]
    try:
        values, points = cam.detector.detectAndDecode(frame)
        for value, quad in zip(values, points):
            if value:
                p = np.asarray(quad)
                results.append((value, float(p[:, 0].mean()),
                                float(p[:, 1].mean()), frame_w, frame_h))
        if len(results) == len(points) and results:
            return results

        # Anything WeChat located but could not read gets the local threshold.
        points = [np.asarray(q) for v, q in zip(values, points) if not v]
        if not points:
            return results

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

            data, _, _ = cam.fallback.detectAndDecode(binary)
            if data:
                results.append((data, centre_x, centre_y, frame_w, frame_h))
                continue

            upscaled = cv2.resize(binary, None, fx=3.0, fy=3.0,
                                  interpolation=cv2.INTER_CUBIC)
            data_up, _, _ = cam.fallback.detectAndDecode(upscaled)
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
    handle_frame(msg, HIRES)


def on_rear_image(msg):
    """
    The same, through the rear tracking camera, looking at the opposite face.

    Flying the aisle centre puts both shelf faces at the same distance, so one
    pass can read both: the hires reads what the nose is pointed at and this
    reads what is behind. Every face still gets covered, in half the passes.

    Nothing else needs to change to place these boxes. The camera looks the
    other way, so the pose is recorded with the heading turned through 180
    degrees, and the existing geometry then puts the code on the far side of
    the aisle and snaps it to the face that is actually there.

    Its field of view is not the same as the hires, though. These are 90 degree
    lenses against 60, so a code at the same pixel offset is at a different
    bearing, and using one figure for both would place the far shelf wrongly.
    """
    handle_frame(msg, REAR)


def handle_frame(msg, cam):
    """Decode one frame from one camera and queue what it found."""
    taken_at = stamp_seconds(msg)
    age = sim_now["s"] - taken_at
    if sim_now["s"]:
        frame_ages.append(age)
        if age > MAX_FRAME_AGE_S:
            # Not because it cannot be placed; the history would place it. To
            # keep the queue from growing for the whole flight and costing the
            # codes at the end of it.
            cam.stale += 1
            frames_stale["n"] += 1
            return
    depth = cam.depth
    if depth is None:
        # No face for this camera on this lane, so anything it sees belongs to
        # a shelf the route is not scanning from here.
        return
    if cam.busy:
        # Still working on the previous frame. Drop this one rather than let
        # it wait: a queued frame is decoded with the pose of whenever the
        # decoder gets to it, not the pose it was taken at.
        cam.dropped += 1
        return
    cam.busy = True
    try:
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(
            (msg.height, msg.width, 3))
        # Where the vehicle was when this frame was taken, not where it is
        # now. Reading the current position instead is what put codes up to
        # 6.86 m from their shelves on a machine fast enough to outrun the
        # decoder. The drift offset is added so the result is in the true
        # Gazebo frame rather than the estimator's.
        was = pose_at(taken_at)
        if was is None:
            # Older than the history reaches. Nothing to place it against.
            cam.stale += 1
            frames_stale["n"] += 1
            return
        pose = (was[0] + drift_offset["n"],
                was[1] + drift_offset["e"],
                was[2],
                was[3] + cam.yaw_offset_deg)

        hits = decode_qr(cv2.cvtColor(img, cv2.COLOR_RGB2BGR), cam)
        cam.frames += 1
        for value, cx, cy, fw, fh in hits:
            pending.append((value, cx, cy, fw, fh, pose, cam.hfov_deg,
                            cam.name, depth))
    except Exception:
        pass
    finally:
        cam.busy = False


async def track_odometry(drone):
    """
    Mirror the estimator's pose, and keep a history of it stamped in
    simulation time.

    This replaces separate subscriptions to position_velocity_ned and
    attitude_euler. Odometry carries position, orientation and a timestamp
    together, and the position agrees with position_velocity_ned to 0.2 mm, so
    nothing is given up by taking all three from one message.

    The timestamp is the point. It is on the same clock as the image headers,
    and it arrives over UDP rather than through gz transport, so it stays
    current no matter how far behind the cameras fall.
    """
    global current_pos, current_yaw
    try:
        async for o in drone.telemetry.odometry():
            when = o.time_usec / 1e6
            q = o.q
            yaw = math.degrees(math.atan2(
                2 * (q.w * q.z + q.x * q.y),
                1 - 2 * (q.y * q.y + q.z * q.z)))

            current_pos["n"] = o.position_body.x_m
            current_pos["e"] = o.position_body.y_m
            current_pos["d"] = o.position_body.z_m
            current_yaw["deg"] = yaw
            sim_now["s"] = when

            pose_history.append((when, current_pos["n"], current_pos["e"],
                                 current_pos["d"], yaw))
            # Trim from the front rather than using a deque, so that the
            # binary search in pose_at can index it.
            cutoff = when - POSE_HISTORY_S
            if pose_history[0][0] < cutoff:
                keep = 0
                while (keep < len(pose_history)
                       and pose_history[keep][0] < cutoff):
                    keep += 1
                del pose_history[:keep]
    except Exception:
        pass


def record_detection(qr_id, cx_px, cy_px, frame_w, frame_h, pose,
                     hfov_deg=None, camera="camera_hires_link", depth=None):
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
    h_fov = math.radians(hfov_deg if hfov_deg else CAMERA_HFOV_DEG)
    v_fov = 2 * math.atan(math.tan(h_fov / 2) * frame_h / frame_w)

    dx_norm = (cx_px - frame_w / 2) / (frame_w / 2)    # -1 left, +1 right
    dy_norm = (cy_px - frame_h / 2) / (frame_h / 2)    # -1 top,  +1 bottom

    bearing = math.atan(dx_norm * math.tan(h_fov / 2))
    elevation = math.atan(dy_norm * math.tan(v_fov / 2))

    # The shelf face is a known perpendicular distance away, so the depth
    # along the optical axis is fixed and the lateral offset is what varies.
    # The two cameras are not the same distance from their faces: the lane
    # sits off centre so that each one is close enough to read. Using one
    # figure for both would scale every bearing on the far camera by the ratio
    # between them.
    if depth is None:
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
        # Which camera read it. Two cameras now scan two different faces on
        # the same pass, so a bare count no longer says which one is
        # earning its place.
        "camera": camera,
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
        qr, cx, cy, fw, fh, pose, hfov, cam, depth = pending.popleft()
        record_detection(qr, cx, cy, fw, fh, pose, hfov, cam, depth)


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
            return
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
    Where the vehicle flies to read a face with the hires camera:
    SHELF_STANDOFF out from the shelf surface, on the side it looks from.

    yaw +90 looks towards +x, so the vehicle sits at smaller x than the face;
    yaw -90 looks towards -x, so it sits at larger x. Deriving the lane this
    way rather than naming it directly keeps the standoff a single number that
    can be tuned for the camera without touching any coordinates.
    """
    if face["yaw_deg"] > 0:
        return face["face_x"] - SHELF_STANDOFF
    return face["face_x"] + SHELF_STANDOFF


def opposite_face(face, lane_x):
    """
    The face across the aisle from this one, or None if there is not one.

    A face is across the aisle when it looks back the other way and the lane
    lies between the two of them. The nearest such face is the one the rear
    camera sees; anything further is behind it.

    This is what makes the pairing a property of the warehouse rather than
    something written down. A row along a wall has nothing facing it and
    returns None, and its pass then uses the hires alone, exactly as every
    pass did before the rear camera was wired in.
    """
    candidates = []
    for other in AISLE_FACES:
        if other is face or other["yaw_deg"] * face["yaw_deg"] > 0:
            continue
        # The vehicle must sit between the two faces for one to be behind it.
        if (other["face_x"] - lane_x) * (face["face_x"] - lane_x) >= 0:
            continue
        candidates.append(other)
    if not candidates:
        return None
    return min(candidates, key=lambda f: abs(f["face_x"] - lane_x))


def build_route():
    """
    Build one pass per flight lane per level, in a continuous boustrophedon.

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
    # Group the faces by the lane they are flown from. Two faces across an
    # aisle share a lane exactly when the standoff equals half the aisle
    # width, which is what flying the centre line means. Where they do, the
    # hires reads the face ahead and the rear tracking camera reads the one
    # behind, so the pair costs one pass instead of two. Where a face has no
    # partner, an outer row along a wall, the group has one member and it is
    # flown as before.
    # One lane per face the hires reads, skipping any face that is already
    # covered by the rear camera on an earlier lane. Two faces across an aisle
    # therefore cost one pass rather than two.
    lanes = []
    covered = []
    for face in AISLE_FACES:
        if any(face is c for c in covered):
            continue
        lane_x = round(face_lane_x(face), 3)
        behind = opposite_face(face, lane_x)
        # What the rear camera is looking at from this lane, if anything.
        rear_depth = abs(behind["face_x"] - lane_x) if behind else None
        lanes.append({"x": lane_x, "yaw": face["yaw_deg"],
                      "rear_depth": rear_depth})
        covered.append(face)
        if behind:
            covered.append(behind)

    route = []
    heading_north = True
    levels_ascending = True
    for lane in lanes:
        levels = FLIGHT_Z if levels_ascending else list(reversed(FLIGHT_Z))
        for z in levels:
            ends = ((Y_SOUTH, Y_NORTH) if heading_north
                    else (Y_NORTH, Y_SOUTH))
            for y in ends:
                route.append((lane["x"], y, z, lane["yaw"],
                              lane["rear_depth"]))
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
        await send_setpoint(drone, cn, ce,
                            start_d + (target_d - start_d) * fraction, yaw_deg)
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
                # Both axes. Checking only altitude let a change of aisle,
                # which is a move in x, pass straight through: the vehicle was
                # already at the right height, so it began the leg still up to
                # WAYPOINT_TOLERANCE out of the lane.
                lateral = math.sqrt(
                    (current_pos["n"] + drift_offset["n"] - target_n) ** 2 +
                    (current_pos["e"] + drift_offset["e"] - target_e) ** 2)
                if (abs(-current_pos["d"] - z) < SETTLE_TOLERANCE
                        and lateral < SETTLE_LATERAL_M):
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


def save_inventory(reached, planned):
    """
    Write the inventory, without destroying what is already there.

    Opening the target with "w" truncates it before anything is written, so a
    failure at that moment leaves nothing at all. One run ended exactly that
    way: the summary printed, the file was emptied, and the process was killed
    by the kernel before the write completed. 258 decoded codes went with it.

    Writing beside the target and renaming means the previous file survives
    until a complete one replaces it, and rename is atomic.
    """
    payload = {
        "scan_date": datetime.now().isoformat(timespec="seconds"),
        "sensor_configuration": "C27, single front-facing scanning camera",
        "localization": "visual odometry, no GPS",
        "total_detected": len(inventory),
        "waypoints_completed": f"{reached}/{planned}",
        "marker_corrections": marker_events,
        "final_drift_offset_m": round(
            math.hypot(drift_offset["n"], drift_offset["e"]), 3),
        "items": list(inventory.values()),
    }
    tmp = OUTPUT_JSON + ".tmp"
    with open(tmp, "w", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=2, ensure_ascii=False)
    os.replace(tmp, OUTPUT_JSON)


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
        # A frame that arrives while the previous one is still decoding is
        # dropped, so that the pose recorded with a code is the pose the frame
        # was taken at. A high drop count is not itself a fault, but it is the
        # first thing to look at if coverage falls.
        "frames": {
            cam.name: {"decoded": cam.frames, "dropped": cam.dropped,
                       "too_old": cam.stale}
            for cam in (HIRES, REAR)
        },
        # How far behind the cameras ran. This is the measurement whose
        # absence hid a scan that put two per cent of its codes on the right
        # shelf: the report said nothing was wrong, when what it meant was
        # that two broken checks had not fired.
        "frame_age_s": {
            "median": round(sorted(frame_ages)[len(frame_ages) // 2], 3)
                      if frame_ages else None,
            "p95": round(sorted(frame_ages)[int(len(frame_ages) * 0.95)], 3)
                   if frame_ages else None,
            "max": round(max(frame_ages), 3) if frame_ages else None,
            "samples": len(frame_ages),
            "dropped_for_age": frames_stale["n"],
            "limit": MAX_FRAME_AGE_S,
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

    # The TOF is a handful of numbers and is wanted from the moment the
    # vehicle leaves the ground, so it subscribes here. The cameras do not:
    # see the note above the subscription further down.
    tof_node = trans.Node()
    tof_node.subscribe(LaserScan, TOF_SCAN, on_tof_scan)

    drone = System()
    await drone.connect(system_address="udp://:14540")
    print("\n[INFO] Connecting to vehicle")
    async for state in drone.core.connection_state():
        if state.is_connected:
            break

    asyncio.create_task(track_odometry(drone))

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

    # Subscribe to the cameras here, with the vehicle already on the route
    # heading and about to move, rather than at the top of run.
    #
    # Subscribing at the top left gz transport queueing frames through
    # everything above it: connecting to the vehicle, waiting for a position
    # estimate, arming, a seven second ascent and the turn onto the route.
    # Those frames are delivered afterwards, one at a time, each holding an
    # image from before the vehicle moved, and the pose is read when the
    # callback runs rather than when the frame was taken.
    #
    # The first two camera run measured the cost. Nothing was recorded until
    # sixteen seconds into a leg that starts at y = -9, then codes arrived in
    # a burst, each naming a shelf position up to 8.8 m behind the vehicle,
    # with the gap closing steadily as the queue drained. Every one of the
    # remaining 384 codes landed within 0.17 m. The direction is what
    # identifies it: a stale pose would put a code south of its shelf, since
    # the vehicle is heading north, and these were north of it, so the pose
    # was current and the image was old.
    #
    # There is no steady state problem to go with it. 3348 hires frames and
    # 1006 rear frames over 804.5 s is 41.6 per cent of nominal for both,
    # which is the real time factor: every frame rendered was decoded.
    cam_node = trans.Node()
    cam_node.subscribe(Image, CAM_HIRES, on_hires_image)
    rear_node = trans.Node()
    rear_node.subscribe(Image, CAM_REAR, on_rear_image)
    down_node = trans.Node()
    down_node.subscribe(Image, CAM_DOWN, on_down_image)

    # Let the cameras deliver whatever gz transport has been holding while
    # the vehicle is still standing still. Every run so far has put its first
    # fifteen seconds of codes up to three metres along the aisle from their
    # shelves, decaying smoothly to nothing, which is the shape of a queue
    # draining. Drained here, those frames carry a pose that is not moving.
    print("[INFO] Letting the cameras catch up")
    for _ in range(50):
        await send_setpoint(drone, start_n, start_e,
                            start_d - 1.5, YAW_NORTH)
        await asyncio.sleep(0.1)
    pending.clear()
    inventory.clear()

    print("\n[INFO] Starting scan")
    mission_start = time.time()
    reached = 0
    last_yaw = YAW_NORTH
    for index, (x, y, z, yaw, rear_depth) in enumerate(route, 1):
        # Each camera is told how far its face is before the leg begins, so a
        # frame decoded during it is placed against the right distance.
        HIRES.depth = SHELF_STANDOFF - HIRES_MOUNT_X
        REAR.depth = (rear_depth + REAR_MOUNT_X) if rear_depth else None
        if yaw != last_yaw:
            await hold_heading(drone, yaw)
            last_yaw = yaw
        if await goto_waypoint(drone, index, len(route), x, y, z, yaw):
            reached += 1
        # Save as we go. Runs here have been cut short often enough by the
        # simulator running out of memory that keeping everything until the
        # end has already cost one complete scan.
        save_inventory(reached, len(route))

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

    save_inventory(reached, len(route))
    print(f"\n[INFO] Inventory written to {OUTPUT_JSON}")

    write_navigation_report(reached, len(route), time.time() - mission_start)

    print("[INFO] Landing")
    await drone.offboard.stop()
    await drone.action.land()


if __name__ == "__main__":
    asyncio.run(run())
