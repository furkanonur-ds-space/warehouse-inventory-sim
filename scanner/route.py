"""
Where the warehouse is and where the vehicle has to fly to read it.

Split out of scanner.py unchanged. This is the half of the scan that does
not touch a camera, a simulator or an autopilot: it reads layout.json and
answers where the lanes are, what altitude each level is flown at, and in
what order. Everything here is arithmetic over the layout, which is why it
is the piece that can be ported to C and run on the vehicle.

The seam is deliberate. On the drone the route is what travels: the flight
service asks for waypoints and headings, and the reading of codes is done by
something else entirely. Keeping this module free of I/O is what makes both
possible, and it is also what lets the port be checked, since the same
layout must produce the same waypoints from either language.
"""

import json
import math
import os

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
# The furthest each camera can be from a shelf and still read it, measured by
# standoff_sweep.py against real label textures. These are properties of the
# camera and the label, not of the building, and they are what decides where
# the lane goes in an aisle of any width.
HIRES_MAX_STANDOFF = LAYOUT.get("hires_max_standoff", 1.30)
REAR_MAX_STANDOFF = LAYOUT.get("rear_max_standoff", 1.10)

# Where to stand off a face that has no aisle: a row along a wall, read by the
# hires alone. Also the fallback for a layout that names no limits.
SHELF_STANDOFF = LAYOUT.get("shelf_standoff", HIRES_MAX_STANDOFF)

# How wide the vehicle is, so an aisle it does not fit down can be named
# rather than flown into. The Starling 2 is 290 mm across at the propeller
# tips.
VEHICLE_HALF_SPAN = LAYOUT.get("vehicle_half_span", 0.0)

# How much room to leave between a propeller tip and a shelf.
AISLE_CLEARANCE_M = 0.05

# Where each camera sits along the body, from build_c27_drone.py. The bearing
# to a code is turned into a position using the distance from the camera to
# the shelf, and the camera is not at base_link: the hires looks forward from
# the front face and the rear camera looks back from the rear one, so each is
# nearer its own shelf than base_link is. Leaving these out scaled every
# lateral offset by eight per cent.
HIRES_MOUNT_X = 0.06
REAR_MOUNT_X = -0.055

# --- WHAT THE CAMERA CAN SEE VERTICALLY --------------------------------
#
# The frame is a rectangle whose height grows with the distance to the shelf.
# The hires stands 1.24 m off in the 2.40 m aisle and sees 1.07 m of shelf; it
# stands 0.21 m off in the 0.50 m aisle and sees 0.18 m. Everything below
# follows from that collapse, and so does the reason the narrow aisle needs
# the optical axis aimed at its codes while the wide ones do not care.
#
# What the cameras are rendered at, from build_c27_drone.py. The vertical
# field is the horizontal one scaled by the aspect ratio, and the two cameras
# share neither: 60 degrees over 1024x768 against 90 over 1280x800.
HIRES_FRAME_PX = (1024, 768)
REAR_FRAME_PX = (1280, 800)

# How much of the geometric field is worth counting on. report/warehouse_model
# measures 0.23 m of half-frame where the lens gives 0.260 at the same
# distance; the shortfall is vignetting and the decode margin at the edge of
# the frame. Kept as a ratio because it belongs to the camera and not to any
# one distance.
USABLE_FRAME = 0.885

# How tall a code is, so that the question can be whether a whole one fits
# rather than whether its centre does. A code clipped by the frame edge does
# not decode at all; there is no partial credit for most of a QR.
CODE_SIZE_M = LAYOUT.get("code_size_m", 0.072)

# How far the plane the codes sit in is from the shelf surface, into the
# shelf. Reporting the surface instead put every one of 432 codes 0.016 m out
# in x, with the sign following the face. Zero for a layout that does not say.
CODE_PLANE_OFFSET_M = LAYOUT.get("code_plane_offset_m", 0.0)
CODE_MODULE_SIZE_M = LAYOUT.get("code_module_size_m", 0.0)

# How much detail the decoder is given, in pixels per QR module.
#
# WeChat's detector has been measured on these labels down to 1.66 pixels a
# module and is reliable from about 2.0. Four is that with the margin doubled,
# and it is a ceiling rather than a target: a frame is only ever scaled down to
# reach it, never up, so the aisles that resolve 2.06 and 1.76 are handed the
# pixels they have and are not touched.
#
# What it buys is on the narrow aisle, where the camera is 0.21 m from the
# shelf and a code covers 303 pixels. Decoding that took 118 ms of a 157 ms
# frame interval, which is a decoder with no room to be given more frames.
DECODE_TARGET_PX_PER_MODULE = 4.0


def half_frame_m(hfov_deg, frame_px, depth):
    """Half the camera's vertical field, in metres, at this distance."""
    width, height = frame_px
    tan_half_v = math.tan(math.radians(hfov_deg) / 2) * height / width
    return depth * tan_half_v * USABLE_FRAME

def face_at(x):
    """The shelf face nearest this x, or None if the layout has none at all."""
    if not AISLE_FACES:
        return None
    return min(AISLE_FACES, key=lambda f: abs(f["face_x"] - x))


def shelf_name(face_x):
    """The name of the shelf face at this x, or None if the layout has none."""
    for face in AISLE_FACES:
        if abs(face["face_x"] - face_x) < 0.01:
            return face.get("name")
    return None


def code_plane_x(face):
    """
    The x of the plane the codes on this face actually sit in.

    face_x is the shelf surface. A label is mounted on the box behind it, so
    the code plane is code_plane_offset_m deeper into the shelf, away from the
    aisle the vehicle flies. Which way that is comes from the heading the face
    is read at: a face read at +90 is looked at along +x, so deeper is +x.

    Measured, not assumed: scored against ground truth every one of 432 codes
    came out 0.016 m from where it was reported, with the sign following the
    face. It is a property of how the warehouse mounts its labels, so it lives
    in the layout.
    """
    return face["face_x"] + (CODE_PLANE_OFFSET_M if face["yaw_deg"] > 0
                             else -CODE_PLANE_OFFSET_M)


def code_height(face, level_index, measured):
    """
    The height to report for a code, from what was measured and what can exist.

    Keep the measurement, but not a height this shelf cannot hold. The layout
    says the lowest and highest a code sits at on this face and level, so a
    measurement outside that came from a tilted camera rather than from a box.

    The four candidates, scored on a finished run against ground truth, as
    height error alone:

                      median     p95      max   within 5 cm
        measured      0.0243  0.0524   0.2207        93.8%
        flight_z      0.0500  0.0600   0.0600        32.4%
        band median   0.0210  0.0710   0.0890        74.1%
        clamped       0.0149  0.0499   0.1210        95.1%

    flight_z is what this used to report, and it is the worst of the four by a
    long way: it is the altitude the vehicle flew at, which was never an
    estimate of anything on a shelf. It looks respectable only in its maximum,
    and a constant always does.

    The measurement on its own is good for nine codes in ten and then produces
    a 0.22 m outlier. Clamping keeps the nine and bounds the tenth, and wins on
    every figure except a maximum that a constant will always own.

    Falls back to flight_z when the layout does not say where its codes are.
    """
    band = face.get("code_z") if face else None
    if not band or level_index >= len(band):
        return FLIGHT_Z[level_index]
    low, high = band[level_index][0], band[level_index][-1]
    return min(max(measured, low), high)


def face_lane_x(face, standoff):
    """
    Where the vehicle flies to read a face: standoff out from the shelf
    surface, on the side the camera looks from.

    yaw +90 looks towards +x, so the vehicle sits at smaller x than the face;
    yaw -90 looks towards -x, so it sits at larger x. Deriving the lane rather
    than naming it keeps the coordinates out of the layout.
    """
    if face["yaw_deg"] > 0:
        return face["face_x"] - standoff
    return face["face_x"] + standoff


def facing_face(face):
    """
    The face across the aisle from this one, or None if it stands alone.

    A face is across the aisle when it looks back the other way and sits on
    the side the vehicle would be. The nearest such face is the one the rear
    camera sees; anything beyond it is behind a shelf.

    This is what makes the pairing a property of the warehouse rather than
    something written down. A row along a wall has nothing facing it, and its
    pass then uses the hires alone.
    """
    ahead = 1.0 if face["yaw_deg"] < 0 else -1.0     # which way the vehicle is
    candidates = [
        other for other in AISLE_FACES
        if other is not face
        and other["yaw_deg"] * face["yaw_deg"] < 0
        and (other["face_x"] - face["face_x"]) * ahead > 0
    ]
    if not candidates:
        return None
    return min(candidates, key=lambda f: abs(f["face_x"] - face["face_x"]))


def split_aisle(width):
    """
    How far the lane sits from each face of an aisle of this width.

    Both distances add up to the width, and each has to stay inside what its
    camera can read. Splitting in proportion to the two limits keeps both at
    the same fraction of their own reach whatever the aisle, and reduces to
    the limits themselves when the aisle is exactly as wide as they allow.

    Returns None for an aisle too wide to read from one lane, which then costs
    a pass a face instead of one for both.
    """
    total = HIRES_MAX_STANDOFF + REAR_MAX_STANDOFF
    hires = width * HIRES_MAX_STANDOFF / total
    rear = width - hires
    if rear > REAR_MAX_STANDOFF:
        rear = REAR_MAX_STANDOFF
        hires = width - rear
    if hires > HIRES_MAX_STANDOFF:
        return None
    return hires, rear


def aisle_fits(width):
    """Whether the vehicle can fly this aisle at all, with room to spare."""
    if VEHICLE_HALF_SPAN <= 0:
        return True
    return width / 2.0 - VEHICLE_HALF_SPAN >= AISLE_CLEARANCE_M

def lane_levels(reads):
    """
    What altitude to fly at each level on this lane, and whether it is enough.

    One rule, applied to every aisle: put the optical axis in the middle of
    the band of code heights the lane has to read. A face whose boxes are all
    one size carries its codes at a single height, and the axis lands on them
    exactly; a face with mixed box heights spreads them, because the label
    travels with the front of the box it is on, and the axis lands in the
    middle of the spread. That is what flight_z was already approximating with
    a median taken over the whole building.

    It changes almost nothing in a wide aisle and everything in a narrow one.
    The hires frame is 1.07 m tall in the 2.40 m aisle, so an axis 0.05 m out
    is not worth naming. It is 0.18 m tall in the 0.50 m aisle, and rows G and
    H carry their codes 0.049 m below the building median. Measured in the
    2026-09-01 recording, their codes sat at 0.76 of frame height with the
    worst survivor at 0.97, against 0.49 for row A; both of that run's narrow
    aisle misses were on those two faces, opposite each other.

    The second half is the check, and it is the half that travels to a
    warehouse we have not seen. Below some aisle width a band of code heights
    does not fit the frame at all, whatever the axis: at 0.50 m one pass
    covers 0.09 m of band, and the mixed box heights on rows A to F span
    0.11 m. Saying so out loud is the point. A level read half way looks in
    the inventory exactly like a level holding half as many boxes, which is
    the vertical twin of the warning aisle_fits already prints for a width.

    `reads` is one entry per face this lane reads: the face, its camera's
    field of view and frame, and how far that camera is from it.
    """
    levels = []
    for index, fallback in enumerate(FLIGHT_Z):
        bands = [face["code_z"][index] for face, _, _, _ in reads
                 if face.get("code_z")]
        if len(bands) != len(reads):
            # A layout that does not say where its codes are keeps the old
            # behaviour. Guessing would be worse than the median it replaces.
            levels.append(fallback)
            continue

        # The outer two of the three figures a face carries: the lowest
        # code and the highest. What has to fit the frame is the whole
        # spread, not where most of them sit.
        axis = (min(b[0] for b in bands) + max(b[-1] for b in bands)) / 2.0
        levels.append(round(axis, 3))

        for face, hfov_deg, frame_px, depth in reads:
            low, high = (face["code_z"][index][0],
                         face["code_z"][index][-1])
            reach = max(abs(low - axis), abs(high - axis)) + CODE_SIZE_M / 2
            limit = half_frame_m(hfov_deg, frame_px, depth)
            if reach > limit:
                print(f"[WARN] face {face.get('name')} level {index + 1}: its "
                      f"codes span {high - low:.3f} m, and the camera sees "
                      f"{2 * limit:.3f} m of shelf from {depth:.3f} m away. "
                      f"{reach - limit:.3f} m of that band falls outside the "
                      f"frame whatever the altitude, so this level cannot be "
                      f"read completely in one pass.")
    return levels


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
    # One lane per aisle, with both its faces read from it, and a lane of its
    # own for any face that has nothing opposite. Where the lane sits comes
    # from the width of the aisle, since the two cameras have to share it.
    lanes = []
    covered = []
    skipped = []
    for face in AISLE_FACES:
        if any(face is c for c in covered):
            continue
        opposite = facing_face(face)

        if opposite is None:
            # A row along a wall. The hires reads it alone, from as far back
            # as it can still read, and there is nothing behind.
            hires = min(SHELF_STANDOFF, HIRES_MAX_STANDOFF)
            lanes.append({"x": round(face_lane_x(face, hires), 3),
                          "yaw": face["yaw_deg"],
                          "hires_depth": hires, "rear_depth": None,
                          "hires_face": face, "rear_face": None})
            covered.append(face)
            continue

        width = abs(opposite["face_x"] - face["face_x"])
        if not aisle_fits(width):
            skipped.append((face, opposite, width))
            covered.extend((face, opposite))
            continue

        split = split_aisle(width)
        if split is None:
            # Wider than both cameras together can cover, so each face gets
            # its own pass, as everything did before the rear camera existed.
            for one in (face, opposite):
                hires = min(SHELF_STANDOFF, HIRES_MAX_STANDOFF)
                lanes.append({"x": round(face_lane_x(one, hires), 3),
                              "yaw": one["yaw_deg"],
                              "hires_depth": hires, "rear_depth": None,
                              "hires_face": one, "rear_face": None})
            covered.extend((face, opposite))
            continue

        hires, rear = split
        lanes.append({"x": round(face_lane_x(face, hires), 3),
                      "yaw": face["yaw_deg"],
                      "hires_depth": hires, "rear_depth": rear,
                      "hires_face": face, "rear_face": opposite})
        covered.extend((face, opposite))

    for face, opposite, width in skipped:
        print(f"[WARN] aisle between {face.get('name')} and "
              f"{opposite.get('name')} is {width:.2f} m; the vehicle is "
              f"{2 * VEHICLE_HALF_SPAN:.2f} m across and needs "
              f"{AISLE_CLEARANCE_M:.2f} m a side. Not flown.")

    # Where the optical axis goes on each lane. Done here, once the lane knows
    # both the faces it reads and how far its cameras are from them, because
    # the answer depends on all three.
    for lane in lanes:
        reads = [(lane["hires_face"], CAMERA_HFOV_DEG, HIRES_FRAME_PX,
                  lane["hires_depth"] - HIRES_MOUNT_X)]
        if lane["rear_face"] is not None and lane["rear_depth"] is not None:
            reads.append((lane["rear_face"], TRACKING_HFOV_DEG, REAR_FRAME_PX,
                          lane["rear_depth"] + REAR_MOUNT_X))
        lane["z"] = lane_levels(reads)
        if lane["z"] != FLIGHT_Z:
            names = "/".join(f.get("name", "?") for f in
                             (lane["hires_face"], lane["rear_face"]) if f)
            print("[INFO] lane %s: flying %s rather than %s, to put the axis "
                  "on the codes" % (names, lane["z"], FLIGHT_Z))

    route = []
    heading_north = True
    levels_ascending = True
    for lane in lanes:
        levels = lane["z"] if levels_ascending else list(reversed(lane["z"]))
        for z in levels:
            ends = ((Y_SOUTH, Y_NORTH) if heading_north
                    else (Y_NORTH, Y_SOUTH))
            for y in ends:
                route.append((lane["x"], y, z, lane["yaw"],
                              lane["hires_depth"], lane["rear_depth"]))
            heading_north = not heading_north
        levels_ascending = not levels_ascending
    return route
