"""
Check that the camera is aimed at the codes, and that it says so when it cannot be.

The vertical frame is the half of the geometry that had never been checked,
and it cost the narrow aisle two codes a run. The frame is a rectangle whose
height grows with the distance to the shelf: 1.07 m for the hires in the
2.40 m aisle and 0.18 m in the 0.50 m one. An axis 0.05 m off the codes is
invisible in the first and fatal in the second.

Two things can go wrong here and they fail differently. The axis may be put in
the wrong place, which costs codes on whichever face is tightest and shows up
in flight as a handful of misses that move around between runs. Or the fit
check may not fire when a band genuinely does not fit, which shows up as
nothing at all: the scan reports fewer boxes and reads as a shelf holding
fewer boxes, rather than as a shelf that was never fully seen.

So this measures both, against the layout the scanner actually flies, and
takes seconds rather than the thirteen minutes a scan takes to tell you only
its total.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import scanner as s

failures = []


def check(label, ok, detail=""):
    print("  %-58s %s %s" % (label, "ok" if ok else "FAILED", detail))
    if not ok:
        failures.append(label)


def reads_for(hires_face, rear_face, hires_depth, rear_depth):
    """The argument lane_levels takes, built the way build_route builds it."""
    out = [(hires_face, s.CAMERA_HFOV_DEG, s.HIRES_FRAME_PX,
            hires_depth - s.HIRES_MOUNT_X)]
    if rear_face is not None:
        out.append((rear_face, s.TRACKING_HFOV_DEG, s.REAR_FRAME_PX,
                    rear_depth + s.REAR_MOUNT_X))
    return out


faces = {f["name"]: f for f in s.AISLE_FACES}

print("the axis lands where the codes are")

# Rows G and H carry boxes of one size, so their codes sit at a single height
# and the axis should land on it exactly. This is the aisle that was losing
# them: 0.049 m below the building median, in a frame 0.18 m tall.
levels = s.lane_levels(reads_for(faces["G"], faces["H"], 0.2708, 0.2292))
band = faces["G"]["code_z"]
check("single height band puts the axis on the codes",
      all(abs(z - b[0]) < 1e-6 for z, b in zip(levels, band)),
      "(flying %s, codes at %s)" % (levels, [b[0] for b in band]))

# Rows A and B carry mixed box heights, so their codes are spread and the best
# the axis can do is the middle of the spread.
levels = s.lane_levels(reads_for(faces["A"], faces["B"], 1.30, 1.10))
low, high = faces["A"]["code_z"][0][0], faces["A"]["code_z"][0][-1]
check("spread band puts the axis in the middle of the spread",
      abs(levels[0] - (low + high) / 2) < 1e-3,
      "(flying %.3f, band %.3f to %.3f)" % (levels[0], low, high))

check("a lane with no rear face still gets an axis",
      len(s.lane_levels(reads_for(faces["A"], None, 1.30, None)))
      == len(s.FLIGHT_Z))

print("\na layout that does not say where its codes are keeps the old behaviour")
bare = {"name": "bare", "face_x": 0.0, "yaw_deg": 90}
check("falls back to flight_z",
      s.lane_levels(reads_for(bare, None, 1.30, None)) == s.FLIGHT_Z,
      "(got %s)" % s.lane_levels(reads_for(bare, None, 1.30, None)))

print("\nevery code this layout describes fits the frame it is read from")
# Pairing is followed the way build_route follows it, taking each aisle from
# the face that leads it and marking both as covered. Walking every face
# instead would also score the aisle backwards, which is a lane the vehicle
# never flies: it would report face B as read by the hires when the rear
# camera reads it, at a distance nothing stands at.
covered = []
for face in s.AISLE_FACES:
    if any(face is c for c in covered):
        continue
    opposite = s.facing_face(face)
    if opposite is None:
        continue
    covered.extend((face, opposite))
    width = abs(opposite["face_x"] - face["face_x"])
    split = s.split_aisle(width)
    if split is None or not s.aisle_fits(width):
        continue
    hires_depth, rear_depth = split
    reads = reads_for(face, opposite, hires_depth, rear_depth)
    levels = s.lane_levels(reads)
    for one, hfov, frame_px, depth in reads:
        limit = s.half_frame_m(hfov, frame_px, depth)
        worst = max(
            max(abs(low - axis), abs(high - axis)) + s.CODE_SIZE_M / 2
            for axis, band in zip(levels, one["code_z"])
            for low, high in [(band[0], band[-1])])
        check("face %s at %.3f m, worst code reaches %.4f of %.4f m"
              % (one["name"], depth, worst, limit), worst < limit,
              "(margin %+.4f m)" % (limit - worst))

print("\nand it complains when a band genuinely does not fit")
# The same 0.50 m aisle, but carrying the mixed box heights that rows A to F
# have. One pass covers 0.09 m of band there and those rows span 0.11 m, so
# this is not a hypothetical: it is what this warehouse would do if its
# narrowest aisle held its ordinary stock.
crowded = dict(faces["G"], name="crowded", code_z=faces["A"]["code_z"])
axis = s.lane_levels(reads_for(crowded, faces["H"], 0.2708, 0.2292))
low, high = crowded["code_z"][0][0], crowded["code_z"][0][-1]
limit = s.half_frame_m(s.CAMERA_HFOV_DEG, s.HIRES_FRAME_PX,
                       0.2708 - s.HIRES_MOUNT_X)
reach = max(abs(low - axis[0]), abs(high - axis[0])) + s.CODE_SIZE_M / 2
check("a 0.11 m band does not fit the 0.50 m aisle", reach > limit,
      "(needs %.4f m, frame gives %.4f m)" % (reach, limit))
check("  and the shortfall is worth naming", reach - limit > 0.005,
      "(short by %.4f m)" % (reach - limit))

print("\nwhere a code is reported, once it has been read")
# The two axes are constrained differently by the building and are treated
# differently because of it. There are eight shelf planes and a code is on one
# of them, so x is decided. Height is continuous within a level, so the
# measurement is kept and only bounded by what the shelf can hold.
#
# Both used to be snapped to a constant, and the constants were the whole of
# the reported position error: 0.016 m in x on every one of 432 codes, and up
# to 0.060 m in z, against 0.010 m on the one axis that was measured.
for name in ("A", "G"):
    face = faces[name]
    expected = (face["face_x"] + s.CODE_PLANE_OFFSET_M if face["yaw_deg"] > 0
                else face["face_x"] - s.CODE_PLANE_OFFSET_M)
    check("face %s reports the code plane, not the shelf surface" % name,
          abs(s.code_plane_x(face) - expected) < 1e-9,
          "(%.4f, surface at %.4f)" % (s.code_plane_x(face), face["face_x"]))

low, mid, high = faces["A"]["code_z"][0]
check("a height inside the band is kept as measured",
      abs(s.code_height(faces["A"], 0, mid) - mid) < 1e-9)
check("a height above the band is pulled back to it",
      abs(s.code_height(faces["A"], 0, high + 0.5) - high) < 1e-9,
      "(measured %.3f, band tops out at %.3f)" % (high + 0.5, high))
check("a height below the band is pulled back to it",
      abs(s.code_height(faces["A"], 0, low - 0.5) - low) < 1e-9)

# G carries one box size, so its band has no width and the clamp is exact
# whatever the camera thought it saw.
gl = faces["G"]["code_z"][0][0]
check("a band with no width admits one height only",
      abs(s.code_height(faces["G"], 0, gl + 0.4) - gl) < 1e-9,
      "(codes all at %.3f)" % gl)

check("a face that does not say falls back to flight_z",
      abs(s.code_height(bare, 0, 99.0) - s.FLIGHT_Z[0]) < 1e-9,
      "(got %.3f)" % s.code_height(bare, 0, 99.0))

print("\n%s" % ("all checks passed" if not failures
                else "%d FAILED: %s" % (len(failures), ", ".join(failures))))
sys.exit(1 if failures else 0)
