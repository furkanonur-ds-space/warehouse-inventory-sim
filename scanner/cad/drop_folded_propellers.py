"""Take the folded propeller blades out of the Starling 2 visual mesh.

    ~/cad_venv/bin/python drop_folded_propellers.py <in.stl> <out.stl>

ModalAI's STEP has the propellers folded for transport: two blades per motor
lying back along the arm towards the body. Drawn like that the vehicle looks
parked, and the simulator already draws each propeller as a translucent disc
where it spins (build_starling2.py), so the blades are taken out.

A blade is found by shape, not by name, since the mesh has no names left: a
separate part, thin in Y (the file's up) and lying in the propeller plane,
long, and out over an arm rather than over the body. Eight are expected, two
per motor; any other count means the file is not the one this was written
for, and nothing is written.

Coordinates are the file's millimetres and axes, as step_to_stl.py leaves
them: Y up, -Z forward, motors at X +-88 and Z +-66.
"""
import struct
import sys

import numpy as np

PLANE_Y = (14.0, 22.0)      # the propeller plane, both faces of a blade inside
MIN_LENGTH = 40.0           # a blade is about 50 mm long folded
MIN_OFFSET = 30.0           # centre out over an arm in both X and Z
EXPECTED = 8

src, dst = sys.argv[1], sys.argv[2]

with open(src, "rb") as f:
    header = f.read(80)
    n = struct.unpack("<I", f.read(4))[0]
    rec = np.frombuffer(f.read(n * 50), dtype=np.dtype(
        [("n", "<f4", 3), ("v", "<f4", 9), ("a", "<u2")]))
tri = rec["v"].reshape(-1, 3, 3).astype(np.float64)

# Parts are the connected pieces of the mesh: welded vertices, then union-find
# over every triangle's corners.
_, vertex = np.unique(np.round(tri.reshape(-1, 3), 4), axis=0, return_inverse=True)
vertex = vertex.reshape(-1, 3)
parent = np.arange(vertex.max() + 1)


def root(a):
    while parent[a] != a:
        parent[a] = parent[parent[a]]
        a = parent[a]
    return a


for a, b, c in vertex:
    ra = root(a)
    for other in (b, c):
        ro = root(other)
        if ro != ra:
            parent[ro] = ra
part = np.array([root(v) for v in vertex[:, 0]])

blades = []
for p in np.unique(part):
    pts = tri[part == p].reshape(-1, 3)
    lo, hi = pts.min(0), pts.max(0)
    centre = (lo + hi) / 2
    length = max(hi[0] - lo[0], hi[2] - lo[2])
    if (lo[1] >= PLANE_Y[0] and hi[1] <= PLANE_Y[1] and length >= MIN_LENGTH
            and abs(centre[0]) >= MIN_OFFSET and abs(centre[2]) >= MIN_OFFSET):
        blades.append(p)
        print("blade: %5d triangles, centre (%6.1f, %5.1f, %6.1f) mm, %.0f mm long"
              % ((part == p).sum(), *centre, length))

if len(blades) != EXPECTED:
    sys.exit("found %d blades, expected %d; nothing written" % (len(blades), EXPECTED))

keep = ~np.isin(part, blades)
with open(dst, "wb") as out:
    out.write(header)
    out.write(struct.pack("<I", int(keep.sum())))
    out.write(rec[keep].tobytes())
print("%d of %d triangles kept, written %s" % (keep.sum(), n, dst))
