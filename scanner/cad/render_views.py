"""Draw three orthographic views of a binary STL, to check axes by eye.

    ~/autonomous_landing/venv/bin/python render_views.py mesh.stl out_prefix

Writes out_prefix_top.png (looking down the CAD's +Y onto X and Z),
out_prefix_side.png (looking along Z onto X and Y) and out_prefix_end.png
(looking along X onto Z and Y). Each image is labelled with its axes, so
which end is the front can be read off the picture instead of guessed.

No renderer is needed: points are sampled over every triangle, the nearest
one per pixel wins, and its brightness comes from how squarely the triangle
faces the viewer.
"""
import struct
import sys

import numpy as np
from PIL import Image, ImageDraw

path, prefix = sys.argv[1], sys.argv[2]
with open(path, "rb") as handle:
    handle.read(80)
    count = struct.unpack("<I", handle.read(4))[0]
    raw = np.frombuffer(handle.read(count * 50), dtype=np.uint8).reshape(count, 50)
tri = raw[:, 12:48].copy().view("<f4").reshape(count, 3, 3).astype(np.float64)

# Normals and areas, then points spread over each triangle by area.
e1 = tri[:, 1] - tri[:, 0]
e2 = tri[:, 2] - tri[:, 0]
cross = np.cross(e1, e2)
area = 0.5 * np.linalg.norm(cross, axis=1)
normal = cross / np.maximum(np.linalg.norm(cross, axis=1, keepdims=True), 1e-12)
samples = np.clip(np.ceil(area / 0.15).astype(int), 3, 400)
idx = np.repeat(np.arange(count), samples)
r1 = np.sqrt(np.random.rand(len(idx)))
r2 = np.random.rand(len(idx))
pts = ((1 - r1)[:, None] * tri[idx, 0] + (r1 * (1 - r2))[:, None] * tri[idx, 1]
       + (r1 * r2)[:, None] * tri[idx, 2])
nrm = normal[idx]
print("triangles %d, sample points %d" % (count, len(pts)))

lo, hi = pts.min(axis=0), pts.max(axis=0)
SCALE = 4.0   # pixels per millimetre


def view(u, v, depth, sign, name, ulabel, vlabel):
    """u to the right, v up, nearest along depth*sign wins."""
    w = int((hi[u] - lo[u]) * SCALE) + 40
    h = int((hi[v] - lo[v]) * SCALE) + 40
    px = ((pts[:, u] - lo[u]) * SCALE + 20).astype(int)
    py = (h - 1 - ((pts[:, v] - lo[v]) * SCALE + 20)).astype(int)
    d = pts[:, depth] * sign
    shade = np.abs(nrm[:, depth])
    order = np.argsort(d)               # far first, near last overwrites
    img = np.full((h, w), 255, dtype=np.uint8)
    img[py[order], px[order]] = (60 + 170 * shade[order]).astype(np.uint8)
    im = Image.fromarray(img).convert("RGB")
    draw = ImageDraw.Draw(im)
    draw.text((6, 4), "%s   right: +%s   up: +%s" % (name, ulabel, vlabel), fill=(200, 0, 0))
    # A marker at the most positive end of the horizontal axis.
    draw.text((w - 60, h // 2), "+%s >" % ulabel, fill=(0, 0, 200))
    out = "%s_%s.png" % (prefix, name)
    im.save(out)
    print("wrote", out)


view(0, 2, 1, +1, "top", "X", "Z")     # from above (+Y), X right, Z up the page
view(0, 1, 2, +1, "side", "X", "Y")    # from +Z, X right, Y up
view(2, 1, 0, -1, "end", "Z", "Y")     # from -X, Z right, Y up
