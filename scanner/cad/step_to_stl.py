"""Convert the Starling 2 STEP assembly into a light visual mesh for Gazebo.

    ~/cad_venv/bin/python step_to_stl.py <in.STEP> <out.stl> [--min-volume MM3]

The mesh is for looking at, not for physics: the simulated vehicle keeps its
own mass, inertia and collision shapes. So detail is spent only where it can
be seen. Parts smaller than --min-volume (screws, washers, tiny passives) are
left out, and the surface mesh is coarse.

Small parts are hidden rather than removed. Removing them through the CAD
kernel rebuilt the assembly on every deletion and had not finished after
thirty-five minutes; hiding them and meshing only what is visible costs
nothing extra.

Coordinates stay in the file's millimetres and its own axes. The SDF scales
and rotates the mesh; doing it here as well would make two places that have
to agree.
"""
import argparse
import struct
import time

import gmsh

parser = argparse.ArgumentParser()
parser.add_argument("step")
parser.add_argument("stl")
parser.add_argument("--min-volume", type=float, default=20.0)
parser.add_argument("--size-max", type=float, default=8.0)
args = parser.parse_args()

t0 = time.time()


def log(message):
    print("[%5.0f s] %s" % (time.time() - t0, message), flush=True)


gmsh.initialize()
gmsh.option.setNumber("General.Terminal", 0)
gmsh.model.occ.importShapes(args.step)
gmsh.model.occ.synchronize()
log("opened")

vols = gmsh.model.getEntities(3)
keep, hide = [], []
for dim, tag in vols:
    (keep if gmsh.model.occ.getMass(dim, tag) >= args.min_volume else hide).append((dim, tag))
log("parts: %d kept, %d hidden below %.0f mm3" % (len(keep), len(hide), args.min_volume))

# The surfaces that will be meshed and written: the boundary of every kept
# part. Collected before hiding anything, since visibility is per entity.
surfaces = set()
for dim, tag in keep:
    for _, s in gmsh.model.getBoundary([(dim, tag)], oriented=False, recursive=False):
        surfaces.add(abs(s))
all_surfaces = [t for _, t in gmsh.model.getEntities(2)]
hidden_surfaces = [(2, t) for t in all_surfaces if t not in surfaces]
gmsh.model.setVisibility(hidden_surfaces, 0)
gmsh.option.setNumber("Mesh.MeshOnlyVisible", 1)
log("surfaces: %d to mesh, %d hidden" % (len(surfaces), len(hidden_surfaces)))

gmsh.option.setNumber("Mesh.MeshSizeMax", args.size_max)
gmsh.option.setNumber("Mesh.MeshSizeMin", 1.0)
gmsh.option.setNumber("Mesh.MeshSizeFromCurvature", 6)
# MeshAdapt, single threaded. Frontal-Delaunay on eight threads segfaulted
# on this assembly part way through the surfaces; MeshAdapt is the
# slowest of gmsh's 2D algorithms and the most forgiving of awkward CAD.
gmsh.option.setNumber("Mesh.Algorithm", 1)
gmsh.option.setNumber("General.NumThreads", 1)
gmsh.option.setNumber("Mesh.MaxNumThreads2D", 1)
# gmsh's own progress lines, so a long mesh can be followed and a crash
# names the surface it happened on.
gmsh.option.setNumber("General.Terminal", 1)
gmsh.option.setNumber("General.Verbosity", 4)
log("meshing")
gmsh.model.mesh.generate(1)
log("edges meshed")
gmsh.model.mesh.generate(2)
log("surfaces meshed")

node_tags, coords, _ = gmsh.model.mesh.getNodes()
index = {int(t): i for i, t in enumerate(node_tags)}
triangles = []
for s in surfaces:
    types, _, elem_nodes = gmsh.model.mesh.getElements(2, s)
    for etype, nodes in zip(types, elem_nodes):
        if etype != 2:          # 3-node triangle
            continue
        for k in range(0, len(nodes), 3):
            triangles.append((index[int(nodes[k])], index[int(nodes[k + 1])],
                              index[int(nodes[k + 2])]))
log("triangles collected: %d" % len(triangles))


def point(i):
    return coords[3 * i], coords[3 * i + 1], coords[3 * i + 2]


with open(args.stl, "wb") as out:
    out.write(b"Starling 2 C27, visual mesh from ModalAI STEP".ljust(80, b" "))
    out.write(struct.pack("<I", len(triangles)))
    for a, b, c in triangles:
        pa, pb, pc = point(a), point(b), point(c)
        ux, uy, uz = pb[0] - pa[0], pb[1] - pa[1], pb[2] - pa[2]
        vx, vy, vz = pc[0] - pa[0], pc[1] - pa[1], pc[2] - pa[2]
        n = (uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx)
        out.write(struct.pack("<12fH", *n, *pa, *pb, *pc, 0))
log("written %s" % args.stl)
gmsh.finalize()
