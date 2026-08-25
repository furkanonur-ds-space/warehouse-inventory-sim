"""
Drop a test obstacle into the running world.

Spawned through the Gazebo service rather than written into the world file, so
the warehouse generator is untouched and the obstacle disappears with the
simulator. Nothing in Ibrahim's half needs to change to test avoidance.

    python3 spawn_obstacle.py            aisle 1, in face A's lane
    python3 spawn_obstacle.py -8.9 -3.0  a specific spot
"""
import subprocess
import sys

X = float(sys.argv[1]) if len(sys.argv) > 2 else -8.9
Y = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0
W, D, H = 0.8, 1.2, 1.5          # a stacked pallet, roughly

SDF = f"""<?xml version="1.0"?>
<sdf version="1.9">
  <model name="test_obstacle">
    <static>true</static>
    <pose>{X} {Y} {H / 2} 0 0 0</pose>
    <link name="link">
      <collision name="collision">
        <geometry><box><size>{W} {D} {H}</size></box></geometry>
      </collision>
      <visual name="visual">
        <geometry><box><size>{W} {D} {H}</size></box></geometry>
        <material>
          <ambient>0.8 0.2 0.1 1</ambient>
          <diffuse>0.8 0.2 0.1 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>"""

# The request is a single-line protobuf text field, so the SDF has to be one
# line with its quotes escaped.
flat = " ".join(SDF.split()).replace('"', '\\"')
req = 'sdf: "%s"' % flat

result = subprocess.run(
    ["gz", "service", "-s", "/world/warehouse/create",
     "--reqtype", "gz.msgs.EntityFactory",
     "--reptype", "gz.msgs.Boolean",
     "--timeout", "5000",
     "--req", req],
    capture_output=True, text=True)

out = [ln for ln in (result.stdout + result.stderr).splitlines()
       if "libprotobuf" not in ln and ln.strip()]
print("\n".join(out[-4:]) if out else "(no output)")
print("obstacle at x=%.1f y=%.1f, %.1f m tall" % (X, Y, H))
