"""
Build the airframe at the size of the vehicle we actually fly.

The simulation had been standing on PX4's x500: 500 mm between motors and
2.0 kg. The vehicle is a ModalAI Starling 2, 230 mm between motors and 285 g
with its battery. That is not a detail. Aisle width, obstacle clearance and
where the cameras sit on the body all come from the size of the airframe, and
every one of those was being validated against a machine more than twice as
wide and seven times as heavy as the real one.

  x500          motors at 174 mm from centre, 2.0 kg, 254 mm propellers
  Starling 2    motors at  81 mm from centre, 285 g, 120 mm propellers

With 120 mm propellers the widest part of the vehicle is about 282 mm across,
against roughly 500 mm for the x500.

Written from simple shapes rather than by rescaling the x500 meshes. Nothing
looks at the vehicle, so its appearance buys nothing, and scaling a file full
of mesh transforms, collision boxes and landing gear poses has more ways to go
quietly wrong than to go right.

The sensors PX4 needs, the barometer, magnetometer, IMU and the navsat it
never reads, are lifted from x500_base unchanged, so their noise models stay
exactly what PX4 was tuned against.

Thrust to weight is deliberately held at the x500's 1.74 rather than the
higher figure the real vehicle has. The point of this change is size, and
keeping the ratio means PX4's hover throttle and the thrust related gains stay
valid instead of everything moving at once.

One thing this drops: x500_base carries a 640x480 downward camera at 30 Hz,
which nothing in this project reads. It was costing 9.2 Mpx/s, so the render
budget was 24.2 rather than the 15.0 it was thought to be, against an out of
memory kill measured at 20.2. That camera has been quietly eating the margin
the whole time.
"""
import math
import os
import re
import shutil
import sys

PX4_MODELS = os.path.expanduser(
    "~/PX4-Autopilot/Tools/simulation/gz/models")

# --- THE VEHICLE, FROM MODALAI'S OWN PARAMETERS ---------------------------
# params/v1.18/D0014/D0014_Starling_2_base.params, which is the Starling 2.
# The datasheet gives a diagonal and a mass; these are the numbers the flight
# controller is actually configured with.
PROP_DIAMETER_M = 0.120
MASS_KG = 0.285             # take-off weight, 182 g of it without the battery
ROTOR_MASS_KG = 0.008       # motor plus propeller, one arm

# CA_ROTOR0_PX and PY. The motors are not on a square: the frame is 170 mm
# front to back and 125 mm across, so roll and pitch do not behave alike and
# PX4's gains for them differ, 0.072 against 0.097.
ARM_X = 0.085
ARM_Y = 0.0625
DIAGONAL_M = 2 * math.sqrt(ARM_X ** 2 + ARM_Y ** 2)      # 0.211 m

# The body, inside the motors.
BODY_X, BODY_Y, BODY_Z = 0.110, 0.085, 0.045
# How far base_link sits above the floor when landed. Everything the scanner
# commands is measured from the spawn point and the world is not, so this
# number has to match reality; measure it in the simulator rather than trust
# it, as ground_offset in layout.json has been wrong before.
REST_HEIGHT_M = 0.060

# --- WHAT IT LOOKS LIKE ---------------------------------------------------
# By default the vehicle is drawn as the boxes and discs its physics is made
# of. Given --body-mesh, it is drawn from ModalAI's own CAD instead, D0014
# Starling 2 C27 (STEP, converted by ~/starling2_CAD/step_to_stl.py). The mesh
# is not in this repository: it is derived from ModalAI's file and this
# repository is public, so it is built locally from the STEP.
#
# Only the look changes. Mass, inertia, collisions and the rotor joints stay
# exactly as above, from ModalAI's PX4 parameters, because that is what the
# vehicle is flown with and what every scan result was measured against.
#
# Measured in the STEP, millimetres, Y up:
#   the camera clusters sit at the two ends of Z, and ModalAI's extrinsics
#   (D0014_Starling_2.conf) put the front cameras 66 to 68 mm ahead of the
#   body and the rear one 62 mm behind, so -Z is forward and X is across;
#   the motor mounts are at X +-88, Z +-66: 66 mm fore and aft, 88 across;
#   the feet are 29.7 mm below the file's origin.
#
# That motor layout disagrees with the parameters above, which put the
# motors 85 mm fore and aft and 62.5 across, the other way round. Both come
# from ModalAI. The parameters are what PX4 flies, so the physics keeps them;
# the drawn propellers follow the CAD, so what is seen is the real airframe.
# Raised with ModalAI and the company as a question, not settled here.
CAD_MOTOR_FWD = 0.066
CAD_MOTOR_LAT = 0.088
CAD_FEET_BELOW_ORIGIN = 0.0297
CAD_PROP_PLANE = 0.018          # propeller plane above the file's origin
# Feet on the ground: base_link rests REST_HEIGHT_M up, the CAD's feet are
# CAD_FEET_BELOW_ORIGIN below its origin. ModalAI's extrinsics put the real
# body 33 mm above the ground, against REST_HEIGHT_M here; left alone for now
# because it moves every commanded altitude.
MESH_Z = -(REST_HEIGHT_M - CAD_FEET_BELOW_ORIGIN)
# Y up and -Z forward in the file, to x forward, y left, z up here: roll 90
# degrees about x, then yaw -90 about z.
MESH_RPY = "1.570796 0 -1.570796"
MESH_FILE = "starling2_c27_body.stl"

# The bit every vehicle visual carries, and the one the vehicle's own
# sensors are told not to see (build_c27_drone.SENSOR_VISIBILITY_MASK). The
# CAD body wraps the cameras as the real one does, and every scan so far was
# measured by sensors that looked past the vehicle.
VEHICLE_VISUAL_FLAG = 0x2

BODY_MESH = None                # set from --body-mesh in main()

# --- THE MOTORS -----------------------------------------------------------
# 1504 3000 kv on 2S. Under load the propellers fall well short of the no load
# speed of 3000 x 7.4 volts.
MAX_ROT_VELOCITY = 1500.0
MIN_ROT_VELOCITY = 150.0        # SIM_GZ_EC_MIN, inherited from the x500

# The motor constant follows from the hover throttle rather than from a chosen
# thrust ratio. PX4 maps its normalised demand onto rotor speed between MIN and
# MAX, and thrust goes as the square of that, so MPC_THR_HOVER fixes it.
HOVER_THROTTLE = 0.34           # MPC_THR_HOVER, from D0014
HOVER_ROT_VELOCITY = MIN_ROT_VELOCITY + HOVER_THROTTLE * (
    MAX_ROT_VELOCITY - MIN_ROT_VELOCITY)
HOVER_THRUST_PER_ROTOR = MASS_KG * 9.81 / 4
MOTOR_CONSTANT = HOVER_THRUST_PER_ROTOR / HOVER_ROT_VELOCITY ** 2
MAX_THRUST_PER_ROTOR = MOTOR_CONSTANT * MAX_ROT_VELOCITY ** 2

# Torque per unit thrust. PX4 is told CA_ROTOR_KM 0.05, the same as the x500,
# so the model keeps the same relationship the x500 has between the two, which
# is what flies.
MOMENT_CONSTANT = 0.016
# Small corrections, scaled by mass for want of anything better.
ROTOR_DRAG = 8.06428e-05 * MASS_KG / 2.0
ROLLING_MOMENT = 1e-06 * MASS_KG / 2.0

MODEL_NAME = "starling2"
BASE_NAME = "starling2_base"


def box_inertia(mass, sx, sy, sz):
    return (mass * (sy ** 2 + sz ** 2) / 12,
            mass * (sx ** 2 + sz ** 2) / 12,
            mass * (sx ** 2 + sy ** 2) / 12)


def borrow_sensors():
    """
    The barometer, magnetometer, IMU and navsat blocks from x500_base.

    Taken verbatim. Their noise densities and bias walks are what PX4's
    estimator was tuned against, and rewriting them by hand would change the
    estimator's behaviour while claiming to change only the airframe.

    The downward camera that follows them in the same file is deliberately
    left behind.
    """
    path = os.path.join(PX4_MODELS, "x500_base", "model.sdf")
    with open(path, encoding="utf-8") as handle:
        text = handle.read()

    # From the barometer to the end of the navsat sensor, which is the last
    # one PX4 needs. Bounded by the tags themselves rather than by a comment:
    # an earlier version looked for "<!-- DOWNWARD CAMERA -->" to find the end,
    # and removing that camera, which the setup script now does, took the
    # landmark with it and broke the build. A comment in a file this project
    # does not own is not something to navigate by.
    start = text.find('<sensor name="air_pressure_sensor"')
    last = text.find('<sensor name="navsat_sensor"')
    if start == -1 or last == -1:
        raise SystemExit("x500_base does not carry the sensors PX4 needs")
    end = text.find("</sensor>", last)
    if end == -1:
        raise SystemExit("x500_base navsat sensor is not closed")
    end += len("</sensor>")

    block = text[start:end].rstrip()
    for needed in ("air_pressure_sensor", "magnetometer_sensor",
                   "imu_sensor", "navsat_sensor"):
        if needed not in block:
            raise SystemExit("x500_base is missing %s" % needed)
    return "\n".join("      " + line.lstrip() if line.strip() else ""
                     for line in block.splitlines())


def rotor_visual(index):
    """The propeller as drawn on its own spinning link, or nothing.

    With the CAD body the drawn propellers move to the CAD's motors, which
    are not where the rotor joints are (see CAD_MOTOR_FWD), so they are drawn
    on base_link instead by cad_visuals(). Drawn on the rotor link they would
    orbit the joint axis rather than spin about their own centre. A flat disc
    looks the same spinning or still, so nothing is lost.
    """
    if BODY_MESH:
        return ""
    return f'''      <visual name="rotor_{index}_visual">
        <geometry>
          <cylinder>
            <radius>{PROP_DIAMETER_M / 2:.4f}</radius>
            <length>0.004</length>
          </cylinder>
        </geometry>
        <material>
          <ambient>0.2 0.2 0.2 1</ambient>
          <diffuse>0.2 0.2 0.2 1</diffuse>
        </material>
      </visual>'''


def cad_visuals():
    """The ModalAI body and four propeller discs, all on base_link."""
    discs = "".join(f'''
      <visual name="prop_{i}_visual">
        <pose>{xs * CAD_MOTOR_FWD:.4f} {ys * CAD_MOTOR_LAT:.4f} {MESH_Z + CAD_PROP_PLANE:.4f} 0 0 0</pose>
        <geometry>
          <cylinder><radius>{PROP_DIAMETER_M / 2:.4f}</radius><length>0.002</length></cylinder>
        </geometry>
        <material>
          <ambient>0.3 0.3 0.3 1</ambient>
          <diffuse>0.3 0.3 0.3 1</diffuse>
        </material>
        <transparency>0.7</transparency>
        <visibility_flags>{VEHICLE_VISUAL_FLAG}</visibility_flags>
      </visual>''' for i, (xs, ys) in enumerate(((+1, -1), (-1, +1), (+1, +1), (-1, -1))))
    return f'''      <visual name="body_cad_visual">
        <pose>0 0 {MESH_Z:.4f} {MESH_RPY}</pose>
        <geometry>
          <mesh>
            <uri>model://{BASE_NAME}/meshes/{MESH_FILE}</uri>
            <scale>0.001 0.001 0.001</scale>
          </mesh>
        </geometry>
        <material>
          <ambient>0.12 0.12 0.13 1</ambient>
          <diffuse>0.35 0.35 0.37 1</diffuse>
          <specular>0.2 0.2 0.2 1</specular>
        </material>
        <visibility_flags>{VEHICLE_VISUAL_FLAG}</visibility_flags>
      </visual>{discs}'''


def box_visuals():
    """The body and arms as the boxes the physics is made of."""
    return f'''      <visual name="base_link_visual">
        <geometry>
          <box><size>{BODY_X} {BODY_Y} {BODY_Z}</size></box>
        </geometry>
        <material>
          <ambient>0.1 0.1 0.1 1</ambient>
          <diffuse>0.15 0.15 0.15 1</diffuse>
        </material>
      </visual>'''


def arms_visual():
    if BODY_MESH:
        return ""
    return f'''
      <visual name="arms_visual">
        <geometry>
          <box><size>{2 * ARM_X:.4f} {2 * ARM_Y:.4f} 0.006</size></box>
        </geometry>
        <material>
          <ambient>0.05 0.05 0.05 1</ambient>
          <diffuse>0.08 0.08 0.08 1</diffuse>
        </material>
      </visual>'''


def rotor_link(index, x_sign, y_sign, direction):
    """One arm: a motor and propeller, as a thin disc on a hinge."""
    mass = ROTOR_MASS_KG
    # A propeller is much closer to a rod than a disc, so about its own axis
    # it has almost no inertia and about the others it has a rod's.
    i_axis = mass * PROP_DIAMETER_M ** 2 / 12
    return f'''
    <link name="rotor_{index}">
      <gravity>true</gravity>
      <self_collide>false</self_collide>
      <pose>{x_sign * ARM_X:.4f} {y_sign * ARM_Y:.4f} {BODY_Z / 2:.4f} 0 0 0</pose>
      <inertial>
        <mass>{mass}</mass>
        <inertia>
          <ixx>{i_axis:.6e}</ixx>
          <ixy>0</ixy><ixz>0</ixz>
          <iyy>{i_axis:.6e}</iyy>
          <iyz>0</iyz>
          <izz>{2 * i_axis:.6e}</izz>
        </inertia>
      </inertial>
{rotor_visual(index)}
      <collision name="rotor_{index}_collision">
        <geometry>
          <cylinder>
            <radius>{PROP_DIAMETER_M / 2:.4f}</radius>
            <length>0.004</length>
          </cylinder>
        </geometry>
      </collision>
    </link>
    <joint name="rotor_{index}_joint" type="revolute">
      <parent>base_link</parent>
      <child>rotor_{index}</child>
      <axis>
        <xyz>0 0 1</xyz>
        <limit><lower>-1e+16</lower><upper>1e+16</upper></limit>
        <dynamics>
          <spring_reference>0</spring_reference>
          <spring_stiffness>0</spring_stiffness>
        </dynamics>
      </axis>
    </joint>'''


def leg(index, x_sign, y_sign):
    """A short leg, so the vehicle rests at a height we chose rather than one
    that falls out of whatever collides first."""
    height = REST_HEIGHT_M - BODY_Z / 2
    collision = f'''
      <collision name="leg_{index}_collision">
        <pose>{x_sign * 0.045:.4f} {y_sign * 0.045:.4f} {-BODY_Z / 2 - height / 2:.4f} 0 0 0</pose>
        <geometry>
          <box><size>0.008 0.008 {height:.4f}</size></box>
        </geometry>
      </collision>'''
    # The CAD body draws its own legs; the collision stays, since it is what
    # the vehicle stands on.
    if BODY_MESH:
        return collision
    return collision + f'''
      <visual name="leg_{index}_visual">
        <pose>{x_sign * 0.045:.4f} {y_sign * 0.045:.4f} {-BODY_Z / 2 - height / 2:.4f} 0 0 0</pose>
        <geometry>
          <box><size>0.008 0.008 {height:.4f}</size></box>
        </geometry>
      </visual>'''


def motor_plugin(index, direction):
    return f'''
    <plugin filename="gz-sim-multicopter-motor-model-system"
            name="gz::sim::systems::MulticopterMotorModel">
      <jointName>rotor_{index}_joint</jointName>
      <linkName>rotor_{index}</linkName>
      <turningDirection>{direction}</turningDirection>
      <timeConstantUp>0.005</timeConstantUp>
      <timeConstantDown>0.010</timeConstantDown>
      <maxRotVelocity>{MAX_ROT_VELOCITY}</maxRotVelocity>
      <motorConstant>{MOTOR_CONSTANT:.6e}</motorConstant>
      <momentConstant>{MOMENT_CONSTANT:.6f}</momentConstant>
      <commandSubTopic>command/motor_speed</commandSubTopic>
      <motorNumber>{index}</motorNumber>
      <rotorDragCoefficient>{ROTOR_DRAG:.6e}</rotorDragCoefficient>
      <rollingMomentCoefficient>{ROLLING_MOMENT:.6e}</rollingMomentCoefficient>
      <rotorVelocitySlowdownSim>10</rotorVelocitySlowdownSim>
      <motorType>velocity</motorType>
    </plugin>'''


def main():
    global BODY_MESH
    if "--body-mesh" in sys.argv:
        at = sys.argv.index("--body-mesh")
        if at + 1 >= len(sys.argv):
            raise SystemExit("--body-mesh needs the path of the STL")
        BODY_MESH = os.path.expanduser(sys.argv[at + 1])
        if not os.path.isfile(BODY_MESH):
            raise SystemExit("no mesh at %s; build it from the STEP with "
                             "~/starling2_CAD/step_to_stl.py" % BODY_MESH)

    body_mass = MASS_KG - 4 * ROTOR_MASS_KG
    ixx, iyy, izz = box_inertia(body_mass, BODY_X, BODY_Y, BODY_Z)

    # PX4 numbers the rotors so that 0 and 1 turn one way and 2 and 3 the
    # other, with 0 at the front right. Copied from x500 so the mixer that
    # PX4 already has still applies.
    arms = [(0, +1, -1, "ccw"), (1, -1, +1, "ccw"),
            (2, +1, +1, "cw"), (3, -1, -1, "cw")]

    base = f'''<?xml version="1.0" encoding="UTF-8"?>
<sdf version='1.9'>
  <model name='{BASE_NAME}'>
    <pose>0 0 {REST_HEIGHT_M} 0 0 0</pose>
    <static>false</static>
    <link name="base_link">
      <inertial>
        <mass>{body_mass:.4f}</mass>
        <inertia>
          <ixx>{ixx:.6e}</ixx>
          <ixy>0</ixy><ixz>0</ixz>
          <iyy>{iyy:.6e}</iyy>
          <iyz>0</iyz>
          <izz>{izz:.6e}</izz>
        </inertia>
      </inertial>
      <gravity>true</gravity>
      <velocity_decay />
{cad_visuals() if BODY_MESH else box_visuals()}
      <collision name="base_link_collision">
        <geometry>
          <box><size>{BODY_X} {BODY_Y} {BODY_Z}</size></box>
        </geometry>
      </collision>{arms_visual()}
{"".join(leg(i, xs, ys) for i, xs, ys in [(0, +1, +1), (1, +1, -1), (2, -1, +1), (3, -1, -1)])}

{borrow_sensors()}
    </link>
{"".join(rotor_link(i, xs, ys, d) for i, xs, ys, d in arms)}
  </model>
</sdf>
'''

    model = f'''<?xml version="1.0" encoding="UTF-8"?>
<sdf version='1.9'>
  <model name='{MODEL_NAME}'>
    <include merge='true'>
      <uri>model://{BASE_NAME}</uri>
    </include>
{"".join(motor_plugin(i, d) for i, _, _, d in arms)}
  </model>
</sdf>
'''

    # The mesh travels with the model rather than being referred to where it
    # was built, so the model directory is complete on its own. A run without
    # --body-mesh removes it, so a stale copy cannot outlive the choice.
    meshes = os.path.join(PX4_MODELS, BASE_NAME, "meshes")
    if BODY_MESH:
        os.makedirs(meshes, exist_ok=True)
        shutil.copyfile(BODY_MESH, os.path.join(meshes, MESH_FILE))
    elif os.path.isdir(meshes):
        shutil.rmtree(meshes)

    for name, text in ((BASE_NAME, base), (MODEL_NAME, model)):
        directory = os.path.join(PX4_MODELS, name)
        os.makedirs(directory, exist_ok=True)
        with open(os.path.join(directory, "model.sdf"), "w",
                  encoding="utf-8") as handle:
            handle.write(text)
        with open(os.path.join(directory, "model.config"), "w",
                  encoding="utf-8") as handle:
            handle.write(f'''<?xml version="1.0"?>
<model>
  <name>{name}</name>
  <version>1.0</version>
  <sdf version='1.9'>model.sdf</sdf>
  <description>ModalAI Starling 2, generated by build_starling2.py</description>
</model>
''')

    span_x = 2 * ARM_X + PROP_DIAMETER_M
    span_y = 2 * ARM_Y + PROP_DIAMETER_M
    print(f"{MODEL_NAME} generated, ModalAI Starling 2")
    print(f"  motors            {ARM_X * 1000:.0f} mm fore and aft, "
          f"{ARM_Y * 1000:.0f} mm across, {DIAGONAL_M * 1000:.0f} mm diagonal")
    print(f"  widest extent     {span_x * 1000:.0f} by {span_y * 1000:.0f} mm, "
          f"propeller tip to tip")
    print(f"  mass              {MASS_KG * 1000:.0f} g "
          f"({body_mass * 1000:.0f} g body, {ROTOR_MASS_KG * 1000:.0f} g per arm)")
    print(f"  inertia           ixx {ixx:.3e}  iyy {iyy:.3e}  izz {izz:.3e}")
    print(f"  hover             {HOVER_THROTTLE:.2f} throttle, "
          f"{HOVER_ROT_VELOCITY:.0f} rad/s, {HOVER_THRUST_PER_ROTOR:.3f} N per rotor")
    print(f"  max thrust        {MAX_THRUST_PER_ROTOR:.2f} N per rotor, "
          f"{4 * MAX_THRUST_PER_ROTOR / (MASS_KG * 9.81):.1f} thrust to weight "
          f"({4 * MOTOR_CONSTANT * (MIN_ROT_VELOCITY + 0.60 * (MAX_ROT_VELOCITY - MIN_ROT_VELOCITY)) ** 2 / (MASS_KG * 9.81):.1f} "
          f"at the 0.60 throttle cap)")
    print(f"  rests at          {REST_HEIGHT_M * 1000:.0f} mm, "
          f"measure this before trusting ground_offset")
    print("  no downward camera; x500_base carries one at 9.2 Mpx/s")


if __name__ == "__main__":
    main()
