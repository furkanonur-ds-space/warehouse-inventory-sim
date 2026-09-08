"""
Build the x500_c27 vehicle model, mirroring the C27 sensor configuration.

C27 on the real vehicle carries:

    1 x IMX412   high resolution colour, front facing, used for scanning
    1 x TOF      depth, front facing, used for obstacle detection
    3 x AR0144   mono global shutter tracking cameras: front, rear, down

The tracking cameras are for localization, not for reading labels. Only the
IMX412 has the resolution to decode a 14 cm QR code at aisle range.

This changes the scanning strategy. With side-facing cameras a single pass down
an aisle covered both shelf faces at once. With a single front-facing camera the
vehicle must face the shelf it is scanning, so each shelf face needs its own
pass and the mission takes roughly twice as long.

The base model is x500 rather than x500_flow. x500_flow is broken in this PX4
version: EKF2 reports attitude 0 and never produces a position estimate, so the
vehicle cannot arm. Confirmed by running the stock x500_flow alone and seeing
the same failure. The plain x500 already provides optical flow and a range
sensor.
"""
import os
import sys

# Build a model whose odometry does not go straight to PX4.
#
#   python3 build_c27_drone.py            the vehicle that flies every day
#   python3 build_c27_drone.py --drift    the same vehicle, wired for a test
#
# With --drift the odometry publisher writes to a private topic and PX4 is
# left with nothing on the one it reads, so inject_drift.py has to sit in
# between and is obviously required rather than quietly optional. That is the
# point of doing it this way: the model that flies normally is not carrying a
# test harness it could fail without.
INJECT_DRIFT = "--drift" in sys.argv

# How often the hires camera renders, in Hz. It decides how many looks a code
# gets: on the 0.50 m aisle a code is in view for 0.24 s, which was 2.4 frames
# at the 10 Hz this used to be, and face G read 46 of its 108 codes in exactly
# one frame. At 20 it is 4.9 frames, and G reads exactly one code once.
#
#     10 Hz    432 QR   409 barcode    23 barcode misses, all on G
#     20 Hz    432 QR   431 barcode     1 barcode miss, on E
#
# It was 10 because that was believed to be what the machine could render. It
# is not: measured on an idle simulator the hires delivers 19.2 Hz at a real
# time factor of 0.99. What could not take 20 Hz was the decoder, at 118 ms a
# frame, and that is now 38 because frames are scaled to the detail a code
# needs before decoding.
#
# Raising it costs wall clock and not accuracy. In lockstep, slower rendering
# slows simulated time too, so the vehicle sees the same frames per metre
# either way; the flight just takes longer to sit through. Measured at 20 Hz
# the real time factor is 0.57, so a 580 s flight takes about 17 minutes.
#
# There is less room left than there was. On the 1.77 m aisle the hires
# decoder now runs at 96 per cent of the frame interval and sheds 94 frames a
# flight, because a distant code cannot be scaled down and there are more of
# them in view. A slower machine will shed more. Raise this again only with
# that number in front of you.
HIRES_RATE = 20
for _i, _a in enumerate(sys.argv):
    if _a == "--hires-rate" and _i + 1 < len(sys.argv):
        HIRES_RATE = int(sys.argv[_i + 1])

# How finely the TOF is sampled, horizontally and vertically.
#
# The sensor on the vehicle is a PMD IRS2975C: 240 x 180 points across 106 by
# 86 degrees, which is 0.44 by 0.48 degrees a point. This is a gpu_lidar here
# and every ray costs render time, so it has always been 32 by 8, which is
# 3.31 by 10.75 degrees: 7.5 times coarser across and 22 times coarser up.
#
# In metres at a metre, the simulated sensor steps 58 mm sideways and 188 mm
# vertically from one reading to the next. Anything smaller than that, in the
# gap, is not there as far as the simulation is concerned: a box corner
# protruding into the aisle, a pallet strap, an arm. The real sensor steps 8
# and 8 mm at the same distance.
#
# It matters because clearance is what the safety claim rests on. A run
# reporting a minimum obstacle distance of 0.209 m and no alarms measured that
# with 256 samples of the shelf where the vehicle has 43200.
#
# 240 x 180 at 20 Hz is 864000 rays a second and is not affordable. 96 x 32 is
# 61440 and steps 19 by 47 mm at a metre, smaller than anything the vehicle
# could hit and survive. Measure before adopting, the way the camera rate was:
#
#     python3 build_c27_drone.py --tof 96x32
#     ./scripts/launch_sim.sh nvidia
#     python3 scanner/measure_rate.py
# How often the rear tracking camera renders, in Hz.
#
# It reads face H, and face H is now the only thin face in the building: 37 of
# its codes are read in exactly one frame where every other face reads
# everything five to nine times. That is the same shape as face G's problem
# before the hires went to 20 Hz, and the same cause. The two barcodes missed
# on 2026-09-08 at 16:36 were both on H.
#
# There is room for it. The rear decoder runs at 28 per cent of the frame
# interval on the 0.50 m aisle, where H is, against the hires' 43. It is the
# wide aisles that bind: 54, 60 and 63 per cent, where a distant code cannot be
# scaled down. At 12 Hz those become roughly 81, 90 and 95, which is tight but
# under. At 16 they go over and the decoder starts shedding.
#
# The vehicle runs this camera at 30 fps.
#
# Left at 8 until a flight says otherwise, the same way the hires was.
REAR_RATE = 8
for _i, _a in enumerate(sys.argv):
    if _a == "--rear-rate" and _i + 1 < len(sys.argv):
        REAR_RATE = int(sys.argv[_i + 1])


TOF_H_SAMPLES = 32
TOF_V_SAMPLES = 8
for _i, _a in enumerate(sys.argv):
    if _a == "--tof" and _i + 1 < len(sys.argv):
        TOF_H_SAMPLES, TOF_V_SAMPLES = (
            int(v) for v in sys.argv[_i + 1].lower().split("x"))

GZ_MODELS = os.path.expanduser('~/PX4-Autopilot/Tools/simulation/gz/models')
model_name = "x500_c27"
model_dir = os.path.join(GZ_MODELS, model_name)
os.makedirs(model_dir, exist_ok=True)

config = f'''<?xml version="1.0"?>
<model>
  <name>{model_name}</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
</model>'''
with open(os.path.join(model_dir, 'model.config'), 'w') as f:
    f.write(config)


def camera_block(link_name, joint_name, x_off, y_off, z_off,
                 roll, pitch, yaw, fov, width, height, update_rate=30):
    """
    One fixed-mounted camera.

    The joint is fixed because the cameras do not move relative to the
    airframe. The inertial values are deliberately tiny so the added links do
    not measurably change the flight dynamics.
    """
    return f'''
    <joint name="{joint_name}" type="fixed">
      <parent>base_link</parent>
      <child>{link_name}</child>
      <pose relative_to="base_link">{x_off} {y_off} {z_off} {roll} {pitch} {yaw}</pose>
    </joint>
    <link name="{link_name}">
      <pose relative_to="{joint_name}">0 0 0 0 0 0</pose>
      <inertial>
        <mass>0.01</mass>
        <inertia>
          <ixx>0.00001</ixx><iyy>0.00001</iyy><izz>0.00001</izz>
          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
        </inertia>
      </inertial>
      <sensor name="camera" type="camera">
        <gz_frame_id>{link_name}</gz_frame_id>
        <always_on>1</always_on>
        <update_rate>{update_rate}</update_rate>
        <camera name="camera">
          <horizontal_fov>{fov}</horizontal_fov>
          <image>
            <width>{width}</width>
            <height>{height}</height>
            <format>R8G8B8</format>
          </image>
          <clip><near>0.05</near><far>100</far></clip>
        </camera>
      </sensor>
    </link>'''


def range_block(link_name, joint_name, x_off, y_off, z_off,
                roll, pitch, yaw, max_range=5.0,
                h_fov=1.8500, v_fov=1.5010,
                h_samples=TOF_H_SAMPLES, v_samples=TOF_V_SAMPLES):
    """
    The PMD TOF module, as a ray grid rather than a depth camera.

    Datasheet for the MSU-M0178-1-01 (PMD IRS2975C): 240x180 px, 106 x 86
    degrees, 4 to 6 m range. Rendering a full depth image would cost as much as
    another camera, and nothing here needs per-pixel depth: the question being
    asked is how far away the nearest thing in front is. A 32 x 8 ray grid over
    the same cone answers that and costs almost nothing.

    The width matters more than it looks. Flying an aisle sideways puts the
    direction of travel 90 degrees off the nose, and a 106 degree cone reaches
    within 53 degrees of it. Checking the way ahead is therefore a 50 degree
    turn rather than a 90 degree one, which is most of what makes a periodic
    look affordable.

    A one-beam version modelled this as far blinder than the hardware is, and
    at 10 m it also claimed more than twice the real range.
    """
    return f'''
    <joint name="{joint_name}" type="fixed">
      <parent>base_link</parent>
      <child>{link_name}</child>
      <pose relative_to="base_link">{x_off} {y_off} {z_off} {roll} {pitch} {yaw}</pose>
    </joint>
    <link name="{link_name}">
      <pose relative_to="{joint_name}">0 0 0 0 0 0</pose>
      <inertial>
        <mass>0.005</mass>
        <inertia>
          <ixx>0.000005</ixx><iyy>0.000005</iyy><izz>0.000005</izz>
          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
        </inertia>
      </inertial>
      <sensor name="tof" type="gpu_lidar">
        <gz_frame_id>{link_name}</gz_frame_id>
        <always_on>1</always_on>
        <update_rate>20</update_rate>
        <visualize>false</visualize>
        <ray>
          <scan>
            <horizontal>
              <samples>{h_samples}</samples><resolution>1</resolution>
              <min_angle>{-h_fov / 2:.4f}</min_angle>
              <max_angle>{h_fov / 2:.4f}</max_angle>
            </horizontal>
            <vertical>
              <samples>{v_samples}</samples><resolution>1</resolution>
              <min_angle>{-v_fov / 2:.4f}</min_angle>
              <max_angle>{v_fov / 2:.4f}</max_angle>
            </vertical>
          </scan>
          <range>
            <min>0.1</min><max>{max_range}</max><resolution>0.01</resolution>
          </range>
        </ray>
      </sensor>
    </link>'''


# --- IMX412, front facing, scanning ------------------------------------
#
# Read off the hardware rather than assumed. voxl-camera-server.conf on the
# vehicle configures the IMX412 with three streams: a 640x480 preview, a
# 1024x768 small_video, and a 4056x3040 large_video. voxl-inspect-cam confirms
# all three running at 30 fps, the largest of them at 4.4 Gbps.
#
# 1024x768 is the one a real-time pipeline can consume, so that is what this
# simulates. An earlier 1280x720 matched none of the three, which meant any
# result measured here could not be expected to hold on the aircraft.
# Camera update rates are deliberately low.
#
# Gazebo's memory grows with every rendered frame here, and a full scan takes
# about half an hour. At 30 Hz on all four cameras it reached 26 GB of 27 GB
# and the machine began swapping, which stalled the MAVLink link, stopped the
# offboard setpoint stream and dropped the vehicle out of the air. It happened
# near the end of the route every time, which looked like a flight logic fault
# but is purely elapsed time.
#
# At 20 Hz and the 1 m/s the scan cruises at, that is a frame every 5 cm.
hires_front = camera_block(
    "camera_hires_link", "camera_hires_joint",
    0.06, 0.0, 0.0, 0, 0, 0,
    fov=1.0472, width=1024, height=768, update_rate=HIRES_RATE)

# --- AR0144 tracking cameras -------------------------------------------
#
# 1280x800 on the vehicle, all three of them, confirmed from
# voxl-camera-server.conf. Simulated at the same size now: they were 640x480
# here, which understated them by half in each direction and would have made
# any judgement about what they can read too pessimistic.
#
# They exist to represent the C27 sensor set.
# Nothing consumes their images: the VIO they would feed is simulated by
# OdometryPublisher instead, which reads the model pose directly. They are kept
# for fidelity but rendered as rarely as possible, since every frame costs
# memory that the run cannot spare.
tracking_front = camera_block(
    "camera_track_front_link", "camera_track_front_joint",
    0.055, 0.0, -0.015, 0, 0, 0,
    fov=1.5708, width=1280, height=800, update_rate=1)

# The rear camera is no longer decorative: it reads the shelf behind the
# vehicle while the hires reads the one in front, so a single pass covers
# both faces of an aisle. That makes its frame rate part of the result.
#
# At 1.20 m from the face it resolves 1.49 px per module on axis, and that
# falls as the square of the cosine of the bearing, because the range grows
# and the label foreshortens by the same cosine. A code is readable over
# roughly 0.6 m of travel, which at 0.6 m/s is one second. At 1 Hz that is
# one frame per box and frequently none, so a run would report almost
# nothing whether or not the camera can read, measuring the frame rate
# rather than the camera.
#
# 3 Hz puts three frames inside that window. The hardware runs these at
# 30 fps, confirmed by voxl-inspect-cam, so this moves the simulation
# towards the vehicle rather than away from it. It costs 2.0 Mpx/s, taking
# the budget to 15.1, still under the 20.2 that ran gz out of memory.
# 8 Hz, not 3.
#
# At 1 m/s the narrowest aisle puts this camera 0.229 m from its shelf, where
# it sees 0.46 m of it at a time, so a box is in frame for 0.46 s. Three hertz
# is 1.4 frames on it. The face only this camera reads lost 24 of 54 at that
# rate; four frames needs 8.7 Hz.
#
# The vehicle runs these at 30 fps, so this is still well short of the
# hardware. It costs 5.12 Mpx/s, paid for by the downward camera below.
tracking_rear = camera_block(
    "camera_track_rear_link", "camera_track_rear_joint",
    -0.055, 0.0, -0.015, 0, 0, 3.14159,
    fov=1.5708, width=1280, height=800, update_rate=REAR_RATE)

# The downward tracking camera doubles as the ArUco marker reader for drift
# correction.
#
# 3 Hz, not 10. Gazebo's memory grows with every pixel it renders, and raising
# this camera from 640x480 to its real 1280x800 tripled its share: the render
# budget went from 12.9 to 20.2 Mpx/s and gz reached 27 GB before the mission
# finished, at which point the kernel killed it. The scan lost its output file
# and the decode rate collapsed, both of which looked like unrelated faults.
#
# Corrections are only taken during the 1.5 second settle at the end of a leg,
# so 3 Hz still offers four or five frames of a marker, and the earlier 10 Hz
# was spending most of its frames on stretches where sightings are ignored.
tracking_down = camera_block(
    "camera_track_down_link", "camera_track_down_joint",
    0.0, 0.0, -0.025, 0, 1.5708, 0,
    fov=1.5708, width=1280, height=800, update_rate=5)

# --- Sensors required for GPS-free position estimation -----------------
#
# The base x500 provides a barometer, magnetometer, IMU, GPS and a downward
# camera. It does NOT provide optical flow or a downward range sensor.
#
# Without those two, disabling GPS leaves EKF2 with no way to estimate
# horizontal position: xy_valid stays false and the vehicle cannot hold
# position. This was not obvious for several days because the airframe still
# had GPS enabled, so the estimate came from GPS while the project described
# itself as GPS-free.
#
# The definitions below mirror those in the stock x500_flow model. That model
# is not used as a base because EKF2 fails to initialise with it in this PX4
# version; copying the two sensors avoids that problem.

# Optical flow, defined inline rather than by including the stock model.
#
# The stock model is brought in with:
#
#     <include merge='true'><uri>model://optical_flow</uri></include>
#
# and that breaks EKF2 in this PX4 version: the estimator reports attitude 0
# and zero updates, so the vehicle never gets an attitude let alone a position.
# Bisection confirmed it: with the include present EKF2 recorded 0 updates,
# without it 1589 in the same interval. It is also why the stock x500_flow
# model fails, since x500_flow uses the same include.
#
# Defining the sensor directly avoids whatever the merge does to the link
# structure, while producing the same measurements.
# Optical flow, defined inline rather than by including the stock model.
#
# The stock model is normally brought in with:
#
#     <include merge='true'><uri>model://optical_flow</uri></include>
#
# and that breaks EKF2 in this PX4 version: the estimator reports attitude 0
# and records zero updates, so the vehicle never gets an attitude, let alone a
# position. Bisection confirmed it: with the include present EKF2 logged 0
# updates, without it 1589 over the same interval. The same include is why the
# stock x500_flow model fails.
#
# The structure below mirrors the stock model exactly, because the flow plugin
# depends on it. Two sensors sit on the same link:
#
#   flow_camera   an ordinary downward camera that produces the image
#   optical_flow  the plugin, which finds that camera by looking on its own
#                 link and computes motion from consecutive frames
#
# A first attempt merged the two into one sensor with an inline camera block.
# Gazebo created the topics but PX4 never received flow data, because the
# plugin had no camera to read from.
# VIO simulation via OdometryPublisher.
# This simulates the VOXL 2 computing VIO from the tracking cameras.
#
# Note what this does NOT simulate: the plugin reports the model's true pose,
# so the simulated VIO is exact and never drifts. Real VIO does. That makes the
# ArUco drift correction untestable by default, because there is nothing for it
# to correct, and any claim that the correction "works" is unfalsifiable.
#
# Neither knob this plugin offers can simulate that drift, so do not reach for
# them:
#
#   xyz_offset      is a mounting offset, not a bias. It rotates with the body,
#                   so a 0.5 m value gives the odometry a 0.5 m lever arm: yaw
#                   becomes apparent translation, EKF2 reads it as violent
#                   motion, and the vehicle tumbles before it can take off.
#                   Tried, and it does exactly that.
#   gaussian_noise  is zero-mean. EKF2 averages it out, so it adds per-sample
#                   noise but no accumulating error, which is what the ArUco
#                   correction exists to cancel.
#
# The correction geometry is therefore verified offline instead, by
# test_drift_correction.py, which drives it with synthetic frames and a known
# injected error. That test found a real defect the simulator could never have
# surfaced: the correction summed its measurement instead of converging on it.
#
# What --drift changes. PX4's gz bridge subscribes to exactly
# /model/<model>/odometry_with_covariance, built from the model name in
# GZBridge.cpp, so the way to put something in front of PX4 is to move the
# plugin off that topic. It then publishes the true pose on a private name and
# inject_drift.py reads it, adds an accumulating error and publishes the
# result on the name PX4 is waiting for.
#
# That puts the error where a real one is. Corrupting what the scanner reads
# instead does displace the vehicle, through the lateral check in the settle
# loop, but only as far as that check's own tolerance and timeout allow, and
# it leaves PX4's estimator being fed the truth. On the vehicle it is the
# estimator that is wrong, and every setpoint lands displaced because of it.
DRIFT_TOPIC = "/model/%s/odometry_with_covariance_true" % model_name

vio_odometry = '''
    <plugin
      filename="gz-sim-odometry-publisher-system"
      name="gz::sim::systems::OdometryPublisher">
      <dimensions>3</dimensions>%s
    </plugin>''' % (
    "\n      <odom_covariance_topic>%s</odom_covariance_topic>" % DRIFT_TOPIC
    if INJECT_DRIFT else "")

# --- PMD TOF, front facing, obstacle distance --------------------------
#
# The link is called tof_link and not lidar_sensor_link. That name is not
# cosmetic: PX4's gz bridge subscribes to exactly two hardcoded lidar topics,
# .../link/link/sensor/lidar_2d_v2/scan and
# .../link/lidar_sensor_link/sensor/lidar/scan, and publishes whatever arrives
# as distance_sensor. A forward-facing beam on that link would reach EKF2 as a
# height above ground, which is what the earlier "collides with the range
# sensor" note was about. Under any other name PX4 ignores it and the reading
# is ours alone, read straight from Gazebo.
tof_front = range_block(
    "tof_link", "tof_joint",
    0.06, 0.0, 0.0, 0, 0, 0,
    max_range=5.0)

sdf = f'''<?xml version="1.0" encoding="UTF-8"?>
<sdf version='1.9'>
  <model name='{model_name}'>
    <self_collide>false</self_collide>
    <include merge='true'>
      <uri>starling2</uri>
    </include>
{hires_front}
{tracking_front}
{tracking_rear}
{tracking_down}
{vio_odometry}
{tof_front}
  </model>
</sdf>'''
with open(os.path.join(model_dir, 'model.sdf'), 'w') as f:
    f.write(sdf)

print(f"{model_name} model generated, C27 sensor configuration")
print("  camera_hires_link        1024x768  front, 60 deg   scanning")
print("  camera_track_front_link  1280x800  front, 90 deg   odometry")
print("  camera_track_rear_link   1280x800  rear,  90 deg   odometry")
print("  camera_track_down_link   1280x800  down,  90 deg   odometry and ArUco")
print("  OdometryPublisher         plugin   VIO simulation")
if INJECT_DRIFT:
    print()
    print("  WIRED FOR A DRIFT TEST. The odometry goes to")
    print("    %s" % DRIFT_TOPIC)
    print("  and not to PX4. Nothing will fly until inject_drift.py is")
    print("  relaying it, because PX4 has no position source without it.")
    print("  Rebuild without --drift to get the vehicle back.")
print("  tof_link                  PMD TOF, 106x86 deg, 5 m, 32x8 rays")
print()
print("  Note: scanning now requires the vehicle to face the shelf, so each")
print("  shelf face needs its own pass. Mission time roughly doubles.")
