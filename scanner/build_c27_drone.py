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
                h_fov=1.8500, v_fov=1.5010, h_samples=32, v_samples=8):
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
# 10 Hz at 0.6 m/s cruise is a frame every 6 cm, far more than a box needs.
hires_front = camera_block(
    "camera_hires_link", "camera_hires_joint",
    0.10, 0.0, 0.0, 0, 0, 0,
    fov=1.0472, width=1024, height=768, update_rate=10)

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
    0.08, 0.0, -0.03, 0, 0, 0,
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
tracking_rear = camera_block(
    "camera_track_rear_link", "camera_track_rear_joint",
    -0.08, 0.0, -0.03, 0, 0, 3.14159,
    fov=1.5708, width=1280, height=800, update_rate=3)

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
    0.0, 0.0, -0.05, 0, 1.5708, 0,
    fov=1.5708, width=1280, height=800, update_rate=3)

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
vio_odometry = '''
    <plugin
      filename="gz-sim-odometry-publisher-system"
      name="gz::sim::systems::OdometryPublisher">
      <dimensions>3</dimensions>
    </plugin>'''

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
    0.12, 0.0, 0.0, 0, 0, 0,
    max_range=5.0)

sdf = f'''<?xml version="1.0" encoding="UTF-8"?>
<sdf version='1.9'>
  <model name='{model_name}'>
    <self_collide>false</self_collide>
    <include merge='true'>
      <uri>x500</uri>
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
print("  tof_link                  PMD TOF, 106x86 deg, 5 m, 32x8 rays")
print()
print("  Note: scanning now requires the vehicle to face the shelf, so each")
print("  shelf face needs its own pass. Mission time roughly doubles.")
