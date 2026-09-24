# hitl_bridge

A Gazebo Harmonic system plugin that connects a simulated vehicle to PX4
running on a real VOXL2 board, so that the flight stack can be flown on the
bench with no propellers and no warehouse.

ModalAI documents this bridge at https://docs.modalai.com/m0197-fpv-hil/ and
calls it `mavlink_hitl_gazebosim`, inside a folder named
`px4-gz-jetty-plugins`. The document gives the messages, the rates, the ports
and the frame conventions, but the source is not published anywhere we could
find: not in modalai/px4-firmware (631 branches searched), not on the public
GitLab, not through GitHub code search. Everything else the document needs is
public and already works here, so this repository fills the one gap.

It targets Gazebo Harmonic (gz-sim8) rather than Jetty, which is what this
machine runs. The document's own compatibility table says that is a matter of
the package names in CMakeLists.txt.

## State

Stage 3 of 4. The loop is closed: sensors leave, motor commands come back and
turn the rotors. It has not yet met a real VOXL2, and the flight controller
driving it so far has been a stand-in.

| stage | what it does | state |
|---|---|---|
| 1 | read the simulated sensors, report rates and values | **done** |
| 2 | pack HIL_SENSOR and HIL_GPS, send over UDP 14560 | **done** |
| 3 | receive HIL_ACTUATOR_CONTROLS, drive the rotors | **done** |
| 4 | send ODOMETRY on UDP 14570 for the VIO path | next |

## Testing without a drone

`scripts/fake_px4.py` stands in for the flight controller at the socket:
it listens for HIL_SENSOR and answers with HIL_ACTUATOR_CONTROLS at 200 Hz,
holding the motors at a fixed value. It does not fly the vehicle, which is
the point: it isolates the bridge from the flight stack.

    # terminal 1
    python3 scripts/fake_px4.py --throttle 0.9 --seconds 4
    # terminal 2
    ./scripts/run_sim.sh 1200

`scripts/check_stream.py` is the other half: it decodes the outgoing stream
with pymavlink and checks the values, not only the rates.

## Build and run

Everything is local. No sudo, nothing installed system wide.

    ./scripts/build.sh
    ./scripts/run_sim.sh 1250      # 1250 steps, five seconds of sim time
    ./scripts/run_sim.sh           # until interrupted

## Measured on 2026-09-24, stage 3

Four seconds each, the same world, only the commanded throttle differing:

    throttle 0.0    actuators 189 Hz   height 0.250 m to 0.227 m   (settles)
    throttle 0.9    actuators 189 Hz   height 0.250 m to 40.02 m   (climbs)

The height is read from Gazebo's own pose stream rather than from anything
the bridge reports, so it is an independent witness that the commands
reached the rotors.

## Measured on 2026-09-24, stage 1

Five seconds, headless, on this machine. The rates are the ones the document
specifies, which is what stage 2 depends on:

    imu 250 Hz   mag 50 Hz   baro 10 Hz   gps 30 Hz
    imu accel m/s^2: 0, 0, 9.8066
    baro Pa: 101322
    gps lat/lon/alt: 39.925, 32.837, 890.227

The IMU reads +9.8066 on z while at rest. That is Gazebo's ENU/FLU frame, and
PX4 expects NED/FRD, so stage 2 has to rotate it. The document names this as
the boundary where the rotation belongs.

One thing to check rather than assume at stage 2: the magnetometer message
reads 0.248 on x from a world field of 2.4e-05. The field is in tesla and the
reading is 1e4 times larger, which is the ratio between tesla and gauss.
HIL_SENSOR wants gauss. The units have to be confirmed against the message
definition before any scale factor is written down.

## Contents

    src/hitl_bridge.cpp        the plugin
    worlds/hitl_x500.sdf       HITL world: 250 Hz physics, sensor systems
    models/x500_voxl/          ModalAI's model, from px4-firmware
    scripts/build.sh           build
    scripts/run_sim.sh         run headless

`models/x500_voxl/model.sdf` is ModalAI's file with one change: its mesh paths
say `model://x500/meshes/...` and on this machine those meshes live in the
`x500_base` model, so the paths point there instead.

## Relation to the warehouse scanner

Separate from `~/starling`. That repository is the warehouse scan and does not
depend on this one. This exists so the scan can eventually be flown against a
real flight controller rather than only against PX4 SITL.
