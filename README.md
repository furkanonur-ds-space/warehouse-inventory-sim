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

All four stages of the host side are written and tested against a stand-in
flight controller. What has not happened yet is the part that matters: this
has never been connected to a real VOXL2.

| stage | what it does | state |
|---|---|---|
| 1 | read the simulated sensors, report rates and values | **done** |
| 2 | pack HIL_SENSOR and HIL_GPS, send over UDP 14560 | **done** |
| 3 | receive HIL_ACTUATOR_CONTROLS, drive the rotors | **done** |
| 4 | send ODOMETRY on UDP 14570 for the VIO path | **done** |

Next, when a drone is available and with the propellers off: point
`mavlink_addr` at the board, start `voxl-px4-hitl` on it, and find out what
this is still wrong about.

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

Against a real board:

    ./scripts/check_link.sh 192.168.101.2      # can this machine reach it
    ./scripts/run_hitl.sh -i 192.168.101.2     # real time, until Ctrl+C
    ./scripts/run_hitl.sh -i 192.168.101.2 -g  # with the Gazebo window

`run_hitl.sh` writes the address into a generated copy of the world and
checks that it arrived, since a world still pointed at 127.0.0.1 looks from
here exactly like a board that is not answering. The board is answering
when the bridge's report shows `| back: actuators` above 0 Hz.

Under WSL2 the USB link is only visible if WSL shares Windows' network:
`networkingMode=mirrored` under `[wsl2]` in `.wslconfig`, then
`wsl --shutdown`. `check_link.sh` says so when the ping fails.

What was checked against ModalAI's own sources before any board was
connected, motor order and the range of a motor command, is in
`docs/voxl2_compatibility.md`.

## Rehearsed against real PX4 on 2026-10-01

`scripts/rehearse_px4.sh` starts PX4's own flight stack on this machine,
standing where the board will stand: no simulator of its own, HIL messages
over UDP 14560, the same bridge on the other side. Armed, took off, held a
steady hover, landed on command.

The landing bounced two or three times before it settled. Not yet
explained. The likeliest suspect is that the rehearsal flies PX4's generic
none_iris parameters on the x500_voxl model, so hover thrust and the land
detector are tuned for a different airframe; the board will fly ModalAI's
HITL parameters, which were written for this model. That is a suspicion,
not a measurement.

Three faults turned up on the way, any one of which would have stopped the
first flight on the board:

- **Timestamps.** Messages were stamped with the update loop's clock, which
  moves once a physics step, so two IMU samples could leave with the same
  stamp. PX4 takes its sense of time from these stamps and refused the
  second one. Each message now carries its own sample time; 999 in a row
  checked, every gap exactly 4000 us.
- **The magnetic field.** The world's field pointed east and up at a
  northern latitude, which would have put the heading about 90 degrees out.
  It now comes from PX4's own World Magnetic Model tables for the world's
  latitude and longitude, by `scripts/magnetic_field.py`, and PX4's
  magnetometer checks pass with nothing relaxed.
- **Gazebo discovery.** With WSL in mirrored networking, which the board
  needs, Gazebo's multicast discovery has no route and the GUI cannot find
  the server. `GZ_IP=127.0.0.1` keeps it on this machine, as PX4 itself
  does.

## Measured on 2026-09-24, stage 4

    ODOMETRY 243 Hz, frames LOCAL_NED / BODY_FRD
    at rest    z -0.250 m   roll -0.0 deg   pitch 0.0 deg   heading 90.0 deg
    climbing   z -27.28 m   vz -13.97 m/s

The heading is 90 degrees rather than 0 because the model sits with its nose
along Gazebo's +x, which is east, and the message is in a north-referenced
frame. PX4's own `GZBridge::rotateQuaternion` produces the same value from
the same pose. An earlier version of the check called this a failure; the
check was wrong, not the bridge.

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
