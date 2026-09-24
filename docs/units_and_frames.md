# Units and frames at the boundary

What stage 2 has to convert, and where each answer came from. Written before
the conversion code, because guessing any of these produces a vehicle that
flies but leans, drifts or flips, and the symptom never names the cause.

## What HIL_SENSOR asks for

From `message_definitions/v1.0/common.xml` in the MAVLink repository,
message 107, which describes itself as "IMU readings in SI units in NED body
frame":

| field | unit |
|---|---|
| xacc, yacc, zacc | m/s/s |
| xgyro, ygyro, zgyro | rad/s |
| xmag, ymag, zmag | **gauss** |
| abs_pressure, diff_pressure | **hPa** |
| temperature | degC |

HIL_GPS (113) is integers, not SI: lat and lon in degE7, altitude in mm,
velocities in cm/s, course over ground in cdeg.

## What Gazebo Harmonic delivers

Measured on this machine, five seconds at rest, and cross-checked against
PX4's own Gazebo bridge.

**Accelerometer.** Reads +9.8066 on z at rest. Gazebo's body frame is FLU
(forward-left-up), PX4 wants FRD (forward-right-down), so y and z are
negated.

**Magnetometer.** Already in gauss, not tesla, despite the field being named
`field_tesla`. The world here is configured with 2.4e-05 and the message
carries 0.248, which is the tesla-to-gauss ratio. No scale factor is needed
for HIL_SENSOR, and applying one would be wrong.

The frame is not FLU either. PX4's `GZBridge::magnetometerCallback` says so
and compensates:

    // The magnetometer plugin publishes in units of gauss and in a weird
    // left handed coordinate system
    // https://github.com/gazebosim/gz-sim/pull/2460
    report.x = -msg.field_tesla().y();
    report.y = -msg.field_tesla().x();
    report.z =  msg.field_tesla().z();

`sensor_mag` is in body FRD, which is what HIL_SENSOR wants, so this mapping
carries over unchanged. The linked gz-sim pull request fixes the orientation
in a later Gazebo release, so this is a Harmonic-specific correction and has
to be rechecked if this machine ever moves to Jetty.

**Barometer.** Gazebo publishes pascals; the message wants hectopascals, so
the value is divided by 100. PX4 does the same in reverse when it receives
HIL_SENSOR: `_last_baro_pressure = sensors.abs_pressure * 100.f; // hPa to Pa`

**GPS.** Degrees and metres from Gazebo, integers for the message: degrees
multiplied by 1e7, altitude in millimetres.

## Sources

- MAVLink message definitions, `common.xml`, cloned 2026-09-24
- `~/PX4-Autopilot/src/modules/simulation/gz_bridge/GZBridge.cpp`, the
  magnetometer and air pressure callbacks
- `~/PX4-Autopilot/src/modules/simulation/simulator_mavlink/SimulatorMavlink.cpp`,
  which is the receiving end and shows the hPa to Pa conversion
- ModalAI's HITL document, which names the ENU/FLU to NED/FRD rotation as the
  bridge's job but does not give the magnetometer quirk
