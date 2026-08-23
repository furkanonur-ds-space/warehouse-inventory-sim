"""
Verify that the simulation is configured the way the project claims it is.

This exists because a configuration claim went unchecked for several days:
the airframe disabled GPS, but the vehicle model had no optical flow or range
sensor, so the position estimate silently came from GPS anyway. Measurements
taken in that period were reported as GPS-free when they were not.

Run this before any measurement run. It checks the claim against the system,
not against the intention.

    python3 verify_setup.py
"""
import os
os.environ["PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION"] = "python"

import asyncio
import subprocess
import sys

from mavsdk import System

WORLD = "warehouse"
DRONE = "x500_c27_0"

REQUIRED_TOPICS = {
    "visual odometry":  f"/model/{DRONE}/odometry_with_covariance",
    "scanning camera":  f"/world/{WORLD}/model/{DRONE}/link/camera_hires_link/sensor/camera/image",
    "downward camera":  f"/world/{WORLD}/model/{DRONE}/link/camera_track_down_link/sensor/camera/image",
}

results = []


def record(name, ok, detail=""):
    results.append((name, ok, detail))
    mark = "PASS" if ok else "FAIL"
    line = f"  [{mark}] {name}"
    if detail:
        line += f"  {detail}"
    print(line)


def check_gazebo_topics():
    print("\nGazebo topics")
    try:
        out = subprocess.run(["gz", "topic", "-l"], capture_output=True,
                             text=True, timeout=10).stdout
    except Exception as error:
        record("gz topic -l", False, str(error))
        return

    for name, topic in REQUIRED_TOPICS.items():
        record(name, topic in out, "" if topic in out else "topic absent")


def check_model_file():
    print("\nVehicle model")
    path = os.path.expanduser(
        "~/PX4-Autopilot/Tools/simulation/gz/models/x500_c27/model.sdf")
    try:
        with open(path) as handle:
            sdf = handle.read()
    except Exception as error:
        record("model.sdf readable", False, str(error))
        return

    record("odometry publisher included", "OdometryPublisher" in sdf)


def check_airframe():
    print("\nAirframe configuration")
    path = os.path.expanduser(
        "~/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/"
        "4023_gz_x500_c27")
    try:
        with open(path) as handle:
            text = handle.read()
    except Exception as error:
        record("airframe readable", False, str(error))
        return

    record("SYS_HAS_GPS 0", "SYS_HAS_GPS 0" in text)
    record("EKF2_GPS_CTRL 0", "EKF2_GPS_CTRL 0" in text)
    record("EKF2_OF_CTRL 0", "EKF2_OF_CTRL 0" in text)
    record("EKF2_EV_CTRL 15", "EKF2_EV_CTRL 15" in text,
           "external vision must be enabled for VIO")
    record("EKF2_HGT_REF 3", "EKF2_HGT_REF 3" in text,
           "height must come from external vision")


async def check_estimator():
    """
    Check what the estimator is actually doing.

    EKF2 needs a little time after startup before optical flow settles, so the
    health flags are polled for a while rather than read once.
    """
    print("\nEstimator state")
    drone = System()
    try:
        await drone.connect(system_address="udp://:14540")
    except Exception as error:
        record("vehicle connection", False, str(error))
        return

    connected = False
    try:
        async for state in drone.core.connection_state():
            if state.is_connected:
                connected = True
                break
    except Exception as error:
        record("vehicle connection", False, str(error))
        return

    record("vehicle connection", connected)
    if not connected:
        return

    print("  waiting up to 40 s for the estimator to settle")
    best = None
    deadline = asyncio.get_event_loop().time() + 40
    async for health in drone.telemetry.health():
        best = health
        if health.is_local_position_ok and health.is_home_position_ok:
            break
        if asyncio.get_event_loop().time() > deadline:
            break

    if best is None:
        record("health readout", False, "no telemetry received")
        return

    record("local position valid", best.is_local_position_ok)
    record("home position set", best.is_home_position_ok)
    record("global position absent", not best.is_global_position_ok,
           "a valid global position means GPS is still in use")

    if not best.is_local_position_ok:
        print("\n  Sensor flags at the end of the wait:")
        print(f"    gyro calibrated      {best.is_gyrometer_calibration_ok}")
        print(f"    accel calibrated     {best.is_accelerometer_calibration_ok}")
        print(f"    mag calibrated       {best.is_magnetometer_calibration_ok}")
        print(f"    local position ok    {best.is_local_position_ok}")
        print(f"    global position ok   {best.is_global_position_ok}")
        print(f"    home position ok     {best.is_home_position_ok}")


async def _first(stream):
    async for item in stream:
        return item
    return None


async def main():
    print("=" * 66)
    print("  SIMULATION SETUP VERIFICATION")
    print("=" * 66)

    check_airframe()
    check_model_file()
    check_gazebo_topics()
    await check_estimator()

    passed = sum(1 for _, ok, _ in results if ok)
    total = len(results)

    print("\n" + "=" * 66)
    print(f"  {passed}/{total} checks passed")
    print("=" * 66)

    failures = [name for name, ok, _ in results if not ok]
    if failures:
        print("\n  Failed:")
        for name in failures:
            print(f"    {name}")
        print("\n  Do not record measurements until these pass. A run with")
        print("  GPS silently active does not measure GPS-free performance.")
        sys.exit(1)

    print("\n  Configuration matches the GPS-free claim. Measurements taken")
    print("  now are valid.")


if __name__ == "__main__":
    asyncio.run(main())
