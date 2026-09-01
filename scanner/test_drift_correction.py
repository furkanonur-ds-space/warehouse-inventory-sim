"""
Test the ArUco drift correction geometry without flying.

Why this exists: in simulation the position estimate comes from Gazebo's
OdometryPublisher, which reports the model's true pose. The simulated VIO
therefore never drifts, the correction has nothing to correct, and a flight can
never show whether it would cancel a real error or make one worse. On real
hardware VIO does drift, so the geometry has to be right before it matters.

The test drives the correction backwards and forwards. It places the vehicle at
a known true position over a marker of known world position, projects where
that marker must appear in the downward camera, renders a frame containing it,
then hands the frame to on_down_image together with a deliberately wrong
position estimate. A correct implementation recovers the injected error.

Run:  python3 test_drift_correction.py
"""
import math
import os
import sys

import cv2
import numpy as np

import scanner as ws


class FakeImage:
    """Minimal stand-in for the Gazebo image message."""

    def __init__(self, bgr):
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        self.data = rgb.tobytes()
        self.height, self.width = bgr.shape[:2]


def render_marker_frame(marker_id, vehicle_x, vehicle_y, altitude, yaw_deg,
                        marker_x, marker_y, width=640, height=480,
                        marker_size_m=0.4):
    """
    Render what the downward camera sees, using the forward projection.

    This is the inverse of the calculation under test: it turns a world
    geometry into a pixel position, so the test does not simply reuse the
    implementation it is checking.
    """
    h_fov = math.radians(ws.DOWN_CAM_HFOV_DEG)
    v_fov = 2 * math.atan(math.tan(h_fov / 2) * height / width)

    # Offset from vehicle to marker, in the world frame
    dx_world = marker_x - vehicle_x
    dy_world = marker_y - vehicle_y

    # Rotate into the body frame. Yaw 0 is north, which is +Y in Gazebo.
    yaw_rad = math.radians(yaw_deg)
    fx, fy = math.sin(yaw_rad), math.cos(yaw_rad)
    rx, ry = math.cos(yaw_rad), -math.sin(yaw_rad)
    forward_m = fx * dx_world + fy * dy_world
    right_m = rx * dx_world + ry * dy_world

    # Ground offsets back to normalised image coordinates
    dx_norm = right_m / (altitude * math.tan(h_fov / 2))
    dy_norm = -forward_m / (altitude * math.tan(v_fov / 2))
    cx = width / 2 + dx_norm * (width / 2)
    cy = height / 2 + dy_norm * (height / 2)

    # A mid-grey floor, matching how Gazebo renders it
    frame = np.full((height, width, 3), 160, dtype=np.uint8)

    px_per_m = width / (2 * altitude * math.tan(h_fov / 2))
    side = int(round(marker_size_m * px_per_m))
    if side < 24:
        raise ValueError(f"marker only {side} px, too small to detect")

    marker = cv2.aruco.generateImageMarker(
        cv2.aruco.getPredefinedDictionary(ws.ARUCO_DICT), marker_id, side)
    quiet = max(4, side // 10)
    marker = cv2.copyMakeBorder(marker, quiet, quiet, quiet, quiet,
                                cv2.BORDER_CONSTANT, value=255)
    marker = cv2.cvtColor(marker, cv2.COLOR_GRAY2BGR)

    mh, mw = marker.shape[:2]
    x0 = int(round(cx - mw / 2))
    y0 = int(round(cy - mh / 2))
    if x0 < 0 or y0 < 0 or x0 + mw > width or y0 + mh > height:
        raise ValueError("marker falls outside the frame")
    frame[y0:y0 + mh, x0:x0 + mw] = marker
    return frame


def reset_state(est_n, est_e, altitude, yaw_deg):
    """Put the module into a known state before a sighting."""
    ws.marker_map = {"1": {"x": -6.0, "y": -8.0}, "5": {"x": 2.0, "y": -8.0}}
    # Chosen so the NED frame and the Gazebo world frame coincide:
    # gazebo_to_ned(x, y) then returns (y, x), matching the real flights.
    ws.origin_pos = {"n": ws.SPAWN_Y, "e": ws.SPAWN_X, "d": 0.0}
    ws.current_pos = {"n": est_n, "e": est_e, "d": -altitude}
    ws.current_yaw = {"deg": yaw_deg}
    ws.drift_offset = {"n": 0.0, "e": 0.0}
    ws.marker_events = []
    ws.last_correction_time = [0.0]
    ws.is_settled = True


def run_case(name, err_n, err_e, yaw_deg=0.0, altitude=1.8,
             marker_id=1, sightings=12, expect_rejected=False):
    """
    Inject a known estimator error and see what the correction recovers.

    The vehicle sits directly over the marker. The estimate is wrong by
    (err_n, err_e), which is exactly what the correction should cancel.
    """
    marker = ws.marker_map_for_test[str(marker_id)]
    true_x, true_y = marker["x"], marker["y"]

    # NED estimate of a vehicle that is really at (true_x, true_y)
    true_n, true_e = true_y, true_x
    est_n, est_e = true_n + err_n, true_e + err_e

    reset_state(est_n, est_e, altitude, yaw_deg)

    frame = render_marker_frame(marker_id, true_x, true_y, altitude, yaw_deg,
                                true_x, true_y)
    msg = FakeImage(frame)

    for _ in range(sightings):
        ws.last_correction_time[0] = 0.0      # bypass the 1 s throttle
        ws.current_pos = {"n": est_n, "e": est_e, "d": -altitude}
        ws.on_down_image(msg)

    # The correction converges to the negative of the estimator error, so that
    # corrected() adds the error back onto every commanded setpoint.
    got_n = ws.drift_offset["n"]
    got_e = ws.drift_offset["e"]
    residual = math.hypot(got_n + err_n, got_e + err_e)
    injected = math.hypot(err_n, err_e)

    if expect_rejected:
        ok = len(ws.marker_events) == 0 and injected > 0
        verdict = "PASS" if ok else "FAIL"
        print(f"  [{verdict}] {name}")
        print(f"          injected {injected:.2f} m, beyond the 2 m gate")
        print(f"          sightings accepted: {len(ws.marker_events)} "
              f"(expected 0), offset {math.hypot(got_n, got_e):.3f} m")
        return ok

    ok = residual < 0.15 and len(ws.marker_events) > 0
    verdict = "PASS" if ok else "FAIL"
    print(f"  [{verdict}] {name}")
    print(f"          injected error   n={err_n:+.2f} e={err_e:+.2f}  "
          f"({injected:.2f} m)")
    print(f"          recovered offset n={got_n:+.2f} e={got_e:+.2f}  "
          f"(want {-err_n:+.2f}, {-err_e:+.2f})")
    print(f"          residual {residual:.3f} m over {len(ws.marker_events)} "
          f"sightings")
    return ok


def main():
    # Keep a pristine copy: reset_state rewrites ws.marker_map each time.
    ws.marker_map_for_test = {"1": {"x": -6.0, "y": -8.0},
                              "5": {"x": 2.0, "y": -8.0}}

    print("=" * 68)
    print("  ARUCO DRIFT CORRECTION, GEOMETRY TEST")
    print("  No simulator: the correction is driven with synthetic frames.")
    print("=" * 68)
    print()

    results = []
    results.append(run_case("no error, correction must not invent one",
                            0.0, 0.0))
    results.append(run_case("0.50 m north", 0.50, 0.0))
    results.append(run_case("0.40 m east", 0.0, 0.40))
    results.append(run_case("0.35 m diagonal", 0.35, -0.30))
    results.append(run_case("0.50 m north, vehicle yawed 90 deg",
                            0.50, 0.0, yaw_deg=90.0))
    results.append(run_case("0.40 m diagonal, yawed -90 deg",
                            -0.30, 0.40, yaw_deg=-90.0))
    results.append(run_case("second marker id, 0.45 m north",
                            0.45, 0.0, marker_id=5))
    results.append(run_case("3.0 m error is implausible, must be rejected",
                            3.0, 0.0, expect_rejected=True))

    passed = sum(1 for r in results if r)
    print()
    print("=" * 68)
    print(f"  {passed}/{len(results)} passed")
    print("=" * 68)
    if passed != len(results):
        sys.exit(1)


if __name__ == "__main__":
    os.chdir(os.path.dirname(os.path.abspath(__file__)))
    main()
