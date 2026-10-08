#!/usr/bin/env python3
"""Listen to what the bridge sends and check it against what PX4 expects.

This is the test that stage 2 stands on. It decodes the UDP stream with an
independent implementation, pymavlink, rather than trusting the encoder that
produced it, and it checks the values rather than only the rates: a bridge
that sends 250 well formed messages a second with the sign of gravity wrong
looks perfectly healthy until the vehicle flips.

    python3 scripts/check_stream.py --seconds 6

Run it before starting the simulator; it binds the port and waits.
"""

import argparse
import math
import sys
import time
from collections import defaultdict

from pymavlink import mavutil

# What the vehicle is doing while this runs: sitting on the ground, not
# moving. Everything below follows from that.
EXPECTED = {
    # field, expected, tolerance, unit, why
    "zacc": (-9.8066, 0.05, "m/s^2", "gravity in FRD points down, so z is negative"),
    "xacc": (0.0, 0.05, "m/s^2", "at rest"),
    "yacc": (0.0, 0.05, "m/s^2", "at rest"),
    "xgyro": (0.0, 0.01, "rad/s", "at rest"),
    "ygyro": (0.0, 0.01, "rad/s", "at rest"),
    "zgyro": (0.0, 0.01, "rad/s", "at rest"),
    "abs_pressure": (1013.22, 5.0, "hPa", "sea level, and hPa not Pa"),
}

# Magnetic field magnitude. Earth's is 0.25 to 0.65 gauss; a value a
# thousand times smaller means tesla leaked through, and a value that size
# is exactly the mistake this project nearly made.
MAG_MIN_GAUSS = 0.2
MAG_MAX_GAUSS = 0.7

RATES = {"HIL_SENSOR": 250.0, "HIL_GPS": 30.0}
RATE_TOLERANCE = 0.1  # 10 per cent

# fields_updated, from PX4's SensorSource enum in SimulatorMavlink.hpp. PX4
# tests (fields_updated & X) == X, so BARO is three bits and a message that
# sets only the pressure bit is ignored without a word.
FIELD_ACCEL = 0b111
FIELD_GYRO = 0b111000
FIELD_MAG = 0b111000000
FIELD_BARO = 0b1101000000000

# The magnetometer and the barometer run slower than the IMU and ride on the
# HIL_SENSOR messages that follow their own arrivals. These are the rates
# their bits should appear at, not the rate of the message that carries them.
FIELD_RATES = {"mag": (FIELD_MAG, 50.0), "baro": (FIELD_BARO, 10.0)}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=14560)
    parser.add_argument("--seconds", type=float, default=6.0)
    args = parser.parse_args()

    link = mavutil.mavlink_connection(
        "udpin:127.0.0.1:%d" % args.port, dialect="common")
    print("listening on udp %d, waiting for the first message" % args.port)

    counts = defaultdict(int)
    last = {}
    # The last message that actually carried each slow field, which is not
    # the last message overall: most HIL_SENSOR messages carry neither.
    last_with_field = {}
    field_counts = defaultdict(int)
    started = None
    first_time_usec = None
    last_time_usec = None
    deadline = time.time() + args.seconds + 30.0

    while time.time() < deadline:
        msg = link.recv_match(blocking=True, timeout=1.0)
        if msg is None:
            if started is not None:
                break
            continue
        name = msg.get_type()
        if name == "BAD_DATA":
            counts["BAD_DATA"] += 1
            continue
        if started is None:
            started = time.time()
            print("first message after %s" % name)
        counts[name] += 1
        last[name] = msg
        if name == "HIL_SENSOR":
            if first_time_usec is None:
                first_time_usec = msg.time_usec
            last_time_usec = msg.time_usec
            for field, (mask, _) in FIELD_RATES.items():
                if msg.fields_updated & mask == mask:
                    field_counts[field] += 1
                    last_with_field[field] = msg
        if time.time() - started >= args.seconds:
            break

    if started is None:
        print("FAIL nothing arrived on port %d" % args.port)
        return 1

    # Rates are measured in the simulator's own clock, taken from the
    # message timestamps, not from this machine's wall clock. A slow
    # machine stretches the wall clock; PX4 sees the stamps.
    span_s = ((last_time_usec - first_time_usec) / 1e6
              if first_time_usec is not None and last_time_usec else 0.0)
    if span_s <= 0.0:
        print("FAIL message timestamps did not advance")
        return 1

    print()
    print("over %.2f s of simulated time" % span_s)
    failures = []

    for name, expected_hz in RATES.items():
        hz = counts[name] / span_s
        ok = abs(hz - expected_hz) <= expected_hz * RATE_TOLERANCE
        print("  %-10s %7.1f Hz   expected %5.0f   %s"
              % (name, hz, expected_hz, "ok" if ok else "FAIL"))
        if not ok:
            failures.append("%s rate %.1f Hz" % (name, hz))

    if counts["BAD_DATA"]:
        print("  %d undecodable packets" % counts["BAD_DATA"])
        failures.append("undecodable packets")

    sensor = last.get("HIL_SENSOR")
    if sensor is None:
        print("FAIL no HIL_SENSOR arrived")
        return 1

    # The slow fields arrive on their own schedule, marked in fields_updated.
    # PX4 ignores them when the bits are absent, so their rate is the rate
    # PX4 actually gets.
    print()
    for field, (mask, expected_hz) in FIELD_RATES.items():
        hz = field_counts[field] / span_s
        ok = abs(hz - expected_hz) <= expected_hz * RATE_TOLERANCE
        print("  %-10s %7.1f Hz   expected %5.0f   %s   (marked 0x%04x in "
              "fields_updated)"
              % (field, hz, expected_hz, "ok" if ok else "FAIL", mask))
        if not ok:
            failures.append("%s field rate %.1f Hz" % (field, hz))

    print()
    print("last HIL_SENSOR, vehicle at rest:")
    for field, (want, tol, unit, why) in EXPECTED.items():
        # Read each field from the last message that claimed to carry it.
        source = sensor
        if field == "abs_pressure":
            source = last_with_field.get("baro")
            if source is None:
                failures.append("no message ever marked the barometer")
                continue
        got = getattr(source, field)
        ok = abs(got - want) <= tol
        print("  %-13s %10.4f %-7s expected %8.3f   %s   (%s)"
              % (field, got, unit, want, "ok" if ok else "FAIL", why))
        if not ok:
            failures.append("%s = %g" % (field, got))

    mag_msg = last_with_field.get("mag")
    if mag_msg is None:
        failures.append("no message ever marked the magnetometer")
    else:
        mag = math.sqrt(mag_msg.xmag ** 2 + mag_msg.ymag ** 2
                        + mag_msg.zmag ** 2)
        mag_ok = MAG_MIN_GAUSS <= mag <= MAG_MAX_GAUSS
        print("  %-13s %10.4f %-7s expected %.2f to %.2f   %s   (%s)"
              % ("|mag|", mag, "gauss", MAG_MIN_GAUSS, MAG_MAX_GAUSS,
                 "ok" if mag_ok else "FAIL", "already gauss, no scaling"))
        if not mag_ok:
            failures.append("|mag| = %g gauss" % mag)

    baro_msg = last_with_field.get("baro")
    if baro_msg is not None:
        print("  %-13s %10.1f %-7s (%s)"
              % ("pressure_alt", baro_msg.pressure_alt, "m",
                 "PX4 requires it alongside the pressure"))

    print("  fields_updated of the last message: 0x%04x"
          % sensor.fields_updated)
    if sensor.fields_updated & (FIELD_ACCEL | FIELD_GYRO) != (
            FIELD_ACCEL | FIELD_GYRO):
        failures.append("accel/gyro not marked updated")

    gps = last.get("HIL_GPS")
    if gps is not None:
        print()
        print("last HIL_GPS:")
        print("  lat %d (%.6f deg)" % (gps.lat, gps.lat / 1e7))
        print("  lon %d (%.6f deg)" % (gps.lon, gps.lon / 1e7))
        print("  alt %d mm (%.2f m)" % (gps.alt, gps.alt / 1000.0))
        print("  fix_type %d, satellites %d" % (gps.fix_type, gps.satellites_visible))
        if gps.fix_type < 3:
            failures.append("gps fix_type %d" % gps.fix_type)
        if not (-900000000 < gps.lat < 900000000) or gps.lat == 0:
            failures.append("gps latitude %d" % gps.lat)

    print()
    if failures:
        print("FAIL: " + "; ".join(failures))
        return 1
    print("PASS: the stream is what PX4 expects")
    return 0


if __name__ == "__main__":
    sys.exit(main())
