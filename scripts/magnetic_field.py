#!/usr/bin/env python3
"""
The magnetic field a world should carry, from PX4's own model of the earth.

PX4 compares the magnetometer against the World Magnetic Model for wherever
GPS says the vehicle is, and turns the difference into a heading. So the
field in the world file is not decoration: get it wrong and the vehicle
believes north is somewhere else. The first version of this world had the
field pointing east and up at a northern latitude, which would have put the
heading about 90 degrees out on the first HITL flight.

This reads the same tables PX4 is built with
(src/lib/world_magnetic_model/geo_magnetic_tables.hpp) and interpolates them
the way geo_mag_declination.cpp does, so the world and the flight controller
agree by construction rather than by a number copied from somewhere.

    python3 scripts/magnetic_field.py 39.925 32.837

Prints the <magnetic_field> line for the world, in tesla, ENU.
"""

import math
import os
import re
import sys

TABLES = os.path.expanduser(
    "~/PX4-Autopilot/src/lib/world_magnetic_model/geo_magnetic_tables.hpp")


def read_tables(path):
    with open(path, encoding="utf-8") as handle:
        text = handle.read()

    def number(name):
        match = re.search(r"%s\s*=\s*(-?[\d.]+)f?;" % name, text)
        return float(match.group(1))

    def table(name):
        match = re.search(r"%s\[\d+\]\[\d+\]\s*\{(.*?)\};" % name, text,
                          re.S)
        rows = re.findall(r"\{([^{}]*)\}", match.group(1))
        return [[int(v) for v in re.findall(r"-?\d+", row)] for row in rows]

    return {
        "res": number("SAMPLING_RES"),
        "min_lat": number("SAMPLING_MIN_LAT"),
        "min_lon": number("SAMPLING_MIN_LON"),
        "declination": table("declination_table"),
        "inclination": table("inclination_table"),
        "strength": table("totalintensity_table"),
        "dec_scale": number("WMM_DECLINATION_SCALE_TO_DEGREES"),
        "inc_scale": number("WMM_INCLINATION_SCALE_TO_DEGREES"),
        "str_scale": number("WMM_TOTALINTENSITY_SCALE_TO_NANOTESLA"),
    }


def lookup(tables, grid, lat, lon):
    """Bilinear interpolation, as get_table_data() in PX4 does it."""
    res = tables["res"]
    min_lat = math.floor(lat / res) * res
    min_lon = math.floor(lon / res) * res
    lat_index = int((min_lat - tables["min_lat"]) / res)
    lon_index = int((min_lon - tables["min_lon"]) / res)

    sw = grid[lat_index][lon_index]
    se = grid[lat_index][lon_index + 1]
    ne = grid[lat_index + 1][lon_index + 1]
    nw = grid[lat_index + 1][lon_index]

    lat_scale = (lat - min_lat) / res
    lon_scale = (lon - min_lon) / res
    low = lon_scale * (se - sw) + sw
    high = lon_scale * (ne - nw) + nw
    return lat_scale * (high - low) + low


def main():
    if len(sys.argv) != 3:
        print("usage: %s <lat> <lon>" % sys.argv[0], file=sys.stderr)
        return 2
    lat, lon = float(sys.argv[1]), float(sys.argv[2])
    tables = read_tables(TABLES)

    declination = lookup(tables, tables["declination"], lat, lon) \
        * tables["dec_scale"]
    inclination = lookup(tables, tables["inclination"], lat, lon) \
        * tables["inc_scale"]
    strength_nt = lookup(tables, tables["strength"], lat, lon) \
        * tables["str_scale"]

    # North, east, down from strength, inclination and declination.
    strength_t = strength_nt * 1e-9
    horizontal = strength_t * math.cos(math.radians(inclination))
    north = horizontal * math.cos(math.radians(declination))
    east = horizontal * math.sin(math.radians(declination))
    down = strength_t * math.sin(math.radians(inclination))

    print("# lat %.4f lon %.4f: declination %.2f deg, inclination %.2f deg, "
          "%.0f nT" % (lat, lon, declination, inclination, strength_nt))
    # Gazebo's world frame is ENU: east, north, up.
    print("<magnetic_field>%.4e %.4e %.4e</magnetic_field>"
          % (east, north, -down))
    return 0


if __name__ == "__main__":
    sys.exit(main())
