# Warehouse Inventory Simulation

A UAV scans a warehouse and reports where every box is. The vehicle flies a
planned route past each shelf face, reads the QR label on every box, and writes
a JSON inventory mapping each code to a 3D position. No GPS is used at any
point.

The project has two halves, developed separately:

- `warehouse/` - the simulated warehouse: real pallet-racking dimensions, 432
  boxes with shipping labels, floor markers, and the ground truth to score
  against. Built by Ibrahim Cobankose.
- `scanner/` - the vehicle, the route and the scanning pipeline. Built by
  Furkan Onur.

The halves meet at one file, `scanner/layout.json`, which is the only place the
scanner learns anything about a warehouse.

## Results

Latest run, 2026-08-23, against `warehouse/ground_truth.json`:

| Metric | Value |
|---|---|
| Box QR codes decoded | 426 / 432 (99%) |
| Waypoints reached | 48 / 48 |
| Position error, median | 0.062 m |
| Position error, p95 | 0.087 m |
| Marker fixes applied | 614, across all eight markers |
| Final drift offset | 0.032 m |

By shelf level: 97%, 100%, 99%. Six of the eight shelf faces scanned 100%.

The six misses and the two positions worse than 0.2 m all fall in one window,
on face F, where the simulator stalled: the console showed
`vehicle_imu timestamp error` and `NodeShared::Publish() Interrupted system
call`, and the position estimate froze for a few seconds. The battery was
checked and ruled out - it never went below 50%, and `vehicle_status.failsafe`
was never set in 1800 samples.

## Localization: no GPS

Warehouses are indoor, so GNSS is either unavailable or too coarse. GPS is
disabled at three levels: the vehicle declares no GPS hardware, the simulator
publishes no usable fix, and EKF2 is told not to fuse GPS even if one appears.
Position comes from visual odometry, delivered to EKF2 as an external vision
source.

Measured live during this run:

```
cs_gnss_pos: False        xy_valid:   True
cs_ev_pos:   True         v_xy_valid: True
cs_ev_vel:   True
cs_ev_hgt:   True
cs_opt_flow: False
```

The parameters live in `scanner/px4_config/4023_gz_x500_c27`.

## layout.json

Everything the scanner knows about a warehouse. Nothing here is baked into
`scanner.py`, so flying a different warehouse is a new layout file.

| Field | Meaning | Where it comes from in `warehouse.yaml` |
|---|---|---|
| `world` | Gazebo world name | the world `gen_world.py` writes |
| `model` | vehicle model name | `scanner/build_c27_drone.py` |
| `aisle_faces` | one entry per scannable shelf face: the x of the shelf surface, and the heading held to look at it | `racking.rows` (`y0`, `facing`) mapped through `world_yaw` |
| `flight_z` | one altitude per shelf level | median z of the QR labels at each level, from `racking.level_heights` plus load height |
| `y_south`, `y_north` | the ends of each pass | `codes.aisle_marker.positions` |
| `spawn_x`, `spawn_y` | where the vehicle starts; the NED origin | `spawn.pose` |
| `shelf_standoff` | camera to shelf face | chosen for the camera, see below |
| `ground_offset` | how far base_link rests above the floor | measured, see below |
| `aruco_dictionary` | which ArUco dictionary the world was generated with | `codes.aisle_marker.dictionary` |
| `marker_map`, `output` | paths, relative to the layout file | |

Faces are listed rather than derived. An earlier version worked them out from
island geometry, which assumed every shelf run has an aisle on both sides. This
warehouse has two outer rows along the walls that do not, and deriving faces
would have invented a pass down the wall for each.

### Standoff

The vehicle does not fly the aisle centre line. It stands `shelf_standoff` out
from the face it is scanning, which puts it 0.40 m off centre in a 2.40 m
aisle, with 1.60 m to the opposite face.

The reason is resolution. A QR code needs about 3 pixels per module to decode.
The label modules here are 2.8 mm, and the scanning camera renders 1280 px
across a 60 degree field:

```
px per module = 3.10 / distance
```

At the 1.216 m aisle centre that is 2.55, below the threshold. At 0.80 m it is
3.79, which decodes reliably.

### ground_offset

This one cost 63% of a scan, so it is worth stating plainly.

Commanded altitudes are measured from the NED origin, which is captured at
startup while the vehicle is on the ground. base_link, and therefore the
camera, already sits 0.227 m above the floor at that moment. The shelf
altitudes in `flight_z` are world heights. Without correcting for the
difference, every commanded altitude is 0.227 m too high in world terms.

The effect was not subtle, but it looked like something else. Labels sat 0.16
to 0.26 m below the optical axis instead of on it. The camera's vertical
half-frame is 0.234 m at this distance, so of the three box heights at each
shelf level only the topmost fell inside it:

| Label height | Below the axis | Angle | Decoded |
|---|---|---|---|
| `flight_z` + 0.05 | 0.163 m | 12.8 deg | 98% |
| `flight_z` | 0.213 m | 16.5 deg | 22% |
| `flight_z` - 0.05 | 0.263 m | 20.1 deg | 0% |

Every other explanation was measured and ruled out first: readability (3.79 px
per module, ample), label distance (identical for all 432), lighting (decode
rate flat across the three levels), vehicle attitude (roll p95 2.9 deg, pitch
p95 0.3 deg - level), altitude tracking (commanded 0.75, actual 0.74). The
offset was then recovered by back-computing the camera height from decoded
labels of known position: +0.221, +0.232 and +0.227 m at the three levels.

The estimate for a box's height was wrong by about 0.25 m for the same reason,
but the snap-to-grid step rounded it to the nearest shelf level and hid it.

This never showed up in the original warehouse, where the racking was 2 m tall
with 0.65 m between levels: a 0.23 m shift stayed inside the frame. It took a
5 m pallet rack with three box heights per level to expose it.

## Drift correction

Eight ArUco markers sit on the floor at the aisle ends, at known positions. A
sighting gives an absolute fix, and the difference from the estimator is the
accumulated error. PX4 offers no way to reset the estimator through MAVSDK, so
the correction is applied as an offset subtracted from every commanded
setpoint: the estimator keeps its own mistaken idea of where the vehicle is,
and the vehicle still arrives where it was asked to go.

Corrections are only taken while hovering. In motion the airframe pitches and
the downward camera tilts with it, and the geometry reads that tilt as position
error.

The correction converges on the measured error rather than accumulating it.
Summing instead was a real defect: the offset was right after exactly two
sightings and then overshot without bound. It is unfalsifiable in simulation,
because Gazebo's OdometryPublisher reports the true pose and the quantity being
accumulated is always about zero. `scanner/test_drift_correction.py` drives the
geometry with synthetic frames and a known injected error, with no simulator,
and is what found it.

```sh
python3 test_drift_correction.py
```

## Naming

The scanner is `x500_c27`, its airframe `4023_gz_x500_c27`. Both halves of the
project previously wrote `x500_scanner` and `4022_gz_x500_scanner` into the
same PX4 paths, so whichever ran second overwrote the other's setup. The names
are deliberately distinct now, and `setup_px4.sh` touches nothing else.

## Setup

Requires Ubuntu 22.04, PX4 Autopilot (SITL), Gazebo Harmonic, and Python 3.10
with `mavsdk`, `opencv-python`, `numpy`, `qrcode`, `python-barcode` and `PyYAML`.

```sh
./setup_px4.sh
```

That generates the world and its 873 label textures, installs them into the PX4
tree, builds the vehicle model, and registers the airframe. Safe to re-run.

## Running

```sh
# Terminal 1
cd ~/PX4-Autopilot
export HEADLESS=1
export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA   # see below, worth 6x
export PX4_GZ_MODEL_POSE="-8.5,-9,0.30,0,0,0"
make px4_sitl gz_x500_c27_warehouse
```

### Pick the right GPU

On a laptop with both an integrated and a discrete GPU, Mesa under WSL picks
the integrated one, and Gazebo renders four cameras on it. Naming the discrete
adapter is the single largest speed-up available here.

Measured on this machine, same world, same model, 60 seconds of wall clock:

| Adapter | Simulated seconds elapsed | Real-time factor |
|---|---|---|
| Intel Iris Xe (default) | 9.6 | 0.16 |
| NVIDIA RTX 3050 | 56.2 | 0.94 |

A full 48-waypoint scan is about 40 minutes of simulated time. That is close
to four hours of waiting on the integrated GPU and about forty-five minutes on
the discrete one.

It is worth saying what this is not, because the advice is common and out of
date: WSL is not falling back to the CPU. GPU paravirtualization exposes the
adapter at `/dev/dxg`, `nvidia-smi` runs inside the guest, and `glxinfo`
reports `D3D12 (...)` rather than `llvmpipe`. Software rendering would show
llvmpipe, and choosing a different adapter would then change nothing at all.
Both GPUs are real; one is simply much faster than the other.

More CPU does not help. Gazebo sat between 130 and 180 percent across this
measurement, so it was not short of the eight processors WSL is given.

```sh
# Terminal 2
cd scanner
python3 verify_setup.py     # checks the GPS-free claim against the running system
python3 scanner.py
```

Output goes to `out/inventory_scanned.json`.

## Reports

Four tools in `report/` score a finished scan and draw it. They read the JSON
a run leaves behind and nothing else: no simulator, no ROS, no MAVSDK, and no
import from `scanner/`. They can be run on a different machine from the one
that flew, and a broken report cannot break a scan.

```sh
python3 report/validate_inventory.py --list-worst 5
python3 report/coverage_report.py --html out/coverage.html
python3 report/view_inventory.py
python3 report/drift_report.py --html out/drift.html
```

| Tool | Answers | Writes |
|---|---|---|
| `validate_inventory.py` | is the right product recorded in the right location, and how far out are the positions | `out/validation_report.json` |
| `coverage_report.py` | which parts of the warehouse the run actually covered, down to the bay | `out/coverage_report.json`, `out/coverage.html` |
| `view_inventory.py` | does every product sit in its rack, how far each estimate drifted, and what the boxes that were never decoded look like | `out/inventory_3d.html` |
| `drift_report.py` | how far the estimate wandered and whether the correction pulled it back | `out/drift_report.json`, `out/drift.html` |

Both HTML reports and the 3D view are single self-contained files with no
external asset, so they open offline and survive being mailed to someone. The
3D view embeds a small perspective renderer for the same reason: a three.js
from a CDN would show a blank page without a network.

The 3D view carries the drift and the misses, not only the positions. Every
decoded product is coloured by its distance from ground truth, with the spread
broken into bands beside the median - a tail of two records and a tail of fifty
read the same otherwise. Each floor marker carries an arrow showing the drift
measured there during the run, at ten times scale, because centimetres in a
twenty metre warehouse are otherwise a single pixel. Press `v` to cycle the
threshold above which error vectors are drawn.

A box that was never decoded is drawn as a wireframe box at its real
dimensions, so a miss that is a small carton on a top shelf looks different
from one that is not. Hovering any box gives its size, how far its label sat
from the optical axis, and how many pixels per QR module the camera had at
that standoff - the three properties that have actually explained a missed
read here. Those dimensions come from the generated world rather than being
recomputed from the yaml: which size landed in which slot was a seeded random
draw at generation time, and re-rolling it here would be a second
implementation of the same decision. `validate_inventory.py --list-missed`
prints the same properties as a table, and says whether the misses share
anything against the warehouse as a whole.

The headline number is inventory accuracy, not position error. A record counts
as correct when its payload exists in ground truth and the shelf face, level
and bay it was filed under are the right ones. Position error is reported in
metres beside it but stays out of that definition: a few centimetres that
leave the box in its own bay have not misfiled anything. The two are kept
apart because the scanner snaps each estimate to the nearest shelf face and
flight altitude, so much of the residual error is the gap between a flight
altitude and where the label sits inside that level - real, measurable, and
not a filing mistake.

`report/warehouse_model.py` holds what the tools share. It is the only place
that reads `warehouse.yaml`, and it applies the same `world_yaw` rotation
`gen_world.py` applies on the way out: the yaml is written in the generator's
own frame, so a rack position read straight out of it is not where that rack
is in the world. Ground truth and the scanner's estimates are both already in
world coordinates and are compared directly.

The scanner names a shelf face and a level but not a bay, so the bay each
record was filed under is read back out of the position it wrote. That keeps
these tools pure consumers - scoring a run needs no change to `scanner.py`.

Ground truth is read here for MEASUREMENT ONLY, and by these tools only. It
never enters an estimate; `scanner.py` does not open the file.

### Logging a run

A scan prints to the console and then the console is gone. `scripts/scan_logged.sh`
runs one with everything kept:

```sh
bash scripts/scan_logged.sh
```

| File | What it holds |
|---|---|
| `out/logs/scan_<timestamp>.log` | the scanner console: waypoints, every DETECTED and MARKER line, the closing summary |
| `out/logs/latest.log` | a link to the most recent of those |
| `out/flight_log.csv` | where the vehicle really was, 10 Hz: `t_s, wall_ms, x, y, z, yaw_deg` |

The track comes from `report/flight_log.py`, which can also be run on its own
in a second terminal. It is passive by design: one Gazebo topic in, one file
out, no MAVLink and nothing sent anywhere, so recording a flight cannot change
it. That is also why it does not log the estimate - reading that means opening
a MAVLink connection, which PX4 counts as a ground station, and one appearing
and disappearing mid-flight is a real disturbance. PX4 logs its own estimate at
full rate to `build/px4_sitl_default/rootfs/log/<date>/<time>.ulg`; that file
and this one are the two halves and can be compared afterwards.

`scanner.py` does not exit once it has landed. It writes the inventory and
the navigation report, calls land, and the process stays up - two runs left a
scanner sitting idle for hours before this was noticed. The results are on
disk by then, so watch for `SCAN COMPLETE` and `[INFO] Landing` in the console
and Ctrl-C the wrapper, which stops the recorder through its trap. `latest.log`
is linked before the flight starts rather than after it ends, so the console is
readable during the forty minutes that matter.

The wrapper sets `GZ_IP=127.0.0.1`. Without it gz-transport discovery is
unreliable here: topics that are publishing normally can read as empty, which
has already sent one diagnosis down the wrong path.

`out/synthetic_demo/` holds a set of reports built from a fabricated scan, so
the output can be seen without waiting on a 45 minute flight. Nothing in it is
a measurement of anything.

## Known issues

**Gazebo memory growth.** `gz sim` grows with every rendered frame. A 48
waypoint scan runs over 40 minutes of simulated time, and the stall that cost
the six missed codes is the same failure in milder form. Camera update rates
are already reduced for this (10 Hz on the scanning and downward cameras, 1 Hz
on the front and rear tracking cameras, which nothing consumes). Restart the
simulator between runs.

**Readability margin is thin.** 3.79 px per module at 0.80 m, against a
threshold of 3. Enlarging the label QR from 70 mm to 100 mm would let the
vehicle stand back at 0.95 m and give 3.73 px per module with far more room on
both sides. That is a change on the warehouse side.
