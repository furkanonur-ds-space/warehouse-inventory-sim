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
../.venv/bin/python test_drift_correction.py
```

## Naming

The scanner is `x500_c27`, its airframe `4023_gz_x500_c27`. Both halves of the
project previously wrote `x500_scanner` and `4022_gz_x500_scanner` into the
same PX4 paths, so whichever ran second overwrote the other's setup. The names
are deliberately distinct now, and `setup_px4.sh` touches nothing else.

## Setup

Requires Ubuntu, PX4 Autopilot (SITL) built at `~/PX4-Autopilot`, and Gazebo
Harmonic with its Python bindings (`python3-gz-transport13`,
`python3-gz-msgs10`, from the Gazebo apt repository).

### 1. A virtualenv for the project

```sh
python3 -m venv --system-site-packages .venv
.venv/bin/pip install mavsdk opencv-python qrcode python-barcode PyYAML numpy pyzbar
```

`--system-site-packages` is not optional. The Gazebo Python bindings are
installed by apt and are not on PyPI, so a sealed virtualenv cannot see them
and every tool that reads a camera or a pose fails at import.

`opencv-python` is also not optional, even where apt has already put OpenCV on
the machine. Ubuntu ships 4.6, which has neither `cv2.aruco.generateImageMarker`
nor `cv2.aruco.ArucoDetector`: the world generator cannot draw the floor
markers and the scanner cannot detect them. The pip build shadows the system
one inside the virtualenv and leaves the rest of the machine alone.

### 2. Install into PX4

```sh
PYTHON="$PWD/.venv/bin/python" ./setup_px4.sh
```

That generates the world and its 873 label textures, installs them into the PX4
tree, builds the vehicle model, and registers the airframe. Safe to re-run.
`PYTHON` overrides the interpreter it uses; without it the script looks for one
at a path that only exists on the machine it was written on.

### 3. Reconfigure PX4 once

```sh
cd ~/PX4-Autopilot/build/px4_sitl_default && cmake .
```

Do this after the first `setup_px4.sh`, and again any time an airframe file is
added or removed. PX4 builds its `gz_<model>_<world>` make targets from a
`file(GLOB ...)` over the airframes directory, and a glob is evaluated when
CMake configures, not when make runs. A newly registered airframe is invisible
until something makes CMake configure again, and the symptom is
`No rule to make target 'gz_x500_c27_warehouse'` after a build that otherwise
succeeded.

## Running

```sh
# Terminal 1
cd ~/PX4-Autopilot
export HEADLESS=1                               # drop this to watch the scene
export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA   # WSL only, see below, worth 6x
export PX4_GZ_MODEL_POSE="-8.5,-9,0.30,0,0,0"
make px4_sitl gz_x500_c27_warehouse
```

Wait for `INFO [commander] Ready for takeoff!` before starting the scan.

`HEADLESS=1` starts the server with no window. Everything still renders -
the cameras are sensors, not a display - and a scan runs faster without the
viewport competing for the GPU. Note that `export` outlives the command: a
shell that has set it once keeps it set, and `unset HEADLESS` is what turns the
window back on.

```sh
# Terminal 2
bash scripts/scan_logged.sh
```

That is the flight, with the console and a position track written to disk as
it goes. See "Logging a run" below for what it leaves behind. The scan itself
is `scanner/scanner.py`, which can be run directly if a recording is not
wanted:

```sh
cd scanner
GZ_IP=127.0.0.1 ../.venv/bin/python verify_setup.py   # checks the GPS-free claim live
GZ_IP=127.0.0.1 ../.venv/bin/python -u scanner.py
```

`GZ_IP=127.0.0.1` matters. PX4 launches the simulator with it set, and a
process that subscribes without it can find some topics and silently miss
others - a camera that is publishing at 10 Hz reads as dead. `-u` keeps the
console unbuffered; without it a forty minute flight prints nothing until it
ends. `scan_logged.sh` sets both.

Output goes to `out/inventory_scanned.json` and `out/navigation_report.json`.

`scanner.py` does not exit after it lands. It writes both files, calls land,
and the process stays up; Ctrl-C once `SCAN COMPLETE` and `[INFO] Landing` have
printed. Leave the simulator running only if another scan is coming - a PX4
still alive from a previous run refuses the next one with `PX4 server already
running for instance 0`.

Do not run a second MAVLink client while a scan is in the air. PX4 counts each
connection as a ground station, and one that appears and disappears produces
`Connection to ground station lost` and a preflight failure in the middle of a
flight. Run `verify_setup.py` before the scan, not beside it.

### Pick the right GPU

This section is about WSL. On a native Linux install with PRIME set to the
discrete card, `MESA_D3D12_DEFAULT_ADAPTER_NAME` does nothing: there is no
D3D12 layer to steer, `glxinfo -B` already names the discrete GPU, and the
`libEGL warning: failed to create dri2 screen` lines Gazebo prints there are
noise from Mesa not recognising the card, after which the vendor EGL takes
over and the cameras render normally.

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


## Reports

Five tools in `report/` score a finished scan and draw it. They read the JSON
a run leaves behind and nothing else: no simulator, no ROS, no MAVSDK, and no
import from `scanner/`. They can be run on a different machine from the one
that flew, and a broken report cannot break a scan.

```sh
.venv/bin/python report/validate_inventory.py --list-worst 5 --list-missed
.venv/bin/python report/coverage_report.py --html out/coverage.html
.venv/bin/python report/view_inventory.py
.venv/bin/python report/drift_report.py --html out/drift.html
.venv/bin/python report/missed_log.py
```

Or all five at once, after a scan has landed:

```sh
bash scripts/make_reports.sh
```

`out/` is gitignored: every run rewrites it. A run worth keeping is copied
whole into `results/<date>/`, which is what `results/2026-08-27/` holds -
the reports, the raw inventory, the flight track, the consoles and the
barcode readings, exactly as that flight left them.

| Tool | Answers | Writes |
|---|---|---|
| `validate_inventory.py` | is the right product recorded in the right location, and how far out are the positions | `out/validation_report.json`, `out/position_offsets.csv` |
| `coverage_report.py` | which parts of the warehouse the run actually covered, down to the bay | `out/coverage_report.json`, `out/coverage.html` |
| `view_inventory.py` | does every product sit in its rack, how far each estimate shifted from where the box really is, and what the boxes that were never decoded look like | `out/inventory_3d.html` |
| `drift_report.py` | how far the estimate wandered and whether the correction pulled it back | `out/drift_report.json`, `out/drift.html` |
| `missed_log.py` | which boxes go undecoded, and whether it is the same ones every run | `out/missed_boxes.jsonl` |

`missed_log.py` is the one report that accumulates. Every other tool
overwrites its output with the current run; this one appends to a single file
that holds nothing but misses, one line per box per run, so a box that fails
every flight can be told apart from one that failed once. Running it twice on
the same scan changes nothing unless `--force` is given, and past runs can be
folded in with `--inventory docs/runs/<date>/inventory_scanned.json`.

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

Every scored box appears twice in the 3D view: a plain hollow marker at the
position ground truth gives for its label, and the coloured dot the scan
produced, with a line between them. That line is the shift, and it is drawn for
all of them by default - `v` raises the threshold when the picture gets busy.
The same thing as data is `out/position_offsets.csv`, one row per box, worst
first: both positions and the signed difference on each axis. The summary says
how large the error is; the per-axis columns are the only place a systematic
lean can be told apart from scatter.

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

`docs/runs/2026-08-25/` holds one complete run - the three HTML reports, the
JSON they came from, the scanner console and the position track - so the output
can be read without flying for it. `out/` itself is gitignored, since every run
rewrites it.

`out/synthetic_demo/` holds a set of reports built from a fabricated scan, so
the output can be seen without waiting on a 45 minute flight. Nothing in it is
a measurement of anything.

## Barcode, live

Every box carries a CODE128 bay placard under its QR label. `perception/` reads
those placards in its own process, beside a flight or afterwards on saved
frames:

```sh
.venv/bin/pip install pyzbar
.venv/bin/python perception/barcode_scanner.py               # live, with a window
.venv/bin/python perception/barcode_scanner.py --headless    # no window
.venv/bin/python perception/barcode_scanner.py --replay out/frames
```

The window shows the camera stream with every decoded code outlined: green for
a box QR, blue for a placard, and the placard's label says which box it was
tied to. `q` closes it, `s` saves the frame as drawn.

Nothing in `scanner/` is touched, imported or written to. This is a second
subscriber on a camera topic Gazebo is already publishing, so it can be started
and stopped at any point in a flight and a crash in it cannot reach the scan.
It does cost CPU beside the simulator; `--headless` is the cheap way to run it
during a long scan.

A placard payload names a SLOT, not a box: `A0303` is carried by all three
boxes in that bay. Each reading is therefore linked to the QR directly above it
in the same frame, using the label geometry from `warehouse/gen_labels.py`, and
what the tool reports is whether the placard agrees with the slot the QR was
filed under. Results go to `out/barcode_readings.jsonl` (every reading) and
`out/barcode_inventory.json` (one record per box).

## When it does not start

Each of these cost real time, and none of them says what it means.

**`No rule to make target 'gz_x500_c27_warehouse'`** after a build that
otherwise finished. The make targets come from a glob over the airframes
directory, evaluated when CMake configures. Reconfigure:
`cd ~/PX4-Autopilot/build/px4_sitl_default && cmake .`

**CMake configure fails with `add_custom_target cannot create target
"gz_x500_scanner" because another target with the same name already exists`.**
Two airframe files in the PX4 tree reduce to the same model name - the target
name is everything after `_gz_`, so `4022_gz_x500_scanner` and
`4023_gz_x500_scanner` collide and the whole configure aborts, taking every
other target with it. Remove the one nothing installs any more, and delete its
line from `airframes/CMakeLists.txt`. This is what the naming rule above exists
to prevent.

**`PX4 server already running for instance 0`.** A PX4 from an earlier run is
still alive, usually because `scanner.py` never exited and neither did the
simulator around it. `pgrep -af "px4|gz sim"`, then `pkill -9 -f "gz sim"` and
kill the px4 process.

**`commander check` prints FAILED, and MAVSDK reports `is_armable: False`.**
Indoors with GPS disabled there is no global position, so no home position is
ever set, and both of those report a failure on that basis. Neither blocks the
flight: the scan arms through offboard mode and flies. Look at
`Ready for takeoff!` and at whether the vehicle actually leaves the ground,
not at these two.

**`ERROR [param] Parameter SIM_GZ_EN_ODOM not found`** at startup. Harmless on
PX4 v1.16.2, which has no such parameter: `gz_bridge` subscribes to the model's
odometry unconditionally. Visual odometry working is confirmed by
`is_local_position_ok`, not by this line.

**A Gazebo topic reads as empty although it is publishing.** Set
`GZ_IP=127.0.0.1`. Without it discovery is unreliable and a subscriber can miss
topics entirely - a camera at 10 Hz and a barometer at 44 Hz both read as dead
here, which looked exactly like a broken sensor for about an hour.

**`vehicle_imu timestamp error` and `Preflight Fail: No valid data from Baro 0`**
in the first seconds. These clear as the simulator comes up. They matter only
if they keep repeating after `Ready for takeoff!`.

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
