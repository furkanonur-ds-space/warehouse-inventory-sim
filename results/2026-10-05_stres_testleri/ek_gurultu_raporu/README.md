# Camera noise in the dim warehouse, 2026-10-05

Three flights over the same dimmed warehouse (`lights_stress:` on: ambient
x0.2, lamps x0.5, 8 of 16 lamps dark), with the cameras made noisier each
time. The question: the dim flight read everything, even at a fifth of the
clean world's light, because the simulated camera has no noise. Does that
still hold once it has some?

Short answer: no. With noise of about 14 grey levels the barcodes on the
darkest labels stop reading entirely, and reads fall in order of light. Nothing
was misread, ever: noise costs missed codes, not wrong ones.

## The flights

| Folder | Camera noise setting | Noise measured in the frames | 3D views |
|---|---|---|---|
| `dim/` | none | 0.1 grey levels | `../06_los_isik` |
| `noise03/` | `--camera-noise 0.03` | ~1.5 grey levels | `../07_los_hafif_gurultu` |
| `noise15/` | `--camera-noise 0.15` | ~14 grey levels | `../08_los_guclu_gurultu` |

All three: both cameras reading, YOLO locator on (`YOLO=1`), `nvidia`, on
mains power. Same world, same route, 24/24 waypoints each.

**The setting is not the noise.** Gazebo's camera noise (`<noise>` in the
camera, gz-rendering 8 / ogre2) came out far weaker than its stddev and not
Gaussian: at 0.03, 74% of pixels did not move at all and a few jumped by up to
56 levels; at 0.15, two thirds of the pixels move and the spread is ~14 levels.
Five times the setting gave nine times the noise. The middle column is
measured from the saved frames (residual against a 3x3 median, in flat areas
only), and it is the number to quote - not the setting.

How the noise was put in: `scanner/build_c27_drone.py --camera-noise SIGMA`
adds Gazebo's noise to every camera of the vehicle, ArUco reader included;
`scripts/launch_sim.sh` refuses to fly a noisy model unless `CAMERA_NOISE=1`.
Rebuild without the flag for clean cameras.

## What it measured

| | dim, no noise | ~1.5 levels | ~14 levels |
|---|---|---|---|
| QR (inventory) | 416 / 416 | 416 / 416 | **231 / 416** |
| Barcode | 416 / 416 | 416 / 416 | **253 / 416** |
| Cartons found | 419 / 432 | 427 / 432 | 376 / 432 |
| Misreads (wrong face, level, bay, unknown payload) | 0 | 0 | **0** |
| QR position error, median / max | 11 / 78 mm | 20 / 67 mm | 10 / 50 mm |
| Readings only the YOLO crop pass got | 15 | 9 | **354** |
| Marker fixes / max drift at a fix | 334 / 1.38 m | 351 / 1.41 m | 325 / 1.50 m |
| Clearance alarms (min distance, threshold 0.2 m) | 0 (0.208) | 0 (0.207) | **5 (0.189)** |

~1.5 grey levels costs nothing at all. ~14 costs about 45% of the QRs and 40%
of the barcodes.

### By light: darkness matters once there is noise

Barcodes read, by the light each label got (the generator's light model,
`light.E` in ground truth; see `report/light_report.py`):

| Light E | dim, no noise | ~14 levels |
|---|---|---|
| 0 - 0.25 | 69 / 69 | **0 / 69** |
| 0.25 - 0.5 | 41 / 41 | 14 / 41 (34%) |
| 0.5 - 1 | 115 / 115 | 87 / 115 (76%) |
| 1 - 2 | 184 / 184 | 145 / 184 (79%) |
| 2 - 4 | 7 / 7 | 7 / 7 |

This is what the dim flight could not show. Without noise a dark label still
has its grey levels and reads; with noise its contrast drowns. The QRs do not
fall as neatly by band - the darkest band is mostly face G, where the camera
is 0.25 m from the labels and the modules are large (G read 43 of 52 QRs) -
and they are also hit by the decoder load below.

## Caveat: part of the loss is the decoder, not the camera

zbar takes about three times longer on a noisy frame, and both readers fell
behind the camera:

| Frames decoded / dropped | dim | ~1.5 levels | ~14 levels |
|---|---|---|---|
| `scanner.py` (QR), hires | 5242 / 1427 | 3528 / 2977 | **856 / 5236** |
| `scanner.py` (QR), rear | 2534 / 134 | 1845 / 758 | **397 / 2041** |
| `barcode_scanner.py`, front (shed) | 24 of 7424 | 97 of 9055 | **3481 of 10239** |
| `barcode_scanner.py`, rear (shed) | 5 of 2955 | 5 of 3605 | 4 of 4105 |

In the noisiest flight the QR scanner (`scanner/`) decoded only 14% of the
hires frames, so the QR figure of 231 measures the CPU as much as the camera.
The cleanest single number is the rear camera's barcode reader, which kept
up the whole flight: **123 of 208** barcodes on its faces, against 208 of 208
without noise. That is noise alone.

The raw frames of the noisiest flight were deleted afterwards to save
space, so the replay that would strip the decoder loss out of the front
camera can no longer be done on this flight.

## Files

Each folder holds that flight's reports as written by `make_reports.sh`, the
two barcode readers' summaries, and the scanner console:
`light_report.json`, `validation_report.json` (QR), `validation_report_barcode.json`,
`navigation_report.json`, `drift_report.json`, `yolo_report.json`,
`barcode_inventory_{front,rear}.json`, `scan_console.log`.

The 3D views of each flight are in the sibling folders named above; the
rest of each archive was deleted. The full write-up of every stress flight
is `../stres_testleri_raporu.docx`.
