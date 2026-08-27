# Reference run, 2026-08-25

One complete scan, kept whole so the reports can be read without flying. Open
the three HTML files directly in a browser; they are self-contained and need no
network.

Everything here is generated. `out/` is gitignored because these files are
rewritten by every run; this copy exists so a person can see what a finished
scan looks like without waiting forty minutes for one.

| File | What it is |
|---|---|
| `inventory_3d.html` | the warehouse in 3D: every product coloured by its position error, the ten boxes that were never decoded drawn at their real dimensions, and the drift measured at each floor marker |
| `coverage.html` | which parts of the warehouse the run covered, down to the bay |
| `drift.html` | the drift at every marker fix, over the run and by marker |
| `inventory_scanned.json` | what the scanner wrote: 422 items and 607 marker corrections |
| `navigation_report.json` | how it flew |
| `validation_report.json`, `coverage_report.json`, `drift_report.json` | the same reports as data |
| `console.log` | the scanner console, 2062 lines |
| `flight_log.csv` | where the vehicle really was, 10 Hz, 15000 samples |

## What this run measured

| Metric | Value |
|---|---|
| Box QR codes decoded | 422 / 432 (97.7%) |
| Waypoints reached | 48 / 48 |
| Inventory accuracy | 100% - no wrong face, level or bay, no duplicates |
| Position error, median / p95 / max | 0.066 / 0.080 / 0.087 m |
| Drift at marker fix, median / p95 / max | 0.035 / 0.218 / 0.414 m |
| Final drift offset | 0.030 m |
| Direction bias | 0.003 m, so no systematic lean |
| Markers seen | 8 / 8, 607 fixes |
| Duration | 1455 s |

Position error is separate from inventory accuracy on purpose: every one of the
422 records landed in its own bay, so nothing was misfiled, and the residual
centimetres are the gap between a flight altitude and where a label sits inside
its level.

## The misses are worth a look

Ten boxes were never decoded, and the properties are in the 3D view and in
`validate_inventory.py --list-missed`. None of them is explained by geometry:
all ten sat within 5 cm of the optical axis against a 23 cm half-frame, and all
ten had 3.88 pixels per QR module against a threshold of 3.

Two patterns:

- `D-06-L1`, `F-02-L1` and `F-02-L3` were also missed by the two runs before
  this one. Three runs, same three boxes.
- Five of the ten are on face E, which read 100% in both earlier runs. The
  markers at the ends of that aisle, 5 and 6, recorded this run's worst drift -
  0.413 and 0.414 m against 0.24 to 0.29 m at the other six. The miss cluster
  and the drift excursion are in the same aisle. That is a correlation and not
  yet an explanation.

This run was flown with the Gazebo window open; the two before it were
headless. Repeating it headless is the way to find out whether that mattered.
