# Authors

Two people built this, during a summer internship, working on separate halves
that meet at the warehouse floor plan and the inventory file.

## Furkan Onur - the scanning drone

Everything under `scanner/`.

The flight and the perception that goes with it: planning a route from a floor
plan, holding position indoors with no GPS at all, and reading the codes off
the shelves while moving.

- GPS-free localisation. The estimator is fed external vision instead of a
  satellite fix, proved from the flight's own log rather than asserted, with
  ArUco floor markers correcting drift through a setpoint offset because PX4
  exposes no estimator reset.
- Reading both faces of an aisle in one pass, the forward hires camera ahead
  and the rear tracking camera behind, which halved the route from 48
  waypoints to 24. 428 of 432 codes, 99.1 per cent, nothing on the wrong shelf
  or the wrong level, position along the aisle out by 10 mm at the median.
- Where the vehicle flies, chosen by measuring where each camera stops reading
  rather than by rule of thumb: `standoff_sweep.py`.
- The vehicle model and its sensors, matched to the C27 configuration on the
  hardware: `build_c27_drone.py`, and `build_starling2.py` for the airframe the
  simulation should eventually stand on.
- The layout file, which is the whole of what the scanner knows about a
  warehouse, so that flying a different one is a new file and not an edit.

## Ibrahim Cobankose - the warehouse, the barcode reader, the reporting

Everything under `warehouse/`, `perception/`, `report/` and `scripts/`.

The world the drone flies in, and everything that says how well a run went.

- The warehouse itself: pallet racking, 432 boxes, and the ground truth that
  every score in this project is measured against. `gen_world.py` and
  `gen_labels.py` generate it, so the layout is data rather than a fixed
  scene.
- Reading the CODE128 bay placards live, in its own process, and linking each
  one to the QR sitting above it in the same frame. The placard names a slot
  and not a box, so it cannot replace a QR, but it can say that a slot holds a
  box whose QR did not read, which is what a torn or missing label looks like.
- The reports: coverage, accuracy, drift, the boxes a run missed, a 3D view of
  where each box really is next to where the scan put it, and the flight log
  behind all of them.
- The scoring tool the scanner is held to, `report/validate_inventory.py`.

## Checking this

`git log` is the record; every commit carries its author. Counting lines will
not tell you the same story, because the repository also carries generated
data: the ground truth alone is 24,000 lines and the saved run outputs are
another 110,000, none of it written by hand.
