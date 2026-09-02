#!/bin/bash
#
# Fly a scan with the barcode reader alongside it, from one terminal.
#
#   ./scripts/scan_with_barcode.sh            both cameras      (default)
#   ./scripts/scan_with_barcode.sh rear       rear camera only
#   ./scripts/scan_with_barcode.sh front      front camera only
#   ./scripts/scan_with_barcode.sh none       no barcode, just the scan
#
# Why both. The route reads one face of an aisle with the hires camera and the
# other with the rear one, so a reader on a single camera has nothing to say
# about half the warehouse: a slot on a face it never sees cannot be reported
# as occupied whatever its QR did.
#
# This used to default to the rear camera alone, on the grounds that the front
# one had read 216 of 216 over five runs and every miss lay on a face the rear
# camera reads. That is no longer what the log says. Across the eight runs in
# out/missed_boxes.jsonl the misses split 100 on faces the hires reads against
# 115 on the rear ones - A, C, E and G against B, D, F and H - so the front
# camera is now exactly as worth covering as the back.
#
# It is not free. The reader is a second subscriber on a camera topic, so
# Gazebo serialises and sends every frame twice and the run loses a few per
# cent of them: 3173 hires frames without it against 3067 with, about two
# codes out of 432. Paying it on both cameras is the point of the mode, but
# "rear" and "front" are still there for a run that cannot spare the frames.
#
# The barcode names a slot and not a box, so it can never fill a hole in the
# inventory. What it can say is that a slot holds a box whose QR did not read,
# which is what a torn or missing label looks like from the air.
set -u
HERE="$(cd "$(dirname "$0")/.." && pwd)"
PY="${PY:-$HOME/autonomous_landing/venv/bin/python}"
if [ ! -x "$PY" ]; then
    PY="$HERE/.venv/bin/python"
fi
if [ ! -x "$PY" ]; then
    echo "no interpreter; set PY, for example"
    echo "  PY=~/autonomous_landing/venv/bin/python $0"
    exit 1
fi
MODE="${1:-both}"

LAYOUT="$HERE/scanner/layout.json"
WORLD=$("$PY" -c "import json,sys; print(json.load(open(sys.argv[1]))['world'])" "$LAYOUT")
MODEL=$("$PY" -c "import json,sys; print(json.load(open(sys.argv[1]))['model'])" "$LAYOUT")
BASE="/world/$WORLD/model/${MODEL}_0/link"

mkdir -p "$HERE/out"
pids=()

start_barcode() {
    local link="$1" tag="$2"
    "$PY" "$HERE/perception/barcode_scanner.py" --headless \
        --topic "$BASE/$link/sensor/camera/image" \
        --readings "$HERE/out/barcode_readings_$tag.jsonl" \
        --summary  "$HERE/out/barcode_inventory_$tag.json" \
        > "$HERE/out/barcode_$tag.log" 2>&1 &
    pids+=($!)
    echo "  barcode on $tag -> out/barcode_readings_$tag.jsonl (pid ${pids[-1]})"
}

case "$MODE" in
    rear)  start_barcode camera_track_rear_link rear ;;
    front) start_barcode camera_hires_link front ;;
    both)  start_barcode camera_hires_link front
           start_barcode camera_track_rear_link rear ;;
    none)  echo "  no barcode reader" ;;
    *)     echo "unknown mode: $MODE (rear, front, both, none)"; exit 1 ;;
esac

# Stop the readers whatever happens to the scan, including a ctrl-c.
cleanup() {
    for p in ${pids+"${pids[@]}"}; do kill "$p" 2>/dev/null; done
}
trap cleanup EXIT INT TERM

echo "  scanning ..."
cd "$HERE/scanner" && "$PY" scanner.py
status=$?

cleanup
sleep 1
echo
echo "compare what the barcodes added:"
# No file name: the report reads every readings file the run left, which is
# one per camera. Naming one here was wrong for "both", where the files are
# tagged front and rear and nothing is called barcode_readings_both.jsonl.
echo "  $PY $HERE/report/barcode_vs_qr.py"
exit $status
