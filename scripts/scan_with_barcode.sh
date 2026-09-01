#!/bin/bash
#
# Fly a scan with the barcode reader alongside it, from one terminal.
#
#   ./scripts/scan_with_barcode.sh            rear camera only  (default)
#   ./scripts/scan_with_barcode.sh front      front camera only
#   ./scripts/scan_with_barcode.sh both       both cameras
#   ./scripts/scan_with_barcode.sh none       no barcode, just the scan
#
# Why rear by default. The barcode reader is a second subscriber on a camera
# topic, so Gazebo serialises and sends every frame twice and the run loses a
# few per cent of them. Measured: 3173 hires frames without it and 3067 with,
# which cost about two codes out of 432.
#
# That price is worth paying on the rear camera and not on the front one. Over
# five runs the forward hires camera has read 216 of 216 every time, so a
# barcode there confirms what is already known. Every miss has been on a face
# the rear camera reads, and those are the slots where knowing that a box is
# present, even without reading its QR, is the whole point.
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
MODE="${1:-rear}"

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
echo "  $PY $HERE/report/barcode_vs_qr.py barcode_readings_$MODE.jsonl"
exit $status
