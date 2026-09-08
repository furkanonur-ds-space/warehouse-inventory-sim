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
# The barcode names the BOX, not the slot it stands in, and has since it
# started carrying the box's own four-digit number. So it can fill a hole in
# the inventory: a barcode read where the QR failed is a box recovered, and
# report/barcode_inventory.py files those as an inventory of their own, scored
# against the barcode labels in ground truth rather than against the QR ones.
#
# SAVE_FRAMES=1 keeps the frames where a QR read and the barcode beside it did
# not, lossless, in out/barcode_frames. That is the only way to ask a question
# about the LABEL on real pixels, and it is a few hundred frames rather than a
# recording of the whole flight:
#
#   SAVE_FRAMES=1 ./scripts/scan_with_barcode.sh
#   .venv/bin/python perception/barcode_scanner.py --headless \
#       --replay out/barcode_frames \
#       --readings out/replay_readings.jsonl --summary out/replay.json
#
# NOT RECORD_VIDEO=1. That is the scanner's own switch and it records both
# cameras through two cv2.VideoWriter threads; on 2026-09-04 it took the scan
# down with `corrupted double-linked list` at waypoint 11 of 24, half way up
# the second aisle. It is in scanner/ and not ours to fix. The videos it had
# written to that point were intact, so it is the writing that is unsafe and
# not the files, but a run that dies half way is a run.
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
# Clear the last run's readings first. out/ is rewritten by every run, but
# only under the names that run uses: a file from a run with a different set
# of cameras survives and gets merged into this run's totals by anything that
# reads them all.
rm -f "$HERE"/out/barcode_readings*.jsonl "$HERE"/out/barcode_inventory*.json
pids=()

SAVE_FRAMES="${SAVE_FRAMES:-0}"
SHOTS="$HERE/out/barcode_frames"
if [ "$SAVE_FRAMES" != "0" ]; then
    # Same reason the readings are cleared: frames from an older run would be
    # replayed as if they belonged to this one.
    rm -rf "$SHOTS"
    echo "  keeping unread-barcode frames in out/barcode_frames"
fi

start_barcode() {
    local link="$1" tag="$2"
    local shots=()
    if [ "$SAVE_FRAMES" != "0" ]; then
        shots=(--save-frames "$SHOTS")
    fi
    "$PY" "$HERE/perception/barcode_scanner.py" --headless \
        --topic "$BASE/$link/sensor/camera/image" \
        --readings "$HERE/out/barcode_readings_$tag.jsonl" \
        --summary  "$HERE/out/barcode_inventory_$tag.json" \
        ${shots+"${shots[@]}"} \
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

# GZ_IP for the same reason scan_logged.sh sets it: without it gz-transport
# discovery is unreliable here and a camera publishing at 10 Hz can read as
# dead, which has already sent one diagnosis down the wrong path. The barcode
# reader sets it for itself; the scan was not getting it at all.
export GZ_IP="${GZ_IP:-127.0.0.1}"

mkdir -p "$HERE/out/logs"
TS="$(date +%Y%m%d_%H%M%S)"
LOG="$HERE/out/logs/scan_${TS}.log"
: > "$LOG"
ln -sf "scan_${TS}.log" "$HERE/out/logs/latest.log"

echo "  scanning ...  console -> out/logs/latest.log"
# -u keeps the console unbuffered. Without it a forty minute flight prints
# nothing until it ends, which is also nothing to read if it dies first.
(cd "$HERE/scanner" && "$PY" -u scanner.py) 2>&1 | tee "$LOG"
status=${PIPESTATUS[0]}

cleanup
sleep 1
echo
echo "compare what the barcodes added:"
# No file name: the report reads every readings file the run left, which is
# one per camera. Naming one here was wrong for "both", where the files are
# tagged front and rear and nothing is called barcode_readings_both.jsonl.
echo "  $PY $HERE/report/barcode_vs_qr.py"
exit $status
