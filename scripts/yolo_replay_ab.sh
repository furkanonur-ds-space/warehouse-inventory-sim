#!/bin/bash
#
# The same saved frames, read twice: once by zbar alone, once with the YOLO
# locating the labels first. Prints both totals side by side.
#
#   ./scripts/yolo_replay_ab.sh [frame-dir]        default out/barcode_frames
#
# This is the only honest way to ask whether the locator is worth its runtime.
# Both passes get identical pixels, so a difference in the totals is the
# locator and nothing else - not a different flight, not a different world,
# not the frame phase that makes two runs of the same route disagree.
#
# MIND WHICH FRAMES THESE ARE. out/barcode_frames holds the frames where a QR
# read and the barcode beside it did not: the reader's own failures, kept on
# purpose. So it is the worst case for the barcode by construction, and the
# QR column will look unimpressive because every QR in it already read. What
# the run is being asked is how many of ITS OWN misses the locator recovers.
#
# PY picks the interpreter. A replay needs pyzbar and, for the second pass,
# ultralytics; it does NOT need gz-transport, so the dataset's venv is a fine
# place to run it from even though a live flight is not:
#
#   PY=~/Desktop/warehouse_dataset/.venv/bin/python ./scripts/yolo_replay_ab.sh
set -u
HERE="$(cd "$(dirname "$0")/.." && pwd)"
PY="${PY:-$HERE/.venv/bin/python}"
FRAMES="${1:-$HERE/out/barcode_frames}"

if [ ! -x "$PY" ]; then
    echo "no interpreter at $PY; set PY"
    exit 1
fi
if [ ! -d "$FRAMES" ]; then
    echo "no frames at $FRAMES"
    echo "  fly a run with SAVE_FRAMES=1 ./scripts/scan_with_barcode.sh"
    exit 1
fi
echo "frames: $FRAMES  ($(find "$FRAMES" -name '*.png' | wc -l) images)"
echo "python: $PY"
echo

run() {
    local tag="$1"; shift
    "$PY" "$HERE/perception/barcode_scanner.py" --headless \
        --replay "$FRAMES" \
        --readings "$HERE/out/ab_${tag}_readings.jsonl" \
        --summary  "$HERE/out/ab_${tag}.json" \
        "$@" > "$HERE/out/ab_${tag}.log" 2>&1
    local rc=$?
    if [ $rc -ne 0 ]; then
        echo "$tag FAILED (rc $rc), see out/ab_${tag}.log"
        tail -5 "$HERE/out/ab_${tag}.log"
        return $rc
    fi
    echo "== $tag"
    grep -E "barcode readings|boxes consistent|YOLO saw" \
        "$HERE/out/ab_${tag}.log" | sed 's/^/   /'
}

# YOLO_ARGS passes extra flags to the second pass, e.g. to sweep the margin:
#   YOLO_ARGS="--yolo-margin 0.6" ./scripts/yolo_replay_ab.sh
# Unquoted on purpose so several flags split into several words; an empty one
# must expand to nothing at all, which is why it is not "${YOLO_ARGS:-}".
read -r -a extra <<< "${YOLO_ARGS:-}"

run zbar || exit 1
echo
run yolo --yolo ${extra+"${extra[@]}"} || exit 1
echo
echo "logs: out/ab_zbar.log  out/ab_yolo.log"
echo "readings: out/ab_zbar_readings.jsonl  out/ab_yolo_readings.jsonl"
