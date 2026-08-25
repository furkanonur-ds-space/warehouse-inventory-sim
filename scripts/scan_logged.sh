#!/usr/bin/env bash
# Run a scan with everything recorded, so a finished run can be read back
# rather than remembered.
#
#   bash scripts/scan_logged.sh
#
# Writes three things:
#   out/logs/scan_<timestamp>.log   the scanner console: waypoints, DETECTED,
#                                   MARKER, and the closing summary
#   out/logs/latest.log             a link to the most recent of those
#   out/flight_log.csv              where the vehicle really was, 10 Hz
#
# The simulator must already be running in another terminal. Arguments are
# passed through to scanner.py.
#
# scanner.py does not exit by itself once it has landed. It writes the
# inventory and the navigation report, calls land, and the process stays up.
# Watch for "SCAN COMPLETE" and "[INFO] Landing" in the console, then Ctrl-C:
# the results are already on disk by then, and the recorder is stopped by the
# trap below.
#
# GZ_IP is set here on purpose. Without it gz-transport discovery is
# unreliable on this setup: topics that are publishing normally can appear
# empty, which has already sent one diagnosis down the wrong path.
set -uo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$HERE"
PY="$HERE/.venv/bin/python"
export GZ_IP="${GZ_IP:-127.0.0.1}"

mkdir -p out/logs
TS="$(date +%Y%m%d_%H%M%S)"
LOG="out/logs/scan_${TS}.log"

# Point latest.log at this run before it starts, not after it ends. The log is
# most wanted while the flight is still going, and scanner.py does not return
# on its own after it lands: it writes its results, calls land, and the process
# stays up. Anything that waited for the exit would leave nothing to read for
# the forty minutes that matter.
: > "$LOG"
ln -sf "scan_${TS}.log" out/logs/latest.log

# The position recorder runs beside the flight. It only reads one Gazebo
# topic, so it cannot affect what it is recording.
"$PY" report/flight_log.py --quiet &
LOGGER_PID=$!
trap 'kill "$LOGGER_PID" 2>/dev/null' EXIT INT TERM

echo "== scan  log -> $LOG  +  out/flight_log.csv =="
# -u keeps the console unbuffered. Without it the output arrives in blocks and
# a 40 minute flight shows nothing until it ends.
(cd scanner && "$PY" -u scanner.py "$@") 2>&1 | tee "$LOG"

echo
echo "== done. console: out/logs/latest.log   track: out/flight_log.csv =="
