#!/usr/bin/env bash
# Every report for the run that is currently in out/, in one command.
#
#   bash scripts/make_reports.sh
#
# Run it after a scan has landed and scan_logged.sh has been stopped. It only
# reads what the flight left in out/; no simulator, no ROS, nothing that can
# reach a running scan.
#
# The reason this exists: the reports were a list of four commands pasted by
# hand, and the fifth one added later kept being left off. A list that has to
# be remembered is a list that goes stale.
#
# Each tool is run even if the one before it failed, because they answer
# different questions and a missing marker fix should not cost the coverage
# report. The exit status is the number of tools that failed.
set -u

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$HERE"
# The project local virtualenv by default, but let the environment say
# otherwise: the two machines this runs on keep theirs in different
# places, and hardcoding one of them makes the script unusable on the
# other.
PY="${PY:-$HOME/autonomous_landing/venv/bin/python}"
if [ ! -x "$PY" ]; then
    PY="$HERE/.venv/bin/python"
fi
if [ ! -x "$PY" ]; then
    echo "no interpreter; set PY, for example"
    echo "  PY=~/autonomous_landing/venv/bin/python $0"
    exit 1
fi

failed=0
run() {
  echo
  echo "== $* =="
  "$PY" "$@" || { echo "!! failed: $*"; failed=$((failed + 1)); }
}

run report/validate_inventory.py --list-worst 5 --list-missed
run report/coverage_report.py --html out/coverage.html
run report/view_inventory.py
run report/drift_report.py --html out/drift.html
# Appends this run to the one log that accumulates across flights. Safe to
# repeat: a scan already in the log is not added twice.
run report/missed_log.py

# The barcode half, when the run flew with a reader. A box carries two labels
# that say different things, so a run produces two inventories and each is
# scored against its own truth - the same tool, --code-type apart. Neither
# label stands in for the other anywhere in here.
#
# missed_log is deliberately not run over it. That log accumulates across
# flights to tell a box that fails every run from one that failed once, and
# feeding it a second inventory per flight would count every run twice.
if ls out/barcode_readings*.jsonl >/dev/null 2>&1; then
  echo
  echo "-- barcode --"
  run report/barcode_inventory.py
  run report/validate_inventory.py --inventory out/inventory_barcode.json \
      --code-type box_placard \
      --out out/validation_report_barcode.json \
      --offsets out/position_offsets_barcode.csv --list-missed
  run report/coverage_report.py --inventory out/inventory_barcode.json \
      --code-type box_placard \
      --json out/coverage_report_barcode.json \
      --html out/coverage_barcode.html
  run report/view_inventory.py --inventory out/inventory_barcode.json \
      --code-type box_placard --out out/inventory_3d_barcode.html
  run report/barcode_vs_qr.py
fi

echo
if [ "$failed" -eq 0 ]; then
  echo "== all reports written to out/ =="
else
  echo "== $failed report(s) failed =="
fi
exit "$failed"
