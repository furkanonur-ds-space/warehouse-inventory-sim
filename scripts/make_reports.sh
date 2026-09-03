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
# One line per flight in a spreadsheet. Everything it writes is already in
# out/ and all of it is overwritten by the next scan, which is the wrong way
# round: the console logs survive and the numbers do not. Same rule as the
# miss log, a run is added once.
run report/run_log.py

echo
if [ "$failed" -eq 0 ]; then
  echo "== all reports written to out/ =="
else
  echo "== $failed report(s) failed =="
fi
exit "$failed"
