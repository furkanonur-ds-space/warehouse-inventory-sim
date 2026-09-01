#!/bin/bash
#
# Stop everything the simulation leaves behind, and say how much memory is
# free afterwards.
#
# Run this before every launch. Gazebo's memory grows with every pixel it
# renders and a scan has ended at 26 GB of 27; a simulator that size does not
# always go on the first signal, and starting a second one beside it leaves
# neither with enough memory to work. The symptom is confusing rather than
# obvious: PX4 reports "Accel Sensor 0 missing", "Found 0 compass", and the
# scan never starts, so nothing appears on the terminal at all.
set -u

# pgrep and pkill match the whole command line, including this script's own
# and its parent shell's. Excluded here so that neither is killed nor listed:
# a wait loop written the obvious way found its own shell, which exits at
# once, and reported a scan finished while it was on its third waypoint.
mine="^($$|$PPID)$"

for pat in 'gz sim' 'bin/px4' 'cmake -E env PX4_SIM_MODEL' 'make px4_sitl' \
           'scanner.py' 'barcode_scanner'; do
    for pid in $(pgrep -f "$pat" 2>/dev/null); do
        echo "$pid" | grep -qE "$mine" || kill -9 "$pid" 2>/dev/null
    done
done
sleep 4

echo "still running:"
left=0
for pat in 'gz sim' 'bin/px4' 'scanner.py' 'barcode_scanner'; do
    for pid in $(pgrep -f "$pat" 2>/dev/null); do
        echo "$pid" | grep -qE "$mine" && continue
        tr '\0' ' ' < "/proc/$pid/cmdline" 2>/dev/null | cut -c1-70 \
            | sed "s/^/  $pid  /"
        echo
        left=1
    done
done
[ "$left" -eq 0 ] && echo "  nothing"

echo
free -m | awk '/^Mem:/ {printf "memory: %.1f GB total, %.1f used, %.1f free\n", \
                        $2/1024, $3/1024, $7/1024}'
