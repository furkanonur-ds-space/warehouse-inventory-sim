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

for pat in 'gz sim' 'bin/px4' 'cmake -E env PX4_SIM_MODEL' 'make px4_sitl' \
           'scanner.py' 'barcode_scanner'; do
    pkill -9 -f "$pat" 2>/dev/null
done
sleep 4

echo "still running:"
left=0
for pat in 'gz sim' 'bin/px4' 'scanner.py' 'barcode_scanner'; do
    if pgrep -f "$pat" >/dev/null 2>&1; then
        pgrep -af "$pat" | sed 's/^/  /'
        left=1
    fi
done
[ "$left" -eq 0 ] && echo "  nothing"

echo
free -m | awk '/^Mem:/ {printf "memory: %.1f GB total, %.1f used, %.1f free\n", \
                        $2/1024, $3/1024, $7/1024}'
