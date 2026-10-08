#!/bin/bash
# Can this machine reach the board, and by which road?
#
#   ./scripts/check_link.sh 192.168.101.2
#
# Worth running before Gazebo, because a bridge with nobody answering and a
# bridge with a wrong address look the same from here: sensors leaving at
# 250 Hz and nothing coming back.
#
# It checks the network only. Whether PX4 on the board is running in HITL
# mode is a different question, answered by the bridge's own report once it
# starts: '| back: actuators' above 0 Hz.
ADDR="${1:-192.168.101.2}"

echo "== which interface would carry it"
ip route get "$ADDR" 2>&1 | head -1

echo
echo "== ping"
if ping -c 3 -W 2 "$ADDR"; then
  echo
  echo "OK the board answers at $ADDR"
  exit 0
fi

echo
echo "FAIL no answer from $ADDR"
echo
echo "Things to check, most likely first:"
echo "  1. WSL has to share Windows' network to see the USB link at all."
echo "     C:\\Users\\<you>\\.wslconfig needs, under [wsl2]:"
echo "         networkingMode=mirrored"
echo "     then 'wsl --shutdown' from Windows PowerShell."
echo "  2. Is the board up and on USB? 'adb devices' should list it."
echo "  3. Is that its address? 'adb shell ip -brief addr' shows it."
echo "  4. Windows firewall may drop it; try the ping from PowerShell too."
exit 1
