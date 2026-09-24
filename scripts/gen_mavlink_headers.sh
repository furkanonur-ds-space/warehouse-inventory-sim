#!/bin/bash
# Generate the MAVLink C headers this plugin includes.
#
# They are generated rather than committed: they are 7 MB of machine output
# and they belong to the MAVLink project, not to this one. Nothing is
# installed system wide and no sudo is needed.
set -e
cd "$(dirname "$0")/.."
ROOT="$(pwd)"

MAVLINK_SRC="${MAVLINK_SRC:-$HOME/mavlink}"
PYTHON="${PYTHON:-$HOME/autonomous_landing/venv/bin/python}"

if [ ! -d "$MAVLINK_SRC" ]; then
  echo "cloning mavlink into $MAVLINK_SRC"
  git clone --depth 1 --recursive https://github.com/mavlink/mavlink.git "$MAVLINK_SRC"
fi

mkdir -p "$ROOT/third_party/mavlink"
cd "$MAVLINK_SRC"
"$PYTHON" -m pymavlink.tools.mavgen \
  --lang=C --wire-protocol=2.0 \
  --output="$ROOT/third_party/mavlink" \
  message_definitions/v1.0/common.xml

echo "headers in $ROOT/third_party/mavlink"
