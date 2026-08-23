#!/bin/bash
# Install this project into a PX4 tree: the warehouse world, its assets, the
# C27 vehicle model and the airframe.
#
# Everything this writes is named for the C27 build (x500_c27,
# 4023_gz_x500_c27). It deliberately does not touch x500_scanner or
# 4022_gz_x500_scanner, which belong to the other half of the project. Sharing
# those names is what let one side's setup overwrite the other's.
#
# Safe to re-run.
set -e

PX4="${PX4_DIR:-$HOME/PX4-Autopilot}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PY="${PYTHON:-$HOME/autonomous_landing/venv/bin/python}"

GEN="$HERE/warehouse/generated"
PX4_MODELS="$PX4/Tools/simulation/gz/models"
PX4_WORLDS="$PX4/Tools/simulation/gz/worlds"
AIRFRAMES="$PX4/ROMFS/px4fmu_common/init.d-posix/airframes"

[ -d "$PX4" ] || { echo "PX4 tree not found at $PX4"; exit 1; }

echo "== 1/4  generating the warehouse world =="
"$PY" "$HERE/warehouse/gen_world.py" \
      --config "$HERE/warehouse/warehouse.yaml" \
      --out "$GEN" >/dev/null
echo "   world and $(ls "$GEN/gz/models/warehouse_assets/materials/textures" | wc -l) textures generated"

echo "== 2/4  installing world and assets into PX4 =="
mkdir -p "$PX4_WORLDS" "$PX4_MODELS"
cp "$GEN/gz/worlds/warehouse.sdf" "$PX4_WORLDS/"
rm -rf "$PX4_MODELS/warehouse_assets"
cp -a "$GEN/gz/models/warehouse_assets" "$PX4_MODELS/"
echo "   $PX4_WORLDS/warehouse.sdf"

echo "== 3/4  building the C27 vehicle model =="
"$PY" "$HERE/scanner/build_c27_drone.py" >/dev/null
echo "   $PX4_MODELS/x500_c27"

echo "== 4/4  installing the airframe =="
cp "$HERE/scanner/px4_config/4023_gz_x500_c27" "$AIRFRAMES/"
chmod +x "$AIRFRAMES/4023_gz_x500_c27"
if grep -q "4023_gz_x500_c27" "$AIRFRAMES/CMakeLists.txt"; then
    echo "   already registered in CMakeLists.txt"
else
    # Register it next to the other gz airframes, keeping the list sorted.
    sed -i "s/^\(\s*\)4022_gz_x500_scanner$/&\n\14023_gz_x500_c27/" \
        "$AIRFRAMES/CMakeLists.txt"
    grep -q "4023_gz_x500_c27" "$AIRFRAMES/CMakeLists.txt" \
        || { echo "   could not register: add 4023_gz_x500_c27 to CMakeLists.txt by hand"; exit 1; }
    echo "   registered in CMakeLists.txt"
fi

cat <<'EOF'

Done. The airframe is compiled into ROMFS, so PX4 rebuilds on the next run.

  cd ~/PX4-Autopilot
  export HEADLESS=1
  export PX4_GZ_MODEL_POSE="-8.5,-9,0.30,0,0,0"
  make px4_sitl gz_x500_c27_warehouse

then, in a second terminal:

  cd ~/warehouse-inventory-sim/scanner
  ~/autonomous_landing/venv/bin/python scanner.py
EOF
