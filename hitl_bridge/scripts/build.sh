#!/bin/bash
# Build the bridge plugin. No sudo, nothing installed system wide.
set -e
cd "$(dirname "$0")/.."
mkdir -p build
cd build
cmake .. > /dev/null
make -j"$(nproc)"
echo
echo "built: $(pwd)/libhitl_bridge.so"
