#!/bin/bash
# Build the route tool. No sudo, nothing installed system wide.
set -e
cd "$(dirname "$0")/.."
mkdir -p build
cc -std=c11 -Wall -Wextra -O2 \
   -o build/route_tool \
   main.c route.c third_party/cJSON.c \
   -lm
echo "built: $(pwd)/build/route_tool"
