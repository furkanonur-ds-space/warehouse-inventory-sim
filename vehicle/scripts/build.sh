#!/bin/bash
# Build the tools. No sudo, nothing installed system wide.
set -e
cd "$(dirname "$0")/.."
mkdir -p build

cc -std=c11 -Wall -Wextra -O2 \
   -o build/route_tool \
   main.c route.c third_party/cJSON.c \
   -lm

cc -std=c11 -Wall -Wextra -O2 \
   -o build/flight_tool \
   flight_tool.c flight.c third_party/cJSON.c \
   -lm

echo "built: $(pwd)/build/route_tool"
echo "built: $(pwd)/build/flight_tool"
