#!/usr/bin/env sh
# Linux/macOS convenience script: firmware + host tools + tests.
set -e
BOARD=${1:-pico2}
cmake -S firmware -B build -DPICO_BOARD="$BOARD"
cmake --build build -j
cmake -S firmware/host -B build-host -DCMAKE_BUILD_TYPE=Release
cmake --build build-host -j
ctest --test-dir build-host --output-on-failure
echo "Firmware: build/edge_pq.uf2"
