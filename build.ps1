# Windows convenience script: builds the Pico 2 firmware and the host tools/tests.
#   .\build.ps1                 # firmware (pico2) + host + tests
#   .\build.ps1 -Board pico     # RP2040 firmware
param([string]$Board = "pico2", [switch]$SkipHost)
$ErrorActionPreference = "Stop"
if (-not $env:PICO_SDK_PATH) { Write-Warning "PICO_SDK_PATH is not set (the Pico VS Code extension sets it for its terminals)" }
cmake -S firmware -B build -G Ninja -DPICO_BOARD=$Board
cmake --build build
Write-Host "Firmware: build\edge_pq.uf2  (hold BOOTSEL, plug in, copy to the RP2350/RPI-RP2 drive)"
if (-not $SkipHost) {
  cmake -S firmware/host -B build-host -DCMAKE_BUILD_TYPE=Release
  cmake --build build-host --config Release
  ctest --test-dir build-host -C Release --output-on-failure
}
