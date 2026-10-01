# Edge-Intelligent Power Quality & Grid Disturbance Analysis — firmware stack

Firmware for a **Raspberry Pi Pico 2 (RP2350)** power-quality analyser with an external
24-bit SPI ADC (TI **ADS131M02**, 2 simultaneous channels: voltage + current), plus the
PC-side tools, ML training pipeline, simulator and tests. The same firmware also builds
for the original Pico (RP2040).

```
 ADS131M02 ──SPI/PIO──DMA ring──► calibration ─► DSP (RMS · frequency · FFT · THD · wavelet)
 (or simulator)                              ─► PQ detection ─► 14 features ─► TinyML (int8 MLP)
                                             ─► decision fusion ─► events · metrics · control refs
      core 1: everything above (hard real time)      core 0: USB/UART protocol, log, alarms, flash
```

**What is in the box**

| Path | Content |
|---|---|
| `firmware/` | Pico SDK project (`CMakeLists.txt`) → `edge_pq.uf2` |
| `firmware/src/` | firmware sources — see [docs/architecture.md](docs/architecture.md) |
| `firmware/host/` | PC build: unit/integration tests, `pq_sim` (whole firmware on a PC), `pq_datagen`, `pq_bench` |
| `firmware/tests/` | 65 unit + integration tests, PIO cycle-level test |
| `ml/` | dataset → training → int8 quantisation → `model_data.cpp` export |
| `tools/` | `serial_monitor.py`, `live_plot.py`, `data_logger.py`, `protocol.py`, `test_tools.py` |
| `docs/` | architecture, DSP, TinyML, protocol, configuration, testing, performance, hardware interface |
| `prebuilt/` | `edge_pq_pico2.uf2`, `edge_pq_pico.uf2` built from this tree (simulation mode at boot) |

## What was verified, and what was not

Verified in the build environment:

- The Pico 2 and Pico images compile and link with Pico SDK 2.2.0 and arm-none-eabi-gcc 13.2.
- All 65 host tests pass. They cover ADC conversion, the ADS131M02 frame/CRC protocol, filters, RMS, frequency, FFT, THD, wavelet, transients, every disturbance class, features, fusion, the ML model, the protocol, config, control outputs, the logger, and full end-to-end sessions over the serial protocol.
- The embedded TFLM model output matches the Python TFLite interpreter bit for bit.
- The PIO SPI program was checked cycle by cycle in a PIO emulator against a model of the ADC.
- The Windows host build compiles with MinGW.

Not verified: nothing has run on a physical Pico or ADC. On-target timing is measured by the firmware itself — run `BENCH` on your board (see [docs/performance.md](docs/performance.md)). No invented numbers are given anywhere. The ML accuracy figures come from **simulated** disturbances, so validate the model with recordings from your own hardware.

## Quick start on Windows

### 1. Tools (once)

1. **Pico toolchain.** The easiest route is the official *Raspberry Pi Pico* extension for VS Code. It installs the Pico SDK (use 2.1.1 or newer), the Arm GNU toolchain, CMake, Ninja and picotool under `%USERPROFILE%\.pico-sdk`. For a manual install you need:
   - Arm GNU Toolchain `arm-none-eabi` 13.x (add its `bin` to PATH)
   - CMake ≥ 3.16 and Ninja
   - Git
   - the SDK: `git clone -b 2.2.0 https://github.com/raspberrypi/pico-sdk` then `git submodule update --init`, and `setx PICO_SDK_PATH C:\path\to\pico-sdk`.
2. **Host compiler**, for the tests, the simulator and ML dataset generation. Use either Visual Studio 2022 Build Tools (C++ workload) or MSYS2 UCRT64 (`pacman -S mingw-w64-ucrt-x86_64-gcc mingw-w64-ucrt-x86_64-cmake mingw-w64-ucrt-x86_64-ninja`).
3. **Python 3.10+**: `pip install -r ml/requirements.txt`. This installs TensorFlow, pyserial and matplotlib. Tensorflow is only needed if you retrain.

TensorFlow Lite Micro is fetched automatically by CMake from `raspberrypi/pico-tflmicro` at a pinned commit. For offline builds, clone it once and pass `-DPQ_TFLM_DIR=C:\path\to\pico-tflmicro`.

### 2. Build the firmware (.uf2)

In a "Developer PowerShell" (or any shell with the toolchain on PATH):

```powershell
cd edge-pq
cmake -S firmware -B build -G Ninja -DPICO_BOARD=pico2      # -DPICO_BOARD=pico for RP2040
cmake --build build
# -> build\edge_pq.uf2   (size report printed at the end)
```

Or simply run `.\build.ps1` (firmware + host tests; `.\build.ps1 -Board pico` for RP2040).

Useful options: `-DPQ_DEFAULT_SIMULATION=0` boots on the real ADC. `-DPQ_ENABLE_TFLM=OFF` builds a DSP-only image. `-DPQ_USE_MCP4822=ON` uses an external DAC for the reference outputs. See the top of `firmware/CMakeLists.txt` for the full list.

### 3. Flash

1. Hold **BOOTSEL** and plug in the Pico 2. It mounts as the drive `RP2350`.
2. Copy `build\edge_pq.uf2` onto that drive. The board reboots and enumerates as a USB serial port (COMx).

Alternatively, run `picotool load -x build\edge_pq.uf2` while the board is in BOOTSEL mode.

### 4. Watch it work (no ADC needed; the image boots in simulation mode)

```powershell
python tools\serial_monitor.py --port COM5 --demo      # live metrics + injected sag/swell/... events
python tools\live_plot.py --port COM5 --demo --wave
python tools\data_logger.py --port COM5 --out runs --duration 120
```

In the monitor you can type commands: `STATUS`, `BENCH`, `EVENTS TEXT`, `SIM sag 0.6 0.2`, `HELP`, and so on.
When the hardware is ready, send `SOURCE ADC` and then `CONFIG SAVE`. Calibrate with `CALIBRATE OFFSET`, then `CALIBRATE VGAIN 230`, `CALIBRATE IGAIN <A>`, and `CALIBRATE PHASE` (see [docs/configuration.md](docs/configuration.md)).

### 5. Host build: tests + simulator (no Pico at all)

```powershell
cmake -S firmware\host -B build-host -DCMAKE_BUILD_TYPE=Release
cmake --build build-host --config Release
ctest --test-dir build-host -C Release --output-on-failure
python tools\serial_monitor.py --port sim --demo          # the whole firmware running on your PC
python tools\test_tools.py                                 # end-to-end check of the PC tools
```

On Linux or macOS the commands are the same. `pq_sim --pty` exposes a pseudo-terminal that behaves like the Pico's serial port.

### 6. Retrain the TinyML model (optional)

```powershell
build-host\Release\pq_datagen --clips 1200 --seed 1   --out data\train.csv
build-host\Release\pq_datagen --clips 300  --seed 777 --out data\test.csv
python ml\train_model.py --train data\train.csv --test data\test.csv     # writes firmware\src\tinyml\model_data.*
cmake --build build        # rebuild the .uf2 with the new model
```

The feature vector is defined once, in `firmware/src/features/feature_spec.h`. The firmware refuses to load a model whose feature-spec hash differs from its own, so training and inference cannot drift apart silently.

## Safety

The firmware produces **low-voltage logic and reference signals only**: trip request, heartbeat, event code, PWM/DAC references and a UART reference link. All actuating outputs are disabled until you send `CONTROL ARM`. Isolation, protection and the power stage are the hardware design's responsibility, and any SSCB, DVR or APF must have its own independent protection.
