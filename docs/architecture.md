# Firmware architecture

## Design goals

1. **Deterministic acquisition.** No sample is ever lost to software latency. The ADC is clocked by hardware, frames are moved by PIO and DMA, and the CPU only processes complete blocks.
2. **DSP never blocks acquisition.** Real-time processing runs alone on **core 1**. Everything that can wait (USB, flash, logging, benchmarks) runs on **core 0**.
3. **One code base, two platforms.** Everything above a thin platform layer is plain C++17 with no SDK dependency. The same objects run on the Pico, in the unit tests, in the PC simulator (`pq_sim`) and in the ML dataset generator. That last point is why training features and embedded features are identical.
4. **Fail loudly.** Every data-quality problem is counted, flagged on the affected frames, and reported in `STATUS`.

## Directory layout (as built)

```
firmware/
├── CMakeLists.txt            Pico SDK build -> edge_pq.uf2
├── pico_sdk_import.cmake
├── cmake/pq_sources.cmake    platform-independent source list (shared by Pico + host)
├── cmake/pq_tflm.cmake       TensorFlow Lite Micro (pico-tflmicro) as static lib
├── config/pq_build_config.h  ALL compile-time defaults and buffer limits
├── host/                     PC build: tests, pq_sim, pq_datagen, pq_bench
├── tests/                    unit + integration tests, PIO emulation test
└── src/
    ├── main.cpp              Pico entry: clocks, dual-core start, watchdog
    ├── common/               types, SPSC lock-free ring, math/CRC, platform API
    ├── adc/                  HAL: adc_interface.h (abstract), adc_driver (ADC_Init/Start/ReadSamples/Stop),
    │                         adc_calibration, ads131m02_protocol (device protocol, host-testable),
    │                         waveform_generator + sim_adc (simulated backend)
    ├── dsp/                  filtering (biquad, DC blocker, half-band FIR), frequency, rms,
    │                         fft, thd (harmonics), wavelet, transient
    ├── power_quality/        pq_events.h, disturbance_detector
    ├── features/             feature_spec.h (single source of truth), feature_extractor
    ├── tinyml/               classifier (TFLM), model_data (generated), tflm_platform
    ├── fusion/               decision_fusion
    ├── app/                  pq_engine (whole pipeline), realtime_task (core 1),
    │                         supervisor (core 0), rt_link (inter-core), benchmark
    ├── communication/        transport, protocol, json_writer
    ├── outputs/              control_output (SSCB/DVR/APF/event), alarm_output
    ├── storage/              event_logger
    ├── system/               config (+ parameter table), config_store, log_sink, system_monitor
    └── platform/
        ├── pico/             ADS131M02 PIO+DMA backend, USB/UART, GPIO/PWM/DAC, flash, pin map
        └── host/             host timing, stdio/pty transport, file config, recorders
```

Changes from the proposed layout: the pipeline is gathered into one `PqEngine` class, so the simulator, tests, dataset generator and firmware cannot diverge. The core split is made explicit (`realtime_task` on core 1, `supervisor` on core 0, `rt_link` between them). And all SDK-specific code is isolated under `platform/`.

## Data flow

```
          ┌──────────────── core 1 (real time) ─────────────────────────────────────────┐
 DRDY ─►  │ PIO frame reader ─► DMA ring (16 × 50 samples) ─IRQ: block counter+time─┐    │
 CLKIN ◄─ │                                                                          ▼    │
 PWM      │ RealtimeTask::poll()  ── ADC_ReadSamples(RawBlock) ── Calibrator ──► PqEngine │
          │   per sample: DC block → zero-crossing freq → ½-cycle RMS → detector          │
          │               wavelet + transient detector (sample-accurate onset)            │
          │               half-band ↓2 → analysis ring                                    │
          │   every hop (512 @ 5 kHz = 102 ms): FFT(V,I) → harmonics/THD → window stats   │
          │               → frame detection → features → TinyML → fusion → FrameResult    │
          │   fast outputs: SSCB trip · DVR reference (per ½ cycle) · event code          │
          └───────────┬───────────────────────┬──────────────────────────────▲───────────┘
             frames (SPSC ring)     events (SPSC ring)        commands + config mailbox
          ┌───────────▼───────────────────────▼──────────────────────────────┴───────────┐
          │ core 0: Supervisor — protocol (USB CDC / UART), event log, metrics stream,     │
          │ alarms, calibration procedures, flash config, BENCH, watchdog (both cores)     │
          └────────────────────────────────────────────────────────────────────────────────┘
```

### Real-time loop (core 1)

`RealtimeTask::poll()` is a non-blocking state machine iteration, not a monolithic `while(true)`. Each iteration does the following:

1. Drain the command ring (config apply, start/stop, simulator control, calibration, waveform capture).
2. Process up to 4 ready blocks. The DMA ring holds 16 blocks = 80 ms of slack at 10 kS/s.
3. Run the ADC health check (DRDY watchdog). The ADC is re-initialised automatically, at most once per second.
4. Publish a status snapshot (seqlock) every 50 ms, or immediately after a command.

### Supervisor (core 0)

`Supervisor::poll()` does the following:

- parses commands from USB CDC and, optionally, UART0;
- drains the frame, event and log rings;
- streams metrics and events;
- keeps the RAM event log;
- drives alarm patterns;
- runs calibration procedures;
- persists the configuration to flash.

The watchdog is fed only while **both** cores make progress.

### Inter-core communication

This is all lock-free (`SpscRing`, acquire/release atomics). A single producer and a single consumer per ring means no locks and no priority inversion. If a ring is full, the message is dropped, counted, and flagged as `BUFFER_OVERFLOW`, so the real-time core never waits.

## Hardware abstraction

`AdcBackend` (in `adc/adc_interface.h`) is the only interface the application sees:

```cpp
AdcStatus init(const AdcHwConfig&);  AdcStatus start();  void stop();
bool readBlock(RawBlock&);           AdcStatus poll();   float actualSampleRate();
```

`adc_driver.h` wraps it as `ADC_Init / ADC_Start / ADC_ReadSamples / ADC_Stop`. There are two implementations:

- `Ads131m02Backend` (`platform/pico/`): the real 24-bit ADC.
- `SimAdcBackend` (`adc/`): the waveform model rendered into ADC counts, paced by the RP2 hardware timer.

The DSP cannot tell the two apart. To support another ADC (for example an ADS131M04 for three phases), implement `AdcBackend` and raise `kAdcChannels`.

`RawBlock` is planar: `int32_t ch[kAdcChannels][50]`, plus a per-sample validity bitmap, a sequence number and a timestamp. Timestamps come from the sample counter, `t(n) = t0 + n/fs`, which is exact because the ADC clock is derived from the MCU clock. The DMA completion times are still recorded, to detect missing samples.

## Memory (static only)

There is no `malloc` after start-up. Every buffer is a static member or global. The biggest users are:

- the engine's FFT buffers, including a 2048-point capacity (~51 KB);
- the benchmark analyzer;
- the DMA ring (9.6 KB);
- the TFLM arena (4 KB).

See [performance.md](performance.md) for the measured `.bss` and flash sizes.
