# Performance and resources

**Rule: no invented numbers.** Everything below is measured. Where a number depends on the MCU, the firmware measures it itself — run `BENCH` on your board and paste the output into the table at the end.

## Memory — measured from the linked images (Pico SDK 2.2.0, GCC 13.2, Release)

| | Pico 2 (RP2350) | Pico (RP2040) |
|---|---|---|
| flash (`.text` + rodata) | 221,604 B | 224,148 B |
| static RAM (`.data` + `.bss`) | 226,072 B of 520 KB | 158,872 B of 264 KB (FFT capped at 1024) |
| heap after init | 0 used by the firmware (only SDK internals) — `STATUS.memory.heap_used` | same |
| core 0 stack | 4 KB (SCRATCH_Y), free space reported in `STATUS` | same |
| core 1 stack | 8 KB static, painted; free space reported in `STATUS` | same |

Largest RAM users on the Pico 2 build:

| object | size |
|---|---|
| real-time task incl. pipeline and FFT buffers for N ≤ 2048 | 99 KB |
| benchmark analyzer (core 0) | 51 KB |
| inter-core link: frame and event rings, wave snapshot | 9.5 KB |
| DMA sample ring (16 × 50 × 12 B) | 9.6 KB |
| USB TX buffer | 8 KB |
| TFLM tensor arena | 4 KB |

## TinyML

| | value |
|---|---|
| model | 1,144 parameters, int8 |
| flatbuffer in flash | 5,160 B |
| tensor arena | 4,096 B reserved (static, 16-byte aligned); 1,936 B used on the 64-bit host. The MCU reports its own value in `MODEL.arena_used` |
| ops linked | FullyConnected, Softmax, Relu, Quantize, Dequantize |

## Timing on the build machine (host PC, `pq_bench`; Intel Xeon @ 2.1 GHz)

These numbers show the relative cost of the stages. **They are not MCU numbers.**

FFT size selection (5 kS/s analysis rate, hop = N/2, V and I):

| N | resolution | window | FFT | FFT + harmonics + THD, 1 channel |
|---|---|---|---|---|
| 256 | 19.53 Hz | 51.2 ms | 3 µs | 6 µs |
| 512 | 9.77 Hz | 102.4 ms | 3 µs | 6 µs |
| 1024 | 4.88 Hz | 204.8 ms | 12 µs | 13 µs |
| 2048 | 2.44 Hz | 409.6 ms | 16 µs | 25 µs |

Streaming kernels, in ns per sample:

| calibration (2 ch) | DC block | frequency | RMS | Haar ×3 | db4 ×3 | transient incl. db4 | half-band ↓2 |
|---|---|---|---|---|---|---|---|
| 7.5 | 2.9 | 11.7 | 5.9 | 17.6 | 22.0 | 63.0 | 8.3 |

Full pipeline on 20 s of signal: 17.9 ms of processing, which is 0.089 % of real time. The average per 102.4 ms hop is:

| stage | time |
|---|---|
| streaming stages | 63.6 µs |
| FFT + THD | 21.5 µs |
| window statistics | 0.3 µs |
| detection | 0.1 µs |
| features | 0.1 µs |
| ML | 1.8 µs (`Invoke` 1.4 µs) |
| fusion | 0.1 µs |
| **total** | **87.6 µs** (max 253 µs) |

## Timing on the MCU — measure with `BENCH`

`BENCH` returns three messages:

1. **`part: live`** — measured on core 1 during normal operation, averaged over all frames since the last `STATUS CLEAR`:
   - per stage: per-sample path per hop, FFT/THD, window, detection, features, ML total and `Invoke()`, fusion, frame total, total per hop;
   - `cpu_load_pct`: core-1 busy time over wall time, excluding simulator waveform generation; `cpu_total_pct` includes it;
   - `max_block_us` against `block_budget_us` (5000 µs at 10 kS/s);
   - `ring_high_water` out of `ring_blocks`, i.e. buffer utilisation;
   - RAM, flash, model size and arena use.
2. **`part: fft`** — FFT and analysis time for N = 256, 512, 1024, 2048 (as compiled) on core 0, with the resulting load for 2 channels at 50 % overlap.
3. **`part: kernels_ns_per_sample`** — the per-sample costs above, on the MCU.

Results table (fill in from your board):

| metric | Pico 2 @ 153.6 MHz | Pico @ 128 MHz |
|---|---|---|
| sampling rate | 10 000 S/s (5.12 MHz / 2 / 256) | 10 000 S/s |
| FFT size | 1024 @ 5 kS/s | 1024 @ 5 kS/s |
| DSP latency per hop (`total_per_hop.avg_us`) | | |
| feature extraction (`feature_extraction`) | | |
| ML latency (`ml_invoke`) | | |
| total per hop / hop period 102 400 µs | | |
| CPU utilisation core 1 (`cpu_load_pct`) | | |
| buffer utilisation (`buffer_util_pct`) | | |
| RAM / flash | see above | see above |

## Choices that keep the real-time path cheap

- **Zero CPU per sample for acquisition.** PIO reads each frame, DMA fills the ring, and one IRQ per 50 samples only increments a counter.
- **Decimation before the FFT.** Halving the rate halves the FFT cost, and harmonics up to H40 are still resolved.
- **A 14-number feature vector instead of raw samples.** The model is about 1 k parameters and the arena under 2 KB.
- **The wavelet is optional**, and Haar or db4 is selectable.
- **RP2040 builds** automatically cap the FFT buffers at 1024 points. If `BENCH` on an RP2040 shows too much load, the knobs are `acq.decimation 4`, `acq.fft_size 512` and `dsp.wavelet_type 0`. The RP2040 has no FPU, so all DSP uses the SDK's optimised ROM float routines. The primary target is the Pico 2.
- **`-DPQ_COPY_TO_RAM=ON`** runs the whole image from SRAM. This avoids XIP cache misses and gives the most deterministic timing, and it fits on the RP2350.
