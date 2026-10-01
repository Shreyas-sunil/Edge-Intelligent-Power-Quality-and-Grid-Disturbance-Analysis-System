// =============================================================================
//  pq_build_config.h — compile-time limits and default configuration values.
//
//  This is the ONE place where default thresholds / rates live. Runtime code
//  reads them through SystemConfig (src/system/config.h); nothing else in the
//  code base hard-codes these numbers. Every value below can be overridden by
//  -D on the compiler command line (see firmware/CMakeLists.txt) and most can be
//  changed at run time with `CONFIG SET <key> <value>` over the serial link.
// =============================================================================
#pragma once

// ---------------------------------------------------------------- static limits
// These size static buffers (no heap is used in the real-time path).
#ifndef PQ_BLOCK_SIZE
#define PQ_BLOCK_SIZE 50           // samples per acquisition block (5 ms @ 10 kHz)
#endif
#ifndef PQ_ACQ_RING_BLOCKS
#define PQ_ACQ_RING_BLOCKS 16      // DMA ring depth (power of two) -> 80 ms slack @10 kHz
#endif
#ifndef PQ_FFT_MAX
#define PQ_FFT_MAX 2048            // largest FFT size that can be selected at run time
#endif
#ifndef PQ_MAX_HARMONIC
#define PQ_MAX_HARMONIC 40         // highest harmonic order evaluated (IEC 61000-4-7: 40/50)
#endif
#ifndef PQ_REPORT_HARMONICS
#define PQ_REPORT_HARMONICS 15     // harmonics carried in every FrameResult (H1..H15)
#endif
#ifndef PQ_MAX_SAMPLE_RATE
#define PQ_MAX_SAMPLE_RATE 25600.0f
#endif
#ifndef PQ_EVENT_LOG_SIZE
#define PQ_EVENT_LOG_SIZE 64       // RAM event log entries (power of two not required)
#endif
#ifndef PQ_WAVE_SNAPSHOT
#define PQ_WAVE_SNAPSHOT 512       // samples per channel in a WAVE snapshot
#endif

// ---------------------------------------------------------- acquisition defaults
#ifndef PQ_DEFAULT_SAMPLE_RATE
#define PQ_DEFAULT_SAMPLE_RATE 10000.0f
#endif
#ifndef PQ_DEFAULT_DECIMATION
#define PQ_DEFAULT_DECIMATION 2    // analysis rate = 5 kHz -> harmonics to H40 @ 50 Hz
#endif
#ifndef PQ_DEFAULT_FFT_SIZE
#define PQ_DEFAULT_FFT_SIZE 1024   // 1024 @ 5 kHz = 204.8 ms ~ IEC 10-cycle window
#endif
#ifndef PQ_DEFAULT_HOP
#define PQ_DEFAULT_HOP 512         // analysis every 102.4 ms (50 % overlap)
#endif
#ifndef PQ_DEFAULT_SIMULATION
#define PQ_DEFAULT_SIMULATION 1    // 1 = simulated ADC backend at boot, 0 = hardware ADC
#endif

// ------------------------------------------------------------- nominal system
#ifndef PQ_DEFAULT_NOMINAL_VOLTAGE
#define PQ_DEFAULT_NOMINAL_VOLTAGE 230.0f
#endif
#ifndef PQ_DEFAULT_NOMINAL_FREQUENCY
#define PQ_DEFAULT_NOMINAL_FREQUENCY 50.0f
#endif
#ifndef PQ_DEFAULT_NOMINAL_CURRENT
#define PQ_DEFAULT_NOMINAL_CURRENT 10.0f   // only used for per-unit current features
#endif

// ------------------------------------------------------- detection thresholds
#define PQ_DEFAULT_SAG_THRESHOLD        0.90f  // pu of nominal (IEC 61000-4-30 dip)
#define PQ_DEFAULT_SWELL_THRESHOLD      1.10f  // pu
#define PQ_DEFAULT_INTERRUPTION_THRESHOLD 0.10f // pu
#define PQ_DEFAULT_RMS_HYSTERESIS       0.02f  // pu (IEC 61000-4-30 suggests 2 %)
#define PQ_DEFAULT_THD_THRESHOLD        8.0f   // % (IEEE 519-2014 limit, <=1 kV bus)
#define PQ_DEFAULT_THD_HYSTERESIS       0.5f   // %
#define PQ_DEFAULT_FREQ_TOLERANCE       0.5f   // Hz (EN 50160: +/-1 % of 50 Hz)
#define PQ_DEFAULT_FREQ_MIN_FRAMES      2      // consecutive frames before FREQ event
#define PQ_DEFAULT_TRANSIENT_DEV_PU     0.15f  // cycle-to-cycle deviation, pu of Vpk
#define PQ_DEFAULT_IMPULSE_PU           1.40f  // instantaneous |v| in pu of nominal Vpk
#define PQ_DEFAULT_DVDT_FACTOR          4.0f   // x max fundamental slope
#define PQ_DEFAULT_WAVELET_K            8.0f   // x adaptive noise floor
#define PQ_DEFAULT_WAVELET_FLOOR_PU     0.03f  // absolute floor, pu of Vpk
#define PQ_DEFAULT_LOAD_STEP            0.30f  // 30 % step in cycle Irms
#define PQ_DEFAULT_LOAD_MIN_CURRENT     0.20f  // A — ignore steps below this
#define PQ_DEFAULT_MIN_EVENT_HALFCYCLES 1

// ----------------------------------------------------------------- DSP options
#define PQ_DEFAULT_DC_BLOCK_HZ          1.0f
#define PQ_DEFAULT_FREQ_LPF_FACTOR      1.5f   // LPF corner = 1.5 x nominal frequency
#define PQ_DEFAULT_FREQ_AVG_CYCLES      5
#define PQ_DEFAULT_WAVELET_ENABLE       1
#define PQ_DEFAULT_WAVELET_TYPE         1      // 0 = Haar, 1 = Daubechies-4
#define PQ_DEFAULT_WAVELET_LEVELS       3

// ------------------------------------------------------------- ML / fusion
#define PQ_DEFAULT_ML_ENABLE            1
#define PQ_DEFAULT_FUSION_DSP_WEIGHT    0.60f
#define PQ_DEFAULT_ML_ONLY_CONFIDENCE   0.90f
#define PQ_DEFAULT_ML_ONLY_FRAMES       3

// -------------------------------------------------------------- communication
#define PQ_DEFAULT_METRICS_PERIOD_MS    1000
#define PQ_DEFAULT_UART_BAUD            115200
#define PQ_DEFAULT_LOGGING_ENABLE       1

// ------------------------------------------------------------------- control
#define PQ_DEFAULT_CONTROL_ARMED        0      // outputs are inert until armed
#define PQ_DEFAULT_TRIP_SWELL_PU        1.30f
#define PQ_DEFAULT_TRIP_IMPULSE_PU      2.00f
