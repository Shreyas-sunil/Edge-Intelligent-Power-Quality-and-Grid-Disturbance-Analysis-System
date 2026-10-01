// =============================================================================
//  Micro-benchmarks of individual DSP kernels, executed on the calling core
//  with the real code and real data (a synthetic distorted waveform). Used by
//  the BENCH command (core0 on the Pico, so acquisition on core1 is untouched)
//  and by the host `pq_bench` tool. All numbers are measured, never estimated.
// =============================================================================
#pragma once
#include <cstdint>

namespace pq {

struct FftBench {
    int      n;
    float    resolution_hz;   // bin width at the analysis rate
    float    window_ms;
    uint32_t fft_us;          // one real FFT
    uint32_t analysis_us;     // window + FFT + harmonic grouping + THD (one channel)
    float    budget_pct;      // 2 channels / hop period, hop = n/2
};

struct KernelBench {
    float dc_block_ns;        // per sample
    float frequency_ns;
    float rms_ns;
    float haar_ns;            // 3-level streaming DWT
    float db4_ns;
    float transient_ns;       // incl. db4 wavelet
    float decimator_ns;       // per input sample, /2
    float calibration_ns;     // per sample, 2 channels
    float per_sample_total_ns;
    float per_sample_budget_pct;   // of 1/fs
};

struct BenchReport {
    int         nfft;
    FftBench    fft[4];       // 256, 512, 1024, 2048
    KernelBench k;
    float       fs;
    float       analysis_fs;
    uint32_t    iterations;
};

// fs = ADC rate, dec = decimation. Takes roughly 0.1–1 s depending on the CPU.
void runBenchmarks(float fs, int dec, BenchReport& out);

}  // namespace pq
