#include "app/benchmark.h"

#include <cmath>
#include <cstring>

#include "adc/adc_calibration.h"
#include "adc/waveform_generator.h"
#include "common/platform.h"
#include "dsp/filtering.h"
#include "dsp/frequency.h"
#include "dsp/rms.h"
#include "dsp/thd.h"
#include "dsp/transient.h"
#include "dsp/wavelet.h"

namespace pq {

namespace {
// Static working set kept small (the analyzer is the only big object) because
// this runs on the supervisor core next to the live pipeline.
constexpr int kN = PQ_FFT_MAX;
float g_v[kN];
HarmonicAnalyzer g_an;
volatile float g_sink;   // defeat dead-code elimination

void makeSignal(float fs) {
    WaveformGenerator gen;
    GeneratorParams p;
    p.base_h[0] = 0.05f;
    p.base_h[1] = 0.03f;
    p.base_h[2] = 0.02f;
    gen.configure(p, fs);
    float i_tmp[64];
    for (int k = 0; k < kN; k += 64) gen.generate(64, g_v + k, i_tmp);
}

template <typename F>
float nsPerSample(int n, F&& body) {
    const uint64_t t0 = plat::timeUs();
    for (int k = 0; k < n; ++k) body(k);
    const uint64_t t1 = plat::timeUs();
    return 1000.0f * static_cast<float>(t1 - t0) / static_cast<float>(n);
}
}  // namespace

void runBenchmarks(float fs, int dec, BenchReport& out) {
    std::memset(&out, 0, sizeof(out));
    out.fs = fs;
    out.analysis_fs = fs / dec;
    makeSignal(fs);
    const int sizes[4] = {256, 512, 1024, 2048};
    const int reps = 8;
    out.iterations = reps;
    for (int s = 0; s < 4; ++s) {
        const int n = sizes[s];
        FftBench& b = out.fft[s];
        b.n = n;
        b.resolution_hz = out.analysis_fs / n;
        b.window_ms = 1000.0f * n / out.analysis_fs;
        if (n > PQ_FFT_MAX) continue;   // not compiled in (PQ_FFT_MAX)
        g_an.init(n, out.analysis_fs);
        uint64_t t0 = plat::timeUs();
        for (int r = 0; r < reps; ++r) g_an.fftOnly(g_v);
        uint64_t t1 = plat::timeUs();
        b.fft_us = static_cast<uint32_t>((t1 - t0) / reps);
        HarmonicResult hr;
        t0 = plat::timeUs();
        for (int r = 0; r < reps; ++r) g_an.analyze(g_v, 50.0f, 50.0f, PQ_MAX_HARMONIC, hr);
        t1 = plat::timeUs();
        b.analysis_us = static_cast<uint32_t>((t1 - t0) / reps);
        g_sink = hr.thd_pct;
        const float hop_us = 1e6f * (n / 2) / out.analysis_fs;
        b.budget_pct = 100.0f * 2.0f * b.analysis_us / hop_us;
    }

    KernelBench& k = out.k;
    const int n = kN;
    {
        DcBlocker d;
        d.configure(1.0f, fs);
        float acc = 0;
        k.dc_block_ns = nsPerSample(n, [&](int j) { acc += d.process(g_v[j]); });
        g_sink = acc;
    }
    {
        FrequencyEstimator f;
        f.configure(fs, 50.0f, 1.5f, 5, 325.0f);
        float acc = 0;
        k.frequency_ns = nsPerSample(n, [&](int j) { acc += f.process(g_v[j], j).frac; });
        g_sink = acc + f.frequency();
    }
    {
        // RMS needs the crossings: time frequency+RMS together, subtract frequency.
        FrequencyEstimator f;
        f.configure(fs, 50.0f, 1.5f, 5, 325.0f);
        HalfCycleRms r;
        r.configure(fs, 50.0f);
        HalfCycleResult hc;
        float acc = 0;
        const float both = nsPerSample(n, [&](int j) {
            const CrossingEvent ce = f.process(g_v[j], j);
            if (r.process(g_v[j], 0.02f * g_v[j], j, ce, hc)) acc += hc.v_rms;
        });
        k.rms_ns = both > k.frequency_ns ? both - k.frequency_ns : 0.0f;
        g_sink = acc;
    }
    {
        StreamingDwt w;
        w.configure(WaveletType::Haar, 3);
        uint32_t acc = 0;
        k.haar_ns = nsPerSample(n, [&](int j) { acc += w.push(g_v[j]); });
        w.configure(WaveletType::Db4, 3);
        k.db4_ns = nsPerSample(n, [&](int j) { acc += w.push(g_v[j]); });
        g_sink = static_cast<float>(acc);
    }
    {
        StreamingDwt w;
        w.configure(WaveletType::Db4, 3);
        TransientDetector t;
        TransientParams tp{fs, 50.0f, 325.0f, 0.15f, 1.4f, 4.0f, 8.0f, 0.03f, true, true};
        t.configure(tp);
        float acc = 0;
        k.transient_ns = nsPerSample(n, [&](int j) {
            const uint32_t pr = w.push(g_v[j]);
            acc += t.process(g_v[j], j, fs / 50.0f, &w, pr);
        });
        g_sink = acc;
    }
    {
        HalfbandDecimator d;
        d.configure(2);
        float acc = 0, y;
        k.decimator_ns = nsPerSample(n, [&](int j) {
            if (d.push(g_v[j], y)) acc += y;
        });
        g_sink = acc;
    }
    {
        static RawBlock rb;
        static CalibratedBlock cb;
        std::memset(&rb, 0, sizeof(rb));
        for (uint32_t j = 0; j < kBlockSize; ++j) {
            rb.ch[0][j] = static_cast<int32_t>(g_v[j] * 10000.0f);
            rb.ch[1][j] = static_cast<int32_t>(g_v[j] * 100.0f);
        }
        rb.n = kBlockSize;
        Calibrator c;
        CalConfig cc{1e-4f, 3.0f, 1e-4f, -2.0f, 1.0f};
        c.configure(cc, fs, 50.0f);
        const int blocks = 40;
        const uint64_t t0 = plat::timeUs();
        for (int b = 0; b < blocks; ++b) c.apply(rb, cb);
        const uint64_t t1 = plat::timeUs();
        k.calibration_ns = 1000.0f * static_cast<float>(t1 - t0) / (blocks * kBlockSize);
        g_sink = cb.v[3];
    }
    // Per-ADC-sample cost of the streaming path (both channels where applicable).
    k.per_sample_total_ns = k.calibration_ns + 2.0f * k.dc_block_ns + k.frequency_ns + k.rms_ns +
                            k.transient_ns + 2.0f * k.decimator_ns;
    k.per_sample_budget_pct = 100.0f * k.per_sample_total_ns * 1e-9f * fs;
}

}  // namespace pq
