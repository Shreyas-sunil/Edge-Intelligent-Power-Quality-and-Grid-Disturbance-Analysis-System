// Filtering, RMS, frequency, FFT, THD, wavelet, transient detector.
#include <cmath>
#include <vector>

#include "adc/waveform_generator.h"
#include "common/math_util.h"
#include "dsp/fft.h"
#include "dsp/filtering.h"
#include "dsp/frequency.h"
#include "dsp/rms.h"
#include "dsp/thd.h"
#include "dsp/transient.h"
#include "dsp/wavelet.h"
#include "test_framework.h"

using namespace pq;

namespace {
std::vector<float> sine(int n, float fs, float f, float amp, float phase = 0.0f) {
    std::vector<float> x(n);
    for (int k = 0; k < n; ++k) x[k] = amp * std::sin(2.0 * M_PI * f * k / fs + phase);
    return x;
}
float rmsOf(const std::vector<float>& x, int from) {
    double s = 0;
    for (size_t k = from; k < x.size(); ++k) s += x[k] * x[k];
    return static_cast<float>(std::sqrt(s / (x.size() - from)));
}
}  // namespace

// ------------------------------------------------------------------ filters
PQ_TEST(filter, butterworth_lowpass_response) {
    BiquadCascade lp;
    lp.designButterworthLowpass(4, 75.0f, 10000.0f);
    PQ_CHECK_NEAR(lp.magnitude(75.0f, 10000.0f), std::sqrt(0.5), 0.01);   // -3 dB at fc
    PQ_CHECK(lp.magnitude(50.0f, 10000.0f) > 0.97f);
    const float h3 = 20 * std::log10(lp.magnitude(150.0f, 10000.0f));
    PQ_INFO("LPF(75 Hz): |H(50)|=%.4f  H3 attenuation %.1f dB", lp.magnitude(50.0f, 10000.0f), h3);
    PQ_CHECK(h3 < -23.0f);   // 4th order: ~24 dB/octave
    // time domain: steady-state amplitude of a 150 Hz tone
    auto x = sine(20000, 10000.0f, 150.0f, 1.0f);
    std::vector<float> y(x.size());
    for (size_t k = 0; k < x.size(); ++k) y[k] = lp.process(x[k]);
    PQ_CHECK_NEAR(rmsOf(y, 5000) * kSqrt2, lp.magnitude(150.0f, 10000.0f), 0.005);
}

PQ_TEST(filter, dc_blocker_removes_offset_keeps_50hz) {
    DcBlocker d;
    d.configure(1.0f, 10000.0f);
    auto x = sine(60000, 10000.0f, 50.0f, 100.0f);
    double mean = 0;
    std::vector<float> y(x.size());
    for (size_t k = 0; k < x.size(); ++k) y[k] = d.process(x[k] + 25.0f);   // 25 V offset
    for (size_t k = 50000; k < y.size(); ++k) mean += y[k];
    mean /= 10000;
    PQ_CHECK(std::fabs(mean) < 0.05);
    PQ_CHECK_NEAR(rmsOf(y, 50000), 100.0 / std::sqrt(2.0), 0.02);   // gain at 50 Hz ~ 0.9998
}

PQ_TEST(filter, halfband_decimator_passband_and_alias_rejection) {
    const float* h = HalfbandDecimator::prototype();
    double sum = 0;
    for (int k = 0; k < HalfbandDecimator::kTaps; ++k) sum += h[k];
    PQ_CHECK_NEAR(sum, 1.0, 1e-6);
    PQ_CHECK(h[(HalfbandDecimator::kTaps - 1) / 2 + 2] == 0.0f);   // even taps are zero
    auto meas = [](float f) {
        HalfbandDecimator d;
        d.configure(2);
        auto x = sine(40000, 10000.0f, f, 1.0f);
        std::vector<float> y;
        float o;
        for (float v : x)
            if (d.push(v, o)) y.push_back(o);
        return rmsOf(y, 2000) * kSqrt2;
    };
    const float g2k = meas(2000.0f), g3k = meas(3050.0f), g4k = meas(4000.0f);
    PQ_INFO("half-band /2: |H(2 kHz)|=%.5f  |H(3.05 kHz)|=%.2e (%.1f dB)  |H(4 kHz)|=%.2e", g2k, g3k,
            20 * std::log10(g3k), g4k);
    PQ_CHECK(std::fabs(g2k - 1.0f) < 0.002f);   // H40 at 50 Hz passes flat
    PQ_CHECK(g3k < 1e-3f);                       // would alias to 1.95 kHz: > 60 dB down
    PQ_CHECK(g4k < 1e-3f);
}

// ------------------------------------------------------------ frequency
PQ_TEST(freq, zero_crossing_accuracy) {
    const float fs = 10000.0f;
    for (float f : {45.3f, 49.98f, 50.0f, 50.37f, 61.2f}) {
        FrequencyEstimator fe;
        fe.configure(fs, f > 55 ? 60.0f : 50.0f, 1.5f, 5, 325.0f);
        Rng r(5);
        for (int k = 0; k < 20000; ++k) {
            const float v = 325.0f * std::sin(2.0 * M_PI * f * k / fs) + 0.5f * r.gauss();
            fe.process(v, k);
        }
        PQ_CHECK(fe.valid());
        PQ_INFO("f=%.3f -> %.5f Hz (err %.2f mHz)", f, fe.frequency(), 1000 * (fe.frequency() - f));
        PQ_CHECK_NEAR(fe.frequency(), f, 0.003);
    }
}

PQ_TEST(freq, robust_to_harmonics_and_loss_of_signal) {
    const float fs = 10000.0f;
    FrequencyEstimator fe;
    fe.configure(fs, 50.0f, 1.5f, 5, 325.0f);
    for (int k = 0; k < 20000; ++k) {
        const double t = k / fs;
        const float v = 325.0f * (std::sin(2 * M_PI * 50.2 * t) + 0.2 * std::sin(3 * 2 * M_PI * 50.2 * t + 1.0) +
                                  0.1 * std::sin(5 * 2 * M_PI * 50.2 * t));
        fe.process(v, k);
    }
    PQ_CHECK_NEAR(fe.frequency(), 50.2, 0.005);
    // interruption: only noise -> estimator must declare itself invalid
    Rng r(1);
    for (int k = 20000; k < 22000; ++k) fe.process(0.5f * r.gauss(), k);
    PQ_CHECK(!fe.valid());
}

// ------------------------------------------------------------------ RMS
PQ_TEST(rms, half_cycle_rms_exact_for_offnominal_frequency) {
    const float fs = 10000.0f;
    for (float f : {49.5f, 50.0f, 50.7f}) {
        FrequencyEstimator fe;
        fe.configure(fs, 50.0f, 1.5f, 5, 325.0f);
        HalfCycleRms rms;
        rms.configure(fs, 50.0f);
        HalfCycleResult hc{};
        float worst = 0;
        int n_hc = 0;
        for (int k = 0; k < 20000; ++k) {
            const float v = 325.27f * std::sin(2.0 * M_PI * f * k / fs + 0.3);
            const float i = 14.142f * std::sin(2.0 * M_PI * f * k / fs + 0.3 - 0.5);
            const CrossingEvent ce = fe.process(v, k);
            if (rms.process(v, i, k, ce, hc) && k > 5000 && hc.synced) {
                worst = std::fmax(worst, std::fabs(hc.v_rms - 230.0f));
                PQ_CHECK_NEAR(hc.i_rms, 10.0, 0.02);
                PQ_CHECK_NEAR(hc.p, 2300.0 * std::cos(0.5), 5.0);
                ++n_hc;
            }
        }
        PQ_INFO("f=%.1f Hz: %d half cycles, max |Vrms - 230| = %.4f V", f, n_hc, worst);
        PQ_CHECK(worst < 0.05f);
        PQ_CHECK(n_hc > 2 * f * 1.4f);
    }
}

PQ_TEST(rms, updates_during_interruption_via_timeout) {
    const float fs = 10000.0f;
    FrequencyEstimator fe;
    fe.configure(fs, 50.0f, 1.5f, 5, 325.0f);
    HalfCycleRms rms;
    rms.configure(fs, 50.0f);
    HalfCycleResult hc{};
    int after = 0;
    float last = 999;
    for (int k = 0; k < 20000; ++k) {
        const float v = k < 10000 ? 325.0f * std::sin(2.0 * M_PI * 50 * k / fs) : 0.0f;
        const CrossingEvent ce = fe.process(v, k);
        if (rms.process(v, 0, k, ce, hc) && k > 10000) {
            ++after;
            last = hc.v_rms;
        }
    }
    PQ_CHECK(after >= 75);   // still ~80 updates/s without zero crossings
    PQ_CHECK(last < 0.01f);
}

// ------------------------------------------------------------------ FFT
PQ_TEST(fft, matches_reference_dft) {
    for (int n : {256, 512, 1024, 2048}) {
        RealFft fft;
        PQ_CHECK(fft.init(n));
        std::vector<float> x(n), X(n + 2);
        Rng r(n);
        for (int k = 0; k < n; ++k) x[k] = r.gauss();
        fft.forward(x.data(), X.data());
        double maxerr = 0;
        for (int k = 0; k <= n / 2; k += (k < 8 ? 1 : 37)) {
            double re = 0, im = 0;
            for (int j = 0; j < n; ++j) {
                re += x[j] * std::cos(2 * M_PI * k * j / n);
                im -= x[j] * std::sin(2 * M_PI * k * j / n);
            }
            maxerr = std::fmax(maxerr, std::hypot(X[2 * k] - re, X[2 * k + 1] - im));
        }
        PQ_INFO("N=%d max |FFT - DFT| = %.2e", n, maxerr);
        PQ_CHECK(maxerr < 1e-3 * std::sqrt(n));
    }
    RealFft bad;
    PQ_CHECK(!bad.init(1000));
}

// ------------------------------------------------------------------ THD
PQ_TEST(thd, harmonic_magnitudes_and_thd_nonsynchronous) {
    const float fs = 5000.0f;
    HarmonicAnalyzer an;
    PQ_CHECK(an.init(1024, fs));
    const float f0 = 49.87f;   // deliberately not bin-aligned
    const float V1 = 230.0f, r3 = 0.08f, r5 = 0.05f, r7 = 0.03f, r11 = 0.02f;
    std::vector<float> x(1024);
    for (int k = 0; k < 1024; ++k) {
        const double w = 2 * M_PI * f0 * k / fs;
        x[k] = kSqrt2 * V1 * (std::sin(w + 0.4) + r3 * std::sin(3 * w + 1) + r5 * std::sin(5 * w + 2) +
                              r7 * std::sin(7 * w) + r11 * std::sin(11 * w + 0.5));
    }
    HarmonicResult h;
    an.analyze(x.data(), f0, 50.0f, 40, h);
    const double thd = 100 * std::sqrt(r3 * r3 + r5 * r5 + r7 * r7 + r11 * r11);
    PQ_INFO("H1=%.3f H3=%.3f H5=%.3f H7=%.3f THD=%.3f %% (true %.3f %%)", h.mag[1], h.mag[3], h.mag[5], h.mag[7],
            h.thd_pct, thd);
    PQ_CHECK_NEAR(h.mag[1], V1, V1 * 0.002);
    PQ_CHECK_NEAR(h.mag[3], V1 * r3, V1 * 0.002);
    PQ_CHECK_NEAR(h.mag[5], V1 * r5, V1 * 0.002);
    PQ_CHECK_NEAR(h.mag[7], V1 * r7, V1 * 0.002);
    PQ_CHECK_NEAR(h.thd_pct, thd, 0.1);
    PQ_CHECK_NEAR(h.phase[1], 0.4, 0.02);   // sine-referenced phase
    PQ_CHECK_NEAR(h.fft_peak_hz, f0, 0.25);
    PQ_CHECK(h.valid);
}

PQ_TEST(thd, pure_sine_has_negligible_thd_and_configurable_order) {
    HarmonicAnalyzer an;
    an.init(1024, 5000.0f);
    std::vector<float> x(1024);
    Rng r(3);
    for (int k = 0; k < 1024; ++k) x[k] = 325.0f * std::sin(2 * M_PI * 50.1 * k / 5000.0) + 0.05f * r.gauss();
    HarmonicResult h;
    an.analyze(x.data(), 50.1f, 50.0f, 40, h);
    PQ_CHECK(h.thd_pct < 0.05f);
    PQ_CHECK(h.h_max == 40);
    an.analyze(x.data(), 50.1f, 50.0f, 7, h);
    PQ_CHECK(h.h_max == 7);
}

PQ_TEST(thd, interharmonic_peak_reported) {
    HarmonicAnalyzer an;
    an.init(1024, 5000.0f);
    std::vector<float> x(1024);
    for (int k = 0; k < 1024; ++k)
        x[k] = 325.0f * std::sin(2 * M_PI * 50 * k / 5000.0) + 10.0f * std::sin(2 * M_PI * 175 * k / 5000.0);
    HarmonicResult h;
    an.analyze(x.data(), 50.0f, 50.0f, 40, h);
    PQ_CHECK(h.npeaks >= 1);
    PQ_CHECK_NEAR(h.peaks[0].hz, 175.0, 2.0);
    PQ_CHECK_NEAR(h.peaks[0].rms, 10.0 / std::sqrt(2.0), 0.3);
}

// -------------------------------------------------------------- wavelet
PQ_TEST(wavelet, db4_rejects_fundamental_haar_does_not) {
    const float fs = 10000.0f;
    auto x = sine(4096, fs, 50.0f, 325.0f);
    for (auto type : {WaveletType::Haar, WaveletType::Db4}) {
        StreamingDwt w;
        w.configure(type, 3);
        float maxd1 = 0;
        for (int k = 0; k < 4096; ++k) {
            const uint32_t pr = w.push(x[k]);
            if (k > 100 && (pr & 1)) maxd1 = std::fmax(maxd1, std::fabs(w.detail(1)));
        }
        PQ_INFO("%s: max |d1| on clean 50 Hz = %.4f V (%.4f %% of Vpk)", type == WaveletType::Haar ? "Haar" : "db4",
                maxd1, 100 * maxd1 / 325.0f);
        if (type == WaveletType::Haar) PQ_CHECK(maxd1 > 0.5f);
        else PQ_CHECK(maxd1 < 0.01f);
    }
}

PQ_TEST(wavelet, energy_is_preserved_by_orthogonal_transform) {
    // Parseval for the orthonormal DWT: sum x^2 = sum a_J^2 + sum_j sum d_j^2
    StreamingDwt w;
    w.configure(WaveletType::Haar, 1);
    Rng r(9);
    double ex = 0;
    for (int k = 0; k < 4096; ++k) {
        const float v = r.gauss();
        ex += v * v;
        w.push(v);
    }
    // level-1 detail energy of white noise = half the energy
    PQ_CHECK_NEAR(w.energy(1) / ex, 0.5, 0.05);
}

// ------------------------------------------------------------- transient
PQ_TEST(transient, detects_impulse_on_the_sample_it_occurs) {
    const float fs = 10000.0f;
    StreamingDwt w;
    w.configure(WaveletType::Db4, 3);
    TransientDetector t;
    TransientParams p{fs, 50.0f, 325.0f, 0.15f, 1.4f, 4.0f, 8.0f, 0.03f, true, true};
    t.configure(p);
    Rng r(2);
    int64_t start = -1;
    int ends = 0;
    for (int k = 0; k < 30000; ++k) {
        float v = 325.0f * std::sin(2 * M_PI * 50 * k / fs) + 0.3f * r.gauss();
        if (k >= 20000 && k < 20004) v += 200.0f;   // 400 us, 0.6 pu impulse
        const uint32_t pr = w.push(v);
        const auto res = t.process(v, k, fs / 50.0f, &w, pr);
        if (res == TransientDetector::kStarted && start < 0) start = k;
        if (res == TransientDetector::kStarted && k < 19990) PQ_CHECK(false);   // no false alarm
        if (res == TransientDetector::kEnded) ++ends;
    }
    PQ_INFO("impulse at n=20000 detected at n=%lld (sources 0x%x)", static_cast<long long>(start),
            t.info().sources);
    PQ_CHECK(start >= 20000 && start <= 20003);
    PQ_CHECK(ends == 1);   // the "echo" one cycle later must not re-trigger
}

PQ_TEST(transient, no_false_alarms_with_steady_harmonics_and_noise) {
    const float fs = 10000.0f;
    for (int wav = 0; wav < 2; ++wav) {
        StreamingDwt w;
        w.configure(WaveletType::Db4, 3);
        TransientDetector t;
        TransientParams p{fs, 50.0f, 325.0f, 0.15f, 1.4f, 4.0f, 8.0f, 0.03f, wav == 1, true};
        t.configure(p);
        Rng r(4);
        int starts = 0;
        for (int k = 0; k < 100000; ++k) {
            const double ph = 2 * M_PI * 50.05 * k / fs;
            const float v = 325.0f * (std::sin(ph) + 0.08 * std::sin(3 * ph + 1) + 0.05 * std::sin(5 * ph)) +
                            1.0f * r.gauss();
            const uint32_t pr = w.push(v);
            if (t.process(v, k, fs / 50.05f, &w, pr) == TransientDetector::kStarted) ++starts;
        }
        PQ_CHECK(starts == 0);
    }
}
