#include "dsp/thd.h"

#include <cstring>

#include "common/math_util.h"

namespace pq {

bool HarmonicAnalyzer::init(int n, float fs) {
    if (!fft_.init(n)) return false;
    n_ = n;
    fs_ = fs;
    df_ = fs / n;
    double s2 = 0.0;
    for (int k = 0; k < n; ++k) {
        // Periodic Hann: symmetric about n/2 -> linear-phase term is exactly pi*delta.
        const double w = 0.5 - 0.5 * std::cos(2.0 * M_PI * k / n);
        win_[k] = static_cast<float>(w);
        s2 += w * w;
    }
    norm_ = static_cast<float>(n * s2);
    return true;
}

float HarmonicAnalyzer::groupEnergy(float kc, int g) const {
    const int m = n_ / 2;
    const int k0 = static_cast<int>(std::lround(kc));
    float e = 0.0f;
    for (int k = k0 - g; k <= k0 + g; ++k)
        if (k >= 1 && k < m) e += pw_[k];
    return e;
}

void HarmonicAnalyzer::analyze(const float* x, float f0_hint, float f_nominal, int max_h,
                               HarmonicResult& out) {
    const int m = n_ / 2;
    for (int k = 0; k < n_; ++k) xw_[k] = x[k] * win_[k];
    fft_.forward(xw_, spec_);
    double tot = 0.0;
    for (int k = 0; k <= m; ++k) {
        pw_[k] = spec_[2 * k] * spec_[2 * k] + spec_[2 * k + 1] * spec_[2 * k + 1];
        tot += (k == 0 || k == m) ? pw_[k] : 2.0 * pw_[k];
    }
    out.rms_total = static_cast<float>(std::sqrt(tot / norm_));

    // --- FFT peak near nominal (cross-check / fallback for the fundamental)
    int kmin = static_cast<int>(0.7f * f_nominal / df_);
    int kmax = static_cast<int>(1.3f * f_nominal / df_ + 1.0f);
    if (kmin < 1) kmin = 1;
    if (kmax > m - 2) kmax = m - 2;
    int kp = kmin;
    for (int k = kmin; k <= kmax; ++k)
        if (pw_[k] > pw_[kp]) kp = k;
    float delta = 0.0f;
    if (kp > 0 && kp < m) {
        const float a = std::log(pw_[kp - 1] + 1e-30f), b = std::log(pw_[kp] + 1e-30f),
                    c = std::log(pw_[kp + 1] + 1e-30f);
        const float den = a - 2.0f * b + c;
        if (den < 0.0f) delta = clampv(0.5f * (a - c) / den, -0.5f, 0.5f);
    }
    out.fft_peak_hz = (kp + delta) * df_;

    const float f0 = f0_hint > 0.0f ? f0_hint : out.fft_peak_hz;
    out.f0 = f0;
    const float spacing = f0 / df_;
    int g = static_cast<int>((spacing - 1.0f) * 0.5f);
    g = g < 0 ? 0 : (g > 2 ? 2 : g);
    out.group_halfwidth = g;

    int hmax = static_cast<int>(((m - 1 - g) * df_) / f0);
    if (hmax > max_h) hmax = max_h;
    if (hmax > PQ_MAX_HARMONIC) hmax = PQ_MAX_HARMONIC;
    out.h_max = hmax;

    std::memset(out.mag, 0, sizeof(out.mag));
    std::memset(out.phase, 0, sizeof(out.phase));
    double hsum = 0.0;
    for (int h = 1; h <= hmax; ++h) {
        const float kc = h * spacing;
        out.mag[h] = rmsFromEnergy(groupEnergy(kc, g));
        if (h <= PQ_REPORT_HARMONICS) {
            const int k0 = static_cast<int>(std::lround(kc));
            const float d = kc - static_cast<float>(k0);
            float ph = std::atan2(spec_[2 * k0 + 1], spec_[2 * k0]) - kPi * d;
            // X is a cosine phasor; report sine-referenced phase (v = sqrt2 V sin(wt + ph)).
            ph += 0.5f * kPi;
            while (ph > kPi) ph -= kTwoPi;
            while (ph < -kPi) ph += kTwoPi;
            out.phase[h] = ph;
        }
        if (h >= 2) hsum += static_cast<double>(out.mag[h]) * out.mag[h];
    }
    out.valid = out.mag[1] > 1e-6f && out.mag[1] > 0.05f * out.rms_total;
    out.thd_pct = out.valid ? static_cast<float>(std::sqrt(hsum)) / out.mag[1] * 100.0f : 0.0f;

    // --- strongest non-harmonic peaks (interharmonics / HF components)
    out.npeaks = 0;
    for (auto& p : out.peaks) p = SpectralPeak{0.0f, 0.0f};
    const float floor_e = 1e-6f * pw_[static_cast<int>(std::lround(spacing))];   // -60 dB
    for (int k = 3; k < m - 1; ++k) {
        if (!(pw_[k] > pw_[k - 1] && pw_[k] >= pw_[k + 1] && pw_[k] > floor_e)) continue;
        const float hk = k / spacing;
        const float nearest = std::round(hk);
        if (nearest >= 1.0f && std::fabs(k - nearest * spacing) <= g + 1.0f) continue;
        const float r = rmsFromEnergy(groupEnergy(static_cast<float>(k), g));
        int slot;
        if (out.npeaks < 3) slot = out.npeaks++;
        else if (r > out.peaks[2].rms) slot = 2;
        else continue;
        const float a = std::log(pw_[k - 1] + 1e-30f), b = std::log(pw_[k] + 1e-30f),
                    c = std::log(pw_[k + 1] + 1e-30f);
        const float den = a - 2.0f * b + c;
        const float dd = den < 0.0f ? clampv(0.5f * (a - c) / den, -0.5f, 0.5f) : 0.0f;
        out.peaks[slot] = SpectralPeak{(k + dd) * df_, r};
        while (slot > 0 && out.peaks[slot - 1].rms < out.peaks[slot].rms) {   // keep sorted
            const SpectralPeak t = out.peaks[slot - 1];
            out.peaks[slot - 1] = out.peaks[slot];
            out.peaks[slot] = t;
            --slot;
        }
    }
}

}  // namespace pq
