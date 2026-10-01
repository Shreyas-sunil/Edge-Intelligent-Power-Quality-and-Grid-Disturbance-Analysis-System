// =============================================================================
//  Harmonic analysis & THD from a Hann-windowed real FFT.
//
//  The window is generally NOT synchronous with the fundamental (f drifts), so
//  the energy of each harmonic spreads over the Hann main lobe (+/-2 bins).
//  Harmonic h is therefore measured by summing |X[k]|^2 over the bins around
//  k_h = h*f0/df ("spectral grouping", as in IEC 61000-4-7), with f0 from the
//  zero-crossing estimator. Parseval then gives the RMS amplitude:
//
//        V_h,rms = sqrt( 2 * sum|X[k]|^2 / (N * sum w[n]^2) )
//
//  THD = sqrt(V2^2 + ... + VH^2) / V1 * 100 %.
// =============================================================================
#pragma once
#include <cmath>
#include <cstdint>

#include "dsp/fft.h"
#include "pq_build_config.h"

namespace pq {

struct SpectralPeak {
    float hz;
    float rms;
};

struct HarmonicResult {
    float f0;                                  // frequency used for grouping [Hz]
    float fft_peak_hz;                         // interpolated FFT fundamental peak [Hz]
    float mag[PQ_MAX_HARMONIC + 1];            // RMS of harmonic h ([1] = fundamental)
    float phase[PQ_REPORT_HARMONICS + 1];      // phase [rad] at window start
    int   h_max;                               // highest harmonic evaluated
    float thd_pct;                             // THD [%]
    float rms_total;                           // RMS from spectrum (Parseval)
    int   group_halfwidth;                     // bins summed each side
    SpectralPeak peaks[3];                     // strongest non-harmonic components
    int   npeaks;
    bool  valid;                               // fundamental present
};

class HarmonicAnalyzer {
public:
    bool init(int n, float fs);
    int   size() const { return n_; }
    float binHz() const { return df_; }
    // x: n samples. f0_hint <= 0 -> fundamental taken from the FFT peak.
    void analyze(const float* x, float f0_hint, float f_nominal, int max_h, HarmonicResult& out);
    const float* power() const { return pw_; }   // |X[k]|^2, k = 0..n/2
    void fftOnly(const float* x) { fft_.forward(x, spec_); }   // benchmark hook
    const float* spectrum() const { return spec_; }

private:
    float groupEnergy(float kc, int g) const;
    float rmsFromEnergy(float e) const { return std::sqrt(2.0f * e / norm_); }
    RealFft fft_;
    int   n_ = 0;
    float fs_ = 5000.0f, df_ = 1.0f, norm_ = 1.0f;
    float win_[PQ_FFT_MAX];
    float xw_[PQ_FFT_MAX];
    float spec_[PQ_FFT_MAX + 2];
    float pw_[PQ_FFT_MAX / 2 + 1];
};

}  // namespace pq
