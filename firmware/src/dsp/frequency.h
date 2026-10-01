// =============================================================================
//  Frequency estimation by interpolated zero crossings.
//
//  Chosen over FFT-peak because it costs ~10 operations per sample, gives a new
//  estimate every cycle (FFT: every hop), and with linear interpolation of the
//  crossing instant reaches ~1 mHz resolution at 10 kS/s — far better than the
//  FFT bin width (4.9 Hz) without zoom/interpolation tricks. The FFT peak
//  (parabolic-interpolated) is still computed as a cross-check in harmonics.
//
//  Signal path: v -> 4th-order Butterworth LPF (1.5 x f_nom) -> hysteresis
//  comparator (10 % of tracked amplitude) -> linear interpolation of the
//  crossing instant -> frequency = K / (t_K - t_0) over the last K cycles.
//  The LPF delay is constant at a given f, so it cancels in period differences.
// =============================================================================
#pragma once
#include <cstdint>

#include "dsp/filtering.h"

namespace pq {

struct CrossingEvent {
    bool   rising;
    bool   falling;
    float  frac;   // crossing is at (n - 1 + frac) where n = current sample index
};

class FrequencyEstimator {
public:
    static constexpr int kMaxAvg = 64;

    void configure(float fs, float f_nominal, float lpf_factor, int avg_cycles, float v_peak_nominal);
    void reset();

    // Process one sample; `n` is the absolute sample index.
    CrossingEvent process(float v, int64_t n);

    bool  valid() const { return valid_; }
    float frequency() const { return freq_; }        // averaged over K cycles
    float cycleFrequency() const { return fcycle_; } // last cycle only
    float rocof() const { return rocof_; }           // Hz/s, smoothed
    float periodSamples() const { return valid_ ? fs_ / freq_ : fs_ / fnom_; }
    float lpfGainAtNominal() const { return lpf_gain_; }
    uint32_t cyclesSeen() const { return cycles_; }

private:
    BiquadCascade lpf_;
    float   fs_ = 10000.0f, fnom_ = 50.0f;
    int     avg_ = 5;
    float   h_min_ = 1.0f;
    float   env_ = 0.0f, env_decay_ = 0.999f;
    float   y_prev_ = 0.0f;
    bool    arm_rise_ = false, arm_fall_ = false;
    double  hist_[kMaxAvg + 1];
    int     nh_ = 0;
    int64_t last_rise_n_ = -1;
    bool    valid_ = false;
    float   freq_ = 50.0f, fcycle_ = 50.0f, rocof_ = 0.0f;
    float   lpf_gain_ = 1.0f;
    uint32_t cycles_ = 0;
    int     settle_ = 0;
};

}  // namespace pq
