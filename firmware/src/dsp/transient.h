// =============================================================================
//  Lightweight sample-by-sample transient detector. Four cheap detectors are
//  OR-ed; each is normalised to a "score" (>= 1 means triggered):
//
//   1. amplitude    |v| > impulse_pu * Vpk_nom
//   2. dv/dt        |v[n]-v[n-1]|*fs > k * (2*pi*f * Vpk_nom)
//   3. cycle dev.   |v[n] - v[n-T]| > dev_pu * Vpk_nom   (T = measured period,
//                   fractional). Steady harmonics cancel; only *changes* remain.
//                   The "echo" one period after a transient is blanked.
//   4. wavelet      |d_j| > max(K * adaptive_floor_j, floor_pu * Vpk)   (optional)
//
//  Detection happens on the very sample that crosses (no windowing delay).
// =============================================================================
#pragma once
#include <cstdint>

#include "dsp/wavelet.h"

namespace pq {

enum TransientSource : uint8_t {
    TS_AMPLITUDE = 1u << 0,
    TS_DVDT      = 1u << 1,
    TS_CYCLE_DEV = 1u << 2,
    TS_WAVELET   = 1u << 3,
};

struct TransientParams {
    float fs, f_nominal, vpk_nominal;
    float dev_pu, impulse_pu, dvdt_factor;
    float wavelet_k, wavelet_floor_pu;
    bool  use_wavelet;
    bool  enable;
};

struct TransientInfo {
    int64_t n_start;
    int64_t n_end;
    float   peak_abs_v;    // V
    float   peak_dev;      // V
    float   peak_dvdt;     // V/s
    float   peak_score;
    uint8_t sources;       // TransientSource mask
    bool    step;          // waveform did NOT return to its pre-event shape
                           // (sag/swell onset, phase jump, harmonic load switched in)
    float   residual;      // rms(v_after - v_before) / rms(v_before) over one cycle
};

class TransientDetector {
public:
    enum Result : uint8_t { kNone = 0, kStarted = 1, kEnded = 2 };
    static constexpr int kHist = 2048;   // >= 5 periods at the highest sample rate

    void configure(const TransientParams& p);
    void reset();

    // v: conditioned voltage, n: absolute index, period: samples per cycle,
    // dwt/produced: wavelet state after pushing this sample (may be null).
    Result process(float v, int64_t n, float period, const StreamingDwt* dwt, uint32_t produced);

    bool  active() const { return active_; }
    const TransientInfo& info() const { return info_; }
    float lastScore() const { return score_; }

    // Frame accumulators (for features): sum of dev^2 excluding blanked echoes.
    float devEnergy() const { return dev_energy_; }
    void  clearEnergy() { dev_energy_ = 0.0f; }

private:
    bool inEcho(int64_t n, float period) const;
    float histAt(double idx) const;   // fractional index into the history ring
    TransientParams p_{};
    float   hist_[kHist];
    int64_t count_ = 0;
    float   v_prev_ = 0.0f;
    float   thr_amp_ = 1.0f, thr_dvdt_ = 1.0f, thr_dev_ = 1.0f, thr_wfloor_ = 1.0f;
    float   wfloor_[StreamingDwt::kMaxLevels]{};
    float   wthr_[StreamingDwt::kMaxLevels]{};
    bool    active_ = false;
    bool    verifying_ = false;
    int64_t verify_left_ = 0;
    double  ev_diff_ = 0.0, ev_ref_ = 0.0;
    int64_t quiet_ = 0;
    int64_t rebase_ = 0;
    int64_t settle_ = 0;
    int64_t echo_a_[4]{}, echo_b_[4]{};
    int     echo_i_ = 0;
    TransientInfo info_{};
    float   score_ = 0.0f;
    float   dev_energy_ = 0.0f;
};

}  // namespace pq
