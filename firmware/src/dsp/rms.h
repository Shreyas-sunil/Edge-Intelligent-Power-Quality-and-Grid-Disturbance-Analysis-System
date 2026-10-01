// =============================================================================
//  Cycle-synchronised RMS, refreshed every half cycle (IEC 61000-4-30 Urms(1/2)).
//
//  Windows are delimited by the fundamental zero crossings from the
//  FrequencyEstimator. The crossing instant is fractional; the boundary sample
//  is split between the two half-cycles in proportion to that fraction, so a
//  "one-cycle" window is exactly one period long (to ~1e-3 sample) and the RMS
//  has no +/-1-sample jitter error. If crossings disappear (interruption), the
//  window is closed on a timeout of one nominal half period so RMS keeps
//  updating — that is what makes interruptions detectable within one cycle.
// =============================================================================
#pragma once
#include <cstdint>

#include "dsp/frequency.h"

namespace pq {

struct HalfCycleResult {
    float  v_rms;        // RMS over the last full cycle (two halves)
    float  i_rms;
    float  p;            // mean(v*i) over the last cycle [W]
    float  v_rms_half;   // RMS of this half only
    double t_start;      // cycle window start [samples, fractional]
    double t_end;        // cycle window end   [samples, fractional]
    float  e_v, e_i, len;  // this half: sum v^2, sum i^2, length [samples]
    bool   synced;       // both halves delimited by zero crossings (false during
                         // interruptions / at re-synchronisation)
};

class HalfCycleRms {
public:
    void configure(float fs, float f_nominal);
    void reset();
    // Returns true when a half cycle completed (result in `out`).
    bool process(float v, float i, int64_t n, const CrossingEvent& ev, HalfCycleResult& out);

private:
    struct Acc {           // float is enough: <= ~300 terms of O(1e5)
        float ev = 0, ei = 0, evi = 0, len = 0;
    };
    void close(double t_boundary, bool synced, HalfCycleResult& out);

    float  half_nom_ = 100.0f;
    float  min_half_ = 30.0f;
    Acc    cur_{}, prev_{};
    bool   have_prev_ = false;
    double t_seg_start_ = 0.0;
    double prev_start_ = 0.0;
    float  last_v2_ = 0, last_i2_ = 0, last_vi_ = 0;
    bool   started_ = false;
    bool   timeout_mode_ = false;
    bool   prev_synced_ = false;
};

}  // namespace pq
