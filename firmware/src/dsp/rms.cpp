#include "dsp/rms.h"

#include <cmath>

namespace pq {

void HalfCycleRms::configure(float fs, float f_nominal) {
    half_nom_ = fs / (2.0f * f_nominal);
    min_half_ = 0.3f * half_nom_;
    reset();
}

void HalfCycleRms::reset() {
    cur_ = Acc{};
    prev_ = Acc{};
    have_prev_ = false;
    t_seg_start_ = 0.0;
    prev_start_ = 0.0;
    last_v2_ = last_i2_ = last_vi_ = 0.0f;
    started_ = false;
    timeout_mode_ = false;
    prev_synced_ = false;
}

void HalfCycleRms::close(double t_boundary, bool synced, HalfCycleResult& out) {
    const float lc = cur_.len > 1e-3f ? cur_.len : 1e-3f;
    if (have_prev_) {
        const float l = prev_.len + cur_.len;
        out.v_rms = std::sqrt((prev_.ev + cur_.ev) / l);
        out.i_rms = std::sqrt((prev_.ei + cur_.ei) / l);
        out.p = (prev_.evi + cur_.evi) / l;
        out.t_start = prev_start_;
    } else {
        out.v_rms = std::sqrt(cur_.ev / lc);
        out.i_rms = std::sqrt(cur_.ei / lc);
        out.p = cur_.evi / lc;
        out.t_start = t_seg_start_;
    }
    out.v_rms_half = std::sqrt(cur_.ev / lc);
    out.t_end = t_boundary;
    out.e_v = cur_.ev;
    out.e_i = cur_.ei;
    out.len = cur_.len;
    // A one-cycle value is trustworthy only if BOTH halves were delimited by
    // real zero crossings (a partial half cycle at re-sync reads high/low).
    out.synced = synced && prev_synced_;
    prev_synced_ = synced;

    prev_ = cur_;
    have_prev_ = true;
    prev_start_ = t_seg_start_;
    t_seg_start_ = t_boundary;
    cur_ = Acc{};
}

bool HalfCycleRms::process(float v, float i, int64_t n, const CrossingEvent& ev, HalfCycleResult& out) {
    bool emitted = false;
    if (!started_) {
        t_seg_start_ = static_cast<double>(n);
        started_ = true;
    }
    if ((ev.rising || ev.falling)) {
        const float move = 1.0f - ev.frac;   // part of sample n-1 after the crossing
        if (cur_.len - move >= min_half_) {
            cur_.ev -= move * last_v2_;
            cur_.ei -= move * last_i2_;
            cur_.evi -= move * last_vi_;
            cur_.len -= move;
            close(static_cast<double>(n - 1) + ev.frac, true, out);
            cur_.ev = move * last_v2_;
            cur_.ei = move * last_i2_;
            cur_.evi = move * last_vi_;
            cur_.len = move;
            timeout_mode_ = false;
            emitted = true;
        }
    }
    last_v2_ = v * v;
    last_i2_ = i * i;
    last_vi_ = v * i;
    cur_.ev += last_v2_;
    cur_.ei += last_i2_;
    cur_.evi += last_vi_;
    cur_.len += 1.0f;

    // Timeout: no crossing -> close after 1.25 nominal half cycles, so RMS keeps
    // updating (80/s) during an interruption. The 25 % margin guarantees that
    // real crossings, when present, always win and re-synchronise the windows.
    const float limit = 1.25f * half_nom_;
    if (!emitted && cur_.len >= limit) {
        close(static_cast<double>(n + 1), false, out);
        timeout_mode_ = true;
        emitted = true;
    }
    return emitted;
}

}  // namespace pq
