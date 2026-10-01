#include "dsp/transient.h"

#include <cmath>
#include <cstring>

#include "common/math_util.h"

namespace pq {

void TransientDetector::configure(const TransientParams& p) {
    p_ = p;
    thr_amp_ = p.impulse_pu * p.vpk_nominal;
    thr_dvdt_ = p.dvdt_factor * kTwoPi * p.f_nominal * p.vpk_nominal;
    thr_dev_ = p.dev_pu * p.vpk_nominal;
    thr_wfloor_ = p.wavelet_floor_pu * p.vpk_nominal;
    reset();
}

void TransientDetector::reset() {
    std::memset(hist_, 0, sizeof(hist_));
    count_ = 0;
    v_prev_ = 0.0f;
    active_ = false;
    verifying_ = false;
    rebase_ = 0;
    quiet_ = 0;
    settle_ = static_cast<int64_t>(10.0f * p_.fs / p_.f_nominal);   // 10 cycles
    for (int j = 0; j < StreamingDwt::kMaxLevels; ++j) {
        wfloor_[j] = thr_wfloor_ / (p_.wavelet_k > 0 ? p_.wavelet_k : 1.0f);
        wthr_[j] = thr_wfloor_;
    }
    for (int k = 0; k < 4; ++k) echo_a_[k] = echo_b_[k] = -1000000;
    echo_i_ = 0;
    info_ = TransientInfo{};
    score_ = 0.0f;
    dev_energy_ = 0.0f;
}

float TransientDetector::histAt(double idx) const {
    const int64_t a = static_cast<int64_t>(std::floor(idx));
    const float f = static_cast<float>(idx - static_cast<double>(a));
    return hist_[a & (kHist - 1)] * (1.0f - f) + hist_[(a + 1) & (kHist - 1)] * f;
}

bool TransientDetector::inEcho(int64_t n, float period) const {
    const int64_t P = static_cast<int64_t>(period + 0.5f);
    for (int k = 0; k < 4; ++k)
        if (n >= echo_a_[k] + P - 3 && n <= echo_b_[k] + P + 3) return true;
    return false;
}

TransientDetector::Result TransientDetector::process(float v, int64_t n, float period,
                                                     const StreamingDwt* dwt, uint32_t produced) {
    Result res = kNone;
    // --- cycle-to-cycle deviation (fractional delay by linear interpolation)
    float dev = 0.0f;
    if (count_ > static_cast<int64_t>(period) + 2 && period > 2.0f && period < kHist - 2) {
        const float t = static_cast<float>(count_) - period;
        const int64_t a = static_cast<int64_t>(std::floor(t));
        const float f = t - static_cast<float>(a);
        const float ref = hist_[a & (kHist - 1)] * (1.0f - f) + hist_[(a + 1) & (kHist - 1)] * f;
        dev = v - ref;
    }
    hist_[count_ & (kHist - 1)] = v;
    ++count_;

    // --- post-event verification: compare one cycle after the event with the
    // same phase of a cycle BEFORE the event started.
    if (verifying_) {
        const double back = std::ceil(static_cast<double>(n - info_.n_start + 1) / period) * period;
        if (back + 2.0 < kHist - 2) {
            const float ref = histAt(static_cast<double>(count_ - 1) - back);
            const float d = v - ref;
            ev_diff_ += static_cast<double>(d) * d;
            ev_ref_ += static_cast<double>(ref) * ref;
        }
        if (--verify_left_ <= 0) {
            verifying_ = false;
            info_.residual = ev_ref_ > 0.0 ? static_cast<float>(std::sqrt(ev_diff_ / ev_ref_)) : 0.0f;
            info_.step = info_.residual > 0.05f;
            // Persistent waveform change: the HF background (wavelet floors) is
            // now different -> re-learn it quickly over 5 cycles.
            if (info_.step) rebase_ = static_cast<int64_t>(5.0f * period);
            res = kEnded;
        }
    }

    const bool echo = inEcho(n, period);
    if (echo) dev = 0.0f;
    dev_energy_ += dev * dev;

    const float dvdt = std::fabs(v - v_prev_) * p_.fs;
    v_prev_ = v;

    float s = std::fabs(v) / thr_amp_;
    uint8_t src = s >= 1.0f ? TS_AMPLITUDE : 0;
    const float sd = dvdt / thr_dvdt_;
    if (sd >= 1.0f) src |= TS_DVDT;
    if (sd > s) s = sd;
    const float sdev = std::fabs(dev) / thr_dev_;
    if (sdev >= 1.0f) src |= TS_CYCLE_DEV;
    if (sdev > s) s = sdev;

    if (p_.use_wavelet && dwt && produced) {
        // Floors track the steady-state background. While a transient is active
        // they are frozen, except when the "transient" outlasts one cycle: then
        // it is a steady-state waveform change (e.g. new harmonic load) and the
        // floors re-baseline so the detector cannot latch up.
        const bool long_active = active_ && (n - info_.n_start) > static_cast<int64_t>(period);
        const bool rebasing = rebase_ > 0;
        const float alpha = (settle_ > 0 || long_active || rebasing) ? 0.02f : 0.001f;
        for (int j = 0; j < dwt->levels(); ++j) {
            if (!(produced & (1u << j))) continue;
            const float ad = std::fabs(dwt->detail(j + 1));
            if (!active_ || long_active || rebasing) wfloor_[j] += alpha * (ad - wfloor_[j]);
            float thr = p_.wavelet_k * wfloor_[j];
            if (thr < thr_wfloor_) thr = thr_wfloor_;
            wthr_[j] = thr;
            const float sw = rebasing ? 0.0f : ad / thr;   // wavelet muted while re-baselining
            if (sw >= 1.0f) src |= TS_WAVELET;
            if (sw > s) s = sw;
        }
    }
    score_ = s;
    if (rebase_ > 0) --rebase_;

    if (settle_ > 0) {
        --settle_;
        return kNone;
    }
    if (!p_.enable) return kNone;

    if (!active_) {
        if (s >= 1.0f && !verifying_) {
            active_ = true;
            info_ = TransientInfo{};
            info_.n_start = n;
            info_.n_end = n;
            info_.sources = src;
            info_.peak_score = s;
            info_.peak_abs_v = std::fabs(v);
            info_.peak_dev = std::fabs(dev);
            info_.peak_dvdt = dvdt;
            quiet_ = 0;
            res = kStarted;
        }
    } else {
        info_.sources |= src;
        if (s > info_.peak_score) info_.peak_score = s;
        if (std::fabs(v) > info_.peak_abs_v) info_.peak_abs_v = std::fabs(v);
        if (std::fabs(dev) > info_.peak_dev) info_.peak_dev = std::fabs(dev);
        if (dvdt > info_.peak_dvdt) info_.peak_dvdt = dvdt;
        if (s >= 0.5f) {
            quiet_ = 0;
            info_.n_end = n;
        } else if (++quiet_ > static_cast<int64_t>(0.002f * p_.fs)) {   // 2 ms quiet
            active_ = false;
            echo_a_[echo_i_] = info_.n_start;
            echo_b_[echo_i_] = info_.n_end;
            echo_i_ = (echo_i_ + 1) & 3;
            // kEnded is reported after one verification cycle.
            verifying_ = true;
            verify_left_ = static_cast<int64_t>(period + 0.5f);
            ev_diff_ = ev_ref_ = 0.0;
        }
    }
    return res;
}

}  // namespace pq
