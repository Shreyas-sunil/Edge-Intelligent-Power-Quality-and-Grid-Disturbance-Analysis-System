#include "dsp/frequency.h"

#include <cmath>

namespace pq {

void FrequencyEstimator::configure(float fs, float f_nominal, float lpf_factor, int avg_cycles,
                                   float v_peak_nominal) {
    fs_ = fs;
    fnom_ = f_nominal;
    avg_ = avg_cycles < 1 ? 1 : (avg_cycles > kMaxAvg ? kMaxAvg : avg_cycles);
    lpf_.designButterworthLowpass(4, lpf_factor * f_nominal, fs);
    lpf_gain_ = lpf_.magnitude(f_nominal, fs);
    // Envelope follower: ~2 cycle decay time constant.
    env_decay_ = std::exp(-f_nominal / (2.0f * fs));
    // Absolute hysteresis floor: 2 % of nominal peak — below that we refuse to
    // detect crossings (noise during an interruption must not look like 50 Hz).
    h_min_ = 0.02f * v_peak_nominal * lpf_gain_;
    reset();
}

void FrequencyEstimator::reset() {
    lpf_.reset();
    env_ = 0.0f;
    y_prev_ = 0.0f;
    arm_rise_ = arm_fall_ = false;
    nh_ = 0;
    last_rise_n_ = -1;
    valid_ = false;
    freq_ = fcycle_ = fnom_;
    rocof_ = 0.0f;
    cycles_ = 0;
    settle_ = static_cast<int>(3.0f * fs_ / fnom_);   // let the LPF settle (3 cycles)
}

CrossingEvent FrequencyEstimator::process(float v, int64_t n) {
    CrossingEvent ev{false, false, 0.0f};
    const float y = lpf_.process(v);
    if (settle_ > 0) {
        --settle_;
        y_prev_ = y;
        return ev;
    }
    const float ay = std::fabs(y);
    env_ = ay > env_ ? ay : env_ * env_decay_;
    float h = 0.1f * env_;
    if (h < h_min_) h = h_min_;

    if (y < -h) arm_rise_ = true;
    if (y > h) arm_fall_ = true;

    if (arm_rise_ && y_prev_ < 0.0f && y >= 0.0f) {
        arm_rise_ = false;
        ev.rising = true;
        ev.frac = -y_prev_ / (y - y_prev_);
        const double t = static_cast<double>(n - 1) + ev.frac;

        // Store rising-crossing time; validate the period.
        if (nh_ > 0) {
            const double period = t - hist_[nh_ - 1];
            const double f = fs_ / period;
            if (f < 0.8 * fnom_ || f > 1.2 * fnom_) {
                nh_ = 0;         // implausible: restart averaging
                valid_ = false;
            } else {
                const float prev = fcycle_;
                fcycle_ = static_cast<float>(f);
                if (valid_) {
                    const float r = (fcycle_ - prev) * fcycle_;   // Hz/s
                    rocof_ = 0.8f * rocof_ + 0.2f * r;
                }
                ++cycles_;
            }
        }
        if (nh_ == avg_ + 1) {
            for (int k = 0; k < avg_; ++k) hist_[k] = hist_[k + 1];
            nh_ = avg_;
        }
        hist_[nh_++] = t;
        if (nh_ >= 2) {
            freq_ = static_cast<float>((nh_ - 1) * static_cast<double>(fs_) / (hist_[nh_ - 1] - hist_[0]));
            valid_ = true;
        }
        last_rise_n_ = n;
    } else if (arm_fall_ && y_prev_ > 0.0f && y <= 0.0f) {
        arm_fall_ = false;
        ev.falling = true;
        ev.frac = y_prev_ / (y_prev_ - y);
    }

    // Loss of signal: no rising crossing for 2.5 nominal periods.
    if (valid_ && last_rise_n_ >= 0 && (n - last_rise_n_) > static_cast<int64_t>(2.5f * fs_ / fnom_)) {
        valid_ = false;
        nh_ = 0;
        rocof_ = 0.0f;
    }
    y_prev_ = y;
    return ev;
}

}  // namespace pq
