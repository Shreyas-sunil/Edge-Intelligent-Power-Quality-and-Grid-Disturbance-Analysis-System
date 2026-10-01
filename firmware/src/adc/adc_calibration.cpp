#include "adc/adc_calibration.h"

#include <cmath>
#include <cstring>

namespace pq {

void Calibrator::configure(const CalConfig& cal, float fs, float f_nominal) {
    cal_ = cal;
    // Phase error phi (deg) at f_nominal corresponds to a time shift phi/360/f.
    delay_ = cal.phase_deg / 360.0f * fs / f_nominal;
    float mag = std::fabs(delay_);
    if (mag > kDelayLen - 2) mag = kDelayLen - 2;
    d_int_ = static_cast<int>(mag);
    d_frac_ = mag - static_cast<float>(d_int_);
}

void Calibrator::reset() {
    std::memset(vline_, 0, sizeof(vline_));
    std::memset(iline_, 0, sizeof(iline_));
    widx_ = 0;
    last_good_[0] = last_good_[1] = 0;
}

void Calibrator::resetRawStats() {
    raw_sum_[0] = raw_sum_[1] = 0.0;
    raw_n_ = 0;
}

// Fractional delay by linear interpolation. At 50 Hz / 10 kHz the interpolation
// low-pass effect is < 0.001 % at the fundamental and < 0.2 % at H40.
float Calibrator::delayed(float* line, float x) {
    line[widx_] = x;
    int a = widx_ - d_int_;
    if (a < 0) a += kDelayLen;
    int b = a - 1;
    if (b < 0) b += kDelayLen;
    return (1.0f - d_frac_) * line[a] + d_frac_ * line[b];
}

void Calibrator::apply(const RawBlock& in, CalibratedBlock& out) {
    out.seq = in.seq;
    out.t_first_us = in.t_first_us;
    out.n = in.n;
    out.bad_count = in.bad_count;
    out.sat_count = in.sat_count;
    out.flags = in.flags;

    const bool delay_current = delay_ > 0.0f;   // current leads -> delay current
    const bool no_delay = (d_int_ == 0 && d_frac_ < 1e-4f);

    for (uint32_t k = 0; k < in.n; ++k) {
        int32_t cv = in.ch[kChVoltage][k];
        int32_t ci = in.ch[kChCurrent][k];
        if (in.isBad(k)) {        // hold last good sample (isolated CRC errors)
            cv = last_good_[kChVoltage];
            ci = last_good_[kChCurrent];
        } else {
            last_good_[kChVoltage] = cv;
            last_good_[kChCurrent] = ci;
            raw_sum_[kChVoltage] += cv;
            raw_sum_[kChCurrent] += ci;
            ++raw_n_;
        }
        const float v = (static_cast<float>(cv) - cal_.v_offset) * cal_.v_gain;
        const float i = (static_cast<float>(ci) - cal_.i_offset) * cal_.i_gain;

        if (no_delay) {
            out.v[k] = v;
            out.i[k] = i;
        } else if (delay_current) {
            out.v[k] = v;
            out.i[k] = delayed(iline_, i);
        } else {
            out.v[k] = delayed(vline_, v);
            out.i[k] = i;
        }
        if (!no_delay) widx_ = (widx_ + 1) % kDelayLen;
    }
}

}  // namespace pq
