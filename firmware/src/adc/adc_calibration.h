// =============================================================================
//  Software calibration: ADC counts -> engineering units (V, A).
//
//    v[n] = (counts_v[n] - v_offset) * v_gain
//    i[n] = (counts_i[n] - i_offset) * i_gain
//
//  plus phase alignment of the two channels (CT / divider phase error) by a
//  fractional-sample delay of whichever channel leads, and repair of samples
//  flagged invalid by the ADC backend (sample-and-hold of last good value).
//  The constants live in CalConfig (system/config.h), never in DSP code.
// =============================================================================
#pragma once
#include <cstdint>

#include "adc/adc_types.h"
#include "system/config.h"

namespace pq {

struct CalibratedBlock {
    uint64_t seq;
    uint64_t t_first_us;
    uint32_t n;
    uint16_t bad_count;
    uint16_t sat_count;
    uint8_t  flags;          // RawBlockFlags propagated
    float    v[kBlockSize];  // volts
    float    i[kBlockSize];  // amps
};

class Calibrator {
public:
    void configure(const CalConfig& cal, float fs, float f_nominal);
    void reset();
    void apply(const RawBlock& in, CalibratedBlock& out);

    // Raw-count statistics, used by `CALIBRATE OFFSET`.
    void   resetRawStats();
    double rawMean(int ch) const { return raw_n_ ? raw_sum_[ch] / static_cast<double>(raw_n_) : 0.0; }
    uint64_t rawCount() const { return raw_n_; }

    float phaseDelaySamples() const { return delay_; }   // signed: + delays current

private:
    static constexpr int kDelayLen = 64;  // max ~31 samples needed (see config limits)
    float delayed(float* line, float x);

    CalConfig cal_{};
    float     delay_ = 0.0f;      // signed delay in samples
    int       d_int_ = 0;
    float     d_frac_ = 0.0f;
    float     vline_[kDelayLen]{};
    float     iline_[kDelayLen]{};
    int       widx_ = 0;
    int32_t   last_good_[kAdcChannels]{};
    double    raw_sum_[kAdcChannels]{};
    uint64_t  raw_n_ = 0;
};

}  // namespace pq
