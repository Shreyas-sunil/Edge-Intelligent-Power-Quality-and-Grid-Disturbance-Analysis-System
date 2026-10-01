#include "adc/sim_adc.h"

#include <cstring>

namespace pq {

AdcStatus SimAdcBackend::init(const AdcHwConfig& cfg) {
    if (!(cfg.sample_rate_hz >= 1000.0f && cfg.sample_rate_hz <= PQ_MAX_SAMPLE_RATE))
        return AdcStatus::InvalidParam;
    fs_ = cfg.sample_rate_hz;
    gen_.configure(gen_.params(), fs_);
    std::memset(&diag_, 0, sizeof(diag_));
    return AdcStatus::Ok;
}

AdcStatus SimAdcBackend::start() {
    consumed_ = 0;
    last_due_ = 0;
    seq_ = 0;
    gen_.reset(0);
    const uint32_t period_us = static_cast<uint32_t>(1e6f * kBlockSize / fs_ + 0.5f);
    pacer_->start(period_us);
    t0_us_ = pacer_->startTimeUs();
    running_ = true;
    return AdcStatus::Ok;
}

void SimAdcBackend::stop() {
    pacer_->stop();
    running_ = false;
}

bool SimAdcBackend::readBlock(RawBlock& out) {
    if (!running_) return false;
    const uint32_t due = pacer_->blocksDue();
    if (stalled_) {                     // simulated dead ADC: data never arrives
        if (!stall_active_) {
            stall_active_ = true;
            stall_start_due_ = due;
        }
        consumed_ = due;
        gap_pending_ = true;
        if (!stall_timeout_ && due - stall_start_due_ >= 3) {   // 3 block periods
            stall_timeout_ = true;
            ++diag_.timeouts;
            diag_.last_error = AdcStatus::Timeout;
        }
        return false;
    }
    if (stall_active_) {                // recovered: account for the lost time
        const uint64_t lost = static_cast<uint64_t>(consumed_ - stall_start_due_) * kBlockSize;
        gen_.skip(lost);
        seq_ += lost;
        diag_.missing_samples += lost;
        stall_active_ = false;
        stall_timeout_ = false;
    }
    last_due_ = due;
    if (due == consumed_) return false;

    uint32_t behind = due - consumed_;
    if (behind > PQ_ACQ_RING_BLOCKS) {
        // The DMA ring would have overwritten unread blocks: drop them.
        const uint32_t lost = behind - 1;
        diag_.overruns += lost;
        diag_.missing_samples += static_cast<uint64_t>(lost) * kBlockSize;
        diag_.last_error = AdcStatus::Overrun;
        gen_.skip(static_cast<uint64_t>(lost) * kBlockSize);
        seq_ += static_cast<uint64_t>(lost) * kBlockSize;
        consumed_ += lost;
        gap_pending_ = true;
        behind = 1;
    }
    if (behind > diag_.ring_high_water) diag_.ring_high_water = behind;

    std::memset(out.bad, 0, sizeof(out.bad));
    out.seq = seq_;
    out.t_first_us = t0_us_ + static_cast<uint64_t>(static_cast<double>(seq_) * 1e6 / fs_);
    out.n = kBlockSize;
    out.flags = gap_pending_ ? RB_GAP_BEFORE : 0;
    gap_pending_ = false;
    out.sat_count = static_cast<uint16_t>(
        gen_.generateCounts(kBlockSize, out.ch[kChVoltage], out.ch[kChCurrent]));
    if (out.sat_count) {
        out.flags |= RB_SATURATED;
        diag_.saturated += out.sat_count;
    }
    out.bad_count = 0;
    if (bad_prob_ > 0.0f) {
        for (uint32_t k = 0; k < kBlockSize; ++k) {
            if (fault_rng_.uniform() < bad_prob_) {
                out.markBad(k);
                out.ch[kChVoltage][k] = static_cast<int32_t>(fault_rng_.next() >> 8);  // garbage
                ++out.bad_count;
            }
        }
        if (out.bad_count) {
            out.flags |= RB_HAS_BAD;
            diag_.crc_errors += out.bad_count;
        }
    }
    seq_ += kBlockSize;
    ++consumed_;
    ++diag_.blocks;
    return true;
}

AdcStatus SimAdcBackend::poll() {
    if (!running_) return AdcStatus::NotInitialized;
    return stall_timeout_ ? AdcStatus::Timeout : AdcStatus::Ok;
}

}  // namespace pq
