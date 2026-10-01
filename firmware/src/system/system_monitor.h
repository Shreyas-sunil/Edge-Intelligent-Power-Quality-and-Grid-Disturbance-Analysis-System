// System health & timing statistics (supervisor side).
#pragma once
#include <cstdint>

#include "app/pq_engine.h"
#include "app/rt_link.h"

namespace pq {

struct StageStat {
    uint32_t n = 0;
    uint64_t sum = 0;
    uint32_t max = 0;
    void add(uint32_t v) {
        ++n;
        sum += v;
        if (v > max) max = v;
    }
    float avg() const { return n ? static_cast<float>(sum) / n : 0.0f; }
};

struct TimingStats {
    StageStat per_sample;   // streaming per-sample stages, accumulated per hop
    StageStat spectral, window, detect, feature, ml_total, ml_invoke, fusion, frame;
    StageStat e2e;          // per_sample + frame: total processing per hop
    void add(const FrameResult& f);
    void reset() { *this = TimingStats{}; }
};

// Error bits that make measurements untrustworthy right now -> fault indication.
constexpr uint32_t kFaultMask = ERR_ADC_COMM | ERR_CONFIG_INVALID;

inline bool isFault(uint32_t errors_now) { return (errors_now & kFaultMask) != 0; }

}  // namespace pq
