// ADC data types shared by every backend (hardware and simulator).
#pragma once
#include <cstdint>

#include "pq_build_config.h"

namespace pq {

constexpr int      kAdcChannels = 2;          // add channels here (e.g. 3-phase)
constexpr uint32_t kBlockSize   = PQ_BLOCK_SIZE;
enum AdcChannel : uint8_t { kChVoltage = 0, kChCurrent = 1 };

enum class AdcStatus : uint8_t {
    Ok = 0,
    NotInitialized,
    CommFailure,      // SPI transaction did not return the expected response
    BadId,            // device ID register mismatch
    ConfigMismatch,   // register read-back differs from what was written
    Timeout,          // no data-ready within the expected time
    Overrun,          // DMA ring overwritten before the consumer read it
    InvalidParam,
    Unsupported,
};
const char* adcStatusName(AdcStatus s);

// Block flags
enum RawBlockFlags : uint8_t {
    RB_GAP_BEFORE = 1u << 0,   // samples were lost immediately before this block
    RB_HAS_BAD    = 1u << 1,   // at least one sample failed CRC/status validation
    RB_SATURATED  = 1u << 2,   // at least one sample at ADC full scale
};

// One acquisition block. Planar (structure-of-arrays) layout: the DSP walks each
// channel contiguously, the DMA ring stays compact, and adding a channel is just
// a larger kAdcChannels. Timestamps are derived from the sample counter (exact,
// jitter-free) rather than stored per sample; t(n) = t_first_us + n / fs.
struct RawBlock {
    uint64_t seq;                         // absolute index of the first sample
    uint64_t t_first_us;                  // time of the first sample
    uint32_t n;                           // valid samples (== kBlockSize normally)
    uint16_t bad_count;                   // samples flagged invalid (CRC / status)
    uint16_t sat_count;                   // samples at full scale
    uint8_t  flags;                       // RawBlockFlags
    uint8_t  bad[(kBlockSize + 7) / 8];   // per-sample invalid bitmap
    int32_t  ch[kAdcChannels][kBlockSize];

    bool isBad(uint32_t i) const { return (bad[i >> 3] >> (i & 7)) & 1u; }
    void markBad(uint32_t i) { bad[i >> 3] |= static_cast<uint8_t>(1u << (i & 7)); }
};

struct AdcHwConfig {
    float   sample_rate_hz;
    uint8_t pga_gain[kAdcChannels];
    bool    crc_check;
};

struct AdcDiagnostics {
    uint32_t blocks;          // blocks delivered
    uint32_t overruns;        // DMA ring overruns (blocks lost)
    uint64_t missing_samples; // samples lost in total
    uint32_t crc_errors;      // frames with CRC mismatch
    uint32_t status_errors;   // frames with unexpected status word
    uint32_t saturated;       // samples at full scale
    uint32_t timeouts;        // data-ready watchdog expiries
    uint32_t reinit_count;    // automatic recoveries
    uint32_t ring_high_water; // max blocks waiting in the ring
    AdcStatus last_error;
};

}  // namespace pq
