// Shared enums / small types used across the whole firmware.
#pragma once
#include <cstdint>

namespace pq {

// Order matters: indices 0..7 are ALSO the TinyML output classes. Do not reorder
// without retraining the model (ml/train_model.py reads the same order via
// the class list embedded in the generated model header).
enum class PqClass : uint8_t {
    Normal = 0,
    Sag,
    Swell,
    Interruption,
    Harmonic,
    Transient,
    FrequencyVariation,
    LoadSwitching,
    Combined,          // DSP/fusion only – not a model output
    Count
};

constexpr int kNumPqClasses = static_cast<int>(PqClass::Count);   // 9
constexpr int kNumMlClasses = 8;                                   // model outputs

inline constexpr uint16_t classBit(PqClass c) { return static_cast<uint16_t>(1u << static_cast<unsigned>(c)); }

const char* pqClassName(PqClass c);       // "VOLTAGE_SAG"
const char* pqClassShortName(PqClass c);  // "SAG"
bool        pqClassFromString(const char* s, PqClass& out);

// System-wide error / quality flags (bitmask). Reported in STATUS and attached
// to every metrics frame so the host can tell *why* a value is not trusted.
enum ErrorFlag : uint32_t {
    ERR_NONE            = 0,
    ERR_ADC_COMM        = 1u << 0,   // SPI/ADC not responding, bad ID, stuck DRDY
    ERR_DMA_OVERRUN     = 1u << 1,   // DMA ring lapped the consumer (data lost)
    ERR_BUFFER_OVERFLOW = 1u << 2,   // an inter-core queue was full (message lost)
    ERR_INVALID_SAMPLES = 1u << 3,   // CRC / status-word errors in frames
    ERR_SATURATION      = 1u << 4,   // ADC codes at full scale (clipping)
    ERR_MISSING_SAMPLES = 1u << 5,   // gap in the sample sequence
    ERR_ML_INIT         = 1u << 6,   // TinyML failed to initialise / spec mismatch
    ERR_COMM_TIMEOUT    = 1u << 7,   // host not draining serial / partial command
    ERR_CONFIG_INVALID  = 1u << 8,   // stored config rejected (CRC/version/range)
    ERR_FLASH           = 1u << 9,   // flash persistence failure
    ERR_FREQ_LOST       = 1u << 10,  // no valid zero crossings (frequency held)
    ERR_WATCHDOG_RESET  = 1u << 11,  // last reset was caused by the watchdog
};

}  // namespace pq
