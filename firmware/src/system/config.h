// =============================================================================
//  Centralised run-time configuration.
//
//  * Defaults come from config/pq_build_config.h (compile time).
//  * Every field is reachable by a dotted key through the parameter table, so
//    `CONFIG GET pq.sag_threshold` / `CONFIG SET pq.sag_threshold 0.88` work
//    without any per-field code.
//  * The whole struct (plus magic/version/CRC) is what gets persisted in flash.
// =============================================================================
#pragma once
#include <cstddef>
#include <cstdint>

#include "pq_build_config.h"

namespace pq {

enum class AdcSource : uint8_t { Hardware = 0, Simulated = 1 };

struct AcqConfig {
    float    sample_rate_hz;   // ADC output data rate
    uint16_t fft_size;         // analysis window (in decimated samples), power of 2
    uint16_t hop;              // decimated samples between analyses (window % hop == 0)
    uint8_t  decimation;       // 1, 2 or 4 (half-band FIR cascade before the FFT)
    uint8_t  source;           // AdcSource
    uint8_t  v_pga_gain;       // ADC PGA gain (1..128) — hardware backend only
    uint8_t  i_pga_gain;
    uint8_t  crc_check;        // verify ADC frame CRC
    uint8_t  _pad[3];
};

// Calibration: eng = (counts - offset) * gain, then phase alignment.
struct CalConfig {
    float v_gain;      // volts per ADC count (includes divider ratio)
    float v_offset;    // counts
    float i_gain;      // amps per ADC count (includes CT/shunt ratio)
    float i_offset;    // counts
    float phase_deg;   // current-channel phase error at nominal f (+ = current leads)
};

// Detection thresholds (the PQConfig requested in the specification).
struct PQConfig {
    float nominalVoltage;          // V rms
    float nominalFrequency;        // Hz
    float nominalCurrent;          // A rms (feature scaling only)
    float sagThreshold;            // pu
    float swellThreshold;          // pu
    float interruptionThreshold;   // pu
    float rmsHysteresis;           // pu
    float thdThreshold;            // %
    float thdHysteresis;           // %
    float frequencyTolerance;      // Hz
    float transientDevPu;          // cycle-to-cycle deviation threshold, pu of Vpk
    float impulsePu;               // instantaneous amplitude threshold, pu of Vpk
    float dvdtFactor;              // x nominal max slope
    float waveletK;                // x adaptive wavelet noise floor
    float waveletFloorPu;          // absolute wavelet floor, pu of Vpk
    float loadStepThreshold;       // relative step in cycle Irms
    float loadMinCurrent;          // A
    uint8_t minEventHalfCycles;    // half cycles beyond threshold before an event opens
    uint8_t freqMinFrames;         // frames beyond tolerance before FREQ event opens
    uint8_t _pad[2];
};

struct DspConfig {
    float   dc_block_hz;
    float   freq_lpf_factor;   // LPF corner as multiple of nominal f
    uint8_t freq_avg_cycles;
    uint8_t wavelet_enable;
    uint8_t wavelet_type;      // 0 Haar, 1 db4
    uint8_t wavelet_levels;    // 1..4
    uint8_t transient_enable;
    uint8_t max_harmonic;      // <= PQ_MAX_HARMONIC
    uint8_t _pad[2];
};

struct MlConfig {
    uint8_t ml_enable;
    uint8_t _pad[3];
};

struct FusionConfig {
    float   dsp_weight;          // alpha in fused = a*DSP + (1-a)*ML
    float   ml_only_confidence;  // ML-only advisory events need this confidence...
    uint8_t ml_only_frames;      // ...for this many consecutive frames
    uint8_t _pad[3];
};

struct CommConfig {
    uint32_t metrics_period_ms;  // 0 = no periodic metrics
    uint32_t uart_baud;
    uint8_t  metrics_format;     // 0 JSON, 1 CSV
    uint8_t  uart_enable;        // mirror protocol on UART0
    uint8_t  event_stream;       // push event messages as they happen
    uint8_t  logging_enable;     // RAM event log
};

struct ControlConfig {
    uint8_t armed;               // master enable for trip/reference outputs
    uint8_t trip_on_interruption;
    uint8_t dvr_enable;
    uint8_t apf_enable;
    float   trip_swell_pu;       // SSCB trip if half-cycle Vrms exceeds this
    float   trip_impulse_pu;     // SSCB trip if |v| exceeds this x nominal Vpk
};

struct SystemConfig {
    AcqConfig     acq;
    CalConfig     cal;
    PQConfig      pq;
    DspConfig     dsp;
    MlConfig      ml;
    FusionConfig  fusion;
    CommConfig    comm;
    ControlConfig ctrl;
};

// ------------------------------------------------------------ parameter table
enum class ParamType : uint8_t { F32, U8, U16, U32 };
enum ParamFlags : uint8_t {
    PF_NONE        = 0,
    PF_RESTART_ACQ = 1,   // change requires acquisition/pipeline restart
    PF_BOOL        = 2,
};

struct ParamDesc {
    const char* key;
    ParamType   type;
    uint16_t    offset;
    float       min;
    float       max;
    uint8_t     flags;
    const char* help;
};

const ParamDesc* configParams(size_t& count);
const ParamDesc* configFindParam(const char* key);

SystemConfig configDefaults();

// Range + consistency validation. Returns nullptr when OK, else a message.
const char* configValidate(const SystemConfig& c);

bool  configGet(const SystemConfig& c, const ParamDesc& p, float& out);
// Returns nullptr on success, else a message. Only modifies `c` on success.
const char* configSet(SystemConfig& c, const ParamDesc& p, float value);

// Persistence image.
struct StoredConfig {
    uint32_t     magic;
    uint16_t     version;
    uint16_t     size;
    SystemConfig cfg;
    uint32_t     crc;
};
constexpr uint32_t kConfigMagic   = 0x50514346u;  // "PQCF"
constexpr uint16_t kConfigVersion = 3;

void configPack(const SystemConfig& c, StoredConfig& out);
bool configUnpack(const StoredConfig& in, SystemConfig& out);

}  // namespace pq
