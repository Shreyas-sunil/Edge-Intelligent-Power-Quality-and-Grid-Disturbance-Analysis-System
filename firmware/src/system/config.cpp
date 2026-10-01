#include "system/config.h"

#include <cmath>
#include <cstring>

#include "common/math_util.h"

namespace pq {

#define PQ_F(field, mn, mx, fl, h) {#field, ParamType::F32, (uint16_t)offsetof(SystemConfig, field), mn, mx, fl, h}
#define PQ_U8(field, mn, mx, fl, h) {#field, ParamType::U8, (uint16_t)offsetof(SystemConfig, field), mn, mx, fl, h}
#define PQ_U16(field, mn, mx, fl, h) {#field, ParamType::U16, (uint16_t)offsetof(SystemConfig, field), mn, mx, fl, h}
#define PQ_U32(field, mn, mx, fl, h) {#field, ParamType::U32, (uint16_t)offsetof(SystemConfig, field), mn, mx, fl, h}

// NOTE: keys are generated from the member path, e.g. "acq.sample_rate_hz".
static const ParamDesc kParams[] = {
    PQ_F(acq.sample_rate_hz, 1000.0f, PQ_MAX_SAMPLE_RATE, PF_RESTART_ACQ, "ADC sample rate [Hz]"),
    PQ_U16(acq.fft_size, 256, PQ_FFT_MAX, PF_RESTART_ACQ, "FFT/window size (decimated samples, pow2)"),
    PQ_U16(acq.hop, 64, PQ_FFT_MAX, PF_RESTART_ACQ, "analysis hop (decimated samples)"),
    PQ_U8(acq.decimation, 1, 4, PF_RESTART_ACQ, "pre-FFT decimation 1/2/4"),
    PQ_U8(acq.source, 0, 1, PF_RESTART_ACQ, "0 = hardware ADC, 1 = simulator"),
    PQ_U8(acq.v_pga_gain, 1, 128, PF_RESTART_ACQ, "voltage channel PGA gain"),
    PQ_U8(acq.i_pga_gain, 1, 128, PF_RESTART_ACQ, "current channel PGA gain"),
    PQ_U8(acq.crc_check, 0, 1, PF_BOOL, "verify ADC frame CRC"),

    PQ_F(cal.v_gain, 1e-9f, 1.0f, PF_NONE, "V per count"),
    PQ_F(cal.v_offset, -1e6f, 1e6f, PF_NONE, "voltage offset [counts]"),
    PQ_F(cal.i_gain, 1e-10f, 1.0f, PF_NONE, "A per count"),
    PQ_F(cal.i_offset, -1e6f, 1e6f, PF_NONE, "current offset [counts]"),
    PQ_F(cal.phase_deg, -20.0f, 20.0f, PF_NONE, "current phase correction [deg]"),

    PQ_F(pq.nominalVoltage, 50.0f, 1000.0f, PF_NONE, "nominal voltage [V rms]"),
    PQ_F(pq.nominalFrequency, 45.0f, 65.0f, PF_RESTART_ACQ, "nominal frequency [Hz]"),
    PQ_F(pq.nominalCurrent, 0.01f, 10000.0f, PF_NONE, "nominal current [A rms]"),
    PQ_F(pq.sagThreshold, 0.5f, 0.99f, PF_NONE, "sag threshold [pu]"),
    PQ_F(pq.swellThreshold, 1.01f, 2.0f, PF_NONE, "swell threshold [pu]"),
    PQ_F(pq.interruptionThreshold, 0.01f, 0.5f, PF_NONE, "interruption threshold [pu]"),
    PQ_F(pq.rmsHysteresis, 0.0f, 0.1f, PF_NONE, "RMS hysteresis [pu]"),
    PQ_F(pq.thdThreshold, 0.5f, 50.0f, PF_NONE, "THD threshold [%]"),
    PQ_F(pq.thdHysteresis, 0.0f, 5.0f, PF_NONE, "THD hysteresis [%]"),
    PQ_F(pq.frequencyTolerance, 0.01f, 5.0f, PF_NONE, "frequency tolerance [Hz]"),
    PQ_F(pq.transientDevPu, 0.02f, 2.0f, PF_NONE, "transient cycle deviation [pu Vpk]"),
    PQ_F(pq.impulsePu, 1.05f, 5.0f, PF_NONE, "impulse amplitude [pu Vpk]"),
    PQ_F(pq.dvdtFactor, 1.5f, 50.0f, PF_NONE, "dv/dt threshold [x max 50Hz slope]"),
    PQ_F(pq.waveletK, 2.0f, 100.0f, PF_NONE, "wavelet threshold [x noise floor]"),
    PQ_F(pq.waveletFloorPu, 0.001f, 1.0f, PF_NONE, "wavelet absolute floor [pu Vpk]"),
    PQ_F(pq.loadStepThreshold, 0.05f, 5.0f, PF_NONE, "load switching step [relative]"),
    PQ_F(pq.loadMinCurrent, 0.0f, 1000.0f, PF_NONE, "load switching min current [A]"),
    PQ_U8(pq.minEventHalfCycles, 1, 10, PF_NONE, "half-cycles beyond threshold to open event"),
    PQ_U8(pq.freqMinFrames, 1, 50, PF_NONE, "frames beyond tolerance to open FREQ event"),

    PQ_F(dsp.dc_block_hz, 0.1f, 10.0f, PF_RESTART_ACQ, "DC blocker corner [Hz]"),
    PQ_F(dsp.freq_lpf_factor, 1.1f, 4.0f, PF_RESTART_ACQ, "frequency LPF corner [x f_nom]"),
    PQ_U8(dsp.freq_avg_cycles, 1, 50, PF_RESTART_ACQ, "cycles averaged for frequency"),
    PQ_U8(dsp.wavelet_enable, 0, 1, PF_BOOL | PF_RESTART_ACQ, "wavelet transient detector"),
    PQ_U8(dsp.wavelet_type, 0, 1, PF_RESTART_ACQ, "0 Haar, 1 db4"),
    PQ_U8(dsp.wavelet_levels, 1, 4, PF_RESTART_ACQ, "wavelet decomposition levels"),
    PQ_U8(dsp.transient_enable, 0, 1, PF_BOOL, "transient detection"),
    PQ_U8(dsp.max_harmonic, 2, PQ_MAX_HARMONIC, PF_NONE, "highest harmonic in THD"),

    PQ_U8(ml.ml_enable, 0, 1, PF_BOOL, "TinyML inference"),

    PQ_F(fusion.dsp_weight, 0.0f, 1.0f, PF_NONE, "fusion weight of DSP evidence"),
    PQ_F(fusion.ml_only_confidence, 0.5f, 1.0f, PF_NONE, "confidence for ML-only advisories"),
    PQ_U8(fusion.ml_only_frames, 1, 50, PF_NONE, "consecutive frames for ML-only advisory"),

    PQ_U32(comm.metrics_period_ms, 0, 60000, PF_NONE, "metrics stream period [ms], 0=off"),
    PQ_U32(comm.uart_baud, 9600, 3000000, PF_NONE, "UART baud"),
    PQ_U8(comm.metrics_format, 0, 1, PF_NONE, "0 JSON, 1 CSV"),
    PQ_U8(comm.uart_enable, 0, 1, PF_BOOL, "mirror protocol on UART0 (CONFIG SAVE + REBOOT)"),
    PQ_U8(comm.event_stream, 0, 1, PF_BOOL, "stream events as they occur"),
    PQ_U8(comm.logging_enable, 0, 1, PF_BOOL, "RAM event log"),

    PQ_U8(ctrl.armed, 0, 1, PF_BOOL, "enable trip/reference outputs"),
    PQ_U8(ctrl.trip_on_interruption, 0, 1, PF_BOOL, "SSCB trip on interruption"),
    PQ_U8(ctrl.dvr_enable, 0, 1, PF_BOOL, "DVR reference output"),
    PQ_U8(ctrl.apf_enable, 0, 1, PF_BOOL, "APF reference output"),
    PQ_F(ctrl.trip_swell_pu, 1.05f, 3.0f, PF_NONE, "SSCB trip swell level [pu]"),
    PQ_F(ctrl.trip_impulse_pu, 1.2f, 10.0f, PF_NONE, "SSCB trip impulse level [pu Vpk]"),
};

#undef PQ_F
#undef PQ_U8
#undef PQ_U16
#undef PQ_U32

const ParamDesc* configParams(size_t& count) {
    count = sizeof(kParams) / sizeof(kParams[0]);
    return kParams;
}

const ParamDesc* configFindParam(const char* key) {
    for (const auto& p : kParams)
        if (std::strcmp(p.key, key) == 0) return &p;
    return nullptr;
}

SystemConfig configDefaults() {
    SystemConfig c;
    std::memset(&c, 0, sizeof(c));

    c.acq.sample_rate_hz = PQ_DEFAULT_SAMPLE_RATE;
    c.acq.fft_size = PQ_DEFAULT_FFT_SIZE;
    c.acq.hop = PQ_DEFAULT_HOP;
    c.acq.decimation = PQ_DEFAULT_DECIMATION;
    c.acq.source = PQ_DEFAULT_SIMULATION ? 1 : 0;
    c.acq.v_pga_gain = 1;
    c.acq.i_pga_gain = 1;
    c.acq.crc_check = 1;

    // Defaults for an ADS131M02 (+/-1.2 V FS, 24 bit) behind a front end that
    // maps +/-600 V peak and +/-50 A peak onto full scale. CALIBRATE on hardware.
    c.cal.v_gain = 600.0f / 8388608.0f;
    c.cal.v_offset = 0.0f;
    c.cal.i_gain = 50.0f / 8388608.0f;
    c.cal.i_offset = 0.0f;
    c.cal.phase_deg = 0.0f;

    c.pq.nominalVoltage = PQ_DEFAULT_NOMINAL_VOLTAGE;
    c.pq.nominalFrequency = PQ_DEFAULT_NOMINAL_FREQUENCY;
    c.pq.nominalCurrent = PQ_DEFAULT_NOMINAL_CURRENT;
    c.pq.sagThreshold = PQ_DEFAULT_SAG_THRESHOLD;
    c.pq.swellThreshold = PQ_DEFAULT_SWELL_THRESHOLD;
    c.pq.interruptionThreshold = PQ_DEFAULT_INTERRUPTION_THRESHOLD;
    c.pq.rmsHysteresis = PQ_DEFAULT_RMS_HYSTERESIS;
    c.pq.thdThreshold = PQ_DEFAULT_THD_THRESHOLD;
    c.pq.thdHysteresis = PQ_DEFAULT_THD_HYSTERESIS;
    c.pq.frequencyTolerance = PQ_DEFAULT_FREQ_TOLERANCE;
    c.pq.transientDevPu = PQ_DEFAULT_TRANSIENT_DEV_PU;
    c.pq.impulsePu = PQ_DEFAULT_IMPULSE_PU;
    c.pq.dvdtFactor = PQ_DEFAULT_DVDT_FACTOR;
    c.pq.waveletK = PQ_DEFAULT_WAVELET_K;
    c.pq.waveletFloorPu = PQ_DEFAULT_WAVELET_FLOOR_PU;
    c.pq.loadStepThreshold = PQ_DEFAULT_LOAD_STEP;
    c.pq.loadMinCurrent = PQ_DEFAULT_LOAD_MIN_CURRENT;
    c.pq.minEventHalfCycles = PQ_DEFAULT_MIN_EVENT_HALFCYCLES;
    c.pq.freqMinFrames = PQ_DEFAULT_FREQ_MIN_FRAMES;

    c.dsp.dc_block_hz = PQ_DEFAULT_DC_BLOCK_HZ;
    c.dsp.freq_lpf_factor = PQ_DEFAULT_FREQ_LPF_FACTOR;
    c.dsp.freq_avg_cycles = PQ_DEFAULT_FREQ_AVG_CYCLES;
    c.dsp.wavelet_enable = PQ_DEFAULT_WAVELET_ENABLE;
    c.dsp.wavelet_type = PQ_DEFAULT_WAVELET_TYPE;
    c.dsp.wavelet_levels = PQ_DEFAULT_WAVELET_LEVELS;
    c.dsp.transient_enable = 1;
    c.dsp.max_harmonic = PQ_MAX_HARMONIC;

    c.ml.ml_enable = PQ_DEFAULT_ML_ENABLE;

    c.fusion.dsp_weight = PQ_DEFAULT_FUSION_DSP_WEIGHT;
    c.fusion.ml_only_confidence = PQ_DEFAULT_ML_ONLY_CONFIDENCE;
    c.fusion.ml_only_frames = PQ_DEFAULT_ML_ONLY_FRAMES;

    c.comm.metrics_period_ms = PQ_DEFAULT_METRICS_PERIOD_MS;
    c.comm.uart_baud = PQ_DEFAULT_UART_BAUD;
    c.comm.metrics_format = 0;
    c.comm.uart_enable = 0;
    c.comm.event_stream = 1;
    c.comm.logging_enable = PQ_DEFAULT_LOGGING_ENABLE;

    c.ctrl.armed = PQ_DEFAULT_CONTROL_ARMED;
    c.ctrl.trip_on_interruption = 0;
    c.ctrl.dvr_enable = 1;
    c.ctrl.apf_enable = 1;
    c.ctrl.trip_swell_pu = PQ_DEFAULT_TRIP_SWELL_PU;
    c.ctrl.trip_impulse_pu = PQ_DEFAULT_TRIP_IMPULSE_PU;
    return c;
}

static bool isPow2(uint32_t v) { return v && !(v & (v - 1)); }

const char* configValidate(const SystemConfig& c) {
    size_t n;
    const ParamDesc* ps = configParams(n);
    for (size_t i = 0; i < n; ++i) {
        float v = 0.0f;
        configGet(c, ps[i], v);
        if (!(v >= ps[i].min && v <= ps[i].max)) return ps[i].key;  // also rejects NaN
    }
    if (!isPow2(c.acq.fft_size)) return "acq.fft_size must be a power of two";
    if (c.acq.decimation != 1 && c.acq.decimation != 2 && c.acq.decimation != 4)
        return "acq.decimation must be 1, 2 or 4";
    if (c.acq.hop > c.acq.fft_size || (c.acq.fft_size % c.acq.hop) != 0)
        return "acq.hop must divide acq.fft_size";
    if (c.acq.fft_size / c.acq.hop > 8) return "acq.fft_size/acq.hop must be <= 8";
    const float fsd = c.acq.sample_rate_hz / c.acq.decimation;
    if (fsd < 8.0f * c.pq.nominalFrequency) return "decimated rate too low for nominal frequency";
    const float window_s = c.acq.fft_size / fsd;
    if (window_s * c.pq.nominalFrequency < 2.0f) return "analysis window shorter than 2 cycles";
    if (c.pq.interruptionThreshold >= c.pq.sagThreshold) return "interruption >= sag threshold";
    const float max_period = c.acq.sample_rate_hz / (0.8f * c.pq.nominalFrequency);
    if (max_period > 1000.0f) return "sample rate too high for cycle buffer";
    return nullptr;
}

bool configGet(const SystemConfig& c, const ParamDesc& p, float& out) {
    const auto* base = reinterpret_cast<const uint8_t*>(&c) + p.offset;
    switch (p.type) {
        case ParamType::F32: { float v; std::memcpy(&v, base, 4); out = v; return true; }
        case ParamType::U8: out = *base; return true;
        case ParamType::U16: { uint16_t v; std::memcpy(&v, base, 2); out = v; return true; }
        case ParamType::U32: { uint32_t v; std::memcpy(&v, base, 4); out = static_cast<float>(v); return true; }
    }
    return false;
}

const char* configSet(SystemConfig& c, const ParamDesc& p, float value) {
    if (!(value >= p.min && value <= p.max)) return "value out of range";
    SystemConfig tmp = c;
    auto* base = reinterpret_cast<uint8_t*>(&tmp) + p.offset;
    switch (p.type) {
        case ParamType::F32: std::memcpy(base, &value, 4); break;
        case ParamType::U8: *base = static_cast<uint8_t>(std::lround(value)); break;
        case ParamType::U16: { uint16_t v = static_cast<uint16_t>(std::lround(value)); std::memcpy(base, &v, 2); break; }
        case ParamType::U32: { uint32_t v = static_cast<uint32_t>(std::llround(value)); std::memcpy(base, &v, 4); break; }
    }
    if (const char* err = configValidate(tmp)) return err;
    c = tmp;
    return nullptr;
}

void configPack(const SystemConfig& c, StoredConfig& out) {
    std::memset(&out, 0, sizeof(out));
    out.magic = kConfigMagic;
    out.version = kConfigVersion;
    out.size = sizeof(SystemConfig);
    out.cfg = c;
    out.crc = crc32(&out, offsetof(StoredConfig, crc));
}

bool configUnpack(const StoredConfig& in, SystemConfig& out) {
    if (in.magic != kConfigMagic || in.version != kConfigVersion || in.size != sizeof(SystemConfig))
        return false;
    if (crc32(&in, offsetof(StoredConfig, crc)) != in.crc) return false;
    if (configValidate(in.cfg) != nullptr) return false;
    out = in.cfg;
    return true;
}

}  // namespace pq
