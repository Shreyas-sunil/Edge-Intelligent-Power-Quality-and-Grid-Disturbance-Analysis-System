// =============================================================================
//  Physical waveform model used by the simulated ADC backend, the unit tests,
//  and the ML dataset generator. Produces voltage/current in engineering units
//  or directly as ADC counts (inverse calibration + quantisation + clipping),
//  so the simulated path exercises exactly the same code as the hardware path.
// =============================================================================
#pragma once
#include <cstdint>

#include "common/math_util.h"
#include "common/pq_types.h"

namespace pq {

enum class DistKind : uint8_t {
    Sag = 0,               // magnitude = retained voltage [pu], p1 = phase jump [deg]
    Swell,                 // magnitude = voltage [pu]
    Interruption,          // magnitude = residual voltage [pu] (e.g. 0.02)
    Harmonic,              // p1,p2,p3 = H3,H5,H7 fractions of fundamental, magnitude = H11/H13 fraction
    ImpulsiveTransient,    // magnitude = peak [pu of Vpk] (signed), p1 = rise tau [s], p2 = fall tau [s]
    OscillatoryTransient,  // magnitude = peak [pu of Vpk], p1 = ringing freq [Hz], p2 = decay tau [s]
    FrequencyDeviation,    // magnitude = delta f [Hz], p1 = ramp rate [Hz/s]
    LoadStep,              // magnitude = current multiplier, p1 = source impedance drop [pu per pu current]
    Count
};
const char* distKindName(DistKind k);
PqClass     distKindClass(DistKind k);

struct Disturbance {
    DistKind kind = DistKind::Sag;
    double   t_start = 0.0;    // s
    double   duration = 0.0;   // s (<0 -> permanent)
    float    magnitude = 1.0f;
    float    p1 = 0.0f, p2 = 0.0f, p3 = 0.0f;
    float    phase = 0.0f;     // transient phase / sign helper [rad]
    // Effective interval used for ground truth (transients: ~5 decay constants).
    double effectiveEnd() const;
};

struct GeneratorParams {
    float vnom_rms = 230.0f;
    float fnom = 50.0f;
    float f_offset = 0.0f;        // steady deviation from nominal [Hz]
    float v_scale = 1.0f;         // steady voltage level [pu]
    float noise_v_rms = 0.05f;    // V
    float noise_i_rms = 0.002f;   // A
    float base_h[3] = {0.0f, 0.0f, 0.0f};    // background H3,H5,H7 fractions
    float base_h_phase[3] = {0.0f, 0.0f, 0.0f};
    float i_rms = 5.0f;           // load current [A rms]
    float i_nom = 10.0f;          // nominal current [A rms] (for load-step dip)
    float pf_angle_deg = 20.0f;   // current lags voltage by this
    float i_h[3] = {0.05f, 0.03f, 0.02f};    // current H3,H5,H7 fractions (non-linear load)
    float i_phase_err_deg = 0.0f; // simulated CT phase error (current leads by this)
    // Front-end model for count generation (inverse of the calibration)
    float v_gain = 600.0f / 8388608.0f;
    float i_gain = 50.0f / 8388608.0f;
    float v_offset_counts = 0.0f;
    float i_offset_counts = 0.0f;
    int32_t full_scale = 8388607;
    uint32_t seed = 12345;
};

class WaveformGenerator {
public:
    static constexpr int kMaxDist = 8;
    static constexpr int kMaxH = 15;

    void configure(const GeneratorParams& p, float fs);
    void reset(uint64_t start_sample = 0);
    const GeneratorParams& params() const { return p_; }
    GeneratorParams&       mutableParams() { return p_; }
    float  fs() const { return fs_; }
    double time() const { return static_cast<double>(n_) / fs_; }
    uint64_t sampleIndex() const { return n_; }

    bool addDisturbance(const Disturbance& d);   // false if table full
    void clearDisturbances();
    int  activeDisturbances() const { return nd_; }
    const Disturbance& disturbance(int i) const { return dist_[i]; }

    void generate(uint32_t n, float* v, float* i);
    // Advance time by n samples without producing output (used when the
    // simulated DMA ring overruns, exactly like lost hardware samples).
    void skip(uint64_t n);
    // Returns number of saturated samples.
    uint32_t generateCounts(uint32_t n, int32_t* cv, int32_t* ci);

    // Bitmask (classBit) of disturbance classes overlapping [t0, t1] by at
    // least `min_overlap` seconds (transients: any overlap).
    uint16_t truthMask(double t0, double t1, double min_overlap) const;

private:
    void  purgeExpired(double t);
    GeneratorParams p_{};
    float    fs_ = 10000.0f;
    uint64_t n_ = 0;
    float    theta_ = 0.0f;      // fundamental phase [rad]
    float    fcur_ = 50.0f;      // current instantaneous frequency
    float    hcos_[kMaxH + 1]{};
    float    hsin_[kMaxH + 1]{};
    Disturbance dist_[kMaxDist];
    int      nd_ = 0;
    Rng      rng_;
};

// Convenience factory for scenario presets used by `SIM <name>` and tests.
bool makeScenario(const char* name, double t_start, float magnitude, float duration_s,
                  Rng& rng, Disturbance& out);

}  // namespace pq
