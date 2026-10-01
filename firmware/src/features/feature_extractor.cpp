#include "features/feature_extractor.h"

#include <cmath>
#include <initializer_list>

#include "common/math_util.h"

namespace pq {

namespace {
const char* const kNames[kNumFeatures] = {
#define PQ_FEATURE_NAME(name, desc) #name,
    PQ_FEATURE_LIST(PQ_FEATURE_NAME)
#undef PQ_FEATURE_NAME
};
const char* const kDesc[kNumFeatures] = {
#define PQ_FEATURE_DESC(name, desc) desc,
    PQ_FEATURE_LIST(PQ_FEATURE_DESC)
#undef PQ_FEATURE_DESC
};

struct SpecString {
    char s[512];
    unsigned hash;
    SpecString() {
        int p = 0;
        for (int i = 0; i < kNumFeatures; ++i) {
            for (const char* c = kNames[i]; *c && p < 510; ++c) s[p++] = *c;
            if (i + 1 < kNumFeatures) s[p++] = ',';
        }
        s[p] = 0;
        hash = fnv1a32(s);
    }
};
const SpecString& spec() {
    static const SpecString sp;
    return sp;
}
}  // namespace

const char* featureName(int i) { return (i >= 0 && i < kNumFeatures) ? kNames[i] : "?"; }
const char* featureDescription(int i) { return (i >= 0 && i < kNumFeatures) ? kDesc[i] : "?"; }
const char* featureSpecString() { return spec().s; }
unsigned featureSpecHash() { return spec().hash; }

void extractFeatures(const PqMetrics& m, float y_step, const PQConfig& cfg, FeatureVector& out) {
    const float vn = cfg.nominalVoltage;
    const float h1 = m.v_h[1];
    const bool hv = m.harm_valid && h1 > 1e-3f * vn;
    float* f = out.v;
    f[F_vrms_pu] = m.vrms / vn;
    f[F_vmin_pu] = m.vmin_half / vn;
    f[F_vmax_pu] = m.vmax_half / vn;
    f[F_y_step] = y_step;
    f[F_freq_dev_hz] = m.freq_valid ? (m.freq - cfg.nominalFrequency) : 0.0f;
    f[F_thd_v_pct] = hv ? m.thd_v : 0.0f;
    f[F_h3_pct] = hv ? 100.0f * m.v_h[3] / h1 : 0.0f;
    f[F_h5_pct] = hv ? 100.0f * m.v_h[5] / h1 : 0.0f;
    f[F_h7_pct] = hv ? 100.0f * m.v_h[7] / h1 : 0.0f;
    f[F_crest_factor] = m.crest;
    f[F_kurtosis] = m.kurtosis;
    f[F_peak_pu] = m.peak_v / (kSqrt2 * vn);
    f[F_wavelet_energy_log] = m.wavelet_energy;
    f[F_transient_energy_log] = m.transient_energy;

    // Guard rails: the model must never see NaN/Inf or absurd magnitudes.
    f[F_freq_dev_hz] = clampv(sanitize(f[F_freq_dev_hz]), -10.0f, 10.0f);
    f[F_thd_v_pct] = clampv(sanitize(f[F_thd_v_pct]), 0.0f, 200.0f);
    for (int i : {F_h3_pct, F_h5_pct, F_h7_pct}) f[i] = clampv(sanitize(f[i]), 0.0f, 200.0f);
    f[F_crest_factor] = clampv(sanitize(f[F_crest_factor]), 0.0f, 50.0f);
    f[F_kurtosis] = clampv(sanitize(f[F_kurtosis]), 0.0f, 100.0f);
    for (int i : {F_vrms_pu, F_vmin_pu, F_vmax_pu, F_peak_pu}) f[i] = clampv(sanitize(f[i]), 0.0f, 10.0f);
    f[F_y_step] = clampv(sanitize(f[F_y_step]), 0.0f, 20.0f);
    for (int i : {F_wavelet_energy_log, F_transient_energy_log}) f[i] = clampv(sanitize(f[i]), -12.0f, 3.0f);
}

}  // namespace pq
