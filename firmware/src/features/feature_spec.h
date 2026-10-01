// =============================================================================
//  THE feature vector definition — single source of truth.
//
//  The same X-macro list drives:
//    * the embedded feature extractor (feature_extractor.cpp),
//    * the CSV header written by the host dataset generator (pq_datagen),
//    * the Python trainer (reads the CSV header and records the names in the
//      generated model header), and
//    * a run-time check: the FNV-1a hash of the joined names is stored in the
//      model; Classifier_Init() refuses a model whose hash differs.
//  => ordering is identical in training, conversion and embedded inference.
//
//  Add/remove/reorder a feature ONLY here, then regenerate the dataset and
//  retrain (ml/README.md).
// =============================================================================
#pragma once

//        name                 description
#define PQ_FEATURE_LIST(X)                                                              \
    X(vrms_pu,              "window true-RMS voltage / V_nom")                          \
    X(vmin_pu,              "minimum Urms(1/2) in window / V_nom")                      \
    X(vmax_pu,              "maximum Urms(1/2) in window / V_nom")                      \
    X(y_step,               "relative step of load admittance Irms/Vpu in window")     \
    X(freq_dev_hz,          "frequency - f_nom [Hz]")                                   \
    X(thd_v_pct,            "voltage THD [%]")                                          \
    X(h3_pct,               "H3 / H1 [%]")                                              \
    X(h5_pct,               "H5 / H1 [%]")                                              \
    X(h7_pct,               "H7 / H1 [%]")                                              \
    X(crest_factor,         "peak |v| / Vrms")                                          \
    X(kurtosis,             "kurtosis of v samples (sine = 1.5)")                       \
    X(peak_pu,              "peak |v| / (sqrt2 * V_nom)")                               \
    X(wavelet_energy_log,   "log10(sum wavelet detail^2 / sum v^2)")                    \
    X(transient_energy_log, "log10(sum cycle-deviation^2 / sum v^2)")

namespace pq {

enum FeatureIndex : int {
#define PQ_FEATURE_ENUM(name, desc) F_##name,
    PQ_FEATURE_LIST(PQ_FEATURE_ENUM)
#undef PQ_FEATURE_ENUM
    kNumFeatures
};

const char* featureName(int i);
const char* featureDescription(int i);
// Comma-joined names ("vrms_pu,vmin_pu,...") and its FNV-1a hash.
const char* featureSpecString();
unsigned    featureSpecHash();

}  // namespace pq
