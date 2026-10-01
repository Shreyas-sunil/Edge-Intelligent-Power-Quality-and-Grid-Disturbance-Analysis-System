// =============================================================================
//  Decision fusion: deterministic DSP evidence + TinyML probabilities.
//
//  Per class c (0..7):  fused_c = a * e_c(DSP) + (1 - a) * p_c(ML),  a = dsp_weight
//  (a = 1 when ML is disabled/unavailable).
//
//   1. DSP-asserted conditions (thresholds crossed, from the state machines)
//      are authoritative: the final class is chosen ONLY among them, ranked by
//      the fused score. ML can re-rank but never veto a standard-defined event.
//   2. >= 2 asserted conditions from different families (voltage level,
//      waveform/harmonic, frequency, transient, load) -> COMBINED_EVENT, with
//      the best-ranked one reported as primary.
//   3. Nothing asserted, but the best non-normal FUSED score >= 0.5 with some
//      DSP evidence (> 0) -> that class (ML confident + DSP partially agrees).
//   4. Nothing asserted but ML predicts a non-normal class with confidence
//      >= ml_only_confidence for ml_only_frames consecutive frames -> an
//      ADVISORY event (EVF_ML_ONLY): logged/reported, never drives control.
//   5. Otherwise NORMAL.
//  ML/DSP disagreement is flagged (EVF_ML_DISAGREE) for model monitoring.
// =============================================================================
#pragma once
#include "power_quality/disturbance_detector.h"
#include "system/config.h"
#include "tinyml/classifier.h"

namespace pq {

struct FusionResult {
    PqClass  final_class;
    PqClass  primary;          // best single class (== final unless Combined)
    float    confidence;       // fused score of the primary class
    uint8_t  source;           // EventSource
    uint16_t asserted;         // DSP asserted mask
    uint16_t flags;            // EVF_ML_ONLY / EVF_ML_DISAGREE / EVF_COMBINED
    float    fused[kNumMlClasses];
};

class DecisionFusion {
public:
    void configure(const FusionConfig& cfg) { cfg_ = cfg; reset(); }
    void reset() { ml_only_count_ = 0; ml_last_ = 0; }
    FusionResult fuse(const DspEvidence& dsp, const ClassificationResult& ml);

    static int family(PqClass c);

private:
    FusionConfig cfg_{};
    int     ml_only_count_ = 0;
    uint8_t ml_last_ = 0;
};

}  // namespace pq
