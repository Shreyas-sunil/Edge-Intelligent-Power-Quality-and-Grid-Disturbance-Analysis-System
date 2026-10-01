#include "fusion/decision_fusion.h"

namespace pq {

int DecisionFusion::family(PqClass c) {
    switch (c) {
        case PqClass::Sag:
        case PqClass::Swell:
        case PqClass::Interruption: return 1;
        case PqClass::Harmonic: return 2;
        case PqClass::FrequencyVariation: return 3;
        case PqClass::Transient: return 4;
        case PqClass::LoadSwitching: return 5;
        default: return 0;
    }
}

FusionResult DecisionFusion::fuse(const DspEvidence& dsp, const ClassificationResult& ml) {
    FusionResult r{};
    r.asserted = dsp.asserted;
    const float a = ml.valid ? cfg_.dsp_weight : 1.0f;
    for (int c = 0; c < kNumMlClasses; ++c)
        r.fused[c] = a * dsp.e[c] + (1.0f - a) * (ml.valid ? ml.probs[c] : 0.0f);

    const uint16_t useful = dsp.asserted & static_cast<uint16_t>(((1u << kNumMlClasses) - 1u) & ~1u);
    if (useful) {
        int best = -1;
        unsigned fam_mask = 0;
        for (int c = 1; c < kNumMlClasses; ++c) {
            if (!(dsp.asserted & classBit(static_cast<PqClass>(c)))) continue;
            fam_mask |= 1u << family(static_cast<PqClass>(c));
            if (best < 0 || r.fused[c] > r.fused[best]) best = c;
        }
        r.primary = static_cast<PqClass>(best);
        r.confidence = r.fused[best];
        int nfam = 0;
        for (unsigned m = fam_mask; m; m &= m - 1) ++nfam;
        r.final_class = nfam >= 2 ? PqClass::Combined : r.primary;
        if (nfam >= 2) r.flags |= EVF_COMBINED;
        r.source = SRC_DSP;
        if (ml.valid) {
            if (ml.class_id == best) r.source = SRC_DSP_ML;
            else if (ml.confidence > 0.8f && !(dsp.asserted & classBit(static_cast<PqClass>(ml.class_id))))
                r.flags |= EVF_ML_DISAGREE;
        }
        ml_only_count_ = 0;
        return r;
    }

    // Nothing asserted by DSP. A class whose FUSED score reaches 0.5 is
    // corroborated: the ML is confident AND the DSP has partial evidence
    // (e.g. a transient still being verified, a value just below threshold).
    {
        int best = 1;
        for (int c = 2; c < kNumMlClasses; ++c)
            if (r.fused[c] > r.fused[best]) best = c;
        if (ml.valid && r.fused[best] >= 0.5f && dsp.e[best] > 0.0f) {
            r.final_class = r.primary = static_cast<PqClass>(best);
            r.confidence = r.fused[best];
            r.source = SRC_DSP_ML;
            ml_only_count_ = 0;
            return r;
        }
    }
    if (ml.valid && ml.class_id != 0 && ml.confidence >= cfg_.ml_only_confidence) {
        ml_only_count_ = (ml.class_id == ml_last_) ? ml_only_count_ + 1 : 1;
        ml_last_ = ml.class_id;
        if (ml_only_count_ >= cfg_.ml_only_frames) {
            r.final_class = r.primary = static_cast<PqClass>(ml.class_id);
            r.confidence = r.fused[ml.class_id];
            r.source = SRC_ML;
            r.flags |= EVF_ML_ONLY;
            return r;
        }
    } else {
        ml_only_count_ = 0;
        ml_last_ = ml.valid ? ml.class_id : 0;
    }
    r.final_class = r.primary = PqClass::Normal;
    r.confidence = r.fused[0];
    r.source = ml.valid ? SRC_DSP_ML : SRC_DSP;
    if (ml.valid && ml.class_id != 0 && ml.confidence > 0.8f) r.flags |= EVF_ML_DISAGREE;
    return r;
}

}  // namespace pq
