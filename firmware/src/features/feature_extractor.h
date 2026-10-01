// Feature extraction: PqMetrics (+ window dynamics) -> fixed-order float vector.
#pragma once
#include "features/feature_spec.h"
#include "power_quality/pq_events.h"
#include "system/config.h"

namespace pq {

struct FeatureVector {
    float v[kNumFeatures];
};

// y_step: relative admittance step within the window (from the engine).
void extractFeatures(const PqMetrics& m, float y_step, const PQConfig& cfg, FeatureVector& out);

}  // namespace pq
