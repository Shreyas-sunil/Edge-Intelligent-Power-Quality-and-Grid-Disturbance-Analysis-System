#include "system/system_monitor.h"

namespace pq {

void TimingStats::add(const FrameResult& f) {
    const StageTiming& t = f.timing;
    per_sample.add(t.per_sample_us);
    spectral.add(t.spectral_us);
    window.add(t.window_us);
    detect.add(t.detect_us);
    feature.add(t.feature_us);
    ml_total.add(t.ml_us);
    if (f.ml.valid) ml_invoke.add(f.ml.inference_time_us);
    fusion.add(t.fusion_us);
    frame.add(t.frame_us);
    e2e.add(t.per_sample_us + t.frame_us);
}

}  // namespace pq
