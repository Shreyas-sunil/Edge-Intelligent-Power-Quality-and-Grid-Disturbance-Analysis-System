// Disturbance detection, features, fusion, ML — through the real pipeline.
#include <cmath>
#include <cstring>

#include "features/feature_extractor.h"
#include "features/feature_spec.h"
#include "fusion/decision_fusion.h"
#include "test_framework.h"
#include "test_helpers.h"
#include "tinyml/classifier.h"
#include "tinyml/model_data.h"

using namespace pq;
using namespace pqtest;

namespace {
Disturbance dist(const char* name, double t0, float mag = 0, float dur = 0) {
    Rng r(11);
    Disturbance d;
    makeScenario(name, t0, mag, dur, r, d);
    return d;
}
SystemConfig noMl() {
    SystemConfig c = configDefaults();
    c.ml.ml_enable = 0;
    return c;
}
}  // namespace

// ------------------------------------------------------------- detector
PQ_TEST(detector, normal_grid_raises_no_events) {
    GeneratorParams g;
    g.noise_v_rms = 0.5f;
    g.base_h[0] = 0.03f;
    g.base_h[1] = 0.02f;
    g.f_offset = 0.2f;
    auto r = runPipeline(noMl(), g, {}, 10.0);
    PQ_CHECK(countEndedAny(r) == 0);
    PQ_CHECK(r.events.empty());
    PQ_CHECK(r.stats.frames > 90);
    PQ_CHECK(r.stats.frames_invalid == 0);
}

PQ_TEST(detector, sag_detected_with_depth_duration_latency) {
    auto r = runPipeline(noMl(), cleanParams(), {dist("sag", 1.0, 0.55f, 0.2f)}, 2.0);
    PQ_CHECK(countEndedAny(r) == 1);
    const EventRecord* e = firstEnded(r, PqClass::Sag);
    PQ_CHECK(e != nullptr);
    if (!e) return;
    PQ_INFO("sag: t=%.4f s  depth %.3f pu  dur %u ms  det latency %.1f ms", e->t_start_us * 1e-6, e->magnitude,
            e->duration_ms, e->det_latency_ms);
    PQ_CHECK_NEAR(e->t_start_us * 1e-6, 1.0, 0.012);
    PQ_CHECK_NEAR(e->magnitude, 0.55, 0.01);
    PQ_CHECK_NEAR(e->duration_ms, 200, 25);
    PQ_CHECK(e->det_latency_ms < 25.0f);   // <= ~1 cycle + block
}

PQ_TEST(detector, swell_and_interruption) {
    auto r = runPipeline(noMl(), cleanParams(), {dist("swell", 1.0, 1.3f, 0.15f), dist("interruption", 2.0, 0.0f, 0.3f)},
                         3.0);
    const EventRecord* s = firstEnded(r, PqClass::Swell);
    const EventRecord* i = firstEnded(r, PqClass::Interruption);
    PQ_CHECK(s && i);
    if (s) PQ_CHECK_NEAR(s->magnitude, 1.3, 0.01);
    if (i) {
        PQ_CHECK(i->magnitude < 0.02f);
        PQ_CHECK_NEAR(i->duration_ms, 300, 30);
    }
    PQ_CHECK(countEndedAny(r) == 2);
}

PQ_TEST(detector, short_sag_below_one_cycle_is_still_caught) {
    auto r = runPipeline(noMl(), cleanParams(), {dist("sag", 1.0, 0.3f, 0.012f)}, 2.0);   // 12 ms
    PQ_CHECK(firstEnded(r, PqClass::Sag) != nullptr);
}

PQ_TEST(detector, hysteresis_prevents_chatter) {
    // voltage hovering around the sag threshold: 0.895..0.905 pu wobble
    GeneratorParams g = cleanParams();
    g.v_scale = 0.899f;
    auto r = runPipeline(noMl(), g, {dist("swell", 1.0, 1.006f, 0.05f), dist("swell", 1.5, 1.006f, 0.05f)}, 3.0);
    int starts = 0;
    for (auto& e : r.events)
        if (e.state == EV_START) ++starts;
    PQ_CHECK(starts == 1);   // one continuous sag, not a burst
}

PQ_TEST(detector, harmonic_distortion_with_thd) {
    auto r = runPipeline(noMl(), cleanParams(), {dist("harmonic", 1.0, 1.0f, 2.0f)}, 4.0);
    const EventRecord* e = firstEnded(r, PqClass::Harmonic);
    PQ_CHECK(e != nullptr);
    if (e) {
        PQ_INFO("harmonic: max THD %.2f %% dur %u ms", e->magnitude, e->duration_ms);
        PQ_CHECK(e->magnitude > 12.0f && e->magnitude < 15.0f);   // 10/7/5/2/1.4 % -> 13.4 %
    }
    PQ_CHECK(countEnded(r, PqClass::Sag) + countEnded(r, PqClass::Swell) == 0);
}

PQ_TEST(detector, frequency_variation) {
    auto r = runPipeline(noMl(), cleanParams(), {dist("frequency", 1.0, -1.2f, 2.0f)}, 5.0);
    const EventRecord* e = firstEnded(r, PqClass::FrequencyVariation);
    PQ_CHECK(e != nullptr);
    if (e) PQ_CHECK_NEAR(e->magnitude, -1.2, 0.02);
    PQ_CHECK(countEndedAny(r) == 1);
}

PQ_TEST(detector, transient_detected_standalone_but_attached_to_sag) {
    auto r = runPipeline(noMl(), cleanParams(), {dist("transient", 1.0, 0.8f)}, 2.0);
    const EventRecord* t = firstEnded(r, PqClass::Transient);
    PQ_CHECK(t != nullptr);
    if (t) {
        PQ_INFO("transient onset %.5f s, peak %.3f pu", t->t_start_us * 1e-6, t->magnitude);
        PQ_CHECK_NEAR(t->t_start_us * 1e-6, 1.0, 0.0005);
    }
    PQ_CHECK(countEndedAny(r) == 1);
    // A sag with a phase jump produces a waveform discontinuity; it must be
    // reported as one sag event (flagged WITH_TRANSIENT), not as two events.
    Disturbance s = dist("sag", 1.0, 0.5f, 0.2f);
    s.p1 = 30.0f;
    auto r2 = runPipeline(noMl(), cleanParams(), {s}, 2.0);
    PQ_CHECK(countEnded(r2, PqClass::Transient) == 0);
    const EventRecord* e = firstEnded(r2, PqClass::Sag);
    PQ_CHECK(e && (e->flags & EVF_WITH_TRANSIENT));
}

PQ_TEST(detector, oscillatory_transient) {
    auto r = runPipeline(noMl(), cleanParams(), {dist("oscillatory", 1.0, 0.4f)}, 2.0);
    PQ_CHECK(firstEnded(r, PqClass::Transient) != nullptr);
}

PQ_TEST(detector, load_switching_vs_sag) {
    auto r = runPipeline(noMl(), cleanParams(), {dist("load", 1.0, 3.0f, 1.0f)}, 3.0);
    PQ_CHECK(countEnded(r, PqClass::LoadSwitching) == 2);   // switch-on and switch-off
    PQ_CHECK(countEnded(r, PqClass::Sag) == 0);
    // A sag makes the current fall too (constant-impedance load), but the
    // admittance does not change -> no load-switching event.
    auto r2 = runPipeline(noMl(), cleanParams(), {dist("sag", 1.0, 0.7f, 0.3f)}, 2.0);
    PQ_CHECK(countEnded(r2, PqClass::LoadSwitching) == 0);
}

PQ_TEST(detector, thresholds_are_configurable) {
    SystemConfig c = noMl();
    c.pq.sagThreshold = 0.5f;   // 0.6 pu sag no longer counts
    auto r = runPipeline(c, cleanParams(), {dist("sag", 1.0, 0.6f, 0.2f)}, 2.0);
    PQ_CHECK(countEndedAny(r) == 0);
    c.pq.sagThreshold = 0.9f;
    c.pq.thdThreshold = 20.0f;   // 13 % THD tolerated
    auto r2 = runPipeline(c, cleanParams(), {dist("harmonic", 1.0, 1.0f, 2.0f)}, 3.5);
    PQ_CHECK(countEnded(r2, PqClass::Harmonic) == 0);
}

PQ_TEST(detector, combined_event_reported) {
    auto r = runPipeline(noMl(), cleanParams(), {dist("harmonic", 0.8, 1.0f, 3.0f), dist("sag", 2.5, 0.6f, 0.5f)}, 4.5);
    bool combined = false;
    for (const auto& f : r.frames)
        if (f.fusion.final_class == PqClass::Combined) combined = true;
    PQ_CHECK(combined);
    const EventRecord* s = firstEnded(r, PqClass::Sag);
    PQ_CHECK(s && (s->components & classBit(PqClass::Harmonic)));
}

// ------------------------------------------------------------- features
PQ_TEST(feature, spec_order_and_hash_are_stable) {
    PQ_CHECK(kNumFeatures == 14);
    PQ_CHECK(std::strcmp(featureName(0), "vrms_pu") == 0);
    PQ_CHECK(std::strcmp(featureName(kNumFeatures - 1), "transient_energy_log") == 0);
    PQ_CHECK(featureSpecHash() == fnv1a32(featureSpecString()));
    PQ_INFO("feature spec: %s (hash 0x%08X)", featureSpecString(), featureSpecHash());
}

PQ_TEST(feature, values_for_clean_sine) {
    auto r = runPipeline(noMl(), cleanParams(), {}, 2.0);
    PQ_CHECK(!r.frames.empty());
    const FeatureVector& f = r.frames.back().features;
    PQ_CHECK_NEAR(f.v[F_vrms_pu], 1.0, 0.002);
    PQ_CHECK_NEAR(f.v[F_vmin_pu], 1.0, 0.002);
    PQ_CHECK_NEAR(f.v[F_freq_dev_hz], 0.0, 0.003);
    PQ_CHECK_NEAR(f.v[F_crest_factor], std::sqrt(2.0), 0.01);
    PQ_CHECK_NEAR(f.v[F_kurtosis], 1.5, 0.02);   // kurtosis of a sine
    PQ_CHECK(f.v[F_thd_v_pct] < 0.1f);
    PQ_CHECK(f.v[F_transient_energy_log] < -5.0f);
}

PQ_TEST(feature, sanitizes_non_finite_inputs) {
    PqMetrics m{};
    m.vrms = NAN;
    m.kurtosis = INFINITY;
    m.freq = 1e9f;
    m.freq_valid = 1;
    m.harm_valid = 1;
    m.v_h[1] = 230;
    m.thd_v = -5;
    FeatureVector f;
    extractFeatures(m, NAN, configDefaults().pq, f);
    for (int k = 0; k < kNumFeatures; ++k) PQ_CHECK(std::isfinite(f.v[k]));
    PQ_CHECK(f.v[F_freq_dev_hz] <= 10.0f);
}

// --------------------------------------------------------------- fusion
PQ_TEST(fusion, dsp_asserted_is_authoritative) {
    FusionConfig fc = configDefaults().fusion;
    DecisionFusion df;
    df.configure(fc);
    DspEvidence ev{};
    ev.e[0] = 0.0f;
    ev.e[static_cast<int>(PqClass::Sag)] = 1.0f;
    ev.asserted = classBit(PqClass::Sag);
    ClassificationResult ml{};
    ml.valid = true;
    ml.class_id = static_cast<uint8_t>(PqClass::Swell);   // ML wrong with high confidence
    ml.confidence = 0.95f;
    ml.probs[static_cast<int>(PqClass::Swell)] = 0.95f;
    ml.probs[static_cast<int>(PqClass::Sag)] = 0.05f;
    FusionResult r = df.fuse(ev, ml);
    PQ_CHECK(r.final_class == PqClass::Sag);
    PQ_CHECK(r.flags & EVF_ML_DISAGREE);
    // agreement -> source dsp+ml
    ml.class_id = static_cast<uint8_t>(PqClass::Sag);
    ml.probs[static_cast<int>(PqClass::Sag)] = 0.94f;
    ml.probs[static_cast<int>(PqClass::Swell)] = 0.01f;
    r = df.fuse(ev, ml);
    PQ_CHECK(r.final_class == PqClass::Sag && r.source == SRC_DSP_ML);
    PQ_CHECK_NEAR(r.confidence, 0.6 * 1.0 + 0.4 * 0.94, 1e-5);
}

PQ_TEST(fusion, ml_only_needs_persistence_and_is_advisory) {
    DecisionFusion df;
    df.configure(configDefaults().fusion);
    DspEvidence ev{};
    ev.e[0] = 1.0f;
    ClassificationResult ml{};
    ml.valid = true;
    ml.class_id = static_cast<uint8_t>(PqClass::Harmonic);
    ml.confidence = 0.97f;
    ml.probs[static_cast<int>(PqClass::Harmonic)] = 0.97f;
    PQ_CHECK(df.fuse(ev, ml).final_class == PqClass::Normal);
    PQ_CHECK(df.fuse(ev, ml).final_class == PqClass::Normal);
    FusionResult r = df.fuse(ev, ml);   // 3rd consecutive frame
    PQ_CHECK(r.final_class == PqClass::Harmonic);
    PQ_CHECK(r.flags & EVF_ML_ONLY);
    ml.confidence = 0.6f;   // low confidence resets the streak
    PQ_CHECK(df.fuse(ev, ml).final_class == PqClass::Normal);
}

PQ_TEST(fusion, multiple_families_give_combined) {
    DecisionFusion df;
    df.configure(configDefaults().fusion);
    DspEvidence ev{};
    ev.e[static_cast<int>(PqClass::Sag)] = 1.0f;
    ev.e[static_cast<int>(PqClass::Harmonic)] = 0.8f;
    ev.asserted = classBit(PqClass::Sag) | classBit(PqClass::Harmonic);
    FusionResult r = df.fuse(ev, ClassificationResult{});
    PQ_CHECK(r.final_class == PqClass::Combined);
    PQ_CHECK(r.primary == PqClass::Sag);
}

// ------------------------------------------------------------------- ML
PQ_TEST(ml, model_loads_with_matching_spec) {
    const ClassifierInfo& ci = Classifier_Info();
    PQ_INFO("model: ready=%d error=%s bytes=%u arena=%u used=%u version=%s acc=%.4f", ci.ready,
            ci.error ? ci.error : "-", ci.model_bytes, ci.arena_bytes, ci.arena_used,
            ci.model_version ? ci.model_version : "-", ci.reported_accuracy);
#if PQ_HAVE_TFLM
    PQ_CHECK(ci.ready);
    PQ_CHECK(ci.spec_hash_model == featureSpecHash());
    PQ_CHECK(ci.arena_used > 0 && ci.arena_used <= ci.arena_bytes);
#endif
}

PQ_TEST(ml, reference_vectors_match_training_output) {
#if PQ_HAVE_TFLM
    if (!Classifier_Info().ready) return;
    // Reference vectors exported by ml/export_model.py: the TFLite interpreter's
    // int8 output in Python must equal the embedded TFLM result exactly.
    for (int k = 0; k < model::kNumRefVectors; ++k) {
        const ClassificationResult r = Classifier_Run(model::kRefInput[k]);
        PQ_CHECK(r.valid);
        PQ_CHECK(r.class_id == model::kRefClass[k]);
        for (int c = 0; c < kNumMlClasses; ++c) PQ_CHECK_NEAR(r.probs[c], model::kRefProbs[k][c], 1e-6);
    }
    PQ_INFO("%d reference vectors bit-exact vs. Python TFLite interpreter", model::kNumRefVectors);
#endif
}

PQ_TEST(ml, classifies_pipeline_features) {
#if PQ_HAVE_TFLM
    if (!Classifier_Info().ready) return;
    SystemConfig c = configDefaults();
    struct Case {
        const char* scen;
        float mag, dur;
        PqClass expect;
    } cases[] = {{"sag", 0.5f, 0.6f, PqClass::Sag},
                 {"swell", 1.3f, 0.6f, PqClass::Swell},
                 {"interruption", 0.0f, 0.6f, PqClass::Interruption},
                 {"harmonic", 1.0f, 2.0f, PqClass::Harmonic},
                 {"frequency", 1.5f, 3.0f, PqClass::FrequencyVariation}};
    for (const Case& cs : cases) {
        auto r = runPipeline(c, cleanParams(), {dist(cs.scen, 1.0, cs.mag, cs.dur)}, 1.0 + cs.dur);
        // the last frame fully inside the disturbance
        const FrameResult* f = nullptr;
        for (const auto& fr : r.frames)
            if (fr.window_start_us > 1.05e6 && fr.t_end_us < (1.0 + cs.dur) * 1e6 && fr.ml.valid) f = &fr;
        PQ_CHECK(f != nullptr);
        if (!f) continue;
        PQ_INFO("%-12s -> ML %s (%.3f), inference %u us", cs.scen,
                pqClassShortName(static_cast<PqClass>(f->ml.class_id)), f->ml.confidence, f->ml.inference_time_us);
        PQ_CHECK(f->ml.class_id == static_cast<uint8_t>(cs.expect));
    }
    auto rn = runPipeline(c, cleanParams(), {}, 2.0);
    PQ_CHECK(rn.frames.back().ml.class_id == 0);
#endif
}
