// =============================================================================
//  pq_datagen — labelled training data from the REAL firmware pipeline.
//
//  Each clip: random but physically plausible grid conditions + one
//  disturbance, rendered by the waveform model into ADC counts, then pushed
//  through SimAdcBackend -> Calibrator -> PqEngine exactly as on the MCU.
//  One CSV row per analysis frame: features (in firmware order) + label.
//  Because the features come from the embedded code itself, there is no
//  training/inference skew.
//
//    pq_datagen --clips 1500 --seed 1 --out train.csv
//    pq_datagen --clips 300  --seed 99 --out test.csv
//    pq_datagen --eval --clips 200 --seed 7     (uses the compiled-in model)
// =============================================================================
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

#include "adc/adc_calibration.h"
#include "adc/sim_adc.h"
#include "app/pq_engine.h"
#include "features/feature_spec.h"
#include "tinyml/classifier.h"

using namespace pq;

namespace {

struct Clip {
    GeneratorParams gp;
    Disturbance     d;
    bool            has_d;
    double          label_from, label_to;   // for FREQUENCY: settled interval
    double          duration;
    PqClass         cls;
};

float logUniform(Rng& r, float a, float b) { return std::exp(r.uniform(std::log(a), std::log(b))); }

Clip makeClip(PqClass cls, Rng& r, const SystemConfig& cfg) {
    Clip c{};
    c.cls = cls;
    GeneratorParams& g = c.gp;
    g.vnom_rms = cfg.pq.nominalVoltage;
    g.fnom = cfg.pq.nominalFrequency;
    g.v_scale = r.uniform(0.95f, 1.05f);
    g.f_offset = r.uniform(-0.3f, 0.3f);
    g.noise_v_rms = logUniform(r, 0.02f, 0.8f);
    g.noise_i_rms = logUniform(r, 0.0005f, 0.02f);
    const float thd_bg = r.uniform(0.0f, 0.05f);          // background THD <= 5 %
    g.base_h[0] = thd_bg * r.uniform(0.5f, 0.9f);
    g.base_h[1] = thd_bg * r.uniform(0.2f, 0.6f);
    g.base_h[2] = thd_bg * r.uniform(0.1f, 0.4f);
    for (float& p : g.base_h_phase) p = r.uniform(0.0f, kTwoPi);
    g.i_rms = logUniform(r, 0.5f, 15.0f);
    g.i_nom = cfg.pq.nominalCurrent;
    g.pf_angle_deg = r.uniform(-10.0f, 45.0f);
    g.i_h[0] = r.uniform(0.0f, 0.3f);
    g.i_h[1] = r.uniform(0.0f, 0.2f);
    g.i_h[2] = r.uniform(0.0f, 0.1f);
    g.v_gain = cfg.cal.v_gain;
    g.i_gain = cfg.cal.i_gain;
    g.v_offset_counts = r.uniform(-3000.0f, 3000.0f);   // uncalibrated offsets: DC blocker must cope
    g.i_offset_counts = r.uniform(-3000.0f, 3000.0f);
    g.seed = r.next();

    c.duration = 1.4;
    const double t0 = r.uniform(0.45f, 0.75f);
    Disturbance& d = c.d;
    d.t_start = t0;
    c.has_d = cls != PqClass::Normal;
    c.label_from = t0;
    switch (cls) {
        case PqClass::Normal: break;
        case PqClass::Sag:
            d.kind = DistKind::Sag;
            d.magnitude = r.uniform(0.12f, 0.86f) / g.v_scale;
            d.duration = logUniform(r, 0.03f, 0.5f);
            d.p1 = r.uniform() < 0.3f ? r.uniform(-30.0f, 30.0f) : 0.0f;   // phase jump
            break;
        case PqClass::Swell:
            d.kind = DistKind::Swell;
            d.magnitude = r.uniform(1.14f, 1.6f) / g.v_scale;
            d.duration = logUniform(r, 0.03f, 0.5f);
            break;
        case PqClass::Interruption:
            d.kind = DistKind::Interruption;
            d.magnitude = r.uniform(0.0f, 0.07f);
            d.duration = logUniform(r, 0.04f, 0.6f);
            break;
        case PqClass::Harmonic: {
            d.kind = DistKind::Harmonic;
            const float thd = r.uniform(0.10f, 0.32f);
            float w3 = r.uniform(0.2f, 1.0f), w5 = r.uniform(0.2f, 1.0f), w7 = r.uniform(0.0f, 0.7f),
                  w11 = r.uniform(0.0f, 0.3f);
            const float norm = std::sqrt(w3 * w3 + w5 * w5 + w7 * w7 + w11 * w11 * 1.49f);
            d.p1 = thd * w3 / norm;
            d.p2 = thd * w5 / norm;
            d.p3 = thd * w7 / norm;
            d.magnitude = thd * w11 / norm;
            d.duration = -1.0;                     // stays for the rest of the clip
            c.label_from = t0 + 0.02;
            break;
        }
        case PqClass::Transient:
            if (r.uniform() < 0.5f) {
                d.kind = DistKind::ImpulsiveTransient;
                d.magnitude = (r.uniform() < 0.5f ? -1.0f : 1.0f) * r.uniform(0.25f, 0.8f);
                d.p1 = logUniform(r, 5e-6f, 60e-6f);
                d.p2 = logUniform(r, 80e-6f, 1.5e-3f);
                d.duration = 0.01;
            } else {
                d.kind = DistKind::OscillatoryTransient;
                d.magnitude = r.uniform(0.2f, 0.8f);
                d.p1 = logUniform(r, 300.0f, 4000.0f);
                d.p2 = logUniform(r, 0.4e-3f, 4e-3f);
                d.phase = r.uniform(0.0f, kTwoPi);
                d.duration = 0.03;
            }
            break;
        case PqClass::FrequencyVariation: {
            d.kind = DistKind::FrequencyDeviation;
            const float df = r.uniform(0.75f, 3.0f);
            d.magnitude = r.uniform() < 0.5f ? -df : df;
            d.p1 = r.uniform(4.0f, 20.0f);         // Hz/s
            d.duration = -1.0;
            d.t_start = 0.35;
            // label once settled + one analysis window has passed
            c.label_from = d.t_start + df / d.p1 + 0.22;
            c.duration = c.label_from + 0.8;
            break;
        }
        case PqClass::LoadSwitching:
            d.kind = DistKind::LoadStep;
            d.magnitude = r.uniform() < 0.6f ? r.uniform(1.6f, 5.0f) : r.uniform(0.15f, 0.55f);
            d.p1 = r.uniform(0.0f, 0.03f);         // source impedance -> small voltage dip
            d.duration = r.uniform(0.25f, 0.6f);
            g.i_rms = logUniform(r, 1.0f, 12.0f);
            break;
        default: break;
    }
    c.label_to = c.duration;
    return c;
}

// Ground-truth label of an analysis window, or -1 if ambiguous (skipped).
int labelFor(const Clip& c, double w0, double w1) {
    if (!c.has_d) return 0;
    const Disturbance& d = c.d;
    const double cyc = 0.02;
    switch (c.cls) {
        case PqClass::Sag:
        case PqClass::Swell:
        case PqClass::Interruption: {
            const double a = d.t_start, b = d.t_start + d.duration;
            const double ov = std::fmin(b, w1) - std::fmax(a, w0);
            if (ov >= cyc) return static_cast<int>(c.cls);
            if (ov > -2 * cyc) return -1;          // edge within/near window: ambiguous
            return 0;
        }
        case PqClass::Harmonic:
            if (w0 >= c.label_from) return static_cast<int>(PqClass::Harmonic);
            if (w1 <= d.t_start) return 0;
            return -1;
        case PqClass::Transient: {
            const double a = d.t_start, b = d.effectiveEnd();
            if (a >= w0 + 0.002 && b <= w1 - 0.001) return static_cast<int>(PqClass::Transient);
            if (b < w0 - 0.025 || a > w1) return 0;   // echo region excluded
            return -1;
        }
        case PqClass::FrequencyVariation:
            if (w0 >= c.label_from) return static_cast<int>(PqClass::FrequencyVariation);
            if (w1 <= d.t_start) return 0;
            return -1;
        case PqClass::LoadSwitching: {
            const double on = d.t_start, off = d.t_start + d.duration;
            auto inside = [&](double t) { return t >= w0 + cyc && t <= w1 - cyc; };
            auto near = [&](double t) { return t >= w0 - 2 * cyc && t <= w1 + cyc; };
            if (inside(on) || inside(off)) return static_cast<int>(PqClass::LoadSwitching);
            if (near(on) || near(off)) return -1;
            return 0;
        }
        default: return 0;
    }
}

struct Confusion {
    uint32_t m[kNumPqClasses][kNumPqClasses]{};
    void add(int t, int p) { ++m[t][p]; }
    void print(const char* title) const {
        std::printf("\n%s (rows = truth, cols = predicted)\n%-14s", title, "");
        for (int p = 0; p < kNumPqClasses; ++p) std::printf("%6.6s ", pqClassShortName(static_cast<PqClass>(p)));
        std::printf("  recall\n");
        uint32_t ok = 0, all = 0;
        for (int t = 0; t < kNumMlClasses; ++t) {
            uint32_t row = 0;
            for (int p = 0; p < kNumPqClasses; ++p) row += m[t][p];
            if (!row) continue;
            std::printf("%-14.14s", pqClassShortName(static_cast<PqClass>(t)));
            for (int p = 0; p < kNumPqClasses; ++p) std::printf("%6u ", m[t][p]);
            std::printf("  %5.1f%%\n", 100.0 * m[t][t] / row);
            ok += m[t][t];
            all += row;
        }
        std::printf("accuracy: %.2f%% (%u frames)\n", all ? 100.0 * ok / all : 0.0, all);
    }
};

}  // namespace

int main(int argc, char** argv) {
    int clips = 500;
    uint32_t seed = 1;
    const char* out_path = "dataset.csv";
    bool eval = false, verbose = false;
    for (int k = 1; k < argc; ++k) {
        if (!std::strcmp(argv[k], "--clips") && k + 1 < argc) clips = std::atoi(argv[++k]);
        else if (!std::strcmp(argv[k], "--seed") && k + 1 < argc) seed = static_cast<uint32_t>(std::atoi(argv[++k]));
        else if (!std::strcmp(argv[k], "--out") && k + 1 < argc) out_path = argv[++k];
        else if (!std::strcmp(argv[k], "--eval")) eval = true;
        else if (!std::strcmp(argv[k], "--verbose")) verbose = true;
        else {
            std::fprintf(stderr, "usage: pq_datagen [--clips N_per_class] [--seed S] [--out file.csv] [--eval [--verbose]]\n");
            return 2;
        }
    }

    SystemConfig cfg = configDefaults();
    cfg.ml.ml_enable = eval ? 1 : 0;
    if (eval && !Classifier_Init()) {
        std::fprintf(stderr, "classifier init failed: %s\n", Classifier_Info().error);
        return 1;
    }

    static FreeRunPacer pacer;
    static SimAdcBackend sim(&pacer);
    static Calibrator cal;
    static PqEngine eng;
    static RawBlock rb;
    static CalibratedBlock cb;
    static EngineOutput out;

    FILE* f = nullptr;
    if (!eval) {
        f = std::fopen(out_path, "w");
        if (!f) {
            std::perror(out_path);
            return 1;
        }
        std::fprintf(f, "# spec_hash=0x%08X nominal_v=%.1f nominal_f=%.1f seed=%u\n", featureSpecHash(),
                     static_cast<double>(cfg.pq.nominalVoltage), static_cast<double>(cfg.pq.nominalFrequency), seed);
        std::fprintf(f, "clip,frame,label,dsp_class,%s\n", featureSpecString());
    }

    Rng rng(seed);
    Confusion c_ml, c_dsp, c_fused;
    // event-level: per clip class -> detected correctly / missed / false alarms
    uint32_t ev_ok[kNumPqClasses] = {0}, ev_n[kNumPqClasses] = {0}, ev_false[kNumPqClasses] = {0};
    double lat_sum[kNumPqClasses] = {0};
    uint32_t false_by[kNumPqClasses][kNumPqClasses] = {{0}};
    uint64_t rows = 0, skipped = 0, rows_per_class[kNumPqClasses] = {0};
    int clip_id = 0;
    for (int n = 0; n < clips; ++n) {
        for (int ci = 0; ci < kNumMlClasses; ++ci) {
            const Clip clip = makeClip(static_cast<PqClass>(ci), rng, cfg);
            sim.generator().configure(clip.gp, cfg.acq.sample_rate_hz);
            sim.generator().clearDisturbances();
            if (clip.has_d) sim.generator().addDisturbance(clip.d);
            AdcHwConfig hw{cfg.acq.sample_rate_hz, {1, 1}, true};
            sim.init(hw);
            sim.start();
            cal.configure(cfg.cal, cfg.acq.sample_rate_hz, cfg.pq.nominalFrequency);
            cal.reset();
            if (const char* e = eng.configure(cfg, cfg.acq.sample_rate_hz)) {
                std::fprintf(stderr, "engine config: %s\n", e);
                return 1;
            }
            const uint32_t nblocks = static_cast<uint32_t>(clip.duration * cfg.acq.sample_rate_hz / kBlockSize);
            bool hit = false, falsealarm = false;
            double lat = 0.0;
            for (uint32_t b = 0; b < nblocks; ++b) {
                sim.readBlock(rb);
                cal.apply(rb, cb);
                eng.processBlock(cb, out);
                for (int k = 0; k < out.nevents; ++k) {
                    const EventRecord& e = out.events[k];
                    if (e.state != EV_START || (e.flags & EVF_ML_ONLY)) continue;
                    PqClass t = e.dsp_type;
                    // a sag that deepens into an interruption starts as SAG
                    const bool match = t == clip.cls || (clip.cls == PqClass::Interruption && t == PqClass::Sag);
                    if (match && !hit) {
                        hit = true;
                        lat = e.det_latency_ms;
                    } else if (!match) {
                        falsealarm = true;
                        ++false_by[ci][static_cast<int>(t)];
                        if (verbose)
                            std::printf("  clip %d (%s, onset %.3f s): extra %s at %.4f s, mag %.3f\n", clip_id,
                                        pqClassShortName(clip.cls), clip.d.t_start, pqClassShortName(t),
                                        e.t_start_us * 1e-6, static_cast<double>(e.magnitude));
                    }
                }
                if (!out.has_frame) continue;
                const FrameResult& fr = out.frame;
                const double w1 = fr.t_end_us * 1e-6, w0 = fr.window_start_us * 1e-6;
                if (w0 < 0.3) continue;              // estimator warm-up
                if (!fr.m.valid) continue;
                const int label = labelFor(clip, w0, w1);
                if (label < 0) {
                    ++skipped;
                    continue;
                }
                // DSP-only decision (fusion without ML)
                DecisionFusion df;
                df.configure(cfg.fusion);
                const FusionResult dsp_only = df.fuse(fr.evidence, ClassificationResult{});
                int dsp_cls = static_cast<int>(dsp_only.final_class == PqClass::Combined ? dsp_only.primary
                                                                                          : dsp_only.final_class);
                if (eval) {
                    if (fr.ml.valid) c_ml.add(label, fr.ml.class_id);
                    c_dsp.add(label, dsp_cls);
                    const PqClass fc = fr.fusion.final_class == PqClass::Combined ? fr.fusion.primary
                                                                                   : fr.fusion.final_class;
                    c_fused.add(label, static_cast<int>(fc));
                } else {
                    std::fprintf(f, "%d,%u,%d,%d", clip_id, fr.frame_idx, label, dsp_cls);
                    for (int k = 0; k < kNumFeatures; ++k)
                        std::fprintf(f, ",%.6g", static_cast<double>(fr.features.v[k]));
                    std::fputc('\n', f);
                }
                ++rows;
                ++rows_per_class[label];
            }
            ++ev_n[ci];
            if (ci == 0) ev_ok[ci] += !falsealarm;
            else if (hit) {
                ++ev_ok[ci];
                lat_sum[ci] += lat;
            }
            ev_false[ci] += falsealarm;
            ++clip_id;
        }
    }
    if (f) std::fclose(f);
    std::fprintf(stderr, "%d clips, %llu labelled frames, %llu ambiguous frames skipped\n", clip_id,
                 static_cast<unsigned long long>(rows), static_cast<unsigned long long>(skipped));
    for (int c = 0; c < kNumMlClasses; ++c)
        std::fprintf(stderr, "  %-20s %llu\n", pqClassName(static_cast<PqClass>(c)),
                     static_cast<unsigned long long>(rows_per_class[c]));
    std::printf("\nEvent level (one disturbance per clip): detection rate, clips with an unrelated extra event,\n"
                "mean detection latency (onset -> event start)\n");
    for (int c = 0; c < kNumMlClasses; ++c)
        std::printf("  %-20s detected %5.1f%%  unrelated-event clips %5.1f%%  latency %6.1f ms\n",
                    pqClassName(static_cast<PqClass>(c)), 100.0 * ev_ok[c] / (ev_n[c] ? ev_n[c] : 1),
                    100.0 * ev_false[c] / (ev_n[c] ? ev_n[c] : 1), c && ev_ok[c] ? lat_sum[c] / ev_ok[c] : 0.0);
    for (int c = 0; c < kNumMlClasses; ++c)
        for (int t = 0; t < kNumPqClasses; ++t)
            if (false_by[c][t])
                std::printf("    in %s clips: %u extra %s event(s)\n", pqClassShortName(static_cast<PqClass>(c)),
                            false_by[c][t], pqClassShortName(static_cast<PqClass>(t)));
    if (eval) {
        c_dsp.print("DSP rules only");
        c_ml.print("TinyML only");
        c_fused.print("Fused decision (firmware output)");
    }
    return 0;
}
