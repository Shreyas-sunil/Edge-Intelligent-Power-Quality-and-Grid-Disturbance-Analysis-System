// =============================================================================
//  PqEngine — the complete real-time analysis pipeline, platform independent.
//
//    CalibratedBlock (V, A)
//      -> DC blocker -> zero-crossing frequency -> half-cycle RMS -> detector
//      -> wavelet + transient detector (sample by sample)
//      -> half-band decimation -> analysis ring
//      every `hop` decimated samples:
//      -> Hann FFT (V and I) -> harmonics / THD / spectral peaks
//      -> window statistics -> frame-level detection -> features
//      -> TinyML -> decision fusion -> FrameResult (+ event notes)
//
//  The engine never blocks, never allocates, and is driven by processBlock()
//  on the real-time core. Exactly the same object runs on the Pico (core1) and
//  in the host tests / simulator / dataset generator.
// =============================================================================
#pragma once
#include <cstdint>

#include "adc/adc_calibration.h"
#include "dsp/filtering.h"
#include "dsp/frequency.h"
#include "dsp/rms.h"
#include "dsp/thd.h"
#include "dsp/transient.h"
#include "dsp/wavelet.h"
#include "features/feature_extractor.h"
#include "fusion/decision_fusion.h"
#include "power_quality/disturbance_detector.h"
#include "system/config.h"
#include "tinyml/classifier.h"

namespace pq {

struct StageTiming {
    uint32_t per_sample_us;   // conditioning+RMS+freq+wavelet+transient+decimation over the hop
    uint32_t spectral_us;     // FFT + harmonic grouping, both channels
    uint32_t window_us;       // window statistics
    uint32_t detect_us;       // frame-level detection + evidence
    uint32_t feature_us;
    uint32_t ml_us;           // Classifier_Run total (normalise+quantise+invoke)
    uint32_t fusion_us;
    uint32_t frame_us;        // analyzeFrame total
};

struct FrameResult {
    uint32_t             frame_idx;
    uint64_t             t_end_us;          // sample time of the last sample in the window
    uint64_t             window_start_us;
    PqMetrics            m;
    float                y_step;
    FeatureVector        features;
    ClassificationResult ml;
    DspEvidence          evidence;
    FusionResult         fusion;
    StageTiming          timing;
};

struct EngineOutput {
    bool        has_frame;
    FrameResult frame;
    int         nevents;
    EventRecord events[24];
    bool        trip;               // SSCB trip request (fast path)
    PqClass     trip_class;
    float       trip_value;
    bool        rms_updated;        // a half cycle completed in this block
    float       vrms_half;          // latest Urms(1/2) [V]
    float       vpu_half;
    PqClass     voltage_state;      // Normal / Sag / Swell / Interruption
    uint32_t    block_us;           // processing time of this block (incl. frame)
};

struct EngineStats {
    uint64_t samples;
    uint32_t blocks;
    uint32_t frames;
    uint32_t frames_invalid;
    uint32_t gaps;
    uint32_t events;
    uint32_t transients_raw;
    uint32_t max_block_us;
    uint32_t max_frame_us;
};

class PqEngine {
public:
    // Returns nullptr on success, else an error string.
    const char* configure(const SystemConfig& cfg, float fs);
    void        reset();
    void        processBlock(const CalibratedBlock& b, EngineOutput& out);

    const EngineStats& stats() const { return stats_; }
    const SystemConfig& config() const { return cfg_; }
    float fs() const { return fs_; }
    float analysisRate() const { return fs_ / dec_; }
    int   fftSize() const { return nfft_; }
    int   hop() const { return hop_; }
    bool  mlReady() const { return ml_ready_; }
    // Hot-apply parameters that do not need a restart (thresholds, flags, ...).
    void  updateSoftConfig(const SystemConfig& cfg);
    const HarmonicAnalyzer& analyzer() const { return an_; }

    // Access for tests / benchmark.
    const FrequencyEstimator& frequency() const { return freq_; }
    const DisturbanceDetector& detector() const { return det_; }

private:
    struct HopStats {
        float    s1, s2, s3, s4;       // moments of v / Vpk_nom
        float    ev, ei, evi;          // sum v^2, i^2, v*i (V^2, A^2, W)
        float    sum_v_raw, sum_i_raw; // before DC blocker
        float    peak;                 // max |v|
        float    wave_e, dev_e;
        float    vmin_half, vmax_half;
        float    imin, imax;
        float    ymin, ymax, ysum;
        float    hc_ev, hc_ei, hc_len;   // whole half cycles completed in this hop
        uint32_t ny;
        uint32_t n;
        uint32_t nhalf;
        uint32_t bad;
        uint32_t sat;
        bool     gap;
    };
    void configureTransient();
    // Detection is suppressed while filters / frequency tracking settle after a
    // (re)start: 5 nominal cycles.
    int64_t warmupSamples() const { return static_cast<int64_t>(5.0f * fs_ / cfg_.pq.nominalFrequency); }
    int64_t warmup_ = 0;
    void clearHop(HopStats& h);
    void analyzeFrame(uint64_t now_base_us, uint64_t proc_start_us, EngineOutput& out);
    void classifyPendingEvents(const FrameResult& fr, uint64_t now_us, EngineOutput& out);
    void pushEvent(EngineOutput& out, const EventRecord& r);
    uint64_t sampleToUs(double n) const {
        return static_cast<uint64_t>(t_origin_us_ + n * us_per_sample_);
    }

    SystemConfig cfg_{};
    float  fs_ = 10000.0f;
    int    dec_ = 2, nfft_ = 1024, hop_ = 512, nhops_ = 2;
    float  vpk_ = 325.0f;
    double t_origin_us_ = 0.0, us_per_sample_ = 100.0;
    bool   have_origin_ = false;

    DcBlocker          dcb_v_, dcb_i_;
    FrequencyEstimator freq_;
    HalfCycleRms       rms_;
    StreamingDwt       dwt_;
    TransientDetector  trans_;
    HalfbandDecimator  decv_, deci_;
    HarmonicAnalyzer   an_;
    DisturbanceDetector det_;
    DecisionFusion     fusion_;
    bool               ml_ready_ = false;

    float    ring_v_[PQ_FFT_MAX];
    float    ring_i_[PQ_FFT_MAX];
    float    lin_[PQ_FFT_MAX];
    int      ring_w_ = 0;
    uint32_t dec_count_ = 0;       // decimated samples since reset
    int      hop_count_ = 0;
    HopStats hops_[8];
    int      hop_idx_ = 0;
    uint32_t hops_done_ = 0;
    uint32_t hops_since_gap_ = 0;  // window valid once >= nhops_
    int64_t  last_sample_ = 0;
    uint32_t per_sample_us_acc_ = 0;

    HalfCycleResult last_hc_{};
    bool     have_hc_ = false;
    uint32_t frame_idx_ = 0;

    // Events awaiting their first fused classification.
    struct PendingCls {
        EventRecord rec;
        bool        used;
    };
    PendingCls pend_[8];
    EventRecord advisory_{};
    bool        advisory_open_ = false;
    uint32_t    advisory_id_ = 0x80000000u;

    struct ClsCache {
        uint32_t id;
        PqClass  final_type, ml_type;
        float    ml_conf, fused_conf, cls_latency_ms;
        uint8_t  source;
        uint16_t flags;
    };
    static constexpr int kClsCache = 8;
    ClsCache cls_cache_[kClsCache]{};
    int      cls_idx_ = 0;

    EngineStats stats_{};
};

}  // namespace pq
