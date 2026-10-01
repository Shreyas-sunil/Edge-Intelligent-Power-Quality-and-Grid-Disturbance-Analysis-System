// Shared helpers: run the full firmware pipeline on a simulated scenario.
#pragma once
#include <vector>

#include "adc/adc_calibration.h"
#include "adc/sim_adc.h"
#include "app/pq_engine.h"

namespace pqtest {

struct PipelineRun {
    std::vector<pq::FrameResult> frames;
    std::vector<pq::EventRecord> events;
    std::vector<float> vpu_half;
    bool trip = false;
    pq::PqClass trip_class = pq::PqClass::Normal;
    pq::EngineStats stats{};
};

// Runs `seconds` of simulated signal through SimAdc -> Calibrator -> PqEngine.
inline PipelineRun runPipeline(const pq::SystemConfig& cfg, const pq::GeneratorParams& gp,
                               const std::vector<pq::Disturbance>& dist, double seconds,
                               float crc_fault_prob = 0.0f) {
    using namespace pq;
    static FreeRunPacer pacer;
    static SimAdcBackend sim(&pacer);
    static Calibrator cal;
    static PqEngine eng;
    static RawBlock rb;
    static CalibratedBlock cb;
    static EngineOutput out;
    PipelineRun r;
    sim.generator().configure(gp, cfg.acq.sample_rate_hz);
    sim.generator().clearDisturbances();
    for (const auto& d : dist) sim.generator().addDisturbance(d);
    AdcHwConfig hw{cfg.acq.sample_rate_hz, {1, 1}, true};
    sim.init(hw);
    sim.injectBadSampleRate(crc_fault_prob);
    sim.injectStall(false);
    sim.start();
    cal.configure(cfg.cal, cfg.acq.sample_rate_hz, cfg.pq.nominalFrequency);
    cal.reset();
    if (eng.configure(cfg, cfg.acq.sample_rate_hz)) return r;
    const uint32_t nb = static_cast<uint32_t>(seconds * cfg.acq.sample_rate_hz / kBlockSize);
    for (uint32_t b = 0; b < nb; ++b) {
        sim.readBlock(rb);
        cal.apply(rb, cb);
        eng.processBlock(cb, out);
        if (out.has_frame) r.frames.push_back(out.frame);
        for (int k = 0; k < out.nevents; ++k) r.events.push_back(out.events[k]);
        if (out.rms_updated) r.vpu_half.push_back(out.vpu_half);
        if (out.trip && !r.trip) {
            r.trip = true;
            r.trip_class = out.trip_class;
        }
    }
    r.stats = eng.stats();
    return r;
}

inline pq::GeneratorParams cleanParams() {
    pq::GeneratorParams g;
    g.noise_v_rms = 0.02f;
    g.noise_i_rms = 0.0005f;
    return g;
}

// Final (END) records of a given DSP type.
inline int countEnded(const PipelineRun& r, pq::PqClass c) {
    int n = 0;
    for (const auto& e : r.events)
        if (e.state == pq::EV_END && e.dsp_type == c && !(e.flags & pq::EVF_ML_ONLY)) ++n;
    return n;
}

inline const pq::EventRecord* firstEnded(const PipelineRun& r, pq::PqClass c) {
    for (const auto& e : r.events)
        if (e.state == pq::EV_END && e.dsp_type == c) return &e;
    return nullptr;
}

inline int countEndedAny(const PipelineRun& r) {
    int n = 0;
    for (const auto& e : r.events)
        if (e.state == pq::EV_END && !(e.flags & pq::EVF_ML_ONLY)) ++n;
    return n;
}

}  // namespace pqtest
