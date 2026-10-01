// =============================================================================
//  Real-time task (runs alone on core1 on the Pico).
//
//  Loop (non-blocking state machine, one call of poll() per iteration):
//     commands -> [ADC block ready?] -> calibrate -> PqEngine -> fast outputs
//     (trip / DVR reference / event code) -> queue frame + events to core0
//     -> health supervision of the ADC (timeouts, recovery) -> status publish
// =============================================================================
#pragma once
#include "adc/adc_calibration.h"
#include "adc/adc_driver.h"
#include "adc/sim_adc.h"
#include "app/pq_engine.h"
#include "app/rt_link.h"
#include "outputs/control_output.h"

namespace pq {

class RealtimeTask {
public:
    // hw may be null (simulation-only build/board). sim must be non-null.
    void init(RtShared* link, AdcBackend* hw, SimAdcBackend* sim, ControlHw* ctrl_hw, const SystemConfig& cfg);
    // One iteration. Returns true if any block was processed.
    bool poll();
    // Process at most `max_blocks` blocks per poll (keeps command latency low).
    void setMaxBlocksPerPoll(int n) { max_blocks_ = n; }

    const PqEngine& engine() const { return engine_; }
    SimAdcBackend*  sim() { return sim_; }
    bool running() const { return running_; }

private:
    bool startAcquisition();
    void stopAcquisition();
    void applyConfig(const SystemConfig& c);
    void handleCommand(const RtCmd& c);
    void handleOutput(const EngineOutput& out, uint64_t now);
    void superviseAdc(uint64_t now);
    void superviseCounters();
    uint32_t countersToErrors();
    void publish(uint64_t now);
    AdcBackend* activeBackend() const;

    RtShared*     link_ = nullptr;
    AdcBackend*   hw_ = nullptr;
    SimAdcBackend* sim_ = nullptr;
    SystemConfig  cfg_{};
    Calibrator    cal_;
    PqEngine      engine_;
    ControlOutput ctrl_;
    RawBlock        raw_;
    CalibratedBlock calb_;
    EngineOutput    out_;
    bool     running_ = false;
    bool     want_running_ = true;
    int      max_blocks_ = 4;
    float    fs_ = 10000.0f;

    // waveform history (calibrated)
    float    hist_v_[PQ_WAVE_SNAPSHOT];
    float    hist_i_[PQ_WAVE_SNAPSHOT];
    uint32_t hist_w_ = 0;
    uint64_t hist_last_seq_ = 0;

    // health
    uint32_t errors_ = 0;
    uint32_t errors_now_ = 0;
    uint64_t last_block_us_ = 0;
    uint64_t last_recover_us_ = 0;
    uint32_t reinit_count_ = 0;
    uint32_t prev_overruns_ = 0, prev_crc_ = 0, prev_sat_ = 0;
    const char* cfg_error_ = nullptr;

    // load measurement
    uint64_t win_start_us_ = 0;
    uint64_t busy_us_ = 0, gen_us_ = 0;
    float    cpu_load_ = 0.0f, cpu_total_ = 0.0f, cpu_peak_ = 0.0f;
    uint64_t last_pub_us_ = 0;
    uint32_t blocks_since_pub_ = 0;
    uint32_t raw_resets_ = 0;
    uint64_t start_us_ = 0;
    uint32_t frames_dropped_ = 0, events_dropped_ = 0;
    uint32_t active_ids_[8]{};
    PqClass  active_cls_[8]{};
    int      nactive_ = 0;
};

}  // namespace pq
