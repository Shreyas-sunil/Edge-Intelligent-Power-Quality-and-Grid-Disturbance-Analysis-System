// =============================================================================
//  Simulated ADC backend.
//
//  Produces RawBlocks in ADC counts from WaveformGenerator, paced by a
//  hardware timer on the Pico (block period = kBlockSize / fs) exactly like the
//  DMA "block complete" of the real ADC, so the rest of the firmware cannot
//  tell the difference. It honours the same ring-depth / overrun semantics and
//  supports fault injection (CRC errors, stalled ADC) for error-path testing.
// =============================================================================
#pragma once
#include "adc/adc_interface.h"
#include "adc/waveform_generator.h"

namespace pq {

// Time source that says how many blocks "the DMA would have completed" since
// start. Pico: hardware alarm (platform/pico/sim_pacer_pico.cpp).
class SimPacer {
public:
    virtual ~SimPacer() = default;
    virtual void     start(uint32_t block_period_us) = 0;
    virtual void     stop() = 0;
    virtual uint32_t blocksDue() = 0;     // monotonic since start()
    virtual uint64_t startTimeUs() = 0;
};

// Free-running pacer: a block is always due. Used by host tests / dataset
// generation where we want to run faster than real time.
class FreeRunPacer : public SimPacer {
public:
    void     start(uint32_t) override { n_ = 0; }
    void     stop() override {}
    uint32_t blocksDue() override { return ++n_; }
    uint64_t startTimeUs() override { return 0; }
private:
    uint32_t n_ = 0;
};

class SimAdcBackend : public AdcBackend {
public:
    explicit SimAdcBackend(SimPacer* pacer) : pacer_(pacer) {}

    const char* name() const override { return "simulator"; }
    AdcStatus   init(const AdcHwConfig& cfg) override;
    AdcStatus   start() override;
    void        stop() override;
    bool        readBlock(RawBlock& out) override;
    AdcStatus   poll() override;
    float       actualSampleRate() const override { return fs_; }
    int32_t     fullScaleCounts() const override { return gen_.params().full_scale; }
    bool        isSimulated() const override { return true; }
    const AdcDiagnostics& diagnostics() const override { return diag_; }

    WaveformGenerator& generator() { return gen_; }

    // Fault injection (tests / demos of the error handling paths)
    void injectBadSampleRate(float per_sample_probability) { bad_prob_ = per_sample_probability; }
    void injectStall(bool stalled) { stalled_ = stalled; }

private:
    SimPacer*         pacer_;
    WaveformGenerator gen_;
    AdcDiagnostics    diag_{};
    float    fs_ = 10000.0f;
    bool     running_ = false;
    bool     stalled_ = false;
    float    bad_prob_ = 0.0f;
    uint32_t consumed_ = 0;
    uint32_t last_due_ = 0;
    bool     stall_active_ = false;
    bool     stall_timeout_ = false;
    uint32_t stall_start_due_ = 0;
    uint64_t seq_ = 0;
    uint64_t t0_us_ = 0;
    bool     gap_pending_ = false;
    Rng      fault_rng_{0xC0FFEEu};
};

}  // namespace pq
