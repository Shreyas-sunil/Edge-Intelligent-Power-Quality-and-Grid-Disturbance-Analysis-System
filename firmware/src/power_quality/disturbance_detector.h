// =============================================================================
//  Deterministic (rule-based) disturbance detector.
//
//  * RMS events (sag / swell / interruption) are evaluated on every half cycle
//    from the one-cycle RMS refreshed each half cycle, with hysteresis
//    (IEC 61000-4-30 style) — detection latency <= ~1 cycle.
//  * Load switching: step in the load admittance proxy y = Irms / Vpu, so a
//    constant-impedance load following a sag is NOT mistaken for switching.
//  * Transients: raised by the sample-level TransientDetector; a transient that
//    coincides (+/-1.5 cycles) with an RMS event or load step is attached to that
//    event (EVF_WITH_TRANSIENT) instead of being reported separately.
//  * Harmonic distortion and frequency variation: evaluated per analysis frame,
//    inhibited ("flagged" in IEC terms) while an RMS step is inside the window,
//    because spectral leakage of a step would otherwise read as distortion.
//  All thresholds come from PQConfig.
// =============================================================================
#pragma once
#include <cstdint>

#include "dsp/rms.h"
#include "dsp/transient.h"
#include "power_quality/pq_events.h"
#include "system/config.h"

namespace pq {

// Per-frame inputs derived by the engine.
struct FrameContext {
    uint64_t now_us;          // time the frame result became available
    uint64_t window_start_us; // sample time of the first sample in the window
    float    thd_v;
    float    freq;
    bool     freq_valid;
    bool     harm_valid;
    float    vmin_pu, vmax_pu;
    float    y_step;          // (y_max - y_min) / y_mean over the window
    float    vrms;
};

// DSP evidence per ML class for decision fusion.
struct DspEvidence {
    float    e[kNumMlClasses];   // 0..1 soft evidence, 0.5 == at threshold
    uint16_t asserted;           // classBit mask of conditions active in the window
};

class DisturbanceDetector {
public:
    static constexpr int kMaxNotes = 16;

    void configure(const PQConfig& cfg, float fs);
    void setConfig(const PQConfig& cfg) { cfg_ = cfg; }   // hot update, keeps state
    void reset();

    void onHalfCycle(const HalfCycleResult& r, uint64_t t_start_us, uint64_t t_end_us, uint64_t now_us);
    void onTransientStart(const TransientInfo& t, uint64_t t_us, uint64_t now_us);
    void onTransientEnd(const TransientInfo& t, uint64_t t_start_us, uint64_t t_end_us);
    DspEvidence onFrame(const FrameContext& f);

    // Mark ADC clipping during the currently open events.
    void markClipped();
    void markTripped(PqClass c);

    // Notes (START / END records) produced since the last drain.
    int drain(EventRecord* out, int max);

    bool voltageEventActive() const { return v_state_ != VState::Normal; }
    PqClass voltageState() const;
    uint16_t activeMask() const;
    float currentVpu() const { return last_u_; }

private:
    enum class VState : uint8_t { Normal, Sag, Swell, Interruption };
    struct Open {
        bool        active = false;
        EventRecord rec{};
        uint64_t    last_us = 0;   // last time the condition was seen
    };
    void open(Open& o, PqClass c, uint64_t onset_us, uint64_t now_us, float mag);
    void close(Open& o, uint64_t end_us);
    void emit(const EventRecord& r);
    bool recentlyActive(const Open& o, uint64_t since_us) const;

    PQConfig cfg_{};
    float    fs_ = 10000.0f;
    uint64_t cycle_us_ = 20000;
    uint32_t next_id_ = 1;

    // voltage
    VState   v_state_ = VState::Normal;
    int      pending_n_ = 0;
    VState   pending_state_ = VState::Normal;
    Open     volt_;
    float    last_u_ = 1.0f;
    uint64_t volt_last_end_us_ = 0;

    // load switching (admittance y = Irms / Vpu)
    Open     load_;
    float    y_ref_ = -1.0f;
    float    y_prev_ = 0.0f;
    int      y_stable_ = 0;
    uint64_t load_last_start_us_ = 0;

    // transient pending confirmation
    bool          tr_pending_ = false;
    bool          tr_done_ = false;
    TransientInfo tr_info_{};
    uint64_t      tr_start_us_ = 0, tr_end_us_ = 0, tr_detect_us_ = 0;
    Open          trans_;

    // frame-level
    Open harm_;
    Open freq_;
    int  freq_count_ = 0;

    EventRecord notes_[kMaxNotes];
    int         nnotes_ = 0;
    uint32_t    notes_dropped_ = 0;

    float    last_freq_ = 50.0f, last_thd_ = 0.0f, last_vrms_ = 230.0f;
    PqClass  volt_last_type_ = PqClass::Normal;
    uint64_t trans_confirmed_us_ = 0;
    uint32_t transients_dropped_long_ = 0;

public:
    uint32_t notesDropped() const { return notes_dropped_; }
};

}  // namespace pq
