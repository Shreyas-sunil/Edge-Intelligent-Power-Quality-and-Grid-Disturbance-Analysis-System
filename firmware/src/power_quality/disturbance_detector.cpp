#include "power_quality/disturbance_detector.h"

#include <cmath>

#include "common/math_util.h"

namespace pq {

namespace {
inline float ramp(float d, float width) { return clampv(0.5f + d / width, 0.0f, 1.0f); }
}  // namespace

void DisturbanceDetector::configure(const PQConfig& cfg, float fs) {
    cfg_ = cfg;
    fs_ = fs;
    cycle_us_ = static_cast<uint64_t>(1e6f / cfg.nominalFrequency);
    reset();
}

void DisturbanceDetector::reset() {
    v_state_ = VState::Normal;
    pending_n_ = 0;
    pending_state_ = VState::Normal;
    volt_ = Open{};
    last_u_ = 1.0f;
    volt_last_end_us_ = 0;
    load_ = Open{};
    y_ref_ = -1.0f;
    y_prev_ = 0.0f;
    y_stable_ = 0;
    load_last_start_us_ = 0;
    tr_pending_ = tr_done_ = false;
    trans_ = Open{};
    harm_ = Open{};
    freq_ = Open{};
    freq_count_ = 0;
    nnotes_ = 0;
    last_freq_ = cfg_.nominalFrequency;
    last_thd_ = 0.0f;
    last_vrms_ = cfg_.nominalVoltage;
    volt_last_type_ = PqClass::Normal;
    trans_confirmed_us_ = 0;
}

void DisturbanceDetector::emit(const EventRecord& r) {
    if (nnotes_ < kMaxNotes) notes_[nnotes_++] = r;
    else ++notes_dropped_;
}

int DisturbanceDetector::drain(EventRecord* out, int max) {
    const int n = nnotes_ < max ? nnotes_ : max;
    for (int k = 0; k < n; ++k) out[k] = notes_[k];
    for (int k = n; k < nnotes_; ++k) notes_[k - n] = notes_[k];
    nnotes_ -= n;
    return n;
}

uint16_t DisturbanceDetector::activeMask() const {
    uint16_t m = 0;
    if (volt_.active) m |= classBit(volt_.rec.dsp_type);
    if (load_.active) m |= classBit(PqClass::LoadSwitching);
    if (harm_.active) m |= classBit(PqClass::Harmonic);
    if (freq_.active) m |= classBit(PqClass::FrequencyVariation);
    return m;
}

PqClass DisturbanceDetector::voltageState() const {
    switch (v_state_) {
        case VState::Sag: return PqClass::Sag;
        case VState::Swell: return PqClass::Swell;
        case VState::Interruption: return PqClass::Interruption;
        default: return PqClass::Normal;
    }
}

void DisturbanceDetector::open(Open& o, PqClass c, uint64_t onset_us, uint64_t now_us, float mag) {
    o.active = true;
    o.last_us = now_us;
    EventRecord& r = o.rec;
    r = EventRecord{};
    r.id = next_id_++;
    r.dsp_type = r.final_type = c;
    r.ml_type = PqClass::Normal;
    r.state = EV_START;
    r.source = SRC_DSP;
    r.t_start_us = onset_us;
    r.magnitude = mag;
    r.vrms = last_vrms_;
    r.freq = last_freq_;
    r.thd = last_thd_;
    r.det_latency_ms = now_us > onset_us ? (now_us - onset_us) * 1e-3f : 0.0f;
    // overlapping families -> components
    const uint16_t others = activeMask() & static_cast<uint16_t>(~classBit(c));
    r.components = static_cast<uint16_t>(classBit(c) | others);
    Open* all[] = {&volt_, &load_, &harm_, &freq_};
    for (Open* x : all)
        if (x != &o && x->active) x->rec.components |= classBit(c);
    emit(r);
}

void DisturbanceDetector::close(Open& o, uint64_t end_us) {
    if (!o.active) return;
    EventRecord& r = o.rec;
    r.state = EV_END;
    r.duration_ms = end_us > r.t_start_us ? static_cast<uint32_t>((end_us - r.t_start_us) / 1000u) : 0u;
    emit(r);
    o.active = false;
}

void DisturbanceDetector::markClipped() {
    if (volt_.active) volt_.rec.flags |= EVF_CLIPPED;
}

void DisturbanceDetector::markTripped(PqClass c) {
    Open* all[] = {&volt_, &load_, &harm_, &freq_, &trans_};
    for (Open* x : all)
        if (x->active && (x->rec.dsp_type == c || c == PqClass::Combined)) x->rec.flags |= EVF_TRIPPED;
}

bool DisturbanceDetector::recentlyActive(const Open& o, uint64_t since_us) const {
    return o.active || o.last_us >= since_us;
}

void DisturbanceDetector::onHalfCycle(const HalfCycleResult& r, uint64_t t_start_us, uint64_t t_end_us,
                                      uint64_t now_us) {
    const float u = r.v_rms / cfg_.nominalVoltage;
    last_u_ = u;

    // ---------------------------------------------------------- RMS events
    // Windows not delimited by zero crossings (interruption, re-sync) are only
    // trusted for the interruption decision; they never open or close sag/swell.
    const bool sync = r.synced;
    if (v_state_ == VState::Normal) {
        VState cand = VState::Normal;
        if (u < cfg_.interruptionThreshold) cand = VState::Interruption;
        else if (!sync) cand = VState::Normal;
        else if (u < cfg_.sagThreshold) cand = VState::Sag;
        else if (u > cfg_.swellThreshold) cand = VState::Swell;
        if (cand == VState::Normal) {
            pending_n_ = 0;
        } else {
            const bool same_dir = (cand == VState::Swell) == (pending_state_ == VState::Swell);
            pending_n_ = (pending_n_ > 0 && same_dir) ? pending_n_ + 1 : 1;
            if (pending_n_ == 1 || cand == VState::Interruption) pending_state_ = cand;
            if (pending_n_ >= cfg_.minEventHalfCycles) {
                v_state_ = pending_state_;
                const PqClass c = voltageState();
                open(volt_, c, t_start_us, now_us, u);
                volt_last_type_ = c;
                pending_n_ = 0;
            }
        }
    } else {
        volt_.last_us = now_us;
        EventRecord& rec = volt_.rec;
        if (v_state_ == VState::Swell) {
            if (sync && u > rec.magnitude) rec.magnitude = u;
            if (sync && u < cfg_.swellThreshold - cfg_.rmsHysteresis) {
                close(volt_, t_end_us);
                v_state_ = VState::Normal;
                volt_last_end_us_ = now_us;
            }
        } else {
            if (u < rec.magnitude) rec.magnitude = u;
            if (u < cfg_.interruptionThreshold && v_state_ == VState::Sag) {
                v_state_ = VState::Interruption;   // escalate the same event
                rec.dsp_type = rec.final_type = PqClass::Interruption;
                rec.components |= classBit(PqClass::Interruption);
                volt_last_type_ = PqClass::Interruption;
            }
            if (sync && u > cfg_.sagThreshold + cfg_.rmsHysteresis) {
                close(volt_, t_end_us);
                v_state_ = VState::Normal;
                volt_last_end_us_ = now_us;
            }
        }
    }

    // ------------------------------------------------------ load switching
    const float floor_a = cfg_.loadMinCurrent > 1e-3f ? cfg_.loadMinCurrent : 1e-3f;
    if (u > 0.5f && sync) {
        const float y = r.i_rms / u;
        if (y_ref_ < 0.0f) y_ref_ = y;
        if (!load_.active) {
            const bool volt_busy = volt_.active || (now_us - volt_last_end_us_) < 3 * cycle_us_ ||
                                   v_state_ != VState::Normal || pending_n_ > 0;
            if (volt_busy) {
                y_ref_ = y;
            } else {
                const float d = y - y_ref_;
                const float base = y_ref_ > floor_a ? y_ref_ : floor_a;
                if (std::fabs(d) / base > cfg_.loadStepThreshold && std::fabs(d) > floor_a) {
                    open(load_, PqClass::LoadSwitching, t_start_us, now_us, d / base);
                    load_last_start_us_ = now_us;
                    y_stable_ = 0;
                } else {
                    y_ref_ += 0.05f * (y - y_ref_);
                }
            }
        } else {
            load_.last_us = now_us;
            const float base = y_ref_ > floor_a ? y_ref_ : floor_a;
            load_.rec.magnitude = (y - y_ref_) / base;
            const float yb = y > floor_a ? y : floor_a;
            y_stable_ = (std::fabs(y - y_prev_) / yb < 0.05f) ? y_stable_ + 1 : 0;
            if (y_stable_ >= 4) {
                close(load_, t_end_us);
                y_ref_ = y;
            }
        }
        y_prev_ = y;
    }

    // ------------------------------------------ transient confirmation
    if (tr_pending_ && tr_done_ && now_us > tr_end_us_ + (3 * cycle_us_) / 2) {
        // Voltage-event boundaries are only known to ~1 cycle (a trustworthy
        // RMS needs two synchronised halves) -> 3-cycle association window.
        const uint64_t win = 3 * cycle_us_;
        const uint64_t ts = tr_start_us_;
        auto near = [&](uint64_t a) { return (a > ts ? a - ts : ts - a) <= win; };
        bool attached = false;
        if (volt_.active && (volt_.rec.t_start_us <= ts + win)) attached = true;
        if (volt_.rec.id && near(volt_.rec.t_start_us)) attached = true;
        if (volt_last_end_us_ && near(volt_last_end_us_)) attached = true;
        if (load_.rec.id && near(load_.rec.t_start_us)) attached = true;
        if (load_.active) attached = true;
        // A waveform that did not return to its pre-event shape is a step
        // change (level change, phase jump, new harmonic load), not a
        // transient; so is activity lasting more than 2 cycles.
        const bool too_long = tr_info_.step || (tr_end_us_ - tr_start_us_) > 2 * cycle_us_;
        // A transient is NON-repetitive: it must change the waveform relative to
        // the previous cycle. Triggers with a negligible cycle-to-cycle deviation
        // come from periodic content (notching, high-order harmonics, a steady
        // high crest factor) and are steady-state distortion, not transients.
        const float vpk_n = kSqrt2 * cfg_.nominalVoltage;
        const bool significant = tr_info_.peak_dev >= 0.25f * cfg_.transientDevPu * vpk_n;
        if (attached) {
            if (volt_.active) volt_.rec.flags |= EVF_WITH_TRANSIENT;
            if (load_.active) load_.rec.flags |= EVF_WITH_TRANSIENT;
        } else if (too_long || !significant) {
            ++transients_dropped_long_;
        } else {
            const float vpk = kSqrt2 * cfg_.nominalVoltage;
            open(trans_, PqClass::Transient, ts, tr_detect_us_, tr_info_.peak_dev / vpk);
            trans_.last_us = tr_end_us_;
            close(trans_, tr_end_us_);
            trans_confirmed_us_ = tr_end_us_;
        }
        tr_pending_ = false;
    }
}

void DisturbanceDetector::onTransientStart(const TransientInfo& t, uint64_t t_us, uint64_t now_us) {
    if (tr_pending_) {          // merge with a still-unconfirmed transient
        tr_done_ = false;
        return;
    }
    tr_pending_ = true;
    tr_done_ = false;
    tr_info_ = t;
    tr_start_us_ = t_us;
    tr_detect_us_ = now_us;
}

void DisturbanceDetector::onTransientEnd(const TransientInfo& t, uint64_t t_start_us, uint64_t t_end_us) {
    if (!tr_pending_) {
        tr_pending_ = true;
        tr_start_us_ = t_start_us;
        tr_detect_us_ = t_start_us;
    }
    {
        const float keep = tr_info_.peak_dev > t.peak_dev ? tr_info_.peak_dev : t.peak_dev;
        const bool step = tr_done_ ? (tr_info_.step || t.step) : t.step;
        tr_info_ = t;
        tr_info_.peak_dev = keep;
        tr_info_.step = step;
    }
    tr_done_ = true;
    tr_end_us_ = t_end_us;
}

DspEvidence DisturbanceDetector::onFrame(const FrameContext& f) {
    last_freq_ = f.freq_valid ? f.freq : last_freq_;
    last_thd_ = f.thd_v;
    last_vrms_ = f.vrms;

    const bool volt_in_window =
        volt_.active || (volt_last_end_us_ != 0 && volt_last_end_us_ >= f.window_start_us);
    // Spectral quantities are unreliable ("flagged", IEC 61000-4-30) only while
    // the RMS level CHANGES inside the window; a steady sag/swell does not
    // prevent harmonic or frequency assessment.
    const bool step = (f.vmax_pu - f.vmin_pu) > 0.05f;
    // THD is meaningless without a fundamental (interruption / deep sag).
    const bool harm_ok = f.harm_valid && !step && f.vmin_pu > 0.5f;

    // --------------------------------------------------------- harmonics
    if (!harm_.active) {
        if (harm_ok && f.thd_v > cfg_.thdThreshold)
            open(harm_, PqClass::Harmonic, f.window_start_us, f.now_us, f.thd_v);
    } else if (harm_ok) {
        if (f.thd_v > harm_.rec.magnitude) harm_.rec.magnitude = f.thd_v;
        if (f.thd_v > cfg_.thdThreshold - cfg_.thdHysteresis) harm_.last_us = f.now_us;
        else close(harm_, f.now_us);
    }

    // --------------------------------------------------------- frequency
    const float dev = f.freq - cfg_.nominalFrequency;
    if (f.freq_valid && !step && last_u_ > 0.5f) {
        if (std::fabs(dev) > cfg_.frequencyTolerance) {
            ++freq_count_;
            if (!freq_.active && freq_count_ >= cfg_.freqMinFrames)
                open(freq_, PqClass::FrequencyVariation, f.window_start_us, f.now_us, dev);
            if (freq_.active) {
                freq_.last_us = f.now_us;
                if (std::fabs(dev) > std::fabs(freq_.rec.magnitude)) freq_.rec.magnitude = dev;
            }
        } else if (std::fabs(dev) < 0.9f * cfg_.frequencyTolerance) {
            freq_count_ = 0;
            if (freq_.active) close(freq_, f.now_us);
        }
    }

    // ------------------------------------------------------ DSP evidence
    DspEvidence ev{};
    const float e_int = ramp(cfg_.interruptionThreshold - f.vmin_pu, 0.05f);
    ev.e[static_cast<int>(PqClass::Interruption)] = e_int;
    ev.e[static_cast<int>(PqClass::Sag)] = ramp(cfg_.sagThreshold - f.vmin_pu, 0.05f) * (1.0f - e_int);
    ev.e[static_cast<int>(PqClass::Swell)] = ramp(f.vmax_pu - cfg_.swellThreshold, 0.05f);
    ev.e[static_cast<int>(PqClass::Harmonic)] =
        harm_ok ? ramp(f.thd_v - cfg_.thdThreshold, 2.0f) : 0.0f;
    ev.e[static_cast<int>(PqClass::FrequencyVariation)] =
        (f.freq_valid && !step) ? ramp(std::fabs(dev) - cfg_.frequencyTolerance, 0.2f) : 0.0f;
    const bool tr_in_window = trans_confirmed_us_ >= f.window_start_us && trans_confirmed_us_ != 0;
    ev.e[static_cast<int>(PqClass::Transient)] = tr_in_window ? 1.0f : (tr_pending_ ? 0.4f : 0.0f);
    float e_load = ramp(f.y_step - cfg_.loadStepThreshold, 0.1f);
    if (volt_in_window) e_load *= 0.3f;
    ev.e[static_cast<int>(PqClass::LoadSwitching)] = e_load;
    float mx = 0.0f;
    for (int c = 1; c < kNumMlClasses; ++c) mx = ev.e[c] > mx ? ev.e[c] : mx;
    ev.e[static_cast<int>(PqClass::Normal)] = 1.0f - mx;

    uint16_t a = 0;
    if (volt_in_window) a |= classBit(volt_.active ? volt_.rec.dsp_type : volt_last_type_);
    if (recentlyActive(load_, f.window_start_us) && load_.rec.id) a |= classBit(PqClass::LoadSwitching);
    if (harm_.active) a |= classBit(PqClass::Harmonic);
    if (freq_.active) a |= classBit(PqClass::FrequencyVariation);
    if (tr_in_window) a |= classBit(PqClass::Transient);
    ev.asserted = a;
    return ev;
}

}  // namespace pq
