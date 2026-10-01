#include "app/realtime_task.h"

#include <cstring>

#include "common/platform.h"
#include "system/log_sink.h"

namespace pq {

namespace {
bool restartNeeded(const SystemConfig& a, const SystemConfig& b) {
    size_t n;
    const ParamDesc* ps = configParams(n);
    for (size_t k = 0; k < n; ++k) {
        if (!(ps[k].flags & PF_RESTART_ACQ)) continue;
        float x = 0, y = 0;
        configGet(a, ps[k], x);
        configGet(b, ps[k], y);
        if (x != y) return true;
    }
    return a.cal.phase_deg != b.cal.phase_deg;
}
}  // namespace

void RealtimeTask::init(RtShared* link, AdcBackend* hw, SimAdcBackend* sim, ControlHw* ctrl_hw,
                        const SystemConfig& cfg) {
    link_ = link;
    hw_ = hw;
    sim_ = sim;
    cfg_ = cfg;
    if (cfg_.acq.source == static_cast<uint8_t>(AdcSource::Hardware) && !hw_)
        cfg_.acq.source = static_cast<uint8_t>(AdcSource::Simulated);
    ctrl_.init(ctrl_hw, cfg_.ctrl);
    ControlOutput_Bind(&ctrl_);
    start_us_ = plat::timeUs();
    win_start_us_ = start_us_;
    want_running_ = true;
    startAcquisition();
    publish(plat::timeUs());
}

AdcBackend* RealtimeTask::activeBackend() const {
    return (cfg_.acq.source == static_cast<uint8_t>(AdcSource::Hardware) && hw_) ? hw_
                                                                                  : static_cast<AdcBackend*>(sim_);
}

bool RealtimeTask::startAcquisition() {
    AdcBackend* be = activeBackend();
    AdcHwConfig hc{};
    hc.sample_rate_hz = cfg_.acq.sample_rate_hz;
    hc.pga_gain[kChVoltage] = cfg_.acq.v_pga_gain;
    hc.pga_gain[kChCurrent] = cfg_.acq.i_pga_gain;
    hc.crc_check = cfg_.acq.crc_check != 0;
    AdcStatus st = ADC_Init(be, hc);
    if (st != AdcStatus::Ok) {
        errors_ |= ERR_ADC_COMM;
        errors_now_ |= ERR_ADC_COMM;
        logf(LogLevel::Error, "adc", "%s init failed: %s", be->name(), adcStatusName(st));
        running_ = false;
        return false;
    }
    fs_ = be->actualSampleRate();
    cal_.configure(cfg_.cal, fs_, cfg_.pq.nominalFrequency);
    cal_.reset();
    cfg_error_ = engine_.configure(cfg_, fs_);
    if (cfg_error_) {
        errors_ |= ERR_CONFIG_INVALID;
        logf(LogLevel::Error, "cfg", "engine config rejected: %s", cfg_error_);
        running_ = false;
        return false;
    }
    st = ADC_Start();
    if (st != AdcStatus::Ok) {
        errors_ |= ERR_ADC_COMM;
        errors_now_ |= ERR_ADC_COMM;
        logf(LogLevel::Error, "adc", "start failed: %s", adcStatusName(st));
        running_ = false;
        return false;
    }
    errors_now_ &= ~ERR_ADC_COMM;
    running_ = true;
    last_block_us_ = plat::timeUs();
    const AdcDiagnostics& d = be->diagnostics();
    prev_overruns_ = d.overruns;
    prev_crc_ = d.crc_errors + d.status_errors;
    prev_sat_ = d.saturated;
    logf(LogLevel::Info, "rt", "acquisition started: %s @ %.1f Hz", be->name(), static_cast<double>(fs_));
    return true;
}

void RealtimeTask::stopAcquisition() {
    ADC_Stop();
    running_ = false;
}

void RealtimeTask::applyConfig(const SystemConfig& c) {
    const bool restart = restartNeeded(cfg_, c);
    SystemConfig next = c;
    if (next.acq.source == static_cast<uint8_t>(AdcSource::Hardware) && !hw_) {
        next.acq.source = static_cast<uint8_t>(AdcSource::Simulated);
        logSink(LogLevel::Warn, "rt", "no hardware ADC backend in this build; staying on simulator");
    }
    cfg_ = next;
    ctrl_.configure(cfg_.ctrl);
    if (restart) {
        stopAcquisition();
        if (want_running_) startAcquisition();
        else engine_.configure(cfg_, fs_);
    } else {
        cal_.configure(cfg_.cal, fs_, cfg_.pq.nominalFrequency);
        engine_.updateSoftConfig(cfg_);
    }
}

void RealtimeTask::handleCommand(const RtCmd& c) {
    switch (c.type) {
        case RtCmdType::ApplyConfig: {
            const uint32_t seq = link_->cfg_seq.load(std::memory_order_acquire);
            applyConfig(link_->cfg_mailbox);
            link_->cfg_applied.store(seq, std::memory_order_release);
            break;
        }
        case RtCmdType::Start:
            want_running_ = true;
            if (!running_) startAcquisition();
            break;
        case RtCmdType::Stop:
            want_running_ = false;
            stopAcquisition();
            break;
        case RtCmdType::SimScenario: {
            Rng rng(static_cast<uint32_t>(plat::timeUs()) | 1u);
            Disturbance d;
            const double t0 = sim_->generator().time() + (c.c > 0.0f ? c.c : 0.02f);
            if (makeScenario(c.name, t0, c.a, c.b, rng, d)) {
                if (!sim_->generator().addDisturbance(d)) logSink(LogLevel::Warn, "sim", "disturbance table full");
            }
            break;
        }
        case RtCmdType::SimClear: sim_->generator().clearDisturbances(); break;
        case RtCmdType::SimFaultCrc: sim_->injectBadSampleRate(c.a); break;
        case RtCmdType::SimFaultStall: sim_->injectStall(c.flag != 0); break;
        case RtCmdType::SimBase: {
            GeneratorParams& p = sim_->generator().mutableParams();
            if (!std::strcmp(c.name, "noise")) p.noise_v_rms = c.a;
            else if (!std::strcmp(c.name, "freq")) p.f_offset = c.a;
            else if (!std::strcmp(c.name, "load")) p.i_rms = c.a;
            else if (!std::strcmp(c.name, "level")) p.v_scale = c.a;
            else if (!std::strcmp(c.name, "harm")) { p.base_h[0] = c.a; p.base_h[1] = 0.6f * c.a; p.base_h[2] = 0.4f * c.a; }
            else if (!std::strcmp(c.name, "phase")) p.i_phase_err_deg = c.a;
            else if (!std::strcmp(c.name, "pf")) p.pf_angle_deg = c.a;
            break;
        }
        case RtCmdType::CalRawReset:
            cal_.resetRawStats();
            ++raw_resets_;
            break;
        case RtCmdType::WaveCapture: {
            if (link_->wave_ready.load(std::memory_order_acquire)) break;   // previous not consumed
            const uint32_t N = PQ_WAVE_SNAPSHOT;
            for (uint32_t k = 0; k < N; ++k) {
                const uint32_t r = (hist_w_ + k) % N;
                link_->wave_v[k] = hist_v_[r];
                link_->wave_i[k] = hist_i_[r];
            }
            link_->wave_n = N;
            link_->wave_fs = fs_;
            link_->wave_t0_us = static_cast<uint64_t>((hist_last_seq_ >= N ? hist_last_seq_ - N : 0) * 1e6 / fs_);
            link_->wave_ready.store(1, std::memory_order_release);
            break;
        }
        case RtCmdType::CtrlArm: ctrl_.arm(c.flag != 0); cfg_.ctrl.armed = c.flag; break;
        case RtCmdType::CtrlReset: ctrl_.resetTrip(); break;
        case RtCmdType::CtrlTest: ctrl_.setEvent(static_cast<PqClass>(c.flag % kNumPqClasses), c.flag != 0); break;
        case RtCmdType::StatsReset:
            errors_ = errors_now_;
            cpu_peak_ = 0.0f;
            frames_dropped_ = events_dropped_ = 0;
            break;
    }
}

void RealtimeTask::handleOutput(const EngineOutput& out, uint64_t now) {
    if (out.trip && ctrl_.trip(out.trip_class, out.trip_value, now))
        logf(LogLevel::Warn, "ctrl", "SSCB TRIP (%s, %.3f)", pqClassShortName(out.trip_class),
             static_cast<double>(out.trip_value));
    if (out.rms_updated) ctrl_.updateDvr(out.vpu_half);

    for (int k = 0; k < out.nevents; ++k) {
        EventRecord r = out.events[k];
        if (ctrl_.state().tripped && r.state != EV_START) r.flags |= EVF_TRIPPED;
        if (!(r.flags & EVF_ML_ONLY)) {
            if (r.state == EV_START && nactive_ < 8) {
                active_ids_[nactive_] = r.id;
                active_cls_[nactive_++] = r.dsp_type;
            } else if (r.state == EV_END) {
                for (int j = 0; j < nactive_; ++j)
                    if (active_ids_[j] == r.id) {
                        active_ids_[j] = active_ids_[nactive_ - 1];
                        active_cls_[j] = active_cls_[nactive_ - 1];
                        --nactive_;
                        break;
                    }
            }
            if (r.state != EV_CLASSIFIED) {
                if (nactive_ > 1) ctrl_.setEvent(PqClass::Combined, true);
                else if (nactive_ == 1) ctrl_.setEvent(active_cls_[0], true);
                else ctrl_.setEvent(PqClass::Normal, false);
            }
        }
        if (!link_->events.push(r)) {
            ++events_dropped_;
            errors_ |= ERR_BUFFER_OVERFLOW;
        }
    }
    if (out.has_frame) {
        const PqMetrics& m = out.frame.m;
        ctrl_.sendHarmonicReference(m.i_h, m.i_ph, m.thd_i, out.frame.frame_idx);
        if (!link_->frames.push(out.frame)) {
            ++frames_dropped_;
            errors_ |= ERR_BUFFER_OVERFLOW;
        }
    }
}

void RealtimeTask::superviseCounters() {
    uint32_t e = countersToErrors();
    if (cfg_error_) e |= ERR_CONFIG_INVALID;
    if ((running_ && ADC_Poll() != AdcStatus::Ok) || (!running_ && want_running_)) e |= ERR_ADC_COMM;
    errors_now_ = e;
}

uint32_t RealtimeTask::countersToErrors() {
    AdcBackend* be = activeBackend();
    const AdcDiagnostics& d = be->diagnostics();
    uint32_t now_err = 0;
    if (d.overruns != prev_overruns_) {
        errors_ |= ERR_DMA_OVERRUN | ERR_MISSING_SAMPLES;
        now_err |= ERR_DMA_OVERRUN;
        prev_overruns_ = d.overruns;
    }
    const uint32_t crc = d.crc_errors + d.status_errors;
    if (crc != prev_crc_) {
        errors_ |= ERR_INVALID_SAMPLES;
        now_err |= ERR_INVALID_SAMPLES;
        prev_crc_ = crc;
    }
    if (d.saturated != prev_sat_) {
        errors_ |= ERR_SATURATION;
        now_err |= ERR_SATURATION;
        prev_sat_ = d.saturated;
    }
    return now_err;
}

void RealtimeTask::superviseAdc(uint64_t now) {
    AdcBackend* be = activeBackend();
    uint32_t now_err = countersToErrors();
    // ADC health also follows the backend's own verdict (e.g. simulated stall).
    if (running_) {
        const AdcStatus st = ADC_Poll();
        const uint64_t budget = static_cast<uint64_t>(4.0 * kBlockSize * 1e6 / fs_);   // 4 blocks
        const bool stale = (now - last_block_us_) > budget + 20000;
        if (st == AdcStatus::Timeout || st == AdcStatus::CommFailure || stale) {
            errors_ |= ERR_ADC_COMM | ERR_MISSING_SAMPLES;
            now_err |= ERR_ADC_COMM;
            if (now - last_recover_us_ > 1000000) {   // at most one recovery attempt per second
                last_recover_us_ = now;
                ++reinit_count_;
                logf(LogLevel::Error, "adc", "no data from %s (%s) -> re-initialising", be->name(),
                     adcStatusName(st));
                stopAcquisition();
                startAcquisition();
                last_block_us_ = now;
            }
        }
    } else if (want_running_ && now - last_recover_us_ > 1000000) {
        last_recover_us_ = now;
        ++reinit_count_;
        startAcquisition();
    }
    errors_now_ = now_err;
}

void RealtimeTask::publish(uint64_t now) {
    RtStatus s{};
    AdcBackend* be = activeBackend();
    s.running = running_;
    s.simulated = be->isSimulated();
    s.ml_ready = engine_.mlReady();
    s.backend = be->name();
    s.fs = fs_;
    s.analysis_fs = engine_.analysisRate();
    s.fft_size = engine_.fftSize();
    s.hop = engine_.hop();
    s.adc_status = be->diagnostics().last_error;
    s.adc = be->diagnostics();
    s.eng = engine_.stats();
    s.errors = errors_;
    s.errors_now = errors_now_;
    s.cpu_load_pct = cpu_load_;
    s.cpu_total_pct = cpu_total_;
    s.cpu_peak_pct = cpu_peak_;
    s.block_budget_us = static_cast<uint32_t>(kBlockSize * 1e6f / fs_);
    s.ring_blocks = PQ_ACQ_RING_BLOCKS;
    s.frames_dropped = frames_dropped_;
    s.events_dropped = events_dropped_;
    s.reinit_count = reinit_count_;
    s.uptime_us = now - start_us_;
    s.vrms_half = out_.vrms_half;
    s.voltage_state = out_.voltage_state;
    s.ctrl = ctrl_.state();
    s.raw_mean[0] = cal_.rawMean(0);
    s.raw_mean[1] = cal_.rawMean(1);
    s.raw_n = cal_.rawCount();
    s.raw_resets = raw_resets_;
    s.phase_delay_samples = cal_.phaseDelaySamples();
    s.cfg_applied_seq = link_->cfg_applied.load(std::memory_order_acquire);
    s.last_cfg_error = cfg_error_;
    link_->publishStatus(s);
}

bool RealtimeTask::poll() {
    RtCmd c;
    bool dirty = false;
    while (link_->cmds.pop(c)) {
        handleCommand(c);
        dirty = true;   // make the effect visible to the supervisor immediately
    }

    bool any = false;
    uint64_t now = plat::timeUs();
    if (running_) {
        for (int k = 0; k < max_blocks_; ++k) {
            const uint64_t g0 = plat::timeUs();
            if (!ADC_ReadSamples(raw_)) break;
            const uint64_t t0 = plat::timeUs();
            cal_.apply(raw_, calb_);
            for (uint32_t j = 0; j < calb_.n; ++j) {
                hist_v_[hist_w_] = calb_.v[j];
                hist_i_[hist_w_] = calb_.i[j];
                hist_w_ = (hist_w_ + 1) % PQ_WAVE_SNAPSHOT;
            }
            hist_last_seq_ = calb_.seq + calb_.n;
            engine_.processBlock(calb_, out_);
            handleOutput(out_, plat::timeUs());
            const uint64_t t1 = plat::timeUs();
            busy_us_ += t1 - t0;
            gen_us_ += t0 - g0;
            last_block_us_ = t1;
            any = true;
            ++blocks_since_pub_;
        }
    }
    now = plat::timeUs();
    ctrl_.heartbeat(now);
    // No data this iteration: ask the backend whether it is healthy (cheap), so
    // a dead ADC is handled immediately rather than on the next 50 ms tick.
    if (running_ && !any && ADC_Poll() != AdcStatus::Ok) {
        const uint32_t before = errors_now_;
        superviseAdc(now);
        dirty |= errors_now_ != before;
    }

    if (now - win_start_us_ >= 500000) {
        const float el = static_cast<float>(now - win_start_us_);
        cpu_load_ = 100.0f * static_cast<float>(busy_us_) / el;
        cpu_total_ = 100.0f * static_cast<float>(busy_us_ + gen_us_) / el;
        if (cpu_load_ > cpu_peak_) cpu_peak_ = cpu_load_;
        busy_us_ = gen_us_ = 0;
        win_start_us_ = now;
    }
    // Publish every 50 ms of wall time, and at least every 10 blocks so that
    // faster-than-real-time runs (host tests, --fast) see fresh status too.
    if (dirty || now - last_pub_us_ >= 50000 || blocks_since_pub_ >= 10) {
        if (now - last_pub_us_ >= 50000) superviseAdc(now);
        else superviseCounters();
        publish(now);
        last_pub_us_ = now;
        blocks_since_pub_ = 0;
    }
    return any;
}

}  // namespace pq
