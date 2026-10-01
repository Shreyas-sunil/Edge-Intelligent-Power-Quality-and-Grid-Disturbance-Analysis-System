#include "app/supervisor.h"

#include <cmath>
#include <cstdio>
#include <cstring>

#include "adc/waveform_generator.h"
#include "app/benchmark.h"
#include "common/platform.h"
#include "communication/protocol.h"
#include "features/feature_spec.h"
#include "system/log_sink.h"
#include "tinyml/classifier.h"
#include "tinyml/model_data.h"

namespace pq {

// --------------------------------------------------------------------- setup
void Supervisor::init(const SupervisorEnv& env, const SystemConfig& cfg) {
    env_ = env;
    cfg_ = cfg;
    errors_ = env.boot_errors;
    log_.clear();
    log_.setEnabled(cfg.comm.logging_enable != 0);
    alarm_.init(env.alarm_hw);
    boot_us_ = plat::timeUs();
    last_metrics_us_ = boot_us_;
    const ClassifierInfo& ci = Classifier_Info();
    if (!ci.ready && cfg.ml.ml_enable) errors_ |= ERR_ML_INIT;
    // The greeting is sent by poll() whenever the host (re)connects.
}

void Supervisor::sendHello(int dest) {
    const ClassifierInfo& ci = Classifier_Info();
    char b[220];
    const int n = snprintf(b, sizeof(b),
                           "{\"type\":\"hello\",\"fw\":\"%s\",\"protocol\":%d,\"platform\":\"%s\",\"features\":%d,"
                           "\"model\":\"%s\",\"ml_ready\":%s}\n",
                           kFirmwareVersion, kProtocolVersion, plat::platformName(), kNumFeatures,
                           ci.model_version ? ci.model_version : "none", ci.ready ? "true" : "false");
    if (n > 0) send(dest, b, static_cast<size_t>(n));
}

// ----------------------------------------------------------------- transport
void Supervisor::send(int dest, const char* s, size_t n) {
    if (!n) return;
    Transport* ts[2] = {(dest & kSrcUsb) ? env_.usb : nullptr, (dest & kSrcUart) ? env_.uart : nullptr};
    for (Transport* t : ts) {
        if (!t || !t->connected()) continue;
        if (t->writeAvailable() >= n) {
            t->write(s, n);
        } else {
            ++tx_dropped_;
            errors_ |= ERR_COMM_TIMEOUT;   // host is not draining the port
        }
    }
}

void Supervisor::sendStr(int dest, const char* s) { send(dest, s, std::strlen(s)); }

void Supervisor::ack(int src, const char* cmd, const char* msg) {
    JsonWriter w(out_, sizeof(out_));
    w.beginObj().kv("type", "ack").kv("cmd", cmd);
    if (msg) w.kv("msg", msg);
    w.endObj();
    reply(src, out_, w.finish());
}

void Supervisor::err(int src, const char* cmd, const char* msg) {
    JsonWriter w(out_, sizeof(out_));
    w.beginObj().kv("type", "error").kv("cmd", cmd).kv("msg", msg).endObj();
    reply(src, out_, w.finish());
}

void Supervisor::rtCmd(RtCmdType t, uint8_t flag, float a, float b, float c, const char* name) {
    RtCmd cmd{};
    cmd.type = t;
    cmd.flag = flag;
    cmd.a = a;
    cmd.b = b;
    cmd.c = c;
    if (name) {
        std::strncpy(cmd.name, name, sizeof(cmd.name) - 1);
        cmd.name[sizeof(cmd.name) - 1] = 0;
    }
    if (!env_.link->cmds.push(cmd)) errors_ |= ERR_BUFFER_OVERFLOW;
}

bool Supervisor::pushConfig() {
    RtShared* L = env_.link;
    const uint32_t seq = L->cfg_seq.load(std::memory_order_acquire);
    if (L->cfg_applied.load(std::memory_order_acquire) != seq) {   // previous not yet taken
        cfg_dirty_ = true;
        return false;
    }
    L->cfg_mailbox = cfg_;
    L->cfg_seq.store(seq + 1, std::memory_order_release);
    rtCmd(RtCmdType::ApplyConfig);
    cfg_dirty_ = false;
    log_.setEnabled(cfg_.comm.logging_enable != 0);
    return true;
}

void Supervisor::pumpTransport(Transport* t, LineReader& lr, int src) {
    if (!t) return;
    char buf[64];
    char line[LineReader::kMax];
    for (int guard = 0; guard < 8; ++guard) {
        const size_t n = t->read(buf, sizeof(buf));
        if (!n) break;
        last_rx_us_ = plat::timeUs();
        for (size_t k = 0; k < n; ++k)
            if (lr.feed(buf[k], line)) command(line, src);
    }
    // A partial command that never completes is a communication timeout.
    if (lr.partial() && plat::timeUs() - last_rx_us_ > 2000000) {
        lr.clear();
        errors_ |= ERR_COMM_TIMEOUT;
        err(src, "?", "incomplete command discarded (timeout)");
    }
    t->flush();
}

// ---------------------------------------------------------------- main loop
void Supervisor::poll() {
    const uint64_t now = plat::timeUs();
    if (env_.usb) {
        const bool c = env_.usb->connected();
        if (c && !usb_was_connected_) sendHello(kSrcUsb);   // (re)connected: greet
        usb_was_connected_ = c;
    }
    pumpTransport(env_.usb, lr_usb_, 0);
    if (cfg_.comm.uart_enable) pumpTransport(env_.uart, lr_uart_, 1);
    if (cfg_dirty_) pushConfig();

    drainEvents(now);
    drainFrames(now);
    drainLogs();
    periodicMetrics(now);
    pumpWave();
    pumpCalibration(now);

    RtStatus st;
    if (env_.link->readStatus(st)) alarm_.setFault(isFault(st.errors_now) || (errors_ & ERR_ML_INIT));
    alarm_.update(now);
    if (env_.usb) env_.usb->flush();
    if (env_.uart) env_.uart->flush();
}

void Supervisor::drainFrames(uint64_t now) {
    (void)now;
    FrameResult f;
    while (env_.link->frames.pop(f)) {
        timing_.add(f);
        const FusionResult& fu = f.fusion;
        // remember the most significant classification of the reporting period
        if (fu.final_class != PqClass::Normal &&
            (period_class_ == PqClass::Normal || fu.confidence >= period_conf_)) {
            period_class_ = fu.final_class;
            period_conf_ = fu.confidence;
        }
        if (cal_mode_ == CalMode::VGain || cal_mode_ == CalMode::IGain || cal_mode_ == CalMode::Phase) {
            if (f.m.valid) {
                if (cal_mode_ == CalMode::VGain) cal_acc_ += f.m.vrms;
                else if (cal_mode_ == CalMode::IGain) cal_acc_ += f.m.irms;
                else cal_acc_ += f.m.phi_vi_deg;
                ++cal_n_;
            }
        }
        last_ = f;
        have_frame_ = true;
    }
}

void Supervisor::drainEvents(uint64_t now) {
    EventRecord r;
    while (env_.link->events.pop(r)) {
        log_.record(r);
        if (r.state == EV_START || r.state == EV_CLASSIFIED) {
            const AlarmLevel lvl = AlarmOutput::levelFor(r.flags & EVF_ML_ONLY ? PqClass::Normal : r.final_type);
            if (lvl != AlarmLevel::None) alarm_.raise(lvl, 3000, now);
        }
        if (cfg_.comm.event_stream) send(streamDest(), out_, fmtEventJson(out_, sizeof(out_), r));
    }
}

void Supervisor::drainLogs() {
    LogMsg m;
    for (int k = 0; k < 4 && logPop(m); ++k) {
        JsonWriter w(out_, sizeof(out_));
        w.beginObj()
            .kv("type", "log")
            .kv("t_ms", static_cast<double>(m.t_us) * 1e-3, 1)
            .kv("level", logLevelName(m.level))
            .kv("tag", m.tag)
            .kv("msg", m.text)
            .endObj();
        send(streamDest(), out_, w.finish());
    }
}

void Supervisor::periodicMetrics(uint64_t now) {
    if (!have_frame_) return;
    const bool periodic = cfg_.comm.metrics_period_ms > 0 &&
                          now - last_metrics_us_ >= static_cast<uint64_t>(cfg_.comm.metrics_period_ms) * 1000u;
    if (!periodic && !metrics_once_) return;
    RtStatus st{};
    env_.link->readStatus(st);
    const uint32_t errs = st.errors_now | (errors_ & ERR_ML_INIT);
    PqClass ev = period_class_;
    float conf = period_conf_;
    if (ev == PqClass::Normal) conf = last_.fusion.fused[0];
    const uint64_t ts = last_.t_end_us / 1000u;
    size_t n;
    if (csv_header_pending_ && cfg_.comm.metrics_format == 1) {
        n = fmtMetricsCsvHeader(out_, sizeof(out_));
        send(metrics_once_ ? (metrics_once_src_ == 1 ? kSrcUart : kSrcUsb) : streamDest(), out_, n);
        csv_header_pending_ = false;
    }
    if (cfg_.comm.metrics_format == 1) n = fmtMetricsCsv(out_, sizeof(out_), last_, ev, conf, ts, errs);
    else n = fmtMetricsJson(out_, sizeof(out_), last_, ev, conf, ts, errs);
    if (metrics_once_) {
        reply(metrics_once_src_, out_, n);
        metrics_once_ = false;
    } else {
        send(streamDest(), out_, n);
    }
    if (periodic) {
        last_metrics_us_ = now;
        period_class_ = PqClass::Normal;
        period_conf_ = 0.0f;
    }
}

void Supervisor::pumpWave() {
    RtShared* L = env_.link;
    if (!wave_pending_) return;
    if (!L->wave_ready.load(std::memory_order_acquire)) {
        if (plat::timeUs() - wave_req_us_ > 1000000) {
            wave_pending_ = false;
            err(wave_src_, "WAVE", "capture timeout (acquisition stopped?)");
        }
        return;
    }
    const uint32_t N = L->wave_n < wave_req_n_ ? L->wave_n : wave_req_n_;
    const uint32_t first = L->wave_n - N;   // most recent N samples
    Transport* t = wave_src_ == 1 ? env_.uart : env_.usb;
    for (int guard = 0; guard < 4 && wave_off_ < N; ++guard) {
        const uint32_t chunk = (N - wave_off_) < 32 ? (N - wave_off_) : 32;
        JsonWriter w(out_, sizeof(out_));
        w.beginObj()
            .kv("type", "wave")
            .kv("n", N)
            .kv("offset", wave_off_)
            .kv("fs", L->wave_fs, 1)
            .kv("t0_ms", static_cast<double>(L->wave_t0_us + static_cast<uint64_t>(first * 1e6 / L->wave_fs)) * 1e-3, 1);
        w.beginArr("v");
        for (uint32_t k = 0; k < chunk; ++k) w.val(L->wave_v[first + wave_off_ + k], 2);
        w.endArr().beginArr("i");
        for (uint32_t k = 0; k < chunk; ++k) w.val(L->wave_i[first + wave_off_ + k], 4);
        w.endArr().endObj();
        const size_t n = w.finish();
        if (!t || !t->connected()) {
            wave_off_ = N;
            break;
        }
        if (t->writeAvailable() < n) break;   // resume on the next poll
        t->write(out_, n);
        wave_off_ += chunk;
    }
    if (wave_off_ >= N) {
        wave_pending_ = false;
        L->wave_ready.store(0, std::memory_order_release);
    }
}

void Supervisor::pumpCalibration(uint64_t now) {
    if (cal_mode_ == CalMode::None) return;
    const char* name = cal_mode_ == CalMode::Offset ? "OFFSET"
                       : cal_mode_ == CalMode::VGain ? "VGAIN"
                       : cal_mode_ == CalMode::IGain ? "IGAIN" : "PHASE";
    JsonWriter w(out_, sizeof(out_));
    if (cal_mode_ == CalMode::Offset) {
        RtStatus st;
        if (!env_.link->readStatus(st)) return;
        if (st.raw_resets != cal_reset_seq_ || st.raw_n < cal_need_) {
            if (now < cal_deadline_us_) return;
            err(cal_src_, "CALIBRATE", "not enough samples collected (acquisition stopped?)");
            cal_mode_ = CalMode::None;
            return;
        }
        cfg_.cal.v_offset = static_cast<float>(st.raw_mean[kChVoltage]);
        cfg_.cal.i_offset = static_cast<float>(st.raw_mean[kChCurrent]);
        pushConfig();
        w.beginObj().kv("type", "cal").kv("step", name).kv("v_offset", cfg_.cal.v_offset, 2)
            .kv("i_offset", cfg_.cal.i_offset, 2).kv("samples", st.raw_n).kv("hint", "CONFIG SAVE to persist").endObj();
        reply(cal_src_, out_, w.finish());
        cal_mode_ = CalMode::None;
        return;
    }
    if (cal_n_ < cal_need_) {
        if (now > cal_deadline_us_) {
            err(cal_src_, "CALIBRATE", "timeout waiting for valid frames");
            cal_mode_ = CalMode::None;
        }
        return;
    }
    const float avg = static_cast<float>(cal_acc_ / cal_n_);
    w.beginObj().kv("type", "cal").kv("step", name).kv("measured", avg, 4).kv("reference", cal_ref_, 4);
    bool ok = true;
    if (cal_mode_ == CalMode::VGain || cal_mode_ == CalMode::IGain) {
        if (avg < 1e-6f) ok = false;
        else if (cal_mode_ == CalMode::VGain) {
            cfg_.cal.v_gain *= cal_ref_ / avg;
            w.kv("v_gain", static_cast<double>(cfg_.cal.v_gain), 9);
        } else {
            cfg_.cal.i_gain *= cal_ref_ / avg;
            w.kv("i_gain", static_cast<double>(cfg_.cal.i_gain), 10);
        }
    } else {
        // phi_vi = V1 angle - I1 angle. A sensor making the current lead by e
        // reads phi = ref - (e - c); the correction c must equal e.
        float c = cfg_.cal.phase_deg - (avg - cal_ref_);
        if (c < -20.0f || c > 20.0f) ok = false;
        else cfg_.cal.phase_deg = c;
        w.kv("phase_deg", cfg_.cal.phase_deg, 3);
    }
    if (ok && configValidate(cfg_) == nullptr) {
        pushConfig();
        w.kv("hint", "CONFIG SAVE to persist").endObj();
        reply(cal_src_, out_, w.finish());
    } else {
        err(cal_src_, "CALIBRATE", "result out of range - check the reference signal");
    }
    cal_mode_ = CalMode::None;
}

// -------------------------------------------------------------------- commands
void Supervisor::command(const char* line_in, int src) {
    char line[LineReader::kMax];
    std::strncpy(line, line_in, sizeof(line) - 1);
    line[sizeof(line) - 1] = 0;
    char* tok[8];
    const int n = tokenize(line, tok, 8);
    if (n == 0) return;
    const char* c = tok[0];

    if (tokEq(c, "PING")) {
        ack(src, "PING", "pong");
    } else if (tokEq(c, "STATUS")) {
        if (n > 1 && tokEq(tok[1], "CLEAR")) {
            errors_ &= ERR_ML_INIT;
            rtCmd(RtCmdType::StatsReset);
            timing_.reset();
            ack(src, "STATUS CLEAR");
        } else {
            cmdStatus(src);
        }
    } else if (tokEq(c, "METRICS")) {
        float v;
        if (n == 1 || tokEq(tok[1], "ONCE")) {
            if (!have_frame_) err(src, "METRICS", "no analysis frame yet");
            else {
                metrics_once_ = true;
                metrics_once_src_ = src;
                periodicMetrics(plat::timeUs());
            }
        } else if (tokEq(tok[1], "OFF")) {
            cfg_.comm.metrics_period_ms = 0;
            ack(src, "METRICS", "off");
        } else if (tokEq(tok[1], "ON")) {
            if (!cfg_.comm.metrics_period_ms) cfg_.comm.metrics_period_ms = PQ_DEFAULT_METRICS_PERIOD_MS;
            ack(src, "METRICS", "on");
        } else if (parseFloat(tok[1], v) && v >= 100.0f && v <= 60000.0f) {
            cfg_.comm.metrics_period_ms = static_cast<uint32_t>(v);
            ack(src, "METRICS", "period set");
        } else {
            err(src, "METRICS", "usage: METRICS [ONCE|ON|OFF|<period_ms 100..60000>]");
        }
    } else if (tokEq(c, "FORMAT")) {
        if (n > 1 && tokEq(tok[1], "CSV")) {
            cfg_.comm.metrics_format = 1;
            csv_header_pending_ = true;
            ack(src, "FORMAT", "csv");
        } else if (n > 1 && tokEq(tok[1], "JSON")) {
            cfg_.comm.metrics_format = 0;
            ack(src, "FORMAT", "json");
        } else {
            err(src, "FORMAT", "usage: FORMAT JSON|CSV");
        }
    } else if (tokEq(c, "EVENT") || tokEq(c, "EVENTS")) {
        cmdEvents(src, tok, n);
    } else if (tokEq(c, "HARMONICS")) {
        if (!have_frame_) err(src, "HARMONICS", "no analysis frame yet");
        else reply(src, out_, fmtHarmonicsJson(out_, sizeof(out_), last_));
    } else if (tokEq(c, "FEATURES")) {
        if (!have_frame_) err(src, "FEATURES", "no analysis frame yet");
        else reply(src, out_, fmtFeaturesJson(out_, sizeof(out_), last_));
    } else if (tokEq(c, "CONFIG")) {
        cmdConfig(src, tok, n);
    } else if (tokEq(c, "CALIBRATE") || tokEq(c, "CAL")) {
        cmdCalibrate(src, tok, n);
    } else if (tokEq(c, "START")) {
        rtCmd(RtCmdType::Start);
        ack(src, "START");
    } else if (tokEq(c, "STOP")) {
        rtCmd(RtCmdType::Stop);
        ack(src, "STOP");
    } else if (tokEq(c, "SOURCE")) {
        if (n > 1 && (tokEq(tok[1], "SIM") || tokEq(tok[1], "ADC"))) {
            cfg_.acq.source = tokEq(tok[1], "SIM") ? 1 : 0;
            pushConfig();
            ack(src, "SOURCE", tokEq(tok[1], "SIM") ? "simulator" : "hardware adc");
        } else {
            err(src, "SOURCE", "usage: SOURCE SIM|ADC");
        }
    } else if (tokEq(c, "SIM")) {
        cmdSim(src, tok, n);
    } else if (tokEq(c, "WAVE")) {
        float v = PQ_WAVE_SNAPSHOT;
        if (n > 1 && (!parseFloat(tok[1], v) || v < 16 || v > PQ_WAVE_SNAPSHOT)) {
            err(src, "WAVE", "usage: WAVE [16..512]");
            return;
        }
        if (wave_pending_) {
            err(src, "WAVE", "capture already in progress");
            return;
        }
        wave_req_n_ = static_cast<uint32_t>(v);
        wave_off_ = 0;
        wave_src_ = src;
        wave_pending_ = true;
        wave_req_us_ = plat::timeUs();
        env_.link->wave_ready.store(0, std::memory_order_release);
        rtCmd(RtCmdType::WaveCapture);
    } else if (tokEq(c, "BENCH")) {
        cmdBench(src);
    } else if (tokEq(c, "MODEL")) {
        cmdModel(src);
    } else if (tokEq(c, "CONTROL")) {
        if (n > 1 && (tokEq(tok[1], "ARM") || tokEq(tok[1], "DISARM"))) {
            const bool on = tokEq(tok[1], "ARM");
            cfg_.ctrl.armed = on;
            rtCmd(RtCmdType::CtrlArm, on);
            ack(src, "CONTROL", on ? "armed" : "disarmed");
        } else if (n > 1 && tokEq(tok[1], "RESET")) {
            rtCmd(RtCmdType::CtrlReset);
            ack(src, "CONTROL", "trip reset");
        } else if (n > 2 && tokEq(tok[1], "TEST")) {
            PqClass pc;
            if (!pqClassFromString(tok[2], pc)) err(src, "CONTROL", "unknown class");
            else {
                rtCmd(RtCmdType::CtrlTest, static_cast<uint8_t>(pc));
                ack(src, "CONTROL", "event code output set");
            }
        } else {
            err(src, "CONTROL", "usage: CONTROL ARM|DISARM|RESET|TEST <class>");
        }
    } else if (tokEq(c, "REBOOT")) {
        ack(src, "REBOOT");
        if (env_.usb) env_.usb->flush();
        plat::requestReboot();
    } else if (tokEq(c, "HELP") || tokEq(c, "?")) {
        cmdHelp(src);
    } else {
        err(src, c, "unknown command (HELP lists commands)");
    }
}

void Supervisor::cmdStatus(int src) {
    RtStatus st{};
    env_.link->readStatus(st);
    const ClassifierInfo& ci = Classifier_Info();
    const plat::MemoryInfo mi = plat::memoryInfo();
    JsonWriter w(out_, sizeof(out_));
    w.beginObj()
        .kv("type", "status")
        .kv("fw", kFirmwareVersion)
        .kv("platform", plat::platformName())
        .kv("cpu_mhz", plat::cpuHz() / 1000000u)
        .kv("uptime_s", static_cast<double>(st.uptime_us) * 1e-6, 1)
        .kvBool("running", st.running)
        .kv("source", st.backend ? st.backend : "?")
        .kv("sample_rate", st.fs, 1)
        .kv("analysis_rate", st.analysis_fs, 1)
        .kv("fft_size", static_cast<int32_t>(st.fft_size))
        .kv("hop", static_cast<int32_t>(st.hop))
        .kv("block_size", static_cast<uint32_t>(kBlockSize))
        .kv("cpu_load_pct", st.cpu_load_pct, 1)
        .kv("cpu_total_pct", st.cpu_total_pct, 1)
        .kv("cpu_peak_pct", st.cpu_peak_pct, 1)
        .kv("vrms_half", st.vrms_half, 2)
        .kv("voltage_state", pqClassShortName(st.voltage_state));
    writeErrorList(w, "errors", st.errors | errors_);
    writeErrorList(w, "errors_active", st.errors_now | (errors_ & ERR_ML_INIT));
    w.beginObj("adc")
        .kv("status", adcStatusName(st.adc_status))
        .kv("blocks", st.adc.blocks)
        .kv("overruns", st.adc.overruns)
        .kv("missing_samples", st.adc.missing_samples)
        .kv("crc_errors", st.adc.crc_errors)
        .kv("status_errors", st.adc.status_errors)
        .kv("saturated", st.adc.saturated)
        .kv("timeouts", st.adc.timeouts)
        .kv("reinit", st.reinit_count)
        .kv("ring_high_water", st.adc.ring_high_water)
        .kv("ring_blocks", st.ring_blocks)
        .endObj();
    w.beginObj("pipeline")
        .kv("frames", st.eng.frames)
        .kv("frames_invalid", st.eng.frames_invalid)
        .kv("gaps", st.eng.gaps)
        .kv("events", st.eng.events)
        .kv("max_block_us", st.eng.max_block_us)
        .kv("block_budget_us", st.block_budget_us)
        .kv("max_frame_us", st.eng.max_frame_us)
        .kv("frames_dropped", st.frames_dropped)
        .kv("events_dropped", st.events_dropped)
        .kv("tx_dropped", tx_dropped_)
        .kv("log_dropped", logDropped())
        .endObj();
    w.beginObj("ml").kvBool("ready", ci.ready).kvBool("enabled", cfg_.ml.ml_enable != 0)
        .kv("error", ci.error ? ci.error : "").endObj();
    w.beginObj("memory")
        .kv("ram_total", mi.ram_total)
        .kv("ram_static", mi.ram_static)
        .kv("heap_used", mi.heap_used)
        .kv("flash_image", mi.flash_image)
        .kv("stack0_free", mi.stack_core0_free)
        .kv("stack1_free", mi.stack_core1_free)
        .endObj();
    w.beginObj("control")
        .kvBool("armed", st.ctrl.armed)
        .kvBool("tripped", st.ctrl.tripped)
        .kv("trip_reason", pqClassShortName(st.ctrl.trip_reason))
        .kv("trip_count", st.ctrl.trip_count)
        .kv("dvr_ref_pu", st.ctrl.ref[0], 3)
        .kv("apf_ref", st.ctrl.ref[1], 3)
        .kv("event_code", static_cast<uint32_t>(st.ctrl.event_code))
        .endObj();
    w.kv("log_entries", static_cast<uint32_t>(log_.count())).kv("log_total", log_.total());
    w.endObj();
    reply(src, out_, w.finish());
}

void Supervisor::sendConfigParam(int src, const ParamDesc& p) {
    float v = 0;
    configGet(cfg_, p, v);
    JsonWriter w(out_, sizeof(out_));
    w.beginObj().kv("type", "config").kv("key", p.key);
    if (p.type == ParamType::F32) w.kv("value", static_cast<double>(v), 6);
    else w.kv("value", static_cast<uint32_t>(v));
    w.kv("min", p.min, 4).kv("max", p.max, 4).kvBool("restart", (p.flags & PF_RESTART_ACQ) != 0)
        .kv("help", p.help).endObj();
    reply(src, out_, w.finish());
}

void Supervisor::cmdConfig(int src, char** tok, int n) {
    if (n < 2 || tokEq(tok[1], "LIST")) {
        size_t np;
        const ParamDesc* ps = configParams(np);
        for (size_t k = 0; k < np; ++k) sendConfigParam(src, ps[k]);
        ack(src, "CONFIG LIST");
        return;
    }
    if (tokEq(tok[1], "GET") && n >= 3) {
        const ParamDesc* p = configFindParam(tok[2]);
        if (!p) err(src, "CONFIG GET", "unknown key");
        else sendConfigParam(src, *p);
        return;
    }
    if (tokEq(tok[1], "SET") && n >= 4) {
        const ParamDesc* p = configFindParam(tok[2]);
        float v;
        if (!p) {
            err(src, "CONFIG SET", "unknown key");
            return;
        }
        if (!parseFloat(tok[3], v)) {
            err(src, "CONFIG SET", "value is not a number");
            return;
        }
        if (const char* e = configSet(cfg_, *p, v)) {
            err(src, "CONFIG SET", e);
            return;
        }
        pushConfig();
        if (std::strcmp(p->key, "comm.metrics_format") == 0 && cfg_.comm.metrics_format == 1) csv_header_pending_ = true;
        sendConfigParam(src, *p);
        return;
    }
    if (tokEq(tok[1], "SAVE")) {
        if (!env_.store) {
            err(src, "CONFIG SAVE", "no persistent storage");
            return;
        }
        StoredConfig sc;
        configPack(cfg_, sc);
        if (env_.store->save(sc)) ack(src, "CONFIG SAVE", "saved");
        else {
            errors_ |= ERR_FLASH;
            err(src, "CONFIG SAVE", "flash write/verify failed");
        }
        return;
    }
    if (tokEq(tok[1], "LOAD")) {
        StoredConfig sc;
        SystemConfig c;
        if (env_.store && env_.store->load(sc) && configUnpack(sc, c)) {
            cfg_ = c;
            pushConfig();
            ack(src, "CONFIG LOAD", "loaded");
        } else {
            err(src, "CONFIG LOAD", "no valid stored configuration");
        }
        return;
    }
    if (tokEq(tok[1], "DEFAULTS")) {
        cfg_ = configDefaults();
        pushConfig();
        ack(src, "CONFIG DEFAULTS", "defaults applied (CONFIG SAVE to persist)");
        return;
    }
    if (tokEq(tok[1], "ERASE")) {
        if (env_.store && env_.store->erase()) ack(src, "CONFIG ERASE");
        else err(src, "CONFIG ERASE", "failed");
        return;
    }
    err(src, "CONFIG", "usage: CONFIG LIST|GET k|SET k v|SAVE|LOAD|DEFAULTS|ERASE");
}

void Supervisor::cmdCalibrate(int src, char** tok, int n) {
    if (n < 2 || tokEq(tok[1], "SHOW")) {
        JsonWriter w(out_, sizeof(out_));
        w.beginObj().kv("type", "cal").kv("step", "SHOW")
            .kv("v_gain", static_cast<double>(cfg_.cal.v_gain), 9).kv("v_offset", cfg_.cal.v_offset, 2)
            .kv("i_gain", static_cast<double>(cfg_.cal.i_gain), 10).kv("i_offset", cfg_.cal.i_offset, 2)
            .kv("phase_deg", cfg_.cal.phase_deg, 3).endObj();
        reply(src, out_, w.finish());
        return;
    }
    if (cal_mode_ != CalMode::None) {
        err(src, "CALIBRATE", "calibration already running");
        return;
    }
    const uint64_t now = plat::timeUs();
    cal_src_ = src;
    cal_acc_ = 0.0;
    cal_n_ = 0;
    float v = 0.0f;
    if (tokEq(tok[1], "OFFSET")) {
        float ms = 1000.0f;
        if (n > 2 && (!parseFloat(tok[2], ms) || ms < 100.0f || ms > 10000.0f)) {
            err(src, "CALIBRATE", "usage: CALIBRATE OFFSET [ms 100..10000] (inputs at zero!)");
            return;
        }
        RtStatus cur{};
        env_.link->readStatus(cur);
        cal_reset_seq_ = cur.raw_resets + 1;   // wait until the RT core has reset
        rtCmd(RtCmdType::CalRawReset);
        cal_mode_ = CalMode::Offset;
        cal_need_ = static_cast<uint32_t>(ms * 1e-3f * cfg_.acq.sample_rate_hz);   // samples to average
        cal_deadline_us_ = now + static_cast<uint64_t>(ms * 1000.0f) + 3000000;
        ack(src, "CALIBRATE OFFSET", "measuring - keep inputs at zero");
        return;
    }
    const bool vg = tokEq(tok[1], "VGAIN"), ig = tokEq(tok[1], "IGAIN"), ph = tokEq(tok[1], "PHASE");
    if (vg || ig) {
        if (n < 3 || !parseFloat(tok[2], v) || v <= 0.0f) {
            err(src, "CALIBRATE", "usage: CALIBRATE VGAIN <ref Vrms> | IGAIN <ref Arms>");
            return;
        }
    } else if (ph) {
        if (n > 2 && !parseFloat(tok[2], v)) {
            err(src, "CALIBRATE", "usage: CALIBRATE PHASE [true load angle deg, default 0]");
            return;
        }
    } else {
        err(src, "CALIBRATE", "usage: CALIBRATE SHOW|OFFSET|VGAIN|IGAIN|PHASE");
        return;
    }
    cal_mode_ = vg ? CalMode::VGain : (ig ? CalMode::IGain : CalMode::Phase);
    cal_ref_ = v;
    cal_need_ = 10;
    cal_deadline_us_ = now + 5000000;
    ack(src, "CALIBRATE", "averaging 10 frames");
}

void Supervisor::cmdSim(int src, char** tok, int n) {
    RtStatus st{};
    env_.link->readStatus(st);
    if (n < 2 || tokEq(tok[1], "LIST")) {
        sendStr(src == 1 ? kSrcUart : kSrcUsb,
                "# SIM scenarios: sag swell interruption harmonic transient oscillatory frequency load\n"
                "# SIM <scenario> [magnitude] [duration_s] | SIM DEMO | SIM CLEAR\n"
                "# SIM BASE noise|freq|load|level|harm|phase|pf <value>\n"
                "# SIM FAULT CRC <prob> | SIM FAULT STALL ON|OFF\n");
        ack(src, "SIM LIST");
        return;
    }
    if (!st.simulated && !tokEq(tok[1], "CLEAR")) {
        err(src, "SIM", "hardware ADC active (use SOURCE SIM first)");
        return;
    }
    float a = 0, b = 0;
    if (tokEq(tok[1], "CLEAR")) {
        rtCmd(RtCmdType::SimClear);
        rtCmd(RtCmdType::SimFaultCrc, 0, 0.0f);
        rtCmd(RtCmdType::SimFaultStall, 0);
        ack(src, "SIM CLEAR");
    } else if (tokEq(tok[1], "DEMO")) {
        const char* seq[] = {"sag", "swell", "interruption", "harmonic", "transient", "frequency", "load"};
        float delay = 0.5f;
        for (const char* s : seq) {
            rtCmd(RtCmdType::SimScenario, 0, 0.0f, 0.0f, delay, s);
            delay += 3.0f;
        }
        ack(src, "SIM DEMO", "7 disturbances scheduled over ~21 s");
    } else if (tokEq(tok[1], "BASE") && n >= 4 && parseFloat(tok[3], a)) {
        rtCmd(RtCmdType::SimBase, 0, a, 0, 0, tok[2]);
        ack(src, "SIM BASE");
    } else if (tokEq(tok[1], "FAULT") && n >= 4) {
        if (tokEq(tok[2], "CRC") && parseFloat(tok[3], a) && a >= 0.0f && a <= 1.0f) {
            rtCmd(RtCmdType::SimFaultCrc, 0, a);
            ack(src, "SIM FAULT CRC");
        } else if (tokEq(tok[2], "STALL")) {
            rtCmd(RtCmdType::SimFaultStall, tokEq(tok[3], "ON") ? 1 : 0);
            ack(src, "SIM FAULT STALL");
        } else {
            err(src, "SIM", "usage: SIM FAULT CRC <0..1> | SIM FAULT STALL ON|OFF");
        }
    } else {
        Rng r(1);
        Disturbance d;
        if (n > 2 && !parseFloat(tok[2], a)) a = 0;
        if (n > 3 && !parseFloat(tok[3], b)) b = 0;
        if (!makeScenario(tok[1], 0.0, a, b, r, d)) {
            err(src, "SIM", "unknown scenario (SIM LIST)");
            return;
        }
        rtCmd(RtCmdType::SimScenario, 0, a, b, 0.02f, tok[1]);
        ack(src, "SIM", tok[1]);
    }
}

void Supervisor::cmdEvents(int src, char** tok, int n) {
    if (n > 1 && tokEq(tok[1], "CLEAR")) {
        log_.clear();
        ack(src, "EVENTS CLEAR");
        return;
    }
    const bool text = n > 1 && tokEq(tok[1], "TEXT");
    float lim = 20;
    const int ai = text ? 2 : 1;
    if (n > ai) parseFloat(tok[ai], lim);
    const size_t cnt = log_.count();
    const size_t want = lim < 1 ? 1 : static_cast<size_t>(lim);
    const size_t from = cnt > want ? cnt - want : 0;
    for (size_t k = from; k < cnt; ++k) {
        const EventRecord* r = log_.at(k);
        if (text) {
            char tb[700];
            const size_t tl = EventLogger::formatText(*r, tb, sizeof(tb));
            // prefix every line with "# " so machine parsers can skip it
            size_t o = 0;
            bool bol = true;
            for (size_t j = 0; j < tl && o + 4 < sizeof(out_); ++j) {
                if (bol) {
                    out_[o++] = '#';
                    out_[o++] = ' ';
                    bol = false;
                }
                out_[o++] = tb[j];
                if (tb[j] == '\n') bol = true;
            }
            reply(src, out_, o);
        } else {
            reply(src, out_, fmtEventJson(out_, sizeof(out_), *r, "event_log"));
        }
    }
    JsonWriter w(out_, sizeof(out_));
    w.beginObj().kv("type", "ack").kv("cmd", "EVENTS").kv("count", static_cast<uint32_t>(cnt - from))
        .kv("total", log_.total()).kv("overwritten", log_.overwritten()).endObj();
    reply(src, out_, w.finish());
}

void Supervisor::cmdBench(int src) {
    RtStatus st{};
    env_.link->readStatus(st);
    // 1) live pipeline timing measured on the real-time core
    {
        JsonWriter w(out_, sizeof(out_));
        const float hop_us = st.analysis_fs > 0 ? 1e6f * st.hop / st.analysis_fs : 0.0f;
        w.beginObj().kv("type", "bench").kv("part", "live").kv("platform", plat::platformName())
            .kv("cpu_mhz", plat::cpuHz() / 1000000u)
            .kv("sample_rate", st.fs, 1).kv("analysis_rate", st.analysis_fs, 1)
            .kv("fft_size", static_cast<int32_t>(st.fft_size)).kv("hop", static_cast<int32_t>(st.hop))
            .kv("hop_period_us", hop_us, 0).kv("frames", timing_.frame.n);
        auto stage = [&](const char* k, const StageStat& s) {
            w.beginObj(k).kv("avg_us", s.avg(), 1).kv("max_us", s.max).endObj();
        };
        stage("per_sample_path_per_hop", timing_.per_sample);
        stage("fft_harmonics_thd", timing_.spectral);
        stage("window_stats", timing_.window);
        stage("disturbance_detection", timing_.detect);
        stage("feature_extraction", timing_.feature);
        stage("ml_total", timing_.ml_total);
        stage("ml_invoke", timing_.ml_invoke);
        stage("fusion", timing_.fusion);
        stage("frame_total", timing_.frame);
        stage("total_per_hop", timing_.e2e);
        w.kv("cpu_load_pct", st.cpu_load_pct, 1).kv("cpu_total_pct", st.cpu_total_pct, 1)
            .kv("cpu_peak_pct", st.cpu_peak_pct, 1)
            .kv("max_block_us", st.eng.max_block_us).kv("block_budget_us", st.block_budget_us)
            .kv("ring_high_water", st.adc.ring_high_water).kv("ring_blocks", st.ring_blocks)
            .kv("buffer_util_pct", 100.0f * st.adc.ring_high_water / (st.ring_blocks ? st.ring_blocks : 1), 1);
        const plat::MemoryInfo mi = plat::memoryInfo();
        const ClassifierInfo& ci = Classifier_Info();
        w.kv("ram_static", mi.ram_static).kv("ram_total", mi.ram_total).kv("flash_image", mi.flash_image)
            .kv("model_bytes", ci.model_bytes).kv("arena_bytes", ci.arena_bytes).kv("arena_used", ci.arena_used);
        w.endObj();
        reply(src, out_, w.finish());
    }
    // 2) kernel micro-benchmarks on this core
    BenchReport br;
    runBenchmarks(st.fs > 0 ? st.fs : cfg_.acq.sample_rate_hz, cfg_.acq.decimation, br);
    {
        JsonWriter w(out_, sizeof(out_));
        w.beginObj().kv("type", "bench").kv("part", "fft");
        w.beginArr("sizes");
        for (const FftBench& f : br.fft) {
            w.beginObj().kv("n", static_cast<int32_t>(f.n)).kv("resolution_hz", f.resolution_hz, 2)
                .kv("window_ms", f.window_ms, 1).kv("fft_us", f.fft_us).kv("analysis_us", f.analysis_us)
                .kv("budget_pct_2ch_50pct_overlap", f.budget_pct, 2).endObj();
        }
        w.endArr().endObj();
        reply(src, out_, w.finish());
    }
    {
        const KernelBench& k = br.k;
        JsonWriter w(out_, sizeof(out_));
        w.beginObj().kv("type", "bench").kv("part", "kernels_ns_per_sample")
            .kv("calibration_2ch", k.calibration_ns, 1).kv("dc_block", k.dc_block_ns, 1)
            .kv("frequency", k.frequency_ns, 1).kv("rms", k.rms_ns, 1)
            .kv("wavelet_haar_3lvl", k.haar_ns, 1).kv("wavelet_db4_3lvl", k.db4_ns, 1)
            .kv("transient_incl_db4", k.transient_ns, 1).kv("halfband_decimator", k.decimator_ns, 1)
            .kv("streaming_total", k.per_sample_total_ns, 1)
            .kv("streaming_budget_pct", k.per_sample_budget_pct, 2).endObj();
        reply(src, out_, w.finish());
    }
}

void Supervisor::cmdModel(int src) {
    const ClassifierInfo& ci = Classifier_Info();
    JsonWriter w(out_, sizeof(out_));
    w.beginObj().kv("type", "model").kvBool("ready", ci.ready).kv("error", ci.error ? ci.error : "")
        .kv("version", ci.model_version ? ci.model_version : "").kv("model_bytes", ci.model_bytes)
        .kv("arena_bytes", ci.arena_bytes).kv("arena_used", ci.arena_used)
        .kvHex("spec_hash_model", ci.spec_hash_model).kvHex("spec_hash_fw", ci.spec_hash_firmware)
        .kv("test_accuracy", ci.reported_accuracy, 4).kv("runs", ci.runs)
        .kv("max_inference_us", ci.max_inference_us)
        .kv("in_scale", static_cast<double>(ci.in_scale), 6).kv("in_zp", static_cast<int32_t>(ci.in_zero_point));
    w.beginArr("features");
    for (int k = 0; k < kNumFeatures; ++k) w.val(featureName(k));
    w.endArr().beginArr("classes");
    for (int c = 0; c < kNumMlClasses; ++c) w.val(pqClassShortName(static_cast<PqClass>(c)));
    w.endArr().endObj();
    reply(src, out_, w.finish());
}

void Supervisor::cmdHelp(int src) {
    sendStr(src == 1 ? kSrcUart : kSrcUsb,
            "# Commands (one per line, case-insensitive):\n"
            "#  PING | STATUS [CLEAR] | METRICS [ONCE|ON|OFF|<ms>] | FORMAT JSON|CSV\n"
            "#  EVENTS [n] | EVENTS TEXT [n] | EVENTS CLEAR | HARMONICS | FEATURES | MODEL\n"
            "#  CONFIG LIST|GET k|SET k v|SAVE|LOAD|DEFAULTS|ERASE\n"
            "#  CALIBRATE SHOW|OFFSET [ms]|VGAIN <Vrms>|IGAIN <Arms>|PHASE [deg]\n"
            "#  START | STOP | SOURCE SIM|ADC | SIM ... (SIM LIST) | WAVE [n]\n"
            "#  BENCH | CONTROL ARM|DISARM|RESET|TEST <class> | REBOOT | HELP\n");
    ack(src, "HELP");
}

}  // namespace pq
