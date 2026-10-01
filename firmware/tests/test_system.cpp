// Protocol, configuration, control outputs, event logger, and the full
// application (RealtimeTask + Supervisor) driven through the serial protocol.
#include <cstring>
#include <string>

#include "app/realtime_task.h"
#include "app/supervisor.h"
#include "common/math_util.h"
#include "communication/json_writer.h"
#include "communication/protocol.h"
#include "outputs/control_output.h"
#include "platform/host/host_io.h"
#include "storage/event_logger.h"
#include "test_framework.h"
#include "test_helpers.h"

using namespace pq;
using namespace pqtest;

// ------------------------------------------------------------- protocol
PQ_TEST(protocol, tokenizer_and_parsers) {
    char line[] = "  CONFIG   set pq.sag_threshold\t0.85 ";
    char* t[8];
    PQ_CHECK(tokenize(line, t, 8) == 4);
    PQ_CHECK(tokEq(t[0], "config") && tokEq(t[1], "SET"));
    float v;
    PQ_CHECK(parseFloat(t[3], v) && v == 0.85f);
    PQ_CHECK(!parseFloat("0.8x", v));
    LineReader lr;
    char out[LineReader::kMax];
    const char* in = "PING\r\nSTATUS\n";
    int lines = 0;
    for (const char* p = in; *p; ++p) lines += lr.feed(*p, out);
    PQ_CHECK(lines == 2);
    std::string big(400, 'A');
    big += "\nPING\n";
    lines = 0;
    for (char c : big) lines += lr.feed(c, out);
    PQ_CHECK(lines == 1 && std::strcmp(out, "PING") == 0);   // over-long line discarded
}

PQ_TEST(protocol, json_writer_escaping_and_overflow) {
    char b[64];
    JsonWriter w(b, sizeof(b));
    w.beginObj().kv("a", "q\"x").kv("n", 1.5f, 2).kvBool("t", true).endObj();
    PQ_CHECK(w.finish() > 0);
    PQ_CHECK(std::strcmp(b, "{\"a\":\"q\\\"x\",\"n\":1.50,\"t\":true}\n") == 0);
    char s[16];
    JsonWriter w2(s, sizeof(s));
    w2.beginObj().kv("long_key", "long value that does not fit").endObj();
    PQ_CHECK(w2.finish() == 0 && w2.overflow());
}

PQ_TEST(protocol, metrics_and_event_messages) {
    FrameResult f{};
    f.m.vrms = 176.3f;
    f.m.irms = 2.14f;
    f.m.freq = 49.97f;
    f.m.thd_v = 3.2f;
    f.m.valid = 1;
    char b[1400];
    const size_t n = fmtMetricsJson(b, sizeof(b), f, PqClass::Sag, 0.971f, 124582, 0);
    PQ_CHECK(n > 0);
    PQ_CHECK(std::strstr(b, "\"timestamp_ms\":124582") && std::strstr(b, "\"vrms\":176.30") &&
             std::strstr(b, "\"event\":\"SAG\"") && std::strstr(b, "\"confidence\":0.971"));
    PQ_CHECK(fmtMetricsCsv(b, sizeof(b), f, PqClass::Sag, 0.971f, 124582, 0) > 0);
    PQ_CHECK(std::strncmp(b, "M,124582,", 9) == 0);
    EventRecord e{};
    e.id = 7;
    e.state = EV_END;
    e.final_type = e.dsp_type = PqClass::Sag;
    e.ml_type = PqClass::Sag;
    e.ml_conf = 0.9f;
    e.flags = EVF_WITH_TRANSIENT;
    e.components = classBit(PqClass::Sag);
    PQ_CHECK(fmtEventJson(b, sizeof(b), e) > 0);
    PQ_CHECK(std::strstr(b, "\"event\":\"VOLTAGE_SAG\"") && std::strstr(b, "WITH_TRANSIENT") &&
             std::strstr(b, "\"state\":\"end\""));
}

// --------------------------------------------------------------- config
PQ_TEST(config, defaults_validate_and_parameter_table) {
    SystemConfig c = configDefaults();
    PQ_CHECK(configValidate(c) == nullptr);
    const ParamDesc* p = configFindParam("pq.sagThreshold");
    PQ_CHECK(p != nullptr);
    PQ_CHECK(configSet(c, *p, 0.85f) == nullptr);
    PQ_CHECK_NEAR(c.pq.sagThreshold, 0.85, 1e-6);
    PQ_CHECK(configSet(c, *p, 1.5f) != nullptr);   // out of range, unchanged
    PQ_CHECK_NEAR(c.pq.sagThreshold, 0.85, 1e-6);
    PQ_CHECK(configSet(c, *configFindParam("acq.fft_size"), 1000) != nullptr);   // not pow2
    PQ_CHECK(configSet(c, *configFindParam("acq.fft_size"), 2048) == nullptr);
    PQ_CHECK(configSet(c, *configFindParam("pq.interruptionThreshold"), 0.5f) == nullptr);
    PQ_CHECK(configSet(c, *configFindParam("pq.sagThreshold"), 0.5f) != nullptr);   // int >= sag
    size_t n;
    configParams(n);
    PQ_CHECK(n > 50);
}

PQ_TEST(config, pack_unpack_detects_corruption) {
    SystemConfig c = configDefaults();
    c.pq.nominalVoltage = 120.0f;
    c.pq.nominalFrequency = 60.0f;
    StoredConfig s;
    configPack(c, s);
    SystemConfig o;
    PQ_CHECK(configUnpack(s, o));
    PQ_CHECK(o.pq.nominalVoltage == 120.0f && o.pq.nominalFrequency == 60.0f);
    reinterpret_cast<uint8_t*>(&s.cfg)[5] ^= 0x40;
    PQ_CHECK(!configUnpack(s, o));
    configPack(c, s);
    s.version = kConfigVersion + 1;
    PQ_CHECK(!configUnpack(s, o));
}

PQ_TEST(config, sixty_hz_system_works) {
    SystemConfig c = configDefaults();
    c.ml.ml_enable = 0;
    c.pq.nominalFrequency = 60.0f;
    c.pq.nominalVoltage = 120.0f;
    GeneratorParams g = cleanParams();
    g.fnom = 60.0f;
    g.vnom_rms = 120.0f;
    Rng r(1);
    Disturbance d;
    makeScenario("sag", 1.0, 0.7f, 0.1f, r, d);
    auto run = runPipeline(c, g, {d}, 2.0);
    PQ_CHECK_NEAR(run.frames.back().m.freq, 60.0, 0.005);
    PQ_CHECK_NEAR(run.frames.back().m.vrms, 120.0, 0.3);
    PQ_CHECK(countEnded(run, PqClass::Sag) == 1);
}

// --------------------------------------------------------------- control
PQ_TEST(control, disarmed_outputs_are_inert) {
    RecorderControlHw hw;
    ControlOutput co;
    ControlConfig cc = configDefaults().ctrl;
    co.init(&hw, cc);
    PQ_CHECK(!co.trip(PqClass::Swell, 1.4f, 1));
    PQ_CHECK(!hw.rec.trip);
    co.setReference(RefChannel::DVR, 0.3f);
    PQ_CHECK(hw.rec.ref[0] == 0.0f);
    co.setEvent(PqClass::Sag, true);   // informational code always driven
    PQ_CHECK(hw.rec.event_code == static_cast<uint8_t>(PqClass::Sag));
}

PQ_TEST(control, trip_latches_until_reset_and_refs) {
    RecorderControlHw hw;
    ControlOutput co;
    ControlConfig cc = configDefaults().ctrl;
    cc.armed = 1;
    co.init(&hw, cc);
    PQ_CHECK(co.trip(PqClass::Swell, 1.4f, 10));
    PQ_CHECK(hw.rec.trip && hw.rec.trip_edges == 1);
    PQ_CHECK(!co.trip(PqClass::Swell, 1.5f, 11));   // latched: no second edge
    co.resetTrip();
    PQ_CHECK(!hw.rec.trip);
    co.updateDvr(0.7f);
    PQ_CHECK_NEAR(hw.rec.ref[0], 0.3, 1e-6);
    co.updateDvr(1.2f);
    PQ_CHECK_NEAR(hw.rec.ref[0], -0.2, 1e-6);
    co.updateDvr(1.01f);
    PQ_CHECK(hw.rec.ref[0] == 0.0f);   // dead band
    co.updateDvr(0.02f);
    PQ_CHECK(hw.rec.ref[0] == 0.0f);   // DVR cannot restore an interruption
}

PQ_TEST(control, apf_packet_format) {
    float ih[PQ_REPORT_HARMONICS + 1], ip[PQ_REPORT_HARMONICS + 1];
    for (int h = 0; h <= PQ_REPORT_HARMONICS; ++h) {
        ih[h] = 0.1f * h;
        ip[h] = -0.01f * h;
    }
    uint8_t pkt[256];
    const size_t n = ControlOutput::buildHarmonicPacket(pkt, sizeof(pkt), ih, ip, 12.5f, 42, PQ_REPORT_HARMONICS);
    PQ_CHECK(n == 3 + 10 + PQ_REPORT_HARMONICS * 8 + 2);
    PQ_CHECK(pkt[0] == 0xA5 && pkt[1] == 0x5A && pkt[2] == n - 5 && pkt[3] == 0x01);
    uint32_t frame;
    std::memcpy(&frame, pkt + 4, 4);
    PQ_CHECK(frame == 42);
    const uint16_t crc = crc16Ccitt(pkt + 3, n - 5);
    PQ_CHECK(pkt[n - 2] == (crc & 0xFF) && pkt[n - 1] == (crc >> 8));
}

PQ_TEST(control, pipeline_trips_on_severe_swell_when_armed) {
    SystemConfig c = configDefaults();
    c.ml.ml_enable = 0;
    c.ctrl.armed = 1;
    Rng r(1);
    Disturbance d;
    makeScenario("swell", 1.0, 1.4f, 0.2f, r, d);
    auto run = runPipeline(c, cleanParams(), {d}, 1.5);
    PQ_CHECK(run.trip && run.trip_class == PqClass::Swell);
    makeScenario("swell", 1.0, 1.2f, 0.2f, r, d);   // below the 1.3 pu trip level
    auto run2 = runPipeline(c, cleanParams(), {d}, 1.5);
    PQ_CHECK(!run2.trip);
}

// ----------------------------------------------------------------- logger
PQ_TEST(logger, merges_updates_and_wraps) {
    EventLogger lg;
    lg.clear();
    EventRecord e{};
    e.id = 1;
    e.state = EV_START;
    e.dsp_type = e.final_type = PqClass::Sag;
    lg.record(e);
    e.state = EV_CLASSIFIED;
    e.ml_type = PqClass::Sag;
    e.ml_conf = 0.97f;
    e.cls_latency_ms = 110;
    lg.record(e);
    EventRecord end = e;
    end.state = EV_END;
    end.ml_conf = 0;
    end.cls_latency_ms = 0;
    end.duration_ms = 200;
    lg.record(end);
    PQ_CHECK(lg.count() == 1);
    const EventRecord* r = lg.findById(1);
    PQ_CHECK(r && r->state == EV_END && r->duration_ms == 200 && r->ml_conf == 0.97f);
    char txt[700];
    PQ_CHECK(EventLogger::formatText(*r, txt, sizeof(txt)) > 100);
    PQ_CHECK(std::strstr(txt, "Event: VOLTAGE_SAG") && std::strstr(txt, "Confidence: 97.0 %"));
    for (uint32_t k = 2; k < 2 + PQ_EVENT_LOG_SIZE + 5; ++k) {
        e.id = k;
        lg.record(e);
    }
    PQ_CHECK(lg.count() == PQ_EVENT_LOG_SIZE);
    PQ_CHECK(lg.overwritten() == 6);
    PQ_CHECK(lg.findById(1) == nullptr);
}

// ------------------------------------------- full application over protocol
namespace {
struct App {
    RtShared link;
    FreeRunPacer pacer;
    SimAdcBackend sim{&pacer};
    RecorderControlHw ctrl;
    NullAlarmHw alarm;
    MemTransport io;
    RealtimeTask rt;
    Supervisor sup;
    App() {
        SystemConfig c = configDefaults();
        rt.init(&link, nullptr, &sim, &ctrl, c);
        SupervisorEnv env;
        env.link = &link;
        env.usb = &io;
        env.alarm_hw = &alarm;
        sup.init(env, c);
    }
    // Advance simulated time; the free-run pacer yields one block per poll.
    void run(double seconds) {
        const int blocks = static_cast<int>(seconds * 10000 / kBlockSize);
        for (int k = 0; k < blocks; ++k) {
            rt.poll();
            sup.poll();
        }
    }
    std::vector<std::string> cmd(const char* c, double settle = 0.05) {
        io.feed(std::string(c) + "\n");
        run(settle);
        return io.takeLines();
    }
    static bool has(const std::vector<std::string>& v, const char* s) {
        for (auto& l : v)
            if (l.find(s) != std::string::npos) return true;
        return false;
    }
};
}  // namespace

PQ_TEST(sim, end_to_end_protocol_session) {
    static App a;
    a.run(1.0);
    a.io.takeLines();
    PQ_CHECK(App::has(a.cmd("PING"), "pong"));
    auto st = a.cmd("STATUS");
    PQ_CHECK(App::has(st, "\"type\":\"status\"") && App::has(st, "\"running\":true"));
    PQ_CHECK(App::has(a.cmd("METRICS ONCE"), "\"type\":\"metrics\""));
    PQ_CHECK(App::has(a.cmd("CONFIG GET pq.sagThreshold"), "\"value\":0.9"));
    PQ_CHECK(App::has(a.cmd("CONFIG SET pq.sagThreshold 2"), "\"type\":\"error\""));
    PQ_CHECK(App::has(a.cmd("BOGUS"), "unknown command"));

    // disturbance -> event stream
    auto lines = a.cmd("SIM sag 0.5 0.2", 1.0);
    PQ_CHECK(App::has(lines, "\"state\":\"start\",\"event\":\"VOLTAGE_SAG\""));
    PQ_CHECK(App::has(lines, "\"state\":\"end\""));
    auto ev = a.cmd("EVENTS");
    PQ_CHECK(App::has(ev, "\"type\":\"event_log\""));
    PQ_CHECK(App::has(a.cmd("EVENTS TEXT 1"), "# Event #"));

    // waveform snapshot arrives in chunks
    auto w = a.cmd("WAVE 64", 0.1);
    int chunks = 0;
    for (auto& l : w) chunks += l.find("\"type\":\"wave\"") != std::string::npos;
    PQ_CHECK(chunks == 2);

    // STOP halts the stream of frames, START resumes
    a.cmd("STOP");
    auto s1 = a.cmd("STATUS");
    PQ_CHECK(App::has(s1, "\"running\":false"));
    a.cmd("START");
    a.run(0.5);
    PQ_CHECK(App::has(a.cmd("STATUS"), "\"running\":true"));

    // CSV format
    a.cmd("FORMAT CSV");
    PQ_CHECK(App::has(a.cmd("METRICS ONCE"), "M,"));
    a.cmd("FORMAT JSON");
}

PQ_TEST(sim, config_change_restarts_pipeline_live) {
    static App a;
    a.run(0.5);
    a.cmd("CONFIG SET acq.fft_size 512");
    a.cmd("CONFIG SET acq.hop 256");
    a.run(1.0);
    auto st = a.cmd("STATUS");
    PQ_CHECK(App::has(st, "\"fft_size\":512") && App::has(st, "\"hop\":256"));
    PQ_CHECK(App::has(a.cmd("METRICS ONCE"), "\"valid\":true"));
}

PQ_TEST(sim, fault_injection_is_reported_not_hidden) {
    static App a;
    a.run(0.5);
    a.cmd("SIM FAULT CRC 0.002");
    a.run(1.0);
    auto st = a.cmd("STATUS");
    PQ_CHECK(App::has(st, "INVALID_SAMPLES"));
    a.cmd("SIM FAULT CRC 0");
    a.cmd("SIM FAULT STALL ON");
    a.run(1.5);
    st = a.cmd("STATUS");
    PQ_CHECK(App::has(st, "ADC_COMM"));
    a.cmd("SIM FAULT STALL OFF");
    a.cmd("STATUS CLEAR");
    a.run(1.0);
    st = a.cmd("STATUS");
    PQ_CHECK(App::has(st, "\"errors_active\":[]") || App::has(st, "\"errors_active\":[\"ML_INIT\"]"));
}

PQ_TEST(sim, offset_and_gain_calibration_procedures) {
    static App a;
    // Hardware-like offsets and a 3 % gain error in the "sensor"
    GeneratorParams& g = a.sim.generator().mutableParams();
    g.v_offset_counts = 1500.0f;
    g.i_offset_counts = -800.0f;
    g.v_gain *= 1.03f;   // sensor gives 3 % fewer counts than nominal
    a.run(0.5);
    // offset: inputs at zero
    a.cmd("SIM BASE level 0");
    a.cmd("SIM BASE load 0");
    a.cmd("SIM BASE noise 0");
    a.cmd("CALIBRATE OFFSET 500");
    a.run(0.7);
    auto l = a.io.takeLines();
    PQ_CHECK(App::has(l, "\"step\":\"OFFSET\""));
    // residual = sensor noise averaged over 5000 samples (~5 counts rms)
    PQ_CHECK_NEAR(a.sup.config().cal.v_offset, 1500.0, 15.0);
    PQ_CHECK_NEAR(a.sup.config().cal.i_offset, -800.0, 15.0);
    // gain: 230 V reference applied
    a.cmd("SIM BASE level 1");
    a.cmd("SIM BASE load 5");
    a.run(1.0);
    a.cmd("CALIBRATE VGAIN 230");
    a.run(2.0);
    l = a.io.takeLines();
    PQ_CHECK(App::has(l, "\"step\":\"VGAIN\""));
    a.run(1.0);
    PQ_CHECK_NEAR(a.sup.lastFrame().m.vrms, 230.0, 0.3);
}

PQ_TEST(sim, phase_calibration_procedure) {
    static App a;
    a.cmd("SIM BASE pf 0");      // resistive reference load
    a.cmd("SIM BASE phase 1.5"); // CT makes the current lead by 1.5 deg
    a.run(1.5);
    PQ_CHECK_NEAR(a.sup.lastFrame().m.phi_vi_deg, -1.5, 0.1);
    a.cmd("CALIBRATE PHASE 0");
    a.run(2.0);
    PQ_CHECK_NEAR(a.sup.config().cal.phase_deg, 1.5, 0.1);
    a.run(1.0);
    PQ_CHECK_NEAR(a.sup.lastFrame().m.phi_vi_deg, 0.0, 0.1);
}

// ----------------------------------------------------------- integration
PQ_TEST(integration, all_disturbance_types_end_to_end) {
    SystemConfig c = configDefaults();   // ML on (if a model is compiled in)
    Rng r(5);
    const char* names[] = {"sag", "swell", "interruption", "harmonic", "transient", "frequency", "load"};
    const PqClass expect[] = {PqClass::Sag, PqClass::Swell, PqClass::Interruption, PqClass::Harmonic,
                              PqClass::Transient, PqClass::FrequencyVariation, PqClass::LoadSwitching};
    for (int k = 0; k < 7; ++k) {
        Disturbance d;
        makeScenario(names[k], 1.0, 0, 0, r, d);
        const double len = k == 3 ? 3.5 : (k == 5 ? 5.0 : 2.5);
        auto run = runPipeline(c, cleanParams(), {d}, len);
        const EventRecord* e = firstEnded(run, expect[k]);
        PQ_CHECK(e != nullptr);
        int others = 0;
        for (auto& ev : run.events)
            if (ev.state == EV_END && ev.dsp_type != expect[k] && !(ev.flags & EVF_ML_ONLY)) ++others;
        PQ_CHECK(others == 0);
        if (e)
            PQ_INFO("%-13s -> %-20s ML %-20s conf %.2f  det %.1f ms  cls %.1f ms", names[k], pqClassName(e->final_type),
                    e->ml_conf > 0 ? pqClassShortName(e->ml_type) : "-", e->fused_conf, e->det_latency_ms,
                    e->cls_latency_ms);
    }
}

PQ_TEST(integration, invalid_samples_are_tolerated_and_flagged) {
    SystemConfig c = configDefaults();
    c.ml.ml_enable = 0;
    auto run = runPipeline(c, cleanParams(), {}, 3.0, 0.0005f);   // ~5 bad samples per second
    PQ_CHECK(countEndedAny(run) == 0);                              // no false events from glitches
    bool flagged = false;
    for (auto& f : run.frames) flagged |= (f.m.quality & ERR_INVALID_SAMPLES) != 0;
    PQ_CHECK(flagged);
    PQ_CHECK_NEAR(run.frames.back().m.vrms, 230.0, 0.5);
}
