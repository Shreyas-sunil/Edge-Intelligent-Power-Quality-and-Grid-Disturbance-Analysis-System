#include "communication/protocol.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "features/feature_spec.h"

#include "common/math_util.h"

namespace pq {

int tokenize(char* line, char** tok, int max) {
    int n = 0;
    char* p = line;
    while (*p && n < max) {
        while (*p == ' ' || *p == '\t' || *p == ',') ++p;
        if (!*p) break;
        tok[n++] = p;
        while (*p && *p != ' ' && *p != '\t') ++p;
        if (*p) *p++ = 0;
    }
    return n;
}

bool tokEq(const char* a, const char* b) { return strEqualNoCase(a, b); }

bool parseFloat(const char* s, float& out) {
    if (!s || !*s) return false;
    char* end = nullptr;
    const double v = std::strtod(s, &end);
    if (end == s || *end) return false;
    out = static_cast<float>(v);
    return true;
}

const char* errorFlagName(uint32_t bit) {
    switch (bit) {
        case ERR_ADC_COMM: return "ADC_COMM";
        case ERR_DMA_OVERRUN: return "DMA_OVERRUN";
        case ERR_BUFFER_OVERFLOW: return "BUFFER_OVERFLOW";
        case ERR_INVALID_SAMPLES: return "INVALID_SAMPLES";
        case ERR_SATURATION: return "ADC_SATURATION";
        case ERR_MISSING_SAMPLES: return "MISSING_SAMPLES";
        case ERR_ML_INIT: return "ML_INIT";
        case ERR_COMM_TIMEOUT: return "COMM_TIMEOUT";
        case ERR_CONFIG_INVALID: return "CONFIG_INVALID";
        case ERR_FLASH: return "FLASH";
        case ERR_FREQ_LOST: return "FREQ_LOST";
        case ERR_WATCHDOG_RESET: return "WATCHDOG_RESET";
        default: return "?";
    }
}

void writeErrorList(JsonWriter& w, const char* key, uint32_t mask) {
    w.beginArr(key);
    for (uint32_t b = 1; b && b <= ERR_WATCHDOG_RESET; b <<= 1)
        if (mask & b) w.val(errorFlagName(b));
    w.endArr();
}

const char* eventStateName(uint8_t s) {
    switch (s) {
        case EV_START: return "start";
        case EV_CLASSIFIED: return "classified";
        case EV_END: return "end";
        default: return "?";
    }
}

const char* eventSourceName(uint8_t s) {
    switch (s) {
        case SRC_DSP: return "dsp";
        case SRC_ML: return "ml";
        case SRC_DSP_ML: return "dsp+ml";
        default: return "?";
    }
}

size_t fmtMetricsJson(char* buf, size_t cap, const FrameResult& f, PqClass event, float conf, uint64_t ts_ms,
                      uint32_t errors) {
    const PqMetrics& m = f.m;
    JsonWriter w(buf, cap);
    w.beginObj()
        .kv("type", "metrics")
        .kv("timestamp_ms", ts_ms)
        .kv("frame", f.frame_idx)
        .kv("vrms", m.vrms, 2)
        .kv("irms", m.irms, 3)
        .kv("frequency", m.freq_valid ? m.freq : NAN, 3)   // null while no zero crossings
        .kv("thd", m.thd_v, 2)
        .kv("thd_i", m.thd_i, 2)
        .kv("event", pqClassShortName(event))
        .kv("confidence", conf, 3)
        .kv("ml_class", f.ml.valid ? pqClassShortName(static_cast<PqClass>(f.ml.class_id)) : "NONE")
        .kv("ml_conf", f.ml.valid ? f.ml.confidence : 0.0f, 3)
        .kv("vrms_half", m.vrms_half, 2)
        .kv("vmin", m.vmin_half, 2)
        .kv("vmax", m.vmax_half, 2)
        .kv("h1", m.v_h[1], 2)
        .kv("h3", m.v_h[3], 3)
        .kv("h5", m.v_h[5], 3)
        .kv("h7", m.v_h[7], 3)
        .kv("p", m.p, 1)
        .kv("s", m.s, 1)
        .kv("pf", m.pf, 3)
        .kv("dpf", m.dpf, 3)
        .kv("rocof", m.rocof, 3)
        .kv("crest", m.crest, 3)
        .kvBool("valid", m.valid != 0)
        .kvHex("q", m.quality | errors)
        .endObj();
    return w.finish();
}

size_t fmtMetricsCsvHeader(char* buf, size_t cap) {
    const int n = snprintf(buf, cap,
                           "# M,timestamp_ms,frame,vrms,irms,frequency,thd,thd_i,event,confidence,ml_class,"
                           "ml_conf,vrms_half,h1,h3,h5,h7,p,pf,valid,q\n");
    return (n > 0 && static_cast<size_t>(n) < cap) ? static_cast<size_t>(n) : 0;
}

size_t fmtMetricsCsv(char* buf, size_t cap, const FrameResult& f, PqClass event, float conf, uint64_t ts_ms,
                     uint32_t errors) {
    const PqMetrics& m = f.m;
    const int n = snprintf(
        buf, cap, "M,%llu,%lu,%.2f,%.3f,%.3f,%.2f,%.2f,%s,%.3f,%s,%.3f,%.2f,%.2f,%.3f,%.3f,%.3f,%.1f,%.3f,%d,0x%08lX\n",
        static_cast<unsigned long long>(ts_ms), static_cast<unsigned long>(f.frame_idx), static_cast<double>(m.vrms), static_cast<double>(m.irms),
        m.freq_valid ? static_cast<double>(m.freq) : NAN, static_cast<double>(m.thd_v), static_cast<double>(m.thd_i),
        pqClassShortName(event), static_cast<double>(conf),
        f.ml.valid ? pqClassShortName(static_cast<PqClass>(f.ml.class_id)) : "NONE",
        static_cast<double>(f.ml.valid ? f.ml.confidence : 0.0f), static_cast<double>(m.vrms_half),
        static_cast<double>(m.v_h[1]), static_cast<double>(m.v_h[3]), static_cast<double>(m.v_h[5]),
        static_cast<double>(m.v_h[7]), static_cast<double>(m.p), static_cast<double>(m.pf), m.valid ? 1 : 0,
        static_cast<unsigned long>(m.quality | errors));
    return (n > 0 && static_cast<size_t>(n) < cap) ? static_cast<size_t>(n) : 0;
}

size_t fmtEventJson(char* buf, size_t cap, const EventRecord& r, const char* type) {
    JsonWriter w(buf, cap);
    w.beginObj()
        .kv("type", type)
        .kv("id", r.id)
        .kv("state", eventStateName(r.state))
        .kv("event", pqClassName(r.final_type))
        .kv("dsp", pqClassName(r.dsp_type))
        .kv("ml", r.state == EV_START ? "PENDING" : (r.ml_conf > 0.0f ? pqClassShortName(r.ml_type) : "NONE"))
        .kv("ml_conf", r.ml_conf, 3)
        .kv("confidence", r.fused_conf, 3)
        .kv("source", eventSourceName(r.source))
        .kv("t_start_ms", static_cast<double>(r.t_start_us) * 1e-3, 1)
        .kv("duration_ms", r.duration_ms)
        .kv("magnitude", r.magnitude, 3)
        .kv("vrms", r.vrms, 1)
        .kv("frequency", r.freq, 3)
        .kv("thd", r.thd, 2)
        .kv("det_latency_ms", r.det_latency_ms, 1)
        .kv("cls_latency_ms", r.cls_latency_ms, 1);
    w.beginArr("components");
    for (int c = 1; c < kNumPqClasses; ++c)
        if (r.components & classBit(static_cast<PqClass>(c))) w.val(pqClassShortName(static_cast<PqClass>(c)));
    w.endArr();
    w.beginArr("flags");
    if (r.flags & EVF_CLIPPED) w.val("CLIPPED");
    if (r.flags & EVF_WITH_TRANSIENT) w.val("WITH_TRANSIENT");
    if (r.flags & EVF_ML_ONLY) w.val("ML_ONLY_ADVISORY");
    if (r.flags & EVF_ML_DISAGREE) w.val("ML_DISAGREE");
    if (r.flags & EVF_TRIPPED) w.val("TRIPPED");
    if (r.flags & EVF_COMBINED) w.val("COMBINED");
    if (r.flags & EVF_UNCLASSIFIED) w.val("UNCLASSIFIED");
    w.endArr();
    w.endObj();
    return w.finish();
}

size_t fmtHarmonicsJson(char* buf, size_t cap, const FrameResult& f) {
    const PqMetrics& m = f.m;
    JsonWriter w(buf, cap);
    w.beginObj().kv("type", "harmonics").kv("frame", f.frame_idx).kv("f0", m.freq, 3).kv("fft_f0", m.fft_freq, 3);
    w.kv("thd_v", m.thd_v, 3).kv("thd_i", m.thd_i, 3);
    w.beginArr("v");
    for (int h = 1; h <= PQ_REPORT_HARMONICS; ++h) w.val(m.v_h[h], 3);
    w.endArr().beginArr("i");
    for (int h = 1; h <= PQ_REPORT_HARMONICS; ++h) w.val(m.i_h[h], 4);
    w.endArr().beginArr("i_phase_deg");
    for (int h = 1; h <= PQ_REPORT_HARMONICS; ++h) w.val(m.i_ph[h] * 57.29578f, 1);
    w.endArr().beginArr("peaks");
    for (int k = 0; k < m.npeaks; ++k) {
        w.beginObj().kv("hz", m.peaks[k].hz, 1).kv("rms", m.peaks[k].rms, 3).endObj();
    }
    w.endArr().kv("phi_vi_deg", m.phi_vi_deg, 2).endObj();
    return w.finish();
}

size_t fmtFeaturesJson(char* buf, size_t cap, const FrameResult& f) {
    JsonWriter w(buf, cap);
    w.beginObj().kv("type", "features").kv("frame", f.frame_idx);
    w.beginObj("x");
    for (int k = 0; k < kNumFeatures; ++k) w.kv(featureName(k), f.features.v[k], 5);
    w.endObj();
    w.beginArr("ml_probs");
    for (int c = 0; c < kNumMlClasses; ++c) w.val(f.ml.valid ? f.ml.probs[c] : 0.0f, 3);
    w.endArr().beginArr("dsp_evidence");
    for (int c = 0; c < kNumMlClasses; ++c) w.val(f.evidence.e[c], 3);
    w.endArr().beginArr("fused");
    for (int c = 0; c < kNumMlClasses; ++c) w.val(f.fusion.fused[c], 3);
    w.endArr().kv("final", pqClassName(f.fusion.final_class)).endObj();
    return w.finish();
}

}  // namespace pq
