#include "storage/event_logger.h"

#include <cstdio>

namespace pq {

void EventLogger::clear() {
    head_ = count_ = 0;
    total_ = overwritten_ = 0;
}

const EventRecord* EventLogger::at(size_t i) const {
    if (i >= count_) return nullptr;
    const size_t start = (head_ + PQ_EVENT_LOG_SIZE - count_) % PQ_EVENT_LOG_SIZE;
    return &e_[(start + i) % PQ_EVENT_LOG_SIZE];
}

const EventRecord* EventLogger::findById(uint32_t id) const {
    for (size_t i = count_; i-- > 0;) {
        const EventRecord* r = at(i);
        if (r->id == id) return r;
    }
    return nullptr;
}

const EventRecord* EventLogger::record(const EventRecord& r) {
    if (!enabled_) return nullptr;
    for (size_t i = count_; i-- > 0;) {
        EventRecord* x = const_cast<EventRecord*>(at(i));
        if (x->id == r.id) {
            // keep the classification if the END note arrives without one
            const bool had_cls = x->state >= EV_CLASSIFIED && x->state != EV_START;
            EventRecord merged = r;
            if (r.state == EV_END && had_cls && r.cls_latency_ms == 0.0f) {
                merged.final_type = x->final_type;
                merged.ml_type = x->ml_type;
                merged.ml_conf = x->ml_conf;
                merged.fused_conf = x->fused_conf;
                merged.source = x->source;
                merged.cls_latency_ms = x->cls_latency_ms;
                merged.flags |= x->flags;
            }
            if (r.state == EV_CLASSIFIED && x->state == EV_END) {   // late classification
                merged.state = EV_END;
                merged.duration_ms = x->duration_ms;
                merged.magnitude = x->magnitude;
            }
            merged.components |= x->components;
            *x = merged;
            return x;
        }
    }
    if (count_ == PQ_EVENT_LOG_SIZE) ++overwritten_;
    else ++count_;
    e_[head_] = r;
    const EventRecord* p = &e_[head_];
    head_ = (head_ + 1) % PQ_EVENT_LOG_SIZE;
    ++total_;
    return p;
}

size_t EventLogger::formatText(const EventRecord& r, char* out, size_t cap) {
    const char* state = r.state == EV_END ? "ended" : (r.state == EV_CLASSIFIED ? "active" : "detected");
    int n = snprintf(out, cap,
                     "Event #%lu (%s)\n"
                     "  Timestamp: %.3f s\n"
                     "  Event: %s\n"
                     "  DSP detection: %s\n"
                     "  Magnitude: %.3f\n"
                     "  Duration: %lu ms\n"
                     "  Vrms: %.1f V\n"
                     "  Frequency: %.2f Hz\n"
                     "  THD: %.1f %%\n"
                     "  ML Class: %s\n"
                     "  Confidence: %.1f %%\n"
                     "  Detection latency: %.1f ms\n"
                     "  Classification latency: %.1f ms\n",
                     static_cast<unsigned long>(r.id), state, r.t_start_us * 1e-6, pqClassName(r.final_type),
                     pqClassName(r.dsp_type), static_cast<double>(r.magnitude),
                     static_cast<unsigned long>(r.duration_ms), static_cast<double>(r.vrms),
                     static_cast<double>(r.freq), static_cast<double>(r.thd), pqClassShortName(r.ml_type),
                     static_cast<double>(r.ml_conf * 100.0f), static_cast<double>(r.det_latency_ms),
                     static_cast<double>(r.cls_latency_ms));
    return n < 0 ? 0 : (static_cast<size_t>(n) < cap ? static_cast<size_t>(n) : cap - 1);
}

}  // namespace pq
