#include "outputs/control_output.h"

#include <cstring>

#include "common/math_util.h"

namespace pq {

namespace {
ControlOutput* g_inst = nullptr;
}

void ControlOutput::init(ControlHw* hw, const ControlConfig& cfg) {
    hw_ = hw;
    std::memset(&st_, 0, sizeof(st_));
    configure(cfg);
    if (hw_) {
        hw_->init();
        hw_->setTripLine(false);
        hw_->setEventCode(0, false);
        for (int c = 0; c < static_cast<int>(RefChannel::Count); ++c)
            hw_->setReference(static_cast<RefChannel>(c), 0.0f);
    }
}

void ControlOutput::configure(const ControlConfig& cfg) {
    cfg_ = cfg;
    st_.armed = cfg.armed != 0;
    if (!st_.armed && hw_) {
        for (int c = 0; c < static_cast<int>(RefChannel::Count); ++c)
            hw_->setReference(static_cast<RefChannel>(c), 0.0f);
    }
}

void ControlOutput::arm(bool on) {
    cfg_.armed = on ? 1 : 0;
    configure(cfg_);
}

void ControlOutput::setEvent(PqClass c, bool active) {
    st_.event_code = active ? c : PqClass::Normal;
    // Event code is purely informational -> driven even when disarmed.
    if (hw_) hw_->setEventCode(static_cast<uint8_t>(st_.event_code), active);
}

void ControlOutput::setReference(RefChannel ch, float value_pu) {
    const int i = static_cast<int>(ch);
    if (i < 0 || i >= static_cast<int>(RefChannel::Count)) return;
    const bool enabled = st_.armed && ((ch == RefChannel::DVR && cfg_.dvr_enable) ||
                                       (ch == RefChannel::APF && cfg_.apf_enable));
    const float v = enabled ? clampv(sanitize(value_pu), -1.0f, 1.0f) : 0.0f;
    st_.ref[i] = v;
    if (hw_) hw_->setReference(ch, v);
}

bool ControlOutput::trip(PqClass reason, float value, uint64_t now) {
    if (!st_.armed || st_.tripped) return false;
    st_.tripped = true;
    st_.trip_reason = reason;
    st_.trip_value = value;
    st_.trip_time_us = now;
    ++st_.trip_count;
    if (hw_) hw_->setTripLine(true);
    return true;
}

void ControlOutput::resetTrip() {
    st_.tripped = false;
    if (hw_) hw_->setTripLine(false);
}

void ControlOutput::heartbeat(uint64_t now_us) {
    if (hw_) hw_->service();
    if (now_us - hb_last_us_ >= 5000) {   // 100 Hz square wave
        hb_last_us_ = now_us;
        hb_ = !hb_;
        if (hw_) hw_->setHealthy(hb_);
    }
}

void ControlOutput::updateDvr(float vpu) {
    // Required series injection to restore 1.0 pu; limited to the +/-0.5 pu range
    // a typical DVR can provide. Below 0.1 pu (interruption) a DVR cannot help.
    float ref = (vpu < 0.1f) ? 0.0f : clampv(1.0f - vpu, -0.5f, 0.5f);
    if (vpu > 0.97f && vpu < 1.03f) ref = 0.0f;   // dead band around nominal
    setReference(RefChannel::DVR, ref);
}

size_t ControlOutput::buildHarmonicPacket(uint8_t* out, size_t cap, const float* i_h, const float* i_ph,
                                          float thd_i, uint32_t frame, int n) {
    const size_t payload = 1 + 4 + 4 + 1 + static_cast<size_t>(n) * 8;
    const size_t total = 3 + payload + 2;
    if (total > cap || payload > 255) return 0;
    size_t p = 0;
    out[p++] = 0xA5;
    out[p++] = 0x5A;
    out[p++] = static_cast<uint8_t>(payload);
    const size_t crc_from = p;
    out[p++] = 0x01;
    std::memcpy(out + p, &frame, 4); p += 4;
    std::memcpy(out + p, &thd_i, 4); p += 4;
    out[p++] = static_cast<uint8_t>(n);
    for (int h = 1; h <= n; ++h) {
        std::memcpy(out + p, &i_h[h], 4); p += 4;
        std::memcpy(out + p, &i_ph[h], 4); p += 4;
    }
    const uint16_t crc = crc16Ccitt(out + crc_from, p - crc_from);
    out[p++] = static_cast<uint8_t>(crc & 0xFF);
    out[p++] = static_cast<uint8_t>(crc >> 8);
    return p;
}

void ControlOutput::sendHarmonicReference(const float* i_h, const float* i_ph, float thd_i, uint32_t frame) {
    // APF PWM level: THD_i of 0..100 % -> 0..1
    setReference(RefChannel::APF, clampv(thd_i / 100.0f, 0.0f, 1.0f));
    if (!st_.armed || !cfg_.apf_enable || !hw_) return;
    uint8_t pkt[3 + 10 + PQ_REPORT_HARMONICS * 8 + 2];
    const size_t n = buildHarmonicPacket(pkt, sizeof(pkt), i_h, i_ph, thd_i, frame, PQ_REPORT_HARMONICS);
    if (n) {
        hw_->sendPacket(pkt, n);
        ++st_.apf_packets;
    }
}

void ControlOutput_Bind(ControlOutput* inst) { g_inst = inst; }
void ControlOutput_SetEvent(PqClass c, bool active) {
    if (g_inst) g_inst->setEvent(c, active);
}
void ControlOutput_SetReference(RefChannel ch, float value_pu) {
    if (g_inst) g_inst->setReference(ch, value_pu);
}
bool ControlOutput_Trip(PqClass reason, float value, uint64_t now_us) {
    return g_inst ? g_inst->trip(reason, value, now_us) : false;
}

}  // namespace pq
