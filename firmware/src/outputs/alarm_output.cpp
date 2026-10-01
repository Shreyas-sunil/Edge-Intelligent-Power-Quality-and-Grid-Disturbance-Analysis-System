#include "outputs/alarm_output.h"

namespace pq {

void AlarmOutput::init(AlarmHw* hw) {
    hw_ = hw;
    if (hw_) {
        hw_->init();
        hw_->setAlarm(false);
        hw_->setStatusLed(false);
    }
}

AlarmLevel AlarmOutput::levelFor(PqClass c) {
    switch (c) {
        case PqClass::Interruption:
        case PqClass::Swell:
        case PqClass::Combined: return AlarmLevel::Critical;
        case PqClass::Sag:
        case PqClass::Transient:
        case PqClass::Harmonic:
        case PqClass::FrequencyVariation: return AlarmLevel::Warning;
        case PqClass::LoadSwitching: return AlarmLevel::Info;
        default: return AlarmLevel::None;
    }
}

void AlarmOutput::raise(AlarmLevel lvl, uint32_t hold_ms, uint64_t now_us) {
    if (lvl >= lvl_ || now_us >= until_us_) lvl_ = lvl;
    const uint64_t u = now_us + static_cast<uint64_t>(hold_ms) * 1000u;
    if (u > until_us_) until_us_ = u;
}

void AlarmOutput::update(uint64_t now_us) {
    if (!hw_) return;
    if (now_us >= until_us_) lvl_ = AlarmLevel::None;
    const uint32_t ms = static_cast<uint32_t>(now_us / 1000u);
    // Status LED: 1 Hz heartbeat normally, fast 5 Hz blink on system fault.
    hw_->setStatusLed(fault_ ? ((ms / 100) & 1) : ((ms % 1000) < 100));
    bool a = false;
    switch (fault_ ? AlarmLevel::Fault : lvl_) {
        case AlarmLevel::None: a = false; break;
        case AlarmLevel::Info: a = (ms % 2000) < 50; break;       // short blip
        case AlarmLevel::Warning: a = (ms % 1000) < 250; break;   // slow blink
        case AlarmLevel::Critical: a = (ms % 250) < 125; break;   // fast blink
        case AlarmLevel::Fault: a = lvl_ != AlarmLevel::None ? true : (ms % 2000) < 1000; break;
    }
    hw_->setAlarm(a);
}

}  // namespace pq
