// Local annunciation (status LED + alarm LED/buzzer) with non-blocking patterns.
#pragma once
#include <cstdint>

#include "common/pq_types.h"

namespace pq {

class AlarmHw {
public:
    virtual ~AlarmHw() = default;
    virtual void init() = 0;
    virtual void setStatusLed(bool on) = 0;
    virtual void setAlarm(bool on) = 0;     // red LED / buzzer driver
};

enum class AlarmLevel : uint8_t { None = 0, Info, Warning, Critical, Fault };

class AlarmOutput {
public:
    void init(AlarmHw* hw);
    // Raise an alarm for `hold_ms` (re-raising extends / escalates).
    void raise(AlarmLevel lvl, uint32_t hold_ms, uint64_t now_us);
    void setFault(bool fault) { fault_ = fault; }
    void update(uint64_t now_us);            // call from the supervisor loop
    AlarmLevel level() const { return lvl_; }
    static AlarmLevel levelFor(PqClass c);

private:
    AlarmHw*   hw_ = nullptr;
    AlarmLevel lvl_ = AlarmLevel::None;
    uint64_t   until_us_ = 0;
    bool       fault_ = false;
};

}  // namespace pq
