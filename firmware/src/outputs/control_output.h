// =============================================================================
//  Control / reference output abstraction for downstream compensators.
//
//  This firmware NEVER switches power itself. It only drives low-voltage logic
//  and reference signals that an external, independently protected controller
//  may use:
//
//    SSCB  : TRIP request line (latched until reset) + HEALTHY heartbeat.
//            The breaker hardware should treat "no heartbeat" as a fault so a
//            hung MCU fails safe.
//    DVR   : sag/swell compensation reference = required injection in pu,
//            (1 - Vpu), clamped to [-0.5, +0.5]; output on PWM (RC-filtered to
//            an analog level) or an external SPI DAC. 0 pu == mid-scale.
//    APF   : harmonic compensation reference: current THD level on PWM and a
//            framed binary packet with per-harmonic I_h magnitude/phase on a
//            dedicated UART for a digital APF controller.
//    EVENT : 4-bit event code + strobe on GPIO for a PLC / relay board.
//
//  All actuating outputs are inert until armed (`CONTROL ARM`).
// =============================================================================
#pragma once
#include <cstddef>
#include <cstdint>

#include "common/pq_types.h"
#include "pq_build_config.h"
#include "system/config.h"

namespace pq {

enum class RefChannel : uint8_t { DVR = 0, APF = 1, Count };

// Hardware back-end (Pico GPIO/PWM/UART/DAC, or a host recorder for tests).
class ControlHw {
public:
    virtual ~ControlHw() = default;
    virtual void init() = 0;
    virtual void setTripLine(bool asserted) = 0;
    virtual void setHealthy(bool level) = 0;
    virtual void setEventCode(uint8_t code, bool strobe) = 0;
    virtual void setReference(RefChannel ch, float normalized) = 0;   // -1..+1
    virtual void sendPacket(const uint8_t* data, size_t len) = 0;     // APF link
    virtual void service() {}                                          // periodic housekeeping
};

struct ControlState {
    bool     armed;
    bool     tripped;
    PqClass  trip_reason;
    float    trip_value;
    uint64_t trip_time_us;
    uint32_t trip_count;
    PqClass  event_code;
    float    ref[static_cast<int>(RefChannel::Count)];   // pu
    uint32_t apf_packets;
};

class ControlOutput {
public:
    void init(ControlHw* hw, const ControlConfig& cfg);
    void configure(const ControlConfig& cfg);

    // --- API requested by the specification -----------------------------------
    void setEvent(PqClass c, bool active);                 // ControlOutput_SetEvent
    void setReference(RefChannel ch, float value_pu);      // ControlOutput_SetReference
    bool trip(PqClass reason, float value, uint64_t now);  // ControlOutput_Trip (armed only)

    void arm(bool on);
    void resetTrip();
    void heartbeat(uint64_t now_us);                       // call >= every 10 ms
    // Per-frame APF packet: I_h RMS [A] & phase [rad] for h = 1..PQ_REPORT_HARMONICS.
    void sendHarmonicReference(const float* i_h, const float* i_ph, float thd_i, uint32_t frame);
    // DVR reference from the latest half-cycle Vpu.
    void updateDvr(float vpu);

    const ControlState& state() const { return st_; }

    // Packet format (little-endian): 0xA5 0x5A len(1) type(1)=0x01 frame(u32)
    //   thd_i(f32) n(u8) {mag f32, phase f32} x n  crc16-ccitt(u16 over type..payload)
    static size_t buildHarmonicPacket(uint8_t* out, size_t cap, const float* i_h, const float* i_ph,
                                      float thd_i, uint32_t frame, int n);

private:
    ControlHw*    hw_ = nullptr;
    ControlConfig cfg_{};
    ControlState  st_{};
    bool          hb_ = false;
    uint64_t      hb_last_us_ = 0;
};

// C-style wrappers bound to the global instance (spec API).
void ControlOutput_Bind(ControlOutput* inst);
void ControlOutput_SetEvent(PqClass c, bool active);
void ControlOutput_SetReference(RefChannel ch, float value_pu);
bool ControlOutput_Trip(PqClass reason, float value, uint64_t now_us);

}  // namespace pq
