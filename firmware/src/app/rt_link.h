// =============================================================================
//  Inter-core link between the real-time task (core1) and the supervisor
//  (core0). Everything is lock-free:
//    core0 -> core1 : command ring + config mailbox (acknowledged by sequence)
//    core1 -> core0 : frame ring, event ring, status snapshot (seqlock),
//                     waveform snapshot (flag handshake)
//  On the host build both "cores" run in one thread; the same code applies.
// =============================================================================
#pragma once
#include <atomic>
#include <cstdint>

#include "adc/adc_types.h"
#include "app/pq_engine.h"
#include "common/spsc_ring.h"
#include "outputs/control_output.h"
#include "system/config.h"

namespace pq {

enum class RtCmdType : uint8_t {
    ApplyConfig,    // take cfg from mailbox
    Start,
    Stop,
    SimScenario,    // name, a = magnitude (0 = default), b = duration (0 = default), c = delay s
    SimClear,
    SimFaultCrc,    // a = per-sample probability
    SimFaultStall,  // flag
    SimBase,        // name = "noise"|"freq"|"load"|"level"|"harm", a = value
    CalRawReset,
    WaveCapture,
    CtrlArm,        // flag
    CtrlReset,
    CtrlTest,       // assert event code outputs briefly (flag = code)
    StatsReset,
};

struct RtCmd {
    RtCmdType type;
    uint8_t   flag;
    float     a, b, c;
    char      name[16];
};

struct RtStatus {
    bool           running;
    bool           simulated;
    bool           ml_ready;
    const char*    backend;
    float          fs;
    float          analysis_fs;
    int            fft_size;
    int            hop;
    AdcStatus      adc_status;
    AdcDiagnostics adc;
    EngineStats    eng;
    uint32_t       errors;           // ErrorFlag bits (sticky until STATUS CLEAR)
    uint32_t       errors_now;       // currently active conditions
    float          cpu_load_pct;     // processing only (excl. simulator generation)
    float          cpu_total_pct;    // incl. simulator waveform generation
    float          cpu_peak_pct;
    uint32_t       block_budget_us;
    uint32_t       ring_blocks;
    uint32_t       frames_dropped;   // frame ring full (supervisor too slow)
    uint32_t       events_dropped;
    uint32_t       reinit_count;
    uint64_t       uptime_us;
    float          vrms_half;
    PqClass        voltage_state;
    ControlState   ctrl;
    double         raw_mean[kAdcChannels];
    uint64_t       raw_n;
    uint32_t       raw_resets;       // increments on every CalRawReset (handshake)
    float          phase_delay_samples;
    uint32_t       cfg_applied_seq;
    const char*    last_cfg_error;
};

struct RtShared {
    SpscRing<RtCmd, 16>       cmds;
    SpscRing<FrameResult, 4>  frames;
    SpscRing<EventRecord, 32> events;

    // config mailbox
    SystemConfig          cfg_mailbox;
    std::atomic<uint32_t> cfg_seq{0};
    std::atomic<uint32_t> cfg_applied{0};

    // status seqlock
    std::atomic<uint32_t> status_seq{0};
    RtStatus              status{};

    // waveform snapshot
    std::atomic<uint8_t> wave_ready{0};
    float    wave_v[PQ_WAVE_SNAPSHOT];
    float    wave_i[PQ_WAVE_SNAPSHOT];
    uint32_t wave_n = 0;
    float    wave_fs = 0.0f;
    uint64_t wave_t0_us = 0;

    void publishStatus(const RtStatus& s) {
        status_seq.fetch_add(1, std::memory_order_acq_rel);   // odd: writing
        status = s;
        status_seq.fetch_add(1, std::memory_order_acq_rel);   // even: stable
    }
    bool readStatus(RtStatus& out) const {
        for (int tries = 0; tries < 8; ++tries) {
            const uint32_t a = status_seq.load(std::memory_order_acquire);
            if (a & 1u) continue;
            out = status;
            std::atomic_thread_fence(std::memory_order_acquire);
            if (status_seq.load(std::memory_order_acquire) == a) return true;
        }
        return false;
    }
};

}  // namespace pq
