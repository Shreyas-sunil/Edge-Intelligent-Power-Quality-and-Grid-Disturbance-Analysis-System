// =============================================================================
//  Supervisor (core0): command protocol, metrics/event streaming, event log,
//  alarms, calibration procedures, configuration persistence, benchmarks and
//  system monitoring. It never touches the ADC or the DSP state directly — all
//  interaction with the real-time core goes through RtShared.
// =============================================================================
#pragma once
#include <cstddef>
#include <cstdint>

#include "app/rt_link.h"
#include "communication/transport.h"
#include "outputs/alarm_output.h"
#include "storage/event_logger.h"
#include "system/config_store.h"
#include "system/system_monitor.h"

namespace pq {

struct SupervisorEnv {
    RtShared*    link = nullptr;
    Transport*   usb = nullptr;       // primary (USB CDC / host stdio)
    Transport*   uart = nullptr;      // optional mirror
    ConfigStore* store = nullptr;
    AlarmHw*     alarm_hw = nullptr;
    uint32_t     boot_errors = 0;     // e.g. ERR_WATCHDOG_RESET, ERR_CONFIG_INVALID
};

class Supervisor {
public:
    void init(const SupervisorEnv& env, const SystemConfig& cfg);
    void poll();

    // Execute one command line as if received from source `src` (0 = usb, 1 = uart).
    void command(const char* line, int src = 0);

    const SystemConfig& config() const { return cfg_; }
    const EventLogger&  log() const { return log_; }
    uint32_t errors() const { return errors_; }
    bool haveFrame() const { return have_frame_; }
    const FrameResult& lastFrame() const { return last_; }
    const TimingStats& timing() const { return timing_; }

private:
    enum { kSrcUsb = 1, kSrcUart = 2, kSrcAll = 3 };
    void send(int dest, const char* s, size_t n);
    void sendStr(int dest, const char* s);
    void reply(int src, const char* s, size_t n) { send(src == 1 ? kSrcUart : kSrcUsb, s, n); }
    void ack(int src, const char* cmd, const char* msg = nullptr);
    void sendHello(int dest);
    void err(int src, const char* cmd, const char* msg);
    int  streamDest() const { return kSrcUsb | (cfg_.comm.uart_enable ? kSrcUart : 0); }

    void pumpTransport(Transport* t, LineReader& lr, int src);
    void drainFrames(uint64_t now);
    void drainEvents(uint64_t now);
    void drainLogs();
    void periodicMetrics(uint64_t now);
    void pumpWave();
    void pumpCalibration(uint64_t now);
    bool pushConfig();
    void rtCmd(RtCmdType t, uint8_t flag = 0, float a = 0, float b = 0, float c = 0, const char* name = nullptr);

    void cmdStatus(int src);
    void cmdConfig(int src, char** tok, int n);
    void cmdCalibrate(int src, char** tok, int n);
    void cmdSim(int src, char** tok, int n);
    void cmdEvents(int src, char** tok, int n);
    void cmdBench(int src);
    void cmdModel(int src);
    void cmdHelp(int src);
    void sendConfigParam(int src, const ParamDesc& p);

    SupervisorEnv env_{};
    SystemConfig  cfg_{};
    EventLogger   log_;
    AlarmOutput   alarm_;
    LineReader    lr_usb_, lr_uart_;
    TimingStats   timing_;
    FrameResult   last_{};
    bool          have_frame_ = false;
    uint32_t      errors_ = 0;
    uint32_t      tx_dropped_ = 0;
    uint64_t      last_metrics_us_ = 0;
    uint64_t      last_rx_us_ = 0;
    uint64_t      boot_us_ = 0;
    bool          metrics_once_ = false;
    int           metrics_once_src_ = 0;
    bool          csv_header_pending_ = false;
    bool          cfg_dirty_ = false;
    bool          usb_was_connected_ = false;

    // worst classification since the last metrics line
    PqClass period_class_ = PqClass::Normal;
    float   period_conf_ = 0.0f;

    // waveform streaming
    bool     wave_pending_ = false;
    int      wave_src_ = 0;
    uint32_t wave_off_ = 0;
    uint32_t wave_req_n_ = PQ_WAVE_SNAPSHOT;
    uint64_t wave_req_us_ = 0;

    // calibration state machine
    enum class CalMode : uint8_t { None, Offset, VGain, IGain, Phase };
    CalMode  cal_mode_ = CalMode::None;
    int      cal_src_ = 0;
    uint64_t cal_deadline_us_ = 0;
    float    cal_ref_ = 0.0f;
    double   cal_acc_ = 0.0;
    uint32_t cal_n_ = 0;
    uint32_t cal_need_ = 10;
    uint32_t cal_reset_seq_ = 0;

    char out_[1400];
};

}  // namespace pq
