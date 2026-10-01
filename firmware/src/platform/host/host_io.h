// Host-side implementations of the hardware interfaces (tests / simulator).
#pragma once
#include <cstdint>
#include <string>
#include <vector>

#include "adc/sim_adc.h"
#include "communication/transport.h"
#include "outputs/alarm_output.h"
#include "outputs/control_output.h"
#include "system/config_store.h"

namespace pq {

// File-descriptor transport (stdin/stdout, or a pseudo-terminal master).
class FdTransport : public Transport {
public:
    FdTransport(int fd_in, int fd_out, const char* name);
    const char* name() const override { return name_; }
    bool   connected() const override { return true; }
    size_t read(char* buf, size_t max) override;
    size_t writeAvailable() const override;
    size_t write(const char* data, size_t len) override;
    void   flush() override;

private:
    int fd_in_, fd_out_;
    const char* name_;
    std::vector<char> tx_;
    struct RxThread;
    RxThread* rx_ = nullptr;   // Windows: blocking stdin read on a helper thread
    static constexpr size_t kCap = 64 * 1024;
};

// Memory transport for tests: feed() input, collect output lines.
class MemTransport : public Transport {
public:
    const char* name() const override { return "mem"; }
    bool   connected() const override { return true; }
    size_t read(char* buf, size_t max) override;
    size_t writeAvailable() const override { return 1u << 20; }
    size_t write(const char* data, size_t len) override;
    void   feed(const std::string& s) { in_ += s; }
    std::vector<std::string> takeLines();

private:
    std::string in_, out_;
};

// Opens a pseudo-terminal; returns master fd and fills slave path (Linux/macOS).
int openPty(std::string& slave_path);

class FileConfigStore : public ConfigStore {
public:
    explicit FileConfigStore(std::string path) : path_(std::move(path)) {}
    bool load(StoredConfig& out) override;
    bool save(const StoredConfig& in) override;
    bool erase() override;

private:
    std::string path_;
};

// Wall-clock pacer: blocks become due in real time (like the Pico timer).
class RealtimePacer : public SimPacer {
public:
    void     start(uint32_t block_period_us) override;
    void     stop() override {}
    uint32_t blocksDue() override;
    uint64_t startTimeUs() override { return t0_; }

private:
    uint64_t t0_ = 0;
    uint32_t period_ = 5000;
};

struct RecordedControl {
    bool     trip = false;
    uint32_t trip_edges = 0;
    uint8_t  event_code = 0;
    float    ref[2] = {0, 0};
    uint32_t packets = 0;
    std::vector<uint8_t> last_packet;
};

class RecorderControlHw : public ControlHw {
public:
    void init() override {}
    void setTripLine(bool a) override {
        if (a && !rec.trip) ++rec.trip_edges;
        rec.trip = a;
    }
    void setHealthy(bool) override {}
    void setEventCode(uint8_t code, bool) override { rec.event_code = code; }
    void setReference(RefChannel ch, float v) override { rec.ref[static_cast<int>(ch)] = v; }
    void sendPacket(const uint8_t* d, size_t n) override {
        ++rec.packets;
        rec.last_packet.assign(d, d + n);
    }
    RecordedControl rec;
};

class NullAlarmHw : public AlarmHw {
public:
    void init() override {}
    void setStatusLed(bool) override {}
    void setAlarm(bool on) override { alarm = on; }
    bool alarm = false;
};

}  // namespace pq
