// Pico implementations of the hardware-facing interfaces.
#pragma once
#include <cstddef>
#include <cstdint>

#include "adc/sim_adc.h"
#include "communication/transport.h"
#include "outputs/alarm_output.h"
#include "outputs/control_output.h"
#include "system/config_store.h"

namespace pq {

// Sample clock for the simulated ADC: the RP2 64-bit hardware timer. Blocks
// become due exactly every kBlockSize / fs, like DMA completions of a real ADC.
class PicoSimPacer : public SimPacer {
public:
    void     start(uint32_t block_period_us) override;
    void     stop() override {}
    uint32_t blocksDue() override;
    uint64_t startTimeUs() override { return t0_; }

private:
    uint64_t t0_ = 0;
    uint32_t period_ = 5000;
};

// Byte ring used by the transports (non-blocking writers).
template <size_t N>
struct ByteRing {
    uint8_t  d[N];
    size_t   head = 0, tail = 0;
    size_t   used() const { return head - tail; }
    size_t   free() const { return N - used(); }
    void     put(const char* s, size_t n) {
        for (size_t k = 0; k < n; ++k) d[(head++) % N] = static_cast<uint8_t>(s[k]);
    }
    // Contiguous readable span.
    size_t   span(const uint8_t*& p) const {
        const size_t off = tail % N;
        const size_t n = used();
        p = &d[off];
        return (off + n > N) ? N - off : n;
    }
    void     drop(size_t n) { tail += n; }
};

class UsbTransport : public Transport {
public:
    void        init();
    const char* name() const override { return "usb"; }
    bool        connected() const override;
    size_t      read(char* buf, size_t max) override;
    size_t      writeAvailable() const override { return tx_.free(); }
    size_t      write(const char* data, size_t len) override;
    void        flush() override;

private:
    ByteRing<8192> tx_;
};

class UartTransport : public Transport {
public:
    void        init(uint32_t baud);
    const char* name() const override { return "uart"; }
    bool        connected() const override { return inited_; }
    size_t      read(char* buf, size_t max) override;
    size_t      writeAvailable() const override { return tx_.free(); }
    size_t      write(const char* data, size_t len) override;
    void        flush() override;

private:
    ByteRing<4096> tx_;
    bool inited_ = false;
};

class PicoControlHw : public ControlHw {
public:
    void init() override;
    void setTripLine(bool asserted) override;
    void setHealthy(bool level) override;
    void setEventCode(uint8_t code, bool strobe) override;
    void setReference(RefChannel ch, float normalized) override;
    void sendPacket(const uint8_t* data, size_t len) override;
    void service() override;   // drain the APF UART ring (called from the RT loop)

private:
    ByteRing<1024> apf_tx_;
    uint16_t pwm_wrap_ = 4095;
};

class PicoAlarmHw : public AlarmHw {
public:
    void init() override;
    void setStatusLed(bool on) override;
    void setAlarm(bool on) override;
};

// Configuration in the last 4 KB sector of flash. Writes go through
// flash_safe_execute(), which parks core1 (multicore lockout) for the few ms of
// the erase/program; the resulting sample gap is detected and flagged by the
// acquisition layer like any other gap.
class FlashConfigStore : public ConfigStore {
public:
    bool load(StoredConfig& out) override;
    bool save(const StoredConfig& in) override;
    bool erase() override;
};

}  // namespace pq
