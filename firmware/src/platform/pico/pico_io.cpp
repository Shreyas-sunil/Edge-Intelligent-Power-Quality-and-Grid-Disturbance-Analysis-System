#include "platform/pico/pico_io.h"

#include <cstring>
#include <initializer_list>

#include "hardware/flash.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/spi.h"
#include "hardware/uart.h"
#include "pico/flash.h"
#include "pico/stdio/driver.h"
#include "pico/stdio_usb.h"
#include "pico/stdlib.h"
#include "platform/pico/board_pins.h"
#include "tusb.h"

namespace pq {

// ------------------------------------------------------------------- pacer
void PicoSimPacer::start(uint32_t block_period_us) {
    period_ = block_period_us ? block_period_us : 1;
    t0_ = time_us_64();
}

uint32_t PicoSimPacer::blocksDue() { return static_cast<uint32_t>((time_us_64() - t0_) / period_); }

// ------------------------------------------------------------------- USB CDC
void UsbTransport::init() { stdio_usb_init(); }

bool UsbTransport::connected() const { return stdio_usb_connected(); }

size_t UsbTransport::read(char* buf, size_t max) {
    const int n = stdio_usb.in_chars(buf, static_cast<int>(max));
    return n > 0 ? static_cast<size_t>(n) : 0;
}

size_t UsbTransport::write(const char* data, size_t len) {
    const size_t n = len < tx_.free() ? len : tx_.free();
    tx_.put(data, n);
    return n;
}

void UsbTransport::flush() {
    if (!stdio_usb_connected()) {
        tx_.drop(tx_.used());   // nobody listening: do not accumulate stale data
        return;
    }
    // Only hand TinyUSB as many bytes as it can take right now, so the stdio
    // driver never enters its blocking wait.
    for (int guard = 0; guard < 4 && tx_.used(); ++guard) {
        const uint32_t room = tud_cdc_write_available();
        if (!room) break;
        const uint8_t* p;
        size_t n = tx_.span(p);
        if (n > room) n = room;
        stdio_usb.out_chars(reinterpret_cast<const char*>(p), static_cast<int>(n));
        tx_.drop(n);
    }
}

// ------------------------------------------------------------------- UART
void UartTransport::init(uint32_t baud) {
    uart_init(PQ_HOST_UART, baud);
    gpio_set_function(PQ_PIN_HOST_UART_TX, GPIO_FUNC_UART);
    gpio_set_function(PQ_PIN_HOST_UART_RX, GPIO_FUNC_UART);
    uart_set_fifo_enabled(PQ_HOST_UART, true);
    inited_ = true;
}

size_t UartTransport::read(char* buf, size_t max) {
    size_t n = 0;
    while (n < max && uart_is_readable(PQ_HOST_UART)) buf[n++] = static_cast<char>(uart_getc(PQ_HOST_UART));
    return n;
}

size_t UartTransport::write(const char* data, size_t len) {
    const size_t n = len < tx_.free() ? len : tx_.free();
    tx_.put(data, n);
    return n;
}

void UartTransport::flush() {
    while (tx_.used() && uart_is_writable(PQ_HOST_UART)) {
        const uint8_t* p;
        tx_.span(p);
        uart_get_hw(PQ_HOST_UART)->dr = *p;
        tx_.drop(1);
    }
}

// ------------------------------------------------------------------- control
void PicoControlHw::init() {
    const uint outs[] = {PQ_PIN_TRIP, PQ_PIN_HEALTHY, PQ_PIN_EVENT_STROBE, PQ_PIN_EVENT_BIT0, PQ_PIN_EVENT_BIT0 + 1,
                         PQ_PIN_EVENT_BIT0 + 2, PQ_PIN_EVENT_BIT0 + 3};
    for (uint p : outs) {
        gpio_init(p);
        gpio_set_dir(p, GPIO_OUT);
        gpio_put(p, 0);
    }
    // 12-bit PWM references (RC-filter to an analog level); mid-scale = 0 pu.
    for (uint pin : {static_cast<uint>(PQ_PIN_PWM_DVR), static_cast<uint>(PQ_PIN_PWM_APF)}) {
        gpio_set_function(pin, GPIO_FUNC_PWM);
        const uint slice = pwm_gpio_to_slice_num(pin);
        pwm_set_wrap(slice, pwm_wrap_);
        pwm_set_chan_level(slice, pwm_gpio_to_channel(pin), 0);
        pwm_set_enabled(slice, true);
    }
    uart_init(PQ_APF_UART, PQ_APF_UART_BAUD);
    gpio_set_function(PQ_PIN_APF_UART_TX, GPIO_FUNC_UART);
    gpio_set_function(PQ_PIN_APF_UART_RX, GPIO_FUNC_UART);
#if PQ_USE_MCP4822
    spi_init(PQ_DAC_SPI, 10000000);
    spi_set_format(PQ_DAC_SPI, 16, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    gpio_set_function(PQ_PIN_DAC_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PQ_PIN_DAC_MOSI, GPIO_FUNC_SPI);
    gpio_init(PQ_PIN_DAC_CS);
    gpio_set_dir(PQ_PIN_DAC_CS, GPIO_OUT);
    gpio_put(PQ_PIN_DAC_CS, 1);
#endif
}

void PicoControlHw::setTripLine(bool a) { gpio_put(PQ_PIN_TRIP, a); }
void PicoControlHw::setHealthy(bool l) { gpio_put(PQ_PIN_HEALTHY, l); }

void PicoControlHw::setEventCode(uint8_t code, bool strobe) {
    gpio_put_masked(0xFu << PQ_PIN_EVENT_BIT0, static_cast<uint32_t>(code & 0xFu) << PQ_PIN_EVENT_BIT0);
    gpio_put(PQ_PIN_EVENT_STROBE, strobe);
}

void PicoControlHw::setReference(RefChannel ch, float v) {
    // DVR: -1..+1 -> 0..wrap (mid-scale = 0). APF: 0..1 -> 0..wrap.
    const float x = ch == RefChannel::DVR ? 0.5f * (v + 1.0f) : v;
    const float c = x < 0.0f ? 0.0f : (x > 1.0f ? 1.0f : x);
    const uint16_t level = static_cast<uint16_t>(c * pwm_wrap_ + 0.5f);
    const uint pin = ch == RefChannel::DVR ? PQ_PIN_PWM_DVR : PQ_PIN_PWM_APF;
    pwm_set_chan_level(pwm_gpio_to_slice_num(pin), pwm_gpio_to_channel(pin), level);
#if PQ_USE_MCP4822
    // MCP4822: A = DVR, B = APF; gain 2x (0..4.096 V), active.
    const uint16_t cmd = static_cast<uint16_t>((ch == RefChannel::APF ? 0x8000u : 0u) | 0x1000u | (level & 0x0FFFu));
    gpio_put(PQ_PIN_DAC_CS, 0);
    spi_write16_blocking(PQ_DAC_SPI, &cmd, 1);
    gpio_put(PQ_PIN_DAC_CS, 1);
#endif
}

void PicoControlHw::sendPacket(const uint8_t* data, size_t len) {
    if (apf_tx_.free() < len) return;   // link slower than the frame rate: drop whole packet
    apf_tx_.put(reinterpret_cast<const char*>(data), len);
    service();
}

void PicoControlHw::service() {
    while (apf_tx_.used() && uart_is_writable(PQ_APF_UART)) {
        const uint8_t* p;
        apf_tx_.span(p);
        uart_get_hw(PQ_APF_UART)->dr = *p;
        apf_tx_.drop(1);
    }
}

// ------------------------------------------------------------------- alarm
void PicoAlarmHw::init() {
    gpio_init(PQ_PIN_STATUS_LED);
    gpio_set_dir(PQ_PIN_STATUS_LED, GPIO_OUT);
    gpio_init(PQ_PIN_ALARM);
    gpio_set_dir(PQ_PIN_ALARM, GPIO_OUT);
}
void PicoAlarmHw::setStatusLed(bool on) { gpio_put(PQ_PIN_STATUS_LED, on); }
void PicoAlarmHw::setAlarm(bool on) { gpio_put(PQ_PIN_ALARM, on); }

// ------------------------------------------------------------------- flash
namespace {
constexpr uint32_t kCfgOffset = PICO_FLASH_SIZE_BYTES - FLASH_SECTOR_SIZE;
static_assert(sizeof(StoredConfig) <= FLASH_SECTOR_SIZE, "config must fit one sector");

struct FlashJob {
    const uint8_t* data;
    size_t len;
    bool erase_only;
};

void __no_inline_not_in_flash_func(flashWorker)(void* p) {
    const FlashJob* j = static_cast<const FlashJob*>(p);
    flash_range_erase(kCfgOffset, FLASH_SECTOR_SIZE);
    if (!j->erase_only) flash_range_program(kCfgOffset, j->data, j->len);
}

alignas(4) uint8_t g_page_buf[((sizeof(StoredConfig) + FLASH_PAGE_SIZE - 1) / FLASH_PAGE_SIZE) * FLASH_PAGE_SIZE];
}  // namespace

bool FlashConfigStore::load(StoredConfig& out) {
    const auto* p = reinterpret_cast<const uint8_t*>(XIP_BASE + kCfgOffset);
    std::memcpy(&out, p, sizeof(out));
    return out.magic == kConfigMagic;
}

bool FlashConfigStore::save(const StoredConfig& in) {
    std::memset(g_page_buf, 0xFF, sizeof(g_page_buf));
    std::memcpy(g_page_buf, &in, sizeof(in));
    FlashJob j{g_page_buf, sizeof(g_page_buf), false};
    if (flash_safe_execute(flashWorker, &j, 500) != PICO_OK) return false;
    return std::memcmp(reinterpret_cast<const void*>(XIP_BASE + kCfgOffset), &in, sizeof(in)) == 0;
}

bool FlashConfigStore::erase() {
    FlashJob j{nullptr, 0, true};
    return flash_safe_execute(flashWorker, &j, 500) == PICO_OK;
}

}  // namespace pq
