#include "platform/pico/ads131m02_pico.h"

#include <cstring>

#include "adc/ads131m02_protocol.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/pio.h"
#include "hardware/pio_instructions.h"
#include "hardware/pwm.h"
#include "hardware/spi.h"
#include "pico/stdlib.h"
#include "platform/pico/board_pins.h"

namespace pq {

namespace A = ads131m02;

namespace {
constexpr uint32_t kRing = PQ_ACQ_RING_BLOCKS;
static_assert((kRing & (kRing - 1)) == 0 && kRing <= 16, "ring must be a power of two <= 16 (DMA ring on table)");

// DMA destination ring and the table of block addresses read by the control
// channel (the table uses the DMA read-address ring feature -> must be
// aligned to its size).
alignas(4) uint32_t g_ring[kRing][3 * kBlockSize];
alignas(64) uint32_t* g_addr_table[16];

volatile uint32_t g_blocks_done = 0;              // written only by the DMA IRQ
volatile uint64_t g_block_time_us[kRing];         // completion time of each slot
int g_irq_dma_ch = -1;
}  // namespace

// ------------------------------------------------------------------ IRQ
void __not_in_flash_func(Ads131m02Backend::dmaIrqHandler)() {
    const uint32_t mask = 1u << g_irq_dma_ch;
    if (dma_hw->ints1 & mask) {
        dma_hw->ints1 = mask;
        const uint32_t n = g_blocks_done;
        g_block_time_us[n & (kRing - 1)] = time_us_64();
        g_blocks_done = n + 1;
    }
}

// -------------------------------------------------------------- SPI (config)
void Ads131m02Backend::pinsToSpi() {
    gpio_set_function(PQ_PIN_ADC_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PQ_PIN_ADC_SCLK, GPIO_FUNC_SPI);
    gpio_set_function(PQ_PIN_ADC_MOSI, GPIO_FUNC_SPI);
    gpio_init(PQ_PIN_ADC_CS);
    gpio_set_dir(PQ_PIN_ADC_CS, GPIO_OUT);
    gpio_put(PQ_PIN_ADC_CS, 1);
}

AdcStatus Ads131m02Backend::spiFrame(uint16_t cmd, const uint16_t* data, int ndata, uint16_t* reply0) {
    uint8_t tx[A::kFrameBytes + 12], rx[A::kFrameBytes + 12];
    const size_t n = A::encodeCommand(cmd, tx, sizeof(tx), data, ndata);
    if (!n) return AdcStatus::InvalidParam;
    gpio_put(PQ_PIN_ADC_CS, 0);
    busy_wait_us_32(1);
    const int got = spi_write_read_blocking(PQ_ADC_SPI, tx, rx, n);
    busy_wait_us_32(1);
    gpio_put(PQ_PIN_ADC_CS, 1);
    busy_wait_us_32(2);
    if (got != static_cast<int>(n)) return AdcStatus::CommFailure;
    if (reply0) *reply0 = static_cast<uint16_t>((rx[0] << 8) | rx[1]);
    return AdcStatus::Ok;
}

AdcStatus Ads131m02Backend::readReg(uint8_t reg, uint16_t& val) {
    // The response to a command appears in the FIRST word of the NEXT frame.
    AdcStatus s = spiFrame(A::cmdRreg(reg), nullptr, 0, nullptr);
    if (s != AdcStatus::Ok) return s;
    return spiFrame(A::CMD_NULL, nullptr, 0, &val);
}

AdcStatus Ads131m02Backend::writeRegChecked(uint8_t reg, uint16_t val) {
    uint16_t ack = 0;
    AdcStatus s = spiFrame(A::cmdWreg(reg), &val, 1, nullptr);
    if (s != AdcStatus::Ok) return s;
    s = spiFrame(A::CMD_NULL, nullptr, 0, &ack);
    if (s != AdcStatus::Ok) return s;
    if (ack != A::wregAck(reg)) return AdcStatus::CommFailure;
    uint16_t back = 0;
    s = readReg(reg, back);
    if (s != AdcStatus::Ok) return s;
    return back == val ? AdcStatus::Ok : AdcStatus::ConfigMismatch;
}

bool Ads131m02Backend::startClkin() {
    const uint32_t sys = clock_get_hz(clk_sys);
    uint32_t div = (sys + A::kClkinHz / 2) / A::kClkinHz;   // integer divider -> no jitter
    if (div < 2) div = 2;
    clkin_hz_ = sys / div;
    const uint slice = pwm_gpio_to_slice_num(PQ_PIN_ADC_CLKIN);
    const uint chan = pwm_gpio_to_channel(PQ_PIN_ADC_CLKIN);
    gpio_set_function(PQ_PIN_ADC_CLKIN, GPIO_FUNC_PWM);
    pwm_config c = pwm_get_default_config();
    pwm_config_set_clkdiv_int(&c, 1);
    pwm_config_set_wrap(&c, div - 1);
    pwm_init(slice, &c, false);
    pwm_set_chan_level(slice, chan, div / 2);
    pwm_set_enabled(slice, true);
    // Exact only when clk_sys is a multiple of 5.12 MHz (main.cpp sets it so).
    return sys % A::kClkinHz == 0;
}

AdcStatus Ads131m02Backend::init(const AdcHwConfig& cfg) {
    stop();
    cfg_ = cfg;
    std::memset(&diag_, 0, sizeof(diag_));
    inited_ = false;

    const bool exact = startClkin();
    const A::OsrChoice osr = A::chooseOsr(cfg.sample_rate_hz, clkin_hz_);
    odr_ = static_cast<float>(clkin_hz_) / 2.0f / osr.osr;
    (void)exact;

    // Hardware reset: SYNC/RESET low for > 2048 CLKIN periods (400 us @ 5.12 MHz).
    gpio_init(PQ_PIN_ADC_RESET);
    gpio_set_dir(PQ_PIN_ADC_RESET, GPIO_OUT);
    gpio_put(PQ_PIN_ADC_RESET, 0);
    sleep_us(1000);
    gpio_put(PQ_PIN_ADC_RESET, 1);
    sleep_ms(2);

    gpio_init(PQ_PIN_ADC_DRDY);
    gpio_set_dir(PQ_PIN_ADC_DRDY, GPIO_IN);
    gpio_pull_up(PQ_PIN_ADC_DRDY);

    spi_init(PQ_ADC_SPI, 4000000);
    spi_set_format(PQ_ADC_SPI, 8, SPI_CPOL_0, SPI_CPHA_1, SPI_MSB_FIRST);   // SPI mode 1
    pinsToSpi();

    // Flush the power-on frame, then identify the device.
    spiFrame(A::CMD_NULL, nullptr, 0, nullptr);
    AdcStatus s = readReg(A::REG_ID, id_);
    if (s != AdcStatus::Ok) return diag_.last_error = s;
    if ((id_ & 0x0F00u) != 0x0200u) {   // CHANCNT field must read 2 (ADS131M02)
        diag_.last_error = (id_ == 0x0000 || id_ == 0xFFFF) ? AdcStatus::CommFailure : AdcStatus::BadId;
        return diag_.last_error;
    }
    if ((s = writeRegChecked(A::REG_MODE, A::kModeReg)) != AdcStatus::Ok) return diag_.last_error = s;
    if ((s = writeRegChecked(A::REG_GAIN, A::gainReg(cfg.pga_gain[0], cfg.pga_gain[1]))) != AdcStatus::Ok)
        return diag_.last_error = s;
    if ((s = writeRegChecked(A::REG_CLOCK, A::clockReg(osr.code))) != AdcStatus::Ok) return diag_.last_error = s;

    // DMA channels + PIO state machine (claimed once, kept across restarts).
    if (dma_data_ < 0) {
        dma_data_ = dma_claim_unused_channel(true);
        dma_ctrl_ = dma_claim_unused_channel(true);
    }
    if (sm_ < 0) {
        buildPioProgram();
        pio_program_t prog{};
        prog.instructions = prog_;
        prog.length = 8;
        prog.origin = -1;
        uint sm = 0, off = 0;
        if (!pio_claim_free_sm_and_add_program(&prog, &pio_, &sm, &off)) return diag_.last_error = AdcStatus::Unsupported;
        sm_ = static_cast<int>(sm);
        prog_offset_ = static_cast<int>(off);
    }
    inited_ = true;
    diag_.last_error = AdcStatus::Ok;
    return AdcStatus::Ok;
}

// PIO program (side-set 1 bit = SCLK, `set` pin = CS, `in` pin = MISO):
//   0: set pins,1        side 0      ; CS high (idle)
//   1: wait 1 gpio DRDY  side 0
//   2: wait 0 gpio DRDY  side 0      ; falling edge = new conversion
//   3: set pins,0        side 0 [1]  ; CS low + setup time
//   4: mov x, y          side 0      ; y = 95 (bits per frame - 1)
//   5: nop               side 1 [2]  ; SCLK high: ADC shifts DOUT on the rising edge
//   6: in pins,1         side 1      ; sample late in the high phase
//   7: jmp x-- 5         side 0 [3]  ; SCLK low; wrap -> 0 when x == 0
void Ads131m02Backend::buildPioProgram() {
    const uint ss0 = pio_encode_sideset(1, 0), ss1 = pio_encode_sideset(1, 1);
    prog_[0] = static_cast<uint16_t>(pio_encode_set(pio_pins, 1) | ss0);
    prog_[1] = static_cast<uint16_t>(pio_encode_wait_gpio(true, PQ_PIN_ADC_DRDY) | ss0);
    prog_[2] = static_cast<uint16_t>(pio_encode_wait_gpio(false, PQ_PIN_ADC_DRDY) | ss0);
    prog_[3] = static_cast<uint16_t>(pio_encode_set(pio_pins, 0) | ss0 | pio_encode_delay(1));
    prog_[4] = static_cast<uint16_t>(pio_encode_mov(pio_x, pio_y) | ss0);
    prog_[5] = static_cast<uint16_t>(pio_encode_nop() | ss1 | pio_encode_delay(2));
    prog_[6] = static_cast<uint16_t>(pio_encode_in(pio_pins, 1) | ss1);
    prog_[7] = static_cast<uint16_t>(pio_encode_jmp_x_dec(5) | ss0 | pio_encode_delay(3));
}

void Ads131m02Backend::pinsToPio() {
    // MOSI stays low (NULL command) as a plain GPIO while streaming.
    gpio_init(PQ_PIN_ADC_MOSI);
    gpio_set_dir(PQ_PIN_ADC_MOSI, GPIO_OUT);
    gpio_put(PQ_PIN_ADC_MOSI, 0);
    pio_gpio_init(pio_, PQ_PIN_ADC_SCLK);
    pio_gpio_init(pio_, PQ_PIN_ADC_CS);
    pio_gpio_init(pio_, PQ_PIN_ADC_MISO);
    pio_sm_set_consecutive_pindirs(pio_, sm_, PQ_PIN_ADC_SCLK, 1, true);
    pio_sm_set_consecutive_pindirs(pio_, sm_, PQ_PIN_ADC_CS, 1, true);
    pio_sm_set_consecutive_pindirs(pio_, sm_, PQ_PIN_ADC_MISO, 1, false);
}

AdcStatus Ads131m02Backend::start() {
    if (!inited_) return AdcStatus::NotInitialized;
    pinsToPio();

    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, prog_offset_ + 0, prog_offset_ + 7);
    sm_config_set_sideset(&c, 1, false, false);
    sm_config_set_sideset_pins(&c, PQ_PIN_ADC_SCLK);
    sm_config_set_set_pins(&c, PQ_PIN_ADC_CS, 1);
    sm_config_set_in_pins(&c, PQ_PIN_ADC_MISO);
    sm_config_set_in_shift(&c, false, true, 32);          // MSB first, autopush 32 bits
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_RX);        // 8-deep RX FIFO
    // 8 PIO cycles per bit (nop[2] = 3, in = 1, jmp[3] = 4) -> 8 MHz SCLK
    // (ADS131M02 allows 25 MHz; 96 bits take 12 us of the 100 us sample period).
    const float div = static_cast<float>(clock_get_hz(clk_sys)) / (8.0f * 8000000.0f);
    sm_config_set_clkdiv(&c, div < 1.0f ? 1.0f : div);
    pio_sm_init(pio_, sm_, prog_offset_, &c);
    pio_sm_put_blocking(pio_, sm_, A::kFrameWords * 24 - 1);   // y = 95
    pio_sm_exec(pio_, sm_, pio_encode_pull(false, false));
    pio_sm_exec(pio_, sm_, pio_encode_mov(pio_y, pio_osr));

    // DMA ring: data channel PIO RX FIFO -> ring block, then chains to the control
    // channel which writes the next block address into the data channel's
    // write-address trigger register.
    for (uint32_t k = 0; k < 16; ++k) g_addr_table[k] = g_ring[k & (kRing - 1)];
    g_blocks_done = 0;
    consumed_ = 0;
    gap_pending_ = false;

    dma_channel_config dc = dma_channel_get_default_config(dma_data_);
    channel_config_set_transfer_data_size(&dc, DMA_SIZE_32);
    channel_config_set_read_increment(&dc, false);
    channel_config_set_write_increment(&dc, true);
    channel_config_set_dreq(&dc, pio_get_dreq(pio_, sm_, false));
    channel_config_set_chain_to(&dc, dma_ctrl_);
    dma_channel_configure(dma_data_, &dc, g_ring[0], &pio_->rxf[sm_], kBlockWords, false);

    dma_channel_config cc = dma_channel_get_default_config(dma_ctrl_);
    channel_config_set_transfer_data_size(&cc, DMA_SIZE_32);
    channel_config_set_read_increment(&cc, true);
    channel_config_set_write_increment(&cc, false);
    channel_config_set_ring(&cc, false, 6);                 // wrap reads on the 64-byte table
    dma_channel_configure(dma_ctrl_, &cc, &dma_hw->ch[dma_data_].al2_write_addr_trig, &g_addr_table[1], 1,
                          false);

    g_irq_dma_ch = dma_data_;
    irq_set_exclusive_handler(DMA_IRQ_1, &Ads131m02Backend::dmaIrqHandler);
    dma_channel_set_irq1_enabled(dma_data_, true);
    irq_set_enabled(DMA_IRQ_1, true);   // enabled on the calling core (core1)

    dma_channel_start(dma_data_);
    t_start_us_ = time_us_64();
    pio_sm_set_enabled(pio_, sm_, true);
    running_ = true;
    return AdcStatus::Ok;
}

void Ads131m02Backend::stop() {
    if (!running_) return;
    pio_sm_set_enabled(pio_, sm_, false);
    dma_channel_set_irq1_enabled(dma_data_, false);
    // Abort both: clear chaining first so an abort cannot re-trigger the other.
    hw_clear_bits(&dma_hw->ch[dma_data_].al1_ctrl, DMA_CH0_CTRL_TRIG_EN_BITS);
    dma_channel_abort(dma_ctrl_);
    dma_channel_abort(dma_data_);
    pio_sm_clear_fifos(pio_, sm_);
    pio_sm_restart(pio_, sm_);
    pinsToSpi();
    gpio_set_function(PQ_PIN_ADC_MOSI, GPIO_FUNC_SPI);
    running_ = false;
}

bool Ads131m02Backend::readBlock(RawBlock& out) {
    if (!running_) return false;
    const uint32_t done = g_blocks_done;
    if (done == consumed_) return false;
    uint32_t behind = done - consumed_;
    if (behind >= kRing) {   // slot about to be (or being) overwritten by DMA
        const uint32_t lost = behind - 1;
        diag_.overruns += lost;
        diag_.missing_samples += static_cast<uint64_t>(lost) * kBlockSize;
        diag_.last_error = AdcStatus::Overrun;
        consumed_ += lost;
        gap_pending_ = true;
        behind = 1;
    }
    if (behind > diag_.ring_high_water) diag_.ring_high_water = behind;

    const uint32_t slot = consumed_ & (kRing - 1);
    // Missing-sample check: consecutive block completions must be one block
    // period apart (ADC clock is derived from clk_sys, so +/- IRQ latency only).
    if (consumed_ > 0) {
        const uint64_t dt = g_block_time_us[slot] - g_block_time_us[(slot - 1) & (kRing - 1)];
        const uint64_t period = static_cast<uint64_t>(kBlockSize * 1e6f / odr_);
        if (dt > period + period / 2 && !gap_pending_) {
            diag_.missing_samples += (dt - period) * static_cast<uint64_t>(odr_) / 1000000u;
            gap_pending_ = true;
        }
    }

    const uint32_t* w = g_ring[slot];
    std::memset(out.bad, 0, sizeof(out.bad));
    out.bad_count = out.sat_count = 0;
    out.seq = static_cast<uint64_t>(consumed_) * kBlockSize;
    out.t_first_us = t_start_us_ + static_cast<uint64_t>(static_cast<double>(out.seq) * 1e6 / odr_);
    out.n = kBlockSize;
    out.flags = gap_pending_ ? RB_GAP_BEFORE : 0;
    gap_pending_ = false;
    for (uint32_t k = 0; k < kBlockSize; ++k) {
        uint8_t b[A::kFrameBytes];
        for (int j = 0; j < kWordsPerSample; ++j) {
            const uint32_t v = w[k * kWordsPerSample + j];
            b[4 * j + 0] = static_cast<uint8_t>(v >> 24);
            b[4 * j + 1] = static_cast<uint8_t>(v >> 16);
            b[4 * j + 2] = static_cast<uint8_t>(v >> 8);
            b[4 * j + 3] = static_cast<uint8_t>(v);
        }
        const A::Frame f = A::decodeFrame(b);
        out.ch[kChVoltage][k] = f.ch[0];
        out.ch[kChCurrent][k] = f.ch[1];
        const bool crc_bad = cfg_.crc_check && !f.crc_ok;
        if (crc_bad) ++diag_.crc_errors;
        if (!f.status_ok) ++diag_.status_errors;
        if (crc_bad || !f.status_ok) {
            out.markBad(k);
            ++out.bad_count;
        }
        if (f.saturated) ++out.sat_count;
    }
    if (out.bad_count) out.flags |= RB_HAS_BAD;
    if (out.sat_count) {
        out.flags |= RB_SATURATED;
        diag_.saturated += out.sat_count;
    }
    // If the DMA lapped us while decoding, the data may be torn.
    if (g_blocks_done - consumed_ >= kRing) {
        out.flags |= RB_HAS_BAD;
        gap_pending_ = true;
    }
    ++consumed_;
    ++diag_.blocks;
    return true;
}

AdcStatus Ads131m02Backend::poll() {
    if (!running_) return inited_ ? AdcStatus::Ok : AdcStatus::NotInitialized;
    const uint32_t done = g_blocks_done;
    const uint64_t now = time_us_64();
    const uint64_t period = static_cast<uint64_t>(kBlockSize * 1e6f / odr_);
    const uint64_t last = done ? g_block_time_us[(done - 1) & (kRing - 1)] : t_start_us_;
    if (now > last + 3 * period + 2000) {   // DRDY stopped (CLKIN lost, ADC reset, wiring)
        ++diag_.timeouts;
        diag_.last_error = AdcStatus::Timeout;
        return AdcStatus::Timeout;
    }
    return AdcStatus::Ok;
}

}  // namespace pq
