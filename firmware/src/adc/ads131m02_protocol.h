// =============================================================================
//  TI ADS131M02 (2-ch, 24-bit, simultaneous-sampling delta-sigma ADC) —
//  device protocol, kept free of any Pico SDK dependency so it is unit-tested
//  on the host. The Pico transport (SPI config + PIO/DMA streaming) lives in
//  src/platform/pico/ads131m02_pico.cpp.
//
//  Why this ADC: two simultaneously sampled channels (V and I share the same
//  sampling instant -> no inter-channel skew), 24-bit, PGA per channel,
//  frame CRC, and an output data rate derived from CLKIN that the RP2 can
//  generate exactly (CLKIN = 5.12 MHz, OSR 256 -> 10.000 kS/s).
//
//  Frame (24-bit words, MSB first): STATUS | CH0 | CH1 | CRC  = 12 bytes.
// =============================================================================
#pragma once
#include <cstddef>
#include <cstdint>

namespace pq::ads131m02 {

// Registers
enum Reg : uint8_t {
    REG_ID = 0x00,
    REG_STATUS = 0x01,
    REG_MODE = 0x02,
    REG_CLOCK = 0x03,
    REG_GAIN = 0x04,
    REG_CFG = 0x06,
    REG_CH0_CFG = 0x09,
    REG_CH1_CFG = 0x0E,
};

// Commands (16-bit, sent left-aligned in a 24-bit word)
constexpr uint16_t CMD_NULL = 0x0000;
constexpr uint16_t CMD_RESET = 0x0011;
constexpr uint16_t CMD_STANDBY = 0x0022;
constexpr uint16_t CMD_WAKEUP = 0x0033;
constexpr uint16_t CMD_LOCK = 0x0555;
constexpr uint16_t CMD_UNLOCK = 0x0655;
constexpr uint16_t RESET_ACK = 0xFF22;          // response word after RESET (M02)
inline uint16_t cmdRreg(uint8_t addr, uint8_t n = 1) {
    return static_cast<uint16_t>(0xA000u | ((addr & 0x3Fu) << 7) | ((n - 1u) & 0x7Fu));
}
inline uint16_t cmdWreg(uint8_t addr, uint8_t n = 1) {
    return static_cast<uint16_t>(0x6000u | ((addr & 0x3Fu) << 7) | ((n - 1u) & 0x7Fu));
}
inline uint16_t wregAck(uint8_t addr, uint8_t n = 1) {
    return static_cast<uint16_t>(0x4000u | ((addr & 0x3Fu) << 7) | ((n - 1u) & 0x7Fu));
}

constexpr int      kWordBytes = 3;
constexpr int      kFrameWords = 4;                 // status, ch0, ch1, crc
constexpr int      kFrameBytes = kWordBytes * kFrameWords;
constexpr int32_t  kFullScalePos = 0x7FFFFF;
constexpr int32_t  kFullScaleNeg = -0x800000;
constexpr float    kVref = 1.2f;                    // internal reference, FS = +/-1.2 V / gain

// STATUS bits
constexpr uint16_t ST_LOCK = 1u << 15;
constexpr uint16_t ST_RESYNC = 1u << 14;
constexpr uint16_t ST_REG_MAP = 1u << 13;
constexpr uint16_t ST_CRC_ERR = 1u << 12;
constexpr uint16_t ST_RESET = 1u << 10;
constexpr uint16_t ST_WLENGTH_MASK = 3u << 8;
constexpr uint16_t ST_WLENGTH_24 = 1u << 8;
constexpr uint16_t ST_DRDY1 = 1u << 1;
constexpr uint16_t ST_DRDY0 = 1u << 0;

// Clocking: ODR = CLKIN / 2 / OSR
constexpr uint32_t kClkinHz = 5120000;
struct OsrChoice {
    uint8_t  code;   // CLOCK[4:2]
    uint16_t osr;
    float    odr;
};
OsrChoice chooseOsr(float requested_hz, uint32_t clkin_hz = kClkinHz);

uint16_t clockReg(uint8_t osr_code);                       // both channels on, high-resolution mode
uint16_t gainReg(uint8_t gain_ch0, uint8_t gain_ch1);      // gains 1..128 (powers of two)
uint8_t  gainCode(uint8_t gain);                           // 1->0 ... 128->7
constexpr uint16_t kModeReg = 0x0510;                      // 24-bit words, SPI timeout on, DRDY low-active

// Encode a command frame: cmd word (+ optional data word), zero padded to the
// full frame length so the ADC sees a complete frame.
size_t encodeCommand(uint16_t cmd, uint8_t* out, size_t cap, const uint16_t* data = nullptr, int ndata = 0);

struct Frame {
    uint16_t status;
    int32_t  ch[2];
    uint16_t crc_rx;
    uint16_t crc_calc;
    bool     crc_ok;
    bool     status_ok;     // word length 24 bit and both DRDY bits set
    bool     saturated;
};

inline int32_t signExtend24(uint32_t v) { return static_cast<int32_t>(v << 8) >> 8; }
Frame decodeFrame(const uint8_t* b);                        // kFrameBytes bytes
// Build a valid frame (for tests / the loop-back self-test).
void  encodeFrame(uint16_t status, int32_t ch0, int32_t ch1, uint8_t* out);

}  // namespace pq::ads131m02
