#include "adc/ads131m02_protocol.h"

#include <cmath>
#include <cstring>

#include "common/math_util.h"

namespace pq::ads131m02 {

OsrChoice chooseOsr(float requested_hz, uint32_t clkin_hz) {
    static const uint16_t kOsr[8] = {128, 256, 512, 1024, 2048, 4096, 8192, 16256};
    OsrChoice best{0, 128, 0.0f};
    float best_err = 1e30f;
    for (uint8_t c = 0; c < 8; ++c) {
        const float odr = static_cast<float>(clkin_hz) / 2.0f / kOsr[c];
        const float err = std::fabs(std::log(odr / requested_hz));
        if (err < best_err) {
            best_err = err;
            best = OsrChoice{c, kOsr[c], odr};
        }
    }
    return best;
}

uint16_t clockReg(uint8_t osr_code) {
    // CH1_EN | CH0_EN | OSR[2:0] | PWR = 10b (high resolution)
    return static_cast<uint16_t>(0x0300u | ((osr_code & 7u) << 2) | 0x2u);
}

uint8_t gainCode(uint8_t gain) {
    uint8_t c = 0;
    while (c < 7 && (1u << c) < gain) ++c;
    return c;
}

uint16_t gainReg(uint8_t g0, uint8_t g1) {
    return static_cast<uint16_t>((gainCode(g1) << 4) | gainCode(g0));
}

size_t encodeCommand(uint16_t cmd, uint8_t* out, size_t cap, const uint16_t* data, int ndata) {
    const size_t words = (1 + ndata) > kFrameWords ? static_cast<size_t>(1 + ndata) : kFrameWords;
    const size_t len = words * kWordBytes;
    if (len > cap) return 0;
    std::memset(out, 0, len);
    out[0] = static_cast<uint8_t>(cmd >> 8);
    out[1] = static_cast<uint8_t>(cmd);
    for (int k = 0; k < ndata; ++k) {
        out[(k + 1) * kWordBytes] = static_cast<uint8_t>(data[k] >> 8);
        out[(k + 1) * kWordBytes + 1] = static_cast<uint8_t>(data[k]);
    }
    return len;
}

Frame decodeFrame(const uint8_t* b) {
    Frame f{};
    f.status = static_cast<uint16_t>((b[0] << 8) | b[1]);
    for (int c = 0; c < 2; ++c) {
        const uint8_t* p = b + (c + 1) * kWordBytes;
        f.ch[c] = signExtend24((static_cast<uint32_t>(p[0]) << 16) | (static_cast<uint32_t>(p[1]) << 8) | p[2]);
    }
    const uint8_t* cw = b + 3 * kWordBytes;
    f.crc_rx = static_cast<uint16_t>((cw[0] << 8) | cw[1]);
    // Output CRC covers every preceding word of the frame (CCITT, seed 0xFFFF).
    f.crc_calc = crc16Ccitt(b, 3 * kWordBytes, 0xFFFF);
    f.crc_ok = f.crc_rx == f.crc_calc;
    f.status_ok = (f.status & ST_WLENGTH_MASK) == ST_WLENGTH_24 && (f.status & (ST_DRDY0 | ST_DRDY1)) ==
                                                                        (ST_DRDY0 | ST_DRDY1);
    f.saturated = f.ch[0] >= kFullScalePos || f.ch[0] <= kFullScaleNeg || f.ch[1] >= kFullScalePos ||
                  f.ch[1] <= kFullScaleNeg;
    return f;
}

void encodeFrame(uint16_t status, int32_t ch0, int32_t ch1, uint8_t* out) {
    std::memset(out, 0, kFrameBytes);
    out[0] = static_cast<uint8_t>(status >> 8);
    out[1] = static_cast<uint8_t>(status);
    const int32_t ch[2] = {ch0, ch1};
    for (int c = 0; c < 2; ++c) {
        const uint32_t v = static_cast<uint32_t>(ch[c]) & 0xFFFFFFu;
        uint8_t* p = out + (c + 1) * kWordBytes;
        p[0] = static_cast<uint8_t>(v >> 16);
        p[1] = static_cast<uint8_t>(v >> 8);
        p[2] = static_cast<uint8_t>(v);
    }
    const uint16_t crc = crc16Ccitt(out, 3 * kWordBytes, 0xFFFF);
    out[9] = static_cast<uint8_t>(crc >> 8);
    out[10] = static_cast<uint8_t>(crc);
}

}  // namespace pq::ads131m02
