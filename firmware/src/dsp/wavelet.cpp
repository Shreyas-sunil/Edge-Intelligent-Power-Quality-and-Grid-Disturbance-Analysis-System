#include "dsp/wavelet.h"

#include <cstring>

namespace pq {

namespace {
// Decomposition filters (PyWavelets conventions).
const float kHaarLo[2] = {0.70710678118654752f, 0.70710678118654752f};
const float kHaarHi[2] = {-0.70710678118654752f, 0.70710678118654752f};
const float kDb4Lo[8] = {-0.010597401784997278f, 0.032883011666982945f, 0.030841381835986965f,
                         -0.18703481171888114f,  -0.02798376941698385f, 0.6308807679295904f,
                         0.7148465705525415f,    0.23037781330885523f};
const float kDb4Hi[8] = {-0.23037781330885523f,  0.7148465705525415f,   -0.6308807679295904f,
                         -0.02798376941698385f,  0.18703481171888114f,  0.030841381835986965f,
                         -0.032883011666982945f, -0.010597401784997278f};
}  // namespace

void StreamingDwt::configure(WaveletType type, int levels) {
    type_ = type;
    levels_ = levels < 1 ? 1 : (levels > kMaxLevels ? kMaxLevels : levels);
    if (type == WaveletType::Haar) {
        taps_ = 2;
        lo_ = kHaarLo;
        hi_ = kHaarHi;
    } else {
        taps_ = 8;
        lo_ = kDb4Lo;
        hi_ = kDb4Hi;
    }
    reset();
}

void StreamingDwt::reset() {
    for (auto& l : lv_) {
        std::memset(l.buf, 0, sizeof(l.buf));
        l.pos = 0;
        l.phase = 0;
    }
    std::memset(d_, 0, sizeof(d_));
    clearEnergy();
}

void StreamingDwt::clearEnergy() { std::memset(e_, 0, sizeof(e_)); }

uint32_t StreamingDwt::push(float x) {
    uint32_t produced = 0;
    float in = x;
    for (int j = 0; j < levels_; ++j) {
        Level& L = lv_[j];
        L.buf[L.pos] = in;
        L.buf[L.pos + taps_] = in;
        L.pos = (L.pos + 1 == taps_) ? 0 : L.pos + 1;
        L.phase ^= 1;
        if (L.phase) break;                     // decimate by 2
        // window oldest..newest = buf[pos .. pos+taps-1]; filter taps applied newest-first
        const float* w = &L.buf[L.pos];
        float a = 0.0f, d = 0.0f;
        for (int k = 0; k < taps_; ++k) {
            const float s = w[taps_ - 1 - k];
            a += lo_[k] * s;
            d += hi_[k] * s;
        }
        d_[j] = d;
        e_[j] += d * d;
        produced |= 1u << j;
        in = a;                                  // approximation feeds next level
    }
    return produced;
}

}  // namespace pq
