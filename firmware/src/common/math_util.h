#pragma once
#include <cmath>
#include <cstddef>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include <cstdint>

namespace pq {

constexpr float kPi    = 3.14159265358979323846f;
constexpr float kTwoPi = 6.28318530717958647692f;
constexpr float kSqrt2 = 1.41421356237309504880f;

template <typename T>
inline T clampv(T v, T lo, T hi) { return v < lo ? lo : (v > hi ? hi : v); }

inline float sanitize(float v, float lim = 1.0e6f) {
    if (!std::isfinite(v)) return 0.0f;
    return clampv(v, -lim, lim);
}

// Portable case-insensitive string equality (no POSIX strcasecmp on MSVC).
bool strEqualNoCase(const char* a, const char* b);

uint32_t crc32(const void* data, size_t len, uint32_t seed = 0);
uint16_t crc16Ccitt(const uint8_t* data, size_t len, uint16_t init = 0xFFFF);
uint32_t fnv1a32(const char* s);

// Deterministic PRNG (xorshift32) — used by the waveform simulator so every
// test / dataset is reproducible from a seed.
class Rng {
public:
    explicit Rng(uint32_t seed = 0x1234567u) : s_(seed ? seed : 0x1234567u) {}
    void     seed(uint32_t s) { s_ = s ? s : 0x1234567u; }
    uint32_t next() {
        uint32_t x = s_;
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        return s_ = x;
    }
    float uniform() { return static_cast<float>(next() >> 8) * (1.0f / 16777216.0f); }  // [0,1)
    float uniform(float a, float b) { return a + (b - a) * uniform(); }
    // Approximate N(0,1) via the sum of 4 uniforms (Irwin–Hall), cheap on an MCU.
    float gauss() {
        return (uniform() + uniform() + uniform() + uniform() - 2.0f) * 1.7320508f;
    }

private:
    uint32_t s_;
};

}  // namespace pq
