// =============================================================================
//  Streaming (sample-by-sample) discrete wavelet transform for transient
//  detection. Implemented as the Mallat filter bank: each level low/high-pass
//  filters its input and decimates by 2, so the total cost is < 2x level-1:
//
//      level j detail band = [fs/2^(j+1), fs/2^j]   (fs = 10 kHz ->
//          d1: 2.5-5 kHz, d2: 1.25-2.5 kHz, d3: 625-1250 Hz)
//
//  Haar (2 taps) vs Daubechies-4 (8 taps): Haar has one vanishing moment, so
//  its d1 still "sees" the 50 Hz slope (|d1| ~ 2 % of Vpk at 10 kS/s) and the
//  threshold must sit above that; db4 has four vanishing moments and rejects
//  the fundamental by > 100 dB, allowing a ~10x lower detection threshold for
//  ~4x the multiply count. Both are selectable (dsp.wavelet_type).
// =============================================================================
#pragma once
#include <cstdint>

namespace pq {

enum class WaveletType : uint8_t { Haar = 0, Db4 = 1 };

class StreamingDwt {
public:
    static constexpr int kMaxLevels = 4;
    static constexpr int kMaxTaps = 8;

    void configure(WaveletType type, int levels);
    void reset();
    // Push one sample. Returns a bitmask of levels that produced a new detail
    // coefficient this call (bit j-1 for level j); read them with detail(j).
    uint32_t push(float x);
    float detail(int level) const { return d_[level - 1]; }
    int   levels() const { return levels_; }
    int   taps() const { return taps_; }
    WaveletType type() const { return type_; }

    // Energy accumulators (sum of d^2 per level) — reset by the caller.
    float energy(int level) const { return e_[level - 1]; }
    void  clearEnergy();

private:
    struct Level {
        float buf[kMaxTaps * 2];
        int   pos = 0;
        int   phase = 0;
    };
    WaveletType type_ = WaveletType::Db4;
    int   levels_ = 3;
    int   taps_ = 8;
    const float* lo_ = nullptr;
    const float* hi_ = nullptr;
    Level lv_[kMaxLevels];
    float d_[kMaxLevels]{};
    float e_[kMaxLevels]{};
};

}  // namespace pq
