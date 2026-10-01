// =============================================================================
//  Real-input radix-2 FFT (single precision).
//
//  An N-point real FFT is computed as an N/2-point complex FFT of the even/odd
//  interleaved samples followed by a split step — half the work of a complex
//  FFT of size N. Twiddles and the bit-reversal table are precomputed at init()
//  so the per-frame cost is butterflies only. Float is the right choice on the
//  RP2350 (Cortex-M33 single-precision FPU: FMUL/FADD 1 cycle); on an RP2040
//  build it falls back to the SDK's optimised soft-float in ROM.
// =============================================================================
#pragma once
#include <cstdint>

#include "pq_build_config.h"

namespace pq {

class RealFft {
public:
    bool init(int n);            // n: power of two, 16..PQ_FFT_MAX
    int  size() const { return n_; }
    // in: n real samples. out: (n/2+1) complex bins interleaved {re, im}.
    void forward(const float* in, float* out) ;

private:
    void complexFft(float* z) const;   // in-place, m = n/2 points, input bit-reversed
    int      n_ = 0, m_ = 0, log2m_ = 0;
    float    tw_[PQ_FFT_MAX / 2];      // m/2 complex twiddles e^{-j2pi k/m}
    float    sp_[PQ_FFT_MAX];          // m complex split twiddles e^{-j2pi k/n}
    uint16_t rev_[PQ_FFT_MAX / 2];
    float    work_[PQ_FFT_MAX];        // m complex
};

}  // namespace pq
