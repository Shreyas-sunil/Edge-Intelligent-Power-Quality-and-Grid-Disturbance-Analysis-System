#include "dsp/fft.h"

#include <cmath>

namespace pq {

bool RealFft::init(int n) {
    if (n < 16 || n > PQ_FFT_MAX || (n & (n - 1))) return false;
    n_ = n;
    m_ = n / 2;
    log2m_ = 0;
    while ((1 << log2m_) < m_) ++log2m_;
    for (int k = 0; k < m_ / 2; ++k) {
        const double a = -2.0 * M_PI * k / m_;
        tw_[2 * k] = static_cast<float>(std::cos(a));
        tw_[2 * k + 1] = static_cast<float>(std::sin(a));
    }
    for (int k = 0; k < m_; ++k) {
        const double a = -2.0 * M_PI * k / n_;
        sp_[2 * k] = static_cast<float>(std::cos(a));
        sp_[2 * k + 1] = static_cast<float>(std::sin(a));
    }
    for (int k = 0; k < m_; ++k) {
        unsigned r = 0, v = static_cast<unsigned>(k);
        for (int b = 0; b < log2m_; ++b) {
            r = (r << 1) | (v & 1u);
            v >>= 1;
        }
        rev_[k] = static_cast<uint16_t>(r);
    }
    return true;
}

void RealFft::complexFft(float* z) const {
    // Iterative radix-2 decimation-in-time, input already in bit-reversed order.
    for (int len = 2; len <= m_; len <<= 1) {
        const int half = len >> 1;
        const int step = m_ / len;               // twiddle stride
        for (int base = 0; base < m_; base += len) {
            for (int j = 0; j < half; ++j) {
                const float wr = tw_[2 * j * step];
                const float wi = tw_[2 * j * step + 1];
                float* a = &z[2 * (base + j)];
                float* b = &z[2 * (base + j + half)];
                const float tr = wr * b[0] - wi * b[1];
                const float ti = wr * b[1] + wi * b[0];
                b[0] = a[0] - tr;
                b[1] = a[1] - ti;
                a[0] += tr;
                a[1] += ti;
            }
        }
    }
}

void RealFft::forward(const float* in, float* out) {
    // Pack real input as complex z[k] = x[2k] + j x[2k+1], in bit-reversed order.
    for (int k = 0; k < m_; ++k) {
        const int r = rev_[k];
        work_[2 * r] = in[2 * k];
        work_[2 * r + 1] = in[2 * k + 1];
    }
    complexFft(work_);

    // Split: X[k] = Fe[k] + W^k Fo[k],  Fe = (Z[k] + Z*[m-k])/2,  Fo = -j(Z[k] - Z*[m-k])/2
    out[0] = work_[0] + work_[1];
    out[1] = 0.0f;
    out[2 * m_] = work_[0] - work_[1];
    out[2 * m_ + 1] = 0.0f;
    for (int k = 1; k < m_; ++k) {
        const float zr = work_[2 * k], zi = work_[2 * k + 1];
        const float cr = work_[2 * (m_ - k)], ci = -work_[2 * (m_ - k) + 1];   // conj(Z[m-k])
        const float fer = 0.5f * (zr + cr), fei = 0.5f * (zi + ci);
        const float dr = 0.5f * (zr - cr), di = 0.5f * (zi - ci);
        const float for_ = di, foi = -dr;                                        // -j * d
        const float wr = sp_[2 * k], wi = sp_[2 * k + 1];
        out[2 * k] = fer + wr * for_ - wi * foi;
        out[2 * k + 1] = fei + wr * foi + wi * for_;
    }
}

}  // namespace pq
