#include "dsp/filtering.h"

#include <cmath>
#include <cstring>

#include "common/math_util.h"

namespace pq {

BiquadCoeffs designLowpass(float fc, float fs, float q) {
    const double w0 = 2.0 * M_PI * fc / fs;
    const double cw = std::cos(w0), sw = std::sin(w0);
    const double alpha = sw / (2.0 * q);
    const double a0 = 1.0 + alpha;
    BiquadCoeffs c;
    c.b0 = static_cast<float>((1.0 - cw) / 2.0 / a0);
    c.b1 = static_cast<float>((1.0 - cw) / a0);
    c.b2 = c.b0;
    c.a1 = static_cast<float>(-2.0 * cw / a0);
    c.a2 = static_cast<float>((1.0 - alpha) / a0);
    return c;
}

BiquadCoeffs designHighpass(float fc, float fs, float q) {
    const double w0 = 2.0 * M_PI * fc / fs;
    const double cw = std::cos(w0), sw = std::sin(w0);
    const double alpha = sw / (2.0 * q);
    const double a0 = 1.0 + alpha;
    BiquadCoeffs c;
    c.b0 = static_cast<float>((1.0 + cw) / 2.0 / a0);
    c.b1 = static_cast<float>(-(1.0 + cw) / a0);
    c.b2 = c.b0;
    c.a1 = static_cast<float>(-2.0 * cw / a0);
    c.a2 = static_cast<float>((1.0 - alpha) / a0);
    return c;
}

float biquadMagnitude(const BiquadCoeffs& c, float f, float fs) {
    const double w = 2.0 * M_PI * f / fs;
    // H(e^jw) = (b0 + b1 z^-1 + b2 z^-2)/(1 + a1 z^-1 + a2 z^-2)
    const double nr = c.b0 + c.b1 * std::cos(w) + c.b2 * std::cos(2 * w);
    const double ni = -(c.b1 * std::sin(w) + c.b2 * std::sin(2 * w));
    const double dr = 1.0 + c.a1 * std::cos(w) + c.a2 * std::cos(2 * w);
    const double di = -(c.a1 * std::sin(w) + c.a2 * std::sin(2 * w));
    return static_cast<float>(std::sqrt((nr * nr + ni * ni) / (dr * dr + di * di)));
}

void BiquadCascade::designButterworthLowpass(int order, float fc, float fs) {
    if (order < 2) order = 2;
    if (order > 2 * kMaxStages) order = 2 * kMaxStages;
    n_ = order / 2;
    for (int k = 1; k <= n_; ++k) {
        const double q = 1.0 / (2.0 * std::sin((2.0 * k - 1.0) * M_PI / (2.0 * order)));
        st_[k - 1].setCoeffs(designLowpass(fc, fs, static_cast<float>(q)));
    }
    reset();
}

void BiquadCascade::reset() {
    for (int s = 0; s < n_; ++s) st_[s].reset();
}

float BiquadCascade::magnitude(float f, float fs) const {
    float m = 1.0f;
    for (int s = 0; s < n_; ++s) m *= biquadMagnitude(st_[s].coeffs(), f, fs);
    return m;
}

void DcBlocker::configure(float fc, float fs) {
    r_ = static_cast<float>(std::exp(-2.0 * M_PI * fc / fs));
    reset();
}

// ------------------------------------------------------------ half-band FIR
namespace {
double besselI0(double x) {
    double sum = 1.0, term = 1.0;
    for (int k = 1; k < 50; ++k) {
        term *= (x / (2.0 * k)) * (x / (2.0 * k));
        sum += term;
        if (term < 1e-12 * sum) break;
    }
    return sum;
}

struct HalfbandTable {
    float h[HalfbandDecimator::kTaps];
    HalfbandTable() {
        const int M = (HalfbandDecimator::kTaps - 1) / 2;
        const double beta = 7.857;   // Kaiser beta for ~80 dB stop-band
        const double i0b = besselI0(beta);
        double sum = 0.0;
        double tmp[HalfbandDecimator::kTaps];
        for (int n = -M; n <= M; ++n) {
            double ideal;
            if (n == 0) ideal = 0.5;
            else if (n % 2 == 0) ideal = 0.0;
            else ideal = std::sin(M_PI * n / 2.0) / (M_PI * n);
            const double r = static_cast<double>(n) / M;
            const double w = besselI0(beta * std::sqrt(1.0 - r * r)) / i0b;
            tmp[n + M] = ideal * w;
            sum += tmp[n + M];
        }
        for (int k = 0; k < HalfbandDecimator::kTaps; ++k) h[k] = static_cast<float>(tmp[k] / sum);
    }
};
const HalfbandTable& halfband() {
    static const HalfbandTable t;
    return t;
}
}  // namespace

const float* HalfbandDecimator::prototype() { return halfband().h; }

void HalfbandDecimator::configure(int decimation) {
    dec_ = (decimation == 4) ? 4 : (decimation == 2 ? 2 : 1);
    nst_ = dec_ == 4 ? 2 : (dec_ == 2 ? 1 : 0);
    (void)halfband();
    reset();
}

void HalfbandDecimator::reset() {
    for (auto& s : st_) {
        std::memset(s.buf, 0, sizeof(s.buf));
        s.pos = 0;
        s.phase = 0;
    }
}

int HalfbandDecimator::groupDelayInput() const {
    const int d = (kTaps - 1) / 2;
    return nst_ == 0 ? 0 : (nst_ == 1 ? d : d + 2 * d);
}

bool HalfbandDecimator::stagePush(Stage& s, float x, float& y) {
    s.buf[s.pos] = x;
    s.buf[s.pos + kTaps] = x;
    s.pos = (s.pos + 1 == kTaps) ? 0 : s.pos + 1;
    s.phase ^= 1;
    if (s.phase) return false;       // output on every second input
    const float* w = &s.buf[s.pos];  // oldest .. newest, contiguous
    const float* h = halfband().h;
    constexpr int M = (kTaps - 1) / 2;
    float acc = h[M] * w[M];
    for (int k = 1; k <= M; k += 2) acc += h[M - k] * (w[M - k] + w[M + k]);  // odd taps only
    y = acc;
    return true;
}

bool HalfbandDecimator::push(float x, float& y) {
    if (nst_ == 0) {
        y = x;
        return true;
    }
    float t;
    if (!stagePush(st_[0], x, t)) return false;
    if (nst_ == 1) {
        y = t;
        return true;
    }
    return stagePush(st_[1], t, y);
}

}  // namespace pq
