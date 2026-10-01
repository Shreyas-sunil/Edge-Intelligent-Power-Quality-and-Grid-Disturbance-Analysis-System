// =============================================================================
//  Digital filters used in the pipeline (and ONLY those that have a job):
//
//   DcBlocker          1st-order IIR high-pass, fc ~1 Hz. Tracks ADC/sensor
//                      offset drift that static calibration cannot.
//   BiquadCascade      4th-order Butterworth low-pass (2 biquads, TDF-II) at
//                      1.5 x f_nom: conditions the voltage for zero-crossing
//                      detection only (suppresses H3+ and noise so crossings
//                      are unique). NOT applied to RMS/FFT paths.
//   HalfbandDecimator  Kaiser-windowed half-band FIR, /2 per stage. Anti-alias
//                      filter in front of the FFT (fs 10 kHz -> 5 kHz keeps
//                      harmonics to 2 kHz = H40 with >80 dB alias rejection).
// =============================================================================
#pragma once
#include <cstdint>

namespace pq {

struct BiquadCoeffs {
    float b0, b1, b2, a1, a2;   // a0 normalised to 1
};

class Biquad {
public:
    void  setCoeffs(const BiquadCoeffs& c) { c_ = c; }
    void  reset() { z1_ = z2_ = 0.0f; }
    // Transposed direct form II: best float behaviour for low fc/fs.
    inline float process(float x) {
        const float y = c_.b0 * x + z1_;
        z1_ = c_.b1 * x - c_.a1 * y + z2_;
        z2_ = c_.b2 * x - c_.a2 * y;
        return y;
    }
    const BiquadCoeffs& coeffs() const { return c_; }

private:
    BiquadCoeffs c_{1, 0, 0, 0, 0};
    float z1_ = 0.0f, z2_ = 0.0f;
};

// Bilinear-transform designs with frequency pre-warping.
BiquadCoeffs designLowpass(float fc, float fs, float q);
BiquadCoeffs designHighpass(float fc, float fs, float q);
// |H(f)| of a biquad (for tests / documentation).
float biquadMagnitude(const BiquadCoeffs& c, float f, float fs);

class BiquadCascade {
public:
    static constexpr int kMaxStages = 4;
    // Butterworth low-pass of even order (2 or 4 or 6 or 8).
    void  designButterworthLowpass(int order, float fc, float fs);
    void  reset();
    inline float process(float x) {
        for (int s = 0; s < n_; ++s) x = st_[s].process(x);
        return x;
    }
    float magnitude(float f, float fs) const;
    int   stages() const { return n_; }

private:
    Biquad st_[kMaxStages];
    int    n_ = 0;
};

class DcBlocker {
public:
    void configure(float fc, float fs);
    void reset() { x1_ = y1_ = 0.0f; primed_ = false; }
    inline float process(float x) {
        if (!primed_) {             // start from the first sample -> no startup step
            x1_ = x;
            primed_ = true;
        }
        const float y = x - x1_ + r_ * y1_;
        x1_ = x;
        y1_ = y;
        return y;
    }
    float r() const { return r_; }

private:
    float r_ = 0.999f, x1_ = 0.0f, y1_ = 0.0f;
    bool  primed_ = false;
};

class HalfbandDecimator {
public:
    static constexpr int kTaps = 51;             // 4*12+3: half-band length
    static constexpr int kMaxStages = 2;         // /2 or /4
    // decimation 1, 2 or 4. attenuation ~ 80 dB, transition 0.2..0.3 fs_in.
    void configure(int decimation);
    void reset();
    // Push one input sample; returns true when an output sample is produced.
    bool push(float x, float& y);
    int  decimation() const { return dec_; }
    int  groupDelayInput() const;                // in input samples
    static const float* prototype();             // kTaps coefficients (for tests)

private:
    struct Stage {
        float buf[kTaps * 2];   // doubled circular buffer -> contiguous window
        int   pos = 0;
        int   phase = 0;
    };
    bool  stagePush(Stage& s, float x, float& y);
    Stage st_[kMaxStages];
    int   nst_ = 0;
    int   dec_ = 1;
};

}  // namespace pq
