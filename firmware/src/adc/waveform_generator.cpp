#include "adc/waveform_generator.h"

#include <cmath>
#include <cstring>

#include "common/math_util.h"

namespace pq {

const char* distKindName(DistKind k) {
    switch (k) {
        case DistKind::Sag: return "sag";
        case DistKind::Swell: return "swell";
        case DistKind::Interruption: return "interruption";
        case DistKind::Harmonic: return "harmonic";
        case DistKind::ImpulsiveTransient: return "impulse";
        case DistKind::OscillatoryTransient: return "oscillatory";
        case DistKind::FrequencyDeviation: return "frequency";
        case DistKind::LoadStep: return "load";
        default: return "?";
    }
}

PqClass distKindClass(DistKind k) {
    switch (k) {
        case DistKind::Sag: return PqClass::Sag;
        case DistKind::Swell: return PqClass::Swell;
        case DistKind::Interruption: return PqClass::Interruption;
        case DistKind::Harmonic: return PqClass::Harmonic;
        case DistKind::ImpulsiveTransient:
        case DistKind::OscillatoryTransient: return PqClass::Transient;
        case DistKind::FrequencyDeviation: return PqClass::FrequencyVariation;
        case DistKind::LoadStep: return PqClass::LoadSwitching;
        default: return PqClass::Normal;
    }
}

double Disturbance::effectiveEnd() const {
    switch (kind) {
        case DistKind::ImpulsiveTransient: return t_start + 5.0 * static_cast<double>(p2);
        case DistKind::OscillatoryTransient: return t_start + 5.0 * static_cast<double>(p2);
        case DistKind::FrequencyDeviation: {
            // ramp back to nominal after the end
            double ramp = (p1 > 0.0f) ? std::fabs(magnitude) / p1 : 0.0;
            return duration < 0 ? 1e30 : t_start + duration + ramp;
        }
        default: return duration < 0 ? 1e30 : t_start + duration;
    }
}

void WaveformGenerator::configure(const GeneratorParams& p, float fs) {
    p_ = p;
    fs_ = fs;
    rng_.seed(p.seed);
    // Fixed harmonic phase angles (sin(h*theta + phi_h)); cached so the per-sample
    // loop needs a single sin/cos pair per channel.
    float ph[kMaxH + 1] = {0};
    ph[3] = p.base_h_phase[0];
    ph[5] = p.base_h_phase[1] + 0.6f;
    ph[7] = p.base_h_phase[2] + 1.1f;
    ph[11] = 0.3f;
    ph[13] = 2.0f;
    for (int h = 0; h <= kMaxH; ++h) {
        hcos_[h] = std::cos(ph[h]);
        hsin_[h] = std::sin(ph[h]);
    }
    reset(0);
}

void WaveformGenerator::reset(uint64_t start_sample) {
    n_ = start_sample;
    theta_ = 0.0f;
    fcur_ = p_.fnom + p_.f_offset;
}

bool WaveformGenerator::addDisturbance(const Disturbance& d) {
    purgeExpired(time());
    if (nd_ >= kMaxDist) return false;
    dist_[nd_++] = d;
    return true;
}

void WaveformGenerator::clearDisturbances() { nd_ = 0; }

void WaveformGenerator::purgeExpired(double t) {
    int w = 0;
    for (int r = 0; r < nd_; ++r)
        if (dist_[r].effectiveEnd() + 0.5 > t) dist_[w++] = dist_[r];
    nd_ = w;
}

void WaveformGenerator::generate(uint32_t n, float* v, float* i) {
    const float vpk = kSqrt2 * p_.vnom_rms;
    const float pf = p_.pf_angle_deg * (kPi / 180.0f);
    const float ierr = p_.i_phase_err_deg * (kPi / 180.0f);
    const float dt = 1.0f / fs_;

    for (uint32_t k = 0; k < n; ++k, ++n_) {
        const double t = static_cast<double>(n_) / fs_;

        float f = p_.fnom + p_.f_offset;
        float vscale = p_.v_scale;
        float jump = 0.0f;
        float kload = 1.0f;
        float zdrop = 0.0f;
        float vh[kMaxH + 1];
        std::memset(vh, 0, sizeof(vh));
        vh[3] = p_.base_h[0];
        vh[5] = p_.base_h[1];
        vh[7] = p_.base_h[2];
        float vtrans = 0.0f;

        for (int d = 0; d < nd_; ++d) {
            const Disturbance& ds = dist_[d];
            const double tau = t - ds.t_start;
            if (tau < 0.0) continue;
            const bool in = ds.duration < 0.0 || tau < ds.duration;
            switch (ds.kind) {
                case DistKind::Sag:
                case DistKind::Swell:
                case DistKind::Interruption:
                    if (in) {
                        vscale *= ds.magnitude;
                        jump += ds.p1 * (kPi / 180.0f);
                    }
                    break;
                case DistKind::Harmonic:
                    if (in) {
                        vh[3] += ds.p1; vh[5] += ds.p2; vh[7] += ds.p3;
                        vh[11] += ds.magnitude; vh[13] += 0.7f * ds.magnitude;
                    }
                    break;
                case DistKind::ImpulsiveTransient: {
                    const float tr = ds.p1 > 1e-7f ? ds.p1 : 20e-6f;
                    const float tf = ds.p2 > tr * 1.01f ? ds.p2 : tr * 5.0f;
                    if (tau < 10.0 * tf) {
                        const float tp = std::log(tf / tr) * tr * tf / (tf - tr);
                        const float norm = std::exp(-tp / tf) - std::exp(-tp / tr);
                        const float ft = static_cast<float>(tau);
                        vtrans += ds.magnitude * vpk * (std::exp(-ft / tf) - std::exp(-ft / tr)) / norm;
                    }
                    break;
                }
                case DistKind::OscillatoryTransient: {
                    const float td = ds.p2 > 1e-5f ? ds.p2 : 1e-3f;
                    if (tau < 10.0 * td) {
                        const float ft = static_cast<float>(tau);
                        vtrans += ds.magnitude * vpk * std::exp(-ft / td) *
                                  std::sin(kTwoPi * ds.p1 * ft + ds.phase);
                    }
                    break;
                }
                case DistKind::FrequencyDeviation: {
                    const float rate = ds.p1 > 0.0f ? ds.p1 : 1e9f;
                    const float target = ds.magnitude;
                    float up = static_cast<float>(tau) * rate;
                    float df;
                    if (in) {
                        df = up >= std::fabs(target) ? target : std::copysign(up, target);
                    } else {
                        const float down = static_cast<float>(tau - ds.duration) * rate;
                        const float rem = std::fabs(target) - down;
                        df = rem > 0.0f ? std::copysign(rem, target) : 0.0f;
                    }
                    f += df;
                    break;
                }
                case DistKind::LoadStep:
                    if (in) {
                        kload *= ds.magnitude;
                        zdrop += ds.p1 * (ds.magnitude - 1.0f) * p_.i_rms / p_.i_nom;
                    }
                    break;
                default: break;
            }
        }
        vscale *= (1.0f - zdrop);
        fcur_ = f;

        // Fundamental and harmonics via complex rotation (one sin/cos per sample).
        const float th = theta_ + jump;
        const float s1 = std::sin(th), c1 = std::cos(th);
        float sv = s1, cv = c1;
        float vsum = s1;
        for (int h = 2; h <= kMaxH; ++h) {
            const float ns = sv * c1 + cv * s1;
            const float nc = cv * c1 - sv * s1;
            sv = ns; cv = nc;
            if (vh[h] != 0.0f)
                vsum += vh[h] * (sv * hcos_[h] + cv * hsin_[h]);
        }

        const float thi = th - pf + ierr;
        const float si = std::sin(thi), ci = std::cos(thi);
        float isum = si;
        {
            float sh = si, chh = ci;
            for (int h = 2; h <= 7; ++h) {
                const float ns = sh * ci + chh * si;
                const float nc = chh * ci - sh * si;
                sh = ns; chh = nc;
                if (h == 3) isum += p_.i_h[0] * sh;
                if (h == 5) isum += p_.i_h[1] * sh;
                if (h == 7) isum += p_.i_h[2] * sh;
            }
        }

        v[k] = vpk * vscale * vsum + vtrans + p_.noise_v_rms * rng_.gauss();
        // constant-impedance load: current follows the voltage envelope
        i[k] = kSqrt2 * p_.i_rms * kload * vscale * isum + p_.noise_i_rms * rng_.gauss();

        theta_ += kTwoPi * f * dt;
        if (theta_ >= kTwoPi) theta_ -= kTwoPi;
    }
    purgeExpired(time());
}

void WaveformGenerator::skip(uint64_t n) {
    const double adv = static_cast<double>(kTwoPi) * fcur_ * static_cast<double>(n) / fs_;
    theta_ = static_cast<float>(std::fmod(static_cast<double>(theta_) + adv, static_cast<double>(kTwoPi)));
    n_ += n;
    purgeExpired(time());
}

uint32_t WaveformGenerator::generateCounts(uint32_t n, int32_t* cv, int32_t* ci) {
    float v[256], i[256];
    uint32_t sat = 0;
    const int32_t fs = p_.full_scale;
    uint32_t done = 0;
    while (done < n) {
        const uint32_t m = (n - done) > 256 ? 256 : (n - done);
        generate(m, v, i);
        for (uint32_t k = 0; k < m; ++k) {
            float a = v[k] / p_.v_gain + p_.v_offset_counts;
            float b = i[k] / p_.i_gain + p_.i_offset_counts;
            int32_t ca = static_cast<int32_t>(std::lround(clampv(a, -2.0e9f, 2.0e9f)));
            int32_t cb = static_cast<int32_t>(std::lround(clampv(b, -2.0e9f, 2.0e9f)));
            if (ca > fs) { ca = fs; ++sat; } else if (ca < -fs - 1) { ca = -fs - 1; ++sat; }
            if (cb > fs) { cb = fs; ++sat; } else if (cb < -fs - 1) { cb = -fs - 1; ++sat; }
            cv[done + k] = ca;
            ci[done + k] = cb;
        }
        done += m;
    }
    return sat;
}

uint16_t WaveformGenerator::truthMask(double t0, double t1, double min_overlap) const {
    uint16_t mask = 0;
    for (int d = 0; d < nd_; ++d) {
        const Disturbance& ds = dist_[d];
        const PqClass c = distKindClass(ds.kind);
        if (ds.kind == DistKind::LoadStep) {
            // switching instants are the events
            const bool on = ds.t_start >= t0 && ds.t_start <= t1;
            const bool off = ds.duration >= 0 && (ds.t_start + ds.duration) >= t0 &&
                             (ds.t_start + ds.duration) <= t1;
            if (on || off) mask |= classBit(c);
            continue;
        }
        const double a = ds.t_start, b = ds.effectiveEnd();
        const double ov = (b < t1 ? b : t1) - (a > t0 ? a : t0);
        const bool transient = (c == PqClass::Transient);
        if ((transient && ov > 0.0) || (!transient && ov >= min_overlap)) mask |= classBit(c);
    }
    return mask;
}

bool makeScenario(const char* name, double t0, float mag, float dur, Rng& rng, Disturbance& d) {
    d = Disturbance{};
    d.t_start = t0;
    auto pick = [&](float dflt) { return mag != 0.0f ? mag : dflt; };
    auto len = [&](float dflt) { return dur > 0.0f ? static_cast<double>(dur) : static_cast<double>(dflt); };
    if (strEqualNoCase(name, "sag")) {
        d.kind = DistKind::Sag; d.magnitude = pick(0.60f); d.duration = len(0.2f);
    } else if (strEqualNoCase(name, "swell")) {
        d.kind = DistKind::Swell; d.magnitude = pick(1.25f); d.duration = len(0.2f);
    } else if (strEqualNoCase(name, "interruption")) {
        d.kind = DistKind::Interruption; d.magnitude = pick(0.01f); d.duration = len(0.3f);
    } else if (strEqualNoCase(name, "harmonic")) {
        const float s = pick(1.0f);
        d.kind = DistKind::Harmonic; d.p1 = 0.10f * s; d.p2 = 0.07f * s; d.p3 = 0.05f * s;
        d.magnitude = 0.02f * s; d.duration = len(2.0f);
    } else if (strEqualNoCase(name, "transient") || strEqualNoCase(name, "impulse")) {
        d.kind = DistKind::ImpulsiveTransient; d.magnitude = pick(1.0f);
        d.p1 = 30e-6f; d.p2 = 400e-6f; d.duration = 0.005;
    } else if (strEqualNoCase(name, "oscillatory")) {
        d.kind = DistKind::OscillatoryTransient; d.magnitude = pick(0.5f);
        d.p1 = 1200.0f; d.p2 = 2e-3f; d.phase = rng.uniform(0.0f, kTwoPi); d.duration = 0.02;
    } else if (strEqualNoCase(name, "frequency")) {
        d.kind = DistKind::FrequencyDeviation; d.magnitude = pick(1.5f); d.p1 = 5.0f; d.duration = len(3.0f);
    } else if (strEqualNoCase(name, "load")) {
        d.kind = DistKind::LoadStep; d.magnitude = pick(3.0f); d.p1 = 0.01f; d.duration = len(1.0f);
    } else {
        return false;
    }
    return true;
}

}  // namespace pq
