// Power-quality result/event types shared by detector, fusion, logger, protocol.
#pragma once
#include <cstdint>

#include "common/pq_types.h"
#include "dsp/thd.h"
#include "pq_build_config.h"

namespace pq {

// Metrics computed for every analysis frame (window = FFT size, every hop).
struct PqMetrics {
    float vrms;            // true RMS over the last ~10 cycles (full bandwidth) [V]
    float irms;            // [A]
    float vrms_half;       // latest Urms(1/2) (one cycle, refreshed per half cycle) [V]
    float vmin_half;       // min / max Urms(1/2) inside this frame's window [V]
    float vmax_half;
    float imin_cycle;      // min / max cycle Irms inside the window [A]
    float imax_cycle;
    float freq;            // zero-crossing frequency [Hz]
    float fft_freq;        // FFT-peak frequency (cross-check) [Hz]
    float rocof;           // [Hz/s]
    float thd_v;           // [%]
    float thd_i;           // [%]
    float v_h[PQ_REPORT_HARMONICS + 1];   // harmonic RMS [V], [1] = fundamental
    float i_h[PQ_REPORT_HARMONICS + 1];   // [A]
    float i_ph[PQ_REPORT_HARMONICS + 1];  // current harmonic phase rel. to V1 [rad]
    float p;               // active power [W]
    float s;               // apparent power [VA]
    float pf;              // true power factor
    float dpf;             // displacement PF cos(phi1)
    float phi_vi_deg;      // V1 angle - I1 angle [deg] (used by phase calibration)
    float vdc, idc;        // DC component before the DC blocker [V], [A]
    float peak_v;          // max |v| in window [V]
    float crest;           // peak / rms
    float kurtosis;        // of the voltage samples in the window
    float wavelet_energy;  // log10(sum d^2 / sum v^2)
    float transient_energy;// log10(sum dev^2 / sum v^2)
    SpectralPeak peaks[3];
    uint8_t npeaks;
    uint8_t freq_valid;
    uint8_t harm_valid;
    uint8_t valid;         // 0 when samples were missing/invalid in the window
    uint32_t quality;      // ErrorFlag bits relevant to this frame
};

enum EventState : uint8_t { EV_START = 0, EV_CLASSIFIED = 1, EV_END = 2 };
enum EventSource : uint8_t { SRC_DSP = 0, SRC_ML = 1, SRC_DSP_ML = 2 };
enum EventFlags : uint16_t {
    EVF_CLIPPED        = 1u << 0,   // ADC saturated during the event (magnitude is a bound)
    EVF_WITH_TRANSIENT = 1u << 1,   // a transient accompanied the onset/end
    EVF_ML_ONLY        = 1u << 2,   // advisory: only the ML model saw it
    EVF_ML_DISAGREE    = 1u << 3,   // ML's class differs from the DSP class
    EVF_TRIPPED        = 1u << 4,   // SSCB trip was issued
    EVF_COMBINED       = 1u << 5,   // overlapped with another event family
    EVF_UNCLASSIFIED   = 1u << 6,   // ended before any ML frame covered it
};

struct EventRecord {
    uint32_t id;
    PqClass  dsp_type;
    PqClass  final_type;
    PqClass  ml_type;
    uint8_t  state;          // EventState
    uint8_t  source;         // EventSource
    uint16_t flags;          // EventFlags
    uint16_t components;     // classBit() mask of DSP conditions seen during the event
    uint64_t t_start_us;
    uint32_t duration_ms;    // 0 while active
    float    magnitude;      // sag/int: min pu, swell: max pu, harmonic: max THD %,
                             // freq: extreme df Hz, transient: peak pu, load: rel. step
    float    ml_conf;
    float    fused_conf;
    float    vrms;           // at detection [V]
    float    freq;
    float    thd;
    float    det_latency_ms; // detection time - estimated onset
    float    cls_latency_ms; // fused classification time - estimated onset
};

}  // namespace pq
