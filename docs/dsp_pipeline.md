# DSP pipeline

Defaults are 10 kS/s acquisition, decimation to a 5 kS/s analysis rate, a 1024-point FFT (204.8 ms ≈ 10 cycles at 50 Hz) and a hop of 512. Everything is configurable (see [configuration.md](configuration.md)).

## 1. Calibration (`adc/adc_calibration.cpp`)

`v = (counts − v_offset)·v_gain`, and the same for `i`. Samples the ADC flagged invalid (bad CRC or status) are replaced by the last good value, counted, and the frame is marked. **Phase compensation**: a CT or divider phase error φ at f_nom is a time shift, corrected by a fractional delay of the leading channel of `φ/360·fs/f_nom` samples using linear interpolation. The residual measured in `adc.phase_compensation_delays_current` is 0.005 % of full scale.

## 2. Filters — only the ones that have a job

| Filter | Where | Design | Why this cut-off |
|---|---|---|---|
| DC blocker | V and I, every sample | 1st-order IIR HP, fc = 1 Hz (`r = e^{-2π·1/fs}`) | Removes ADC and sensor offset drift that static calibration cannot. At 1 Hz the gain at 50 Hz is 0.9998, a negligible RMS error. Settling takes ~0.16 s. |
| Frequency LPF | voltage → zero-crossing detector only | 4th-order Butterworth (two biquads, TDF-II), fc = 1.5·f_nom = 75 Hz | Guarantees exactly two crossings per cycle even with 20 % H3. Measured: −24 dB at 150 Hz, gain 0.981 at 50 Hz. The constant group delay cancels in period differences. Not used for RMS or FFT, which stay full bandwidth. |
| Half-band FIR decimator | before the FFT | 51-tap Kaiser (β = 7.86) half-band, ↓2 (or ↓4) | Anti-alias filter: 10 → 5 kS/s keeps harmonics up to 2 kHz (H40 at 50 Hz) flat, 0.9999 at 2 kHz, and rejects anything that would alias by 79.8 dB (measured at 3.05 kHz). Even taps are zero, so it costs ~13 MACs per output. |
| Wavelet filter bank | transient detection | Haar or Daubechies-4, 3 levels (Mallat) | Bands are d1 = 2.5–5 kHz, d2 = 1.25–2.5 kHz and d3 = 0.625–1.25 kHz. This is high-frequency energy that the fundamental and low harmonics do not have. |

No other smoothing is applied. PQ quantities (RMS, THD) are defined on the unfiltered waveform.

## 3. Frequency (`dsp/frequency.cpp`) — interpolated zero crossing

The chain is LPF → hysteresis comparator (10 % of the tracked envelope, with a floor of 2 % of V_nom) → linear interpolation of the crossing instant → `f = K·fs/(t_K − t_0)` over the last K = 5 cycles (configurable). A per-cycle frequency and a smoothed ROCOF are also produced. Periods outside 0.8–1.2 f_nom are rejected, and the estimate is invalidated after 2.5 periods without a crossing (interruption). The frequency is then reported as `null`, not 0.

**Why this method.** It costs about 10 operations per sample and updates every cycle. The measured error is ≤ 0.8 mHz from 45 to 61 Hz with noise (test `freq.zero_crossing_accuracy`). A plain FFT peak has 4.9 Hz bins at N = 1024; the interpolated FFT peak is still computed as a cross-check (`fft_freq`).

## 4. RMS (`dsp/rms.cpp`) — cycle-synchronous, refreshed every half cycle

Windows are delimited by the fractional zero-crossing instants. The boundary sample is split by its fraction, so a one-cycle window is exactly one period long. The result is **Urms(1/2)** in IEC 61000-4-30 terms: a one-cycle RMS refreshed every half cycle. The measured error is < 3 mV on 230 V at 49.5, 50 and 50.7 Hz.

Without crossings (interruption) a window is closed after 1.25 nominal half cycles, so RMS keeps updating about 80 times per second. Values are flagged `synced=false` unless both halves were crossing-delimited. Unsynchronised windows may only open or escalate an interruption, never a sag or swell. This prevents a partial half cycle at re-synchronisation from being reported as a swell.

The frame `vrms`, `irms` and `p` sum whole half cycles inside the window. A raw 10.24-cycle window would ripple by ±1.5 %.

## 5. FFT (`dsp/fft.cpp`)

The FFT is a real-input radix-2 transform: an N/2 complex FFT plus a split step, with twiddles and the bit-reversal table precomputed. Single-precision float is used, because the RP2350's Cortex-M33 has a single-cycle FPU. On the RP2040 the SDK's optimised ROM float routines are used.

Accuracy against a double-precision DFT is ≤ 1.2e-5 relative (test `fft.matches_reference_dft`).

**Size selection** (`pq_bench` / `BENCH` print this table with real timings):

| N at 5 kS/s | bin | window | trade-off |
|---|---|---|---|
| 256 | 19.5 Hz | 51 ms (2.5 cycles) | fast, but Hann main lobes of adjacent harmonics overlap; poor THD |
| 512 | 9.8 Hz | 102 ms (5 cycles) | usable |
| **1024 (default)** | 4.9 Hz | 205 ms (≈10 cycles, the IEC 61000-4-7 window) | harmonic grouping ±2 bins, standard-comparable THD |
| 2048 | 2.4 Hz | 410 ms | better interharmonic resolution, slower event reporting |

The 50 % overlap (hop N/2) gives a new spectrum every 102 ms.

## 6. Harmonics and THD (`dsp/thd.cpp`)

A Hann window is applied. The window is usually **not** synchronous with the fundamental, so harmonic h is measured by summing |X[k]|² over ±g bins around `h·f0/Δf` (spectral grouping, g = 2 at N = 1024), with f0 taken from the zero-crossing estimator. Parseval gives

`V_h = sqrt(2·Σ|X|² / (N·Σw²))`   and   `THD = sqrt(V2² + … + V_H²)/V1 · 100 %`

H is configurable with `dsp.max_harmonic` (up to 40). Also reported are the harmonic phases (used for P, DPF and the APF reference) and the three strongest non-harmonic peaks (interharmonics).

Measured with f0 = 49.87 Hz (non-synchronous), H3/5/7/11 = 8/5/3/2 %: H1 error 0.01 %, THD 10.099 % against a true 10.100 %.

## 7. Wavelet (`dsp/wavelet.cpp`) — lightweight and optional

This is a streaming Mallat filter bank: each level filters and decimates by 2, so the total cost is under 2× the first level. It is configurable (Haar or db4, 1–4 levels) and can be switched off (`dsp.wavelet_enable`).

**Trade-off, measured.** Haar needs 2 taps per level. But its single vanishing moment lets the 50 Hz slope leak into d1: 7.2 V peak on 325 V (2.2 % of Vpk) at 10 kS/s, so the detection threshold must sit above that. db4 needs 8 taps and leaks only 0.0002 V (≈ −120 dB), allowing a ~10× lower threshold.

Host cost for 3 levels: Haar 17.6 ns per sample, db4 22.0 ns per sample. On the Pico, run `BENCH` to see `wavelet_haar_3lvl` and `wavelet_db4_3lvl`. The default is db4.

## 8. Transient detection (`dsp/transient.cpp`)

Four detectors run on every sample. Each is normalised to a score where ≥ 1 means triggered:

1. **Amplitude:** |v| > 1.4 pu of V_pk.
2. **dv/dt:** above 4× the maximum slope of the nominal sine.
3. **Cycle-to-cycle deviation:** `v[n] − v[n−T]`, with a fractional period T taken from the frequency estimator. Steady harmonics cancel, so only *changes* remain. The "echo" one period later is blanked.
4. **Wavelet detail:** above K × an adaptive noise floor per level.

Detection happens on the sample that crosses (latency 0 in the test; 2.5 ms mean end-to-end including block buffering).

Two checks separate transients from steady-state changes:

- **Post-event verification:** one cycle after the activity ends, the waveform is compared with a cycle recorded *before* the event. If they still differ by more than 5 % RMS, it was a *step* (sag onset, phase jump, harmonic load switched in), not a transient. It is then attached to the voltage event or dropped, and the wavelet floors are re-learnt.
- **Non-repetitive criterion:** a transient must change the waveform relative to the previous cycle (peak deviation ≥ 25 % of the deviation threshold). Periodic features such as notching, a high crest factor or high-order harmonics are steady-state distortion.

A transient within 3 cycles of a sag, swell or interruption boundary, or of a load switching, is attached to that event (`WITH_TRANSIENT`) instead of being reported separately.

## 9. Disturbance detection (`power_quality/disturbance_detector.cpp`)

| Class | Rule (thresholds in `PQConfig`) | Evaluated |
|---|---|---|
| VOLTAGE_SAG | Urms(½) < 0.90 pu; ends above 0.90 + 0.02 hysteresis | every ½ cycle |
| VOLTAGE_SWELL | Urms(½) > 1.10 pu; ends below 1.10 − 0.02 | every ½ cycle |
| INTERRUPTION | Urms(½) < 0.10 pu (a sag that deepens is escalated in place) | every ½ cycle |
| LOAD_SWITCHING | step > 30 % in load admittance `Irms/Vpu` | every ½ cycle |
| TRANSIENT | see §8 | every sample |
| HARMONIC_DISTORTION | THD > 8 % (off below 7.5 %), with V > 0.5 pu and a steady RMS in the window | every frame |
| FREQUENCY_VARIATION | \|f − f_nom\| > 0.5 Hz for 2 frames | every frame |
| COMBINED_EVENT | ≥ 2 conditions from different families at once | fusion |

Using admittance instead of current for load switching means a constant-impedance load whose current falls during a sag is not reported as switching (test `detector.load_switching_vs_sag`). Harmonic and frequency assessment is suspended ("flagged", IEC 61000-4-30) only while the RMS *changes* inside the window, not during a steady sag.
