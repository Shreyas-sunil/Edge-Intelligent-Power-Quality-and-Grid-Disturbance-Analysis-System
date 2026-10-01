# Configuration

There are three layers, and each has exactly one home:

1. **Compile-time defaults and buffer limits:** `firmware/config/pq_build_config.h`. Every `#define` there can be overridden with `-D...` (for example `cmake ... -DCMAKE_CXX_FLAGS="-DPQ_FFT_MAX=1024"`). Buffer limits include `PQ_BLOCK_SIZE`, `PQ_ACQ_RING_BLOCKS`, `PQ_FFT_MAX`, `PQ_MAX_HARMONIC` and `PQ_EVENT_LOG_SIZE`.
2. **Run-time configuration:** `SystemConfig` in `firmware/src/system/config.h`. It is made of `AcqConfig`, `CalConfig`, `PQConfig` (the thresholds), `DspConfig`, `MlConfig`, `FusionConfig`, `CommConfig` and `ControlConfig`. No threshold is hard-coded anywhere else; the detector, fusion and outputs read it from here.
3. **Persistence:** `CONFIG SAVE` writes the whole structure, with magic, version and CRC-32, to the last 4 KB flash sector. At boot it is validated: magic, version, size, CRC, and a range check of every parameter. If it is invalid, the defaults are used and `CONFIG_INVALID` is flagged.

Every field is reachable through a parameter table, with no per-field code:

```
CONFIG LIST
CONFIG GET pq.sagThreshold
CONFIG SET pq.sagThreshold 0.88          -> validated, applied immediately
CONFIG SET acq.fft_size 2048             -> "restart" key: acquisition restarts automatically
CONFIG SAVE
```

`configSet()` validates each value against its range and against cross-constraints:

- the FFT size is a power of two;
- the hop divides the window;
- the decimated rate is at least 8× f_nom;
- the window is at least 2 cycles;
- the interruption threshold is below the sag threshold.

An invalid change is rejected with a message, and the running configuration is left untouched.

## Parameters (generated from the firmware's parameter table)

| Key | Default | Range | Restart | Description |
|---|---|---|---|---|
| `acq.sample_rate_hz` | 10000 | 1000 … 25600 | yes | ADC sample rate [Hz] |
| `acq.fft_size` | 1024 | 256 … 2048 | yes | FFT/window size (decimated samples, pow2) |
| `acq.hop` | 512 | 64 … 2048 | yes | analysis hop (decimated samples) |
| `acq.decimation` | 2 | 1 … 4 | yes | pre-FFT decimation 1/2/4 |
| `acq.source` | 1 | 0 … 1 | yes | 0 = hardware ADC, 1 = simulator |
| `acq.v_pga_gain` | 1 | 1 … 128 | yes | voltage channel PGA gain |
| `acq.i_pga_gain` | 1 | 1 … 128 | yes | current channel PGA gain |
| `acq.crc_check` | 1 | 0 … 1 |  | verify ADC frame CRC |
| `cal.v_gain` | 7.2e-05 | 0 … 1 |  | V per count |
| `cal.v_offset` | 0 | -1e+06 … 1e+06 |  | voltage offset [counts] |
| `cal.i_gain` | 6e-06 | 0 … 1 |  | A per count |
| `cal.i_offset` | 0 | -1e+06 … 1e+06 |  | current offset [counts] |
| `cal.phase_deg` | 0 | -20 … 20 |  | current phase correction [deg] |
| `pq.nominalVoltage` | 230 | 50 … 1000 |  | nominal voltage [V rms] |
| `pq.nominalFrequency` | 50 | 45 … 65 | yes | nominal frequency [Hz] |
| `pq.nominalCurrent` | 10 | 0.01 … 10000 |  | nominal current [A rms] |
| `pq.sagThreshold` | 0.9 | 0.5 … 0.99 |  | sag threshold [pu] |
| `pq.swellThreshold` | 1.1 | 1.01 … 2 |  | swell threshold [pu] |
| `pq.interruptionThreshold` | 0.1 | 0.01 … 0.5 |  | interruption threshold [pu] |
| `pq.rmsHysteresis` | 0.02 | 0 … 0.1 |  | RMS hysteresis [pu] |
| `pq.thdThreshold` | 8 | 0.5 … 50 |  | THD threshold [%] |
| `pq.thdHysteresis` | 0.5 | 0 … 5 |  | THD hysteresis [%] |
| `pq.frequencyTolerance` | 0.5 | 0.01 … 5 |  | frequency tolerance [Hz] |
| `pq.transientDevPu` | 0.15 | 0.02 … 2 |  | transient cycle deviation [pu Vpk] |
| `pq.impulsePu` | 1.4 | 1.05 … 5 |  | impulse amplitude [pu Vpk] |
| `pq.dvdtFactor` | 4 | 1.5 … 50 |  | dv/dt threshold [x max 50Hz slope] |
| `pq.waveletK` | 8 | 2 … 100 |  | wavelet threshold [x noise floor] |
| `pq.waveletFloorPu` | 0.03 | 0.001 … 1 |  | wavelet absolute floor [pu Vpk] |
| `pq.loadStepThreshold` | 0.3 | 0.05 … 5 |  | load switching step [relative] |
| `pq.loadMinCurrent` | 0.2 | 0 … 1000 |  | load switching min current [A] |
| `pq.minEventHalfCycles` | 1 | 1 … 10 |  | half-cycles beyond threshold to open event |
| `pq.freqMinFrames` | 2 | 1 … 50 |  | frames beyond tolerance to open FREQ event |
| `dsp.dc_block_hz` | 1 | 0.1 … 10 | yes | DC blocker corner [Hz] |
| `dsp.freq_lpf_factor` | 1.5 | 1.1 … 4 | yes | frequency LPF corner [x f_nom] |
| `dsp.freq_avg_cycles` | 5 | 1 … 50 | yes | cycles averaged for frequency |
| `dsp.wavelet_enable` | 1 | 0 … 1 | yes | wavelet transient detector |
| `dsp.wavelet_type` | 1 | 0 … 1 | yes | 0 Haar, 1 db4 |
| `dsp.wavelet_levels` | 3 | 1 … 4 | yes | wavelet decomposition levels |
| `dsp.transient_enable` | 1 | 0 … 1 |  | transient detection |
| `dsp.max_harmonic` | 40 | 2 … 40 |  | highest harmonic in THD |
| `ml.ml_enable` | 1 | 0 … 1 |  | TinyML inference |
| `fusion.dsp_weight` | 0.6 | 0 … 1 |  | fusion weight of DSP evidence |
| `fusion.ml_only_confidence` | 0.9 | 0.5 … 1 |  | confidence for ML-only advisories |
| `fusion.ml_only_frames` | 3 | 1 … 50 |  | consecutive frames for ML-only advisory |
| `comm.metrics_period_ms` | 1000 | 0 … 60000 |  | metrics stream period [ms], 0=off |
| `comm.uart_baud` | 115200 | 9600 … 3e+06 |  | UART baud |
| `comm.metrics_format` | 0 | 0 … 1 |  | 0 JSON, 1 CSV |
| `comm.uart_enable` | 0 | 0 … 1 |  | mirror protocol on UART0 (CONFIG SAVE + REBOOT) |
| `comm.event_stream` | 1 | 0 … 1 |  | stream events as they occur |
| `comm.logging_enable` | 1 | 0 … 1 |  | RAM event log |
| `ctrl.armed` | 0 | 0 … 1 |  | enable trip/reference outputs |
| `ctrl.trip_on_interruption` | 0 | 0 … 1 |  | SSCB trip on interruption |
| `ctrl.dvr_enable` | 1 | 0 … 1 |  | DVR reference output |
| `ctrl.apf_enable` | 1 | 0 … 1 |  | APF reference output |
| `ctrl.trip_swell_pu` | 1.3 | 1.05 … 3 |  | SSCB trip swell level [pu] |
| `ctrl.trip_impulse_pu` | 2 | 1.2 … 10 |  | SSCB trip impulse level [pu Vpk] |

Keys correspond to the requested configuration items as follows:

| Requested item | Key |
|---|---|
| sampling rate | `acq.sample_rate_hz` (the ADS131M02 supports 5.12 MHz/2/OSR, i.e. 20000, 10000, 5000, 2500, …; the nearest one is used and reported) |
| FFT size / window size | `acq.fft_size`, `acq.hop` |
| nominal voltage / frequency | `pq.nominalVoltage`, `pq.nominalFrequency` (50 or 60 Hz systems) |
| THD / sag / swell thresholds | `pq.thdThreshold`, `pq.sagThreshold`, `pq.swellThreshold`, `pq.interruptionThreshold` |
| ML enable | `ml.ml_enable` |
| wavelet enable | `dsp.wavelet_enable` (+ `dsp.wavelet_type`, `dsp.wavelet_levels`) |
| logging enable | `comm.logging_enable` |
| communication rate | `comm.metrics_period_ms`, `comm.uart_baud` |

## Calibration procedure (hardware)

1. `SOURCE ADC` and let the front end warm up.
2. **Offset.** Short the inputs, or apply 0 V / 0 A, then run `CALIBRATE OFFSET 2000`. This averages 2 s of raw counts per channel.
3. **Voltage gain.** Apply a known voltage, for example 230.0 V from a calibrator, then run `CALIBRATE VGAIN 230.0`. This averages 10 frames; the gain is scaled by reference/measured.
4. **Current gain.** Apply a known current, then run `CALIBRATE IGAIN 5.0`.
5. **Phase.** Use a resistive load (PF = 1), then run `CALIBRATE PHASE 0`. The measured V1–I1 angle becomes the correction; the current channel is delayed by a fractional sample.
6. `CALIBRATE SHOW`, then `CONFIG SAVE`.

Each step is covered by an automated test against the simulator with injected offsets, a 3 % gain error and a 1.5° CT phase error (`sim.offset_and_gain_calibration_procedures`, `sim.phase_calibration_procedure`).
