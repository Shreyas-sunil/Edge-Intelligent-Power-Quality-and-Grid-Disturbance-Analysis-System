# Testing without hardware

The firmware is layered so that everything above `src/platform/` compiles unchanged on a PC. The tests therefore exercise the real firmware code, not a model of it.

## 1. Unit and integration tests (65 tests, `firmware/tests/`)

```
cmake -S firmware/host -B build-host -DCMAKE_BUILD_TYPE=Release
cmake --build build-host
ctest --test-dir build-host --output-on-failure      # 18 ctest groups
build-host/pq_tests detector                         # one group, verbose measurements printed
```

The framework has no dependencies (`tests/test_framework.h`). Each test prints the quantities it measured, so failures are diagnosable.

| group | tests |
|---|---|
| adc | crc16_ccitt_known_vector, raw_to_engineering_units, invalid_samples_are_held, phase_compensation_delays_current, ads131m02_commands_and_registers, ads131m02_frame_decode_and_crc, sim_backend_blocks_and_timestamps, sim_overrun_is_detected_and_flagged, sim_stall_times_out_and_recovers, sim_crc_fault_injection_marks_samples, sim_counts_saturate_at_full_scale |
| filter | butterworth_lowpass_response, dc_blocker_removes_offset_keeps_50hz, halfband_decimator_passband_and_alias_rejection |
| freq | zero_crossing_accuracy, robust_to_harmonics_and_loss_of_signal |
| rms | half_cycle_rms_exact_for_offnominal_frequency, updates_during_interruption_via_timeout |
| fft | matches_reference_dft |
| thd | harmonic_magnitudes_and_thd_nonsynchronous, pure_sine_has_negligible_thd_and_configurable_order, interharmonic_peak_reported |
| wavelet | db4_rejects_fundamental_haar_does_not, energy_is_preserved_by_orthogonal_transform |
| transient | detects_impulse_on_the_sample_it_occurs, no_false_alarms_with_steady_harmonics_and_noise |
| detector | normal_grid_raises_no_events, sag_detected_with_depth_duration_latency, swell_and_interruption, short_sag_below_one_cycle_is_still_caught, hysteresis_prevents_chatter, harmonic_distortion_with_thd, frequency_variation, transient_detected_standalone_but_attached_to_sag, oscillatory_transient, load_switching_vs_sag, thresholds_are_configurable, combined_event_reported |
| feature | spec_order_and_hash_are_stable, values_for_clean_sine, sanitizes_non_finite_inputs |
| fusion | dsp_asserted_is_authoritative, ml_only_needs_persistence_and_is_advisory, multiple_families_give_combined |
| ml | model_loads_with_matching_spec, reference_vectors_match_training_output (TFLM == Python TFLite, bit-exact), classifies_pipeline_features |
| protocol | tokenizer_and_parsers, json_writer_escaping_and_overflow, metrics_and_event_messages |
| config | defaults_validate_and_parameter_table, pack_unpack_detects_corruption, sixty_hz_system_works |
| control | disarmed_outputs_are_inert, trip_latches_until_reset_and_refs, apf_packet_format, pipeline_trips_on_severe_swell_when_armed |
| logger | merges_updates_and_wraps |
| sim | end_to_end_protocol_session, config_change_restarts_pipeline_live, fault_injection_is_reported_not_hidden, offset_and_gain_calibration_procedures, phase_calibration_procedure |
| integration | all_disturbance_types_end_to_end, invalid_samples_are_tolerated_and_flagged |

The **integration** and **sim** groups run the complete chain requested in the specification:

```
Simulated ADC (counts) → Calibrator → DSP → PQ detection → features → TinyML → fusion → final event
```

The **sim** group also runs `RealtimeTask` (core-1 code) and `Supervisor` (core-0 code), driven only through the serial protocol, exactly as a PC would drive the board.

## 2. Evaluation on large random scenario sets

```
build-host/pq_datagen --eval --clips 150 --seed 4242 [--verbose]
```

This command prints the following:

- event-level detection rate, extra-event rate and latency per class;
- frame-level confusion matrices for DSP rules only, TinyML only, and the fused firmware output.

With `--verbose`, every unexpected event is listed with its time relative to the injected onset. See [tinyml.md](tinyml.md) for the current results.

## 3. PIO program — cycle-level emulation

```
pip install rp2040-pio-emulator
python firmware/tests/pio/test_ads131m02_pio.py
```

The ADS131M02 streaming state machine is emulated instruction by instruction (opcodes as produced by the Pico SDK encoders) against a model of the ADC's SPI output. The RX FIFO words must equal the transmitted 96-bit frames.

## 4. PC tools against the PC simulator

```
python tools/test_tools.py
python tools/serial_monitor.py --port sim --demo
```

`pq_sim` is the complete firmware application compiled for the PC, paced in real time. `test_tools.py` checks, over the real protocol, the following:

- STATUS, MODEL and CONFIG queries, including rejection of out-of-range values;
- streamed events from injected disturbances;
- periodic metrics;
- WAVE snapshots;
- CSV mode;
- the event log.

## 5. On the board, before the ADC exists

The image boots in **simulation mode** (`PQ_DEFAULT_SIMULATION=1`), so the whole firmware runs on the real MCU with synthetic samples paced by the hardware timer. From there:

- `SIM DEMO`, or individual `SIM` scenarios, exercise detection.
- `SIM FAULT CRC 0.001` and `SIM FAULT STALL ON` exercise the error paths: repaired samples, ADC timeout and automatic re-initialisation, all visible in `STATUS`.
- `BENCH` measures real timing on the MCU.
- `CONTROL ARM`, then `SIM swell 1.4` checks the trip line with a scope. Afterwards run `CONTROL RESET` and `CONTROL DISARM`.

## 6. Hardware bring-up checklist (ADS131M02)

1. Check CLKIN on GP21 with a scope: 5.12 MHz, 50 % duty.
2. Run `SOURCE ADC`, then `STATUS`. The field `adc.status` must be `OK`. `BAD_ID` or `COMM_FAILURE` means SPI wiring or mode is wrong; `CONFIG_MISMATCH` means the register write/read-back failed.
3. `STATUS` → `adc.blocks` should increase by 200 per second at 10 kS/s. `crc_errors` must stay at 0.
4. Run `WAVE 400` and check the shape and polarity of V and I.
5. Calibrate (see [configuration.md](configuration.md)) and `CONFIG SAVE`.
6. Run `BENCH` and record the numbers in performance.md.
