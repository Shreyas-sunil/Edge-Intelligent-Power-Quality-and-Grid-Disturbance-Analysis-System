# Serial protocol (v1)

**Transports.** The protocol runs over USB CDC, which is the primary transport: any baud setting works and the port appears as COMx. It can optionally be mirrored on UART0 (GP0/GP1, `comm.uart_enable = 1`, 115200 8N1 by default). The PC simulator speaks the same protocol on stdin/stdout, or on a pseudo-terminal with `pq_sim --pty`.

- **Host → device:** one ASCII command per line (`\n` or `\r\n`, at most 159 characters). Keywords are case-insensitive. A partial line left for more than 2 s is discarded with an error (`COMM_TIMEOUT`).
- **Device → host:** one **JSON object per line**, always with a `"type"` field. Lines starting with `#` are human-readable text; parsers skip them. With `FORMAT CSV`, metrics become compact `M,...` lines preceded by a `# M,...` header line.
- The device never blocks on a slow host. If the output buffer is full, whole lines are dropped, counted in `tx_dropped`, and `COMM_TIMEOUT` is flagged. Raw sample buffers are sent only on request (`WAVE`).

## Commands

| Command | Reply type(s) | Notes |
|---|---|---|
| `PING` | `ack` | |
| `STATUS` / `STATUS CLEAR` | `status` / `ack` | health, errors, ADC diagnostics, CPU load, memory, control state. CLEAR resets sticky error flags |
| `METRICS` \| `METRICS ONCE` | `metrics` | one metrics line now |
| `METRICS ON` \| `OFF` \| `<ms>` | `ack` | periodic stream (100–60000 ms) |
| `FORMAT JSON` \| `CSV` | `ack` | metrics format |
| `EVENTS [n]` | n × `event_log`, `ack` | last n log entries (default 20) |
| `EVENTS TEXT [n]` | `#` text, `ack` | human-readable event reports |
| `EVENTS CLEAR` | `ack` | |
| `HARMONICS` | `harmonics` | V and I harmonics H1–H15, phases, interharmonic peaks |
| `FEATURES` | `features` | current feature vector, ML probabilities, DSP evidence, fused scores |
| `MODEL` | `model` | model size, arena use, feature list, accuracy, inference time |
| `CONFIG LIST` | n × `config`, `ack` | all parameters with range and description |
| `CONFIG GET <key>` \| `SET <key> <value>` | `config` / `error` | validated; applied live (restart-only keys restart acquisition) |
| `CONFIG SAVE` \| `LOAD` \| `DEFAULTS` \| `ERASE` | `ack` / `error` | flash persistence (CRC-protected, versioned) |
| `CALIBRATE SHOW` | `cal` | |
| `CALIBRATE OFFSET [ms]` | `ack`, `cal` | inputs must be at zero |
| `CALIBRATE VGAIN <Vrms>` \| `IGAIN <Arms>` | `ack`, `cal` | apply a known reference signal first |
| `CALIBRATE PHASE [deg]` | `ack`, `cal` | resistive reference load (true angle, default 0°) |
| `START` \| `STOP` | `ack` | acquisition on or off |
| `SOURCE SIM` \| `ADC` | `ack` | select the simulator or the hardware ADC |
| `SIM <scenario> [mag] [dur_s]` | `ack` | `sag swell interruption harmonic transient oscillatory frequency load` |
| `SIM DEMO` \| `CLEAR` \| `LIST` | `ack` | DEMO schedules 7 disturbances over ~21 s |
| `SIM BASE noise\|freq\|load\|level\|harm\|phase\|pf <v>` | `ack` | steady-state simulator conditions |
| `SIM FAULT CRC <p>` \| `STALL ON\|OFF` | `ack` | inject ADC faults to exercise error handling |
| `WAVE [n]` | `wave` × ⌈n/32⌉ | last n (≤ 512) calibrated samples of V and I, in chunks of 32 |
| `BENCH` | `bench` × 3 | live stage timings (core 1) + FFT-size and kernel micro-benchmarks (core 0) |
| `CONTROL ARM` \| `DISARM` \| `RESET` \| `TEST <class>` | `ack` | control outputs (see hardware_interface.md) |
| `REBOOT`, `HELP` | | |

Errors reply as `{"type":"error","cmd":"CONFIG SET","msg":"value out of range"}`.

## Messages

### `metrics` (periodic; values below are illustrative)

```json
{"type":"metrics","timestamp_ms":124582,"frame":1215,"vrms":176.30,"irms":2.140,"frequency":49.970,
 "thd":3.20,"thd_i":8.41,"event":"SAG","confidence":0.971,"ml_class":"SAG","ml_conf":0.994,
 "vrms_half":176.1,"vmin":175.9,"vmax":177.0,"h1":176.2,"h3":4.1,"h5":2.9,"h7":1.2,
 "p":352.1,"s":377.3,"pf":0.933,"dpf":0.940,"rocof":0.002,"crest":1.43,"valid":true,"q":"0x00000000"}
```

- `event` / `confidence` give the most significant fused classification since the previous metrics line, so a 100 ms sag is not lost between 1 s reports.
- `frequency` is `null` while there are no zero crossings, for example during an interruption.
- `q` is the data-quality bitmask (below).
- The timestamp is device time since start of acquisition, derived from the sample counter.

### `event` (streamed as it happens)

Each disturbance produces three messages with the same `id`:

- `start` — at DSP detection, with `det_latency_ms` measured from the estimated onset;
- `classified` — adds the ML class and confidence and the fused confidence, with `cls_latency_ms`;
- `end` — adds `duration_ms` and the extreme magnitude.

```json
{"type":"event","id":12,"state":"end","event":"VOLTAGE_SAG","dsp":"VOLTAGE_SAG","ml":"SAG","ml_conf":0.994,
 "confidence":0.998,"source":"dsp+ml","t_start_ms":124482.0,"duration_ms":200,"magnitude":0.596,
 "vrms":176.3,"frequency":49.970,"thd":3.20,"det_latency_ms":14.1,"cls_latency_ms":29.1,
 "components":["SAG"],"flags":["WITH_TRANSIENT"]}
```

`magnitude` has a different meaning for each event type:

| event | magnitude |
|---|---|
| sag, interruption | minimum pu |
| swell | maximum pu |
| harmonic | maximum THD % |
| frequency | extreme Δf in Hz |
| transient | peak deviation in pu of Vpk |
| load | relative step |

`flags` can contain `CLIPPED`, `WITH_TRANSIENT`, `ML_ONLY_ADVISORY`, `ML_DISAGREE`, `TRIPPED`, `COMBINED` and `UNCLASSIFIED`.

`EVENTS TEXT` prints the same record in the requested human-readable form:

```
# Event #12 (ended)
#   Timestamp: 124.482 s
#   Event: VOLTAGE_SAG
#   Vrms: 176.3 V
#   Frequency: 49.97 Hz
#   THD: 3.2 %
#   ML Class: SAG
#   Confidence: 99.4 %
#   Detection latency: 14.1 ms
```

### Other messages

- `hello`: firmware version, protocol version, platform, model version and ML readiness. Sent on (re)connection.
- `log`: firmware log messages (level, tag, text), for example ADC re-initialisation and SSCB trips.
- `wave`: `{"n":512,"offset":0,"fs":10000,"t0_ms":…,"v":[…32],"i":[…32]}`.
- `bench`: see performance.md.

### Data-quality / error bits (`q`, `STATUS.errors`)

| bit | name | meaning |
|---|---|---|
| 0x001 | ADC_COMM | ADC not responding, wrong ID, register read-back mismatch, DRDY stuck |
| 0x002 | DMA_OVERRUN | the DMA ring lapped the consumer; samples lost |
| 0x004 | BUFFER_OVERFLOW | an inter-core queue was full; a frame or event was dropped |
| 0x008 | INVALID_SAMPLES | CRC or status errors in ADC frames (samples repaired, counted) |
| 0x010 | ADC_SATURATION | codes at full scale; magnitudes are lower bounds (`CLIPPED`) |
| 0x020 | MISSING_SAMPLES | a gap in the sample sequence; the analysis restarts, frames are marked invalid |
| 0x040 | ML_INIT | model missing or mismatched; the system runs DSP-only |
| 0x080 | COMM_TIMEOUT | host not reading, or an incomplete command |
| 0x100 | CONFIG_INVALID | stored configuration rejected; defaults in use |
| 0x200 | FLASH | flash write or verify failed |
| 0x400 | FREQ_LOST | no valid zero crossings in this frame |
| 0x800 | WATCHDOG_RESET | the last reset was caused by the watchdog |

Frames with `valid:false` (a gap, or more than 1 % repaired samples in the window) are not classified by the ML model.

## PC tools

`tools/protocol.py` implements the protocol (`PQLink.open("COM5")`, or `"sim"` to spawn the PC simulator). It is used by `serial_monitor.py`, `live_plot.py`, `data_logger.py` and `test_tools.py`.
