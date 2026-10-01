# Hardware interface (what the firmware expects from the board)

All pin numbers live in `firmware/src/platform/pico/board_pins.h`. Each can be overridden with `-DPQ_PIN_...`.

## ADC: TI ADS131M02 (default backend)

| Pico pin | ADS131M02 | notes |
|---|---|---|
| GP16 (SPI0 RX) | DOUT | also read by PIO while streaming |
| GP17 | CS̅ | GPIO during config, PIO `set` pin while streaming |
| GP18 (SPI0 SCK) | SCLK | SPI mode 1 (4 MHz during config); PIO side-set at 8 MHz while streaming |
| GP19 (SPI0 TX) | DIN | held low (NULL command) while streaming |
| GP20 | DRDY̅ | falling edge = new conversion |
| GP21 (PWM) | CLKIN | 5.12 MHz from clk_sys / 30 (RP2350 at 153.6 MHz) or / 25 (RP2040 at 128 MHz) |
| GP22 | SYNC/RESET̅ | pulsed low at initialisation |

Configuration done by `Ads131m02Backend::init()`, each step verified by read-back:

- hardware reset, then the ID must report 2 channels;
- MODE = 0x0510 (24-bit words, SPI timeout);
- GAIN from `acq.v_pga_gain` / `acq.i_pga_gain`;
- CLOCK = both channels enabled, OSR chosen for `acq.sample_rate_hz`, high-resolution mode.

The output data rate is CLKIN / 2 / OSR, e.g. 10 kS/s at OSR 256.

Every frame is checked for:

- **CRC-16-CCITT:** `acq.crc_check`;
- **status word:** 24-bit length and both DRDY bits set;
- **full-scale codes:** saturation.

Default scaling is set for a front end that maps ±600 V peak and ±50 A peak onto the ±1.2 V input range. Calibrate the real values with `CALIBRATE` (see configuration.md).

**A different ADC** only needs a new `AdcBackend` implementation, for example an ADS131M04 for three phases with `kAdcChannels = 4`. No DSP code changes.

## Control and reference outputs (low-voltage logic only)

| Pin | Signal | Behaviour |
|---|---|---|
| GP10 | SSCB TRIP request | Active high. Latched until `CONTROL RESET`. Only asserted when armed, for a half-cycle Vrms > `ctrl.trip_swell_pu` (1.3 pu), an impulse > `ctrl.trip_impulse_pu` (2 pu), or an interruption if `ctrl.trip_on_interruption` is set. Evaluated on core 1, every half cycle or sample. |
| GP11 | HEALTHY | 100 Hz square wave while the real-time loop runs. The breaker hardware should treat a missing heartbeat as a fault. |
| GP12–15 | EVENT code | 4-bit `PqClass` of the active event: 1 = sag … 7 = load switching, 8 = combined |
| GP9 | EVENT strobe | high while an event is active |
| GP6 | DVR reference (PWM, 12-bit, ~37 kHz) | required series injection = 1 − Vpu, clamped to ±0.5 pu; mid-scale = 0. Dead band ±3 %. Zero below 0.1 pu. Updated every half cycle. |
| GP7 | APF reference (PWM) | current THD 0–100 % → 0–full scale |
| GP4 (UART1 TX, 921 600 Bd) | APF harmonic link | per frame: `A5 5A len 01 frame:u32 thd_i:f32 n:u8 {I_h:f32, phase_h:f32}×15 crc16` (CRC-16-CCITT over type..payload, little-endian) |
| GP26/27/28 | optional MCP4822 | `-DPQ_USE_MCP4822=ON`: DVR on channel A, APF on channel B, 0–4.096 V |

The C API requested in the specification is available as free functions:

```cpp
ControlOutput_SetEvent(PqClass c, bool active);
ControlOutput_SetReference(RefChannel ch, float value_pu);
ControlOutput_Trip(PqClass reason, float value, uint64_t now_us);
```

All actuating outputs (trip and references) are inert until `CONTROL ARM`. The event code is informational and always driven.

## Host links and annunciation

| Pin | Use |
|---|---|
| USB | CDC serial: the protocol |
| GP0/GP1 | UART0 protocol mirror (`comm.uart_enable`) |
| GP25 | status LED: 1 Hz blip when OK, 5 Hz blink on a fault (ADC or config) |
| GP8 | alarm LED/buzzer driver: short blip = info, slow = warning (sag, transient, harmonic, frequency), fast = critical (swell, interruption, combined), solid = fault |

## Fail-safe behaviour

- **Watchdog.** 1 s, fed only while both cores make progress. After a watchdog reset, `WATCHDOG_RESET` is reported.
- **ADC failure.** No DRDY for 3 block periods raises `ADC_COMM`. The ADC is re-initialised automatically, at most once per second. The status LED blinks fast, and frames spanning the gap are marked invalid.
- **Flash writes** (`CONFIG SAVE`) park core 1 for a few ms. The sample gap is detected and flagged like any other gap.
