"""Cycle-level check of the ADS131M02 streaming PIO program (no hardware needed).

The opcodes below are exactly what Ads131m02Backend::buildPioProgram() produces
with the Pico SDK encoders for DRDY on GP20 (dumped from the SDK headers; see
docs/testing.md). The PIO is emulated with `rp2040-pio-emulator` against a
model of the ADC's SPI output (mode 1: DOUT changes on the rising SCLK edge),
and the words autopushed to the RX FIFO must equal the transmitted frames.

    pip install rp2040-pio-emulator
    python firmware/tests/pio/test_ads131m02_pio.py
"""
import random
import sys

from pioemu import State, emulate

PROGRAM = [0xE001, 0x2094, 0x2014, 0xE100, 0xA022, 0xB242, 0x5001, 0x0345]
PULL, MOV_Y_OSR = 0x8080, 0xA047

MISO_BIT = 0          # emulator reads `in pins` from bit 0 (in_base remapped)
CS, SCLK, DRDY = 17, 18, 20
FRAME_BITS = 96
PERIOD = 3000         # PIO clocks between conversions (arbitrary for the model)


def run(frames):
    bits = []
    for f in frames:
        bits.append([(f >> (FRAME_BITS - 1 - k)) & 1 for k in range(FRAME_BITS)])
    dev = {"frame": -1, "bit": 0, "dout": 0, "prev_sclk": 0, "prev_cs": 1}

    def pins(state: State) -> int:
        t = state.clock
        idx = t // PERIOD
        drdy = 0 if (idx >= 1 and idx <= len(frames) and (t % PERIOD) < 400) else 1
        if drdy == 0 and dev["frame"] != idx - 1:
            dev["frame"], dev["bit"] = idx - 1, 0          # new conversion latched
        sclk = (state.pin_values >> SCLK) & 1
        cs = (state.pin_values >> CS) & 1
        if not cs and sclk and not dev["prev_sclk"] and 0 <= dev["frame"] < len(frames):
            b = dev["bit"]
            dev["dout"] = bits[dev["frame"]][b] if b < FRAME_BITS else 0
            dev["bit"] += 1
        dev["prev_sclk"], dev["prev_cs"] = sclk, cs
        return (drdy << DRDY) | (dev["dout"] << MISO_BIT)

    # y = 95 exactly as the firmware does (put + exec pull + exec mov y, osr)
    init = State(pin_directions=(1 << CS) | (1 << SCLK), pin_values=1 << CS, y_register=FRAME_BITS - 1)
    words, last = [], None
    gen = emulate(PROGRAM, stop_when=lambda op, s: s.clock > PERIOD * (len(frames) + 2),
                  initial_state=init, input_source=pins, auto_push=True, push_threshold=32,
                  shift_isr_right=False, set_base=CS, set_count=1, side_set_base=SCLK, side_set_count=1,
                  wrap_target=0, wrap_top=7)
    max_sclk_edges = 0
    for before, after in gen:
        while after.receive_fifo:
            words.append(after.receive_fifo.popleft())
        last = after
    return words


def main():
    rnd = random.Random(1)
    frames = [rnd.getrandbits(FRAME_BITS) for _ in range(3)]
    words = run(frames)
    expect = []
    for f in frames:
        expect += [(f >> 64) & 0xFFFFFFFF, (f >> 32) & 0xFFFFFFFF, f & 0xFFFFFFFF]
    ok = words == expect
    print(f"{len(frames)} frames -> {len(words)} FIFO words: {'OK' if ok else 'MISMATCH'}")
    if not ok:
        for i, (a, b) in enumerate(zip(words, expect)):
            print(f"  word {i}: got {a:08x} expected {b:08x}")
        sys.exit(1)


if __name__ == "__main__":
    main()
