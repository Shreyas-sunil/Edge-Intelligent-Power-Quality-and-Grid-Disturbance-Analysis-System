"""Live plots: Vrms / Irms / frequency / THD trends with event markers, plus an
optional waveform panel refreshed from WAVE snapshots.

    python tools/live_plot.py --port COM5
    python tools/live_plot.py --port sim --demo --wave
    python tools/live_plot.py --port sim --demo --save plot.png --duration 25   (headless)
"""
from __future__ import annotations

import argparse
import collections
import threading
import time

from protocol import add_port_args, open_from_args


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    add_port_args(ap)
    ap.add_argument("--window", type=float, default=60.0, help="seconds shown")
    ap.add_argument("--period", type=int, default=200, help="metrics period [ms]")
    ap.add_argument("--wave", action="store_true", help="show the waveform panel (polls WAVE each second)")
    ap.add_argument("--demo", action="store_true")
    ap.add_argument("--save", help="save a PNG at the end instead of an interactive window")
    ap.add_argument("--duration", type=float, default=0)
    a = ap.parse_args()

    import matplotlib
    if a.save:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    link = open_from_args(a)
    link.send(f"METRICS {a.period}")
    link.send("FORMAT JSON")
    if a.demo:
        link.send("SIM DEMO")

    n = int(a.window * 1000 / a.period) + 10
    ts, v, i, f, thd = (collections.deque(maxlen=n) for _ in range(5))
    events = collections.deque(maxlen=200)
    wave = {"data": None}
    lock = threading.Lock()
    stop = threading.Event()

    def rx():
        last_wave = 0.0
        while not stop.is_set():
            if a.wave and time.time() - last_wave > 1.0:
                last_wave = time.time()
                try:
                    w = link.wave(256, timeout=2)
                    with lock:
                        wave["data"] = w
                except Exception:
                    pass
            m = link.get(0.05)
            if not m:
                continue
            with lock:
                if m.get("type") == "metrics":
                    ts.append(m["timestamp_ms"] / 1000.0)
                    v.append(m["vrms"])
                    i.append(m["irms"])
                    f.append(m["frequency"] if m.get("frequency") is not None else float("nan"))
                    thd.append(m["thd"])
                elif m.get("type") == "event" and m.get("state") == "start":
                    events.append((m["t_start_ms"] / 1000.0, m["event"]))

    th = threading.Thread(target=rx, daemon=True)
    th.start()

    rows = 5 if a.wave else 4
    fig, ax = plt.subplots(rows, 1, figsize=(11, 2.1 * rows), sharex=False)
    fig.suptitle(f"Edge PQ analyser — {link.name}")
    labels = [("Vrms [V]", v), ("Irms [A]", i), ("Frequency [Hz]", f), ("THD [%]", thd)]
    lines = []
    for k, (lab, _) in enumerate(labels):
        (ln,) = ax[k].plot([], [], lw=1.2)
        ax[k].set_ylabel(lab)
        ax[k].grid(alpha=0.3)
        lines.append(ln)
    ax[3].set_xlabel("device time [s]")
    if a.wave:
        (wv,) = ax[4].plot([], [], lw=1, label="v [V]")
        ax4b = ax[4].twinx()
        (wi,) = ax4b.plot([], [], lw=1, color="tab:orange", label="i [A]")
        ax[4].set_xlabel("snapshot time [ms]")
        ax[4].set_ylabel("v [V]")
        ax4b.set_ylabel("i [A]")
    markers = []

    def redraw():
        with lock:
            t = list(ts)
            series = [list(s) for _, s in labels]
            evs = list(events)
            w = wave["data"]
        if not t:
            return
        for k, ln in enumerate(lines):
            ln.set_data(t, series[k])
            ax[k].relim()
            ax[k].autoscale_view()
            ax[k].set_xlim(max(t[0], t[-1] - a.window), t[-1] + 0.1)
        for mk in markers:
            mk.remove()
        markers.clear()
        for te, name in evs:
            if te >= t[-1] - a.window:
                markers.append(ax[0].axvline(te, color="red", alpha=0.5, lw=1))
                markers.append(ax[0].text(te, ax[0].get_ylim()[1], name, rotation=90, fontsize=7, va="top",
                                          color="red"))
        if a.wave and w:
            tt = [w["t0_ms"] + 1000.0 * k / w["fs"] for k in range(len(w["v"]))]
            wv.set_data(tt, w["v"])
            wi.set_data(tt, w["i"])
            ax[4].relim()
            ax[4].autoscale_view()
            ax4b.relim()
            ax4b.autoscale_view()

    t0 = time.time()
    try:
        if a.save:
            while not a.duration or time.time() - t0 < a.duration:
                time.sleep(0.2)
            redraw()
            fig.tight_layout()
            fig.savefig(a.save, dpi=110)
            print(f"saved {a.save}")
        else:
            plt.ion()
            fig.tight_layout()
            plt.show()
            while plt.fignum_exists(fig.number) and (not a.duration or time.time() - t0 < a.duration):
                redraw()
                fig.canvas.draw_idle()
                plt.pause(0.25)
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        link.close()


if __name__ == "__main__":
    main()
