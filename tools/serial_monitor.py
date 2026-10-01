"""Live console monitor for the edge-PQ analyser.

    python tools/serial_monitor.py --port COM5
    python tools/serial_monitor.py --port sim --demo        # no hardware needed
    python tools/serial_monitor.py --port COM5 --log runs/  # also record CSV files

Shows a live metrics line, prints every detected event, and lets you type
protocol commands (STATUS, EVENTS TEXT, SIM sag 0.5 0.2, CONFIG SET ..., HELP).
"""
from __future__ import annotations

import argparse
import sys
import threading
import time

from protocol import add_port_args, open_from_args

COLORS = {"NORMAL": "\033[32m", "event": "\033[33;1m", "error": "\033[31;1m", "dim": "\033[2m", "end": "\033[0m"}


def c(s: str, k: str, color: bool) -> str:
    return f"{COLORS[k]}{s}{COLORS['end']}" if color else s


def fmt_metrics(m: dict, color: bool) -> str:
    ev = m.get("event", "?")
    evs = c(f"{ev:<13}", "NORMAL" if ev == "NORMAL" else "event", color)
    ml = m.get("ml_class", "-")
    return (f"t={m.get('timestamp_ms', 0) / 1000:9.2f}s  V={m.get('vrms', 0):7.2f}V  I={m.get('irms', 0):6.3f}A  "
            f"f={(m.get('frequency') if m.get('frequency') is not None else float('nan')):7.3f}Hz  THD={m.get('thd', 0):5.2f}%  PF={m.get('pf', 0):5.2f}  "
            f"{evs} conf={m.get('confidence', 0):.2f}  ML={ml}({m.get('ml_conf', 0):.2f})"
            + ("" if m.get("valid", True) in (True, 1, 1.0) else c("  [INVALID]", "error", color)))


def fmt_event(e: dict, color: bool) -> str:
    st = e.get("state")
    head = {"start": ">> START", "classified": "   CLASS", "end": "<< END  "}.get(st, st)
    s = (f"{head} #{e.get('id')} {e.get('event')}  mag={e.get('magnitude', 0):.3f}  "
         f"t0={e.get('t_start_ms', 0) / 1000:.3f}s")
    if st != "start":
        s += f"  ML={e.get('ml')}({e.get('ml_conf', 0):.2f}) fused={e.get('confidence', 0):.2f}"
    if st == "end":
        s += f"  dur={e.get('duration_ms')}ms"
    s += f"  det={e.get('det_latency_ms', 0):.1f}ms"
    if e.get("flags"):
        s += "  [" + ",".join(e["flags"]) + "]"
    return c(s, "event", color)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    add_port_args(ap)
    ap.add_argument("--demo", action="store_true", help="run SIM DEMO (simulator source only)")
    ap.add_argument("--period", type=int, default=1000, help="metrics period [ms]")
    ap.add_argument("--log", help="directory for CSV/JSONL logs (uses data_logger)")
    ap.add_argument("--no-color", action="store_true")
    ap.add_argument("--duration", type=float, default=0, help="exit after N seconds (0 = run until Ctrl+C)")
    a = ap.parse_args()
    color = not a.no_color and sys.stdout.isatty()

    link = open_from_args(a)
    logger = None
    if a.log:
        from data_logger import ExperimentLogger
        logger = ExperimentLogger(a.log, link)
    print(f"connected to {link.name}. Type commands (HELP), Ctrl+C to quit.")
    link.send(f"METRICS {a.period}")
    link.send("FORMAT JSON")
    if a.demo:
        link.send("SIM DEMO")

    def stdin_loop():
        for line in sys.stdin:
            if line.strip():
                link.send(line)

    threading.Thread(target=stdin_loop, daemon=True).start()
    t_end = time.time() + a.duration if a.duration else None
    try:
        for m in link.messages():
            if t_end and time.time() > t_end:
                break
            if logger:
                logger.handle(m)
            t = m.get("type")
            if t == "metrics":
                print(fmt_metrics(m, color), flush=True)
            elif t == "event":
                print(fmt_event(m, color), flush=True)
            elif t == "log":
                print(c(f"[{m.get('level')}] {m.get('tag')}: {m.get('msg')}", "dim", color))
            elif t == "error":
                print(c(f"ERROR {m.get('cmd')}: {m.get('msg')}", "error", color))
            elif t == "comment":
                print(m["text"])
            elif t == "link_error":
                print(c(f"link lost: {m.get('error')}", "error", color))
                break
            elif t in ("ack",):
                print(c(f"ok {m.get('cmd')}" + (f": {m['msg']}" if m.get("msg") else ""), "dim", color))
            elif t in ("hello",):
                print(c(f"device: fw {m.get('fw')} platform {m.get('platform', '?')} ML "
                        f"{'ready' if m.get('ml_ready') else 'off'}", "dim", color))
            else:
                d = {k: v for k, v in m.items() if not k.startswith("_")}
                print(d)
    except KeyboardInterrupt:
        pass
    finally:
        if logger:
            logger.close()
        link.close()


if __name__ == "__main__":
    main()
