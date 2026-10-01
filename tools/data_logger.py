"""Record an experiment: metrics CSV, events CSV, raw protocol log, device
snapshot (status, config, model, calibration) and optional periodic waveforms.

    python tools/data_logger.py --port COM5 --out runs/ --duration 600
    python tools/data_logger.py --port sim --out runs/ --duration 30 --demo --wave-every 5

Creates runs/<timestamp>/ with:
    metrics.csv   one row per metrics message
    events.csv    one row per event message (start / classified / end)
    raw.jsonl     every line received (lossless)
    device.json   STATUS, CONFIG LIST, MODEL, CALIBRATE SHOW at start and STATUS at end
    wave_*.csv    waveform snapshots (with --wave-every)
"""
from __future__ import annotations

import argparse
import csv
import datetime as dt
import json
import pathlib
import time

from protocol import add_port_args, open_from_args

METRIC_FIELDS = ["timestamp_ms", "frame", "vrms", "irms", "frequency", "thd", "thd_i", "event", "confidence",
                 "ml_class", "ml_conf", "vrms_half", "vmin", "vmax", "h1", "h3", "h5", "h7", "p", "s", "pf", "dpf",
                 "rocof", "crest", "valid", "q", "_rx_time"]
EVENT_FIELDS = ["id", "state", "event", "dsp", "ml", "ml_conf", "confidence", "source", "t_start_ms", "duration_ms",
                "magnitude", "vrms", "frequency", "thd", "det_latency_ms", "cls_latency_ms", "components", "flags",
                "_rx_time"]


class ExperimentLogger:
    def __init__(self, out_dir: str, link, tag: str = ""):
        stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.dir = pathlib.Path(out_dir) / (stamp + (f"_{tag}" if tag else ""))
        self.dir.mkdir(parents=True, exist_ok=True)
        self.link = link
        self._mf = open(self.dir / "metrics.csv", "w", newline="")
        self._ef = open(self.dir / "events.csv", "w", newline="")
        self._raw = open(self.dir / "raw.jsonl", "w")
        self.mw = csv.DictWriter(self._mf, fieldnames=METRIC_FIELDS, extrasaction="ignore")
        self.ew = csv.DictWriter(self._ef, fieldnames=EVENT_FIELDS, extrasaction="ignore")
        self.mw.writeheader()
        self.ew.writeheader()
        self.n_metrics = self.n_events = 0
        print(f"logging to {self.dir}")

    def handle(self, m: dict) -> None:
        self._raw.write(json.dumps(m) + "\n")
        t = m.get("type")
        if t == "metrics":
            self.mw.writerow(m)
            self.n_metrics += 1
            if self.n_metrics % 10 == 0:
                self._mf.flush()
        elif t in ("event", "event_log"):
            row = dict(m)
            row["components"] = "|".join(m.get("components", []))
            row["flags"] = "|".join(m.get("flags", []))
            self.ew.writerow(row)
            self._ef.flush()
            self.n_events += 1

    def save_json(self, name: str, obj) -> None:
        (self.dir / name).write_text(json.dumps(obj, indent=1))

    def save_wave(self, w: dict) -> pathlib.Path:
        p = self.dir / f"wave_{int(w['t0_ms'])}.csv"
        with open(p, "w", newline="") as f:
            wr = csv.writer(f)
            wr.writerow(["t_ms", "v", "i"])
            for k, (v, i) in enumerate(zip(w["v"], w["i"])):
                wr.writerow([f"{w['t0_ms'] + 1000.0 * k / w['fs']:.3f}", v, i])
        return p

    def close(self) -> None:
        for f in (self._mf, self._ef, self._raw):
            f.close()
        print(f"saved {self.n_metrics} metrics rows, {self.n_events} event rows -> {self.dir}")


def snapshot(link) -> dict:
    snap = {"captured": dt.datetime.now().isoformat()}
    for cmd, typ, coll in (("STATUS", "status", False), ("MODEL", "model", False),
                           ("CALIBRATE SHOW", "cal", False), ("CONFIG LIST", "config", True)):
        try:
            snap[cmd.split()[0].lower()] = link.query(cmd, typ, timeout=5, collect=coll)
        except Exception as e:  # keep logging even if one query fails
            snap[cmd.split()[0].lower()] = {"error": str(e)}
    return snap


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    add_port_args(ap)
    ap.add_argument("--out", default="runs")
    ap.add_argument("--tag", default="", help="suffix for the run folder")
    ap.add_argument("--duration", type=float, default=60.0, help="seconds (0 = until Ctrl+C)")
    ap.add_argument("--period", type=int, default=1000, help="metrics period [ms]")
    ap.add_argument("--wave-every", type=float, default=0, help="save a waveform snapshot every N s")
    ap.add_argument("--demo", action="store_true", help="run SIM DEMO at start (simulator)")
    a = ap.parse_args()

    link = open_from_args(a)
    time.sleep(0.3)
    log = ExperimentLogger(a.out, link, a.tag)
    log.save_json("device.json", snapshot(link))
    link.send(f"METRICS {a.period}")
    link.send("FORMAT JSON")
    if a.demo:
        link.send("SIM DEMO")
    t0 = time.time()
    next_wave = t0 + a.wave_every if a.wave_every else None
    try:
        while not a.duration or time.time() - t0 < a.duration:
            m = link.get(0.1)
            if m:
                log.handle(m)
            if next_wave and time.time() >= next_wave:
                try:
                    log.save_wave(link.wave())
                except Exception as e:
                    print(f"wave capture failed: {e}")
                next_wave += a.wave_every
    except KeyboardInterrupt:
        pass
    try:
        log.save_json("device_end.json", {"status": link.query("STATUS", "status", timeout=3)})
    except Exception:
        pass
    log.close()
    link.close()


if __name__ == "__main__":
    main()
