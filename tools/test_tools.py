"""End-to-end check of the PC tools against the PC simulator (no hardware).

    python tools/test_tools.py            (needs the host build: pq_sim)
Exercises: protocol parsing, STATUS/CONFIG/MODEL queries, event streaming
from injected disturbances, WAVE snapshots, CSV metrics, error replies.
"""
import sys
import time

from protocol import PQLink


def main() -> int:
    fails = 0

    def check(cond, what):
        nonlocal fails
        print(("  ok   " if cond else "  FAIL ") + what)
        fails += 0 if cond else 1

    with PQLink.open("sim") as link:
        st = link.query("STATUS", "status")
        check(st["running"] and st["source"] == "simulator", "STATUS: running on simulator")
        model = link.query("MODEL", "model")
        check(model["ready"], f"MODEL: TinyML ready ({model['model_bytes']} B, arena used {model['arena_used']} B)")
        cfg = link.query("CONFIG GET pq.sagThreshold", "config")
        check(abs(cfg["value"] - 0.9) < 1e-6, "CONFIG GET pq.sagThreshold = 0.9")
        try:
            link.query("CONFIG SET pq.sagThreshold 5", "config")
            check(False, "CONFIG SET out of range rejected")
        except RuntimeError:
            check(True, "CONFIG SET out of range rejected")
        link.send("METRICS 200")
        time.sleep(1.0)
        link.send("SIM sag 0.5 0.3")
        seen = {}
        metrics = 0
        sent_transient = False
        t0 = time.time()
        for m in link.messages(timeout=3.0):
            # a transient coinciding with a sag onset is (correctly) folded into
            # the sag event, so inject it separately afterwards
            if not sent_transient and time.time() - t0 > 1.0:
                link.send("SIM transient 0.8")
                sent_transient = True
            if m.get("type") == "event" and m.get("state") == "end":
                seen[m["dsp"]] = m
            if m.get("type") == "metrics":
                metrics += 1
        check("VOLTAGE_SAG" in seen, "sag event streamed (start..end)")
        check("TRANSIENT" in seen, "transient event streamed")
        if "VOLTAGE_SAG" in seen:
            e = seen["VOLTAGE_SAG"]
            check(abs(e["magnitude"] - 0.5) < 0.03 and 250 <= e["duration_ms"] <= 350,
                  f"sag depth {e['magnitude']:.3f} pu, duration {e['duration_ms']} ms")
        check(metrics >= 5, f"periodic metrics received ({metrics})")
        w = link.wave(128)
        check(len(w["v"]) == 128 and max(w["v"]) > 250, f"WAVE snapshot: {len(w['v'])} samples, peak {max(w['v']):.0f} V")
        link.send("FORMAT CSV")
        link.send("METRICS ONCE")
        got_csv = any(m.get("format") == "csv" for m in link.messages(timeout=1.0))
        check(got_csv, "CSV metrics parsed via header")
        link.send("FORMAT JSON")
        ev = link.query("EVENTS", "event_log", collect=True)
        check(len(ev) >= 2, f"EVENTS log returned {len(ev)} entries")
    print("PASS" if not fails else f"{fails} FAILED")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
