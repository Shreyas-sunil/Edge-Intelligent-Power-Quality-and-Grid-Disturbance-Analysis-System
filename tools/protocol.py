"""Host side of the edge-PQ serial protocol (see docs/communication_protocol.md).

    from protocol import PQLink
    with PQLink.open("COM5") as link:          # or "/dev/ttyACM0", or "sim"
        print(link.query("STATUS", "status"))
        for msg in link.messages(timeout=5):
            print(msg)

Port "sim" starts the PC build of the firmware (pq_sim) and talks to it over
pipes, so every tool works without hardware. The executable is looked up in
$PQ_SIM, ./build-host, ./firmware/build-host (Release/Debug subfolders too).
"""
from __future__ import annotations

import json
import os
import pathlib
import queue
import shutil
import subprocess
import sys
import threading
import time
from typing import Iterator, Optional

REPO = pathlib.Path(__file__).resolve().parents[1]
DEFAULT_BAUD = 115200


def find_sim() -> Optional[str]:
    env = os.environ.get("PQ_SIM")
    if env and pathlib.Path(env).exists():
        return env
    exe = "pq_sim.exe" if os.name == "nt" else "pq_sim"
    for base in (REPO / "build-host", REPO / "firmware" / "build-host", pathlib.Path.cwd() / "build-host"):
        for sub in ("", "Release", "RelWithDebInfo", "Debug"):
            p = base / sub / exe
            if p.exists():
                return str(p)
    return shutil.which(exe)


def parse_line(line: str, csv_header: Optional[list] = None) -> dict:
    """Convert one protocol line into a dict (JSON, CSV metrics, or comment)."""
    line = line.strip()
    if not line:
        return {}
    if line.startswith("{"):
        try:
            return json.loads(line)
        except json.JSONDecodeError:
            return {"type": "garbled", "raw": line}
    if line.startswith("# M,"):
        return {"type": "csv_header", "fields": line[2:].split(",")}
    if line.startswith("#"):
        return {"type": "comment", "text": line[1:].strip()}
    if line.startswith("M,") and csv_header:
        vals = line.split(",")
        d = {"type": "metrics", "format": "csv"}
        for k, v in zip(csv_header[1:], vals[1:]):
            try:
                d[k] = float(v) if k not in ("event", "ml_class", "q") else v
            except ValueError:
                d[k] = v
        return d
    return {"type": "text", "raw": line}


class PQLink:
    """Line-oriented connection with a background reader thread."""

    def __init__(self, reader, writer, closer, name: str):
        self._write = writer
        self._close = closer
        self.name = name
        self._q: "queue.Queue[dict]" = queue.Queue()
        self._csv_header = None
        self._alive = True
        self.raw_log = None      # optional file object receiving every raw line
        self._t = threading.Thread(target=self._reader, args=(reader,), daemon=True)
        self._t.start()

    # ------------------------------------------------------------ factories
    @classmethod
    def open(cls, port: str, baud: int = DEFAULT_BAUD, sim_args: Optional[list] = None) -> "PQLink":
        if port.lower() == "sim":
            exe = find_sim()
            if not exe:
                raise SystemExit("pq_sim not found: build the host target (see README) or set PQ_SIM")
            args = [exe] + (sim_args or [])
            proc = subprocess.Popen(args, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                                    bufsize=0)

            def rd():
                return proc.stdout.readline()

            def wr(b: bytes):
                proc.stdin.write(b)
                proc.stdin.flush()

            def cl():
                try:
                    proc.terminate()
                    proc.wait(timeout=2)
                except Exception:
                    proc.kill()

            return cls(rd, wr, cl, f"sim:{exe}")
        try:
            import serial  # pyserial
        except ImportError:
            raise SystemExit("pyserial is required: pip install pyserial")
        ser = serial.Serial(port, baudrate=baud, timeout=0.2)
        ser.reset_input_buffer()
        return cls(ser.readline, ser.write, ser.close, port)

    # ------------------------------------------------------------ plumbing
    def _reader(self, readline):
        while self._alive:
            try:
                b = readline()
            except Exception as e:  # port unplugged
                self._q.put({"type": "link_error", "error": str(e)})
                break
            if not b:
                if isinstance(b, bytes) and b == b"" and self.name.startswith("sim:"):
                    time.sleep(0.01)
                continue
            line = b.decode("utf-8", errors="replace")
            if self.raw_log:
                self.raw_log.write(line if line.endswith("\n") else line + "\n")
            msg = parse_line(line, self._csv_header)
            if msg.get("type") == "csv_header":
                self._csv_header = msg["fields"]
            if msg:
                msg.setdefault("_rx_time", time.time())
                self._q.put(msg)

    def send(self, cmd: str) -> None:
        self._write((cmd.strip() + "\n").encode())

    def get(self, timeout: float = 0.1) -> Optional[dict]:
        try:
            return self._q.get(timeout=timeout)
        except queue.Empty:
            return None

    def messages(self, timeout: Optional[float] = None) -> Iterator[dict]:
        """Yield messages until `timeout` seconds pass (forever if None)."""
        end = None if timeout is None else time.time() + timeout
        while end is None or time.time() < end:
            m = self.get(0.1)
            if m is not None:
                yield m

    def query(self, cmd: str, expect_type: str, timeout: float = 3.0, collect: bool = False):
        """Send a command and return the first reply of `expect_type` (or all
        replies of that type until the command's ack when collect=True)."""
        self.send(cmd)
        out = []
        end = time.time() + timeout
        verb = cmd.split()[0].upper()
        while time.time() < end:
            m = self.get(0.1)
            if not m:
                continue
            t = m.get("type")
            if t == "error" and m.get("cmd", "").split()[0].upper() == verb:
                raise RuntimeError(f"{cmd}: {m.get('msg')}")
            if t == expect_type:
                if not collect:
                    return m
                out.append(m)
            elif collect and t == "ack" and m.get("cmd", "").split()[0].upper() == verb:
                return out
        if collect:
            return out
        raise TimeoutError(f"no '{expect_type}' reply to '{cmd}' within {timeout}s")

    def wave(self, n: int = 512, timeout: float = 3.0) -> dict:
        """Capture a waveform snapshot -> {'fs':..., 't0_ms':..., 'v': [...], 'i': [...]}"""
        self.send(f"WAVE {n}")
        v, i, fs, t0, total = [], [], None, None, None
        end = time.time() + timeout
        while time.time() < end:
            m = self.get(0.1)
            if not m:
                continue
            if m.get("type") == "error" and m.get("cmd") == "WAVE":
                raise RuntimeError(m.get("msg"))
            if m.get("type") != "wave":
                continue
            if m["offset"] == 0:
                v, i, fs, t0, total = [], [], m["fs"], m["t0_ms"], m["n"]
            v += m["v"]
            i += m["i"]
            if total and len(v) >= total:
                return {"fs": fs, "t0_ms": t0, "v": v, "i": i}
        raise TimeoutError("waveform snapshot incomplete")

    def close(self) -> None:
        self._alive = False
        try:
            self._close()
        except Exception:
            pass

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()


def add_port_args(ap) -> None:
    ap.add_argument("--port", required=True, help="COMx, /dev/ttyACMx, or 'sim' for the PC simulator")
    ap.add_argument("--baud", type=int, default=DEFAULT_BAUD, help="UART baud (ignored for USB CDC)")
    ap.add_argument("--sim-args", default="", help="extra pq_sim arguments, e.g. \"--fast\"")


def open_from_args(a) -> PQLink:
    return PQLink.open(a.port, a.baud, a.sim_args.split() if a.sim_args else None)


if __name__ == "__main__":
    # tiny REPL: python tools/protocol.py sim
    port = sys.argv[1] if len(sys.argv) > 1 else "sim"
    with PQLink.open(port) as l:
        print(l.query("STATUS", "status"))
