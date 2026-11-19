#!/usr/bin/env python3
"""mini-gcs server (standard library only).

Starts a pipeline of child processes, reads the JSON lines that gcs_bridge
prints, and streams them to browsers over Server-Sent Events (/events). Also
serves the dashboard (/) and lets the page change the simulated link noise
(POST /noise?level=0.05).

Sources (--source):
  sim    drone_sim (noisy link built in). Changing the noise restarts the pipeline.
  sitl   tools/mavlink_source.py (pymavlink) -> bit-flip relay -> gcs_bridge.
         Changing the noise does NOT restart the flight.
  mav    build/mavlink_bridge (C++, no pymavlink) reading MAVLink from UDP
         (--udp, default 14550) or a serial port (--serial/--baud) -> relay -> gcs_bridge.
  replay build/blackbox replay FILE (--replay FILE, --speed, --loop) -> gcs_bridge.
         The recording already contains its noise, so the noise buttons are off.

--record FILE puts `blackbox record FILE` right in front of the parser, so the
log holds exactly the bytes the parser saw (noise included). With --source sim,
a noise change restarts the pipeline and starts a new log (FILE-2.ubx, ...).
"""

import argparse
import json
import os
import queue
import random
import signal
import subprocess
import sys
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

ROOT = os.path.dirname(os.path.abspath(__file__))
BUILD = os.path.normpath(os.path.join(ROOT, "..", "build"))
SIM = os.path.join(BUILD, "drone_sim")
BRIDGE = os.path.join(BUILD, "gcs_bridge")
MAVBRIDGE = os.path.join(BUILD, "mavlink_bridge")
BLACKBOX = os.path.join(BUILD, "blackbox")
SITL_SRC = os.path.normpath(os.path.join(ROOT, "..", "tools", "mavlink_source.py"))

RELAY_SOURCES = ("sitl", "mav")  # sources whose bytes pass through the bit-flip relay


class Hub:
    """Owns the child processes and fans parsed telemetry out to subscribers."""

    def __init__(self, args):
        self.rate = args.rate
        self.source = args.source
        self.mav = args.mav
        self.demo = args.demo
        self.udp = args.udp
        self.serial = args.serial
        self.baud = args.baud
        self.record = args.record
        self.replay = args.replay
        self.speed = args.speed
        self.loop = args.loop
        self.lock = threading.Lock()
        self.subs = set()
        self.latest = {}
        self.noise = 0.0
        self.generation = 0
        self.sim = None  # the source process
        self.rec = None  # blackbox record (optional)
        self.bridge = None

    def config(self):
        return {
            "type": "config",
            "noise": self.noise,
            "source": self.source,
            "speed": self.speed if self.source == "replay" else None,
            "record": bool(self.record) and self.source != "replay",
        }

    # -- pipeline -----------------------------------------------------------
    def _source_cmd(self, noise):
        if self.source == "sitl":
            cmd = [sys.executable, SITL_SRC, "--mav", self.mav]
            if self.demo:
                cmd.append("--demo")
            return cmd
        if self.source == "mav":
            if self.serial:
                return [MAVBRIDGE, "--serial", self.serial, "--baud", str(self.baud)]
            return [MAVBRIDGE, "--udp", str(self.udp)]
        if self.source == "replay":
            cmd = [BLACKBOX, "replay", self.replay, "--speed", str(self.speed)]
            if self.loop:
                cmd.append("--loop")
            return cmd
        return [SIM, "--noise", f"{noise:.4f}", "--rate", str(self.rate)]

    def _record_path(self, gen):
        if not self.record or self.source == "replay":
            return None
        if gen == 1:
            return self.record
        base, ext = os.path.splitext(self.record)
        return f"{base}-{gen}{ext}"

    def start(self, noise):
        with self.lock:
            self._stop_locked()
            self.generation += 1
            gen = self.generation
            self.noise = noise
            self.latest = {}
            relay = self.source in RELAY_SOURCES

            # source -> [relay] -> [blackbox record] -> gcs_bridge
            self.sim = subprocess.Popen(self._source_cmd(noise), stdout=subprocess.PIPE)
            stage_in = subprocess.PIPE if relay else self.sim.stdout
            rec_path = self._record_path(gen)
            if rec_path:
                self.rec = subprocess.Popen(
                    [BLACKBOX, "record", rec_path], stdin=stage_in, stdout=subprocess.PIPE
                )
                bridge_in, sink = self.rec.stdout, self.rec
            else:
                self.rec = None
                bridge_in, sink = stage_in, None
            self.bridge = subprocess.Popen(
                [BRIDGE], stdin=bridge_in, stdout=subprocess.PIPE, text=True
            )
            if sink is None:
                sink = self.bridge
            if relay:
                out = getattr(sink.stdin, "buffer", sink.stdin)
                threading.Thread(
                    target=self._relay, args=(self.sim, out), daemon=True
                ).start()
            else:
                self.sim.stdout.close()  # so the next stage sees EOF / SIGPIPE works
            if self.rec:
                self.rec.stdout.close()
            bridge = self.bridge
            cfg = self.config()
        self._publish(cfg)
        threading.Thread(target=self._reader, args=(bridge, gen), daemon=True).start()

    def _relay(self, src, out):
        """copy bytes source -> next stage, flipping random bits at self.noise."""
        try:
            while True:
                chunk = src.stdout.read1(256)
                if not chunk:
                    break
                p = self.noise / 16.0  # per-byte flip probability (~ per-packet level)
                if p > 0:
                    b = bytearray(chunk)
                    for i in range(len(b)):
                        if random.random() < p:
                            b[i] ^= 1 << random.randrange(8)
                    chunk = bytes(b)
                out.write(chunk)
                out.flush()
        except (BrokenPipeError, ValueError, OSError):
            pass
        finally:
            try:
                out.close()
            except Exception:
                pass

    def set_noise(self, level):
        """relay sources: change the noise level live, without restarting the flight."""
        self.noise = level
        self._publish(self.config())

    def _stop_locked(self):
        procs = (self.sim, self.rec, self.bridge)
        for proc in procs:
            if proc and proc.poll() is None:
                proc.terminate()
        for proc in procs:
            if proc:
                try:
                    proc.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    proc.kill()
        self.sim = self.rec = self.bridge = None

    def stop(self):
        with self.lock:
            self.generation += 1
            self._stop_locked()

    def _reader(self, bridge, gen):
        for line in bridge.stdout:
            if gen != self.generation:
                return
            line = line.strip()
            if not line:
                continue
            try:
                msg = json.loads(line)
            except ValueError:
                continue
            self._publish(msg)

    # -- subscribers --------------------------------------------------------
    def _publish(self, msg):
        with self.lock:
            self.latest[msg.get("type")] = msg
            subs = list(self.subs)
        data = json.dumps(msg, separators=(",", ":"))
        for q in subs:
            try:
                q.put_nowait(data)
            except queue.Full:
                pass  # slow browser: drop, the next sample replaces it anyway

    def subscribe(self):
        q = queue.Queue(maxsize=500)
        with self.lock:
            self.subs.add(q)
            snapshot = [json.dumps(m, separators=(",", ":")) for m in self.latest.values()]
        for s in snapshot:
            q.put_nowait(s)
        return q

    def unsubscribe(self, q):
        with self.lock:
            self.subs.discard(q)


HUB = None


class Handler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):  # keep the terminal quiet
        pass

    def _send(self, code, body, ctype):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        path = urlparse(self.path).path
        if path in ("/", "/index.html"):
            with open(os.path.join(ROOT, "dashboard.html"), "rb") as f:
                self._send(200, f.read(), "text/html; charset=utf-8")
        elif path == "/api/state":
            with HUB.lock:
                body = json.dumps({"noise": HUB.noise, "latest": HUB.latest}).encode()
            self._send(200, body, "application/json")
        elif path == "/events":
            self._events()
        else:
            self._send(404, b"not found", "text/plain")

    def do_POST(self):
        url = urlparse(self.path)
        if url.path != "/noise":
            self._send(404, b"not found", "text/plain")
            return
        try:
            level = float(parse_qs(url.query).get("level", ["0"])[0])
        except ValueError:
            self._send(400, b"bad level", "text/plain")
            return
        level = max(0.0, min(0.5, level))
        if HUB.source == "replay":
            self._send(409, b"noise is part of the recording", "text/plain")
            return
        if HUB.source in RELAY_SOURCES:
            HUB.set_noise(level)
        else:
            HUB.start(level)
        self._send(200, json.dumps({"noise": level}).encode(), "application/json")

    def _events(self):
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream")
        self.send_header("Cache-Control", "no-cache")
        self.send_header("Connection", "keep-alive")
        self.end_headers()
        q = HUB.subscribe()
        try:
            # tell the page the current configuration right away
            first = json.dumps(HUB.config())
            self.wfile.write(f"data: {first}\n\n".encode())
            self.wfile.flush()
            while True:
                try:
                    data = q.get(timeout=15)
                    self.wfile.write(f"data: {data}\n\n".encode())
                except queue.Empty:
                    self.wfile.write(b": keepalive\n\n")
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass
        finally:
            HUB.unsubscribe(q)


def main():
    global HUB
    ap = argparse.ArgumentParser(description="mini-gcs server")
    ap.add_argument("--port", type=int, default=8000)
    ap.add_argument("--noise", type=float, default=0.0, help="initial link noise 0..0.5")
    ap.add_argument("--rate", type=int, default=50, help="attitude packets per second")
    ap.add_argument("--source", choices=("sim", "sitl", "mav", "replay"), default="sim",
                    help="sim = drone_sim, sitl = pymavlink, mav = C++ MAVLink bridge, replay = black box log")
    ap.add_argument("--mav", default="udpin:127.0.0.1:14550",
                    help="pymavlink connection string for --source sitl")
    ap.add_argument("--demo", action="store_true",
                    help="with --source sitl: scripted takeoff, square, RTL")
    ap.add_argument("--udp", type=int, default=14550, help="--source mav: UDP port to listen on")
    ap.add_argument("--serial", help="--source mav: serial device instead of UDP (e.g. /dev/ttyUSB0)")
    ap.add_argument("--baud", type=int, default=115200, help="--source mav: serial baud rate")
    ap.add_argument("--record", metavar="FILE", help="record the bytes the parser sees (black box)")
    ap.add_argument("--replay", metavar="FILE", help="replay a black-box log (implies --source replay)")
    ap.add_argument("--speed", type=float, default=1.0, help="replay speed factor")
    ap.add_argument("--loop", action="store_true", help="repeat the replay forever")
    args = ap.parse_args()

    if args.replay:
        args.source = "replay"
    if args.source == "replay" and not args.replay:
        sys.exit("--source replay needs --replay FILE")
    if args.source == "replay" and not os.path.isfile(args.replay):
        sys.exit(f"no such log: {args.replay}")
    if args.speed <= 0:
        sys.exit("--speed must be > 0")

    needed = {
        "sim": (SIM, BRIDGE),
        "sitl": (BRIDGE, SITL_SRC),
        "mav": (MAVBRIDGE, BRIDGE),
        "replay": (BLACKBOX, BRIDGE),
    }[args.source]
    if args.record and args.source != "replay":
        needed += (BLACKBOX,)
    for exe in needed:
        if not os.path.exists(exe):
            sys.exit(f"missing {exe} - run `make` first")

    # `kill`, systemd and Popen.terminate() send SIGTERM, which would skip the
    # `finally` below and leave drone_sim / blackbox / mavlink_bridge running
    # (a recording would keep growing, a UDP port stay bound). Ctrl+C already works.
    for sig in (signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, lambda signum, frame: sys.exit(0))

    HUB = Hub(args)
    HUB.start(max(0.0, min(0.5, args.noise)))

    httpd = ThreadingHTTPServer(("127.0.0.1", args.port), Handler)
    httpd.daemon_threads = True
    print(f"mini-gcs running: http://127.0.0.1:{args.port}   (Ctrl+C to stop)")
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        HUB.stop()
        httpd.server_close()


if __name__ == "__main__":
    main()
