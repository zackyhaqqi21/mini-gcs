#!/usr/bin/env python3
"""mini-gcs server (standard library only).

Starts the pipeline   build/drone_sim | build/gcs_bridge   as two child
processes, reads the JSON lines that gcs_bridge prints, and streams them to
browsers over Server-Sent Events (/events). Also serves the dashboard (/) and
lets the page change the simulated link noise (POST /noise?level=0.05).

--source sim : changing the noise restarts the pipeline (drone_sim applies it).
--source sitl: bytes from mavlink_source.py pass through a relay that flips
               random bits at the chosen level before they reach the C++
               parser, so the CRC check is exercised on real SITL data. The
               flight is NOT restarted when the level changes.
"""

import argparse
import json
import os
import queue
import random
import subprocess
import sys
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

ROOT = os.path.dirname(os.path.abspath(__file__))
BUILD = os.path.normpath(os.path.join(ROOT, "..", "build"))
SIM = os.path.join(BUILD, "drone_sim")
BRIDGE = os.path.join(BUILD, "gcs_bridge")
SITL_SRC = os.path.normpath(os.path.join(ROOT, "..", "tools", "mavlink_source.py"))


class Hub:
    """Owns the child processes and fans parsed telemetry out to subscribers."""

    def __init__(self, rate, source="sim", mav="udpin:127.0.0.1:14550", demo=False):
        self.rate = rate
        self.source = source  # "sim" (drone_sim + noisy link) or "sitl" (MAVLink)
        self.mav = mav
        self.demo = demo
        self.lock = threading.Lock()
        self.subs = set()
        self.latest = {}
        self.noise = 0.0
        self.generation = 0
        self.sim = None
        self.bridge = None

    # -- pipeline -----------------------------------------------------------
    def start(self, noise):
        with self.lock:
            self._stop_locked()
            self.generation += 1
            gen = self.generation
            self.noise = noise
            self.latest = {}
            if self.source == "sitl":
                cmd = [sys.executable, SITL_SRC, "--mav", self.mav]
                if self.demo:
                    cmd.append("--demo")
            else:
                cmd = [SIM, "--noise", f"{noise:.4f}", "--rate", str(self.rate)]
            self.sim = subprocess.Popen(cmd, stdout=subprocess.PIPE)
            if self.source == "sitl":
                # relay so we can corrupt bytes between the source and the parser
                self.bridge = subprocess.Popen(
                    [BRIDGE], stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True
                )
                threading.Thread(
                    target=self._relay, args=(self.sim, self.bridge), daemon=True
                ).start()
            else:
                self.bridge = subprocess.Popen(
                    [BRIDGE], stdin=self.sim.stdout, stdout=subprocess.PIPE, text=True
                )
                self.sim.stdout.close()  # so the bridge sees EOF / SIGPIPE works
            bridge = self.bridge
        self._publish({"type": "config", "noise": noise, "source": self.source})
        threading.Thread(target=self._reader, args=(bridge, gen), daemon=True).start()

    def _relay(self, src, dst):
        """sitl mode: copy bytes source -> parser, flipping random bits at self.noise."""
        out = dst.stdin.buffer
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
        """sitl mode: change the noise level live, without restarting the flight."""
        self.noise = level
        self._publish({"type": "config", "noise": level, "source": self.source})

    def _stop_locked(self):
        for proc in (self.sim, self.bridge):
            if proc and proc.poll() is None:
                proc.terminate()
        for proc in (self.sim, self.bridge):
            if proc:
                try:
                    proc.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    proc.kill()
        self.sim = self.bridge = None

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
        if HUB.source == "sitl":
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
            # tell the page the current noise level right away
            first = json.dumps({"type": "config", "noise": HUB.noise, "source": HUB.source})
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
    ap.add_argument("--source", choices=("sim", "sitl"), default="sim",
                    help="sim = drone_sim, sitl = ArduPilot SITL over MAVLink")
    ap.add_argument("--mav", default="udpin:127.0.0.1:14550",
                    help="pymavlink connection string for --source sitl")
    ap.add_argument("--demo", action="store_true",
                    help="with --source sitl: scripted takeoff, square, RTL")
    args = ap.parse_args()

    needed = (BRIDGE, SITL_SRC) if args.source == "sitl" else (SIM, BRIDGE)
    for exe in needed:
        if not os.path.exists(exe):
            sys.exit(f"missing {exe} - run `make` first")

    HUB = Hub(args.rate, args.source, args.mav, args.demo)
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
