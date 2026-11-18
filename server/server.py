#!/usr/bin/env python3
"""mini-gcs server (standard library only).

Starts the pipeline   build/drone_sim | build/gcs_bridge   as two child
processes, reads the JSON lines that gcs_bridge prints, and streams them to
browsers over Server-Sent Events (/events). Also serves the dashboard (/) and
lets the page change the simulated link noise (POST /noise?level=0.05), which
restarts the pipeline.
"""

import argparse
import json
import os
import queue
import subprocess
import sys
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

ROOT = os.path.dirname(os.path.abspath(__file__))
BUILD = os.path.normpath(os.path.join(ROOT, "..", "build"))
SIM = os.path.join(BUILD, "drone_sim")
BRIDGE = os.path.join(BUILD, "gcs_bridge")


class Hub:
    """Owns the child processes and fans parsed telemetry out to subscribers."""

    def __init__(self, rate):
        self.rate = rate
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
            self.sim = subprocess.Popen(
                [SIM, "--noise", f"{noise:.4f}", "--rate", str(self.rate)],
                stdout=subprocess.PIPE,
            )
            self.bridge = subprocess.Popen(
                [BRIDGE], stdin=self.sim.stdout, stdout=subprocess.PIPE, text=True
            )
            self.sim.stdout.close()  # so the bridge sees EOF / SIGPIPE works
            bridge = self.bridge
        self._publish({"type": "config", "noise": noise})
        threading.Thread(target=self._reader, args=(bridge, gen), daemon=True).start()

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
            first = json.dumps({"type": "config", "noise": HUB.noise})
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
    args = ap.parse_args()

    for exe in (SIM, BRIDGE):
        if not os.path.exists(exe):
            sys.exit(f"missing {exe} - run `make` first")

    HUB = Hub(args.rate)
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
