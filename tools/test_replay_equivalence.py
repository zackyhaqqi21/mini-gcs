#!/usr/bin/env python3
"""Does replaying a black-box log show the parser exactly what it saw live?

    python3 tools/test_replay_equivalence.py [--noise 0.10] [--seconds 3]

Pipeline (all of it your own binaries, nothing mocked):

  live:    drone_sim -> bit-flip relay (like server.py) -> blackbox record -> gcs_bridge
  replay:  blackbox replay (same log)                                      -> gcs_bridge

Checks that
  1. the replayed bytes are identical to the bytes the parser saw live,
  2. gcs_bridge prints the same JSON lines (attitude / altspeed / battery / heartbeat),
  3. the final link statistics (ok, crc_errors, discarded, lost, reordered) are equal,
  4. the run was not trivial: at 10 % noise the parser really did reject packets.

Build first (make). Exit code 0 = equivalent.
"""
import argparse
import json
import os
import random
import subprocess
import sys
import tempfile
import threading
import time

ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
BUILD = os.path.join(ROOT, "build")
SIM, BRIDGE, BB = (os.path.join(BUILD, n) for n in ("drone_sim", "gcs_bridge", "blackbox"))


def flip(chunk, noise, rng):
    """Same per-byte bit-flip rule as Hub._relay in server/server.py."""
    p = noise / 16.0
    b = bytearray(chunk)
    for i in range(len(b)):
        if rng.random() < p:
            b[i] ^= 1 << rng.randrange(8)
    return bytes(b)


def run_bridge(data_source_cmd_stdout):
    """Feed a byte stream into gcs_bridge, return (non-stats lines, final stats)."""
    out = subprocess.run([BRIDGE], stdin=data_source_cmd_stdout, capture_output=True, text=True).stdout
    lines, last_stats = [], None
    for line in out.splitlines():
        m = json.loads(line)
        if m["type"] == "stats":
            last_stats = m  # intermediate stats depend on wall-clock timing; only the last one is compared
        else:
            lines.append(line)
    return lines, last_stats


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--noise", type=float, default=0.10)
    ap.add_argument("--seconds", type=float, default=3.0)
    ap.add_argument("--seed", type=int, default=7)
    a = ap.parse_args()
    for exe in (SIM, BRIDGE, BB):
        if not os.path.exists(exe):
            sys.exit(f"missing {exe} - run `make` first")

    rng = random.Random(a.seed)
    log = os.path.join(tempfile.mkdtemp(prefix="replayeq-"), "flight.ubx")

    # ---- live: sim -> relay -> record -> gcs_bridge --------------------------------
    sim = subprocess.Popen([SIM, "--noise", "0", "--rate", "100", "--duration", str(a.seconds)],
                           stdout=subprocess.PIPE)
    rec = subprocess.Popen([BB, "record", log], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                           stderr=subprocess.DEVNULL)
    seen = bytearray()  # what the parser sees live = what comes out of `record`
    live_bridge = subprocess.Popen([BRIDGE], stdin=rec.stdout, stdout=subprocess.PIPE, text=True)
    rec.stdout.close()

    def pump():
        while True:
            chunk = sim.stdout.read1(256)
            if not chunk:
                break
            chunk = flip(chunk, a.noise, rng)
            seen.extend(chunk)
            rec.stdin.write(chunk)
            rec.stdin.flush()
        rec.stdin.close()

    t = threading.Thread(target=pump)
    t.start()
    live_out = live_bridge.stdout.read()
    t.join()
    sim.wait()
    rec.wait()
    live_bridge.wait()

    live_lines, live_stats = [], None
    for line in live_out.splitlines():
        m = json.loads(line)
        if m["type"] == "stats":
            live_stats = m
        else:
            live_lines.append(line)

    # ---- replay: blackbox replay -> gcs_bridge -----------------------------------
    raw = subprocess.run([BB, "replay", log, "--speed", "20"], capture_output=True).stdout
    rp = subprocess.Popen([BB, "replay", log, "--speed", "20"], stdout=subprocess.PIPE)
    rep_lines, rep_stats = run_bridge(rp.stdout)
    rp.wait()

    # ---- verdict --------------------------------------------------------------------
    keys = ("ok", "crc_errors", "discarded", "lost", "reordered")
    checks = [
        ("replayed bytes == bytes the parser saw live", bytes(seen) == raw and len(raw) > 0),
        ("same JSON telemetry lines", live_lines == rep_lines and len(live_lines) > 0),
        ("same final link stats", live_stats and rep_stats and all(live_stats[k] == rep_stats[k] for k in keys)),
        ("run is not trivial (parser rejected packets)", bool(live_stats) and live_stats["crc_errors"] > 0
         and live_stats["ok"] > 0),
    ]
    print(f"noise {a.noise:.0%}, {len(raw)} bytes, {len(live_lines)} telemetry lines")
    print("live   stats:", {k: live_stats[k] for k in keys} if live_stats else None)
    print("replay stats:", {k: rep_stats[k] for k in keys} if rep_stats else None)
    bad = 0
    for name, ok in checks:
        print(("PASS " if ok else "FAIL ") + name)
        bad += not ok
    sys.exit(1 if bad else 0)


if __name__ == "__main__":
    main()
