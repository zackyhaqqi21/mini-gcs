#!/usr/bin/env python3
"""Round-trip checks for build/blackbox (record / replay / info).

Needs only the blackbox binary (make build/blackbox), no vehicle, no pymavlink:

    python3 tools/test_blackbox.py

A stream of real uavproto packets is damaged like the server relay does it
(random bit flips at 10 % noise, plus a few garbage bursts), fed through
`blackbox record`, and replayed. Replay must give back exactly the same bytes,
so a parser sees the same valid / CRC / lost / reordered counts as live.
"""
import os
import random
import select
import subprocess
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
BB = os.path.normpath(os.path.join(HERE, "..", "build", "blackbox"))
sys.path.insert(0, HERE)
import mavlink_source as ms  # noqa: E402

GAP = 0.005  # seconds between chunks while recording -> ~1 s log for 200 chunks


def make_chunks(n=200, noise=0.10, seed=1):
    rng = random.Random(seed)
    src = ms.Source()
    chunks = []
    p = noise / 16.0  # same per-byte flip probability as server.py's relay
    for i in range(n):
        pkt = bytearray(src.packet(ms.MSG_ATTITUDE, ms.pl_attitude(0.1, -0.05, i / 10.0)))
        for j in range(len(pkt)):
            if rng.random() < p:
                pkt[j] ^= 1 << rng.randrange(8)
        chunks.append(bytes(pkt))
        if i % 40 == 7:  # garbage burst between packets
            chunks.append(bytes(rng.randrange(256) for _ in range(rng.randrange(1, 17))))
    return chunks


def record(path, chunks):
    p = subprocess.Popen([BB, "record", path], stdin=subprocess.PIPE,
                         stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    for c in chunks:
        p.stdin.write(c)
        p.stdin.flush()
        time.sleep(GAP)
    p.stdin.close()
    out, err = p.stdout.read(), p.stderr.read()
    rc = p.wait()
    return rc, out, err.decode()


def replay(path, *args):
    t0 = time.time()
    r = subprocess.run([BB, "replay", path, *args], capture_output=True, timeout=60)
    return r.returncode, r.stdout, r.stderr.decode(), time.time() - t0


def info(path):
    r = subprocess.run([BB, "info", path], capture_output=True, text=True, timeout=20)
    fields = {}
    for line in r.stdout.splitlines():
        k, _, v = line.partition(":")
        fields[k.strip()] = v.strip()
    return r.returncode, fields


class Ctx:
    pass


C = Ctx()


def setup():
    C.dir = tempfile.mkdtemp(prefix="bbtest-")
    C.log = os.path.join(C.dir, "flight.ubx")
    C.chunks = make_chunks()
    C.original = b"".join(C.chunks)
    C.rc, C.passthrough, C.rec_err = record(C.log, C.chunks)


def test_record_passthrough():
    assert C.rc == 0, C.rec_err
    assert len(C.original) > 1000
    assert C.passthrough == C.original, "stdout of `record` differs from stdin"
    assert "recorded" in C.rec_err


def test_info_counts():
    rc, f = info(C.log)
    assert rc == 0
    assert int(f["bytes"]) == len(C.original)
    assert 1 <= int(f["records"]) <= len(C.chunks)
    dur = float(f["duration"].split()[0])
    assert 0.8 * GAP * len(C.chunks) <= dur < 5, dur
    assert f["truncated"] == "no"


def test_replay_is_byte_identical():
    rc, out, err, el = replay(C.log, "--speed", "8")
    assert rc == 0, err
    assert out != b"" and out == C.original
    C.t_fast = el


def test_replay_keeps_timing_and_speed_factor():
    rc, out, err, t1 = replay(C.log, "--speed", "1")
    assert rc == 0 and out == C.original
    _, f = info(C.log)
    dur = float(f["duration"].split()[0])
    assert t1 >= 0.9 * dur, (t1, dur)  # real time
    rc, out4, _, t4 = replay(C.log, "--speed", "4")
    assert out4 == C.original
    assert t4 < 0.5 * t1, (t4, t1)  # clearly faster


def test_truncated_log():
    cut = os.path.join(C.dir, "cut.ubx")
    data = open(C.log, "rb").read()
    open(cut, "wb").write(data[:-3])  # chop into the last record
    rc, f = info(cut)
    assert rc == 0 and f["truncated"] == "yes"
    rc, out, err, _ = replay(cut, "--speed", "8")
    assert rc == 0 and "partial record" in err
    assert out != b"" and len(out) < len(C.original)
    assert C.original.startswith(out), "replay of a cut log must be a prefix"


def test_bad_magic_and_missing_file():
    bad = os.path.join(C.dir, "bad.ubx")
    open(bad, "wb").write(b"NOTALOG!" + b"\0" * 20)
    for path in (bad, os.path.join(C.dir, "nope.ubx")):
        for cmd in ("info", "replay"):
            r = subprocess.run([BB, cmd, path], capture_output=True, timeout=10)
            assert r.returncode == 1 and r.stdout == b"", (cmd, path)


def test_loop_repeats():
    got, want = b"", 2 * len(C.original) + 1
    p = subprocess.Popen([BB, "replay", C.log, "--speed", "8", "--loop"], stdout=subprocess.PIPE)
    try:
        deadline = time.time() + 15
        while len(got) < want and time.time() < deadline:
            if select.select([p.stdout], [], [], 0.5)[0]:
                chunk = os.read(p.stdout.fileno(), 65536)
                if not chunk:
                    break
                got += chunk
    finally:
        p.kill()
        p.wait()
    assert len(got) >= want, f"only {len(got)} bytes, wanted {want}"
    n = len(C.original)
    assert got[:n] == C.original and got[n:2 * n] == C.original


def test_speed_must_be_positive():
    for bad in ("0", "-1"):
        r = subprocess.run([BB, "replay", C.log, "--speed", bad], capture_output=True, timeout=10)
        assert r.returncode == 2 and r.stdout == b""


if __name__ == "__main__":
    if not os.path.exists(BB):
        sys.exit(f"missing {BB} - run `make build/blackbox` first")
    setup()
    tests = [v for k, v in sorted(globals().items(), key=lambda kv: kv[1].__code__.co_firstlineno
                                  if callable(kv[1]) and hasattr(kv[1], "__code__") else 0)
             if k.startswith("test_")]
    failed = 0
    for t in tests:
        try:
            t()
            print(f"ok    {t.__name__}")
        except AssertionError as e:
            failed += 1
            print(f"FAIL  {t.__name__}: {e}")
    print(f"{len(tests) - failed}/{len(tests)} tests passed")
    sys.exit(1 if failed else 0)
