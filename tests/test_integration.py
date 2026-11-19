#!/usr/bin/env python3
"""Integration tests for mavlink_bridge and blackbox.

usage: python3 test_integration.py ./mavlink_bridge ./blackbox

The uavproto decoder below is written from the README packet spec
(START 0xAA | LEN | MSG_ID | SEQ | PAYLOAD | CRC16-CCITT LE over LEN..PAYLOAD),
independent of protocol.cpp.
"""
import math, os, re, socket, struct, subprocess, sys, tempfile, time

BRIDGE, BLACKBOX = sys.argv[1], sys.argv[2]
CRC_EXTRA = {0: 50, 1: 124, 30: 39, 33: 104, 77: 143}
fails = []


def check(name, cond, detail=""):
    print(("PASS " if cond else "FAIL ") + name + ("" if cond else "  -> " + str(detail)))
    if not cond:
        fails.append(name)


# ---------- MAVLink frame builders ----------
def x25(data, extra):
    crc = 0xFFFF
    for b in data + bytes([extra]):
        tmp = (b ^ (crc & 0xFF)) & 0xFF
        tmp = (tmp ^ (tmp << 4)) & 0xFF
        crc = ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
    return crc


def mav2(msgid, payload, seq=0, sysid=1):
    p = payload.rstrip(b"\0") or payload[:1]  # v2 trims trailing zeros
    hdr = bytes([len(p), 0, 0, seq, sysid, 1]) + msgid.to_bytes(3, "little")
    return b"\xFD" + hdr + p + struct.pack("<H", x25(hdr + p, CRC_EXTRA[msgid]))


def mav1(msgid, payload, seq=0):
    hdr = bytes([len(payload), seq, 1, 1, msgid])
    return b"\xFE" + hdr + payload + struct.pack("<H", x25(hdr + payload, CRC_EXTRA[msgid]))


def att(r, p, y):
    return struct.pack("<Iffffff", 1000, r, p, y, 0, 0, 0)


def gpi(rel_mm, vx, vy):
    return struct.pack("<IiiiihhhH", 1000, 0, 0, 0, rel_mm, vx, vy, 0, 0)


def sysst(mv, pct):
    return struct.pack("<IIIHHhHHHHHHb", 0, 0, 0, 0, mv, 0, 0, 0, 0, 0, 0, 0, pct)


def hb(mtype):
    return struct.pack("<IBBBBB", 0, mtype, 3, 0, 4, 3)


# ---------- uavproto decoder (from the README spec) ----------
def crc16(data):
    c = 0xFFFF
    for b in data:
        c ^= b << 8
        for _ in range(8):
            c = ((c << 1) ^ 0x1021) & 0xFFFF if c & 0x8000 else (c << 1) & 0xFFFF
    return c


def decode(stream):
    out, i = [], 0
    while i < len(stream):
        if stream[i] != 0xAA:
            i += 1
            continue
        if i + 5 > len(stream):
            break
        ln = stream[i + 1]
        end = i + 4 + ln + 2
        if end > len(stream):
            break
        body = stream[i + 1:i + 4 + ln]
        if struct.unpack("<H", stream[end - 2:end])[0] == crc16(body):
            out.append((stream[i + 2], stream[i + 3], stream[i + 4:i + 4 + ln]))
            i = end
        else:
            i += 1
    return out


def cdeg(rad, wrap=False):
    d = rad * 18000 / math.pi
    if wrap:
        d = math.fmod(d, 36000)
        d = d - 36000 if d > 18000 else d
        d = d + 36000 if d <= -18000 else d
    return round(d)


# ---------- test 1: file input, mixed good / bad / unknown frames ----------
bad = bytearray(mav2(30, att(0.5, 0.5, 0.5), seq=9))
bad[12] ^= 0x40  # corrupt a payload byte
stream = b"".join([
    b"\x00\x13garbage\xFD\xFE",
    mav2(30, att(0.1, -0.05, 4.0), 1),       # yaw wraps to -130.8 deg
    bytes(bad),                              # must be rejected
    mav2(33, gpi(12340, 300, 400), 2),       # 1234 cm, 500 cm/s
    mav2(1, sysst(11100, 87), 3),
    mav2(0, hb(2), 4),                       # vehicle heartbeat
    mav2(0, hb(6), 5),                       # GCS heartbeat -> ignored
    mav2(77, b"\x01\x02\x03", 6),            # unknown id: skipped, counted
    mav1(30, att(-0.2, 0.3, 1.0), 7),        # MAVLink v1
])
with tempfile.TemporaryDirectory() as td:
    cap = os.path.join(td, "cap.bin")
    open(cap, "wb").write(stream)
    r = subprocess.run([BRIDGE, "--file", cap], capture_output=True)
pk = decode(r.stdout)
ids = [p[0] for p in pk]
check("bridge exit code 0", r.returncode == 0, r.returncode)
check("expected packet ids (ATT, ALT, BAT, HB, ATT)", ids == [2, 3, 4, 1, 2], ids)
check("seq increments from 0", [p[1] for p in pk] == list(range(len(pk))), [p[1] for p in pk])
if len(pk) == 5:
    ro, pi_, ya = struct.unpack("<hhh", pk[0][2])
    check("attitude roll/pitch", (ro, pi_) == (cdeg(0.1), cdeg(-0.05)), (ro, pi_))
    check("yaw wrapped to (-180,180]", ya == cdeg(4.0, True), ya)
    check("ALT_SPEED / BATTERY payloads are 4 bytes (int16 x2), as gcs_bridge expects", len(pk[1][2]) == 4 and len(pk[2][2]) == 4)
    alt, spd = struct.unpack("<hh", pk[1][2])
    check("altitude mm->cm, speed = hypot(vx,vy)", (alt, spd) == (1234, 500), (alt, spd))
    mv, pct = struct.unpack("<hh", pk[2][2])
    check("battery mV and percent", (mv, pct) == (11100, 87), (mv, pct))
    check("heartbeat payload empty", pk[3][2] == b"")
    check("MAVLink v1 frame converted", struct.unpack("<hhh", pk[4][2]) == (cdeg(-0.2), cdeg(0.3), cdeg(1.0, True)))
m = re.search(r"ok=(\d+) crc_bad=(\d+)", r.stderr.decode())
check("stats: 1 bad CRC counted, bad frame not delivered", m and int(m.group(2)) >= 1, r.stderr.decode())

# ---------- test 2: stdin, one byte at a time ----------
p = subprocess.Popen([BRIDGE, "--file", "-"], stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
for b in stream:
    p.stdin.write(bytes([b]))
    p.stdin.flush()
p.stdin.close()
out = p.stdout.read()
p.wait()
check("1-byte chunks give the same packets", len(pk) == 5 and decode(out) == pk, len(decode(out)))
check("unknown-id frame counted, not forwarded",
      re.search(r"unknown=(\d+)", r.stderr.decode()) and int(re.search(r"unknown=(\d+)", r.stderr.decode()).group(1)) >= 1,
      r.stderr.decode())

# ---------- test 2b: false frame starts must not swallow real frames ----------
nasty = (b"\xFD\x00\x00\x00\x00\x00\x4D\x00\x00" + b"junk" * 3   # v2 start, unknown id 77, no real frame after it
         + b"\xFE\xFF\x00\x00\x00\x00" + b"zz"                    # v1 start with LEN=255
         + mav2(30, att(0.3, 0.2, 0.1), 1) + mav2(1, sysst(9000, 10), 2))
r2 = subprocess.run([BRIDGE, "--file", "-"], input=nasty, capture_output=True)
check("false starts with large LEN do not hide the real frames",
      [x[0] for x in decode(r2.stdout)] == [2, 4], [x[0] for x in decode(r2.stdout)])

# ---------- test 2c: int16 saturation and unknown battery values ----------
r3 = subprocess.run([BRIDGE, "--file", "-"], capture_output=True,
                    input=mav2(33, gpi(400000, 0, 0), 1) + mav2(1, sysst(0xFFFF, 50), 2) + mav2(1, sysst(11100, -1), 3))
d3 = decode(r3.stdout)
check("altitude saturates at int16, unknown voltage skipped, unknown percent -> 0",
      [x[0] for x in d3] == [3, 4] and struct.unpack("<hh", d3[0][2]) == (32767, 0) and struct.unpack("<hh", d3[1][2]) == (11100, 0), d3)

# ---------- test 3: UDP ----------
port = 20000 + os.getpid() % 20000
p = subprocess.Popen([BRIDGE, "--udp", str(port)], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
time.sleep(0.4)
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
for fr in (mav2(30, att(0.2, 0.0, 0.0), 1), mav2(1, sysst(12000, 55), 2)):
    s.sendto(fr, ("127.0.0.1", port))
time.sleep(0.4)
p.terminate()
out = p.stdout.read()
p.wait()
check("UDP input converts ATTITUDE + SYS_STATUS", [x[0] for x in decode(out)] == [2, 4], decode(out))

# ---------- test 4: blackbox ----------
with tempfile.TemporaryDirectory() as td:
    log = os.path.join(td, "f.ubx")
    chunks = [out_chunk for out_chunk in (b"AAAA" * 10, b"BBBB" * 10, b"CCCC" * 10)]
    p = subprocess.Popen([BLACKBOX, "record", log], stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    for c in chunks:
        p.stdin.write(c)
        p.stdin.flush()
        time.sleep(0.2)
    p.stdin.close()
    passthru = p.stdout.read()
    p.wait()
    check("record passes bytes through unchanged", passthru == b"".join(chunks))
    info = subprocess.run([BLACKBOX, "info", log], capture_output=True, text=True).stdout
    check("info: records/bytes", "records:  3" in info and "bytes:    120" in info, info)
    t0 = time.time()
    rep = subprocess.run([BLACKBOX, "replay", log], capture_output=True)
    dt = time.time() - t0
    check("replay reproduces the exact bytes", rep.stdout == b"".join(chunks))
    check("replay keeps original timing (~0.4 s)", 0.3 < dt < 0.8, dt)
    t0 = time.time()
    rep4 = subprocess.run([BLACKBOX, "replay", log, "--speed", "4"], capture_output=True)
    dt4 = time.time() - t0
    check("--speed 4 is about 4x faster", rep4.stdout == rep.stdout and dt4 < dt / 2, (dt, dt4))
    data = open(log, "rb").read()
    open(log, "wb").write(data[:-3])  # simulate a crash while writing the last record
    rep = subprocess.run([BLACKBOX, "replay", log, "--speed", "100"], capture_output=True)
    check("truncated log: complete records replayed, exit 0",
          rep.returncode == 0 and rep.stdout == b"".join(chunks[:2]), (rep.returncode, len(rep.stdout)))
    open(log, "wb").write(b"not a log at all")
    rep = subprocess.run([BLACKBOX, "replay", log], capture_output=True)
    check("non-log file rejected", rep.returncode != 0)

print("\n%s" % ("ALL PASSED" if not fails else "FAILED: " + ", ".join(fails)))
sys.exit(1 if fails else 0)
