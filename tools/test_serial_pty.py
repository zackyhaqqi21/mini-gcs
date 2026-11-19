#!/usr/bin/env python3
"""Serial path of mavlink_bridge, tested WITHOUT hardware, over a virtual serial port (pty).

    python3 tools/test_serial_pty.py [path/to/mavlink_bridge]

What this proves: the open / termios (raw mode, baud, CLOCAL|CREAD) / read loop of `--serial`
work and MAVLink bytes arriving in small, odd-sized chunks still become correct uavproto packets.
What it cannot prove: a real USB-serial adapter, a real telemetry radio, real baud-rate mismatch
or flow-control behaviour. That still needs the radio (see HANDOFF_serial.md).
"""
import os
import struct
import subprocess
import sys
import time

BRIDGE = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "build", "mavlink_bridge")
CRC_EXTRA = {0: 50, 1: 124, 30: 39, 33: 104}
fails = []


def check(name, cond, detail=""):
    print(("PASS " if cond else "FAIL ") + name + ("" if cond else "  -> " + str(detail)))
    if not cond:
        fails.append(name)


# ---- MAVLink v2 frame builders (same as tests/test_integration.py) ----
def x25(data, extra):
    crc = 0xFFFF
    for b in data + bytes([extra]):
        tmp = (b ^ (crc & 0xFF)) & 0xFF
        tmp = (tmp ^ (tmp << 4)) & 0xFF
        crc = ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
    return crc


def mav2(msgid, payload, seq=0):
    p = payload.rstrip(b"\0") or payload[:1]
    hdr = bytes([len(p), 0, 0, seq, 1, 1]) + msgid.to_bytes(3, "little")
    return b"\xFD" + hdr + p + struct.pack("<H", x25(hdr + p, CRC_EXTRA[msgid]))


ATT = mav2(30, struct.pack("<Iffffff", 1000, 0.1, -0.05, 4.0, 0, 0, 0), 1)
GPI = mav2(33, struct.pack("<IiiiihhhH", 1000, 0, 0, 0, 12340, 300, 400, 0, 0), 2)
SYS = mav2(1, struct.pack("<IIIHHhHHHHHHb", 0, 0, 0, 0, 11100, 0, 0, 0, 0, 0, 0, 0, 87), 3)
STREAM = ATT + GPI + SYS


# ---- uavproto decoder written from the packet spec ----
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
        if struct.unpack("<H", stream[end - 2:end])[0] == crc16(stream[i + 1:i + 4 + ln]):
            out.append((stream[i + 2], stream[i + 3], stream[i + 4:i + 4 + ln]))
            i = end
        else:
            i += 1
    return out


def run_pty(baud, chunk):
    """Feed STREAM through a pty into `mavlink_bridge --serial`, in chunks of `chunk` bytes."""
    master, slave = os.openpty()
    # The pty starts in normal (cooked) mode on purpose: the bridge itself has to switch it to raw,
    # exactly as it must do on a real /dev/ttyUSB0. Nothing is written until it has had time to do so.
    dev = os.ttyname(slave)
    p = subprocess.Popen([BRIDGE, "--serial", dev, "--baud", str(baud)],
                         stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    time.sleep(0.5)
    for i in range(0, len(STREAM), chunk):
        os.write(master, STREAM[i:i + chunk])
        time.sleep(0.01)
    time.sleep(0.3)
    os.close(master)  # the "radio" goes away: the bridge must notice and exit
    try:
        out, err = p.communicate(timeout=5)
        rc = p.returncode
    except subprocess.TimeoutExpired:
        p.kill()
        out, err = p.communicate()
        rc = "timeout"
    os.close(slave)
    return rc, out, err.decode()


for baud in (57600, 115200, 230400):
    for chunk in (len(STREAM), 7, 1):
        rc, out, err = run_pty(baud, chunk)
        pk = decode(out)
        ok = [x[0] for x in pk] == [2, 3, 4]
        check(f"{baud} baud, {chunk}-byte chunks: ATTITUDE, ALT_SPEED, BATTERY converted",
              ok, [x[0] for x in pk])
        if ok and baud == 57600 and chunk == len(STREAM):
            alt, spd = struct.unpack("<hh", pk[1][2])
            mv, pct = struct.unpack("<hh", pk[2][2])
            check("values survive the serial path (1234 cm, 500 cm/s, 11100 mV, 87 %)",
                  (alt, spd, mv, pct) == (1234, 500, 11100, 87), (alt, spd, mv, pct))
        check(f"{baud} baud, {chunk}-byte chunks: bridge exits cleanly when the port closes",
              rc == 0, (rc, err.strip()[-120:]))

# error paths
r = subprocess.run([BRIDGE, "--serial", "/dev/ttyNOPE", "--baud", "57600"], capture_output=True, timeout=5)
check("missing device: exit 1, message, nothing on stdout",
      r.returncode == 1 and b"cannot open input" in r.stderr and r.stdout == b"", (r.returncode, r.stderr))
master, slave = os.openpty()
r = subprocess.run([BRIDGE, "--serial", os.ttyname(slave), "--baud", "12345"], capture_output=True, timeout=5)
check("unsupported baud rate: exit 1, message, nothing on stdout",
      r.returncode == 1 and b"cannot open input" in r.stderr and r.stdout == b"", (r.returncode, r.stderr))
os.close(master)
os.close(slave)

print("\n%s" % ("ALL PASSED" if not fails else "FAILED: " + ", ".join(fails)))
sys.exit(1 if fails else 0)
