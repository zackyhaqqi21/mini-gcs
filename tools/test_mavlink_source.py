#!/usr/bin/env python3
"""Checks that the Python encoder matches the packet format of protocol.hpp.
Runs without pymavlink or a vehicle:  python3 tools/test_mavlink_source.py
"""
import math
import os
import struct
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import mavlink_source as ms  # noqa: E402


class Fake:
    def __init__(self, t, **kw):
        self._t = t
        self.__dict__.update(kw)

    def get_type(self):
        return self._t


def decode(pkt):
    """Independent decoder written from the spec in protocol.hpp."""
    assert pkt[0] == 0xAA
    n = pkt[1]
    assert len(pkt) == n + 6
    crc = struct.unpack("<H", pkt[-2:])[0]
    assert crc == ms.crc16_ccitt(pkt[1:-2])
    return pkt[2], pkt[3], pkt[4:4 + n]


def test_crc_check_value():
    assert ms.crc16_ccitt(b"123456789") == 0x29B1


def test_heartbeat_layout():
    pkt = ms.encode(ms.MSG_HEARTBEAT, 7)
    assert pkt[:4] == bytes([0xAA, 0, 1, 7]) and len(pkt) == 6


def test_attitude():
    s = ms.Source()
    pkt = s.convert(Fake("ATTITUDE", roll=math.radians(1.5),
                         pitch=math.radians(-0.2), yaw=math.radians(90)))
    mid, seq, pl = decode(pkt)
    assert (mid, seq) == (ms.MSG_ATTITUDE, 0)
    assert struct.unpack("<hhh", pl) == (150, -20, 9000)


def test_altspeed_and_saturation():
    s = ms.Source()
    _, _, pl = decode(s.convert(Fake("GLOBAL_POSITION_INT",
                                     relative_alt=30000, vx=300, vy=400)))
    assert struct.unpack("<hh", pl) == (3000, 500)  # 30 m, 5 m/s
    _, _, pl = decode(s.convert(Fake("GLOBAL_POSITION_INT",
                                     relative_alt=900000, vx=0, vy=0)))
    assert struct.unpack("<hh", pl)[0] == 32767  # 327.67 m limit


def test_battery_and_unknown():
    s = ms.Source()
    _, _, pl = decode(s.convert(Fake("SYS_STATUS", voltage_battery=12600,
                                     battery_remaining=-1)))
    assert struct.unpack("<hh", pl) == (12600, 0)
    assert s.convert(Fake("SYS_STATUS", voltage_battery=0xFFFF,
                          battery_remaining=50)) is None


def test_seq_wraps_across_all_packets():
    s = ms.Source()
    seqs = [decode(s.convert(Fake("HEARTBEAT")))[1] for _ in range(257)]
    assert seqs[255] == 255 and seqs[256] == 0


if __name__ == "__main__":
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for t in tests:
        t()
    print(f"{len(tests)} tests passed")
