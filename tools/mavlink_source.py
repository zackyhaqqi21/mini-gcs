#!/usr/bin/env python3
"""MAVLink -> uav-telemetry-protocol source for mini-gcs.

Connects to ArduPilot SITL (or any MAVLink vehicle), converts the MAVLink
messages to packets of uav-telemetry-protocol and writes the raw byte stream
to stdout, exactly where drone_sim writes it:

    python3 tools/mavlink_source.py | ./build/gcs_bridge

Mapping (MAVLink -> uavproto):
    ATTITUDE            -> MSG_ATTITUDE   roll/pitch/yaw, rad -> centi-degrees
    GLOBAL_POSITION_INT -> MSG_ALT_SPEED  relative_alt mm -> cm, hypot(vx, vy) cm/s
    SYS_STATUS          -> MSG_BATTERY    mV, percent
    HEARTBEAT           -> MSG_HEARTBEAT

Limits inherited from the protocol: altitude and speed are int16, so altitude
saturates at +-327.67 m and battery voltage at 32.767 V.

pymavlink is only needed when talking to a vehicle (pip install pymavlink).
"""

import argparse
import math
import os
import struct
import sys
import time

START_BYTE = 0xAA
MSG_HEARTBEAT, MSG_ATTITUDE, MSG_ALT_SPEED, MSG_BATTERY = 1, 2, 3, 4


# --- protocol (must match lib/protocol.cpp) ---------------------------------
def crc16_ccitt(data, crc=0xFFFF):
    """CRC-16/CCITT-FALSE. crc16_ccitt(b"123456789") == 0x29B1."""
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = (crc << 1) ^ 0x1021 if crc & 0x8000 else crc << 1
            crc &= 0xFFFF
    return crc


def encode(msg_id, seq, payload=b""):
    if len(payload) > 255:
        raise ValueError("payload too long")
    body = bytes([len(payload), msg_id, seq & 0xFF]) + bytes(payload)
    return bytes([START_BYTE]) + body + struct.pack("<H", crc16_ccitt(body))


def i16(v):
    return max(-32768, min(32767, int(round(v))))


# --- MAVLink -> payload ------------------------------------------------------
def pl_attitude(roll, pitch, yaw):  # radians in
    return struct.pack(
        "<hhh",
        i16(math.degrees(roll) * 100),
        i16(math.degrees(pitch) * 100),
        i16(math.degrees(yaw) * 100),
    )


def pl_altspeed(relative_alt_mm, vx_cms, vy_cms):
    return struct.pack("<hh", i16(relative_alt_mm / 10.0), i16(math.hypot(vx_cms, vy_cms)))


def pl_battery(mv, pct):
    return struct.pack("<hh", i16(mv), i16(max(0, pct)))


class Source:
    """Turns MAVLink messages into protocol packets, one SEQ counter for all."""

    def __init__(self):
        self.seq = 0
        self.packets = 0

    def packet(self, msg_id, payload=b""):
        pkt = encode(msg_id, self.seq, payload)
        self.seq = (self.seq + 1) & 0xFF
        self.packets += 1
        return pkt

    def convert(self, msg):
        t = msg.get_type()
        if t == "ATTITUDE":
            return self.packet(MSG_ATTITUDE, pl_attitude(msg.roll, msg.pitch, msg.yaw))
        if t == "GLOBAL_POSITION_INT":
            return self.packet(
                MSG_ALT_SPEED, pl_altspeed(msg.relative_alt, msg.vx, msg.vy))
        if t == "SYS_STATUS":
            if msg.voltage_battery == 0xFFFF:  # unknown
                return None
            return self.packet(MSG_BATTERY, pl_battery(msg.voltage_battery,
                                                       msg.battery_remaining))
        if t == "HEARTBEAT":
            return self.packet(MSG_HEARTBEAT)
        return None


# --- optional scripted flight (so the dashboard has something to show) -------
class Demo:
    """Guided takeoff, a square at constant altitude, then RTL."""

    def __init__(self, master, mavutil, alt_m, size_m, leg_s):
        self.m, self.mu = master, mavutil
        self.alt, self.size, self.leg_s = alt_m, size_m, leg_s
        self.state, self.last, self.armed, self.cur_alt, self.i = "ARM", 0.0, False, 0.0, 0

    def on_heartbeat(self, msg):
        self.armed = bool(msg.base_mode & self.mu.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)

    def on_alt(self, alt_m):
        self.cur_alt = alt_m

    def _goto(self, north, east):
        m = self.m
        m.mav.set_position_target_local_ned_send(
            0, m.target_system, m.target_component,
            self.mu.mavlink.MAV_FRAME_LOCAL_NED, 0b0000111111111000,
            north, east, -self.alt, 0, 0, 0, 0, 0, 0, 0, 0)

    def tick(self):
        now, m = time.time(), self.m
        if self.state == "ARM":
            if now - self.last < 3:
                return
            self.last = now
            if self.armed:
                m.mav.command_long_send(
                    m.target_system, m.target_component,
                    self.mu.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, self.alt)
                self.state = "CLIMB"
                log("takeoff to", self.alt, "m")
            else:
                m.set_mode("GUIDED")
                m.arducopter_arm()  # fails until GPS / pre-arm checks pass; retried
        elif self.state == "CLIMB":
            if self.cur_alt >= 0.9 * self.alt:
                self.state, self.last, self.i = "LEG", now, 0
                self._goto(*self._corner(0))
        elif self.state == "LEG":
            if now - self.last >= self.leg_s:
                self.i += 1
                self.last = now
                if self.i >= 4:
                    m.set_mode("RTL")
                    self.state = "DONE"
                    log("square done, RTL")
                else:
                    self._goto(*self._corner(self.i))

    def _corner(self, i):
        s = self.size
        return [(s, 0), (s, s), (0, s), (0, 0)][i]


def log(*a):
    print("[mavlink_source]", *a, file=sys.stderr, flush=True)


def main():
    ap = argparse.ArgumentParser(description="MAVLink to uavproto byte stream")
    ap.add_argument("--mav", default="udpin:127.0.0.1:14550",
                    help="pymavlink connection string (default: SITL via MAVProxy)")
    ap.add_argument("--duration", type=float, default=0, help="seconds, 0 = forever")
    ap.add_argument("--demo", action="store_true", help="takeoff + square + RTL")
    ap.add_argument("--alt", type=float, default=30.0)
    ap.add_argument("--size", type=float, default=40.0)
    ap.add_argument("--leg", type=float, default=14.0, help="seconds per square leg")
    args = ap.parse_args()

    try:
        from pymavlink import mavutil
    except ImportError:
        sys.exit("pymavlink is missing: pip install pymavlink  (make sitl-deps)")

    log("connecting to", args.mav)
    master = mavutil.mavlink_connection(args.mav)
    if master.wait_heartbeat(timeout=30) is None:
        sys.exit("no heartbeat in 30 s - is SITL running?")
    log("vehicle found, system", master.target_system)

    ids = mavutil.mavlink
    for msg_id, hz in ((ids.MAVLINK_MSG_ID_ATTITUDE, 25),
                       (ids.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 25),
                       (ids.MAVLINK_MSG_ID_SYS_STATUS, 2)):
        master.mav.command_long_send(
            master.target_system, master.target_component,
            ids.MAV_CMD_SET_MESSAGE_INTERVAL, 0, msg_id, int(1e6 / hz), 0, 0, 0, 0, 0)

    src = Source()
    demo = Demo(master, mavutil, args.alt, args.size, args.leg) if args.demo else None
    t0 = time.time()
    try:
        while not args.duration or time.time() - t0 < args.duration:
            if demo:
                demo.tick()
            msg = master.recv_match(blocking=True, timeout=0.5)
            if msg is None or msg.get_srcSystem() != master.target_system:
                continue
            if demo:
                t = msg.get_type()
                if t == "HEARTBEAT":
                    demo.on_heartbeat(msg)
                elif t == "GLOBAL_POSITION_INT":
                    demo.on_alt(msg.relative_alt / 1000.0)
            data = src.convert(msg)
            if data:
                sys.stdout.buffer.write(data)
                sys.stdout.buffer.flush()
    except (BrokenPipeError, KeyboardInterrupt):
        os.dup2(os.open(os.devnull, os.O_WRONLY), sys.stdout.fileno())
    log("sent", src.packets, "packets")


if __name__ == "__main__":
    main()
