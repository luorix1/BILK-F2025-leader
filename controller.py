#!/usr/bin/env python3
"""
controller.py — BILK Leader
Sends joint angles (rad) + velocities over UDP
Packet format MUST match follower.py
"""

import smbus2 as smbus
import time
import socket
import struct
import signal
import argparse
import math

# ============================================================
# Hardware configuration
# ============================================================
I2C_BUS = 1
PCA9548A_ADDR = 0x70
AS5600_ADDR = 0x36

AS5600_REG_RAW_ANGLE_HI = 0x0C
AS5600_REG_RAW_ANGLE_LO = 0x0D

RAD_PER_COUNT = 2.0 * math.pi / 4096.0

# ============================================================
# BILK protocol
# ============================================================
BILK_PRE = b"BILK"
BILK_VERSION = 0x01
BILK_MSG_LEADER_STATE = 0x01

# ============================================================
# Networking
# ============================================================
HOST_IP = "172.26.7.204"   # follower IP
HOST_PORT = 9001

SAMPLE_RATE_HZ = 500
DT = 1.0 / SAMPLE_RATE_HZ

# ============================================================
# Utilities
# ============================================================
def crc16_ccitt_false(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= (byte << 8)
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def wrap_to_pi(x):
    return x - 2.0 * math.pi * math.floor((x + math.pi) / (2.0 * math.pi))


# ============================================================
# AS5600 encoder
# ============================================================
class AS5600Encoder:
    def __init__(self, bus, mux_channel):
        self.bus = bus
        self.channel = mux_channel
        self.last_angle = 0.0

    def select(self):
        self.bus.write_byte(PCA9548A_ADDR, 1 << self.channel)
        time.sleep(0.001)

    def read_raw(self):
        data = self.bus.read_i2c_block_data(AS5600_ADDR, AS5600_REG_RAW_ANGLE_HI, 2)
        return ((data[0] & 0x0F) << 8) | data[1]

    def read_angle(self):
        try:
            self.select()
            raw = self.read_raw()
            angle = raw * RAD_PER_COUNT
            self.last_angle = angle
            return True, angle
        except Exception:
            return False, self.last_angle


# ============================================================
# Leader
# ============================================================
class BILKLeader:
    def __init__(self, debug=False):
        self.debug = debug
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        if not debug:
            self.bus = smbus.SMBus(I2C_BUS)
            self.encoders = [AS5600Encoder(self.bus, i) for i in range(4)]
        else:
            self.encoders = [None] * 4

        self.last_angles = [0.0] * 4
        self.last_vel = [0.0] * 4
        self.angle_offsets = [0.0] * 4
        self.last_time = time.time()

    # --------------------------------------------------------
    def calibrate(self):
        print("[Leader] Calibrating encoders (2 s)… keep arm still")
        samples = [[] for _ in range(4)]
        t0 = time.time()

        while time.time() - t0 < 2.0:
            for i, enc in enumerate(self.encoders):
                ok, ang = enc.read_angle()
                if ok:
                    samples[i].append(ang)
            time.sleep(0.01)

        for i in range(4):
            if samples[i]:
                self.angle_offsets[i] = sum(samples[i]) / len(samples[i])
            else:
                self.angle_offsets[i] = 0.0

            print(f"  joint {i}: offset = {self.angle_offsets[i]:.6f} rad")

    # --------------------------------------------------------
    def read_encoders(self):
        now = time.time()
        dt = now - self.last_time
        self.last_time = now

        for i, enc in enumerate(self.encoders):
            ok, ang = enc.read_angle()
            if ok:
                ang = wrap_to_pi(ang - self.angle_offsets[i])
                vel = (ang - self.last_angles[i]) / dt if dt > 0 else 0.0
                self.last_angles[i] = ang
                self.last_vel[i] = vel

    # --------------------------------------------------------
    def build_frame(self):
        timestamp_us = int(time.time() * 1e6)

        payload = struct.pack("<Q", timestamp_us)
        payload += struct.pack("<ffff", *self.last_angles)
        payload += struct.pack("<ffff", *self.last_vel)
        payload += struct.pack("B", 0)      # buttons
        payload += b"\x00\x00\x00"           # padding

        header = struct.pack("BBH",
                             BILK_VERSION,
                             BILK_MSG_LEADER_STATE,
                             len(payload))

        crc = crc16_ccitt_false(header + payload)

        return BILK_PRE + header + payload + struct.pack("<H", crc)

    # --------------------------------------------------------
    def run(self):
        if not self.debug:
            self.calibrate()

        print(f"[Leader] Sending UDP → {HOST_IP}:{HOST_PORT} @ {SAMPLE_RATE_HZ} Hz")

        try:
            while True:
                t0 = time.time()

                if not self.debug:
                    self.read_encoders()

                frame = self.build_frame()
                self.socket.sendto(frame, (HOST_IP, HOST_PORT))

                dt = time.time() - t0
                sleep_t = DT - dt
                if sleep_t > 0:
                    time.sleep(sleep_t)

        except KeyboardInterrupt:
            print("\n[Leader] Stopped")


# ============================================================
# Main
# ============================================================
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--debug", action="store_true")
    args = parser.parse_args()

    leader = BILKLeader(debug=args.debug)
    leader.run()


if __name__ == "__main__":
    main()
