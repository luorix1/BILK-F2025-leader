#!/usr/bin/env python3

# controller.py - Raspberry Pi Leader with AS5600 Encoders

import smbus2 as smbus
import time
import socket
import struct
import signal
import sys
import argparse

# -------------------------------
# Hardware Configuration
# -------------------------------
I2C_BUS = 1
PCA9548A_ADDR = 0x70
AS5600_ADDR = 0x36
SAMPLE_RATE_HZ = 500  # Match follower control rate
SAMPLE_US = 2000      # 500 Hz = 2000 microseconds

# AS5600 Registers
AS5600_REG_RAW_ANGLE_HI = 0x0C
AS5600_REG_RAW_ANGLE_LO = 0x0D

# Protocol
BILK_PRE = b'BILK'
BILK_VERSION = 0x01
BILK_MSG_LEADER_STATE = 0x01
RAD_PER_COUNT = 2 * 3.14159265359 / 4096.0

HOST_IP = "172.26.7.204"
HOST_PORT = 9001
USB_SERIAL_PORT = "/dev/ttyUSB0"

DEBUG_PRINT_EVERY_N = 20


# ================================================================
#   AS5600 class using EXACT logic from the working minimal script
# ================================================================
class AS5600Encoder:
    def __init__(self, bus, mux_channel):
        self.bus = bus
        self.mux_channel = mux_channel
        self.last_angle = 0.0

    def select_channel(self):
        try:
            self.bus.write_byte(PCA9548A_ADDR, 1 << self.mux_channel)
            time.sleep(0.005)
            return True
        except OSError as e:
            print(f"Error selecting channel {self.mux_channel}: {e}")
            return False

    def read_raw_angle(self):
        try:
            data = self.bus.read_i2c_block_data(AS5600_ADDR, AS5600_REG_RAW_ANGLE_HI, 2)
            raw = ((data[0] & 0x0F) << 8) | data[1]
            return raw
        except OSError:
            return None

    def read_angle(self):
        if not self.select_channel():
            return False, self.last_angle

        raw = self.read_raw_angle()
        if raw is None:
            return False, self.last_angle

        angle = raw * RAD_PER_COUNT
        self.last_angle = angle
        return True, angle


# ================================================================
#                       BILK Leader
# ================================================================
class BILKLeader:
    def __init__(self, debug=False):
        self.debug = debug

        if not self.debug:
            self.bus = smbus.SMBus(I2C_BUS)
            self.encoders = [AS5600Encoder(self.bus, i) for i in range(4)]
        else:
            print("DEBUG MODE: Skipping I2C initialization & using mock encoder data.")
            self.bus = None
            self.encoders = [None] * 4

        self.last_angles = [0.0] * 4
        self.last_vel = [0.0] * 4
        self.last_time = time.time()
        self.debug_counter = 0
        self.socket = None
        self.angle_offsets = [0.0] * 4  # Calibration offsets to zero encoders at rest

    def print_debug_frame(self, frame, counter):
        pre = frame[0:4]
        version = frame[4]
        msg_type = frame[5]
        length = struct.unpack("<H", frame[6:8])[0]
        payload = frame[8:8+length]
        crc = struct.unpack("<H", frame[8+length:8+length+2])[0]

        offset = 0
        timestamp_us = struct.unpack_from("<Q", payload, offset)[0]
        offset += 8
        angles = struct.unpack_from("<ffff", payload, offset)
        offset += 16
        velocities = struct.unpack_from("<ffff", payload, offset)
        offset += 16
        buttons = payload[offset]
        offset += 1
        reserved = payload[offset:offset+3]

        print("\n----- DEBUG FRAME #{:d} -----".format(counter))
        print(f"PREAMBLE:     {pre}  ({pre.decode('ascii', errors='ignore')})")
        print(f"VERSION:      {version}")
        print(f"MSG TYPE:     {msg_type}")
        print(f"PAYLOAD LEN:  {length} bytes")
        print(f"CRC:          0x{crc:04X}\n")
        print(f"Timestamp:    {timestamp_us} us")
        print(f"Angles:       {['%.4f' % a for a in angles]}")
        print(f"Velocities:   {['%.4f' % v for v in velocities]}")
        print(f"Buttons:      {buttons}")
        print(f"Reserved:     {reserved.hex()}")
        print("-----------------------------\n")

    def crc16_ccitt_false(self, data):
        crc = 0xFFFF
        for byte in data:
            crc ^= (byte << 8)
            for _ in range(8):
                if crc & 0x8000:
                    crc = ((crc << 1) & 0xFFFF) ^ 0x1021
                else:
                    crc = (crc << 1) & 0xFFFF
        return crc

    def setup_network(self):
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            print(f"UDP -> {HOST_IP}:{HOST_PORT}")
            return True
        except Exception as e:
            print(f"UDP setup failed: {e}")
            return False

    def send_frame(self, frame):
        if self.debug:
            self.debug_counter += 1
            if self.debug_counter % DEBUG_PRINT_EVERY_N == 0:
                self.print_debug_frame(frame, self.debug_counter)

        if self.socket:
            self.socket.sendto(frame, (HOST_IP, HOST_PORT))

        # Optional USB output
        try:
            with open(USB_SERIAL_PORT, 'wb') as f:
                f.write(frame)
        except Exception:
            pass

    def calibrate_encoders(self):
        """
        Calibrate encoders by reading baseline angles for 2 seconds.
        Computes average offsets to zero encoders at rest position.
        """
        if self.debug:
            print("DEBUG MODE: Skipping encoder calibration.")
            return

        print("\n=== CALIBRATION: Reading encoder baseline for 2 seconds ===")
        print("Please keep the leader arm stationary...")

        t_start = time.time()
        samples = [[] for _ in range(4)]  # One list per encoder

        while time.time() - t_start < 2.0:
            for i, enc in enumerate(self.encoders):
                ok, angle = enc.read_angle()
                if ok:
                    samples[i].append(angle)
            time.sleep(0.01)  # Small delay to avoid overwhelming I2C bus

        # Compute average offsets
        for i in range(4):
            if len(samples[i]) > 0:
                self.angle_offsets[i] = sum(samples[i]) / len(samples[i])
                print(f"Encoder {i}: {len(samples[i])} samples, offset = {self.angle_offsets[i]:.6f} rad ({self.angle_offsets[i] * 180.0 / 3.14159265359:.3f} deg)")
            else:
                print(f"Encoder {i}: WARNING - No samples collected! Using zero offset.")
                self.angle_offsets[i] = 0.0

        print("Calibration complete.\n")

    def read_encoders(self):
        if self.debug:
            self.last_angles = [0.0] * 4
            self.last_vel = [0.0] * 4
            return

        now = time.time()
        dt = now - self.last_time
        self.last_time = now

        for i, enc in enumerate(self.encoders):
            ok, angle = enc.read_angle()
            if ok:
                # Subtract calibration offset to zero encoder at rest
                angle_zeroed = angle - self.angle_offsets[i]
                vel = (angle_zeroed - self.last_angles[i]) / dt if dt > 0 else 0.0
                self.last_vel[i] = vel
                self.last_angles[i] = angle_zeroed
            else:
                print(f"Encoder {i} read failed")

    def build_frame(self):
        timestamp_us = int(time.time() * 1e6)

        payload = struct.pack("<Q", timestamp_us)
        payload += struct.pack("<ffff", *self.last_angles)
        payload += struct.pack("<ffff", *self.last_vel)
        payload += struct.pack("B", 0)
        payload += b"\x00\x00\x00"

        header = struct.pack("BBH", BILK_VERSION, BILK_MSG_LEADER_STATE, len(payload))
        crc = self.crc16_ccitt_false(header + payload)
        return BILK_PRE + header + payload + struct.pack("<H", crc)

    def run(self):
        if not self.debug:
            print("Initializing encoders...")
            for i, enc in enumerate(self.encoders):
                ok, ang = enc.read_angle()
                print(f"Ch{i}: {'OK' if ok else 'FAIL'} -> {ang:.3f} rad")
        else:
            print("DEBUG MODE: Encoders init skipped -> mock data active.")

        # Calibrate encoders to zero at rest position
        self.calibrate_encoders()

        self.setup_network()
        print(f"Running at {SAMPLE_RATE_HZ} Hz...\n")

        cycle_dt = 1.0 / SAMPLE_RATE_HZ

        try:
            while True:
                loop_start = time.time()
                self.read_encoders()
                frame = self.build_frame()
                self.send_frame(frame)

                elapsed = time.time() - loop_start
                sleep_time = cycle_dt - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)

        except KeyboardInterrupt:
            print("\nStopping...")
            if self.socket:
                self.socket.close()


# ================================================================
# Main
# ================================================================
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--debug", action="store_true", help="Run without I2C encoders and send mock data")
    args = parser.parse_args()

    leader = BILKLeader(debug=args.debug)
    leader.run()


if __name__ == "__main__":
    main()
