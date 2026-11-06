#!/usr/bin/env python3
"""
Test exact unpacking of SportModeState and LowState from rosbag2_2026_10_01-16_49_33.
"""
import sqlite3
import struct
import numpy as np

DB3 = "/mnt/d/glass-research/rosbag2_2026_10_01-16_49_33/rosbag2_2026_10_01-16_49_33_0.db3"
conn = sqlite3.connect(f"file:{DB3}?mode=ro", uri=True)
cur = conn.cursor()

# ─── 1. SportModeState ───────────────────────────────────────────────────────
cur.execute("SELECT id FROM topics WHERE name='/sportmodestate'")
tid = cur.fetchone()[0]
cur.execute("SELECT timestamp, data FROM messages WHERE topic_id=? LIMIT 1", (tid,))
b_ts, raw = cur.fetchone()

print(f"SportModeState message len: {len(raw)} bytes (CDR encapsulation 4B)")

# CDR header is 4 bytes.
# Payload starts at offset 4:
sec, nsec, err_code = struct.unpack_from('<iII', raw, 4)
quat = struct.unpack_from('<4f', raw, 16)
gyro = struct.unpack_from('<3f', raw, 32)
acc = struct.unpack_from('<3f', raw, 44)
rpy = struct.unpack_from('<3f', raw, 56)
temp = struct.unpack_from('<b', raw, 68)[0]
# Alignment: after int8 temperature, align 4 bytes: 69 -> 72
mode = struct.unpack_from('<B', raw, 72)[0]
# Align 4: 73 -> 76
progress = struct.unpack_from('<f', raw, 76)[0]
gait_type = struct.unpack_from('<B', raw, 80)[0]
# Align 4: 81 -> 84
foot_raise = struct.unpack_from('<f', raw, 84)[0]
pos = struct.unpack_from('<3f', raw, 88)
body_h = struct.unpack_from('<f', raw, 100)[0]
vel = struct.unpack_from('<3f', raw, 104)
yaw_spd = struct.unpack_from('<f', raw, 116)[0]
range_obs = struct.unpack_from('<4f', raw, 120)
foot_force = struct.unpack_from('<4h', raw, 136)
foot_pos = struct.unpack_from('<12f', raw, 144)
foot_spd = struct.unpack_from('<12f', raw, 192)

print("\n--- SportModeState Decoded ---")
print(f"  Header Stamp : {sec}.{nsec:09d} s ({sec + nsec*1e-9:.6f})")
print(f"  Gait Type    : {gait_type} (0: idle, 1: trot, 2: trot running, 3: climb stair, 4: trot obstacle)")
print(f"  Mode         : {mode}")
print(f"  Body Height  : {body_h:.3f} m")
print(f"  Position     : ({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}) m")
print(f"  Velocity     : ({vel[0]:.3f}, {vel[1]:.3f}, {vel[2]:.3f}) m/s")
print(f"  Yaw Speed    : {yaw_spd:.3f} rad/s")
print(f"  Foot Force   : {foot_force} N (FR, FL, RR, RL)")
print(f"  RPY (deg)    : ({np.rad2deg(rpy[0]):.2f}, {np.rad2deg(rpy[1]):.2f}, {np.rad2deg(rpy[2]):.2f})")

# ─── 2. LowState ─────────────────────────────────────────────────────────────
cur.execute("SELECT id FROM topics WHERE name='/lowstate'")
tid = cur.fetchone()[0]
cur.execute("SELECT timestamp, data FROM messages WHERE topic_id=? LIMIT 1", (tid,))
b_ts, raw = cur.fetchone()

print(f"\nLowState message len: {len(raw)} bytes")
# Let's inspect LowState structure:
# head: uint8[2], level_flag: uint8, frame_reserve: uint8 (4 bytes total: offset 4..8)
head = struct.unpack_from('<2B', raw, 4)
level = raw[6]
frame_res = raw[7]
sn = struct.unpack_from('<2I', raw, 8)
version = struct.unpack_from('<2I', raw, 16)
bandwidth = struct.unpack_from('<H', raw, 24)[0]
# Align 4: offset 28 -> IMUState (quat 16B, gyro 12B, acc 12B, rpy 12B, temp 1B -> 53B, align 4 -> 56B)
# Offset 28 + 56 = 84 -> MotorState[20]
# Each MotorState: mode(1B)+align(3B)=4B, q(4B), dq(4B), ddq(4B), tau_est(4B), q_raw(4B), dq_raw(4B), ddq_raw(4B), temp(1B)+align(3B)=4B, lost(4B), reserve(8B)
# MotorState item size in CDR:
# Let's verify motor state offset and tick / foot_force
print(f"  Header: 0x{head[1]:02X}{head[0]:02X}, SN: {sn}, Version: {version}, Bandwidth: {bandwidth}")

conn.close()

