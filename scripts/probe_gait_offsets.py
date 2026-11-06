#!/usr/bin/env python3
"""
Decode SportModeState and LowState using carefully computed CDR offsets.

CDR alignment rules (XCDR1, little-endian):
  - Primitive types are aligned to their own size (float32->4, int16->2, uint8->1, etc.)
  - Fixed-length arrays are just N repetitions of the element type (no length prefix)
  - Structs start aligned to largest primitive within them
  - After a struct, alignment continues from the struct's end

SportModeState layout:
  [4B CDR header]
  TimeSpec stamp:
    int32 sec        @ 4  (align 4)
    uint32 nanosec   @ 8  (align 4)
  uint32 error_code  @ 12 (align 4)
  IMUState imu_state (largest member = float32, so align 4):
    float32[4] quaternion  @ 16  (16 bytes)
    float32[3] gyroscope   @ 32  (12 bytes)
    float32[3] accelerometer @ 44 (12 bytes)
    float32[3] rpy         @ 56  (12 bytes)
    int8 temperature       @ 68  (1 byte)
  -- after IMUState, next field is uint8 mode (align 1) -> @ 69
  uint8 mode             @ 69  (1 byte)
  -- next: float32 progress (align 4): 70 -> pad to 72
  float32 progress       @ 72  (4 bytes)
  -- next: uint8 gait_type (align 1) -> @ 76
  uint8 gait_type        @ 76  (1 byte)
  -- next: float32 foot_raise_height (align 4): 77 -> pad to 80
  float32 foot_raise_height @ 80 (4 bytes)
  float32[3] position    @ 84  (12 bytes)
  float32 body_height    @ 96  (4 bytes)
  float32[3] velocity    @ 100 (12 bytes)
  float32 yaw_speed      @ 112 (4 bytes)
  float32[4] range_obstacle @ 116 (16 bytes)
  int16[4] foot_force    @ 132 (8 bytes)  -- align 2, 132 is already aligned
  float32[12] foot_position_body @ 140 (48 bytes) -- align 4, 140 is aligned
  float32[12] foot_speed_body    @ 188 (48 bytes)
  TOTAL: 236 bytes payload + 4 bytes CDR header = 240 bytes

Wait - the existing test_decode_gait.py has different offsets. Let me recalculate
more carefully:

After int8 temperature @ 68:
  Offset 69 is the next available byte.
  uint8 mode needs align 1 -> offset 69? 

Actually in CDR (XCDR1), a struct's alignment is based on its largest member.
But after the LAST field of the struct, alignment padding is NOT added at the
end of the struct unless needed by the NEXT field's alignment.

So after IMUState ends at offset 69 (byte 68 is temperature, used 1 byte):
  Next field: uint8 mode (align 1) -> offset 69

BUT WAIT: The CDR spec says structs are aligned to their max-align at the
start, and the struct occupies its full size including trailing padding to its
max alignment. So IMUState has max align 4. Its raw fields end at byte 69
(relative to IMUState start at byte 16 -> local offset 53). So the struct
pads to 56 bytes, putting the next field at offset 16+56 = 72.

Let me re-examine. In ROS2 CDR (using Fast-CDR / CycloneDDS):
- Nested messages/structs ARE padded to their alignment at the end
- The alignment of a struct = max alignment of its members

IMUState:
  - All float32[N] -> align 4
  - int8 -> align 1
  - Max alignment = 4
  - Fields: 4*4 + 3*4 + 3*4 + 3*4 + 1 = 16+12+12+12+1 = 53 bytes raw
  - Padded to align 4 -> 56 bytes
  - So IMUState @ offset 16, ends at offset 16+56 = 72

Actually this depends on the serializer. In standard CDR, struct members are
individually aligned but the struct itself doesn't necessarily get trailing
padding. In ROS2 (XCDR1), nested messages DO get trailing padding for their
alignment.

Let me just try BOTH offset hypotheses by printing actual raw bytes.
"""

import sqlite3
import struct
import sys
import numpy as np

DB3 = "/mnt/d/glass-research/rosbag2_2026_10_01-16_49_33/rosbag2_2026_10_01-16_49_33_0.db3"
conn = sqlite3.connect(f"file:{DB3}?mode=ro", uri=True)
cur = conn.cursor()

# ============================================================================
# Part 1: SportModeState - Probe all possible offset hypotheses
# ============================================================================
print("=" * 80)
print("SPORTMODESTATE CDR OFFSET INVESTIGATION")
print("=" * 80)

cur.execute("SELECT id FROM topics WHERE name='/sportmodestate'")
tid = cur.fetchone()[0]

# Get 10 messages from different times
cur.execute("""
    SELECT timestamp, data FROM messages WHERE topic_id=?
    ORDER BY timestamp LIMIT 10
""", (tid,))
rows = cur.fetchall()

print(f"\nTotal message size: {len(rows[0][1])} bytes")

# Strategy: Print a hex dump of the region around where gait_type and
# foot_force should be, for ALL 10 messages.
# We know position works at offset 88 in old script -> let's check both hypotheses

# Hypothesis A: No trailing struct padding (old offsets)
#   IMUState ends at 68+1=69. mode@69, progress@72(pad), gait_type@76(pad??)
#   Actually old script had: mode@72, progress@76, gait_type@80, foot_raise@84, pos@88

# Hypothesis B: IMUState has trailing padding to align 4
#   IMUState starts@16, is 53 raw bytes, padded to 56 -> ends at 72
#   mode@72, pad to 76 for progress... wait, mode is uint8 at 72, progress is float32 needing align 4
#   So mode@72, then next align-4 is 76, progress@76, gait_type (uint8)@80
#   foot_raise_height@84(align4 from 81->84), position@88
#   This IS the same as hypothesis A for position@88!

# So the old offsets might actually be correct... Let me check what old script gets

print("\n--- Hypothesis A (from test_decode_gait.py) ---")
for i, (ts, raw) in enumerate(rows):
    sec, nsec = struct.unpack_from('<iI', raw, 4)
    mode = struct.unpack_from('<B', raw, 72)[0]
    progress = struct.unpack_from('<f', raw, 76)[0]
    gait_type = struct.unpack_from('<B', raw, 80)[0]
    foot_raise = struct.unpack_from('<f', raw, 84)[0]
    pos = struct.unpack_from('<3f', raw, 88)
    body_h = struct.unpack_from('<f', raw, 100)[0]
    vel = struct.unpack_from('<3f', raw, 104)
    yaw_spd = struct.unpack_from('<f', raw, 116)[0]
    range_obs = struct.unpack_from('<4f', raw, 120)
    foot_force = struct.unpack_from('<4h', raw, 136)
    foot_pos = struct.unpack_from('<12f', raw, 144)
    foot_spd = struct.unpack_from('<12f', raw, 192)

    print(f"  [{i:2d}] stamp={sec}.{nsec:09d}  mode={mode}  gait={gait_type}  "
          f"foot_raise={foot_raise:.3f}  pos=({pos[0]:.3f},{pos[1]:.3f},{pos[2]:.3f})  "
          f"vel=({vel[0]:.3f},{vel[1]:.3f},{vel[2]:.3f})  body_h={body_h:.3f}  "
          f"foot_force={foot_force}  range_obs={range_obs}")

# Now let me also try hypothesis B: maybe the offset for mode is 69 (no struct padding)
print("\n--- Hypothesis B: mode@69 (no struct trailing padding) ---")
for i, (ts, raw) in enumerate(rows[:3]):
    mode_b = struct.unpack_from('<B', raw, 69)[0]
    # progress needs align 4: 70 -> 72
    progress_b = struct.unpack_from('<f', raw, 72)[0]
    gait_b = struct.unpack_from('<B', raw, 76)[0]
    # foot_raise needs align 4: 77 -> 80
    foot_raise_b = struct.unpack_from('<f', raw, 80)[0]
    pos_b = struct.unpack_from('<3f', raw, 84)
    body_h_b = struct.unpack_from('<f', raw, 96)[0]
    vel_b = struct.unpack_from('<3f', raw, 100)
    yaw_spd_b = struct.unpack_from('<f', raw, 112)[0]
    range_obs_b = struct.unpack_from('<4f', raw, 116)
    foot_force_b = struct.unpack_from('<4h', raw, 132)

    print(f"  [{i:2d}] mode={mode_b}  gait={gait_b}  "
          f"foot_raise={foot_raise_b:.3f}  pos=({pos_b[0]:.3f},{pos_b[1]:.3f},{pos_b[2]:.3f})  "
          f"vel=({vel_b[0]:.3f},{vel_b[1]:.3f},{vel_b[2]:.3f})  body_h={body_h_b:.3f}  "
          f"foot_force={foot_force_b}  range_obs={range_obs_b}")

# ============================================================================
# Part 2: Raw hex dump of critical region for first 3 messages
# ============================================================================
print("\n--- Raw hex dump (bytes 60-240) for first 3 SportModeState messages ---")
for i, (ts, raw) in enumerate(rows[:3]):
    print(f"\n  Message {i} (len={len(raw)}):")
    for start in range(60, min(len(raw), 240), 16):
        end = min(start + 16, len(raw))
        hexstr = ' '.join(f'{raw[j]:02x}' for j in range(start, end))
        ascii_str = ''.join(chr(raw[j]) if 32 <= raw[j] < 127 else '.' for j in range(start, end))
        print(f"    {start:4d}: {hexstr:<48s}  {ascii_str}")

# ============================================================================
# Part 3: Check if the raw message size matches expected CDR size
# ============================================================================
print("\n--- Message size analysis ---")
# SportModeState expected CDR sizes:
# Hyp A (with struct padding): 4 + 8 + 4 + 56 + 1(+3pad) + 4 + 1(+3pad) + 4 + 12 + 4 + 12 + 4 + 16 + 8 + 48 + 48 = ?
# Let me just compute it field by field:
# CDR header: 4
# sec: 4, nsec: 4 -> 8
# error_code: 4
# IMUState: quat(16) + gyro(12) + acc(12) + rpy(12) + temp(1) + pad(3) = 56
# mode: 1 + pad(3) = 4
# progress: 4
# gait_type: 1 + pad(3) = 4
# foot_raise: 4
# position: 12
# body_height: 4
# velocity: 12
# yaw_speed: 4
# range_obstacle: 16
# foot_force: 8
# foot_position_body: 48
# foot_speed_body: 48
# Total: 4 + 8 + 4 + 56 + 4 + 4 + 4 + 4 + 12 + 4 + 12 + 4 + 16 + 8 + 48 + 48 = 240

print(f"  Expected CDR size (Hyp A): 240 bytes")
print(f"  Actual message size: {len(rows[0][1])} bytes")

# ============================================================================
# Part 4: Grab messages from DIFFERENT time epochs (early, mid, late)
# ============================================================================
print("\n" + "=" * 80)
print("SPORTMODESTATE: EARLY vs MID vs LATE COMPARISON")
print("=" * 80)

cur.execute("SELECT COUNT(*) FROM messages WHERE topic_id=?", (tid,))
total = cur.fetchone()[0]
print(f"Total SportModeState messages: {total}")

# Get early (first 10), mid (around total/2), late (last 10)
for label, query in [
    ("EARLY (first 5)", f"SELECT timestamp, data FROM messages WHERE topic_id={tid} ORDER BY timestamp LIMIT 5"),
    ("MID (around msg {})".format(total//2), f"SELECT timestamp, data FROM messages WHERE topic_id={tid} ORDER BY timestamp LIMIT 5 OFFSET {total//2}"),
    ("LATE (last 5)", f"SELECT timestamp, data FROM messages WHERE topic_id={tid} ORDER BY timestamp DESC LIMIT 5"),
]:
    print(f"\n  --- {label} ---")
    cur.execute(query)
    for ts, raw in cur.fetchall():
        sec, nsec = struct.unpack_from('<iI', raw, 4)

        # Try BOTH hypotheses for gait_type
        mode_a = struct.unpack_from('<B', raw, 72)[0]
        gait_a = struct.unpack_from('<B', raw, 80)[0]
        pos_a = struct.unpack_from('<3f', raw, 88)
        vel_a = struct.unpack_from('<3f', raw, 104)
        foot_force_a = struct.unpack_from('<4h', raw, 136)

        mode_b = struct.unpack_from('<B', raw, 69)[0]
        gait_b = struct.unpack_from('<B', raw, 76)[0]
        pos_b = struct.unpack_from('<3f', raw, 84)
        vel_b = struct.unpack_from('<3f', raw, 100)
        foot_force_b = struct.unpack_from('<4h', raw, 132)

        print(f"    stamp={sec + nsec*1e-9:.3f}  "
              f"HypA[mode={mode_a} gait={gait_a} ff={foot_force_a} vel=({vel_a[0]:.2f},{vel_a[1]:.2f},{vel_a[2]:.2f})]  "
              f"HypB[mode={mode_b} gait={gait_b} ff={foot_force_b} vel=({vel_b[0]:.2f},{vel_b[1]:.2f},{vel_b[2]:.2f})]")

# ============================================================================
# Part 5: LowState Investigation
# ============================================================================
print("\n" + "=" * 80)
print("LOWSTATE CDR OFFSET INVESTIGATION")
print("=" * 80)

cur.execute("SELECT id FROM topics WHERE name='/lf/lowstate'")
row = cur.fetchone()
if row is None:
    cur.execute("SELECT name FROM topics")
    topics = [r[0] for r in cur.fetchall()]
    lowstate_topics = [t for t in topics if 'low' in t.lower()]
    print(f"  No '/lf/lowstate' topic. Low-related topics: {lowstate_topics}")
    if lowstate_topics:
        cur.execute("SELECT id FROM topics WHERE name=?", (lowstate_topics[0],))
        row = cur.fetchone()

if row:
    tid_low = row[0]
    cur.execute("SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp LIMIT 3", (tid_low,))
    low_rows = cur.fetchall()

    print(f"\n  LowState message size: {len(low_rows[0][1])} bytes")

    # LowState CDR layout:
    # CDR header: 4
    # head: uint8[2] -> 2 bytes @ 4
    # level_flag: uint8 @ 6
    # frame_reserve: uint8 @ 7
    # sn: uint32[2] -> needs align 4: 8 @ 8 (already aligned)
    # version: uint32[2] @ 16
    # bandwidth: uint16 @ 24, pad to 28 for IMUState align 4
    # IMUState: 56 bytes @ 28 -> ends at 84
    # MotorState[20]: each MotorState has:
    #   mode: uint8 @ 0
    #   q: float32 @ 4 (align 4, pad 3)
    #   dq: float32 @ 8
    #   ddq: float32 @ 12
    #   tau_est: float32 @ 16
    #   q_raw: float32 @ 20
    #   dq_raw: float32 @ 24
    #   ddq_raw: float32 @ 28
    #   temperature: int8 @ 32
    #   lost: uint32 @ 36 (align 4, pad 3)
    #   reserve: uint32[2] @ 40
    #   Total: 48 bytes per MotorState
    # MotorState[20] = 20 * 48 = 960 bytes @ 84 -> ends at 1044
    # BmsState:
    #   version_high: uint8 @ 0
    #   version_low: uint8 @ 1
    #   status: uint8 @ 2
    #   soc: uint8 @ 3
    #   current: int32 @ 4 (align 4)
    #   cycle: uint16 @ 8
    #   bq_ntc: int8[2] @ 10
    #   mcu_ntc: int8[2] @ 12
    #   cell_vol: uint16[15] @ 14 (align 2)
    #   Total: 14 + 30 = 44 bytes. Pad to align 4 -> 44 (already aligned)
    # BmsState @ 1044, ends at 1044 + 44 = 1088
    # foot_force: int16[4] @ 1088 -> 8 bytes, ends at 1096
    # foot_force_est: int16[4] @ 1096 -> 8 bytes, ends at 1104
    # tick: uint32 @ 1104 (align 4, 1104 is aligned) -> 4 bytes, ends at 1108
    # wireless_remote: uint8[40] @ 1108 -> 40 bytes, ends at 1148
    # bit_flag: uint8 @ 1148 -> 1 byte
    # adc_reel: float32 @ 1152 (align 4, pad 3) -> 4 bytes, ends at 1156
    # temperature_ntc1: int8 @ 1156
    # temperature_ntc2: int8 @ 1157
    # power_v: float32 @ 1160 (align 4, pad 2) -> 4 bytes, ends at 1164
    # power_a: float32 @ 1164 -> 4 bytes, ends at 1168
    # fan_frequency: uint16[4] @ 1168 -> 8 bytes, ends at 1176
    # reserve: uint32 @ 1176 (align 4) -> 4 bytes, ends at 1180
    # crc: uint32 @ 1180 -> 4 bytes, ends at 1184
    # Total: 4 (CDR) + 1180 = 1184

    expected_sizes = [240, 1184]
    print(f"  Expected LowState CDR size: 1184 bytes")

    for i, (ts, raw) in enumerate(low_rows):
        head = struct.unpack_from('<2B', raw, 4)
        level = raw[6]
        frame_res = raw[7]
        sn = struct.unpack_from('<2I', raw, 8)
        version = struct.unpack_from('<2I', raw, 16)
        bandwidth = struct.unpack_from('<H', raw, 24)[0]

        # IMU at offset 28
        imu_quat = struct.unpack_from('<4f', raw, 28)
        imu_gyro = struct.unpack_from('<3f', raw, 44)
        imu_acc = struct.unpack_from('<3f', raw, 56)
        imu_rpy = struct.unpack_from('<3f', raw, 68)
        imu_temp = struct.unpack_from('<b', raw, 80)[0]

        # MotorState[0] at offset 84 (first motor, with padding)
        # Actually need to handle IMUState trailing padding:
        # IMUState ends at 28 + 53 = 81. Pad to align 4 -> 84.
        motor0_offset = 84
        m0_mode = struct.unpack_from('<B', raw, motor0_offset)[0]
        m0_q = struct.unpack_from('<f', raw, motor0_offset + 4)[0]
        m0_dq = struct.unpack_from('<f', raw, motor0_offset + 8)[0]
        m0_ddq = struct.unpack_from('<f', raw, motor0_offset + 12)[0]
        m0_tau = struct.unpack_from('<f', raw, motor0_offset + 16)[0]
        m0_temp = struct.unpack_from('<b', raw, motor0_offset + 32)[0]

        # Motor 1 at offset 84 + 48 = 132
        m1_offset = motor0_offset + 48
        m1_q = struct.unpack_from('<f', raw, m1_offset + 4)[0]
        m1_dq = struct.unpack_from('<f', raw, m1_offset + 8)[0]
        m1_tau = struct.unpack_from('<f', raw, m1_offset + 16)[0]

        # foot_force at offset 1088
        ff_offset = 84 + 20 * 48 + 44  # motors + bms
        foot_force = struct.unpack_from('<4h', raw, ff_offset)
        foot_force_est = struct.unpack_from('<4h', raw, ff_offset + 8)
        tick = struct.unpack_from('<I', raw, ff_offset + 16)[0]

        print(f"\n  [{i}] head=0x{head[1]:02X}{head[0]:02X}  SN={sn}  Ver={version}  BW={bandwidth}")
        print(f"      IMU quat={imu_quat}  rpy_deg=({np.rad2deg(imu_rpy[0]):.1f},{np.rad2deg(imu_rpy[1]):.1f},{np.rad2deg(imu_rpy[2]):.1f})")
        print(f"      IMU gyro={imu_gyro}  acc={imu_acc}  temp={imu_temp}")
        print(f"      Motor[0]: mode={m0_mode} q={m0_q:.4f} dq={m0_dq:.4f} tau={m0_tau:.4f} temp={m0_temp}")
        print(f"      Motor[1]: q={m1_q:.4f} dq={m1_dq:.4f} tau={m1_tau:.4f}")
        print(f"      foot_force={foot_force}  foot_force_est={foot_force_est}  tick={tick}")

conn.close()
print("\nDone.")

