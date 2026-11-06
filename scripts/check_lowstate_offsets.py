#!/usr/bin/env python3
"""
Inspect exact byte offsets in LowState raw buffer.
"""
import sqlite3, struct

DB3 = "/mnt/d/glass-research/rosbag2_2026_10_01-16_49_33/rosbag2_2026_10_01-16_49_33_0.db3"
conn = sqlite3.connect(f"file:{DB3}?mode=ro", uri=True)
cur = conn.cursor()
cur.execute("SELECT id FROM topics WHERE name='/lowstate'")
tid = cur.fetchone()[0]
cur.execute("SELECT timestamp, data FROM messages WHERE topic_id=? LIMIT 3", (tid,))
rows = cur.fetchall()

for idx, (b_ts, raw) in enumerate(rows):
    print(f"\nMsg {idx}: len={len(raw)}")
    # Header: offset 4
    head = struct.unpack_from('<2B', raw, 4)
    print(f"  head: 0x{head[1]:02X}{head[0]:02X}")
    # In LowState, MotorState[20]:
    # Each motor state has:
    # mode(1B)+pad(3B)=4B, q(f32), dq(f32), ddq(f32), tau_est(f32), q_raw(f32), dq_raw(f32), ddq_raw(f32), temp(i8)+pad(3B)=4B, lost(u32), reserve(2*u32=8B)
    # Total per motor = 4 + 7*4 + 4 + 4 + 8 = 48 bytes!
    # 20 motors = 20 * 48 = 960 bytes!
    # Let's check:
    # CDR: 4B
    # head(4B) + sn(8B) + version(8B) + bandwidth(2B) + align(2B) = 24B
    # imu_state: quat(16B) + gyro(12B) + acc(12B) + rpy(12B) + temp(1B) + align(3B) = 56B
    # Offset before motors = 4 + 24 + 56 = 84!
    # Motors: 20 * 48 = 960B
    # Offset after motors = 84 + 960 = 1044!
    # BmsState: version_high(1B)+version_low(1B)+status(1B)+soc(1B)+current(4B)+cycle(2B)+align(2B)+bq_ntc(2B)+mcu_ntc(2B)+align(2B)+cell_vol(15*2=30B)+align(2B) = 48B
    # Let's check what is at offset 1044..1180:
    print(f"  Bytes 1040..1100 (hex): {raw[1040:1100].hex()}")
    # Let's find tick: uint32 that increments continuously
conn.close()

