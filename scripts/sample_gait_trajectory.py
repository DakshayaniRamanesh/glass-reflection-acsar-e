#!/usr/bin/env python3
"""
Inspect SportModeState and LowState across multiple operational epochs.
"""
import sqlite3
import struct
import numpy as np

DB3 = "/mnt/d/glass-research/rosbag2_2026_10_01-16_49_33/rosbag2_2026_10_01-16_49_33_0.db3"
conn = sqlite3.connect(f"file:{DB3}?mode=ro", uri=True)
cur = conn.cursor()

cur.execute("SELECT id FROM topics WHERE name='/sportmodestate'")
tid = cur.fetchone()[0]

print("--- SportModeState Samples Across Walking Trajectory ---")
for off in [0, 2000, 5000, 8000, 11000, 14000, 18000, 22000]:
    cur.execute("SELECT timestamp, data FROM messages WHERE topic_id=? LIMIT 1 OFFSET ?", (tid, off))
    row = cur.fetchone()
    if not row: continue
    b_ts, raw = row
    sec, nsec = struct.unpack_from('<iI', raw, 4)
    gait_type = struct.unpack_from('<B', raw, 80)[0]
    pos = struct.unpack_from('<3f', raw, 88)
    vel = struct.unpack_from('<3f', raw, 104)
    ff = struct.unpack_from('<4h', raw, 136)
    print(f"Msg #{off:5d} ({b_ts*1e-9 - 1790853573.0:5.1f}s): Header={sec+nsec*1e-9:.3f}s | Gait={gait_type} | Pos=({pos[0]:6.2f}, {pos[1]:6.2f}, {pos[2]:6.2f}) | Vel=({vel[0]:5.2f}, {vel[1]:5.2f}, {vel[2]:5.2f}) | FootForce={ff}")

conn.close()

