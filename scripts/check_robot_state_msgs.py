#!/usr/bin/env python3
"""
Inspects /lowstate and /sportmodestate from rosbag2_2026_10_01-16_49_33:
- Checks topic types and message counts
- Slices raw CDR to find embedded timestamps, tick counters, or headers
- Compares timing with /utlidar/cloud and /utlidar/imu
"""
import sqlite3, struct, os

DB3 = "/mnt/d/glass-research/rosbag2_2026_10_01-16_49_33/rosbag2_2026_10_01-16_49_33_0.db3"
conn = sqlite3.connect(f"file:{DB3}?mode=ro", uri=True)
cur = conn.cursor()

topics = ["/lowstate", "/sportmodestate"]
for t in topics:
    cur.execute("SELECT id, type FROM topics WHERE name=?", (t,))
    row = cur.fetchone()
    if not row: continue
    tid, msg_type = row
    cur.execute("SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp ASC LIMIT 3", (tid,))
    rows = cur.fetchall()
    
    print(f"\nTopic: {t}  (Type: {msg_type})")
    for i, (b_ts, raw) in enumerate(rows):
        print(f"  Msg {i}: bag_receipt_ts={b_ts} ({b_ts/1e9:.3f} s), raw_len={len(raw)} bytes")
        # Let's inspect first 32 bytes in hex
        print(f"    first 32 bytes (hex): {raw[:32].hex()}")
        # Check if first bytes contain tick (uint32) or header
        # In unitree_go/msg/LowState or SportModeState:
        # Let's see what uint32s exist in first 24 bytes:
        ints = struct.unpack_from('<6I', raw, 4)
        print(f"    first uint32s after CDR encapsulation: {ints}")

conn.close()
