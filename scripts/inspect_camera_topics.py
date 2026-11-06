#!/usr/bin/env python3
"""
Inspect Camera Topics and Camera Info across bags 4, 5, 6.
Extract camera intrinsics and check image message structure.
"""

import os
import sys
import sqlite3
import struct
import numpy as np

BAGS = [
    ("bag4", "/mnt/d/glass-research/rosbag2_2026_10_01-16_49_33/rosbag2_2026_10_01-16_49_33_0.db3"),
    ("bag5", "/mnt/d/glass-research/rosbag2_2026_10_01-16_51_04/rosbag2_2026_10_01-16_51_04_0.db3"),
    ("bag6", "/mnt/d/glass-research/rosbag2_2026_10_01-16_52_39/rosbag2_2026_10_01-16_52_39_0.db3"),
]

for label, db_path in BAGS:
    print("=" * 80)
    print(f"CAMERA AUDIT: {label} ({os.path.basename(db_path)})")
    print("=" * 80)
    
    conn = sqlite3.connect(f"file:{db_path}?mode=ro", uri=True)
    cur = conn.cursor()
    
    cur.execute("SELECT id, name, type FROM topics WHERE name LIKE '%camera%' OR name LIKE '%image%'")
    cam_topics = cur.fetchall()
    
    for tid, name, ttype in cam_topics:
        cur.execute("SELECT count(*), min(timestamp), max(timestamp) FROM messages WHERE topic_id=?", (tid,))
        count, min_ts, max_ts = cur.fetchone()
        dur = (max_ts - min_ts) * 1e-9 if (count > 1 and max_ts and min_ts) else 0.0
        rate = (count - 1) / dur if dur > 0 else 0.0
        print(f"  Topic: {name:35s} | Type: {ttype:28s} | Count: {count:5d} | Dur: {dur:5.1f}s | Rate: {rate:5.1f} Hz")
        
        # Check first message structure
        if count > 0:
            cur.execute("SELECT data FROM messages WHERE topic_id=? LIMIT 1", (tid,))
            raw = cur.fetchone()[0]
            print(f"    First msg raw len: {len(raw)} bytes")
            
    conn.close()

