#!/usr/bin/env python3
import sqlite3, struct

db3 = "/mnt/d/glass-research/rosbag2_2026_10_01-16_49_33/rosbag2_2026_10_01-16_49_33_0.db3"
c = sqlite3.connect(f"file:{db3}?mode=ro", uri=True)
tid = c.execute("SELECT id FROM topics WHERE name='/tf_static'").fetchone()[0]
data = c.execute("SELECT data FROM messages WHERE topic_id=?", (tid,)).fetchone()[0]
c.close()

print(f"Total /tf_static length: {len(data)} bytes")
for i in range(0, len(data), 16):
    chunk = data[i:i+16]
    hex_str = " ".join(f"{b:02x}" for b in chunk)
    ascii_str = "".join(chr(b) if 32 <= b <= 126 else "." for b in chunk)
    print(f"{i:03d}: {hex_str:<48s}  {ascii_str}")
