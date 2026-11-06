#!/usr/bin/env python3
import sqlite3, struct

db3 = "/mnt/d/glass-research/rosbag2_2026_10_01-16_49_33/rosbag2_2026_10_01-16_49_33_0.db3"
c = sqlite3.connect(f"file:{db3}?mode=ro", uri=True)
tid = c.execute("SELECT id FROM topics WHERE name='/tf_static'").fetchone()[0]
data = c.execute("SELECT data FROM messages WHERE topic_id=?", (tid,)).fetchone()[0]
c.close()

idx = 0
while True:
    pos = data.find(b"camera_", idx)
    if pos == -1: break
    length = struct.unpack_from("<I", data, pos-4)[0]
    name = data[pos:pos+length].rstrip(b"\x00").decode("utf-8")
    print(f"String at {pos-4:3d}: len={length:2d}, name='{name}'")
    idx = pos + length
