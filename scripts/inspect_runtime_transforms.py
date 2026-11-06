#!/usr/bin/env python3
"""
Inspect transforms and runtime extrinsic calibration across available bag recordings.
"""
import sqlite3
import sys

try:
    from rclpy.serialization import deserialize_message
    from tf2_msgs.msg import TFMessage
    has_rclpy = True
except ImportError:
    has_rclpy = False

print("has_rclpy:", has_rclpy)

paths = [
    "/mnt/d/glass-research/rosbag2_2026_10_01-16_49_33/rosbag2_2026_10_01-16_49_33_0.db3",
    "/mnt/d/office-full-5jqqcxzu/motion/motion_0.db3",
]

for p in paths:
    print(f"\n--- Checking: {p} ---")
    try:
        conn = sqlite3.connect(f"file:{p}?mode=ro", uri=True)
        cur = conn.cursor()
    except Exception as e:
        print("  Error opening:", e); continue

    for tname in ["/tf_static", "/tf"]:
        cur.execute("SELECT id FROM topics WHERE name=?", (tname,))
        row = cur.fetchone()
        if not row:
            print(f"  {tname}: NOT FOUND"); continue
        tid = row[0]
        cur.execute("SELECT COUNT(*) FROM messages WHERE topic_id=?", (tid,))
        cnt = cur.fetchone()[0]
        print(f"  {tname}: {cnt} messages")
        cur.execute("SELECT data FROM messages WHERE topic_id=? LIMIT 10", (tid,))
        rows = cur.fetchall()
        if has_rclpy:
            frames = set()
            details = []
            for raw in rows:
                msg = deserialize_message(raw[0], TFMessage)
                for tr in msg.transforms:
                    frames.add((tr.header.frame_id, tr.child_frame_id))
                    t = tr.transform.translation
                    r = tr.transform.rotation
                    details.append(f"    {tr.header.frame_id} -> {tr.child_frame_id}: trans=({t.x:.4f}, {t.y:.4f}, {t.z:.4f}) rot=({r.x:.4f}, {r.y:.4f}, {r.z:.4f}, {r.w:.4f})")
            print(f"    Unique frame pairs in first 10 msgs: {frames}")
            for d in details[:5]:
                print(d)
    conn.close()

