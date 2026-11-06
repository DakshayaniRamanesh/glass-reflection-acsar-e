#!/usr/bin/env python3
"""
Diagnostic script for PointCloud2 CDR layout:
Verifies exact location of is_dense in raw bytes vs official rclpy deserializer.
"""
import sqlite3
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2

DB3 = "/mnt/d/glass-research/rosbag2_2026_10_01-16_49_33/rosbag2_2026_10_01-16_49_33_0.db3"
conn = sqlite3.connect(f"file:{DB3}?mode=ro", uri=True)
cur = conn.cursor()

topics = ["/utlidar/cloud", "/utlidar/cloud_deskewed", "/lidar_points"]
for t in topics:
    cur.execute("SELECT id FROM topics WHERE name=?", (t,))
    tid = cur.fetchone()[0]
    cur.execute("SELECT data FROM messages WHERE topic_id=? ORDER BY timestamp ASC LIMIT 1", (tid,))
    raw = cur.fetchone()[0]
    
    msg = deserialize_message(raw, PointCloud2)
    print(f"\nTopic: {t}")
    print(f"  Total raw bytes : {len(raw)}")
    print(f"  msg.is_dense    : {msg.is_dense}")
    print(f"  msg.is_bigendian: {msg.is_bigendian}")
    print(f"  len(msg.data)   : {len(msg.data)}")
    
    # Check what is at len(raw) - 1 or after msg.data
    # In raw CDR, encapsulation is 4 bytes.
    # Where does data end?
    # Let's find msg.data bytes in raw
    data_bytes = bytes(msg.data)
    idx = raw.find(data_bytes[:32])
    print(f"  data start in raw: {idx}")
    end_of_data = idx + len(data_bytes)
    print(f"  data end in raw  : {end_of_data}")
    remaining = len(raw) - end_of_data
    print(f"  bytes remaining after data: {remaining}")
    if remaining > 0:
        tail = raw[end_of_data:]
        print(f"  tail bytes (hex): {tail.hex()}")
        print(f"  tail bytes (int): {[b for b in tail]}")
conn.close()
