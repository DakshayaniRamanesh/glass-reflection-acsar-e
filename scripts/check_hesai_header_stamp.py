#!/usr/bin/env python3
"""Check first Hesai header stamp directly using rclpy deserializer."""
import sqlite3
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2

DB_PATH = "/mnt/d/glass-research/rosbag2_2026_10_01-16_49_33/rosbag2_2026_10_01-16_49_33_0.db3"
conn = sqlite3.connect(f"file:{DB_PATH}?mode=ro", uri=True)
cur = conn.cursor()

cur.execute("SELECT id FROM topics WHERE name='/lidar_points'")
tid = cur.fetchone()[0]

cur.execute("SELECT timestamp, data FROM messages WHERE topic_id=? LIMIT 1", (tid,))
b_ts, raw = cur.fetchone()

msg = deserialize_message(raw, PointCloud2)
print("Hesai ROS2 Header Stamp:", msg.header.stamp.sec, msg.header.stamp.nanosec)
print("Hesai ROS2 Header Stamp Sec:", msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
print("Bag Receipt Timestamp Sec:", b_ts * 1e-9)
conn.close()

