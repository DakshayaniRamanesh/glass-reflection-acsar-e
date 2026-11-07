#!/usr/bin/env python3
"""
Diagnostic investigation of:
1. Synchronization / Cross-Correlation discrepancy (5915 ms vs 5.9 ms)
2. Hesai vs IMU / L1 cross-timing
3. Gait data files in exports/
4. Candidate scene images in exports/candidate_scenes/
"""

import os
import sqlite3
import numpy as np

sport = np.load("/mnt/d/glass-research/acsar_e_m1/exports/sportmode_validated.npz")
print("Sport header stamp range:", sport["header_stamp"][0], sport["header_stamp"][-1], 
      "duration:", sport["header_stamp"][-1] - sport["header_stamp"][0])
print("Sport bag ts range:      ", sport["bag_timestamp"][0]*1e-9, sport["bag_timestamp"][-1]*1e-9)

low = np.load("/mnt/d/glass-research/acsar_e_m1/exports/lowstate_validated.npz")
print("LowState bag ts range:   ", low["bag_timestamp"][0]*1e-9, low["bag_timestamp"][-1]*1e-9)
print("LowState tick range:     ", low["tick"][0], low["tick"][-1], "delta:", low["tick"][-1] - low["tick"][0])
print("LowState foot force min/max:", low["foot_force"].min(axis=0), low["foot_force"].max(axis=0))

DB_PATH = "/mnt/d/glass-research/rosbag2_2026_10_01-16_49_33/rosbag2_2026_10_01-16_49_33_0.db3"
conn = sqlite3.connect(f"file:{DB_PATH}?mode=ro", uri=True)
cur = conn.cursor()

cur.execute("SELECT id FROM topics WHERE name='/utlidar/imu'")
tid_imu = cur.fetchone()[0]
cur.execute("SELECT min(timestamp), max(timestamp) FROM messages WHERE topic_id=?", (tid_imu,))
min_b, max_b = cur.fetchone()
print("IMU bag ts range:        ", min_b*1e-9, max_b*1e-9)

from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu
cur.execute("SELECT data FROM messages WHERE topic_id=? ORDER BY timestamp ASC LIMIT 1", (tid_imu,))
msg0 = deserialize_message(cur.fetchone()[0], Imu)
t0_imu = msg0.header.stamp.sec + msg0.header.stamp.nanosec * 1e-9
cur.execute("SELECT data FROM messages WHERE topic_id=? ORDER BY timestamp DESC LIMIT 1", (tid_imu,))
msg1 = deserialize_message(cur.fetchone()[0], Imu)
t1_imu = msg1.header.stamp.sec + msg1.header.stamp.nanosec * 1e-9
print("IMU header stamp range:  ", t0_imu, t1_imu, "duration:", t1_imu - t0_imu)

# Check Hesai
cur.execute("SELECT id FROM topics WHERE name='/lidar_points'")
tid_h = cur.fetchone()[0]
cur.execute("SELECT min(timestamp), max(timestamp) FROM messages WHERE topic_id=?", (tid_h,))
min_bh, max_bh = cur.fetchone()
print("Hesai bag ts range:      ", min_bh*1e-9, max_bh*1e-9)

from sensor_msgs.msg import PointCloud2
cur.execute("SELECT data FROM messages WHERE topic_id=? ORDER BY timestamp ASC LIMIT 1", (tid_h,))
msgh0 = deserialize_message(cur.fetchone()[0], PointCloud2)
t0_h = msgh0.header.stamp.sec + msgh0.header.stamp.nanosec * 1e-9
cur.execute("SELECT data FROM messages WHERE topic_id=? ORDER BY timestamp DESC LIMIT 1", (tid_h,))
msgh1 = deserialize_message(cur.fetchone()[0], PointCloud2)
t1_h = msgh1.header.stamp.sec + msgh1.header.stamp.nanosec * 1e-9
print("Hesai header stamp range:", t0_h, t1_h, "duration:", t1_h - t0_h)

conn.close()

