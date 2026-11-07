#!/usr/bin/env python3
"""
Test Physical Alignment: Check whether Hesai in /lidar_points is already forward-facing.
Compare Hesai (accumulated stationary) vs L1 (accumulated stationary).
"""

import os
import sqlite3
import numpy as np

DB_PATH = "/mnt/d/glass-research/rosbag2_2026_10_01-16_49_33/rosbag2_2026_10_01-16_49_33_0.db3"
conn = sqlite3.connect(f"file:{DB_PATH}?mode=ro", uri=True)
cur = conn.cursor()

# Get topic IDs
cur.execute("SELECT id FROM topics WHERE name='/lidar_points'")
tid_h = cur.fetchone()[0]
cur.execute("SELECT id FROM topics WHERE name='/utlidar/cloud'")
tid_l = cur.fetchone()[0]

# Accumulate 10 scans of Hesai and 30 scans of L1 during stationary start (first 2 seconds)
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

print("Accumulating stationary Hesai scans (first 10 scans)...")
cur.execute("SELECT data FROM messages WHERE topic_id=? ORDER BY timestamp ASC LIMIT 10", (tid_h,))
hesai_pts_list = []
for (raw,) in cur:
    msg = deserialize_message(raw, PointCloud2)
    # Read points using fast numpy from buffer
    # Hesai fields: x, y, z, intensity, ring, timestamp
    gen = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
    pts = np.array(list(gen), dtype=np.float32)
    hesai_pts_list.append(pts)

H_pts = np.vstack(hesai_pts_list)
print(f"Total accumulated Hesai points: {len(H_pts)}")

print("Accumulating stationary L1 scans (first 30 scans)...")
cur.execute("SELECT data FROM messages WHERE topic_id=? ORDER BY timestamp ASC LIMIT 30", (tid_l,))
l1_pts_list = []
for (raw,) in cur:
    msg = deserialize_message(raw, PointCloud2)
    gen = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
    pts = np.array(list(gen), dtype=np.float32)
    l1_pts_list.append(pts)

L_pts = np.vstack(l1_pts_list)
print(f"Total accumulated L1 points: {len(L_pts)}")

conn.close()

# Filter zeros and non-finite
def clean(pts):
    finite = np.isfinite(pts).all(axis=1)
    pts = pts[finite]
    r = np.linalg.norm(pts, axis=1)
    return pts[(r > 0.2) & (r < 15.0)]

H_clean = clean(H_pts)
L_clean = clean(L_pts)

print(f"Clean points: Hesai={len(H_clean)}, L1={len(L_clean)}")

# Check spatial bounding boxes
print("\nHesai Extents:")
print("  X:", H_clean[:, 0].min(), "to", H_clean[:, 0].max())
print("  Y:", H_clean[:, 1].min(), "to", H_clean[:, 1].max())
print("  Z:", H_clean[:, 2].min(), "to", H_clean[:, 2].max())

print("\nL1 Extents:")
print("  X:", L_clean[:, 0].min(), "to", L_clean[:, 0].max())
print("  Y:", L_clean[:, 1].min(), "to", L_clean[:, 1].max())
print("  Z:", L_clean[:, 2].min(), "to", L_clean[:, 2].max())

# Save clean accumulated clouds for calibration
np.savez_compressed("/mnt/d/glass-research/acsar_e_m1/exports/accumulated_stationary_clouds.npz",
                    hesai=H_clean, l1=L_clean)
print("Saved accumulated stationary clouds.")

