#!/usr/bin/env python3
"""
Extract Camera Intrinsics and Candidate Scene Images for M4 Ground-Truth.
Uses rclpy deserialization for sensor_msgs/Image and CameraInfo.
"""

import os
import sys
import sqlite3
import numpy as np
from PIL import Image

sys.stdout.reconfigure(line_buffering=True)

from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image as RosImage, CameraInfo

DB_PATH = "/mnt/d/glass-research/rosbag2_2026_10_01-16_49_33/rosbag2_2026_10_01-16_49_33_0.db3"
IMG_OUT_DIR = "/mnt/d/glass-research/acsar_e_m1/exports/candidate_scenes"
REPORT_DIR = "/mnt/d/glass-research/acsar_e_m1/reports"
os.makedirs(IMG_OUT_DIR, exist_ok=True)
os.makedirs(REPORT_DIR, exist_ok=True)

print("Opening bag:", DB_PATH)
conn = sqlite3.connect(f"file:{DB_PATH}?mode=ro", uri=True)
cur = conn.cursor()

cur.execute("SELECT id, name FROM topics")
topics = dict(cur.fetchall())
inv_topics = {v: k for k, v in topics.items()}

color_img_tid = inv_topics["/camera/color/image_raw"]
color_info_tid = inv_topics["/camera/color/camera_info"]
depth_info_tid = inv_topics["/camera/depth/camera_info"]

# 1. Camera Intrinsics
cur.execute("SELECT data FROM messages WHERE topic_id=? LIMIT 1", (color_info_tid,))
raw_info = cur.fetchone()[0]
info_msg = deserialize_message(raw_info, CameraInfo)
print("\n--- Color Camera Intrinsics ---")
print(f"  Frame ID: {info_msg.header.frame_id}")
print(f"  Resolution: {info_msg.width} x {info_msg.height}")
print(f"  Distortion Model: {info_msg.distortion_model}")
print(f"  D (coeffs): {list(info_msg.d)}")
print(f"  K (intrinsics 3x3): {list(info_msg.k)}")
print(f"  P (projection 3x4): {list(info_msg.p)}")

cur.execute("SELECT data FROM messages WHERE topic_id=? LIMIT 1", (depth_info_tid,))
raw_dinfo = cur.fetchone()[0]
dinfo_msg = deserialize_message(raw_dinfo, CameraInfo)
print("\n--- Depth Camera Intrinsics ---")
print(f"  Frame ID: {dinfo_msg.header.frame_id}")
print(f"  Resolution: {dinfo_msg.width} x {dinfo_msg.height}")
print(f"  K: {list(dinfo_msg.k)}")

# 2. Extract Candidate Scene Images across the timeline
# Get message IDs and timestamps
cur.execute("SELECT id, timestamp FROM messages WHERE topic_id=? ORDER BY timestamp ASC", (color_img_tid,))
img_rows = cur.fetchall()
total_imgs = len(img_rows)
print(f"\nTotal color images: {total_imgs}")

# Select 6 key candidate epochs:
# 0: stationary start (sitting/standing)
# 100: robot stands up and starts walking
# 250: mid-corridor traversal
# 400: passing corridor walls/partitions
# 550: approaching reflective / glass partition
# 640: stationary end near scene boundary
selected_indices = [
    (0, "scene_01_stationary_start"),
    (int(total_imgs * 0.15), "scene_02_stand_and_walk"),
    (int(total_imgs * 0.40), "scene_03_corridor_traversal"),
    (int(total_imgs * 0.60), "scene_04_wall_partitions"),
    (int(total_imgs * 0.85), "scene_05_glass_approach"),
    (total_imgs - 1, "scene_06_stationary_end"),
]

scene_metadata = []

for idx, scene_name in selected_indices:
    msg_id, b_ts = img_rows[idx]
    cur.execute("SELECT data FROM messages WHERE id=?", (msg_id,))
    raw_img = cur.fetchone()[0]
    
    ros_img = deserialize_message(raw_img, RosImage)
    h_sec = ros_img.header.stamp.sec + ros_img.header.stamp.nanosec * 1e-9
    
    w = ros_img.width
    h = ros_img.height
    enc = ros_img.encoding
    step = ros_img.step
    
    # ros_img.data is bytes
    data_bytes = bytes(ros_img.data)
    if enc == 'rgb8':
        img_np = np.frombuffer(data_bytes, dtype=np.uint8).reshape((h, w, 3))
    elif enc == 'bgr8':
        img_np = np.frombuffer(data_bytes, dtype=np.uint8).reshape((h, w, 3))[:, :, ::-1]
    else:
        print(f"Unknown encoding: {enc}")
        img_np = np.frombuffer(data_bytes, dtype=np.uint8).reshape((h, w, -1))
        
    out_path = os.path.join(IMG_OUT_DIR, f"{scene_name}.png")
    pil_img = Image.fromarray(img_np)
    pil_img.save(out_path)
    print(f"Saved candidate image {idx:3d} ({scene_name}): {out_path} [w={w}, h={h}, enc={enc}]")
    
    scene_metadata.append({
        "scene_id": scene_name,
        "image_index": idx,
        "bag_timestamp": b_ts,
        "header_stamp": h_sec,
        "width": w,
        "height": h,
        "encoding": enc,
        "path": out_path
    })

conn.close()

# Save scene metadata
print("\nExtraction complete.")

