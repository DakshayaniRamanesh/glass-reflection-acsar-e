#!/usr/bin/env python3
"""
Deep investigation script addressing all reviewer feedback:
1. rclpy official deserialization vs custom CDR parser
2. Exact zero-point audit (x=y=z=0) across L1 raw, L1 deskewed, and Hesai
3. L1 ring field and point count distribution across multiple messages
4. Official tf2_msgs/TFMessage deserialization
"""
import sqlite3, struct, os, sys
import numpy as np

# Official ROS2 imports
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
from tf2_msgs.msg import TFMessage

BAG_DIR = "/mnt/d/glass-research/rosbag2_2026_10_01-16_49_33"
DB3 = f"{BAG_DIR}/rosbag2_2026_10_01-16_49_33_0.db3"

conn = sqlite3.connect(f"file:{DB3}?mode=ro", uri=True)
cur = conn.cursor()

print("=" * 70)
print("PART 1: Official TFMessage Deserialization of /tf_static")
print("=" * 70)
cur.execute("SELECT id FROM topics WHERE name='/tf_static'")
tid = cur.fetchone()[0]
cur.execute("SELECT data FROM messages WHERE topic_id=?", (tid,))
tf_raw = cur.fetchone()[0]

tf_msg = deserialize_message(tf_raw, TFMessage)
print(f"Total transforms in /tf_static: {len(tf_msg.transforms)}")
for i, t in enumerate(tf_msg.transforms):
    h = t.header
    tr = t.transform.translation
    rot = t.transform.rotation
    print(f"  Transform {i+1}:")
    print(f"    Parent: '{h.frame_id}' -> Child: '{t.child_frame_id}'")
    print(f"    Stamp : {h.stamp.sec}s + {h.stamp.nanosec}ns")
    print(f"    Trans : [{tr.x:+.6f}, {tr.y:+.6f}, {tr.z:+.6f}] m")
    print(f"    Rot   : [{rot.x:+.6f}, {rot.y:+.6f}, {rot.z:+.6f}, {rot.w:+.6f}]")

print("\n" + "=" * 70)
print("PART 2: Zero-Coordinate Investigation (x=y=z=0)")
print("=" * 70)

TOPICS = [
    ("/utlidar/cloud", "L1_raw"),
    ("/utlidar/cloud_deskewed", "L1_deskewed"),
    ("/lidar_points", "Hesai_XT16"),
]

for topic, label in TOPICS:
    cur.execute("SELECT id FROM topics WHERE name=?", (topic,))
    t_id = cur.fetchone()[0]
    cur.execute("SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp ASC LIMIT 5", (t_id,))
    rows = cur.fetchall()

    print(f"\n--- Topic: {topic} ({label}) ---")
    for idx, (bag_ts, raw) in enumerate(rows):
        msg = deserialize_message(raw, PointCloud2)
        n_pts = msg.height * msg.width
        
        # Parse fields from msg.data buffer using msg.fields descriptors
        # Map fields
        field_dict = {f.name: f for f in msg.fields}
        pt_step = msg.point_step
        
        # Extract x, y, z as float32
        x_off = field_dict['x'].offset
        y_off = field_dict['y'].offset
        z_off = field_dict['z'].offset
        
        # Read directly from msg.data bytearray
        data_buf = msg.data
        
        # Convert to numpy structured or buffer
        # Using numpy frombuffer with structured dtype
        dt_list = []
        for f in msg.fields:
            # 7 is FLOAT32, 8 is FLOAT64, 4 is UINT16
            if f.datatype == 7:
                dt_list.append((f.name, '<f4'))
            elif f.datatype == 8:
                dt_list.append((f.name, '<f8'))
            elif f.datatype == 4:
                dt_list.append((f.name, '<u2'))
            elif f.datatype == 2:
                dt_list.append((f.name, '<u1'))
            elif f.datatype == 5:
                dt_list.append((f.name, '<i4'))
            else:
                dt_list.append((f.name, f'V{f.count}'))
        
        # Note: structured array with offsets
        formats = []
        offsets = []
        names = []
        for f in msg.fields:
            names.append(f.name)
            offsets.append(f.offset)
            if f.datatype == 7: formats.append('<f4')
            elif f.datatype == 8: formats.append('<f8')
            elif f.datatype == 4: formats.append('<u2')
            elif f.datatype == 2: formats.append('<u1')
            elif f.datatype == 5: formats.append('<i4')
            else: formats.append('V1')

        st_dt = np.dtype({'names': names, 'formats': formats, 'offsets': offsets, 'itemsize': pt_step})
        arr = np.frombuffer(data_buf[:n_pts * pt_step], dtype=st_dt)

        x = arr['x']
        y = arr['y']
        z = arr['z']

        # Exact zeros
        exact_zero = (x == 0.0) & (y == 0.0) & (z == 0.0)
        n_exact_zero = int(np.sum(exact_zero))
        pct_exact_zero = 100.0 * n_exact_zero / n_pts if n_pts else 0

        # Near zero (|xyz| < 1e-4)
        near_zero = (np.abs(x) < 1e-4) & (np.abs(y) < 1e-4) & (np.abs(z) < 1e-4) & (~exact_zero)
        n_near_zero = int(np.sum(near_zero))

        # NaN / Inf
        nan_pts = np.isnan(x) | np.isnan(y) | np.isnan(z)
        n_nan = int(np.sum(nan_pts))
        inf_pts = np.isinf(x) | np.isinf(y) | np.isinf(z)
        n_inf = int(np.sum(inf_pts))

        # Valid non-zero points
        valid = (~exact_zero) & (~nan_pts) & (~inf_pts)
        n_valid = int(np.sum(valid))
        pct_valid = 100.0 * n_valid / n_pts if n_pts else 0

        print(f"  Scan {idx}: Total={n_pts:5d} | Exact Zero={n_exact_zero:5d} ({pct_exact_zero:5.1f}%) | "
              f"Valid Non-Zero={n_valid:5d} ({pct_valid:5.1f}%) | Near-Zero={n_near_zero} | NaN={n_nan} | Inf={n_inf}")

        # Check intensity of exact zero points vs valid points
        if 'intensity' in names:
            inten = arr['intensity']
            if n_exact_zero > 0:
                zero_inten = inten[exact_zero]
                print(f"    -> Exact zero points intensity: min={zero_inten.min():.2f}, max={zero_inten.max():.2f}, mean={zero_inten.mean():.2f}")
            if n_valid > 0:
                valid_inten = inten[valid]
                print(f"    -> Valid points intensity: min={valid_inten.min():.2f}, max={valid_inten.max():.2f}, mean={valid_inten.mean():.2f}")

print("\n" + "=" * 70)
print("PART 3: L1 Ring and Point Count Audit Across the Bag")
print("=" * 70)
cur.execute("SELECT id FROM topics WHERE name='/utlidar/cloud'")
l1_tid = cur.fetchone()[0]

# Sample 20 messages across the bag
cur.execute("SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp ASC", (l1_tid,))
all_l1 = cur.fetchall()

print(f"Total messages in /utlidar/cloud: {len(all_l1)}")
l1_counts = []
l1_rings_all = set()

for i in range(0, len(all_l1), len(all_l1)//15):
    ts, raw = all_l1[i]
    msg = deserialize_message(raw, PointCloud2)
    n = msg.height * msg.width
    l1_counts.append(n)
    
    # Check ring
    # L1 ring offset is 20, type uint16
    st_dt = np.dtype({'names': ['ring'], 'formats': ['<u2'], 'offsets': [20], 'itemsize': msg.point_step})
    arr = np.frombuffer(msg.data[:n*msg.point_step], dtype=st_dt)
    unique_rings = np.unique(arr['ring'])
    l1_rings_all.update(unique_rings)

print(f"L1 Point counts across sampled messages: {l1_counts}")
print(f"L1 Point count min={min(l1_counts)}, max={max(l1_counts)}, mean={np.mean(l1_counts):.1f}")
print(f"L1 Unique rings across all sampled messages: {sorted(list(l1_rings_all))}")

conn.close()
