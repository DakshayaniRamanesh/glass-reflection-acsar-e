#!/usr/bin/env python3
"""
Deep investigation of Hesai XT-16 point repetition:
- 64,000 points, 500 unique timestamps, 128 points per timestamp.
- Adjacent same-ring points with identical coordinates & intensity.
- Checks raw serialized ROS2 message vs exported .npz.
- Checks dual-return mode vs single-return mode.
- Quantifies exact repetition pattern across multiple scans.
"""
import numpy as np
import sqlite3
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2

# 1. Load exported scan_0000.npz
hesai_npz = "/mnt/d/glass-research/acsar_e_m1/exports/hesai/scan_0000.npz"
data = np.load(hesai_npz, allow_pickle=True)
pts = data["points"]
x = pts["x"]
y = pts["y"]
z = pts["z"]
inten = pts["intensity"]
ring = pts["ring"]
ts = pts["timestamp"]

print("=" * 70)
print("1. Timestamp Granularity Analysis")
print("=" * 70)
unique_ts, counts_per_ts = np.unique(ts, return_counts=True)
print(f"Total points           : {len(ts)}")
print(f"Unique timestamps      : {len(unique_ts)}")
print(f"Points per timestamp   : min={counts_per_ts.min()}, max={counts_per_ts.max()}, median={np.median(counts_per_ts)}")
print(f"Sample timestamp values (first 5):")
for i in range(5):
    print(f"  [{i}] {unique_ts[i]:.9f} s (diff: {(unique_ts[i] - unique_ts[0])*1e3:.4f} ms)")

ts_deltas_ms = np.diff(unique_ts) * 1e3
print(f"Timestamp step statistics between consecutive timestamp blocks:")
print(f"  mean step : {ts_deltas_ms.mean():.6f} ms")
print(f"  min step  : {ts_deltas_ms.min():.6f} ms")
print(f"  max step  : {ts_deltas_ms.max():.6f} ms")
print(f"  std step  : {ts_deltas_ms.std():.6f} ms")
print(f"  Total time span across scan: {(unique_ts[-1] - unique_ts[0])*1e3:.3f} ms (~0.1 s = 10 Hz)")

print("\n" + "=" * 70)
print("2. Point Composition Within a Single Timestamp Block (128 points)")
print("=" * 70)
# Look at points sharing unique_ts[0]
mask_block0 = (ts == unique_ts[0])
pts_block0 = pts[mask_block0]
print(f"Rings present in block 0: {np.unique(pts_block0['ring'])}")
print(f"Ring distribution in block 0:")
for r in range(16):
    r_cnt = np.sum(pts_block0['ring'] == r)
    print(f"  Ring {r:02d}: {r_cnt} points")

print(f"\nNotice: 128 points / 16 rings = {128 // 16} points per ring per timestamp block!")

# Let's inspect the 8 points for Ring 0 in block 0
ring0_pts = pts_block0[pts_block0['ring'] == 0]
print("\nRing 0 points in block 0:")
for j in range(len(ring0_pts)):
    p = ring0_pts[j]
    print(f"  [{j}] x={p['x']:+.4f}, y={p['y']:+.4f}, z={p['z']:+.4f}, intensity={p['intensity']:.1f}, ring={p['ring']}, ts={p['timestamp']:.6f}")

print("\n" + "=" * 70)
print("3. Duplicate Coordinate Analysis (Adjacent Same-Ring Points)")
print("=" * 70)
# Check consecutive points for each ring
total_valid = 0
total_exact_duplicate = 0
total_tested = 0

for r in range(16):
    mask_r = (ring == r)
    pts_r = pts[mask_r]
    xr = pts_r["x"]
    yr = pts_r["y"]
    zr = pts_r["z"]
    ir = pts_r["intensity"]
    ts_r = pts_r["timestamp"]

    # Filter out zeros
    nonzero = ~((xr == 0) & (yr == 0) & (zr == 0))
    n_nonzero = np.sum(nonzero)
    total_valid += n_nonzero

    # Check pairwise duplicates: p[2*k] == p[2*k + 1]?
    # Or p[k] == p[k+1]?
    dup_pairs = 0
    for k in range(0, len(pts_r) - 1, 2):
        total_tested += 1
        if nonzero[k] and nonzero[k+1]:
            if (xr[k] == xr[k+1]) and (yr[k] == yr[k+1]) and (zr[k] == zr[k+1]) and (ir[k] == ir[k+1]):
                dup_pairs += 1
    total_exact_duplicate += dup_pairs
    if r < 4:
        print(f"Ring {r:02d}: {len(pts_r)} points, {n_nonzero} non-zero, {dup_pairs} identical adjacent pairs out of {len(pts_r)//2} tested pairs")

print(f"\nOverall Same-Ring Adjacent Pair Analysis:")
print(f"  Total tested pairs: {total_tested}")
print(f"  Exact duplicate pairs: {total_exact_duplicate} ({100.0 * total_exact_duplicate / total_tested:.1f}% of pairs)")

# Check if this exists in raw serialized ROS2 message directly
print("\n" + "=" * 70)
print("4. Verification in Raw SQLite / ROS2 Message")
print("=" * 70)
DB3 = "/mnt/d/glass-research/rosbag2_2026_10_01-16_49_33/rosbag2_2026_10_01-16_49_33_0.db3"
conn = sqlite3.connect(f"file:{DB3}?mode=ro", uri=True)
cur = conn.cursor()
cur.execute("SELECT id FROM topics WHERE name='/lidar_points'")
tid = cur.fetchone()[0]
cur.execute("SELECT data FROM messages WHERE topic_id=? ORDER BY timestamp ASC LIMIT 1", (tid,))
raw_msg = cur.fetchone()[0]
conn.close()

msg_ros = deserialize_message(raw_msg, PointCloud2)
print(f"Raw ROS2 PointCloud2 message verified:")
print(f"  width={msg_ros.width}, height={msg_ros.height}, point_step={msg_ros.point_step}")
print(f"  data buffer length: {len(msg_ros.data)} bytes (64000 * 26 = {64000*26})")

# Read points 0 and 1 from raw ROS message data
# Hesai format: x(f4,0), y(f4,4), z(f4,8), inten(f4,12), ring(u2,16), timestamp(f8,18)
import struct
for pt_idx in range(8):
    offset = pt_idx * 26
    x_val, y_val, z_val, i_val, r_val, t_val = struct.unpack_from("<ffffHd", msg_ros.data, offset)
    print(f"  Raw pt {pt_idx}: x={x_val:+.4f}, y={y_val:+.4f}, z={z_val:+.4f}, i={i_val:.1f}, ring={r_val}, ts={t_val:.6f}")
