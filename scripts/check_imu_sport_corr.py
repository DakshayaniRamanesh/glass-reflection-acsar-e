#!/usr/bin/env python3
"""
Inspect IMU gyro_z vs SportMode yaw_speed to understand the 5915 ms correlation result.
"""

import sqlite3
import numpy as np
import scipy.signal
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu

DB_PATH = "/mnt/d/glass-research/rosbag2_2026_10_01-16_49_33/rosbag2_2026_10_01-16_49_33_0.db3"
conn = sqlite3.connect(f"file:{DB_PATH}?mode=ro", uri=True)
cur = conn.cursor()

cur.execute("SELECT id FROM topics WHERE name='/utlidar/imu'")
tid_imu = cur.fetchone()[0]
cur.execute("SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp ASC", (tid_imu,))

imu_t = []
imu_gz = []
for b_ts, raw in cur:
    msg = deserialize_message(raw, Imu)
    t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
    imu_t.append(t)
    imu_gz.append(msg.angular_velocity.z)
conn.close()

imu_t = np.array(imu_t)
imu_gz = np.array(imu_gz)

sport = np.load("/mnt/d/glass-research/acsar_e_m1/exports/sportmode_validated.npz")
sp_t = sport['header_stamp']
sp_yaw = sport['yaw_speed']

print(f"IMU points: {len(imu_t)}, Sport points: {len(sp_t)}")
print(f"IMU t0: {imu_t[0]:.6f}, Sport t0: {sp_t[0]:.6f}, diff: {(sp_t[0] - imu_t[0])*1000:.2f} ms")

# Resample to common 100 Hz grid
t_min = max(imu_t[0], sp_t[0])
t_max = min(imu_t[-1], sp_t[-1])
t_grid = np.arange(t_min, t_max, 0.01)

gz_resamp = np.interp(t_grid, imu_t, imu_gz)
yaw_resamp = np.interp(t_grid, sp_t, sp_yaw)

# Check correlation in window [-10s, +10s]
corr = scipy.signal.correlate(gz_resamp - gz_resamp.mean(), yaw_resamp - yaw_resamp.mean(), mode='full')
lags = scipy.signal.correlation_lags(len(gz_resamp), len(yaw_resamp), mode='full') * 0.01

# Find top 5 peaks
peak_idx = scipy.signal.find_peaks(corr, distance=100)[0]
top_peaks = sorted(peak_idx, key=lambda idx: corr[idx], reverse=True)[:5]
print("\nTop 5 correlation peaks:")
for idx in top_peaks:
    print(f"  Lag: {lags[idx]:.3f} s ({lags[idx]*1000:.1f} ms) | Corr value: {corr[idx]:.2f}")

# Also check correlation with negative sign (in case gyro_z is inverted)
corr_neg = scipy.signal.correlate(-gz_resamp - (-gz_resamp).mean(), yaw_resamp - yaw_resamp.mean(), mode='full')
top_peaks_neg = sorted(scipy.signal.find_peaks(corr_neg, distance=100)[0], key=lambda idx: corr_neg[idx], reverse=True)[:5]
print("\nTop 5 correlation peaks (inverted gyro):")
for idx in top_peaks_neg:
    print(f"  Lag: {lags[idx]:.3f} s ({lags[idx]*1000:.1f} ms) | Corr value: {corr_neg[idx]:.2f}")

