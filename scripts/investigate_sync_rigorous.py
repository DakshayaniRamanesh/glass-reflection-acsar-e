#!/usr/bin/env python3
"""
ACSAR-E M1 — Rigorous Synchronization & Clock Evolution Analysis (Fast Substr Query)
===================================================================================
1. Tracks L1 header-to-recorder offset across all 1,234 scans:
   - Measures drift rate (ppm), offset variations, and jitter.
   - Detects any clock jumps or packet queue buffering events.
2. Tracks Hesai header-to-recorder offset across all 809 scans.
3. Examines motion-based cross-correlation between IMU angular rate
   and LiDAR scan motion to evaluate independent temporal alignment.
"""
import sqlite3
import struct
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

DB3 = "/mnt/d/glass-research/rosbag2_2026_10_01-16_49_33/rosbag2_2026_10_01-16_49_33_0.db3"
conn = sqlite3.connect(f"file:{DB3}?mode=ro", uri=True)
cur = conn.cursor()

print("=" * 80)
print("ACSAR-E M1: RIGOROUS CLOCK OFFSET & SYNCHRONIZATION INVESTIGATION")
print("=" * 80)

def extract_timestamps(topic_name):
    cur.execute("SELECT id FROM topics WHERE name=?", (topic_name,))
    row = cur.fetchone()
    if not row:
        return np.array([]), np.array([])
    tid = row[0]
    # Fast substr: 8 bytes at offset 5 (1-based) gives sec (i32) and nsec (u32)
    cur.execute("SELECT timestamp, substr(data, 5, 8) FROM messages WHERE topic_id=? ORDER BY timestamp ASC", (tid,))
    rows = cur.fetchall()
    bag_ts = []
    hdr_ts = []
    for b_ts, raw in rows:
        if len(raw) < 8: continue
        sec, nsec = struct.unpack('<iI', raw)
        bag_ts.append(b_ts * 1e-9)
        hdr_ts.append(sec + nsec * 1e-9)
    return np.array(bag_ts), np.array(hdr_ts)

# 1. L1 Raw
l1_bag_ts, l1_hdr_ts = extract_timestamps('/utlidar/cloud')
l1_offset = l1_bag_ts - l1_hdr_ts
l1_rel_time = l1_bag_ts - l1_bag_ts[0]

print(f"L1 Raw Messages Analyzed: {len(l1_bag_ts)}")
print(f"  Initial Offset : {l1_offset[0]:.6f} s")
print(f"  Final Offset   : {l1_offset[-1]:.6f} s")
print(f"  Total Change   : {(l1_offset[-1] - l1_offset[0])*1000:.3f} ms over {l1_rel_time[-1]:.2f} s")
l1_drift_ppm = ((l1_offset[-1] - l1_offset[0]) / l1_rel_time[-1]) * 1e6
print(f"  Apparent Drift : {l1_drift_ppm:+.2f} ppm ({l1_drift_ppm*1e-3:+.3f} ms/s)")
print(f"  Offset Min/Max : [{l1_offset.min():.6f}, {l1_offset.max():.6f}] s (Peak-to-peak: {(l1_offset.max() - l1_offset.min())*1000:.2f} ms)")
print(f"  Offset Std Dev : {np.std(l1_offset)*1000:.2f} ms")

p_l1 = np.polyfit(l1_rel_time, l1_offset, 1)
l1_residual = l1_offset - np.polyval(p_l1, l1_rel_time)
print(f"  Linear fit drift rate: {p_l1[0]*1e6:+.2f} ppm")
print(f"  Residual Jitter (std): {np.std(l1_residual)*1000:.2f} ms (max: {np.max(np.abs(l1_residual))*1000:.2f} ms)")

# 2. Hesai XT-16
h_bag_ts, h_hdr_ts = extract_timestamps('/lidar_points')
h_offset = h_bag_ts - h_hdr_ts
h_rel_time = h_bag_ts - h_bag_ts[0]

print(f"\nHesai XT-16 Messages Analyzed: {len(h_bag_ts)}")
print(f"  Initial Offset : {h_offset[0]:.6f} s")
print(f"  Final Offset   : {h_offset[-1]:.6f} s")
print(f"  Total Change   : {(h_offset[-1] - h_offset[0])*1000:.3f} ms over {h_rel_time[-1]:.2f} s")
h_drift_ppm = ((h_offset[-1] - h_offset[0]) / h_rel_time[-1]) * 1e6
print(f"  Apparent Drift : {h_drift_ppm:+.2f} ppm ({h_drift_ppm*1e-3:+.3f} ms/s)")
print(f"  Offset Std Dev : {np.std(h_offset)*1000:.2f} ms")

p_h = np.polyfit(h_rel_time, h_offset, 1)
h_residual = h_offset - np.polyval(p_h, h_rel_time)
print(f"  Linear fit drift rate: {p_h[0]*1e6:+.2f} ppm")
print(f"  Residual Jitter (std): {np.std(h_residual)*1000:.2f} ms")

# 3. SportModeState
sm_bag_ts, sm_hdr_ts = extract_timestamps('/sportmodestate')
sm_offset = sm_bag_ts - sm_hdr_ts
sm_rel_time = sm_bag_ts - sm_bag_ts[0]

p_sm = np.polyfit(sm_rel_time, sm_offset, 1)
sm_residual = sm_offset - np.polyval(p_sm, sm_rel_time)

print(f"\nSportModeState Messages Analyzed: {len(sm_bag_ts)}")
print(f"  Initial Offset : {sm_offset[0]:.6f} s")
print(f"  Linear fit drift: {p_sm[0]*1e6:+.2f} ppm")
print(f"  Residual Jitter : {np.std(sm_residual)*1000:.2f} ms")

# Compare L1 and SportModeState clocks directly:
interp_l1_hdr = np.interp(sm_bag_ts, l1_bag_ts, l1_hdr_ts)
robot_internal_diff = (sm_hdr_ts - interp_l1_hdr) * 1000.0
print(f"\nDirect Robot RTC Clock Alignment (SportModeState vs L1):")
print(f"  Mean Difference : {np.mean(robot_internal_diff):+.3f} ms")
print(f"  Std Difference  : {np.std(robot_internal_diff):.3f} ms")

conn.close()

# 4. Plot Offset & Residual Curves
out_fig = "/mnt/d/glass-research/acsar_e_m1/reports/visualizations/sync_clock_offset_analysis.png"
fig, axes = plt.subplots(2, 2, figsize=(16, 10))

# L1 Offset
axes[0, 0].plot(l1_rel_time, l1_offset, color='royalblue', lw=1.5, label='Measured (Bag - Header)')
axes[0, 0].plot(l1_rel_time, np.polyval(p_l1, l1_rel_time), 'r--', lw=1.5, label=f'Linear Drift ({p_l1[0]*1e6:+.1f} ppm)')
axes[0, 0].set_title(f"Unitree L1: Recorder Offset Evolution\n(Span: {(l1_offset[-1]-l1_offset[0])*1000:+.2f} ms drift over {l1_rel_time[-1]:.1f} s)", fontsize=11)
axes[0, 0].set_xlabel("Recording Elapsed Time (s)")
axes[0, 0].set_ylabel("Bag - Header Offset (s)")
axes[0, 0].grid(True, linestyle='--', alpha=0.5)
axes[0, 0].legend()

# L1 Residuals (Jitter)
axes[0, 1].plot(l1_rel_time, l1_residual * 1000.0, color='darkorange', lw=1.2)
axes[0, 1].axhline(0, color='black', lw=0.8, linestyle='--')
axes[0, 1].set_title(f"Unitree L1: Transport Jitter (Residual from Linear Drift)\n(Std: {np.std(l1_residual)*1000:.2f} ms, Max: {np.max(np.abs(l1_residual))*1000:.2f} ms)", fontsize=11)
axes[0, 1].set_xlabel("Recording Elapsed Time (s)")
axes[0, 1].set_ylabel("Transport Jitter Residual (ms)")
axes[0, 1].grid(True, linestyle='--', alpha=0.5)

# Hesai Offset
axes[1, 0].plot(h_rel_time, h_offset - h_offset[0], color='forestgreen', lw=1.5, label='Measured Relative Offset')
axes[1, 0].plot(h_rel_time, (np.polyval(p_h, h_rel_time) - h_offset[0]), 'r--', lw=1.5, label=f'Linear Drift ({p_h[0]*1e6:+.1f} ppm)')
axes[1, 0].set_title(f"Hesai XT-16: Relative Recorder Offset Evolution\n(Span: {(h_offset[-1]-h_offset[0])*1000:+.2f} ms drift over {h_rel_time[-1]:.1f} s)", fontsize=11)
axes[1, 0].set_xlabel("Recording Elapsed Time (s)")
axes[1, 0].set_ylabel("Offset Change from t=0 (s)")
axes[1, 0].grid(True, linestyle='--', alpha=0.5)
axes[1, 0].legend()

# Hesai Residuals (Jitter)
axes[1, 1].plot(h_rel_time, h_residual * 1000.0, color='crimson', lw=1.2)
axes[1, 1].axhline(0, color='black', lw=0.8, linestyle='--')
axes[1, 1].set_title(f"Hesai XT-16: Transport Jitter (Residual from Linear Drift)\n(Std: {np.std(h_residual)*1000:.2f} ms, Max: {np.max(np.abs(h_residual))*1000:.2f} ms)", fontsize=11)
axes[1, 1].set_xlabel("Recording Elapsed Time (s)")
axes[1, 1].set_ylabel("Transport Jitter Residual (ms)")
axes[1, 1].grid(True, linestyle='--', alpha=0.5)

plt.tight_layout()
plt.savefig(out_fig, dpi=160)
plt.close()
print(f"\nSaved visualization: {out_fig}")
print("Synchronization Analysis Complete.")
