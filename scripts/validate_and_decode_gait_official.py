#!/usr/bin/env python3
"""
Official ROS2 Deserialization and Validation of SportModeState and LowState
Uses compiled unitree_go.msg interfaces with rclpy.serialization.
Efficient sequential scan to avoid NTFS SQLite OFFSET slowdowns.
"""

import os
import sys
import sqlite3
import numpy as np

# Force unbuffered stdout
sys.stdout.reconfigure(line_buffering=True)

try:
    from rclpy.serialization import deserialize_message
    from unitree_go.msg import SportModeState, LowState
except ImportError as e:
    print(f"Error importing ROS2 / unitree_go: {e}", file=sys.stderr)
    sys.exit(1)

DB_PATH = "/mnt/d/glass-research/rosbag2_2026_10_01-16_49_33/rosbag2_2026_10_01-16_49_33_0.db3"
OUT_DIR = "/mnt/d/glass-research/acsar_e_m1/exports"
os.makedirs(OUT_DIR, exist_ok=True)

print("=" * 80)
print("ACSAR-E M1: OFFICIAL ROS2 DESERIALIZATION VALIDATION (Gait & LowState)")
print("=" * 80)

conn = sqlite3.connect(f"file:{DB_PATH}?mode=ro", uri=True)
cur = conn.cursor()

# Get topic IDs
cur.execute("SELECT id, name, type FROM topics")
topics = {name: (tid, ttype) for tid, name, ttype in cur.fetchall()}

sport_tid = topics["/sportmodestate"][0]
low_tid = topics["/lowstate"][0]
print(f"/sportmodestate topic id = {sport_tid}")
print(f"/lowstate topic id = {low_tid}")

# -----------------------------------------------------------------------------
# 1. SportModeState: Sample across timeline
# -----------------------------------------------------------------------------
print("\n" + "=" * 80)
print("STEP 1: SportModeState Timeline Sampling")
print("=" * 80)

# Fetch all timestamps and ids first (fast, minimal data)
cur.execute("SELECT id, timestamp FROM messages WHERE topic_id=? ORDER BY timestamp ASC", (sport_tid,))
sport_msg_meta = cur.fetchall()
total_sport = len(sport_msg_meta)
print(f"Total /sportmodestate messages: {total_sport}")

sample_indices = [
    0, 
    min(50, total_sport - 1), 
    total_sport // 4, 
    total_sport // 2, 
    3 * total_sport // 4, 
    total_sport - 1
]

print(f"\nInspecting {len(sample_indices)} sample points across bag duration:")
for idx in sample_indices:
    msg_id, b_ts = sport_msg_meta[idx]
    cur.execute("SELECT data FROM messages WHERE id=?", (msg_id,))
    raw = cur.fetchone()[0]
    
    msg = deserialize_message(raw, SportModeState)
    sec = msg.stamp.sec
    nanosec = msg.stamp.nanosec
    t_sec = sec + nanosec * 1e-9
    
    pos = list(msg.position)
    vel = list(msg.velocity)
    ff = list(msg.foot_force)
    rpy = list(msg.imu_state.rpy)
    
    print(f"\n[SportMode Msg #{idx:5d} | msg_id={msg_id}] Bag ts: {b_ts} | Header Stamp: {t_sec:.6f}s")
    print(f"  mode: {msg.mode} | gait_type: {msg.gait_type} | progress: {msg.progress:.3f}")
    print(f"  body_height: {msg.body_height:.4f} m | foot_raise_height: {msg.foot_raise_height:.4f} m")
    print(f"  position: x={pos[0]:.4f}, y={pos[1]:.4f}, z={pos[2]:.4f}")
    print(f"  velocity: vx={vel[0]:.4f}, vy={vel[1]:.4f}, vz={vel[2]:.4f} (speed: {np.linalg.norm(vel):.4f} m/s)")
    print(f"  yaw_speed: {msg.yaw_speed:.4f} rad/s")
    print(f"  foot_force (FR, FL, RR, RL): {ff}")
    print(f"  rpy (deg): roll={np.rad2deg(rpy[0]):.2f}, pitch={np.rad2deg(rpy[1]):.2f}, yaw={np.rad2deg(rpy[2]):.2f}")

# -----------------------------------------------------------------------------
# 2. LowState: Sample across timeline
# -----------------------------------------------------------------------------
print("\n" + "=" * 80)
print("STEP 2: LowState Timeline Sampling")
print("=" * 80)

cur.execute("SELECT id, timestamp FROM messages WHERE topic_id=? ORDER BY timestamp ASC", (low_tid,))
low_msg_meta = cur.fetchall()
total_low = len(low_msg_meta)
print(f"Total /lowstate messages: {total_low}")

sample_indices_low = [
    0, 
    min(100, total_low - 1), 
    total_low // 4, 
    total_low // 2, 
    3 * total_low // 4, 
    total_low - 1
]

for idx in sample_indices_low:
    msg_id, b_ts = low_msg_meta[idx]
    cur.execute("SELECT data FROM messages WHERE id=?", (msg_id,))
    raw = cur.fetchone()[0]
    
    msg = deserialize_message(raw, LowState)
    ff = list(msg.foot_force)
    ff_est = list(msg.foot_force_est)
    
    # 12 joints: FR(0..2), FL(3..5), RR(6..8), RL(9..11)
    q_vec = [msg.motor_state[j].q for j in range(12)]
    dq_vec = [msg.motor_state[j].dq for j in range(12)]
    tau_vec = [msg.motor_state[j].tau_est for j in range(12)]
    
    print(f"\n[LowState Msg #{idx:5d} | msg_id={msg_id}] Bag ts: {b_ts} | tick: {msg.tick}")
    print(f"  foot_force: {ff} | foot_force_est: {ff_est}")
    print(f"  FR leg [hip, thigh, calf]:")
    print(f"    q:   [{q_vec[0]:.3f}, {q_vec[1]:.3f}, {q_vec[2]:.3f}] rad")
    print(f"    dq:  [{dq_vec[0]:.3f}, {dq_vec[1]:.3f}, {dq_vec[2]:.3f}] rad/s")
    print(f"    tau: [{tau_vec[0]:.3f}, {tau_vec[1]:.3f}, {tau_vec[2]:.3f}] Nm")
    print(f"  FL leg [hip, thigh, calf]:")
    print(f"    q:   [{q_vec[3]:.3f}, {q_vec[4]:.3f}, {q_vec[5]:.3f}] rad")
    print(f"    dq:  [{dq_vec[3]:.3f}, {dq_vec[4]:.3f}, {dq_vec[5]:.3f}] rad/s")
    print(f"    tau: [{tau_vec[3]:.3f}, {tau_vec[4]:.3f}, {tau_vec[5]:.3f}] Nm")
    print(f"  Batt power_v: {msg.power_v:.2f} V | power_a: {msg.power_a:.2f} A | soc: {msg.bms_state.soc}%")

conn.close()
print("\nValidation scan completed successfully.")
