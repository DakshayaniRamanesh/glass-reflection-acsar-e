#!/usr/bin/env python3
"""
Full extraction and statistical audit of SportModeState and LowState
for rosbag2_2026_10_01-16_49_33.
Uses official ROS2 rclpy deserialization.
"""

import os
import sys
import sqlite3
import numpy as np

sys.stdout.reconfigure(line_buffering=True)

from rclpy.serialization import deserialize_message
from unitree_go.msg import SportModeState, LowState

DB_PATH = "/mnt/d/glass-research/rosbag2_2026_10_01-16_49_33/rosbag2_2026_10_01-16_49_33_0.db3"
OUT_DIR = "/mnt/d/glass-research/acsar_e_m1/exports"
REPORT_DIR = "/mnt/d/glass-research/acsar_e_m1/reports"
os.makedirs(OUT_DIR, exist_ok=True)
os.makedirs(REPORT_DIR, exist_ok=True)

print("Opening bag:", DB_PATH)
conn = sqlite3.connect(f"file:{DB_PATH}?mode=ro", uri=True)
cur = conn.cursor()

cur.execute("SELECT id, name FROM topics")
topics = dict(cur.fetchall())
inv_topics = {v: k for k, v in topics.items()}

sport_tid = inv_topics["/sportmodestate"]
low_tid = inv_topics["/lowstate"]

# -----------------------------------------------------------------------------
# 1. Full Extraction: SportModeState
# -----------------------------------------------------------------------------
print("\nExtracting all /sportmodestate messages...")
cur.execute("SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp ASC", (sport_tid,))

sport_b_ts = []
sport_stamp = []
sport_mode = []
sport_gait_type = []
sport_progress = []
sport_body_height = []
sport_foot_raise = []
sport_pos = []
sport_vel = []
sport_yaw_speed = []
sport_range_obs = []
sport_foot_force = []
sport_foot_pos = []
sport_foot_spd = []
sport_imu_quat = []
sport_imu_gyro = []
sport_imu_acc = []
sport_imu_rpy = []

count = 0
for b_ts, raw in cur:
    msg = deserialize_message(raw, SportModeState)
    sec = msg.stamp.sec
    nsec = msg.stamp.nanosec
    t_sec = sec + nsec * 1e-9
    
    sport_b_ts.append(b_ts)
    sport_stamp.append(t_sec)
    sport_mode.append(msg.mode)
    sport_gait_type.append(msg.gait_type)
    sport_progress.append(msg.progress)
    sport_body_height.append(msg.body_height)
    sport_foot_raise.append(msg.foot_raise_height)
    sport_pos.append(list(msg.position))
    sport_vel.append(list(msg.velocity))
    sport_yaw_speed.append(msg.yaw_speed)
    sport_range_obs.append(list(msg.range_obstacle))
    sport_foot_force.append(list(msg.foot_force))
    sport_foot_pos.append(list(msg.foot_position_body))
    sport_foot_spd.append(list(msg.foot_speed_body))
    sport_imu_quat.append(list(msg.imu_state.quaternion))
    sport_imu_gyro.append(list(msg.imu_state.gyroscope))
    sport_imu_acc.append(list(msg.imu_state.accelerometer))
    sport_imu_rpy.append(list(msg.imu_state.rpy))
    
    count += 1
    if count % 5000 == 0:
        print(f"  Processed {count} SportModeState messages...")

print(f"Total SportModeState messages extracted: {count}")

sport_npz_path = os.path.join(OUT_DIR, "sportmode_validated.npz")
np.savez_compressed(
    sport_npz_path,
    bag_timestamp=np.array(sport_b_ts, dtype=np.int64),
    header_stamp=np.array(sport_stamp, dtype=np.float64),
    mode=np.array(sport_mode, dtype=np.uint8),
    gait_type=np.array(sport_gait_type, dtype=np.uint8),
    progress=np.array(sport_progress, dtype=np.float32),
    body_height=np.array(sport_body_height, dtype=np.float32),
    foot_raise_height=np.array(sport_foot_raise, dtype=np.float32),
    position=np.array(sport_pos, dtype=np.float32),
    velocity=np.array(sport_vel, dtype=np.float32),
    yaw_speed=np.array(sport_yaw_speed, dtype=np.float32),
    range_obstacle=np.array(sport_range_obs, dtype=np.float32),
    foot_force=np.array(sport_foot_force, dtype=np.int16),
    foot_position_body=np.array(sport_foot_pos, dtype=np.float32),
    foot_speed_body=np.array(sport_foot_spd, dtype=np.float32),
    imu_quaternion=np.array(sport_imu_quat, dtype=np.float32),
    imu_gyroscope=np.array(sport_imu_gyro, dtype=np.float32),
    imu_accelerometer=np.array(sport_imu_acc, dtype=np.float32),
    imu_rpy=np.array(sport_imu_rpy, dtype=np.float32)
)
print("Saved SportModeState to:", sport_npz_path)

# -----------------------------------------------------------------------------
# 2. Full Extraction: LowState
# -----------------------------------------------------------------------------
print("\nExtracting all /lowstate messages...")
cur.execute("SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp ASC", (low_tid,))

low_b_ts = []
low_tick = []
low_foot_force = []
low_foot_force_est = []
low_q = []
low_dq = []
low_ddq = []
low_tau_est = []
low_motor_temp = []
low_power_v = []
low_power_a = []
low_soc = []
low_imu_quat = []
low_imu_gyro = []
low_imu_acc = []
low_imu_rpy = []

count = 0
for b_ts, raw in cur:
    msg = deserialize_message(raw, LowState)
    
    low_b_ts.append(b_ts)
    low_tick.append(msg.tick)
    low_foot_force.append(list(msg.foot_force))
    low_foot_force_est.append(list(msg.foot_force_est))
    
    # 12 motors
    low_q.append([msg.motor_state[j].q for j in range(12)])
    low_dq.append([msg.motor_state[j].dq for j in range(12)])
    low_ddq.append([msg.motor_state[j].ddq for j in range(12)])
    low_tau_est.append([msg.motor_state[j].tau_est for j in range(12)])
    low_motor_temp.append([msg.motor_state[j].temperature for j in range(12)])
    
    low_power_v.append(msg.power_v)
    low_power_a.append(msg.power_a)
    low_soc.append(msg.bms_state.soc)
    
    low_imu_quat.append(list(msg.imu_state.quaternion))
    low_imu_gyro.append(list(msg.imu_state.gyroscope))
    low_imu_acc.append(list(msg.imu_state.accelerometer))
    low_imu_rpy.append(list(msg.imu_state.rpy))
    
    count += 1
    if count % 10000 == 0:
        print(f"  Processed {count} LowState messages...")

print(f"Total LowState messages extracted: {count}")

low_npz_path = os.path.join(OUT_DIR, "lowstate_validated.npz")
np.savez_compressed(
    low_npz_path,
    bag_timestamp=np.array(low_b_ts, dtype=np.int64),
    tick=np.array(low_tick, dtype=np.uint32),
    foot_force=np.array(low_foot_force, dtype=np.int16),
    foot_force_est=np.array(low_foot_force_est, dtype=np.int16),
    joint_q=np.array(low_q, dtype=np.float32),
    joint_dq=np.array(low_dq, dtype=np.float32),
    joint_ddq=np.array(low_ddq, dtype=np.float32),
    joint_tau_est=np.array(low_tau_est, dtype=np.float32),
    motor_temp=np.array(low_motor_temp, dtype=np.int8),
    power_v=np.array(low_power_v, dtype=np.float32),
    power_a=np.array(low_power_a, dtype=np.float32),
    bms_soc=np.array(low_soc, dtype=np.uint8),
    imu_quaternion=np.array(low_imu_quat, dtype=np.float32),
    imu_gyroscope=np.array(low_imu_gyro, dtype=np.float32),
    imu_accelerometer=np.array(low_imu_acc, dtype=np.float32),
    imu_rpy=np.array(low_imu_rpy, dtype=np.float32)
)
print("Saved LowState to:", low_npz_path)

conn.close()

# -----------------------------------------------------------------------------
# 3. Print Statistical Audit
# -----------------------------------------------------------------------------
print("\n" + "=" * 80)
print("STATISTICAL SUMMARY & AUDIT OF EXTRACTED DATA")
print("=" * 80)

u_mode = np.unique(sport_mode)
u_gait = np.unique(sport_gait_type)
print(f"SportModeState unique 'mode' values: {u_mode}")
print(f"SportModeState unique 'gait_type' values: {u_gait}")

body_h_arr = np.array(sport_body_height)
vel_arr = np.array(sport_vel)
speed_arr = np.linalg.norm(vel_arr, axis=1)
print(f"Body height: min={body_h_arr.min():.4f}m, max={body_h_arr.max():.4f}m, mean={body_h_arr.mean():.4f}m")
print(f"Velocity magnitude: min={speed_arr.min():.4f}m/s, max={speed_arr.max():.4f}m/s, mean={speed_arr.mean():.4f}m/s")

ff_arr = np.array(low_foot_force)
print(f"\nLowState foot_force per leg (FR, FL, RR, RL):")
for leg_idx, leg_name in enumerate(["FR", "FL", "RR", "RL"]):
    leg_ff = ff_arr[:, leg_idx]
    print(f"  {leg_name}: min={leg_ff.min():3d}, max={leg_ff.max():3d}, mean={leg_ff.mean():.1f}, non-zero count={np.count_nonzero(leg_ff)} / {len(leg_ff)}")

dq_arr = np.array(low_dq)
tau_arr = np.array(low_tau_est)
print(f"\nLowState Joint velocity (|dq|) across all 12 motors: max={np.abs(dq_arr).max():.3f} rad/s, mean={np.abs(dq_arr).mean():.3f} rad/s")
print(f"LowState Joint torque (|tau_est|) across all 12 motors: max={np.abs(tau_arr).max():.3f} Nm, mean={np.abs(tau_arr).mean():.3f} Nm")

print("\nAll tasks completed successfully!")

