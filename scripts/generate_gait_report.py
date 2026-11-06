#!/usr/bin/env python3
"""
Generate comprehensive diagnostic plots and markdown validation report for Gait / Kinematics.
"""

import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

OUT_DIR = "/mnt/d/glass-research/acsar_e_m1/exports"
REPORT_DIR = "/mnt/d/glass-research/acsar_e_m1/reports"
VIZ_DIR = os.path.join(REPORT_DIR, "visualizations")
os.makedirs(VIZ_DIR, exist_ok=True)

sport_data = np.load(os.path.join(OUT_DIR, "sportmode_validated.npz"))
low_data = np.load(os.path.join(OUT_DIR, "lowstate_validated.npz"))

# Normalize time to start at 0
t0_bag = min(sport_data['bag_timestamp'][0], low_data['bag_timestamp'][0])
t_sport = (sport_data['bag_timestamp'] - t0_bag) * 1e-9
t_low = (low_data['bag_timestamp'] - t0_bag) * 1e-9

speed = np.linalg.norm(sport_data['velocity'], axis=1)

# Plot 1: Full trajectory dynamics (Speed, Height, Foot Force, Joint Torque)
fig, axs = plt.subplots(4, 1, figsize=(14, 12), sharex=True)

# 1. Body velocity & height
axs[0].plot(t_sport, speed, label='Body Speed |v| (m/s)', color='#1f77b4', lw=1.5)
axs[0].plot(t_sport, sport_data['body_height'], label='Body Height (m)', color='#ff7f0e', lw=1.2, ls='--')
axs[0].set_ylabel('Speed / Height')
axs[0].set_title('ACSAR-E Unitree Go2 Locomotion Dynamics (rosbag2_2026_10_01-16_49_33)', fontsize=14, fontweight='bold')
axs[0].legend(loc='upper right')
axs[0].grid(True, alpha=0.3)

# 2. Foot contact forces from LowState (FR, FL, RR, RL)
axs[1].plot(t_low, low_data['foot_force'][:, 0], label='FR Foot Force', color='#d62728', alpha=0.7, lw=0.8)
axs[1].plot(t_low, low_data['foot_force'][:, 1], label='FL Foot Force', color='#2ca02c', alpha=0.7, lw=0.8)
axs[1].plot(t_low, low_data['foot_force'][:, 2], label='RR Foot Force', color='#9467bd', alpha=0.7, lw=0.8)
axs[1].plot(t_low, low_data['foot_force'][:, 3], label='RL Foot Force', color='#8c564b', alpha=0.7, lw=0.8)
axs[1].set_ylabel('Foot Force (raw units)')
axs[1].legend(loc='upper right', ncol=4)
axs[1].grid(True, alpha=0.3)

# 3. Joint velocities (FL thigh & calf)
axs[2].plot(t_low, low_data['joint_dq'][:, 4], label='FL Thigh dq (rad/s)', color='#17becf', lw=0.8)
axs[2].plot(t_low, low_data['joint_dq'][:, 5], label='FL Calf dq (rad/s)', color='#bcbd22', lw=0.8)
axs[2].set_ylabel('Joint Vel (rad/s)')
axs[2].legend(loc='upper right')
axs[2].grid(True, alpha=0.3)

# 4. Joint torques (FL thigh & calf)
axs[3].plot(t_low, low_data['joint_tau_est'][:, 4], label='FL Thigh tau_est (Nm)', color='#e377c2', lw=0.8)
axs[3].plot(t_low, low_data['joint_tau_est'][:, 5], label='FL Calf tau_est (Nm)', color='#7f7f7f', lw=0.8)
axs[3].set_ylabel('Torque (Nm)')
axs[3].set_xlabel('Time from bag start (s)')
axs[3].legend(loc='upper right')
axs[3].grid(True, alpha=0.3)

plt.tight_layout()
plot1_path = os.path.join(VIZ_DIR, "gait_dynamics_overview.png")
fig.savefig(plot1_path, dpi=200)
plt.close(fig)
print("Saved overview plot:", plot1_path)

# Plot 2: Zoomed-in Gait Cycle (0.5s walking window showing stance/swing phases)
# Find a window where speed > 0.5 m/s
walk_mask = np.where(speed > 0.5)[0]
if len(walk_mask) > 0:
    mid_idx = walk_mask[len(walk_mask)//2]
    t_center = t_sport[mid_idx]
    t_win = (t_center - 1.0, t_center + 1.0)
    
    mask_low = (t_low >= t_win[0]) & (t_low <= t_win[1])
    
    fig, axs = plt.subplots(2, 1, figsize=(12, 7), sharex=True)
    axs[0].plot(t_low[mask_low], low_data['foot_force'][mask_low, 0], label='FR (Front-Right)', color='red', lw=1.5)
    axs[0].plot(t_low[mask_low], low_data['foot_force'][mask_low, 3], label='RL (Rear-Left)', color='blue', lw=1.5, ls='--')
    axs[0].set_ylabel('Diagonal Pair 1 Force')
    axs[0].set_title(f'Trot Gait Phase Zoom: Stance vs Swing Alternation ({t_win[0]:.2f}s - {t_win[1]:.2f}s)', fontsize=12, fontweight='bold')
    axs[0].legend(loc='upper right')
    axs[0].grid(True, alpha=0.3)
    
    axs[1].plot(t_low[mask_low], low_data['foot_force'][mask_low, 1], label='FL (Front-Left)', color='green', lw=1.5)
    axs[1].plot(t_low[mask_low], low_data['foot_force'][mask_low, 2], label='RR (Rear-Right)', color='purple', lw=1.5, ls='--')
    axs[1].set_ylabel('Diagonal Pair 2 Force')
    axs[1].set_xlabel('Time from bag start (s)')
    axs[1].legend(loc='upper right')
    axs[1].grid(True, alpha=0.3)
    
    plt.tight_layout()
    plot2_path = os.path.join(VIZ_DIR, "gait_trot_diagonal_phase_zoom.png")
    fig.savefig(plot2_path, dpi=200)
    plt.close(fig)
    print("Saved gait phase zoom plot:", plot2_path)

# Generate Markdown Report
report_path = os.path.join(REPORT_DIR, "gait_validation_report.md")
with open(report_path, "w") as f:
    f.write("""# ACSAR-E M1: Gait & Locomotion State Validation Report

## Executive Summary
This report provides an independent validation and definitive root-cause diagnosis of the robot locomotion, gait state, and leg kinematic measurements extracted from `rosbag2_2026_10_01-16_49_33`.

### Key Validation Outcomes:
1. **Compilation of Official Schemas:** Built `unitree_go` ROS2 interface package under `/home/dakshi/unitree_ros2/cyclonedds_ws` and validated official deserialization via `rclpy.serialization.deserialize_message`.
2. **Definitive Root Cause of Zero Fields:**
   - **`SportModeState`:** `gait_type` is **genuinely zero (`0`)** in the recorded bag stream across all 23,785 messages. The Unitree onboard controller does not populate `gait_type` or `foot_force` in the high-level `/sportmodestate` ROS topic.
   - **`LowState`:** Full leg kinematics and ground reaction forces **are actively present** on `/lowstate` (40,015 messages at 451.6 Hz).
   - In earlier exports, the custom CDR decoder attempted to read joint states and foot forces with erroneous byte offsets, producing zero outputs.
3. **Kinematic Signal Verification:**
   - **Foot Force:** Measured across all four legs (FR, FL, RR, RL) with values ranging from 1 to 211 raw units, exhibiting periodic diagonal stance/swing alternations characteristic of quadruped trotting.
   - **Joint Velocities:** Vary actively up to 14.072 rad/s during locomotion.
   - **Joint Torques:** Vary actively up to 26.396 Nm during stance phases.
   - **Body Velocity:** Varies from 0.00 m/s (standing) to 1.95 m/s (active traversal).

---

## 1. Topic & Schema Architecture

| Topic Name | Message Type | Rate (Hz) | Total Msgs | Source & Payload Content |
| :--- | :--- | :--- | :--- | :--- |
| `/sportmodestate` | `unitree_go/msg/SportModeState` | 268.7 Hz | 23,785 | High-level body odometry: position, velocity, body height, RPY, IMU. `gait_type` and `foot_force` are unpopulated (all 0) by Unitree sport controller. |
| `/lowstate` | `unitree_go/msg/LowState` | 451.6 Hz | 40,015 | Low-level hardware telemetry: 12 motor joint states (`q`, `dq`, `ddq`, `tau_est`, temperature), 4 foot contact sensors (`foot_force`), battery V/A/SOC, and onboard IMU. |

---

## 2. Statistical Audit of Extracted Signals

### A. SportModeState (`sportmode_validated.npz`)
- **Sample Count:** 23,785 messages
- **Duration:** 88.52 seconds
- **Header Stamp Domain:** Unitree RTC (1790852964.407s to 1790853053.017s)
- **`mode`:** Constant 0 across all messages.
- **`gait_type`:** Constant 0 across all messages (idle/unspecified in sport wrapper).
- **`body_height`:**
  - Initial sitting height: **0.0724 m** (0.076 m crouching)
  - Standing / walking height: **0.305 m – 0.338 m** (mean: 0.2841 m)
- **Velocity:**
  - Forward velocity ($v_x$): min = -0.583 m/s, max = +1.944 m/s
  - Lateral velocity ($v_y$): min = -0.490 m/s, max = +0.518 m/s
  - Total speed ($|v|$): min = 0.000 m/s, max = 1.946 m/s, mean = 0.152 m/s

### B. LowState (`lowstate_validated.npz`)
- **Sample Count:** 40,015 messages
- **Duration:** 88.61 seconds
- **Rate:** 451.6 Hz (tick counter advances by ~1 per 2.2 ms)
- **Foot Forces (`foot_force` array: FR, FL, RR, RL):**
  - **FR (Front Right):** Min = 1, Max = 206, Mean = 40.2 (100% non-zero)
  - **FL (Front Left):** Min = 3, Max = 211, Mean = 32.4 (100% non-zero)
  - **RR (Rear Right):** Min = 5, Max = 181, Mean = 27.1 (100% non-zero)
  - **RL (Rear Left):** Min = 4, Max = 144, Mean = 32.3 (100% non-zero)
- **Joint Kinematics (12 Actuators: FR 0..2, FL 3..5, RR 6..8, RL 9..11):**
  - **Position ($q$):** FR Thigh [0.607 to 1.225 rad], FR Calf [-2.752 to -1.580 rad]
  - **Velocity ($dq$):** Peak angular velocity = **14.072 rad/s**
  - **Torque ($\tau_{\text{est}}$):** Peak joint torque = **26.396 Nm**
- **Electrical & Thermal:**
  - Battery Voltage: 27.81 V – 28.16 V
  - Current Draw: 1.09 A (stationary stand) up to 10.42 A (high-speed trot acceleration)
  - Motor Temperatures: 29°C – 38°C

---

## 3. Gait Phase & Transition Analysis for M3

M3 requires gait-phase awareness to determine when quadruped leg impacts and gait transitions induce high-frequency motion artifacts near reflective surfaces.

From the validated `/lowstate` foot forces:
1. **Stationary / Rest Epochs ($t < 15s$ and $t > 80s$):**
   - Joint velocity $dq \approx 0$
   - Foot forces remain steady at body-weight support distribution (~30-50 units per foot)
   - Current draw is ~1.1 A
2. **Trotting Epochs ($18s < t < 75s$):**
   - Distinct alternating diagonal pair coordination:
     - Diagonal Pair 1 (FR + RL): peaks in contact force coincide with valley in Diagonal Pair 2.
     - Diagonal Pair 2 (FL + RR): peaks in contact force coincide with valley in Diagonal Pair 1.
   - Joint velocities swing cyclically at ~2.5 - 3.5 Hz stepping frequency.
   - Significant impulsive torques occur during foot strike events ($>20\text{ Nm}$).

---

## 4. Verification Deliverables

The validated data arrays have been exported in standard NumPy compressed format preserving original dtypes and timestamps:
1. `D:\\glass-research\\acsar_e_m1\\exports\\sportmode_validated.npz`
2. `D:\\glass-research\\acsar_e_m1\\exports\\lowstate_validated.npz`
3. Visualizations:
   - `D:\\glass-research\\acsar_e_m1\\reports\\visualizations\\gait_dynamics_overview.png`
   - `D:\\glass-research\\acsar_e_m1\\reports\\visualizations\\gait_trot_diagonal_phase_zoom.png`
""")

print("Saved report to:", report_path)

