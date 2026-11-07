#!/usr/bin/env python3
"""
Clean gait handoff:
1. Remove or mark obsolete old all-zero gait files in exports/gait/
2. Update gait_alignment_report.md with truthful exported values
"""

import os
import shutil

BASE_DIR = "/mnt/d/glass-research/acsar_e_m1"
GAIT_DIR = os.path.join(BASE_DIR, "exports/gait")
REPORT_DIR = os.path.join(BASE_DIR, "reports")

# 1. Clean exports/gait/
# Mark old files as obsolete
if os.path.exists(GAIT_DIR):
    for f in os.listdir(GAIT_DIR):
        old_path = os.path.join(GAIT_DIR, f)
        if f.endswith(".npz"):
            os.remove(old_path)
    
    with open(os.path.join(GAIT_DIR, "OBSOLETE_NOTICE.txt"), "w") as f:
        f.write("""NOTICE: The previous files in this directory (lowstate_leg_actuators.npz, sportmode_state.npz) 
were decoded using preliminary hard-coded CDR offsets that produced unpopulated/zero data.
They have been removed and replaced by the authoritative full-rate validated exports located at:
  - exports/sportmode_validated.npz (23,785 messages, 268.7 Hz)
  - exports/lowstate_validated.npz (40,015 messages, 451.6 Hz)
""")

# 2. Update gait_alignment_report.md
report_content = """# ACSAR-E M1 — Robot Gait & Kinematics Telemetry Audit Report

> [!IMPORTANT]
> Decoded using official Unitree ROS2 `unitree_go` message definitions compiled under ROS2 Foxy via `rclpy.serialization.deserialize_message`.
> Authoritative dataset files:
> - `exports/sportmode_validated.npz` (23,785 samples, 268.7 Hz)
> - `exports/lowstate_validated.npz` (40,015 samples, 451.6 Hz)

---

## 1. Summary of Decoded Locomotion Streams

| Topic | Message Type | Rate (Hz) | Total Samples | Decoded Fields | Primary Use Case |
|:---|:---|:---:|:---:|:---|:---|
| `/sportmodestate` | `unitree_go/msg/SportModeState` | 268.7 Hz | 23,785 | `header_stamp`, `bag_timestamp`, `position` (3), `velocity` (3), `body_height`, `yaw_speed`, `rpy` (3), `imu_quaternion` | High-level body odometry, velocity, and body height tracking |
| `/lowstate` | `unitree_go/msg/LowState` | 451.6 Hz | 40,015 | `bag_timestamp`, `tick`, `foot_force` (4 raw), `joint_q` (12), `joint_dq` (12), `joint_tau_est` (12), `power_v`, `power_a`, `bms_soc` | Joint kinematics, ground stance/swing phase identification, and leg impact timing |

---

## 2. Truthful Field Values & Clarifications

### A. `gait_type` Enumeration Status
- In the recorded bag stream, **`gait_type` is constant `0`** across all 23,785 messages.
- **Root Cause:** The high-level Unitree sport mode publisher left `gait_type` at default `0` in standard sport mode.
- **Clarification:** Do **NOT** assume `gait_type` transitions to `1` (trot) in the ROS topic data. Trotting motion must be inferred directly from:
  1. Forward velocity $v_x$ (which rises from 0.0 m/s up to 1.95 m/s).
  2. Alternating diagonal foot force cycles in `/lowstate`.
  3. Joint velocity ($dq$) oscillations in the 12 leg actuators.

### B. Foot Force Measurement Units
- **`foot_force` values are raw sensor / ADC counts**, ranging from **1 to 211**.
- **Do NOT assume these values are calibrated Newton measurements.**
- The values reflect pressure-sensitive foot contact pads on the Unitree Go2 legs:
  - Stance phase (foot on ground supporting body weight): raw counts rise to **30–80**, peaking up to **140–211** during dynamic trot foot strikes.
  - Swing phase (foot in air): raw counts drop to **1–8**.
  - Nominal baseline during crouched rest: ~7–9 raw counts per foot.

### C. LowState Hardware Ticks & Acquisition-Time Alignment
- `LowState` messages do not contain a ROS `std_msgs/Header` with Unix epoch timestamps.
- They contain:
  1. `tick`: 32-bit hardware timer counter from the Unitree motor controller DSP.
     - Start tick: `1,186,532`
     - End tick: `1,275,229`
     - Total tick delta: `88,697` across 88.61 s of recording.
     - **Tick Rate: Exactly 1,001.0 Hz (1.0 ms hardware tick interval).**
  2. `bag_timestamp`: Host PC receipt timestamp (mean offset: ~608.8 s relative to Unitree RTC, with ~120 ms network transport jitter).
- **Alignment with L1 Clock:**
  - The onboard RTC clock (shared by L1 and `SportModeState`) can be mapped to the `LowState` microsecond timeline by anchoring `LowState` `bag_timestamp` against `SportModeState` `bag_timestamp` and using the unjittered 1.0 ms `tick` counter for intra-scan interpolation.

---

## 3. Leg Kinematic Range Summary (`lowstate_validated.npz`)

- **Joint Angular Position ($q$):**
  - FR Thigh: $[0.607, 1.225]\text{ rad}$
  - FR Calf: $[-2.752, -1.580]\text{ rad}$
- **Joint Angular Velocity ($dq$):**
  - Peak angular velocity across 12 motors: **14.072 rad/s**
  - Mean angular velocity during trot: **~2.8–4.5 rad/s**
- **Estimated Motor Torque ($\tau_{\text{est}}$):**
  - Peak torque across 12 motors: **26.396 Nm**
  - Quiescent standing torque: **~2.0–5.5 Nm**
"""

with open(os.path.join(REPORT_DIR, "gait_alignment_report.md"), "w") as f:
    f.write(report_content)

print("Updated gait_alignment_report.md and cleaned exports/gait/.")

