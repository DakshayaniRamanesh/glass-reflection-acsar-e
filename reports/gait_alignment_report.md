# ACSAR-E M1 — Robot Gait & Kinematics Data Audit Report

> [!IMPORTANT]
> Decoded using official Unitree ROS2 `unitree_go` message definitions from `src/unitree/unitree_go/msg/`.

## 1. Summary of Decoded State Streams

| Topic | Message Type | Rate (Original) | Exported Rate | Exported Fields | Primary Use Case |
|:---|:---|:---:|:---:|:---|:---|
| `/sportmodestate` | `unitree_go/msg/SportModeState` | 268.7 Hz | ~54 Hz (4,757 samples) | `header_stamp_s`, `gait_type`, `position`, `velocity`, `yaw_speed`, `foot_force`, `rpy` | High-level locomotion mode & odometry tracking (M3) |
| `/lowstate` | `unitree_go/msg/LowState` | 451.6 Hz | ~45 Hz (4,002 samples) | `joint_position` (12), `joint_velocity` (12), `joint_torque` (12), `foot_force` (4) | Exact leg kinematics & ground stance/swing phase identification (M3) |

## 2. Gait State Enumeration & Trajectory Analysis

- **Gait Enum Definition (`gait_type`):**
  - `0`: Idle / Stance
  - `1`: Trot Walk (Standard quadruped 2-beat diagonal locomotion)
  - `2`: Trot Running / Fast Trot
  - `3`: Stair Climbing
  - `4`: Obstacle Stepping
- **Observed Locomotion Regimes in Bag `16_49_33`:**
  - **Early Epoch (0–12 s):** `gait_type = 0` (Robot standing stationary, zero linear velocity).
  - **Mid Epoch (15–65 s):** `gait_type = 1` (Active trotting, forward velocity $0.3	ext{--}0.8	ext{ m/s}$, alternating diagonal foot contacts).
  - **Late Epoch (70–88 s):** `gait_type = 1 	o 0` (Deceleration and return to stationary stance).

## 3. Timing Alignment & Synchronization Caveats

1. **SportModeState Header Stamp:** Uses `TimeSpec` sharing the exact robot RTC hardware clock domain (`1,790,852,964 s` epoch) with Unitree L1 LiDAR. Mean internal clock offset to L1 is only $+65.8	ext{ ms} \pm 2.25	ext{ ms}$.
2. **LowState Hardware Ticks:** Does not contain an absolute Unix timestamp header; it runs on the DSP motor micro-controller hardware tick. Synchronization to LiDAR should be performed via `bag_receipt_ns` with local polynomial smoothing or cross-referencing `foot_force` spikes with `SportModeState` contact states.
3. **M3 Fusion Guidance:** Gait phase (stance vs. swing) should be computed directly from `foot_force` thresholds (>15 N indicating firm ground contact) combined with `joint_velocity` signs.
