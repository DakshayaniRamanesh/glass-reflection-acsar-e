# ACSAR-E M1: Gait & Locomotion State Validation Report

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
  - **Torque ($	au_{	ext{est}}$):** Peak joint torque = **26.396 Nm**
- **Electrical & Thermal:**
  - Battery Voltage: 27.81 V – 28.16 V
  - Current Draw: 1.09 A (stationary stand) up to 10.42 A (high-speed trot acceleration)
  - Motor Temperatures: 29°C – 38°C

---

## 3. Gait Phase & Transition Analysis for M3

M3 requires gait-phase awareness to determine when quadruped leg impacts and gait transitions induce high-frequency motion artifacts near reflective surfaces.

From the validated `/lowstate` foot forces:
1. **Stationary / Rest Epochs ($t < 15s$ and $t > 80s$):**
   - Joint velocity $dq pprox 0$
   - Foot forces remain steady at body-weight support distribution (~30-50 units per foot)
   - Current draw is ~1.1 A
2. **Trotting Epochs ($18s < t < 75s$):**
   - Distinct alternating diagonal pair coordination:
     - Diagonal Pair 1 (FR + RL): peaks in contact force coincide with valley in Diagonal Pair 2.
     - Diagonal Pair 2 (FL + RR): peaks in contact force coincide with valley in Diagonal Pair 1.
   - Joint velocities swing cyclically at ~2.5 - 3.5 Hz stepping frequency.
   - Significant impulsive torques occur during foot strike events ($>20	ext{ Nm}$).

---

## 4. Verification Deliverables

The validated data arrays have been exported in standard NumPy compressed format preserving original dtypes and timestamps:
1. `D:\glass-research\acsar_e_m1\exports\sportmode_validated.npz`
2. `D:\glass-research\acsar_e_m1\exports\lowstate_validated.npz`
3. Visualizations:
   - `D:\glass-research\acsar_e_m1\reports\visualizations\gait_dynamics_overview.png`
   - `D:\glass-research\acsar_e_m1\reports\visualizations\gait_trot_diagonal_phase_zoom.png`
