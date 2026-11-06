# ACSAR-E M1 — Transform & Calibration Inventory (Validated)

**Date:** 2026-11-06  
**Status:** Verified via official ROS2 `rclpy` `tf2_msgs.msg.TFMessage` deserialization  
**Target Bags:** All six recordings (`16_39_53` through `16_52_39`)  

---

## 1. Verified Static Transforms in `/tf_static`

The `/tf_static` topic was deserialized using the official ROS2 `tf2_msgs/msg/TFMessage` API. All bags that contain `/tf_static` store the identical single message containing **exactly 4 transforms**:

| # | Parent Frame | Child Frame | Translation $[x, y, z]$ (m) | Rotation $[q_x, q_y, q_z, q_w]$ | Header Stamp ($s + ns$) |
|:--:|:---|:---|:---|:---|:---:|
| 1 | `camera_link` | `camera_depth_frame` | $[+0.000000, -0.000000, -0.000000]$ | $[+0.000000, +0.000000, +0.000000, +1.000000]$ | $3,307,191\text{s} + 21,506,862\text{ns}$ |
| 2 | `camera_depth_frame` | `camera_depth_optical_frame` | $[+0.000000, -0.000000, -0.000000]$ | $[-0.500000, +0.500000, -0.500000, +0.500000]$ | $3,307,191\text{s} + 21,506,862\text{ns}$ |
| 3 | `camera_link` | `camera_color_frame` | $[-0.000172, +0.014856, +0.000128]$ | $[+0.000970, +0.002220, +0.000980, +0.999997]$ | $3,307,191\text{s} + 100,265,816\text{ns}$ |
| 4 | `camera_color_frame` | `camera_color_optical_frame` | $[+0.000000, -0.000000, -0.000000]$ | $[-0.500000, +0.500000, -0.500000, +0.500000]$ | $3,307,191\text{s} + 100,265,816\text{ns}$ |

---

## 2. Sensor Frame Connectivity Status in Dataset

| Sensor / Subsystem | Frame ID | Present in `/tf_static`? | TF Connectivity Status |
|:---|:---|:---:|:---|
| **Unitree L1 (Raw)** | `utlidar_lidar` | ❌ NO | **Disconnected** (No transform to robot body or IMU) |
| **Unitree L1 (Deskewed)** | `odom` | ❌ NO | Published directly in global odometry frame |
| **Hesai XT-16** | `hesai_lidar` | ❌ NO | **Disconnected** (No transform to L1, IMU, or body) |
| **Unitree IMU** | `utlidar_imu` | ❌ NO | **Disconnected** (No transform to LiDARs or body) |
| **Robot Body** | `base_link` / `trunk` | ❌ NO | **Missing** from TF tree |
| **RGB-D Camera** | `camera_link` | ✅ YES | Fully connected to depth/color optical frames |

---

## 3. FAST-LIO2 Runtime Configuration Status

1. **Host Storage Audit:**
   - A template configuration was found at `/mnt/d/acsar-3d-mapping-research/03_fastlio2/fastlio2.yaml` with `T_imu_lidar: null`.
   - **Crucial Clarification:** This host file represents an offline post-processing template, not the configuration used during data collection.
2. **Onboard Robot Runtime:**
   - During recording, `/utlidar/cloud_deskewed` and `/utlidar/robot_odom` were published live by the Go2's internal companion computer (onboard Jetson/Rockchip).
   - The runtime extrinsics between Unitree L1 and the internal IMU were compiled into the onboard `unitree_lidar_sdk` firmware or configured directly on the robot's onboard filesystem (`/home/unitree/...`).
   - These runtime extrinsics were **not mirrored into the bag's `/tf_static` topic**.

---

## 4. Required Calibration Inputs for Dual-LiDAR Fusion (M3)

To perform fusion between Hesai XT-16 and Unitree L1:

1. **Hesai $\rightarrow$ L1 Rigid Transform ($T_{\text{L1}}^{\text{Hesai}}$):**
   - **Status:** **MISSING** from all bag recordings and host storage.
   - **Requirement:** Must be measured from CAD drawings of the robot mounting bracket or estimated via hand-eye / target-based point cloud registration (e.g. GICP on static scans of a planar room).
2. **L1 $\rightarrow$ IMU Extrinsics ($T_{\text{IMU}}^{\text{L1}}$):**
   - **Status:** Handled onboard internally by Unitree for L1 deskewing, but unavailable in TF for external nodes.
3. **Sensor Mounts $\rightarrow$ Quadruped Trunk ($T_{\text{trunk}}^{\text{L1}}$, $T_{\text{trunk}}^{\text{Hesai}}$):**
   - **Status:** **MISSING**. Required for gait-phase kinematics projection.
