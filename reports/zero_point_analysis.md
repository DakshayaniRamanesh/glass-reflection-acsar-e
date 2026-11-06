# ACSAR-E M1 — Zero-Coordinate Point Investigation

**Date:** 2026-11-06  
**Status:** Verified via official ROS2 `rclpy` deserializer & raw serialized buffer inspection  
**Dataset:** `rosbag2_2026_10_01-16_49_33`  

---

## 1. Executive Summary & Root Causes

Independent review flagged that exported samples contained high proportions of points with $(x=0, y=0, z=0)$. We verified the serialized ROS2 messages directly using both the official `sensor_msgs/msg/PointCloud2` deserializer and raw CDR byte slicing.

| LiDAR Topic | Total Points | Exact Zeros $(x=y=z=0)$ | Zero % | Valid Non-Zero Returns | Root Cause |
|---|:---:|:---:|:---:|:---:|:---|
| **`/utlidar/cloud`** (L1 Raw) | 1,028 – 1,200 (startup)<br>4,000 – 4,200 (steady) | **0** | **0.00%** | **100% Valid** | Driver publishes only detected physical laser returns without padding. |
| **`/utlidar/cloud_deskewed`** (L1 Deskewed) | 10,404 – 10,614 | **Exactly 10,000** | **94.2% – 96.1%** | **400 – 614** (3.9% – 5.8%) | **Pre-allocated zero padding** from the onboard Unitree deskewing node buffer. |
| **`/lidar_points`** (Hesai XT-16) | Exactly 64,000 | **6,038 – 6,102** | **9.4% – 9.5%** | **57,898 – 57,962** (90.5%) | **Laser non-returns** (beams shooting into sky, absorbed, or specular glass surfaces). |

---

## 2. In-Depth Analysis by Sensor Stream

### 2.1 Unitree L1 Deskewed (`/utlidar/cloud_deskewed`)

Across every sampled deskewed message in the bag, the total point count is $10,000 + N_{\text{valid}}$, where $N_{\text{valid}} \in [400, 614]$:
- The first $N_{\text{valid}}$ or trailing points contain physical returns with $x, y, z \ne 0$ and realistic intensity values ($37.0 \le \text{intensity} \le 255.0$, mean $\approx 155.0$).
- Exactly **10,000 points** in each message have $x = 0.000, y = 0.000, z = 0.000$ and $\text{intensity} = 0.000$.
- **Source of Zeros:** The onboard Unitree / FAST-LIO deskewing node pre-allocates an internal fixed buffer of 10,000 points. When the active scan accumulation produces only 400–600 motion-compensated points, the pre-allocated buffer is published with trailing/leading zero-fill rather than being dynamically truncated.

### 2.2 Hesai XT-16 (`/lidar_points`)

The Hesai XT-16 is a 16-channel organized spinning LiDAR emitting 64,000 pulses per revolution (16 rings $\times$ 4,000 azimuth firings at 10 Hz).
- Across all 5 sampled scans, exactly **~6,040 points (~9.44%)** have $(x=0, y=0, z=0, \text{intensity}=0)$.
- **Source of Zeros:** Standard Hesai ROS2 driver behavior for laser pulses that receive **no detectable optical return** (e.g. upward-angled beams pointing into the sky, beams absorbed by dark surfaces, or specular reflection away from the receiver on smooth glass surfaces).
- Rather than outputting `NaN` or omitting azimuth slots, the driver assigns $(0, 0, 0)$ to preserve the fixed 64,000 point matrix structure.

### 2.3 Unitree L1 Raw (`/utlidar/cloud`)

- Exactly **0 points** have $(x=0, y=0, z=0)$ across all sampled scans.
- All returns have valid coordinates and non-zero intensities ($\text{mean} \approx 135.0$).

---

## 3. Scientific Implications for ACSAR-E Reflection Detection (M2 / M3)

> [!IMPORTANT]
> **Do not blindly delete $(0, 0, 0)$ returns!**
> In LiDAR perception near glass and mirrors, **non-returns (dropouts) carry critical signal**:
> - Specular reflection causes laser energy to bounce away from the receiver, creating an apparent dropout $(0, 0, 0)$ where a physical glass wall actually exists.
> - Treating non-returns as "corrupted data to discard" would destroy the negative evidence needed by Module M2 to identify glass planes.

### Recommended Data Handling Strategy:

1. **Validity Mask (`valid_mask`):**
   Every exported `.npz` scan now includes a boolean `valid_mask` array:
   $$\text{valid\_mask} = \neg(x=0 \land y=0 \land z=0) \land \text{isfinite}(x) \land \text{isfinite}(y) \land \text{isfinite}(z)$$
2. **For Geometric Registration & FAST-LIO2 (M1 / M4):**
   Filter using `points[valid_mask]` to avoid false point clusters at $(0, 0, 0)$.
3. **For Glass Reflection Detection (M2):**
   Use `valid_mask` to distinguish between confirmed physical returns (`valid_mask == True`) and unreturned/absorbed beam trajectories (`valid_mask == False`).
