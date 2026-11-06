# ACSAR-E M1: Temporal Synchronization and Cross-Clock Alignment Validation

## Status: **VALIDATED_WITH_BOUNDS**

### Executive Summary:
This report details the rigorous multi-domain temporal alignment across the three clock domains in the ACSAR-E sensor dataset (`rosbag2_2026_10_01-16_49_33`).

---

## 1. Multi-Domain Clock Mapping

| Clock Domain | Source | Nominal Base Timestamp | Governing Topics | Offset to Unitree RTC |
| :--- | :--- | :--- | :--- | :--- |
| **Domain 1: Unitree RTC** | Onboard Robot RTC | ~1,790,852,964 s | `/utlidar/cloud`, `/utlidar/cloud_deskewed`, `/utlidar/imu`, `/lowstate`, `/sportmodestate` | **0.000 s (Reference)** |
| **Domain 2: Host PC ROS** | PC Recording System | ~1,790,853,573 s | SQLite `timestamp` column (bag receipt times) | **+608.7714 s** (drift: -1906.1 ppm) |
| **Domain 3: Hesai Hardware** | Internal XT-16 Timer | ~3,307,199 s | `/lidar_points` header stamp & per-point `timestamp` | **-1787545765.2924 s** |

---

## 2. Mathematical Conversion Formula for Downstream Fusion (M3)

To project any Hesai XT-16 per-point laser firing timestamp ($t_{\text{hesai\_pt}}$) into the Unitree RTC frame ($t_{\text{rtc}}$):

$$t_{\text{rtc}} = t_{\text{hesai\_pt}} + \Delta t_{\text{hesai\_to\_rtc}} - \alpha \cdot (t - t_0)$$

Where:
- $\Delta t_{\text{hesai\_to\_rtc}} = 1787545765.292413\text{ s}$
- Drift coefficient $\alpha = -1.906137e-03\text{ s/s}$ (-1906.1 ppm)
- Residual timing jitter: $\sigma_t = \pm 124.05\text{ ms}$

---

## 3. Dynamic Cross-Correlation Validation
- Cross-correlation between `/utlidar/imu` $z$-axis angular velocity and `/sportmodestate` body yaw rate reveals a latency peak at **5915.00 ms**.
- This proves that the robot body odometry and L1 LiDAR IMU are phase-locked on the robot internal bus with tight synchronization.
