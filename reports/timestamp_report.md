# ACSAR-E M1 — Validated Timestamp & Clock Domain Analysis Report

**Date:** 2026-11-06  
**Status:** Validated via 300-frame empirical residual & jitter analysis  
**Target Dataset:** `rosbag2_2026_10_01-16_49_33` (88.62 s duration, 107,416 messages)  

---

## 1. Executive Summary & Clock Domain Topology

Empirical measurement across all sensor streams reveals **three separate, unsynchronized clock domains**:

```
                       ┌────────────────────────────────────────┐
                       │       Host Recording Laptop            │
                       │   Wall Clock: Unix Epoch (2026 UTC)    │
                       │   Used for: rosbag2 SQLite `timestamp` │
                       └──────────────────┬─────────────────────┘
                                          │  Network & transport jitter:
                                          │  ±18.5 ms (L1), ±38.1 ms (Hesai)
                   ┌──────────────────────┴──────────────────────┐
                   │                                             │
┌──────────────────▼──────────────────┐       ┌──────────────────▼──────────────────┐
│     Unitree Go2 Robot RTC Domain    │       │     Hesai XT-16 Firmware Domain     │
│  Offset: +608.97 s (~10.15 min lag) │       │  Base: ~3,307,199 s (~38.28 days)   │
│  Rate: +869 ppm relative to Host    │       │  Rate: +1306 ppm relative to Host   │
├─────────────────────────────────────┤       ├─────────────────────────────────────┤
│ • /utlidar/cloud (L1 Raw, 15.4 Hz)  │       │ • /lidar_points (Hesai, 10.0 Hz)    │
│ • /utlidar/cloud_deskewed (14.9 Hz) │       │ • Intra-packet firing blocks        │
│ • /utlidar/imu (IMU, 247 Hz)        │       │                                     │
│ • /utlidar/robot_odom (135 Hz)      │       │ Origin: UNRESOLVED                  │
│ • /sportmodestate (268 Hz)          │       │ (Firmware boot-time / unsynced PTP) │
└─────────────────────────────────────┘       └─────────────────────────────────────┘
```

---

## 2. Topic-by-Topic Timing & Residuals (300 Consecutive Frames)

We measured inter-frame publication intervals, clock drift rates, and recorder transport jitter across 300 consecutive frames per topic:

| Stream | Frame ID | Nominal Rate | Header Interval Mean ± Std (ms) | Interval Range [Min, Max] (ms) | Storage Transport Jitter (Std) | Transport Lag Range (ms) | Initial Offset to Bag Time (s) |
|:---|:---|:---:|:---:|:---:|:---:|:---:|:---:|
| **`/utlidar/cloud`** (L1 Raw) | `utlidar_lidar` | 15.4 Hz | $65.80 \pm 11.84$ | $[63.18, 259.39]$ | **$\pm 18.50$ ms** | $[-30.4, +26.3]$ | $+608.978$ s |
| **`/utlidar/cloud_deskewed`** | `odom` | 14.9 Hz | $66.02 \pm 9.88$ | $[62.72, 194.48]$ | **$\pm 18.43$ ms** | $[-34.2, +31.4]$ | $+608.928$ s |
| **`/utlidar/imu`** | `utlidar_imu` | 247.0 Hz | $4.05 \pm 1.38$ | $[0.16, 13.45]$ | **$\pm 2.15$ ms** | $[-0.39, +20.9]$ | $+608.919$ s |
| **`/utlidar/robot_odom`** | `odom` | 135.2 Hz | $7.40 \pm 1.82$ | $[0.42, 21.10]$ | **$\pm 2.85$ ms** | $[-1.12, +24.6]$ | $+608.915$ s |
| **`/lidar_points`** (Hesai) | `hesai_lidar` | 10.0 Hz | $100.00 \pm 0.59$ | $[97.42, 101.48]$ | **$\pm 38.10$ ms** | $[-29.2, +337.8]$ | $+1,787,546,374.24$ s |

---

## 3. Detailed Clock Domain Findings

### 3.1 Unitree Subsystem Clock Coupling (L1, IMU, Odometry, Robot State)
- **Clock Origin:** The Unitree companion computer's system clock.
- **Systematic Offset:** Runs **$+608.97\text{ s}$ ($\approx 10\text{ minutes } 9\text{ seconds}$)** behind the host laptop's wall clock.
- **Clock Drift:** Drifts at **$+869.5\text{ ppm}$** ($\approx +77\text{ ms}$ over the $88.6\text{ s}$ recording) relative to the laptop crystal oscillator.
- **Inter-Stream Coherence:** L1 Raw, IMU, Odometry, and `/sportmodestate` share this exact same clock domain down to sub-millisecond precision:
  - $\text{Offset}(\text{L1}) - \text{Offset}(\text{IMU}) = +59.4\text{ ms}$ (representing the sensor optical integration window).
  - High confidence for fusing L1 LiDAR with IMU and quadruped leg odometry.

### 3.2 Hesai XT-16 Clock Domain: Unresolved Base Reference
- **Clock Origin:** Sensor internal firmware counter ($T \approx 3,307,199\text{ s} \approx 38.28\text{ days}$).
- **Rate Accuracy:** Advances by exactly **$88.600\text{ s}$** over the $88.485\text{ s}$ bag recording duration ($\Delta t_{\text{hdr}} / \Delta t_{\text{bag}} = 1.00013$), confirming nominal 10.0 Hz physical rotation.
- **Status: UNRESOLVED.** Without access to the Hesai web management interface or PTP grandmaster logs during recording, it cannot be verified whether this base represents elapsed uptime since power-on or an unanchored PTP/NTP epoch. The offset between Hesai and L1 **must be determined empirically** (e.g. through motion-onset cross-correlation).

### 3.3 Recorder Transport Jitter & Synchronization Uncertainty
- ROS2 recording timestamps (`bag_ts` in SQLite) capture message receipt over the robot's local Ethernet switch into the host laptop kernel.
- Because Hesai point clouds are large ($1.66\text{ MB}$ per message at 10 Hz), network buffering and thread scheduling produce transport delay standard deviations of **$\pm 38.10\text{ ms}$**, with buffer spikes reaching up to **$+337.8\text{ ms}$**.
- **Crucial Fusion Implication:**
  > [!WARNING]
  > Relying solely on rosbag receipt timestamps (`bag_ts`) to associate Hesai and Unitree L1 scans introduces an empirical temporal uncertainty of **$\pm 15$ to $\pm 45\text{ ms}$**.
  > This is acceptable for coarse spatial verification, but **insufficient** for microsecond-level gait-transition impact analysis (M3) until an empirical angular velocity cross-correlation calibration is performed.

---

## 4. Confidence & Readiness Matrix for Downstream Modules

| Sensor Pair | Proposed Alignment Key | Empirical Uncertainty | Readiness | Recommendation for M2 / M3 |
|:---|:---|:---:|:---:|:---|
| **L1 Raw ↔ L1 Deskewed** | Header stamp (`stamp_ns`) | $< 0.1\text{ ms}$ | **READY** | Direct 1:1 match with $+62.5\text{ ms}$ scan window offset. |
| **L1 Raw ↔ IMU** | Header stamp (`stamp_ns`) | $< 0.5\text{ ms}$ | **READY** | Direct interpolation of IMU angular velocities to L1 laser pulse times. |
| **L1 Raw ↔ SportModeState** | Header stamp (`stamp_ns`) | $< 1.0\text{ ms}$ | **READY** | Reliable gait-phase and foot contact state correlation. |
| **L1 Raw ↔ Hesai XT-16** | Bag receipt time (`bag_ts`) | $\pm 15\text{ to }\pm 45\text{ ms}$ | **PROTOTYPE ONLY** | Coarse bounding box gating only. Requires calibration before fusion. |
| **Hesai XT-16 ↔ LowState / Gait** | Interpolated via `bag_ts` | $\pm 38\text{ ms}$ | **BLOCKED** | Blocked until Hesai-L1 temporal offset calibration is performed. |
