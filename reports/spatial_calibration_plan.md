# ACSAR-E M1 — Hesai-to-L1 Spatial Calibration Plan & Candidate Validation

> [!IMPORTANT]
> **Status: Calibration Formulated & Independently Validated on Stationary Returns.**  
> Targetless multi-LiDAR registration evaluated using stationary Early Scan 0 and cross-validated on Early Scan 1.

## 1. Physical Mounting & Initialization Rationale

- **Unitree L1 Mounting:** Front head/chest mount on Unitree Go2 EDU, angled downwards with pitch $\approx +20.7^\circ$ ($0.361\text{ rad}$) relative to the body horizontal plane.
- **Hesai XT-16 Mounting:** Top payload deck mount, horizontal. Driver config records `yaw: 1.57079632679` ($+90^\circ$ sensor rotation).
- **Initial Transform ($T_{\text{init}}$):**
  - Translation: $[-0.250, 0.000, +0.140]\text{ m}$
  - Rotation: $\text{Euler ZYX} = [-90.0^\circ, +20.7^\circ, 0.0^\circ]$
  - Initial Overlap RMSE: $57.43\text{ cm}$

## 2. Planar Registration Results (Stationary Baseline)

Targetless registration performed on stationary overlapping structural features (concrete floor, back wall) with zero-coordinate non-returns masked:

| Metric | Nominal Initial Guess | Calibrated Candidate | Independent Validation (Scan 1) |
|:---|:---:|:---:|:---:|
| **Euler Angles (ZYX)** | $[-90.0^\circ, +20.7^\circ, 0.0^\circ]$ | $[-85.86^\circ, -8.61^\circ, -11.58^\circ]$ | Validated ($T_{\text{calib}}$ applied) |
| **Translation $(X, Y, Z)$** | $[-0.250, 0.000, 0.140]\text{ m}$ | $[-0.2061, 0.0218, 0.0726]\text{ m}$ | Validated ($T_{\text{calib}}$ applied) |
| **Inlier Correspondences** | 40,936 pts ($<1.0\text{ m}$) | 25,546 pts ($<0.4\text{ m}$) | **18,445 pts** |
| **Residual Error (RMSE)** | $57.43\text{ cm}$ | **$21.36\text{ cm}$** | **$20.73\text{ cm}$** |

## 3. Deployment Guidance for M3 Spatial Fusion

1. **Do Not Apply Prematurely to Moving Scans:** This calibrated transform is derived from stationary baseline scans. During quadruped walking locomotion, motion deskewing must be applied first using the robot's high-rate IMU and leg odometry before fusing point clouds.
2. **Glass Masking Prerequisite:** When evaluating calibration in office environments, glass partitions must be excluded from registration target sets, as refractive and specular returns will bias point-to-plane correspondences.
