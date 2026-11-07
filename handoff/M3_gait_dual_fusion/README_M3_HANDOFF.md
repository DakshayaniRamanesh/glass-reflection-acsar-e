# ACSAR-E M1 -> M3 Handoff: Gait Dynamics & Dual-LiDAR Fusion

## Milestone Status:
- **Gait Dynamics & Leg Kinematics:** **READY for gait feature extraction.**
- **Dual-LiDAR Fusion:** **NOT READY for validated dual-LiDAR fusion.**

## What is READY for M3:
1. **Authoritative Locomotion Exports:**
   - `exports/sportmode_validated.npz` (23,785 frames, 268.7 Hz): Body odometry, velocity up to 1.95 m/s, body height (0.072m sitting to 0.338m standing), roll/pitch/yaw.
   - `exports/lowstate_validated.npz` (40,015 frames, 451.6 Hz): 12 motor joint positions ($q$), velocities ($dq$ up to 14.1 rad/s), and torques ($	au_{\text{est}}$ up to 26.4 Nm).
   - 4-leg foot contact forces (`foot_force`): active across all 40,015 frames (1 to 211 raw sensor counts).
   - LowState 1.0 ms hardware tick counter (`tick`) preserved for microsecond gait phase timing.
2. **Gait Phase Awareness:**
   - Clear diagonal stance/swing alternations (FR+RL vs FL+RR) at ~2.5–3.5 Hz stepping frequency.

## What is NOT READY for M3 (Dual-LiDAR Fusion):
1. **Spatial Extrinsic Calibration:**
   - Estimated extrinsic transform: $\mathbf{t} = [-0.0968, -0.0415, 0.1358]\text{ m}$, $\text{RPY} = [-9.41^\circ, -2.73^\circ, -11.07^\circ]$.
   - Validation RMSE is 5.92 cm (2.75 cm inlier residual).
   - Along-track corridor translation is weakly observable due to lack of orthogonal reference planes.
2. **Temporal Synchronization:**
   - No hardware PPS or PTP clock sync was active between Hesai and Unitree RTC during recording.
   - Cross-device temporal alignment relies on transport receipt timestamps subject to ~120 ms jitter.
   - **M3 fusion must not claim millimeter or microsecond dual-LiDAR registration** without target-based and hardware-synchronized recordings.
