#!/usr/bin/env python3
"""
Update Team Readiness Statuses in all handoff packages:
- M2: READY for separate single-sensor feature development
- M3: READY for gait feature extraction, NOT READY for validated dual-LiDAR fusion
- M4: READY for developing the ground-truth protocol, NOT READY for classification accuracy measurements
"""

import os

BASE_DIR = "/mnt/d/glass-research/acsar_e_m1"
HANDOFF_DIR = os.path.join(BASE_DIR, "handoff")

m2_readme = os.path.join(HANDOFF_DIR, "M2_reflection_detection/README_M2_HANDOFF.md")
m3_readme = os.path.join(HANDOFF_DIR, "M3_gait_dual_fusion/README_M3_HANDOFF.md")
m4_readme = os.path.join(HANDOFF_DIR, "M4_ground_truth_labeling/README_M4_HANDOFF.md")

with open(m2_readme, "w") as f:
    f.write("""# ACSAR-E M1 -> M2 Handoff: Reflection & Glass Detection

## Milestone Status:
- **Status:** **READY for separate single-sensor feature development.**
- **Dual-Sensor Spatial Consistency:** **PRELIMINARY (NOT RESEARCH-READY)**.

## What is READY for M2:
1. **Hesai XT-16 Single-Sensor Feature Development:**
   - 18 representative scans exported in `exports/hesai/` preserving native `float64` per-point timestamps and `uint16` ring IDs.
   - Zero-coordinate laser dropouts masked via boolean `valid_mask`.
   - Hesai dual-return pairing structure verified (Return 1 in [0..15], Return 2 in [16..31] per 32-point block).
   - Recalculated dual-return divergence: 0.65% (stationary), 3.40%–3.58% (mid-corridor), 2.60%–2.65% (late-corridor).
2. **Unitree L1 Single-Sensor Feature Development:**
   - 18 representative scans exported in `exports/l1_raw/` and `exports/l1_deskewed/`.
   - Point field layout and data integrity validated 100% against official ROS2 PointCloud2 deserializer.

## Usage Constraints & Caveats:
- **Do not assume dual-return divergence equals confirmed glass.** Edge scattering, specular floor reflections, and window frames also produce dual returns.
- **Do not rely on sub-decimeter L1-Hesai spatial alignment** until multi-planar target calibration is completed.
""")

with open(m3_readme, "w") as f:
    f.write("""# ACSAR-E M1 -> M3 Handoff: Gait Dynamics & Dual-LiDAR Fusion

## Milestone Status:
- **Gait Dynamics & Leg Kinematics:** **READY for gait feature extraction.**
- **Dual-LiDAR Fusion:** **NOT READY for validated dual-LiDAR fusion.**

## What is READY for M3:
1. **Authoritative Locomotion Exports:**
   - `exports/sportmode_validated.npz` (23,785 frames, 268.7 Hz): Body odometry, velocity up to 1.95 m/s, body height (0.072m sitting to 0.338m standing), roll/pitch/yaw.
   - `exports/lowstate_validated.npz` (40,015 frames, 451.6 Hz): 12 motor joint positions ($q$), velocities ($dq$ up to 14.1 rad/s), and torques ($\tau_{\\text{est}}$ up to 26.4 Nm).
   - 4-leg foot contact forces (`foot_force`): active across all 40,015 frames (1 to 211 raw sensor counts).
   - LowState 1.0 ms hardware tick counter (`tick`) preserved for microsecond gait phase timing.
2. **Gait Phase Awareness:**
   - Clear diagonal stance/swing alternations (FR+RL vs FL+RR) at ~2.5–3.5 Hz stepping frequency.

## What is NOT READY for M3 (Dual-LiDAR Fusion):
1. **Spatial Extrinsic Calibration:**
   - Estimated extrinsic transform: $\\mathbf{t} = [-0.0968, -0.0415, 0.1358]\\text{ m}$, $\\text{RPY} = [-9.41^\\circ, -2.73^\\circ, -11.07^\\circ]$.
   - Validation RMSE is 5.92 cm (2.75 cm inlier residual).
   - Along-track corridor translation is weakly observable due to lack of orthogonal reference planes.
2. **Temporal Synchronization:**
   - No hardware PPS or PTP clock sync was active between Hesai and Unitree RTC during recording.
   - Cross-device temporal alignment relies on transport receipt timestamps subject to ~120 ms jitter.
   - **M3 fusion must not claim millimeter or microsecond dual-LiDAR registration** without target-based and hardware-synchronized recordings.
""")

with open(m4_readme, "w") as f:
    f.write("""# ACSAR-E M1 -> M4 Handoff: Ground-Truth Protocol & Evaluation

## Milestone Status:
- **Protocol Development:** **READY for developing the ground-truth protocol.**
- **Accuracy Benchmarking:** **NOT READY for classification accuracy measurements.**

## What is READY for M4:
1. **Scene Inventory & Imagery:**
   - `scene_inventory.csv`: 6 benchmark candidate scene epochs isolated. All surface classes explicitly designated **UNKNOWN / UNVERIFIED**.
   - 6 candidate RGB images in `candidate_scenes/`.
   - Camera intrinsics documented ($1280 \\times 720$ RGB8, $f_x=911.99, f_y=911.48$).
2. **Standardized Annotation Framework:**
   - `annotation_template.csv`: Standardized tabular schema (point coordinates, sensor type, return index, surface class, confidence score, verification method). Intentionally blank with no fabricated measurements.
   - `ground_truth_requirements.md`: Comprehensive survey requirements protocol (corridor tape survey, CAD boundaries, optical fiducials).

## Why M4 is NOT READY for Accuracy Measurements:
1. **Downward Camera Angle:**
   - The onboard camera points downwards, predominantly imaging the floor and lower skirting boards. Transparent glass panels are not confirmed from the visual imagery.
2. **Lack of Physical Survey Data:**
   - No architectural CAD or surveyed glass coordinates exist for the recorded bag sessions.
   - M4 must execute a controlled recording session with known glass targets before quantitative classifier accuracy (precision/recall/ROC) can be evaluated.
""")

print("Updated all handoff package READMEs with explicit readiness statuses.")

