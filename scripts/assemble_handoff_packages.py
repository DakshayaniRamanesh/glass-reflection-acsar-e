#!/usr/bin/env python3
"""
Assemble M2, M3, M4 Handoff Packages with explicit status declarations.
"""

import os
import shutil

BASE_DIR = "/mnt/d/glass-research/acsar_e_m1"
HANDOFF_DIR = os.path.join(BASE_DIR, "handoff")

m2_dir = os.path.join(HANDOFF_DIR, "M2_reflection_detection")
m3_dir = os.path.join(HANDOFF_DIR, "M3_gait_dual_fusion")
m4_dir = os.path.join(HANDOFF_DIR, "M4_ground_truth_labeling")

os.makedirs(m2_dir, exist_ok=True)
os.makedirs(m3_dir, exist_ok=True)
os.makedirs(m4_dir, exist_ok=True)

# -----------------------------------------------------------------------------
# 1. M2 Package (Reflection Detection)
# -----------------------------------------------------------------------------
# Copy reports and scan references
shutil.copy(os.path.join(BASE_DIR, "reports/hesai_repetition_analysis.md"), m2_dir)
shutil.copy(os.path.join(BASE_DIR, "reports/scan_index.csv"), m2_dir)

with open(os.path.join(m2_dir, "README_M2_HANDOFF.md"), "w") as f:
    f.write("""# ACSAR-E M1 -> M2 Handoff Package: Reflection & Glass Detection

## Status:
- **Single-Sensor Reflection Features (Hesai):** **READY**
  - Native float64 timestamps and uint16 ring channels preserved.
  - Zero-coordinate laser dropouts masked out via boolean `valid_mask`.
  - Dual-return divergence analysis verified: 0.65% (stationary) to 3.58% (glass partitions).
- **Dual-Sensor Spatial Consistency (L1 vs Hesai):** **PRELIMINARY**
  - Extrinsic calibration residual is 7.20 cm (corridor geometric degeneracy).

## Included Artifacts:
1. `hesai_repetition_analysis.md`: Rigorous 4-way classification of dual-return returns.
2. `scan_index.csv`: Master index of all 18 representative scans (stationary, trot, turning).
3. Representative multi-epoch scans located at: `acsar_e_m1/exports/hesai/` and `acsar_e_m1/exports/l1_raw/`.

## Critical Usage Instructions for M2:
- Do not interpret dual-return divergence alone as definitive glass without cross-referencing surface normal and camera reprojection.
- Use `valid_mask` to filter out zero-coordinate packets before computing spatial features.
""")

# -----------------------------------------------------------------------------
# 2. M3 Package (Gait-Aware Fusion)
# -----------------------------------------------------------------------------
shutil.copy(os.path.join(BASE_DIR, "reports/gait_validation_report.md"), m3_dir)
shutil.copy(os.path.join(BASE_DIR, "reports/time_alignment_candidate.json"), m3_dir)
shutil.copy(os.path.join(BASE_DIR, "reports/time_alignment_validation.md"), m3_dir)
shutil.copy(os.path.join(BASE_DIR, "reports/hesai_to_l1_candidate.yaml"), m3_dir)
shutil.copy(os.path.join(BASE_DIR, "reports/calibration_validation.md"), m3_dir)

with open(os.path.join(m3_dir, "README_M3_HANDOFF.md"), "w") as f:
    f.write("""# ACSAR-E M1 -> M3 Handoff Package: Gait Dynamics & Dual-LiDAR Fusion

## Status:
- **Leg Kinematics & Ground Contact Dynamics:** **READY**
  - Full-rate 451.6 Hz `/lowstate` telemetry: 12 motor joint positions, velocities, torques, and 4 foot contact forces.
  - Body odometry at 268.7 Hz from `/sportmodestate`.
- **Temporal Synchronization:** **VALIDATED_WITH_BOUNDS**
  - Precise piecewise-linear conversion between Hesai hardware clock and Unitree RTC clock.
  - Angular velocity cross-correlation demonstrates sub-10ms bus phase locking.
- **Spatial Dual-LiDAR Extrinsics:** **PRELIMINARY**
  - Transform estimate: `t = [-0.1736, -0.0579, 0.0524] m`, `RPY = [2.69°, -23.87°, -90.27°]`.
  - Independent validation RMSE = 7.20 cm. Designated PRELIMINARY due to along-track corridor degeneracy.

## Included Artifacts:
1. `sportmode_validated.npz` (in `exports/`): 23,785 body odometry frames.
2. `lowstate_validated.npz` (in `exports/`): 40,015 full leg telemetry frames.
3. `gait_validation_report.md`: Complete statistical audit and gait phase visualizations.
4. `time_alignment_candidate.json`: Mathematical clock conversion formulas.
5. `hesai_to_l1_candidate.yaml`: Candidate spatial transform with covariance notes.
""")

# -----------------------------------------------------------------------------
# 3. M4 Package (Ground-Truth Labeling)
# -----------------------------------------------------------------------------
shutil.copy(os.path.join(BASE_DIR, "reports/scene_inventory.csv"), m4_dir)
shutil.copy(os.path.join(BASE_DIR, "reports/annotation_template.csv"), m4_dir)
shutil.copy(os.path.join(BASE_DIR, "reports/ground_truth_requirements.md"), m4_dir)

# Copy candidate scene images
m4_img_dir = os.path.join(m4_dir, "candidate_scenes")
os.makedirs(m4_img_dir, exist_ok=True)
src_img_dir = os.path.join(BASE_DIR, "exports/candidate_scenes")
if os.path.exists(src_img_dir):
    for f_name in os.listdir(src_img_dir):
        if f_name.endswith(".png"):
            shutil.copy(os.path.join(src_img_dir, f_name), m4_img_dir)

with open(os.path.join(m4_dir, "README_M4_HANDOFF.md"), "w") as f:
    f.write("""# ACSAR-E M1 -> M4 Handoff Package: Independent Ground-Truth Protocol

## Status:
- **Visual Grounding & Scene Inventory:** **READY**
  - 6 representative benchmark scene epochs extracted from `rosbag2_2026_10_01-16_49_33`.
  - Color camera intrinsics and 1280x720 RGB candidate images extracted.
- **Physical Ground-Truth Survey Requirements:** **DOCUMENTED**
  - Standardized taxonomy of 8 surface classes.
  - Required physical measurements specified (wall-to-wall corridor width, glass CAD planes, Leica DISTO fiducials).

## Included Artifacts:
1. `scene_inventory.csv`: Index of benchmark scenes with robot motion state and glass presence.
2. `annotation_template.csv`: Standardized tabular schema for point-level ground-truth labeling.
3. `ground_truth_requirements.md`: Experimental protocol and survey requirements for M4.
4. `candidate_scenes/`: 6 high-resolution camera images corresponding to the benchmark epochs.
""")

print("All handoff packages assembled successfully.")

