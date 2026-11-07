#!/usr/bin/env python3
"""
Update calibration and temporal alignment reports with reconciled facts and updated readiness statuses.
"""

import os
import json
import yaml

REPORT_DIR = "/mnt/d/glass-research/acsar_e_m1/reports"
HANDOFF_M2_DIR = "/mnt/d/glass-research/acsar_e_m1/handoff/M2_reflection_detection"
HANDOFF_M3_DIR = "/mnt/d/glass-research/acsar_e_m1/handoff/M3_gait_dual_fusion"
HANDOFF_M4_DIR = "/mnt/d/glass-research/acsar_e_m1/handoff/M4_ground_truth_labeling"

# -----------------------------------------------------------------------------
# 1. Update time_alignment_candidate.json
# -----------------------------------------------------------------------------
time_candidate = {
    "temporal_alignment_status": "PRELIMINARY (UNVERIFIED AT SUB-SCAN SCALE)",
    "primary_reference_clock": "unitree_onboard_rtc",
    "discrepancy_reconciliation": {
        "finding": (
            "The previously reported 5,915 ms cross-correlation peak was an artifact of running "
            "correlation on an inverted gyro axis (the L1 IMU z-axis is inverted relative to robot body yaw). "
            "When properly inverted, the correlation peak between Unitree IMU and SportModeState is at 0.0 ms. "
            "The summary claim of '5.9 ms' was an erroneous typographical division of 5,915 ms by 1000."
        ),
        "scope_limitation": (
            "The IMU-to-SportMode correlation only establishes internal phase alignment within the Unitree robot bus. "
            "It does NOT establish Hesai-to-L1 cross-device synchronization."
        )
    },
    "clock_domains": {
        "unitree_rtc": {
            "description": "Hardware RTC shared by L1 LiDAR, utlidar/imu, and Go2 lowstate",
            "base_epoch_sec": 1790852964.320259
        },
        "host_pc_ros": {
            "description": "Host PC clock recorded in ROS2 bag receipt timestamps",
            "mean_offset_to_unitree_rtc_sec": 608.7714,
            "linear_drift_rate_ppm": -1906.1,
            "transport_jitter_std_ms": 120.77
        },
        "hesai_hardware": {
            "description": "Internal Hesai hardware timestamp (use_timestamp_type: 1 in driver)",
            "base_epoch_sec": 3307199.131274,
            "offset_to_unitree_rtc_sec": 1787545765.292413,
            "status": "PRELIMINARY",
            "notes": (
                "No hardware PPS or PTP synchronization was present during recording. "
                "Hesai-to-Unitree mapping relies on transport receipt timestamps subject to ~120 ms jitter."
            )
        }
    },
    "m3_fusion_readiness": "NOT READY for validated dual-LiDAR fusion due to unverified sub-scan cross-clock synchronization."
}

with open(os.path.join(REPORT_DIR, "time_alignment_candidate.json"), "w") as f:
    json.dump(time_candidate, f, indent=2)

with open(os.path.join(HANDOFF_M3_DIR, "time_alignment_candidate.json"), "w") as f:
    json.dump(time_candidate, f, indent=2)

# -----------------------------------------------------------------------------
# 2. Update time_alignment_validation.md
# -----------------------------------------------------------------------------
time_report = """# ACSAR-E M1: Temporal Synchronization and Cross-Clock Alignment Validation

## Status: **PRELIMINARY (UNVERIFIED AT SUB-SCAN SCALE)**

### Critical Reconciliation of Synchronization Claims:
1. **The 5,915 ms vs 5.9 ms Discrepancy:**
   - In earlier analysis, cross-correlation between `/utlidar/imu` $z$-gyro and `/sportmodestate` yaw rate reported a lag peak of **5,915 ms**.
   - **Root Cause:** The Unitree L1 chin IMU $z$-axis is inverted relative to the robot body coordinate system ($\omega_z^{\\text{imu}} \\approx -\\omega_z^{\\text{sport}}$). Running correlation without inverting sign forced the algorithm to find a weak secondary sidelobe at $+5.91\\text{ s}$ ($r = 95.5$).
   - When inverted ($-\\omega_z^{\\text{imu}}$), the correlation peak is at **exactly 0.0 ms** ($r = 418.6$).
   - The summary mention of "5.9 ms" was an erroneous clerical typo that divided 5,915 ms by 1,000.
2. **Scope of the IMU Correlation:**
   - Both `/utlidar/imu` and `/sportmodestate` run on the internal Unitree robot hardware bus.
   - **This internal correlation cannot and does not establish Hesai-to-L1 synchronization.**
3. **Cross-Device Hesai Timing Evidence:**
   - Hesai XT-16 runs on its independent internal timer (~3,307,199 s from boot).
   - No hardware PPS (pulse-per-second) or PTP (IEEE 1588) synchronization was active during this recording.
   - The estimated offset ($\Delta t \\approx 1,787,545,765.29\\text{ s}$) is derived from host PC receipt timestamps, which are corrupted by network transport jitter ($\sigma \\approx 120.8\\text{ ms}$) and clock drift ($-1906.1\\text{ ppm}$).
   - **Conclusion:** Cross-device Hesai-to-L1 synchronization is designated **PRELIMINARY**. Sub-scan or point-level dual-LiDAR fusion cannot be guaranteed without controlled PTP hardware synchronization.
"""

with open(os.path.join(REPORT_DIR, "time_alignment_validation.md"), "w") as f:
    f.write(time_report)

with open(os.path.join(HANDOFF_M3_DIR, "time_alignment_validation.md"), "w") as f:
    f.write(time_report)

# -----------------------------------------------------------------------------
# 3. Update calibration_validation.md & hesai_to_l1_candidate.yaml
# -----------------------------------------------------------------------------
calib_yaml = {
    "calibration_status": "PRELIMINARY (NOT RESEARCH-READY)",
    "parent_frame": "utlidar_lidar",
    "child_frame": "hesai_lidar",
    "investigation_findings": {
        "double_applied_yaw_bug": (
            "The Hesai driver already applied +90 deg yaw rotation (transform_flag: true). "
            "Earlier calibration scripts mistakenly assumed Hesai was still unrotated and applied another -90 deg yaw, "
            "forcing cross-track features to match along-track features."
        ),
        "inlier_ratio_explanation": (
            "The 7.7% inlier count in earlier reports was caused by comparing a 360-degree surround Hesai cloud "
            "(56,700 points) against a single non-repetitive L1 sub-scan packet containing only 503 points."
        ),
        "refined_forward_icp": {
            "method": "Accumulated stationary scans (18,577 L1 points vs 12,833 Hesai points)",
            "estimated_translation_m": [-0.0968, -0.0415, 0.1358],
            "estimated_rpy_deg": [-9.41, -2.73, -11.07],
            "inlier_rmse_cm": 2.75,
            "validation_rmse_cm": 5.92
        },
        "degeneracy_caveat": (
            "The test corridor environment lacks orthogonal reference planes along the hallway heading. "
            "While vertical and cross-track alignment are constrained, along-track translation remains weakly observable. "
            "Target-based calibration with multi-planar corner reflectors is required."
        )
    },
    "m3_fusion_readiness": "NOT READY for validated dual-LiDAR fusion."
}

with open(os.path.join(REPORT_DIR, "hesai_to_l1_candidate.yaml"), "w") as f:
    yaml.dump(calib_yaml, f, default_flow_style=False)

with open(os.path.join(HANDOFF_M3_DIR, "hesai_to_l1_candidate.yaml"), "w") as f:
    yaml.dump(calib_yaml, f, default_flow_style=False)

calib_report = """# ACSAR-E M1: Spatial Extrinsic Calibration Validation (Hesai XT-16 to L1)

## Status: **PRELIMINARY (NOT RESEARCH-READY)**

### Investigation of Calibration Inconsistencies:
1. **Identification of Double-Applied Yaw Rotation:**
   - In `hesai-config.yaml`, the driver was configured with `transform_flag: true, yaw: 1.57079632679` (+90°).
   - This driver transform rotated Hesai point coordinates into the robot body frame ($X$ forward).
   - Earlier calibration attempts mistakenly assumed Hesai required a $-90^\circ$ yaw rotation, resulting in a **double-applied yaw error** that oriented Hesai points $90^\circ$ perpendicular to L1!
2. **Limited Overlapping Geometry & Inlier Ratio:**
   - Hesai XT-16 has a $360^\circ$ horizontal surround field-of-view out to 20+ meters.
   - Unitree L1 is a front-facing dome sensor focused on the forward hemisphere.
   - When evaluating across all Hesai points, ~80% of points fall outside L1's physical coverage.
3. **Refined Forward-Facing Registration:**
   - By accumulating stationary scans (18,577 L1 points and 12,833 downsampled Hesai points) and using the true nominal forward-facing mounting:
     $$\\mathbf{t} = [-0.0968, -0.0415, +0.1358]\\text{ m}, \\quad \\text{RPY} = [-9.41^\\circ, -2.73^\\circ, -11.07^\\circ]$$
   - Physical plausibility: Hesai sits on the front top deck (~9.7 cm behind chin L1, ~4.1 cm offset, and ~13.6 cm above chin L1).
   - Inlier residual drops to **2.75 cm** (with validation RMSE of **5.92 cm**).
4. **Corridor Degeneracy:**
   - Because the recording was performed in an indoor corridor with smooth parallel walls, the along-track translation axis lacks orthogonal constraint planes.
   - Therefore, M3 is **NOT READY** for validated dual-LiDAR fusion until multi-surface target-based calibration is performed.
"""

with open(os.path.join(REPORT_DIR, "calibration_validation.md"), "w") as f:
    f.write(calib_report)

with open(os.path.join(HANDOFF_M3_DIR, "calibration_validation.md"), "w") as f:
    f.write(calib_report)

print("Updated time alignment and calibration reports.")

