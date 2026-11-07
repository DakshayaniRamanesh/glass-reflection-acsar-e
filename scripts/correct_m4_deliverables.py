#!/usr/bin/env python3
"""
Correct M4 Ground Truth Deliverables:
1. scene_inventory.csv - Marking unverified scenes as UNKNOWN, noting downward camera view
2. annotation_template.csv - Clean, blank template with exact schema headers
3. ground_truth_requirements.md - Rigorous experimental protocol without fabricated claims
"""

import os
import csv

REPORT_DIR = "/mnt/d/glass-research/acsar_e_m1/reports"
HANDOFF_M4_DIR = "/mnt/d/glass-research/acsar_e_m1/handoff/M4_ground_truth_labeling"
os.makedirs(REPORT_DIR, exist_ok=True)
os.makedirs(HANDOFF_M4_DIR, exist_ok=True)

# -----------------------------------------------------------------------------
# 1. scene_inventory.csv
# -----------------------------------------------------------------------------
scenes = [
    {
        "scene_id": "SCENE_01_STAT_START",
        "bag_name": "rosbag2_2026_10_01-16_49_33",
        "time_window_s": "0.0 - 5.0",
        "robot_motion_state": "Stationary (Crouched / Sitting)",
        "sensor_streams_present": "L1_Raw, L1_Deskewed, Hesai, Camera_Color, Camera_Depth, LowState, SportModeState",
        "camera_view_description": "Downward-angled view capturing floor and adjacent robot legs / laboratory base",
        "candidate_surface_type": "UNKNOWN (Floor / Baseboard)",
        "glass_presence_verification": "UNVERIFIED (No surveyed glass fiducials in FOV)",
        "recommended_action": "Use as stationary noise baseline; controlled survey required for surface grounding."
    },
    {
        "scene_id": "SCENE_02_TRANS_STAND",
        "bag_name": "rosbag2_2026_10_01-16_49_33",
        "time_window_s": "10.0 - 18.0",
        "robot_motion_state": "Standing Up & Gait Initiation",
        "sensor_streams_present": "L1_Raw, L1_Deskewed, Hesai, Camera_Color, Camera_Depth, LowState, SportModeState",
        "camera_view_description": "Pitch elevation as robot stands; floor and lower wall boundary visible",
        "candidate_surface_type": "UNKNOWN (Corridor Floor / Wall Base)",
        "glass_presence_verification": "UNVERIFIED",
        "recommended_action": "Benchmark for foot-strike impact vibration on LiDAR point jitter."
    },
    {
        "scene_id": "SCENE_03_CORRIDOR_WALK",
        "bag_name": "rosbag2_2026_10_01-16_49_33",
        "time_window_s": "30.0 - 45.0",
        "robot_motion_state": "Continuous Trot (v ~ 0.85 m/s)",
        "sensor_streams_present": "L1_Raw, L1_Deskewed, Hesai, Camera_Color, Camera_Depth, LowState, SportModeState",
        "camera_view_description": "Corridor floor, skirting boards, lower door frames during forward motion",
        "candidate_surface_type": "UNKNOWN (Interior Hallway Walls / Flooring)",
        "glass_presence_verification": "UNVERIFIED (Potential door glazing unconfirmed without surveyed CAD)",
        "recommended_action": "Assess gait-induced point cloud deskewing quality along hallway."
    },
    {
        "scene_id": "SCENE_04_WALL_PARTITIONS",
        "bag_name": "rosbag2_2026_10_01-16_49_33",
        "time_window_s": "50.0 - 65.0",
        "robot_motion_state": "Trot & Yaw Rotation (yaw_rate ~ 0.25 rad/s)",
        "sensor_streams_present": "L1_Raw, L1_Deskewed, Hesai, Camera_Color, Camera_Depth, LowState, SportModeState",
        "camera_view_description": "Sweeping floor and lower corridor partition boundaries during turning maneuver",
        "candidate_surface_type": "UNKNOWN (Corridor Partitions / Floor)",
        "glass_presence_verification": "UNVERIFIED (Elevated dual-return divergence observed in Hesai, surface unconfirmed)",
        "recommended_action": "Candidate region for M2 reflection feature testing; requires physical verification."
    },
    {
        "scene_id": "SCENE_05_TRAVERSAL_DECEL",
        "bag_name": "rosbag2_2026_10_01-16_49_33",
        "time_window_s": "70.0 - 80.0",
        "robot_motion_state": "Deceleration & Approach to Boundary",
        "sensor_streams_present": "L1_Raw, L1_Deskewed, Hesai, Camera_Color, Camera_Depth, LowState, SportModeState",
        "camera_view_description": "Floor and lower section of end-of-corridor wall / boundary structure",
        "candidate_surface_type": "UNKNOWN (End-Wall Structure / Floor)",
        "glass_presence_verification": "UNVERIFIED (Cannot confirm glass from camera; target recording required)",
        "recommended_action": "Inspect LiDAR multi-echo returns; request controlled glass target session."
    },
    {
        "scene_id": "SCENE_06_STAT_END",
        "bag_name": "rosbag2_2026_10_01-16_49_33",
        "time_window_s": "82.0 - 88.6",
        "robot_motion_state": "Stationary Standing",
        "sensor_streams_present": "L1_Raw, L1_Deskewed, Hesai, Camera_Color, Camera_Depth, LowState, SportModeState",
        "camera_view_description": "Stationary view of floor and proximate vertical structure base",
        "candidate_surface_type": "UNKNOWN (Vertical Structure Base / Floor)",
        "glass_presence_verification": "UNVERIFIED",
        "recommended_action": "Use for stationary extrinsic check and sensor noise floor analysis."
    }
]

for dest_path in [
    os.path.join(REPORT_DIR, "scene_inventory.csv"),
    os.path.join(HANDOFF_M4_DIR, "scene_inventory.csv")
]:
    with open(dest_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(scenes[0].keys()))
        writer.writeheader()
        writer.writerows(scenes)

print("Saved corrected scene_inventory.csv to reports and handoff.")

# -----------------------------------------------------------------------------
# 2. annotation_template.csv (Completely Blank Schema)
# -----------------------------------------------------------------------------
anno_cols = [
    "point_id",
    "scene_id",
    "sensor",               # hesai | l1_raw | l1_deskewed
    "scan_timestamp_s",
    "x_sensor",
    "y_sensor",
    "z_sensor",
    "range_m",
    "intensity",
    "ring_id",
    "return_idx",           # 1 (strongest/first) | 2 (last)
    "gt_surface_class",     # opaque_diffuse | opaque_specular | glass_front | glass_rear | transmitted | multipath_ghost | floor | ceiling | noise_unassigned
    "gt_confidence",        # 1 (speculative) to 5 (verified by physical tape / CAD survey)
    "annotator_id",
    "verification_method",  # physical_survey | camera_reprojection | cad_model | unverified
    "remarks"
]

for dest_path in [
    os.path.join(REPORT_DIR, "annotation_template.csv"),
    os.path.join(HANDOFF_M4_DIR, "annotation_template.csv")
]:
    with open(dest_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(anno_cols)
        # Template is intentionally blank - no fabricated example rows.

print("Saved clean, blank annotation_template.csv to reports and handoff.")

# -----------------------------------------------------------------------------
# 3. ground_truth_requirements.md
# -----------------------------------------------------------------------------
gt_req_content = """# ACSAR-E M4: Ground-Truth Protocol & Physical Survey Requirements

## 1. Scope & Research-Readiness Status
- **Milestone M4 Status:** **READY for Developing Ground-Truth Protocol; NOT READY for Classification Accuracy Measurements.**
- **Current Dataset Limitation:**
  In `rosbag2_2026_10_01-16_49_33` (and bags 5 and 6), the onboard RGB camera is angled downwards toward the floor and lower skirting boards. Visual identification of elevated transparent glass panels, doors, or windows is unverified from the camera stream alone.
- **Mandate:**
  No point in the benchmark dataset may be labeled as `glass_front`, `transmitted`, or `multipath_ghost` based solely on automated LiDAR heuristic filters (such as dual-return divergence or intensity). All ground-truth annotations require demonstrable physical verification.

---

## 2. Standardized Surface Taxonomy

Annotators must classify points into exactly one of the following standardized categories:

| Class Code | Class Name | Verification Criteria |
| :--- | :--- | :--- |
| `opaque_diffuse` | Opaque Diffuse Surface | Standard matte drywall, carpet, concrete pillars confirmed by architectural survey. |
| `opaque_specular`| Opaque Specular Surface| Glossy polished floors, brushed aluminum door mullions, whiteboard panels. |
| `glass_front`    | Glass Front Surface   | Confirmed physical front plane of transparent glass partition/door. |
| `glass_rear`     | Glass Rear Surface    | Secondary return from back plane of double-glazed / thick glass. |
| `transmitted`    | Transmitted Returns    | Points passing through glass to real objects behind it (verified by room layout). |
| `multipath_ghost`| Multipath / Virtual   | Ghost points positioned in free space or outside the building envelope via reflection. |
| `floor`          | Traversable Ground     | Physical floor plane traversed by quadruped. |
| `noise_unassigned`| Unassigned Noise     | Atmospheric particles, beam divergence edge dropouts, unverified points. |

---

## 3. Protocol for Controlled Ground-Truth Recordings

To enable quantitative classification benchmarking for M2 and fusion evaluation for M3, Team Member M4 must execute a controlled test session adhering to this protocol:

1. **Dimensional Architectural Survey:**
   - Accurate 2D/3D tape or laser distance measurements of corridor width, wall positions, and door locations.
   - Exact 3D boundaries (corner coordinates and height extents) of all glass panels.
2. **Optical Survey Targets (Fiducials):**
   - High-contrast retroreflective or ArUco/checkerboard targets affixed at known heights on opaque walls and at glass boundaries.
3. **Camera Tilt Angle Calibration:**
   - Adjust the camera mounting pitch or record with an elevated forward-facing camera to capture full wall height and transparent partitions in visual imagery.
4. **Controlled Surface Encounters:**
   - Robot approaches transparent glass panel at measured frontal angles (0° normal, 15°, 30°, 45° oblique).
   - Robot executes stationary standing scans, slow walking, and trotting at known standoff distances (1.0m, 2.0m, 3.0m).

---

## 4. Annotation Template Schema

The blank annotation template is located at `acsar_e_m1/reports/annotation_template.csv`.
Every submitted ground-truth entry must record:
- Point spatial coordinates (`x_sensor`, `y_sensor`, `z_sensor`, `range_m`)
- Sensor channel (`sensor`, `ring_id`, `return_idx`)
- Ground-truth label (`gt_surface_class`)
- Confidence score (`gt_confidence`: 1 = speculative, 5 = verified with physical measurement)
- Verification method (`physical_survey`, `camera_reprojection`, `cad_model`)
"""

for dest_path in [
    os.path.join(REPORT_DIR, "ground_truth_requirements.md"),
    os.path.join(HANDOFF_M4_DIR, "ground_truth_requirements.md")
]:
    with open(dest_path, "w") as f:
        f.write(gt_req_content)

print("Saved updated ground_truth_requirements.md to reports and handoff.")

