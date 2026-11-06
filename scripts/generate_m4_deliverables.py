#!/usr/bin/env python3
"""
Generate M4 Ground Truth Deliverables:
1. scene_inventory.csv
2. annotation_template.csv
3. ground_truth_requirements.md
"""

import os
import csv

REPORT_DIR = "/mnt/d/glass-research/acsar_e_m1/reports"
os.makedirs(REPORT_DIR, exist_ok=True)

# -----------------------------------------------------------------------------
# 1. scene_inventory.csv
# -----------------------------------------------------------------------------
scenes = [
    {
        "scene_id": "SCENE_01_STAT_START",
        "bag_name": "rosbag2_2026_10_01-16_49_33",
        "time_window_s": "0.0 - 5.0",
        "robot_state": "Stationary / Crouched",
        "sensor_streams": "L1_Raw, L1_Deskewed, Hesai, Camera_Color, Camera_Depth, LowState, SportModeState",
        "candidate_surfaces": "Opaque concrete floor, laboratory bench legs, distant corridor wall",
        "glass_presence": "None in foreground; reference opaque baseline",
        "notes": "Optimal baseline epoch for stationary sensor noise and zero-velocity drift quantification."
    },
    {
        "scene_id": "SCENE_02_TRANS_STAND",
        "bag_name": "rosbag2_2026_10_01-16_49_33",
        "time_window_s": "10.0 - 18.0",
        "robot_state": "Standing up & Initial Walk Transition",
        "sensor_streams": "L1_Raw, L1_Deskewed, Hesai, Camera_Color, Camera_Depth, LowState, SportModeState",
        "candidate_surfaces": "Floor, baseboard, door frame",
        "glass_presence": "Indirect reflections from polished floor",
        "notes": "Captures first gait initiation dynamics and high leg torques (up to 24 Nm)."
    },
    {
        "scene_id": "SCENE_03_CORRIDOR_WALK",
        "bag_name": "rosbag2_2026_10_01-16_49_33",
        "time_window_s": "30.0 - 45.0",
        "robot_state": "Continuous Trot Locomotion (v ~ 0.85 m/s)",
        "sensor_streams": "L1_Raw, L1_Deskewed, Hesai, Camera_Color, Camera_Depth, LowState, SportModeState",
        "candidate_surfaces": "Drywall corridor, glossy floor, metallic door frames",
        "glass_presence": "Office door glass side-panels at glancing incidence",
        "notes": "Primary benchmark for gait impact vibration vs LiDAR scan deskewing fidelity."
    },
    {
        "scene_id": "SCENE_04_WALL_PARTITIONS",
        "bag_name": "rosbag2_2026_10_01-16_49_33",
        "time_window_s": "50.0 - 65.0",
        "robot_state": "Trot & Turning (yaw_speed ~ 0.25 rad/s)",
        "sensor_streams": "L1_Raw, L1_Deskewed, Hesai, Camera_Color, Camera_Depth, LowState, SportModeState",
        "candidate_surfaces": "Matte painted drywall, glazed office partition frames",
        "glass_presence": "Glazed partitions with mixed transmitted and reflected returns",
        "notes": "High divergence observed in Hesai dual-return returns (3.40% - 3.58% differing pairs)."
    },
    {
        "scene_id": "SCENE_05_GLASS_APPROACH",
        "bag_name": "rosbag2_2026_10_01-16_49_33",
        "time_window_s": "70.0 - 80.0",
        "robot_state": "Deceleration near boundary",
        "sensor_streams": "L1_Raw, L1_Deskewed, Hesai, Camera_Color, Camera_Depth, LowState, SportModeState",
        "candidate_surfaces": "Transparent glass door / wall, interior room visible behind glass",
        "glass_presence": "Direct frontal and oblique glass surface; ghost returns beyond surface",
        "notes": "Critical M2/M4 glass classification validation zone."
    },
    {
        "scene_id": "SCENE_06_STAT_END",
        "bag_name": "rosbag2_2026_10_01-16_49_33",
        "time_window_s": "82.0 - 88.6",
        "robot_state": "Stationary Standing",
        "sensor_streams": "L1_Raw, L1_Deskewed, Hesai, Camera_Color, Camera_Depth, LowState, SportModeState",
        "candidate_surfaces": "Glass boundary and adjacent support pillars",
        "glass_presence": "Confirmed glass window/wall within 1.5m - 3.5m range",
        "notes": "Zero-motion scan pair evaluation for spatial extrinsic verification and dual-return analysis."
    }
]

scene_inv_path = os.path.join(REPORT_DIR, "scene_inventory.csv")
with open(scene_inv_path, "w", newline="") as f:
    writer = csv.DictWriter(f, fieldnames=list(scenes[0].keys()))
    writer.writeheader()
    writer.writerows(scenes)
print("Saved scene inventory to:", scene_inv_path)

# -----------------------------------------------------------------------------
# 2. annotation_template.csv
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
    "gt_confidence",        # 1 (low) to 5 (verified with physical tape survey)
    "annotator_id",
    "verification_method",  # physical_survey | camera_reprojection | dual_return_divergence | multi_frame_cad
    "remarks"
]

template_rows = [
    {
        "point_id": "PT_000001",
        "scene_id": "SCENE_05_GLASS_APPROACH",
        "sensor": "hesai",
        "scan_timestamp_s": "1790853035.120",
        "x_sensor": "2.415",
        "y_sensor": "-0.320",
        "z_sensor": "0.115",
        "range_m": "2.439",
        "intensity": "8.0",
        "ring_id": "7",
        "return_idx": "1",
        "gt_surface_class": "glass_front",
        "gt_confidence": "5",
        "annotator_id": "M4_LEAD",
        "verification_method": "physical_survey",
        "remarks": "Measured with Leica DISTO at 2.44m from robot front bumper."
    },
    {
        "point_id": "PT_000002",
        "scene_id": "SCENE_05_GLASS_APPROACH",
        "sensor": "hesai",
        "scan_timestamp_s": "1790853035.120",
        "x_sensor": "4.820",
        "y_sensor": "-0.640",
        "z_sensor": "0.230",
        "range_m": "4.868",
        "intensity": "3.0",
        "ring_id": "7",
        "return_idx": "2",
        "gt_surface_class": "transmitted",
        "gt_confidence": "4",
        "annotator_id": "M4_LEAD",
        "verification_method": "camera_reprojection",
        "remarks": "Point penetrates glass pane; corresponds to interior office chair backrest."
    },
    {
        "point_id": "PT_000003",
        "scene_id": "SCENE_05_GLASS_APPROACH",
        "sensor": "hesai",
        "scan_timestamp_s": "1790853035.120",
        "x_sensor": "3.850",
        "y_sensor": "1.200",
        "z_sensor": "-0.150",
        "range_m": "4.035",
        "intensity": "2.0",
        "ring_id": "5",
        "return_idx": "1",
        "gt_surface_class": "multipath_ghost",
        "gt_confidence": "4",
        "annotator_id": "M4_LEAD",
        "verification_method": "multi_frame_cad",
        "remarks": "Specular reflection via floor bounce into glass."
    }
]

anno_path = os.path.join(REPORT_DIR, "annotation_template.csv")
with open(anno_path, "w", newline="") as f:
    writer = csv.DictWriter(f, fieldnames=anno_cols)
    writer.writeheader()
    writer.writerows(template_rows)
print("Saved annotation template to:", anno_path)

# -----------------------------------------------------------------------------
# 3. ground_truth_requirements.md
# -----------------------------------------------------------------------------
gt_req_path = os.path.join(REPORT_DIR, "ground_truth_requirements.md")
with open(gt_req_path, "w") as f:
    f.write("""# ACSAR-E M4: Ground-Truth Protocol & Physical Survey Requirements

## 1. Scope & Objective
This document outlines the ground-truth collection and validation requirements for Team Member M4 (Independent Ground-Truth Labeling & Evaluation). 

To rigorously benchmark M2's glass detection and M3's gait-aware confidence fusion, labels **must not** be derived solely from automated detector outputs or raw sensor intensity heuristics. Ground truth must be grounded in physical environment measurements and independent camera validation.

---

## 2. Standardized Taxonomy of Surface Classes

Every point evaluated by M4 must be assigned exactly one primary label from this taxonomy:

| Class Code | Class Name | Definition & Physical Interpretation |
| :--- | :--- | :--- |
| `opaque_diffuse` | Opaque Diffuse Surface | Standard Lambertian walls, painted drywall, carpeted or matte flooring, cardboard, wood. |
| `opaque_specular`| Opaque Specular Surface| Glossy polished stone, brushed aluminum door frames, shiny whiteboards where multi-path or intensity dropouts occur without transmission. |
| `glass_front`    | Glass Front Surface   | True physical plane of the transparent glass window, door, or partition boundary facing the robot. |
| `glass_rear`     | Glass Rear Surface    | Secondary return from back surface of double-glazed or thick architectural glass panes. |
| `transmitted`    | Transmitted Returns    | Laser pulses that penetrated through the glass and reflected off real objects inside the room (e.g. desks, chairs, back walls). |
| `multipath_ghost`| Virtual / Ghost Surface| Mirror reflections where laser reflects off glass and bounces to another surface, placing virtual points in empty space behind the mirror plane. |
| `floor`          | Ground Plane           | True physical ground plane traversed by the quadruped. |
| `noise_unassigned`| Atmospheric / Sensor Noise | Dust, beam divergence edge artifacts, or isolated dropouts. |

---

## 3. Physical Survey Requirements for the Test Environment

To achieve research-grade label fidelity (Confidence Level 4–5), M4 must supply the following architectural survey data:

1. **Dimensional Floor Plan / CAD Layout:**
   - Accurate 2D/3D floor plan of the recording site (corridor and adjacent rooms).
   - Exact wall-to-wall corridor widths (measured at minimum 3 cross-sections).
2. **Glass Partition Geometries:**
   - 3D coordinates (or 2D segment + height extent) of all glass panels relative to the starting origin.
   - Glass panel thickness and construction (single-pane monolithic, double-glazed insulated unit, laminated).
   - Frame material and mullion dimensions (e.g., anodized aluminum, vinyl).
3. **Reference Survey Markers (Fiducials):**
   - High-contrast retroreflective or checkerboard survey targets placed at known surveyed 3D locations on opaque walls and at glass boundaries.
   - Laser rangefinder (e.g., Leica DISTO) distance measurements from fixed corridor milestones to the glass surfaces ($\pm 2\text{ mm}$ precision).
4. **Photographic & Lighting Documentation:**
   - High-resolution ambient photography of all glass partitions under the exact lighting conditions present during the recording.
   - Notation of internal blinds, decals, safety manifestation stickers, or tint films on the glass.

---

## 4. Multi-Modal Annotation & Verification Protocol

When creating labeled benchmark datasets, M4 should employ a multi-modal verification pipeline:

```
+---------------------------+       +---------------------------+
| 3D LiDAR Point Cloud      |       | 1280x720 RGB Camera Image |
| (Hesai XT-16 / L1 Raw)   |       | (camera_color_optical)    |
+-------------+-------------+       +-------------+-------------+
              |                                   |
              +-----------------+-----------------+
                                |
                                v
              +-----------------------------------+
              | Extrinsic Camera Reprojection     |
              | K_color * [R_cam|t_cam] * P_lidar |
              +-----------------+-----------------+
                                |
                                v
              +-----------------------------------+
              | CAD Boundary Plane Validation     |
              | Distance to surveyed glass plane  |
              +-----------------+-----------------+
                                |
                                v
              +-----------------------------------+
              | Return Divergence Check           |
              | Return 1 vs Return 2 separation   |
              +-----------------+-----------------+
                                |
                                v
              +-----------------------------------+
              | Ground-Truth Labeled Point        |
              | (annotation_template.csv)         |
              +-----------------------------------+
```

### Verification Criteria:
- **`glass_front`:** Point lies within $\pm 3\text{ cm}$ of the surveyed glass CAD plane AND projects onto the visible glass boundary in RGB camera imagery.
- **`transmitted`:** Point lies strictly behind the surveyed glass plane ($d > d_{\text{glass}} + \text{thickness}$) AND matches a real architectural object or furniture visible through the glass.
- **`multipath_ghost`:** Point lies behind the glass plane in free space or outside the building envelope, where physical survey proves no actual structure exists.

---

## 5. Candidate Benchmark Scenes

As defined in `scene_inventory.csv`, six standardized scenes from `rosbag2_2026_10_01-16_49_33` have been extracted:
1. `SCENE_01_STAT_START`: 0.0s – 5.0s (Stationary baseline, pure opaque returns).
2. `SCENE_02_TRANS_STAND`: 10.0s – 18.0s (Stand-up gait transition).
3. `SCENE_03_CORRIDOR_WALK`: 30.0s – 45.0s (Continuous trot, hallway walls).
4. `SCENE_04_WALL_PARTITIONS`: 50.0s – 65.0s (Turning motion, partition frames).
5. `SCENE_05_GLASS_APPROACH`: 70.0s – 80.0s (Frontal and oblique glass encounter).
6. `SCENE_06_STAT_END`: 82.0s – 88.6s (Stationary near glass, multi-return analysis).
""")
print("Saved ground truth requirements to:", gt_req_path)

