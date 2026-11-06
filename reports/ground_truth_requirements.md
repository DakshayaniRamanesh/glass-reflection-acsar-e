# ACSAR-E M4: Ground-Truth Protocol & Physical Survey Requirements

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
   - Laser rangefinder (e.g., Leica DISTO) distance measurements from fixed corridor milestones to the glass surfaces ($\pm 2	ext{ mm}$ precision).
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
- **`glass_front`:** Point lies within $\pm 3	ext{ cm}$ of the surveyed glass CAD plane AND projects onto the visible glass boundary in RGB camera imagery.
- **`transmitted`:** Point lies strictly behind the surveyed glass plane ($d > d_{	ext{glass}} + 	ext{thickness}$) AND matches a real architectural object or furniture visible through the glass.
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
