# ACSAR-E M4: Ground-Truth Protocol & Physical Survey Requirements

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
