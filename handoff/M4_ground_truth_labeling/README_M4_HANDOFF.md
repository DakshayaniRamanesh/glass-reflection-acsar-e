# ACSAR-E M1 -> M4 Handoff: Ground-Truth Protocol & Evaluation

## Milestone Status:
- **Protocol Development:** **READY for developing the ground-truth protocol.**
- **Accuracy Benchmarking:** **NOT READY for classification accuracy measurements.**

## What is READY for M4:
1. **Scene Inventory & Imagery:**
   - `scene_inventory.csv`: 6 benchmark candidate scene epochs isolated. All surface classes explicitly designated **UNKNOWN / UNVERIFIED**.
   - 6 candidate RGB images in `candidate_scenes/`.
   - Camera intrinsics documented ($1280 \times 720$ RGB8, $f_x=911.99, f_y=911.48$).
2. **Standardized Annotation Framework:**
   - `annotation_template.csv`: Standardized tabular schema (point coordinates, sensor type, return index, surface class, confidence score, verification method). Intentionally blank with no fabricated measurements.
   - `ground_truth_requirements.md`: Comprehensive survey requirements protocol (corridor tape survey, CAD boundaries, optical fiducials).

## Why M4 is NOT READY for Accuracy Measurements:
1. **Downward Camera Angle:**
   - The onboard camera points downwards, predominantly imaging the floor and lower skirting boards. Transparent glass panels are not confirmed from the visual imagery.
2. **Lack of Physical Survey Data:**
   - No architectural CAD or surveyed glass coordinates exist for the recorded bag sessions.
   - M4 must execute a controlled recording session with known glass targets before quantitative classifier accuracy (precision/recall/ROC) can be evaluated.
