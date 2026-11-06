# ACSAR-E M1 -> M4 Handoff Package: Independent Ground-Truth Protocol

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
