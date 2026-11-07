# ACSAR-E M1 -> M2 Handoff: Reflection & Glass Detection

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
