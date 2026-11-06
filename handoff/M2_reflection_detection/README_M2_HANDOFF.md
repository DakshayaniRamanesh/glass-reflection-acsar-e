# ACSAR-E M1 -> M2 Handoff Package: Reflection & Glass Detection

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
