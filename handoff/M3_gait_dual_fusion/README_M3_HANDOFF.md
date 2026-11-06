# ACSAR-E M1 -> M3 Handoff Package: Gait Dynamics & Dual-LiDAR Fusion

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
