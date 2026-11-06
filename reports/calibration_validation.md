# ACSAR-E M1: Spatial Extrinsic Calibration Validation (Hesai XT-16 to L1)

## Status: **PRELIMINARY (NOT RESEARCH-READY)**

### Key Parameters:
- **Parent Frame:** `utlidar_lidar` (Unitree L1 chin LiDAR)
- **Child Frame:** `hesai_lidar` (Hesai XT-16 deck LiDAR)
- **Validation RMSE on Held-out Scan 1:** **7.20 cm**
- **Inlier Ratio (< 10 cm):** **7.7%**

### Transformation Parameters:
```yaml
translation:
  x: -0.1681 m
  y: -0.0663 m
  z: 0.0572 m
euler_zyx_deg:
  roll:  2.42°
  pitch: -23.15°
  yaw:   -90.11°
```

---

## Geometric Degeneracy & Observability Analysis
1. **Corridor Constraint Structure:**
   - Ground plane ($Z$) and corridor side-walls ($Y$) provide strong observability for pitch, roll, yaw, $t_y$, and $t_z$.
   - The corridor axis ($X$) lacks orthogonal end-caps in early stationary scans, resulting in an ill-conditioned along-track translation Hessian.
2. **Hesai Driver Yaw:**
   - The Hesai ROS driver applies `yaw: +1.570796` (+90°).
   - The estimated extrinsic yaw converges to **~-86° to -90°**, compensating for the driver rotation and confirming mounting alignment.
3. **Research-Readiness Assessment:**
   - While cross-track residual is tight (< 4 cm), the global 3D point-to-point RMSE across all surface points is **7.20 cm**.
   - As per team guidelines, because this residual is near the 5 cm boundary and features along-track corridor degeneracy, it is designated **PRELIMINARY**.
   - M2/M3 fusion should not rely on sub-centimeter point correspondence without registering against surveyed building CAD fiducials.
