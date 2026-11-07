# ACSAR-E M1: Spatial Extrinsic Calibration Validation (Hesai XT-16 to L1)

## Status: **PRELIMINARY (NOT RESEARCH-READY)**

### Investigation of Calibration Inconsistencies:
1. **Identification of Double-Applied Yaw Rotation:**
   - In `hesai-config.yaml`, the driver was configured with `transform_flag: true, yaw: 1.57079632679` (+90°).
   - This driver transform rotated Hesai point coordinates into the robot body frame ($X$ forward).
   - Earlier calibration attempts mistakenly assumed Hesai required a $-90^\circ$ yaw rotation, resulting in a **double-applied yaw error** that oriented Hesai points $90^\circ$ perpendicular to L1!
2. **Limited Overlapping Geometry & Inlier Ratio:**
   - Hesai XT-16 has a $360^\circ$ horizontal surround field-of-view out to 20+ meters.
   - Unitree L1 is a front-facing dome sensor focused on the forward hemisphere.
   - When evaluating across all Hesai points, ~80% of points fall outside L1's physical coverage.
3. **Refined Forward-Facing Registration:**
   - By accumulating stationary scans (18,577 L1 points and 12,833 downsampled Hesai points) and using the true nominal forward-facing mounting:
     $$\mathbf{t} = [-0.0968, -0.0415, +0.1358]\text{ m}, \quad \text{RPY} = [-9.41^\circ, -2.73^\circ, -11.07^\circ]$$
   - Physical plausibility: Hesai sits on the front top deck (~9.7 cm behind chin L1, ~4.1 cm offset, and ~13.6 cm above chin L1).
   - Inlier residual drops to **2.75 cm** (with validation RMSE of **5.92 cm**).
4. **Corridor Degeneracy:**
   - Because the recording was performed in an indoor corridor with smooth parallel walls, the along-track translation axis lacks orthogonal constraint planes.
   - Therefore, M3 is **NOT READY** for validated dual-LiDAR fusion until multi-surface target-based calibration is performed.
