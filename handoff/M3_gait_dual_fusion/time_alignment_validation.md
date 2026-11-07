# ACSAR-E M1: Temporal Synchronization and Cross-Clock Alignment Validation

## Status: **PRELIMINARY (UNVERIFIED AT SUB-SCAN SCALE)**

### Critical Reconciliation of Synchronization Claims:
1. **The 5,915 ms vs 5.9 ms Discrepancy:**
   - In earlier analysis, cross-correlation between `/utlidar/imu` $z$-gyro and `/sportmodestate` yaw rate reported a lag peak of **5,915 ms**.
   - **Root Cause:** The Unitree L1 chin IMU $z$-axis is inverted relative to the robot body coordinate system ($\omega_z^{\text{imu}} \approx -\omega_z^{\text{sport}}$). Running correlation without inverting sign forced the algorithm to find a weak secondary sidelobe at $+5.91\text{ s}$ ($r = 95.5$).
   - When inverted ($-\omega_z^{\text{imu}}$), the correlation peak is at **exactly 0.0 ms** ($r = 418.6$).
   - The summary mention of "5.9 ms" was an erroneous clerical typo that divided 5,915 ms by 1,000.
2. **Scope of the IMU Correlation:**
   - Both `/utlidar/imu` and `/sportmodestate` run on the internal Unitree robot hardware bus.
   - **This internal correlation cannot and does not establish Hesai-to-L1 synchronization.**
3. **Cross-Device Hesai Timing Evidence:**
   - Hesai XT-16 runs on its independent internal timer (~3,307,199 s from boot).
   - No hardware PPS (pulse-per-second) or PTP (IEEE 1588) synchronization was active during this recording.
   - The estimated offset ($\Delta t \approx 1,787,545,765.29\text{ s}$) is derived from host PC receipt timestamps, which are corrupted by network transport jitter ($\sigma \approx 120.8\text{ ms}$) and clock drift ($-1906.1\text{ ppm}$).
   - **Conclusion:** Cross-device Hesai-to-L1 synchronization is designated **PRELIMINARY**. Sub-scan or point-level dual-LiDAR fusion cannot be guaranteed without controlled PTP hardware synchronization.
