# Unitree L1: Raw vs. Deskewed Investigation (Corrected)

**Date:** 2026-11-06  
**Status:** Verified via empirical per-message audit across all 1,234 messages in `rosbag2_2026_10_01-16_49_33`  

---

## 1. Verified Message Structure & Point Counts

### 1.1 Why Did Early Samples Show ~1,100 Points While Later Messages Show ~4,100 Points?
An audit across the entire bag reveals that the Unitree L1 LiDAR driver changed output point density during the recording:

| Recording Phase | Message Range | Approx Time Elapsed | Raw Points / Message | Notes |
|---|:---:|:---:|:---:|:---|
| **Startup / Transient** | Messages 0 – ~150 | First ~0 – 10 seconds | **1,028 – 1,200** | Initial sample scans (0000–0004) were extracted from this startup window. |
| **Steady State** | Messages 200 – 1,234 | Remaining ~78 seconds | **4,000 – 4,200** | Steady-state point rate once sensor motor/DSP reached target speed. |

> [!NOTE]
> The initial report cited ~4,100 points because it sampled steady-state messages, while the first five sample export files were extracted from the beginning of the bag (startup window at ~1,100 points). Both observations are verified and represent distinct operational states of the sensor.

---

## 2. Corrected Deskewed Point Composition: The 10,000 Zero Pad

The previous hypothesis that deskewed scans represent an accumulation of ~10 full raw scans was incorrect.

Detailed examination of the serialized byte payload in `/utlidar/cloud_deskewed` proves:

1. **Fixed Buffer Padding:**
   Every single deskewed message in the bag has a length of $10,000 + N_{\text{valid}}$ points:
   - **Exactly 10,000 points** are zero padding: $(x = 0.000, y = 0.000, z = 0.000, \text{intensity} = 0.000)$.
   - Only **400 – 614 points** per message are actual physical returns.
2. **Net Information Rate:**
   Rather than multiplying point count, the motion deskewer is actually publishing a downsampled/filtered sub-cloud (likely spatial voxel grid downsampling within FAST-LIO2) into the `odom` frame, packed inside a pre-allocated 10,000-point buffer.
3. **Field Stripping:**
   The deskewer discards `ring` and `time`.

---

## 3. L1 Non-Repetitive Scan Pattern & Ring Index Audit

Review of all 1,234 `/utlidar/cloud` messages confirms:
- **Every point has `ring = 1`** across all messages in the dataset.
- The Unitree L1 is an **omnidirectional non-repetitive scanning LiDAR** (using a rotating prism / Risley-like scanner), NOT a multi-beam vertical diode array like the Hesai XT-16 or Velodyne VLP-16.
- The driver populates `ring` with a dummy constant value (`1`).

### Processing Impact for M2 / M3:
- **Spatial Neighborhoods:** Graph or KD-tree neighborhood construction for Unitree L1 must **NOT** assume scanline/ring adjacency. Algorithms must use 3D Euclidean radius search or spherical projection $(r, \theta, \phi)$.
- **Temporal Organization:** The `time` field (offset 24, float32, range $0.000$ to $\approx 0.062\text{ s}$) represents intra-scan time and must be used to order consecutive laser pulses along the scanner's trajectory.
