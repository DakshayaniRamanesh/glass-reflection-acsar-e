# ACSAR-E M1 to M2 Handoff Package & Technical Integration Guide

> [!IMPORTANT]
> **To Team Member M2 (Reflection Detection & Surface Classification):**  
> This package provides verified, multi-epoch representative LiDAR scans, gait kinematics, validity masks, and sensor metadata.  
> **Rule: Develop single-sensor reflection features (Hesai dual-return divergence, intensity profiles, incidence angles) independently. Do NOT perform dual-LiDAR spatial fusion until M3 gait-aware registration is calibrated.**

---

## 1. Directory Structure & File Inventory

```
acsar_e_m1/
├── exports/
│   ├── hesai/                           # 360° Hesai XT-16 scans (64,000 pts/scan)
│   │   ├── scan_0000_early.npz          # Startup / stationary baseline
│   │   ├── scan_0001_early.npz
│   │   ├── scan_0404_mid_steady.npz     # Active trotting in office (glass present)
│   │   ├── scan_0405_mid_steady.npz
│   │   ├── scan_0807_late_steady.npz    # Late walking locomotion
│   │   └── scan_0808_late_steady.npz
│   ├── l1_raw/                          # Unitree L1 Raw scans (utlidar_lidar frame)
│   │   ├── scan_0000_early.npz          # Startup (~1,180 valid pts)
│   │   ├── scan_0001_early.npz
│   │   ├── scan_0617_mid_steady.npz     # Steady-state (~4,140 valid pts)
│   │   ├── scan_0618_mid_steady.npz
│   │   ├── scan_1232_late_steady.npz
│   │   └── scan_1233_late_steady.npz
│   ├── l1_deskewed/                     # FAST-LIO2 deskewed scans (odom frame)
│   │   └── scan_*_steady.npz            # Contains 10,000 zero-padding points
│   └── gait/                            # Synchronized robot locomotion states
│       ├── sportmode_state.npz          # 54 Hz: gait_type, pos, vel, yaw_spd, foot_force, RPY
│       └── lowstate_leg_actuators.npz   # 45 Hz: 12 leg joint states (q, dq, tau) & foot_force
└── reports/
    ├── scan_index.csv                   # Master index of all exported scans
    ├── hesai_repetition_analysis.md     # Corrected dual-return analysis & timing
    ├── gait_alignment_report.md         # Kinematic decoding & phase identification
    ├── spatial_calibration_plan.md      # Mounting transform & planar validation
    └── visualizations/                  # 6 detailed diagnostic & visual plots
```

---

## 2. Point Cloud Data Loading & Array Schema

Each `.npz` file contains a NumPy structured array `points` preserving exact sensor precision, plus pre-extracted named arrays:

### Python Example: Loading Hesai Scans & Filtering
```python
import numpy as np

# Load Hesai scan
data = np.load("exports/hesai/scan_0404_mid_steady.npz")
pts = data['points']           # Structured array: x, y, z, intensity, ring, timestamp
valid_mask = data['valid_mask'] # True for physical LiDAR returns, False for (0,0,0) non-returns

# Extract valid physical returns
valid_pts = pts[valid_mask]
x = valid_pts['x']             # float32 (meters)
y = valid_pts['y']             # float32 (meters)
z = valid_pts['z']             # float32 (meters)
intensity = valid_pts['intensity'] # float32 (0-255 reflectivity)
ring = valid_pts['ring']       # uint16 (channel 0-15)
ts = valid_pts['timestamp']    # float64 (sub-microsecond precision hardware timestamp)
```

---

## 3. Hesai Dual-Return Mode: Reflection Feature Extraction

Hesai was operated in **Dual Return Mode (Strongest + Last)**. Each 64,000-point scan contains 32,000 pairs:
- **Return 1 (Strongest):** `pts[32*b + c]` (channels $c \in [0, 15]$)
- **Return 2 (Last):** `pts[32*b + 16 + c]` (channels $c \in [0, 15]$)

### Feature Extraction Code for M2:
```python
# Reshape points into 32,000 pairs: (2000 blocks, 2 returns, 16 channels)
n_blocks = len(pts) // 32
x_pairs = pts['x'].reshape(n_blocks, 2, 16)
y_pairs = pts['y'].reshape(n_blocks, 2, 16)
z_pairs = pts['z'].reshape(n_blocks, 2, 16)
i_pairs = pts['intensity'].reshape(n_blocks, 2, 16)

# Mask valid pairs (where neither return is 0,0,0)
is_valid_0 = ~((x_pairs[:, 0, :] == 0) & (y_pairs[:, 0, :] == 0) & (z_pairs[:, 0, :] == 0))
is_valid_1 = ~((x_pairs[:, 1, :] == 0) & (y_pairs[:, 1, :] == 0) & (z_pairs[:, 1, :] == 0))
valid_pairs = is_valid_0 & is_valid_1

# Distance divergence between Strongest and Last returns
dx = x_pairs[:, 0, :] - x_pairs[:, 1, :]
dy = y_pairs[:, 0, :] - y_pairs[:, 1, :]
dz = z_pairs[:, 0, :] - z_pairs[:, 1, :]
delta_dist = np.sqrt(dx**2 + dy**2 + dz**2)
delta_intensity = np.abs(i_pairs[:, 0, :] - i_pairs[:, 1, :])

# Pairs with diverging returns (candidate reflection or edge boundary)
divergent_mask = valid_pairs & ((delta_dist > 0.05) | (delta_intensity > 10.0))
```

> [!CAUTION]
> **Physical Interpretation Warning:**  
> Diverging returns occur at **both glass surfaces and physical object edges** (split-beam effect). M2 must cross-correlate divergence with local planar surface curvature and intensity consistency.

---

## 4. Robot Gait Data Integration (for M3 Preparation)

Gait state is available in `exports/gait/sportmode_state.npz`:
- `gait_type`: `0` (Stationary/Stance), `1` (Trot Walk, 2-beat diagonal gait).
- `foot_force`: 4-element array (FR, FL, RR, RL) in Newtons. Values $> 15\text{ N}$ indicate ground contact.
- `header_stamp_s`: Shares the **exact RTC clock domain** (`1,790,852,964 s`) as the Unitree L1 LiDAR.

