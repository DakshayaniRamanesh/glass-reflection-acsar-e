# ACSAR-E M1 — Hesai XT-16 Point Repetition & Timestamp Granularity Analysis

**Date:** 2026-11-06  
**Status:** Empirically verified across serialized ROS2 messages and exported scans  
**Target Dataset:** `rosbag2_2026_10_01-16_49_33` (topic `/lidar_points`)  

---

## 1. Observed Anomalies & Quantitative Audit

Detailed examination of Hesai point clouds across multiple scans revealed the following characteristics:

| Property | Value | Notes |
|---|:---:|:---|
| **Total Points per Scan** | **64,000** | $16\text{ rings} \times 4,000\text{ azimuth points}$ |
| **Unique Timestamps per Scan** | **500** | Only 500 discrete timestamp values across all 64,000 points |
| **Points per Timestamp** | **Exactly 128** | Constant across all 500 timestamp blocks ($128 \times 500 = 64,000$) |
| **Points per Ring per Timestamp** | **Exactly 8** | $128\text{ points} / 16\text{ rings} = 8$ |
| **Adjacent Same-Ring Coordinate Duplication** | **89.9% of pairs** | $28,770$ out of $32,000$ adjacent pairs have identical $x, y, z$ and intensity |
| **Non-Duplicate Pairs** | **10.1% of pairs** | Points at obstacle edges, glass boundaries, or retroreflective objects bifurcate |

---

## 2. Root Cause Analysis: Dual Return Mode

The observation that ~90% of adjacent same-ring points have identical coordinates and intensities is the hallmark of **LiDAR Dual Return Mode** (e.g., **Strongest + Last Return**):

```
                        Laser Pulse Emitted
                               │
                ┌──────────────┴──────────────┐
                ▼                             ▼
        Solid Target (Wall)          Edge / Glass / Foliage
                │                             │
    ┌───────────┴───────────┐         ┌───────┴───────┐
    ▼                       ▼         ▼               ▼
Return 1 (Strongest)  Return 2 (Last) Return 1      Return 2
[x, y, z, I]           [x, y, z, I]    [x1, y1, z1, I1] [x2, y2, z2, I2]
   └── IDENTICAL VALUES ───┘             └── DISTINCT VALUES ──┘
```

1. **Why Identical Returns Occur:**
   When a laser beam strikes a single opaque planar obstacle (e.g. concrete wall, floor, door), the echo contains only one optical peak. The sensor DSP assigns the identical range, azimuth, elevation, and intensity to both Return 1 (Strongest) and Return 2 (Last Return).
2. **Why Distinct Returns Occur:**
   When a laser beam strikes a partial obstacle, an object edge, or a **glass pane**, part of the pulse energy is reflected by the glass surface while the remaining pulse passes through to an obstacle behind the glass. In these situations, Return 1 and Return 2 differ in range and intensity.
3. **Driver Assembly:**
   The Hesai ROS2 driver packages both returns in sequence within the same packet firing block, assigning both returns the identical firing timestamp because they originate from the same physical laser emission.

---

## 3. Timestamp Granularity & Motion Compensation Implications

### 3.1 True Timestamp Resolution
- The timestamp field advances in **500 discrete steps** per $100\text{ ms}$ rotation (mean step $\Delta t \approx 0.200\text{ ms}$).
- Intra-block laser firings within each $0.2\text{ ms}$ window share the identical timestamp.
- **Hardware Meaning:** The Hesai driver assigns timestamps at the **packet / azimuth firing block level**, rather than interpolating sub-microsecond firing times for individual channels within the block.

### 3.2 Impact on Motion Compensation (Deskewing)
- Standard continuous deskewing algorithms (such as in FAST-LIO2) assume that every point has a unique, strictly monotonic timestamp along the scan trajectory.
- With 128 points sharing the identical timestamp:
  - Motion compensation can deskew the scan in $500$ discrete pose steps across the $100\text{ ms}$ sweep.
  - At typical Go2 walking velocities ($v \approx 0.5\text{ m/s}$), the robot moves only $\Delta d = 0.5\text{ m/s} \times 0.0002\text{ s} = \mathbf{0.1\text{ mm}}$ per timestamp step.
  - At typical turning rates ($\omega \approx 30^\circ/\text{s}$), the robot rotates only $\Delta \theta = 30^\circ/\text{s} \times 0.0002\text{ s} = \mathbf{0.006^\circ}$ per timestamp step.
- **Conclusion:** A $0.2\text{ ms}$ block timestamp granularity introduces negligible uncompensated motion error for quadruped locomotion and is more than sufficient for high-accuracy deskewing.

---

## 4. Value for ACSAR-E Research (Module M2)

> [!TIP]
> **Do NOT remove duplicate points!**
> In reflection detection on quadruped robots, dual return divergence is a primary physical signature:
> - On regular surfaces: $\text{Return}_1 \equiv \text{Return}_2$.
> - On glass panes: $\text{Return}_1 \ne \text{Return}_2$ due to partial transmission and reflection.
> 
> The presence of dual returns in the Hesai stream provides our research team with a powerful multi-echo feature for glass identification that single-return LiDARs cannot provide.
