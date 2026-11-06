# ACSAR-E M1 — Hesai XT-16 Dual-Return & Timestamp Granularity Report (Corrected)

> [!IMPORTANT]
> **Corrected Analysis:** Dual-return divergence metrics recalculated rigorously across all 32,000 firing pairs per scan. Driver configuration verified directly from onboard YAML configuration (`hesai-config.yaml`).

---

## 1. Dual-Return Pairing Architecture & Verification

In the Hesai XT-16 point cloud stream (`/lidar_points`), each 64,000-point scan is structured as 2,000 blocks of 32 points. Within each block:
- **Indices `0..15`:** Return 1 (Strongest) for channels 0 through 15.
- **Indices `16..31`:** Return 2 (Last) for channels 0 through 15.
- **Return Pair Definition:** For any base block $b \in [0, 1999]$ and channel $c \in [0, 15]$, Return 1 is point index $32b + c$ and Return 2 is point index $32b + 16 + c$.
- **Total Laser Firing Pairs per Scan:** Exactly $64,000 / 2 = 32,000$ pairs.

### Driver Configuration Confirmation
From runtime Hesai driver config ([`hesai-config.yaml`](file:///D:/office-full-5jqqcxzu/hesai-config.yaml)):
```yaml
echo_mode_filter: 0          # 0: Output all dual returns (Strongest + Last)
transform_flag: true
roll: 0; pitch: 0; yaw: 1.57079632679   # Sensor mounted with +90 deg yaw
```
This confirms the hardware was configured in dual-return mode and publishing both strongest and last returns without filtering.

---

## 2. Rigorous Pair Categorization Across Operational Epochs

Every pair $(p_1, p_2)$ is classified into one of four mutually exclusive categories:
1. **Both-Zero Pairs:** Both $p_1$ and $p_2$ equal `(0, 0, 0)` (laser ray discharged into open space / out of range).
2. **One-Valid Pairs:** One return is valid and the other is `(0, 0, 0)`.
3. **Identical Valid Pairs:** Both returns are valid and have identical coordinates and intensity ($p_1 = p_2$).
4. **Differing Valid Pairs:** Both returns are valid, but coordinates or intensity diverge ($p_1 \neq p_2$).

### Measured Breakdown Across 32,000 Pairs

| Scan File | Operational Epoch | Total Pairs | Both-Zero Pairs (Non-returns) | One-Valid Pairs | Identical Valid Pairs ($p_1 = p_2$) | Differing Valid Pairs ($p_1 \neq p_2$) | Differing % (Total Pairs) | Differing % (Valid Pairs) |
|:---|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|
| `scan_0000_early.npz` | Early (Stationary) | 32,000 | 3,023 (9.45%) | 0 (0.00%) | 28,770 (89.91%) | **207** | **0.65%** | **0.71%** |
| `scan_0001_early.npz` | Early (Stationary) | 32,000 | 3,019 (9.43%) | 0 (0.00%) | 28,780 (89.94%) | **201** | **0.63%** | **0.69%** |
| `scan_0404_mid_steady.npz` | Mid (Walking) | 32,000 | 3,257 (10.18%) | 0 (0.00%) | 27,654 (86.42%) | **1,089** | **3.40%** | **3.79%** |
| `scan_0405_mid_steady.npz` | Mid (Walking) | 32,000 | 3,367 (10.52%) | 0 (0.00%) | 27,486 (85.89%) | **1,147** | **3.58%** | **4.01%** |
| `scan_0807_late_steady.npz` | Late (Walking) | 32,000 | 4,133 (12.92%) | 0 (0.00%) | 27,035 (84.48%) | **832** | **2.60%** | **2.99%** |
| `scan_0808_late_steady.npz` | Late (Walking) | 32,000 | 4,130 (12.91%) | 0 (0.00%) | 27,022 (84.44%) | **848** | **2.65%** | **3.04%** |

### Root Cause of Previous 10.1% Figure
The previous 10.1% divergent figure resulted from excluding zero-coordinate non-return pairs from the duplicate count while inadvertently leaving them in the total denominator $(1.0 - 28,770 / 32,000 \approx 10.1\%)$. Once non-returns are correctly isolated into the **Both-Zero** category (9.45%–12.92%), the true proportion of differing physical returns is confirmed at:
- **Stationary Startup:** **0.63%–0.65%**
- **Active Locomotion (Office Environment):** **3.40%–3.58%**
- **Late Locomotion:** **2.60%–2.65%**

> [!CAUTION]
> **Physical Interpretation Warning for M2:**  
> A diverging return pair ($p_1 \neq p_2$) is **NOT exclusive to glass surfaces**. Return divergence physically occurs whenever a laser beam's footprint is split across depth discontinuities, including:
> 1. Sharp obstacle edges (door jambs, chair legs, table borders).
> 2. Highly oblique incident angles.
> 3. Semi-transparent or retroreflective surfaces.
> M2 must use spatial surface continuity, intensity ratios, and geometric normal consistency—not simple return divergence alone—to identify glass reflections.

---

## 3. Timestamp Distribution & True Firing Granularity

Measurements across the 500 distinct timestamp groups per Hesai scan reveal that **timestamps are NOT uniformly spaced at 0.2 ms**:

```
[Packet N (128 pts)]                    [Packet N+1 (128 pts)]
|-- 1 us --|-- 2 us --|-- 3 us --| ... |~~~~ 1.06 ms ~~~~|-- 1 us --|-- 2 us --|
```

- **Intra-Packet Steps:** Consecutive timestamp groups within the same Ethernet packet burst advance by **$1\text{ to }3\ \mu\text{s}$** (firing sequence interval).
- **Inter-Packet Steps:** Jumps between packet bursts average **$1,065\text{ to }1,080\ \mu\text{s}$** (~1.07 ms), reflecting the sensor's UDP packet assembly interval.
- **Motion Deskewing Guidance:** Point deskewing must apply the native `float64` per-point `timestamp` values rather than approximating scan time with a linear synthetic ramp.
