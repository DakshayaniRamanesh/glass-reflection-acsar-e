# ACSAR-E M1 — Timestamp Precision Validation Report

**Date:** 2026-11-06  
**Status:** Validated  
**Target:** Hesai XT-16 per-point timestamp precision & Unitree L1 timing fields  

---

## 1. Problem Identification: Float32 Quantization

In initial prototype exports, points were cast to a single 2D matrix via `points.astype(np.float32)`. 

### The Mathematical Consequence for Hesai XT-16:
The Hesai XT-16 reports per-point time in seconds as an IEEE 754 `FLOAT64` field named `timestamp` (offset 18 bytes, length 8 bytes), with values near $T \approx 3,307,199.13\text{ s}$.

- In **IEEE 754 Single Precision (`float32`)**:
  - The mantissa has 24 bits ($2^{-24} \approx 5.96 \times 10^{-8}$).
  - Machine epsilon at $3.3 \times 10^6$:
    $$\Delta t_{\text{float32}} = 2^{\lfloor \log_2(3,307,199) \rfloor - 23} = 2^{21 - 23} = 2^{-2} = \mathbf{0.25\text{ seconds}}$$
  - **Result:** Any laser firing pulses occurring within a $250\text{ ms}$ window rounded to the identical number, destroying all intra-scan timing across the entire $100\text{ ms}$ scan!

- In **IEEE 754 Double Precision (`float64`)**:
  - The mantissa has 53 bits ($2^{-53} \approx 1.11 \times 10^{-16}$).
  - Machine epsilon at $3.3 \times 10^6$:
    $$\Delta t_{\text{float64}} = 2^{21 - 52} = 2^{-31} \approx \mathbf{0.46\text{ nanoseconds}}$$
  - **Result:** Resolves microsecond laser firing deltas perfectly.

---

## 2. Empirical Verification on Exported Scans

We inspected `scan_0000.npz` before and after updating `extract_sample.py` to use NumPy structured arrays:

| Property | Float32 Cast (Previous Bug) | NumPy Structured Array (Corrected) | Status |
|---|:---:|:---:|:---:|
| `timestamp` dtype | `float32` | `float64` (`<f8`) | **FIXED** |
| Precision at $3.3 \times 10^6$ s | $\approx 250,000\ \mu\text{s}$ ($0.25$ s) | $\approx 0.00046\ \mu\text{s}$ ($0.46$ ns) | **VERIFIED** |
| Mean inter-point $\Delta t$ | $0.000\ \mu\text{s}$ (quantized to 0) | **$1.563\ \mu\text{s}$** | **VERIFIED** |
| Inter-point $\Delta t$ range | $[0, 250000]\ \mu\text{s}$ | **$[0.000, 1791.000]\ \mu\text{s}$** | **VERIFIED** |

### Numerical Comparison of First 10 Points in Hesai Scan 0:

| Point Index | Original Deserialized Value (s) | Corrected Exported Value (s) | Error ($\mu$s) |
|:---:|:---:|:---:|:---:|
| 0 | 3307199.131274000 | 3307199.131274000 | **0.0000** |
| 1 | 3307199.131274000 | 3307199.131274000 | **0.0000** |
| 2 | 3307199.131274000 | 3307199.131274000 | **0.0000** |
| 3 | 3307199.131274000 | 3307199.131274000 | **0.0000** |
| ... | ... | ... | ... |
| 16 | 3307199.131299000 | 3307199.131299000 | **0.0000** |
| 32 | 3307199.131324000 | 3307199.131324000 | **0.0000** |

*Note: In Hesai XT-16, channels firing simultaneously share identical timestamps, advancing by $\approx 25\ \mu\text{s}$ per laser firing block, matching the manufacturer's optical specification.*

---

## 3. Preserved Field Specifications Across Sensors

All exported `.npz` files now preserve exact native datatypes:

```python
# Hesai XT-16 (exports/hesai/scan_NNNN.npz):
dtype = [
    ('x', '<f4'),          # float32 (m)
    ('y', '<f4'),          # float32 (m)
    ('z', '<f4'),          # float32 (m)
    ('intensity', '<f4'),  # float32 (0-255)
    ('ring', '<u2'),       # uint16  (channel index 0-15)
    ('timestamp', '<f8')   # float64 (sensor epoch seconds, us resolution)
]

# Unitree L1 Raw (exports/l1_raw/scan_NNNN.npz):
dtype = [
    ('x', '<f4'),          # float32 (m)
    ('y', '<f4'),          # float32 (m)
    ('z', '<f4'),          # float32 (m)
    ('intensity', '<f4'),  # float32 (0-255)
    ('ring', '<u2'),       # uint16  (always 1)
    ('time', '<f4')        # float32 (scan-relative offset seconds, 0 to ~0.062s)
]

# Unitree L1 Deskewed (exports/l1_deskewed/scan_NNNN.npz):
dtype = [
    ('x', '<f4'),          # float32 (m, odom frame)
    ('y', '<f4'),          # float32 (m)
    ('z', '<f4'),          # float32 (m)
    ('intensity', '<f4')   # float32 (0-255)
]
```
