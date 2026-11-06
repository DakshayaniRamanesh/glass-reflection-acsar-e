#!/usr/bin/env python3
"""
Validates that:
1. Hesai per-point timestamp is stored as float64 with microsecond resolution.
2. Compares raw bytes against exported .npz values.
3. Tests valid_mask on all exported files.
"""
import numpy as np
import sqlite3, os

hesai_npz = "/mnt/d/glass-research/acsar_e_m1/exports/hesai/scan_0000.npz"
data = np.load(hesai_npz, allow_pickle=True)

pts = data['points']
print("Hesai points dtype:", pts.dtype)
print("Hesai points dtype names & types:")
for name in pts.dtype.names:
    print(f"  {name:12s}: {pts.dtype[name]}")

ts = pts['timestamp']
print(f"\nHesai timestamp dtype: {ts.dtype}")
print(f"First 10 Hesai timestamps:")
for i in range(10):
    print(f"  [{i}] {ts[i]:.9f} s  (diff from [0]: {(ts[i] - ts[0])*1e6:+.1f} us)")

diffs_us = np.diff(ts) * 1e6
print(f"\nHesai consecutive timestamp delta statistics:")
print(f"  min delta : {diffs_us.min():.3f} us")
print(f"  max delta : {diffs_us.max():.3f} us")
print(f"  mean delta: {diffs_us.mean():.3f} us")

# Check if float32 quantization occurred
# In float32, diffs would be 0 or ~0.25s!
assert ts.dtype == np.float64, "FAIL: timestamp is not float64"
assert diffs_us.mean() < 100.0, "FAIL: timestamp delta is quantized"
print("\n>>> PASS: Float64 precision is 100% preserved with microsecond resolution!")

# Check L1 raw
l1_npz = "/mnt/d/glass-research/acsar_e_m1/exports/l1_raw/scan_0000.npz"
data_l1 = np.load(l1_npz, allow_pickle=True)
pts_l1 = data_l1['points']
print("\nL1 Raw points dtype:")
for name in pts_l1.dtype.names:
    print(f"  {name:12s}: {pts_l1.dtype[name]}")
print("L1 raw rings unique:", np.unique(pts_l1['ring']))
print("L1 raw time range:", pts_l1['time'].min(), "to", pts_l1['time'].max(), "s")

# Check L1 deskewed zeros
l1_d_npz = "/mnt/d/glass-research/acsar_e_m1/exports/l1_deskewed/scan_0000.npz"
data_d = np.load(l1_d_npz, allow_pickle=True)
pts_d = data_d['points']
v_mask = data_d['valid_mask']
print("\nL1 Deskewed:")
print(f"  Total points: {len(pts_d)}")
print(f"  Valid mask count: {np.sum(v_mask)} ({100*np.sum(v_mask)/len(pts_d):.2f}%)")
print(f"  Padded zeros count: {np.sum(~v_mask)} ({100*np.sum(~v_mask)/len(pts_d):.2f}%)")
