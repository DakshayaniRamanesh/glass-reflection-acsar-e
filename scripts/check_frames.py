#!/usr/bin/env python3
"""
Investigate coordinate frame conventions and orientations of Hesai and L1.
"""

import os
import numpy as np

hesai_scan0 = np.load("/mnt/d/glass-research/acsar_e_m1/exports/hesai/scan_0000.npz")
l1_scan0 = np.load("/mnt/d/glass-research/acsar_e_m1/exports/l1_raw/scan_0000.npz")

h_pts = hesai_scan0['points']
h_vmask = hesai_scan0['valid_mask']
hx = h_pts['x'][h_vmask]
hy = h_pts['y'][h_vmask]
hz = h_pts['z'][h_vmask]

l_pts = l1_scan0['points']
l_vmask = l1_scan0['valid_mask']
lx = l_pts['x'][l_vmask]
ly = l_pts['y'][l_vmask]
lz = l_pts['z'][l_vmask]

print("Hesai (valid points:", len(hx), "):")
print(f"  X range: [{hx.min():.3f}, {hx.max():.3f}], mean: {hx.mean():.3f}")
print(f"  Y range: [{hy.min():.3f}, {hy.max():.3f}], mean: {hy.mean():.3f}")
print(f"  Z range: [{hz.min():.3f}, {hz.max():.3f}], mean: {hz.mean():.3f}")

print("\nL1 Raw (valid points:", len(lx), "):")
print(f"  X range: [{lx.min():.3f}, {lx.max():.3f}], mean: {lx.mean():.3f}")
print(f"  Y range: [{ly.min():.3f}, {ly.max():.3f}], mean: {ly.mean():.3f}")
print(f"  Z range: [{lz.min():.3f}, {lz.max():.3f}], mean: {lz.mean():.3f}")

# Check which direction is the corridor (longest range)
print(f"\nHesai max range by axis: |X|={np.max(np.abs(hx)):.2f}, |Y|={np.max(np.abs(hy)):.2f}, |Z|={np.max(np.abs(hz)):.2f}")
print(f"L1 max range by axis:    |X|={np.max(np.abs(lx)):.2f}, |Y|={np.max(np.abs(ly)):.2f}, |Z|={np.max(np.abs(lz)):.2f}")

# Check ground plane candidates (where is the floor?)
# Robot height is ~0.3m (or crouching at 0.08m)
print(f"Hesai Z distribution: 10th percentile: {np.percentile(hz, 10):.3f}, 50th: {np.percentile(hz, 50):.3f}")
print(f"L1 Z distribution:    10th percentile: {np.percentile(lz, 10):.3f}, 50th: {np.percentile(lz, 50):.3f}")

