#!/usr/bin/env python3
"""
Inspect geometric relationship between L1 raw (utlidar_lidar frame)
and L1 deskewed (odom frame) to recover the runtime extrinsic transform.
"""
import numpy as np

raw_d = np.load("/mnt/d/glass-research/acsar_e_m1/exports/l1_raw/scan_0000_early.npz")
desk_d = np.load("/mnt/d/glass-research/acsar_e_m1/exports/l1_deskewed/scan_0000_early.npz")

raw_pts = raw_d['points']
desk_pts = desk_d['points']
desk_mask = desk_d['valid_mask']

print("L1 Raw (utlidar_lidar):", len(raw_pts), "pts")
print("  Bounding box X:", [raw_pts['x'].min(), raw_pts['x'].max()])
print("  Bounding box Y:", [raw_pts['y'].min(), raw_pts['y'].max()])
print("  Bounding box Z:", [raw_pts['z'].min(), raw_pts['z'].max()])

desk_valid = desk_pts[desk_mask]
print("\nL1 Deskewed (odom):", len(desk_valid), "valid pts")
print("  Bounding box X:", [desk_valid['x'].min(), desk_valid['x'].max()])
print("  Bounding box Y:", [desk_valid['y'].min(), desk_valid['y'].max()])
print("  Bounding box Z:", [desk_valid['z'].min(), desk_valid['z'].max()])

# Let's inspect centroid of raw vs deskewed
c_raw = np.array([raw_pts['x'].mean(), raw_pts['y'].mean(), raw_pts['z'].mean()])
c_desk = np.array([desk_valid['x'].mean(), desk_valid['y'].mean(), desk_valid['z'].mean()])
print("\nCentroid Raw     :", np.round(c_raw, 3))
print("Centroid Deskewed:", np.round(c_desk, 3))
print("Shift (Desk - Raw):", np.round(c_desk - c_raw, 3))

