#!/usr/bin/env python3
"""
Refined Spatial Extrinsic Calibration (Hesai to L1)
Correcting the Double-Applied Yaw Bug: Nominal Yaw is ~0°, Pitch is ~ -20.7°
Using dense accumulated stationary clouds (18,577 L1 points, 577,741 Hesai points).
"""

import os
import yaml
import numpy as np
import scipy.spatial
import scipy.optimize

data = np.load("/mnt/d/glass-research/acsar_e_m1/exports/accumulated_stationary_clouds.npz")
H = data['hesai']
L = data['l1']

# Downsample both clouds with 3cm voxels
def voxel_downsample(pts, voxel_size=0.03):
    coords = np.floor(pts / voxel_size).astype(np.int32)
    _, idx = np.unique(coords, axis=0, return_index=True)
    return pts[idx]

H_sub = voxel_downsample(H, 0.03)
L_sub = voxel_downsample(L, 0.03)

print(f"Voxel-downsampled points: Hesai={len(H_sub)}, L1={len(L_sub)}")

def rpy_to_R(roll, pitch, yaw):
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return Rz @ Ry @ Rx

def R_to_rpy(R):
    pitch = np.arcsin(-np.clip(R[2, 0], -1.0, 1.0))
    if np.abs(np.cos(pitch)) > 1e-6:
        roll = np.arctan2(R[2, 1], R[2, 2])
        yaw = np.arctan2(R[1, 0], R[0, 0])
    else:
        roll = np.arctan2(-R[1, 2], R[1, 1])
        yaw = 0.0
    return roll, pitch, yaw

# Test nominal forward pitch hypothesis:
# L1 is mounted at chin tilted down by ~20.7° (so L1 frame Z is tilted forward/down)
# Hesai is mounted level on top deck (Z is up, X is forward)
# Therefore, R from Hesai to L1 has pitch ~ -20.7° or +20.7°, yaw ~ 0°
# Let's test pitch sweeps around [-30° to +30°] and yaw around [-15° to +15°]

tree_L = scipy.spatial.cKDTree(L_sub)

print("\n--- Testing Forward-Facing Pitch Sweeps (Yaw ~ 0°) ---")
best_p_inliers = 0
best_init = None

for pitch_deg in np.arange(-30.0, 31.0, 5.0):
    for yaw_deg in [-10.0, 0.0, 10.0]:
        for roll_deg in [-5.0, 0.0, 5.0]:
            R_test = rpy_to_R(np.deg2rad(roll_deg), np.deg2rad(pitch_deg), np.deg2rad(yaw_deg))
            # Test physical translation guesses: x in [-0.35, -0.15], y in [-0.05, 0.05], z in [0.05, 0.25]
            for tx in [-0.30, -0.20, -0.10, 0.0]:
                for tz in [0.05, 0.15, 0.25]:
                    t_test = np.array([tx, 0.0, tz])
                    H_t = (R_test @ H_sub.T).T + t_test
                    dists, _ = tree_L.query(H_t, distance_upper_bound=0.10)
                    inliers = np.count_nonzero(dists < 0.10)
                    if inliers > best_p_inliers:
                        best_p_inliers = inliers
                        best_init = (roll_deg, pitch_deg, yaw_deg, tx, tz)

print(f"Best initial guess from sweep: Inliers={best_p_inliers}/{len(H_sub)} ({100*best_p_inliers/len(H_sub):.1f}%)")
print(f"  Params: Roll={best_init[0]}°, Pitch={best_init[1]}°, Yaw={best_init[2]}°, tx={best_init[3]}m, tz={best_init[4]}m")

# Now run refined ICP starting from this best forward hypothesis
R_curr = rpy_to_R(np.deg2rad(best_init[0]), np.deg2rad(best_init[1]), np.deg2rad(best_init[2]))
t_curr = np.array([best_init[3], 0.0, best_init[4]])

dist_thresh = 0.12
print("\nRunning ICP refinement...")
for it in range(30):
    H_t = (R_curr @ H_sub.T).T + t_curr
    dists, indices = tree_L.query(H_t, distance_upper_bound=dist_thresh)
    valid = dists < dist_thresh
    inlier_cnt = np.count_nonzero(valid)
    
    if inlier_cnt < 200:
        print(f"Early break at iter {it}: inliers={inlier_cnt}")
        break
        
    src_valid = H_sub[valid]
    tgt_valid = L_sub[indices[valid]]
    
    c_src = src_valid.mean(axis=0)
    c_tgt = tgt_valid.mean(axis=0)
    
    H_mat = (src_valid - c_src).T @ (tgt_valid - c_tgt)
    U, S, Vt = np.linalg.svd(H_mat)
    R_opt = Vt.T @ U.T
    if np.linalg.det(R_opt) < 0:
        Vt[2, :] *= -1
        R_opt = Vt.T @ U.T
        
    t_opt = c_tgt - R_opt @ c_src
    
    R_curr = R_opt
    t_curr = t_opt
    rmse = np.sqrt(np.mean(dists[valid] ** 2))
    
    if it > 5:
        dist_thresh = max(0.04, dist_thresh * 0.90)
        
    if it % 5 == 0 or it == 29:
        r_d, p_d, y_d = np.rad2deg(R_to_rpy(R_curr))
        print(f"  Iter {it:2d}: Inliers={inlier_cnt:5d} ({100*inlier_cnt/len(H_sub):.1f}%) | "
              f"RMSE={rmse*100:5.2f} cm | RPY=({r_d:5.2f}, {p_d:5.2f}, {y_d:5.2f})° | "
              f"t=[{t_curr[0]:.4f}, {t_curr[1]:.4f}, {t_curr[2]:.4f}] m")

# Final Validation on un-downsampled L1 cloud
tree_L_full = scipy.spatial.cKDTree(L)
H_t_full = (R_curr @ H_sub.T).T + t_curr
dists_final, _ = tree_L_full.query(H_t_full, distance_upper_bound=0.10)
final_inliers = dists_final < 0.10
final_rmse = np.sqrt(np.mean(dists_final[final_inliers] ** 2))
final_inlier_pct = 100 * np.count_nonzero(final_inliers) / len(H_sub)

print("\n" + "=" * 80)
print("FINAL REFINED CALIBRATION RESULT")
print("=" * 80)
print(f"Validation Inliers (< 10 cm): {np.count_nonzero(final_inliers)} / {len(H_sub)} ({final_inlier_pct:.1f}%)")
print(f"Validation RMSE: {final_rmse*100:.2f} cm")
r_d, p_d, y_d = np.rad2deg(R_to_rpy(R_curr))
print(f"Final Extrinsics: RPY=({r_d:.2f}°, {p_d:.2f}°, {y_d:.2f}°) | t=[{t_curr[0]:.4f}, {t_curr[1]:.4f}, {t_curr[2]:.4f}] m")

