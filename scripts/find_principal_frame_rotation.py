#!/usr/bin/env python3
"""
Test all 24 3D principal rotation matrices to identify the exact frame convention
between Hesai and L1 accumulated clouds.
"""

import numpy as np
import scipy.spatial

data = np.load("/mnt/d/glass-research/acsar_e_m1/exports/accumulated_stationary_clouds.npz")
H = data['hesai']
L = data['l1']

# Subsample for fast permutation search
rng = np.random.default_rng(42)
H_sub = H[rng.choice(len(H), min(10000, len(H)), replace=False)]
L_sub = L[rng.choice(len(L), min(5000, len(L)), replace=False)]

# Build KDTree on L_sub
tree_L = scipy.spatial.cKDTree(L_sub)

# Generate all 24 orthogonal 3x3 rotation matrices with det = +1
rot_matrices = []
for p in [[0, 1, 2], [0, 2, 1], [1, 0, 2], [1, 2, 0], [2, 0, 1], [2, 1, 0]]:
    for s0 in [1, -1]:
        for s1 in [1, -1]:
            for s2 in [1, -1]:
                R = np.zeros((3, 3))
                R[0, p[0]] = s0
                R[1, p[1]] = s1
                R[2, p[2]] = s2
                if np.isclose(np.linalg.det(R), 1.0):
                    rot_matrices.append(R)

print(f"Testing {len(rot_matrices)} principal orthogonal rotations...")

results = []
for i, R in enumerate(rot_matrices):
    # Try with physical translation offset: Hesai is top-deck (+Z ~ 0.1-0.2m, -X ~ 0.1-0.2m) relative to chin L1
    # First center centroids
    H_rot = (R @ H_sub.T).T
    # Estimate translation by difference of median/centroids
    t_guess = np.median(L_sub, axis=0) - np.median(H_rot, axis=0)
    H_cand = H_rot + t_guess
    
    dists, _ = tree_L.query(H_cand, distance_upper_bound=0.20)
    inliers = np.count_nonzero(dists < 0.20)
    mean_dist = np.mean(dists[dists < 0.20]) if inliers > 0 else 999.0
    results.append((inliers, mean_dist, i, R, t_guess))

results.sort(key=lambda x: x[0], reverse=True)

print("\nTop 5 Best Orthogonal Rotations:")
for rank, (inliers, mean_dist, idx, R, t_guess) in enumerate(results[:5]):
    print(f"\n[Rank {rank+1}] Inliers (<20cm): {inliers}/{len(H_sub)} ({100*inliers/len(H_sub):.1f}%) | Mean Dist: {mean_dist*100:.2f} cm")
    print(f"  R matrix:\n{R}")
    print(f"  Translation guess: {t_guess}")

