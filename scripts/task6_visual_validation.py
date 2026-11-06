#!/usr/bin/env python3
"""
Task 6 (Steady-State Updated): Visual validation using precision-preserved structured arrays,
explicit validity masks, and representative steady-state operational scans.
Filters zero returns (x=y=z=0) so scatter plots reflect physical spatial geometry,
while documenting zero counts and valid counts explicitly.
"""
import os, sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

EXPORTS_DIR = "/mnt/d/glass-research/acsar_e_m1/exports"
OUT_DIR = "/mnt/d/glass-research/acsar_e_m1/reports/visualizations"
os.makedirs(OUT_DIR, exist_ok=True)

SAMPLES = [
    ("Unitree L1 Raw (Mid Steady-State)",      f"{EXPORTS_DIR}/l1_raw/scan_0617_mid_steady.npz",      "viz_l1_raw.png",      "utlidar_lidar"),
    ("Unitree L1 Deskewed (Mid Steady-State)", f"{EXPORTS_DIR}/l1_deskewed/scan_0617_mid_steady.npz", "viz_l1_deskewed.png", "odom"),
    ("Hesai XT-16 (Mid Steady-State)",        f"{EXPORTS_DIR}/hesai/scan_0404_mid_steady.npz",       "viz_hesai.png",       "hesai_lidar"),
]

print("=" * 70)
print("TASK 6: Validated Visualizations on Steady-State Operating Scans")
print("=" * 70)

for title, fpath, out_png, frame in SAMPLES:
    if not os.path.exists(fpath):
        print(f"[MISSING] {fpath}"); continue

    data = np.load(fpath, allow_pickle=True)
    pts = data["points"]
    v_mask = data["valid_mask"]
    frame_id = data["frame_id"].item().decode('utf-8') if hasattr(data["frame_id"], "item") else str(data["frame_id"])
    hdr_ts = int(data["header_stamp_ns"])

    x = pts['x']
    y = pts['y']
    z = pts['z']
    inten = pts['intensity']

    total_pts = len(x)
    valid_pts = int(np.sum(v_mask))
    zero_pts = int(np.sum((x == 0.0) & (y == 0.0) & (z == 0.0)))
    valid_pct = 100.0 * valid_pts / total_pts if total_pts > 0 else 0
    zero_pct = 100.0 * zero_pts / total_pts if total_pts > 0 else 0

    # Filter for spatial plotting using valid_mask
    fx = x[v_mask]
    fy = y[v_mask]
    fz = z[v_mask]
    fi = inten[v_mask]

    print(f"\n--- {title} ---")
    print(f"  Total points : {total_pts:,}")
    print(f"  Exact zeros  : {zero_pts:,} ({zero_pct:.1f}%)")
    print(f"  Valid returns: {valid_pts:,} ({valid_pct:.1f}%)")
    print(f"  Valid BBox X : [{fx.min():.2f}, {fx.max():.2f}] m")
    print(f"  Valid BBox Y : [{fy.min():.2f}, {fy.max():.2f}] m")
    print(f"  Valid BBox Z : [{fz.min():.2f}, {fz.max():.2f}] m")
    print(f"  Valid Intensity: min={fi.min():.1f}, max={fi.max():.1f}, mean={fi.mean():.1f}")

    # Plot 3-panel figure
    fig = plt.figure(figsize=(18, 5))

    # Panel 1: BEV of Valid Returns
    ax1 = fig.add_subplot(1, 3, 1)
    step = max(1, len(fx) // 30000)
    sc1 = ax1.scatter(fx[::step], fy[::step], c=fi[::step], cmap='viridis', s=1.5, alpha=0.6)
    ax1.set_title(f"{title} — Bird's Eye View (Valid Returns)\n{valid_pts:,} valid / {total_pts:,} total ({zero_pts:,} zeros filtered)", fontsize=10)
    ax1.set_xlabel("X (m)")
    ax1.set_ylabel("Y (m)")
    ax1.grid(True, linestyle='--', alpha=0.5)
    ax1.axis('equal')
    cbar1 = plt.colorbar(sc1, ax=ax1, fraction=0.046, pad=0.04)
    cbar1.set_label("Intensity")

    # Panel 2: Side Elevation (X-Z)
    ax2 = fig.add_subplot(1, 3, 2)
    sc2 = ax2.scatter(fx[::step], fz[::step], c=fz[::step], cmap='plasma', s=1.5, alpha=0.6)
    ax2.set_title(f"{title} — Side Elevation (Valid Returns)\nZ range: [{fz.min():.2f}, {fz.max():.2f}] m", fontsize=10)
    ax2.set_xlabel("X (m)")
    ax2.set_ylabel("Z (m)")
    ax2.grid(True, linestyle='--', alpha=0.5)
    ax2.axis('equal')
    cbar2 = plt.colorbar(sc2, ax=ax2, fraction=0.046, pad=0.04)
    cbar2.set_label("Height Z (m)")

    # Panel 3: Intensity Distribution of Valid Returns
    ax3 = fig.add_subplot(1, 3, 3)
    ax3.hist(fi, bins=50, color='teal', edgecolor='black', alpha=0.7)
    ax3.set_title(f"{title} — Valid Returns Intensity\nMean={fi.mean():.1f}, Std={fi.std():.1f}", fontsize=10)
    ax3.set_xlabel("Intensity")
    ax3.set_ylabel("Count")
    ax3.grid(True, linestyle='--', alpha=0.5)

    plt.tight_layout()
    out_path = f"{OUT_DIR}/{out_png}"
    plt.savefig(out_path, dpi=160)
    plt.close()
    print(f"  Saved plot: {out_path}")

# Combined 3-way plot
print("\nGenerating updated viz_comparison_all3.png...")
fig, axes = plt.subplots(1, 3, figsize=(18, 6))

for idx, (title, fpath, _, frame) in enumerate(SAMPLES):
    data = np.load(fpath, allow_pickle=True)
    pts = data["points"]
    v_mask = data["valid_mask"]
    fx = pts['x'][v_mask]
    fy = pts['y'][v_mask]
    fi = pts['intensity'][v_mask]
    step = max(1, len(fx) // 25000)

    ax = axes[idx]
    sc = ax.scatter(fx[::step], fy[::step], c=fi[::step], cmap='viridis', s=1.5, alpha=0.7)
    zero_c = int(np.sum(~v_mask))
    ax.set_title(f"{title}\n({len(fx):,} valid returns, {zero_c:,} zeros excluded)\nFrame: {frame}", fontsize=11)
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.grid(True, linestyle='--', alpha=0.5)
    ax.axis('equal')
    cbar = plt.colorbar(sc, ax=ax, fraction=0.046, pad=0.04)
    cbar.set_label("Intensity")

plt.suptitle("ACSAR-E M1 — Valid LiDAR Physical Returns (Steady-State Operation)", fontsize=13, y=1.02)
plt.tight_layout()
comp_path = f"{OUT_DIR}/viz_comparison_all3.png"
plt.savefig(comp_path, dpi=180, bbox_inches='tight')
plt.close()
print(f"Saved: {comp_path}")

# Startup vs Steady-State L1 Raw Comparison
print("\nGenerating startup vs steady-state comparison: viz_l1_startup_vs_steady.png...")
fig, axes = plt.subplots(1, 2, figsize=(14, 6))

f_early = f"{EXPORTS_DIR}/l1_raw/scan_0000_early.npz"
f_steady = f"{EXPORTS_DIR}/l1_raw/scan_0617_mid_steady.npz"

if os.path.exists(f_early) and os.path.exists(f_steady):
    d_early = np.load(f_early, allow_pickle=True)
    d_steady = np.load(f_steady, allow_pickle=True)
    
    # Early
    pts_e = d_early['points']
    v_e = d_early['valid_mask']
    sc0 = axes[0].scatter(pts_e['x'][v_e], pts_e['y'][v_e], c=pts_e['intensity'][v_e], cmap='viridis', s=2, alpha=0.7)
    axes[0].set_title(f"L1 Raw Startup Phase (Msg #0000)\n{len(pts_e['x'][v_e]):,} points (Startup Buffer)", fontsize=11)
    axes[0].set_xlabel("X (m)")
    axes[0].set_ylabel("Y (m)")
    axes[0].axis('equal')
    axes[0].grid(True, linestyle='--', alpha=0.5)
    plt.colorbar(sc0, ax=axes[0], fraction=0.046, pad=0.04).set_label("Intensity")

    # Steady
    pts_s = d_steady['points']
    v_s = d_steady['valid_mask']
    sc1 = axes[1].scatter(pts_s['x'][v_s], pts_s['y'][v_s], c=pts_s['intensity'][v_s], cmap='viridis', s=2, alpha=0.7)
    axes[1].set_title(f"L1 Raw Steady-State Phase (Msg #0617)\n{len(pts_s['x'][v_s]):,} points (Full Field-of-View)", fontsize=11)
    axes[1].set_xlabel("X (m)")
    axes[1].set_ylabel("Y (m)")
    axes[1].axis('equal')
    axes[1].grid(True, linestyle='--', alpha=0.5)
    plt.colorbar(sc1, ax=axes[1], fraction=0.046, pad=0.04).set_label("Intensity")

    plt.suptitle("Unitree L1 Raw: Startup Sparse Buffer vs. Operational Steady-State Scan", fontsize=13, y=1.02)
    plt.tight_layout()
    startup_path = f"{OUT_DIR}/viz_l1_startup_vs_steady.png"
    plt.savefig(startup_path, dpi=180, bbox_inches='tight')
    plt.close()
    print(f"Saved: {startup_path}")

print("Task 6 Visualizations generated successfully.")
