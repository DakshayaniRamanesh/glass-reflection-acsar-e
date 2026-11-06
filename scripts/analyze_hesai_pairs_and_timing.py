#!/usr/bin/env python3
"""
ACSAR-E M1 — Corrected Hesai XT-16 Dual-Return & Timestamp Analysis
===================================================================
1. Analyzes point ordering and return pairing (Return 1 vs Return 2).
2. Categorizes all 32,000 return pairs into:
   - Identical valid pairs (Strongest == Last)
   - Differing valid pairs (Strongest != Last, e.g. glass, edges)
   - Both-invalid pairs (both x=y=z=0)
   - One-valid / one-invalid pairs
3. Analyzes exact timestamp distribution across the 500 timestamp groups.
"""
import glob
import os
import numpy as np

EXPORTS_DIR = "/mnt/d/glass-research/acsar_e_m1/exports/hesai"
files = sorted(glob.glob(f"{EXPORTS_DIR}/*.npz"))

print("=" * 80)
print("HESAI XT-16 DUAL-RETURN & TIMING RIGOROUS AUDIT")
print("=" * 80)

# Check first file structure
f0 = files[0]
d0 = np.load(f0)
pts0 = d0['points']
rings0 = pts0['ring']
ts0 = pts0['timestamp']
x0, y0, z0, i0 = pts0['x'], pts0['y'], pts0['z'], pts0['intensity']

print(f"File: {os.path.basename(f0)}")
print(f"First 16 rings: {rings0[:16].tolist()}")
print(f"Rings 16..31  : {rings0[16:32].tolist()}")
print(f"Rings 32..47  : {rings0[32:48].tolist()}")

# Check timestamp pattern
u_ts0, counts0 = np.unique(ts0, return_counts=True)
print(f"\nUnique timestamps count: {len(u_ts0)}")
print(f"Points per timestamp: min={counts0.min()}, max={counts0.max()}, unique counts={np.unique(counts0)}")
print(f"First 10 timestamp values (relative to t0 in ms):")
for idx in range(min(10, len(u_ts0))):
    dt_ms = (u_ts0[idx] - u_ts0[0]) * 1000.0
    print(f"  Group {idx:3d}: {u_ts0[idx]:.9f} s (+{dt_ms:8.4f} ms) | pts={counts0[idx]}")

# Delta between consecutive timestamp groups
diffs_us = np.diff(u_ts0) * 1e6
print(f"\nTimestamp intervals between consecutive groups (microseconds):")
print(f"  Min: {diffs_us.min():.3f} us")
print(f"  Max: {diffs_us.max():.3f} us")
print(f"  Mean: {diffs_us.mean():.3f} us")
print(f"  Median: {np.median(diffs_us):.3f} us")
print(f"  Unique interval values (rounded to 0.1 us):")
u_diffs, d_counts = np.unique(np.round(diffs_us, 1), return_counts=True)
for ud, uc in zip(u_diffs[:10], d_counts[:10]):
    print(f"    {ud:8.1f} us : count {uc}")
if len(u_diffs) > 10:
    print(f"    ... and {len(u_diffs)-10} more unique intervals")

# Now let's inspect pair pairing structure:
# In Hesai XT-16 Dual Return:
# How are the dual returns paired?
# Let's test two candidate pairings:
# Candidate A: Consecutive points along channel (i, i+1) where ring[i] == ring[i+1]?
# Wait! Look at rings0[:32]:
# If rings0 is [0, 1, 2, ... 15, 0, 1, 2, ... 15], then channel c return 1 is at index k, and return 2 is at index k+16!
# Or is rings0 [0, 0, 1, 1, 2, 2, ...]? Let's check!
print("\nChecking ring layout:")
if rings0[0] == rings0[1]:
    print("Ring layout: Interleaved pairs [c, c, c+1, c+1, ...]")
    pair_stride = 1
elif rings0[0] == rings0[16]:
    print("Ring layout: Block pairs [c0..c15, c0..c15] -> Return 1 at [0..15], Return 2 at [16..31]")
    pair_stride = 16
else:
    print("Ring layout is different:")
    print("ring[0..20]:", rings0[:20])

EOF

