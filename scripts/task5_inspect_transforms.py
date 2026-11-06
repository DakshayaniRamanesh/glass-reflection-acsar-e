#!/usr/bin/env python3
"""
Task 5: Inspect existing transforms & configuration files.
1. Decodes /tf_static messages from all bags where it is present.
2. Searches D: drive and workspace for FAST-LIO2 configs, yaml files, launch files.
3. Produces reports/transform_inventory.md.
"""
import sqlite3, struct, os, sys, glob
from pathlib import Path

OUT_DIR = "/mnt/d/glass-research/acsar_e_m1/reports"
BAG_BASE = "/mnt/d/glass-research"
os.makedirs(OUT_DIR, exist_ok=True)

class CDRReader:
    def __init__(self, data):
        self.data = data
        self.pos = 4  # skip encapsulation header
    def _align(self, n):
        r = self.pos % n
        if r: self.pos += (n - r)
    def u8(self):
        v = struct.unpack_from('B', self.data, self.pos)[0]; self.pos+=1; return v
    def i32(self):
        self._align(4); v = struct.unpack_from('<i', self.data, self.pos)[0]; self.pos+=4; return v
    def u32(self):
        self._align(4); v = struct.unpack_from('<I', self.data, self.pos)[0]; self.pos+=4; return v
    def f64(self):
        self._align(8); v = struct.unpack_from('<d', self.data, self.pos)[0]; self.pos+=8; return v
    def string(self):
        length = self.u32()
        if length == 0: return ""
        raw = self.data[self.pos:self.pos+length]; self.pos+=length
        return raw.rstrip(b'\x00').decode('utf-8','replace')

def parse_tf_message(raw):
    """Parses tf2_msgs/msg/TFMessage from CDR raw bytes."""
    r = CDRReader(raw)
    n_transforms = r.u32()
    transforms = []
    for _ in range(n_transforms):
        # Header
        sec = r.i32()
        nanosec = r.u32()
        frame_id = r.string()
        child_frame_id = r.string()
        # Translation (x, y, z float64)
        tx = r.f64()
        ty = r.f64()
        tz = r.f64()
        # Rotation (x, y, z, w float64)
        qx = r.f64()
        qy = r.f64()
        qz = r.f64()
        qw = r.f64()
        transforms.append({
            "stamp_sec": sec, "stamp_nanosec": nanosec,
            "stamp_ns": sec * 1_000_000_000 + nanosec,
            "frame_id": frame_id,
            "child_frame_id": child_frame_id,
            "translation": (tx, ty, tz),
            "rotation": (qx, qy, qz, qw)
        })
    return transforms

print("=" * 70)
print("TASK 5: Inspect /tf_static from all available bags")
print("=" * 70)

tf_records = {}
bags_with_tf = [
    "rosbag2_2026_10_01-16_39_53",
    "rosbag2_2026_10_01-16_42_00",
    "rosbag2_2026_10_01-16_49_33",
    "rosbag2_2026_10_01-16_51_04",
    "rosbag2_2026_10_01-16_52_39",
]

for bag in bags_with_tf:
    db3 = next(Path(f"{BAG_BASE}/{bag}").glob("*.db3"), None)
    if not db3: continue
    conn = sqlite3.connect(f"file:{db3}?mode=ro", uri=True)
    cur = conn.cursor()
    cur.execute("SELECT id FROM topics WHERE name='/tf_static'")
    row = cur.fetchone()
    if not row:
        conn.close(); continue
    tid = row[0]
    cur.execute("SELECT timestamp, data FROM messages WHERE topic_id=?", (tid,))
    msgs = cur.fetchall()
    conn.close()

    print(f"\nBag: {bag}  (/tf_static messages: {len(msgs)})")
    bag_tfs = []
    for ts, raw in msgs:
        try:
            tfs = parse_tf_message(raw)
            bag_tfs.extend(tfs)
            for t in tfs:
                print(f"  Transform: '{t['frame_id']}' -> '{t['child_frame_id']}'")
                print(f"    Translation: x={t['translation'][0]:.6f}, y={t['translation'][1]:.6f}, z={t['translation'][2]:.6f}")
                print(f"    Rotation (xyzw): ({t['rotation'][0]:.6f}, {t['rotation'][1]:.6f}, {t['rotation'][2]:.6f}, {t['rotation'][3]:.6f})")
                print(f"    Stamp: {t['stamp_sec']}s + {t['stamp_nanosec']}ns")
        except Exception as e:
            print(f"  [Error decoding /tf_static]: {e}")
    tf_records[bag] = bag_tfs

# ── Search for configuration files across D: drive ───────────────────────────
print("\n" + "=" * 70)
print("Searching for FAST-LIO, Hesai, or Unitree configuration files...")
print("=" * 70)

search_dirs = [
    "/mnt/d/glass-research",
    "/mnt/d/go2_mapping",
    "/mnt/d/glass_detection_research",
    "/mnt/d/acsar-3d-mapping-research",
    "/mnt/d/projects",
]

config_matches = []
for sdir in search_dirs:
    if os.path.exists(sdir):
        print(f"Searching in: {sdir}")
        for ext in ["*.yaml", "*.yml", "*.launch", "*.launch.py", "*.xml", "*.urdf", "*.xacro"]:
            for p in Path(sdir).rglob(ext):
                p_str = str(p)
                p_lower = p_str.lower()
                if any(k in p_lower for k in ["fast_lio", "fastlio", "hesai", "l1", "utlidar", "lidar", "config", "extrinsic", "calib", "robot"]):
                    config_matches.append(p_str)
                    print(f"  Found: {p_str}")

# Inspect content of candidate configs
print("\nInspecting candidate config contents:")
for cfg in config_matches[:10]:
    print(f"\n--- {cfg} ---")
    try:
        with open(cfg, "r", errors="ignore") as f:
            lines = [line.rstrip() for line in f.readlines()[:40]]
            for l in lines:
                if any(k in l.lower() for k in ["extrinsic", "rot", "tra", "t_", "r_", "imu", "lidar", "body", "transform"]):
                    print(f"  {l}")
    except Exception as e:
        print(f"  Error reading {cfg}: {e}")

# ── Write reports/transform_inventory.md ────────────────────────────────────
md_path = f"{OUT_DIR}/transform_inventory.md"
with open(md_path, "w") as f:
    f.write("# Transform & Calibration Inventory Report\n\n")
    f.write("## 1. Verified `/tf_static` Transforms in Rosbags\n\n")
    if tf_records:
        for bag, tfs in tf_records.items():
            f.write(f"### Bag: `{bag}`\n\n")
            if not tfs:
                f.write("No transforms found in `/tf_static`.\n\n")
            for t in tfs:
                f.write(f"- **Parent frame (`frame_id`):** `{t['frame_id']}`\n")
                f.write(f"- **Child frame (`child_frame_id`):** `{t['child_frame_id']}`\n")
                f.write(f"- **Translation:** `[x={t['translation'][0]:.6f}, y={t['translation'][1]:.6f}, z={t['translation'][2]:.6f}]` (m)\n")
                f.write(f"- **Rotation (qx, qy, qz, qw):** `[{t['rotation'][0]:.6f}, {t['rotation'][1]:.6f}, {t['rotation'][2]:.6f}, {t['rotation'][3]:.6f}]`\n")
                f.write(f"- **Timestamp:** `{t['stamp_sec']}s + {t['stamp_nanosec']}ns`\n\n")
    else:
        f.write("No `/tf_static` messages found in any bag.\n\n")

    f.write("## 2. Frame Graph Analysis\n\n")
    f.write("Frames observed in PointCloud2 & Odometry topics:\n")
    f.write("- `/utlidar/cloud` (L1 raw): `utlidar_lidar`\n")
    f.write("- `/utlidar/cloud_deskewed` (L1 deskewed): `odom`\n")
    f.write("- `/lidar_points` (Hesai XT-16): `hesai_lidar`\n")
    f.write("- `/utlidar/robot_odom`: `odom` -> `utlidar_lidar` (or robot body)\n\n")

    f.write("## 3. Discovered Configuration Files & Extrinsics\n\n")
    if config_matches:
        for c in config_matches:
            f.write(f"- `{c}`\n")
    else:
        f.write("No FAST-LIO2 or sensor extrinsic configuration files found in searched directories.\n\n")

    f.write("## 4. Missing Calibration Inputs for Sensor Fusion\n\n")
    f.write("- **Unitree L1 -> IMU:** Needs FAST-LIO2 extrinsic matrix (R, T)\n")
    f.write("- **Hesai XT-16 -> Unitree L1:** Needs mechanical mount CAD transform or target-based calibration\n")
    f.write("- **Hesai XT-16 -> IMU:** Composition of Hesai->L1 and L1->IMU\n")
    f.write("- **Robot Base -> Sensors:** Transform from `base_link` / `trunk` to `utlidar_lidar` and `hesai_lidar`\n")

print(f"\nWritten: {md_path}")
print("Task 5 DONE.")
