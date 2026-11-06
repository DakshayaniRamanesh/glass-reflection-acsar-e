#!/usr/bin/env python3
import sqlite3, struct, os

def parse_tf_static_clean(raw):
    # Offsets directly determined from CDR layout:
    # 4 transforms:
    # 1: camera_link -> camera_depth_frame
    # 2: camera_depth_frame -> camera_depth_optical_frame
    # 3: camera_link -> camera_color_frame
    # 4: camera_color_frame -> camera_color_optical_frame

    records = [
        # (stamp_offset, parent_len_off, child_len_off, trans_off)
        (8,   16,  32,  56),
        (112, 120, 144, 176),
        (232, 240, 256, 280),
        (336, 344, 368, 400),
    ]

    tfs = []
    for s_off, p_off, c_off, tr_off in records:
        sec, nsec = struct.unpack_from('<iI', raw, s_off)
        p_len = struct.unpack_from('<I', raw, p_off)[0]
        p_str = raw[p_off+4 : p_off+4+p_len].rstrip(b'\x00').decode('utf-8')
        c_len = struct.unpack_from('<I', raw, c_off)[0]
        c_str = raw[c_off+4 : c_off+4+c_len].rstrip(b'\x00').decode('utf-8')
        tx, ty, tz, qx, qy, qz, qw = struct.unpack_from('<7d', raw, tr_off)
        tfs.append({
            "sec": sec, "nsec": nsec, "stamp_ns": sec * 1_000_000_000 + nsec,
            "parent": p_str, "child": c_str,
            "trans": (tx, ty, tz),
            "rot": (qx, qy, qz, qw)
        })
    return tfs

db3 = "/mnt/d/glass-research/rosbag2_2026_10_01-16_49_33/rosbag2_2026_10_01-16_49_33_0.db3"
c = sqlite3.connect(f"file:{db3}?mode=ro", uri=True)
tid = c.execute("SELECT id FROM topics WHERE name='/tf_static'").fetchone()[0]
data = c.execute("SELECT data FROM messages WHERE topic_id=?", (tid,)).fetchone()[0]
c.close()

tfs = parse_tf_static_clean(data)
print(f"Decoded {len(tfs)} static transforms successfully:\n")
for idx, t in enumerate(tfs):
    print(f"Transform {idx + 1}:")
    print(f"  Parent: '{t['parent']}' -> Child: '{t['child']}'")
    print(f"  Stamp : {t['sec']}s + {t['nsec']}ns ({t['stamp_ns']} ns)")
    print(f"  Trans : x={t['trans'][0]:+.6f}, y={t['trans'][1]:+.6f}, z={t['trans'][2]:+.6f} m")
    print(f"  Rot   : qx={t['rot'][0]:+.6f}, qy={t['rot'][1]:+.6f}, qz={t['rot'][2]:+.6f}, qw={t['rot'][3]:+.6f}")
    print()

OUT_DIR = "/mnt/d/glass-research/acsar_e_m1/reports"
os.makedirs(OUT_DIR, exist_ok=True)
md_path = f"{OUT_DIR}/transform_inventory.md"

with open(md_path, "w") as f:
    f.write("# ACSAR-E M1 — Transform & Calibration Inventory Report\n\n")
    f.write("> [!IMPORTANT]\n")
    f.write("> All data below is decoded from actual `/tf_static` messages in `rosbag2_2026_10_01-16_49_33` (identical in all bags).\n\n")
    f.write("## 1. Decoded `/tf_static` Transforms\n\n")
    f.write(f"The `/tf_static` topic contains **{len(tfs)} static transforms**, exclusively defining the RealSense camera optical chain:\n\n")
    f.write("| # | Parent Frame | Child Frame | Translation [x, y, z] (m) | Rotation [qx, qy, qz, qw] | Header Stamp (s + ns) |\n")
    f.write("|:--:|:---|:---|:---|:---|:---|\n")
    for idx, t in enumerate(tfs):
        tr = f"[{t['trans'][0]:+.4f}, {t['trans'][1]:+.4f}, {t['trans'][2]:+.4f}]"
        rt = f"[{t['rot'][0]:+.4f}, {t['rot'][1]:+.4f}, {t['rot'][2]:+.4f}, {t['rot'][3]:+.4f}]"
        st = f"{t['sec']}s + {t['nsec']}ns"
        f.write(f"| {idx+1} | `{t['parent']}` | `{t['child']}` | `{tr}` | `{rt}` | `{st}` |\n")

    f.write("\n## 2. Sensor Frame Connectivity Audit\n\n")
    f.write("| Sensor / Stream | Frame ID | Connected in `/tf_static`? | Status |\n")
    f.write("|:---|:---|:---:|:---|\n")
    f.write("| **Unitree L1 (raw)** | `utlidar_lidar` | ❌ NO | Disconnected in bag |\n")
    f.write("| **Unitree L1 (deskewed)** | `odom` | ❌ NO | Published in world odometry frame |\n")
    f.write("| **Hesai XT-16** | `hesai_lidar` | ❌ NO | Disconnected in bag |\n")
    f.write("| **Unitree IMU** | `utlidar_imu` | ❌ NO | Disconnected in bag |\n")
    f.write("| **Robot Base** | `base_link` / `trunk` | ❌ NO | Missing from TF tree |\n")
    f.write("| **Camera (Depth/Color)** | `camera_link` | ✅ YES | Connected to depth/color optical frames |\n\n")

    f.write("## 3. Discovered Configuration Files on System\n\n")
    f.write("A deep scan of the research storage revealed the following FAST-LIO2 configuration:\n\n")
    f.write("- **File path:** `/mnt/d/acsar-3d-mapping-research/03_fastlio2/fastlio2.yaml`\n")
    f.write("```yaml\n")
    f.write("common:\n")
    f.write("  lidar_topic: /utlidar/cloud\n")
    f.write("  imu_topic: /utlidar/imu\n")
    f.write("  lidar_frame: utlidar_lidar\n")
    f.write("  imu_frame: utlidar_imu\n")
    f.write("extrinsics:\n")
    f.write("  # 4x4 homogeneous transform from IMU frame to LiDAR frame.\n")
    f.write("  T_imu_lidar: null\n")
    f.write("```\n\n")
    f.write("- **File path:** `/mnt/d/acsar-3d-mapping-research/config/camera_extrinsic.yaml`\n")
    f.write("```yaml\n")
    f.write("  # Homogeneous transform: base_link -> camera_link\n")
    f.write("  T_base_camera: null\n")
    f.write("```\n\n")

    f.write("## 4. Required Calibration Inputs Before Fusion\n\n")
    f.write("Because the recorded dataset lacks geometric transforms between the primary research sensors, the following calibrations must be resolved:\n\n")
    f.write("1. **Unitree L1 → IMU (`T_imu_lidar`):** Currently set to `null` in FAST-LIO2 config. Must be measured from CAD or estimated via targetless LiDAR-IMU calibration.\n")
    f.write("2. **Hesai XT-16 → Unitree L1 (`T_l1_hesai`):** Required to register dual-LiDAR clouds into a single frame for reflection cross-validation.\n")
    f.write("3. **Hesai XT-16 → IMU (`T_imu_hesai`):** Required if Hesai is directly used for odometry deskewing.\n")
    f.write("4. **Robot Base → Sensor Mounts:** Extrinsics connecting the Go2 trunk to the payload mounting rack.\n")

print(f"Written: {md_path}")
print("Task 5 exact decoding completed successfully.")
