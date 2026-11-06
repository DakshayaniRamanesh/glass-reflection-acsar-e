#!/usr/bin/env python3
"""
Task 1: Direct SQLite audit of all 6 bags.
Queries topics and messages tables directly — no CDR parsing.
Writes corrected bag_inventory.csv.
"""
import sqlite3, os, csv, json
from pathlib import Path

BAG_BASE = "/mnt/d/glass-research"
BAGS = [
    "rosbag2_2026_10_01-16_39_53",
    "rosbag2_2026_10_01-16_41_04",
    "rosbag2_2026_10_01-16_42_00",
    "rosbag2_2026_10_01-16_49_33",
    "rosbag2_2026_10_01-16_51_04",
    "rosbag2_2026_10_01-16_52_39",
]

TRACKED_TOPICS = [
    "/utlidar/cloud",
    "/utlidar/cloud_deskewed",
    "/lidar_points",
    "/utlidar/imu",
    "/utlidar/robot_odom",
    "/utlidar/robot_pose",
    "/lowstate",
    "/sportmodestate",
    "/tf",
    "/tf_static",
    "/wirelesscontroller",
    "/uslam/cloud_map",
    "/camera/color/image_raw",
    "/camera/depth/image_rect_raw",
    "/camera/color/camera_info",
    "/camera/depth/camera_info",
]

OUT_DIR = "/mnt/d/glass-research/acsar_e_m1/reports"
os.makedirs(OUT_DIR, exist_ok=True)

def audit_bag(bag_name):
    db3 = next(Path(f"{BAG_BASE}/{bag_name}").glob("*.db3"), None)
    if not db3:
        print(f"  [ERROR] No .db3 found in {bag_name}")
        return None

    conn = sqlite3.connect(f"file:{db3}?mode=ro", uri=True)
    cur  = conn.cursor()

    # Bag-level stats from messages table
    cur.execute("SELECT MIN(timestamp), MAX(timestamp), COUNT(*) FROM messages")
    min_ts, max_ts, total_msgs = cur.fetchone()
    duration_s = (max_ts - min_ts) / 1e9 if (min_ts and max_ts) else 0

    # All topics with their stored message counts
    cur.execute("""
        SELECT t.name, t.type, COUNT(m.id) as msg_count
        FROM topics t
        LEFT JOIN messages m ON m.topic_id = t.id
        GROUP BY t.id, t.name, t.type
        ORDER BY t.name
    """)
    all_topics = cur.fetchall()

    # Timestamp range per topic
    topic_detail = {}
    for name, typ, cnt in all_topics:
        if cnt > 0:
            cur.execute("""
                SELECT MIN(timestamp), MAX(timestamp)
                FROM messages m
                JOIN topics t ON m.topic_id = t.id
                WHERE t.name = ?
            """, (name,))
            tmin, tmax = cur.fetchone()
        else:
            tmin, tmax = None, None
        topic_detail[name] = {
            "type": typ, "count": cnt,
            "ts_min": tmin, "ts_max": tmax
        }

    conn.close()
    return {
        "bag": bag_name,
        "db3": str(db3),
        "size_gb": round(db3.stat().st_size / 1e9, 2),
        "total_msgs": total_msgs,
        "ts_start_ns": min_ts,
        "ts_end_ns": max_ts,
        "duration_s": round(duration_s, 2),
        "topics": topic_detail,
    }

print("=" * 70)
print("TASK 1 — Direct SQLite audit of all 6 bags")
print("=" * 70)

results = []
for bag in BAGS:
    print(f"\n[ {bag} ]")
    r = audit_bag(bag)
    if r:
        results.append(r)
        print(f"  Duration : {r['duration_s']:.2f} s")
        print(f"  Size     : {r['size_gb']} GB")
        print(f"  Total msg: {r['total_msgs']}")
        print(f"  Topics with messages:")
        for name, d in sorted(r['topics'].items()):
            if d['count'] > 0:
                rate = d['count'] / r['duration_s'] if r['duration_s'] > 0 else 0
                print(f"    {d['count']:7d}  {rate:6.1f} Hz  {name}")
        print(f"  Topics with 0 messages:")
        for name, d in sorted(r['topics'].items()):
            if d['count'] == 0:
                print(f"              0         {name}")

# ── Write corrected CSV ──────────────────────────────────────────────────────
print("\n\n" + "=" * 70)
print("Writing corrected bag_inventory.csv")
print("=" * 70)

rows = []
for r in results:
    row = {
        "bag_name": r["bag"],
        "db3_size_gb": r["size_gb"],
        "duration_s": r["duration_s"],
        "start_time_ns": r["ts_start_ns"],
        "total_msgs_actual": r["total_msgs"],
    }
    for t in TRACKED_TOPICS:
        d = r["topics"].get(t, {"count": 0})
        col = t.lstrip("/").replace("/", "_")
        row[f"count_{col}"] = d["count"]
    rows.append(row)

csv_path = f"{OUT_DIR}/bag_inventory.csv"
with open(csv_path, "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=rows[0].keys())
    w.writeheader()
    w.writerows(rows)
print(f"Written: {csv_path}")

# ── Cross-check against previous report ──────────────────────────────────────
print("\n\n" + "=" * 70)
print("Cross-check: Which bags have all 3 LiDAR topics with >0 messages?")
print("=" * 70)
for r in results:
    has_raw      = r["topics"].get("/utlidar/cloud",          {"count":0})["count"] > 0
    has_deskewed = r["topics"].get("/utlidar/cloud_deskewed", {"count":0})["count"] > 0
    has_hesai    = r["topics"].get("/lidar_points",           {"count":0})["count"] > 0
    all3 = "✓ ALL THREE" if (has_raw and has_deskewed and has_hesai) else ""
    raw_c = r["topics"].get("/utlidar/cloud",          {"count":0})["count"]
    desk_c= r["topics"].get("/utlidar/cloud_deskewed", {"count":0})["count"]
    hes_c = r["topics"].get("/lidar_points",           {"count":0})["count"]
    print(f"  {r['bag'][-8:]}  raw={raw_c:5d}  deskewed={desk_c:5d}  hesai={hes_c:5d}  {all3}")

print("\nTask 1 DONE.")
