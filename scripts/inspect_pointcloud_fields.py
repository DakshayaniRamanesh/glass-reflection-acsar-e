#!/usr/bin/env python3
"""
Phase 2: Inspect PointCloud2 field structures directly via SQLite CDR parsing.
Fixed CDR alignment for ROS2 string fields.

Usage:  python3 inspect_pointcloud_fields.py
Output: stdout + acsar_e_m1/reports/pointcloud_fields.csv
"""

import sqlite3
import struct
import os
import csv

BAG_DIR = "/mnt/d/glass-research/rosbag2_2026_10_01-16_49_33"
DB3_FILE = os.path.join(BAG_DIR, "rosbag2_2026_10_01-16_49_33_0.db3")
REPORTS_DIR = "/mnt/d/glass-research/acsar_e_m1/reports"
os.makedirs(REPORTS_DIR, exist_ok=True)

TOPICS_OF_INTEREST = {
    "/utlidar/cloud":          "L1_raw",
    "/utlidar/cloud_deskewed": "L1_deskewed",
    "/lidar_points":           "Hesai_XT16",
}

DATATYPE_MAP = {
    1: "INT8",   2: "UINT8",  3: "INT16",  4: "UINT16",
    5: "INT32",  6: "UINT32", 7: "FLOAT32", 8: "FLOAT64",
}


# ── CDR primitives ──────────────────────────────────────────────────────────

class CDRReader:
    """Minimal CDR deserializer (little-endian, no XCDR2)."""

    def __init__(self, data: bytes):
        self.data = data
        self.pos = 0
        # Skip 4-byte CDR encapsulation header (00 01 00 00 for LE)
        self.pos = 4

    def _align(self, n):
        """Advance pos so it is aligned to n bytes."""
        r = self.pos % n
        if r:
            self.pos += (n - r)

    def u8(self):
        v = struct.unpack_from('B', self.data, self.pos)[0]
        self.pos += 1
        return v

    def i32(self):
        self._align(4)
        v = struct.unpack_from('<i', self.data, self.pos)[0]
        self.pos += 4
        return v

    def u32(self):
        self._align(4)
        v = struct.unpack_from('<I', self.data, self.pos)[0]
        self.pos += 4
        return v

    def string(self):
        """CDR string: uint32 length (including null), then bytes."""
        length = self.u32()
        if length == 0:
            return ""
        raw = self.data[self.pos: self.pos + length]
        self.pos += length
        return raw.rstrip(b'\x00').decode('utf-8', errors='replace')

    def skip(self, n):
        self.pos += n


# ── PointCloud2 parser ──────────────────────────────────────────────────────

def parse_pc2(raw: bytes) -> dict:
    r = CDRReader(raw)

    # std_msgs/Header
    stamp_sec  = r.i32()
    stamp_nsec = r.u32()
    frame_id   = r.string()

    # height, width
    height = r.u32()
    width  = r.u32()

    # fields[]
    n_fields = r.u32()
    fields = []
    for _ in range(n_fields):
        fname    = r.string()
        offset   = r.u32()
        datatype = r.u8()
        r._align(4)          # align before next u32
        count    = r.u32()
        fields.append({
            "name":    fname,
            "offset":  offset,
            "datatype": DATATYPE_MAP.get(datatype, f"UNK({datatype})"),
            "count":   count,
        })

    is_bigendian = r.u8()
    r._align(4)
    point_step = r.u32()
    row_step   = r.u32()

    # data[] — just read the array length, don't load bytes
    data_len = r.u32()

    is_dense = None
    if r.pos < len(raw):
        is_dense = bool(r.u8())

    return {
        "stamp_sec":    stamp_sec,
        "stamp_nanosec": stamp_nsec,
        "stamp_ns":     stamp_sec * 1_000_000_000 + stamp_nsec,
        "frame_id":     frame_id,
        "height":       height,
        "width":        width,
        "point_count":  height * width,
        "num_fields":   n_fields,
        "fields":       fields,
        "is_bigendian": bool(is_bigendian),
        "point_step":   point_step,
        "row_step":     row_step,
        "data_bytes":   data_len,
        "is_dense":     is_dense,
    }


# ── Per-topic inspection ────────────────────────────────────────────────────

def inspect_topic(conn, topic_name, label):
    cur = conn.cursor()

    cur.execute("SELECT id FROM topics WHERE name=?", (topic_name,))
    row = cur.fetchone()
    if row is None:
        print(f"\n[MISSING] {topic_name} — not in this bag")
        return None

    topic_id = row[0]

    cur.execute(
        "SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp ASC LIMIT 1",
        (topic_id,),
    )
    msg = cur.fetchone()
    if msg is None:
        print(f"\n[EMPTY]   {topic_name} — topic exists but 0 messages stored")
        return None

    bag_ts, raw = msg

    cur.execute("SELECT COUNT(*) FROM messages WHERE topic_id=?", (topic_id,))
    total = cur.fetchone()[0]

    print(f"\n{'='*62}")
    print(f"Topic : {topic_name}  [{label}]")
    print(f"  bag receipt ts   : {bag_ts} ns  ({bag_ts/1e9:.6f} s)")
    print(f"  raw CDR bytes    : {len(raw)}")

    try:
        info = parse_pc2(raw)
    except Exception as e:
        print(f"  [Parse error]: {e}")
        import traceback; traceback.print_exc()
        return None

    hdr_ns = info["stamp_ns"]
    offset_ms = (bag_ts - hdr_ns) / 1e6
    print(f"  header stamp     : {info['stamp_sec']}s + {info['stamp_nanosec']}ns = {hdr_ns} ns")
    print(f"  bag−header offset: {offset_ms:+.3f} ms")
    print(f"  frame_id         : '{info['frame_id']}'")
    print(f"  height x width   : {info['height']} x {info['width']}  => {info['point_count']} points")
    print(f"  point_step       : {info['point_step']} bytes/point")
    print(f"  row_step         : {info['row_step']} bytes/row")
    print(f"  is_bigendian     : {info['is_bigendian']}")
    print(f"  is_dense         : {info['is_dense']}")
    print(f"  data blob        : {info['data_bytes']} bytes")
    print(f"  total messages   : {total}")
    print(f"  Fields ({info['num_fields']}):")
    for f in info["fields"]:
        bar = "●" * f["count"]
        print(f"    [{f['offset']:3d}B]  {f['name']:<20s}  {f['datatype']:<8s}  ×{f['count']}")

    info["topic"]          = topic_name
    info["label"]          = label
    info["bag_ts_ns"]      = bag_ts
    info["bag_hdr_off_ms"] = offset_ms
    info["total_msgs"]     = total
    return info


# ── Main ────────────────────────────────────────────────────────────────────

def main():
    print(f"Reading: {DB3_FILE}\n")
    conn = sqlite3.connect(f"file:{DB3_FILE}?mode=ro", uri=True)

    results = []
    for topic, label in TOPICS_OF_INTEREST.items():
        r = inspect_topic(conn, topic, label)
        if r:
            results.append(r)

    conn.close()

    # ── Write CSV ──────────────────────────────────────────────────────────
    csv_path = os.path.join(REPORTS_DIR, "pointcloud_fields.csv")
    rows = []
    for r in results:
        for f in r["fields"]:
            rows.append({
                "source_bag":           os.path.basename(BAG_DIR),
                "topic":                r["topic"],
                "sensor_label":         r["label"],
                "frame_id":             r["frame_id"],
                "height":               r["height"],
                "width":                r["width"],
                "point_count":          r["point_count"],
                "point_step_bytes":     r["point_step"],
                "is_bigendian":         r["is_bigendian"],
                "is_dense":             r["is_dense"],
                "field_name":           f["name"],
                "field_offset_bytes":   f["offset"],
                "field_datatype":       f["datatype"],
                "field_count":          f["count"],
                "header_stamp_ns":      r["stamp_ns"],
                "bag_receipt_ns":       r["bag_ts_ns"],
                "bag_header_offset_ms": r["bag_hdr_off_ms"],
                "total_msgs_in_bag":    r["total_msgs"],
            })

    if rows:
        with open(csv_path, "w", newline="") as fh:
            w = csv.DictWriter(fh, fieldnames=rows[0].keys())
            w.writeheader()
            w.writerows(rows)
        print(f"\n[OK] pointcloud_fields.csv  →  {csv_path}")
    else:
        print("\n[WARNING] No data to write to CSV.")

if __name__ == "__main__":
    main()
