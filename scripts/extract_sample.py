#!/usr/bin/env python3
"""
ACSAR-E M1 — Multi-Epoch Precision-Preserving Sample Extractor
=============================================================
Features:
  1. Fixes PointCloud2 CDR 'is_dense' parsing bug (reads AFTER data_len bytes).
  2. Preserves exact native field datatypes using NumPy structured arrays:
       - Hesai: x(f4), y(f4), z(f4), intensity(f4), ring(u2), timestamp(f8)
       - L1 Raw: x(f4), y(f4), z(f4), intensity(f4), ring(u2), time(f4)
       - L1 Deskewed: x(f4), y(f4), z(f4), intensity(f4)
  3. Includes 'valid_mask' boolean array filtering zero-coordinate padding.
  4. Supports multi-epoch extraction across Early (startup), Mid (steady-state),
     and Late recording intervals.
"""

import argparse
import csv
import os
import struct
import sqlite3
import sys
from pathlib import Path
import numpy as np


# ─── CDR Reader ─────────────────────────────────────────────────────────────

class CDRReader:
    def __init__(self, data: bytes):
        self.data = data
        self.pos = 4  # skip 4-byte CDR encapsulation header

    def _align(self, n: int):
        r = self.pos % n
        if r:
            self.pos += (n - r)

    def u8(self) -> int:
        v = struct.unpack_from('B', self.data, self.pos)[0]
        self.pos += 1
        return v

    def i32(self) -> int:
        self._align(4)
        v = struct.unpack_from('<i', self.data, self.pos)[0]
        self.pos += 4
        return v

    def u32(self) -> int:
        self._align(4)
        v = struct.unpack_from('<I', self.data, self.pos)[0]
        self.pos += 4
        return v

    def string(self) -> str:
        length = self.u32()
        if length == 0:
            return ""
        raw = self.data[self.pos: self.pos + length]
        self.pos += length
        return raw.rstrip(b'\x00').decode('utf-8', errors='replace')

    def skip(self, n: int):
        self.pos += n


# ─── Exact Datatype Mapping ──────────────────────────────────────────────────

DTYPE_MAP_NUMPY = {
    1: np.int8,    # INT8
    2: np.uint8,   # UINT8
    3: np.int16,   # INT16
    4: np.uint16,  # UINT16
    5: np.int32,   # INT32
    6: np.uint32,  # UINT32
    7: np.float32, # FLOAT32
    8: np.float64, # FLOAT64
}


def parse_pc2_metadata(raw: bytes):
    r = CDRReader(raw)
    sec = r.i32()
    nsec = r.u32()
    frame_id = r.string()
    height = r.u32()
    width = r.u32()
    n_fields = r.u32()

    fields = []
    for _ in range(n_fields):
        fname = r.string()
        offset = r.u32()
        dtype_id = r.u8()
        r._align(4)
        count = r.u32()
        fields.append({
            'name': fname,
            'offset': offset,
            'dtype_id': dtype_id,
            'np_dtype': DTYPE_MAP_NUMPY.get(dtype_id, np.uint8),
            'count': count
        })

    is_bigendian = r.u8()
    r._align(4)
    point_step = r.u32()
    row_step = r.u32()
    data_len = r.u32()
    data_start = r.pos

    # CRITICAL BUG FIX: is_dense is located AFTER data_len bytes in CDR!
    r.skip(data_len)
    is_dense = bool(r.u8()) if r.pos < len(raw) else None

    return {
        'stamp_ns': sec * 1_000_000_000 + nsec,
        'frame_id': frame_id,
        'height': height,
        'width': width,
        'n_points': height * width,
        'fields': fields,
        'point_step': point_step,
        'is_bigendian': bool(is_bigendian),
        'is_dense': is_dense,
        'data_len': data_len,
        'data_start': data_start
    }


def pc2_to_structured_array(raw: bytes):
    meta = parse_pc2_metadata(raw)
    N = meta['n_points']
    ps = meta['point_step']
    data_start = meta['data_start']

    if N == 0 or ps == 0:
        return np.array([], dtype=[('x', '<f4')]), meta

    names = []
    formats = []
    offsets = []
    for f in meta['fields']:
        dt = f['np_dtype']
        sz = np.dtype(dt).itemsize
        if f['count'] == 1:
            names.append(f['name'])
            formats.append(dt)
            offsets.append(f['offset'])
        else:
            for i in range(f['count']):
                names.append(f"{f['name']}_{i}")
                formats.append(dt)
                offsets.append(f['offset'] + i * sz)

    struct_dt = np.dtype({
        'names': names,
        'formats': formats,
        'offsets': offsets,
        'itemsize': ps
    })

    blob = raw[data_start: data_start + N * ps]
    actual_N = len(blob) // ps
    structured = np.frombuffer(blob[:actual_N * ps], dtype=struct_dt)

    return structured, meta


# ─── Multi-Epoch Selection Logic ────────────────────────────────────────────

LIDAR_TOPICS = {
    '/utlidar/cloud':          ('l1_raw',      'L1_raw'),
    '/utlidar/cloud_deskewed': ('l1_deskewed', 'L1_deskewed'),
    '/lidar_points':           ('hesai',       'Hesai_XT16'),
}


def get_target_indices(total_msgs: int, mode: str = "segments"):
    """
    Selects representative indices spanning Early (startup), Mid (steady-state),
    and Late portions of the bag.
    """
    if mode == "first5":
        return [(i, "early") for i in range(min(5, total_msgs))]

    # Default "segments": 2 early, 2 mid, 2 late
    early_idx = [0, 1]
    mid_idx = [total_msgs // 2, total_msgs // 2 + 1]
    late_idx = [max(0, total_msgs - 2), max(0, total_msgs - 1)]

    targets = []
    for idx in early_idx:
        if 0 <= idx < total_msgs: targets.append((idx, "early"))
    for idx in mid_idx:
        if 0 <= idx < total_msgs: targets.append((idx, "mid_steady"))
    for idx in late_idx:
        if 0 <= idx < total_msgs: targets.append((idx, "late_steady"))
    return targets


def extract_representative_samples(bag_dir: str, out_dir: str, mode: str = "segments"):
    db3_files = list(Path(bag_dir).glob("*.db3"))
    if not db3_files:
        raise FileNotFoundError(f"No .db3 file found in {bag_dir}")
    db3_path = str(db3_files[0])
    bag_name = os.path.basename(bag_dir)

    conn = sqlite3.connect(f"file:{db3_path}?mode=ro", uri=True)
    cur = conn.cursor()

    index_rows = []

    for topic, (subdir, label) in LIDAR_TOPICS.items():
        cur.execute("SELECT id FROM topics WHERE name=?", (topic,))
        row = cur.fetchone()
        if not row:
            print(f"[SKIP] Topic {topic} not found in {bag_name}")
            continue
        topic_id = row[0]

        cur.execute("SELECT COUNT(*) FROM messages WHERE topic_id=?", (topic_id,))
        total_msgs = cur.fetchone()[0]

        target_specs = get_target_indices(total_msgs, mode=mode)
        print(f"\nExtracting representative samples for {topic} ({label}) — total {total_msgs} messages available...")

        target_dir = os.path.join(out_dir, subdir)
        os.makedirs(target_dir, exist_ok=True)

        for seq_in_epoch, (target_msg_idx, epoch_tag) in enumerate(target_specs):
            # Fetch message at target_msg_idx
            cur.execute(
                "SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp ASC LIMIT 1 OFFSET ?",
                (topic_id, target_msg_idx)
            )
            res = cur.fetchone()
            if not res: continue
            bag_ts, raw = res

            points_struct, meta = pc2_to_structured_array(raw)

            # Validity mask
            x = points_struct['x']
            y = points_struct['y']
            z = points_struct['z']

            exact_zero = (x == 0.0) & (y == 0.0) & (z == 0.0)
            nan_or_inf = np.isnan(x) | np.isnan(y) | np.isnan(z) | np.isinf(x) | np.isinf(y) | np.isinf(z)
            valid_mask = (~exact_zero) & (~nan_or_inf)

            n_total = len(points_struct)
            n_valid = int(np.sum(valid_mask))
            n_zeros = int(np.sum(exact_zero))

            fname = f"scan_{target_msg_idx:04d}_{epoch_tag}.npz"
            fpath = os.path.join(target_dir, fname)

            # Preserve exact field types
            export_dict = {
                'points': points_struct,
                'valid_mask': valid_mask,
                'header_stamp_ns': np.int64(meta['stamp_ns']),
                'bag_receipt_ns': np.int64(bag_ts),
                'frame_id': np.bytes_(meta['frame_id'].encode()),
                'height': np.int32(meta['height']),
                'width': np.int32(meta['width']),
                'point_step': np.int32(meta['point_step']),
                'is_dense': np.bool_(meta['is_dense'] if meta['is_dense'] is not None else False),
                'epoch_tag': np.bytes_(epoch_tag.encode()),
                'bag_msg_index': np.int32(target_msg_idx),
            }

            for name in points_struct.dtype.names:
                export_dict[f"field_{name}"] = points_struct[name]

            np.savez_compressed(fpath, **export_dict)

            lag_ms = (bag_ts - meta['stamp_ns']) / 1e6
            print(f"  [{label} #{target_msg_idx:04d} ({epoch_tag:11s})] Total={n_total:5d} | Valid={n_valid:5d} ({100*n_valid/n_total:5.1f}%) | "
                  f"Zeros={n_zeros:5d} | is_dense={meta['is_dense']} -> {fname}")

            index_rows.append({
                'bag_msg_index': target_msg_idx,
                'epoch_tag': epoch_tag,
                'bag_name': bag_name,
                'db3_file': os.path.basename(db3_path),
                'topic': topic,
                'sensor_label': label,
                'export_subdir': subdir,
                'export_file': fname,
                'header_stamp_ns': meta['stamp_ns'],
                'bag_receipt_ns': bag_ts,
                'lag_ms': round(lag_ms, 3),
                'point_count': n_total,
                'valid_point_count': n_valid,
                'zero_point_count': n_zeros,
                'frame_id': meta['frame_id'],
                'is_dense': meta['is_dense'],
                'point_step': meta['point_step'],
                'fields': ",".join([f"{f['name']}:{f['np_dtype'].__name__}" for f in meta['fields']])
            })

    conn.close()

    # Save scan index
    index_csv = os.path.join(out_dir, "../reports/scan_index.csv")
    os.makedirs(os.path.dirname(index_csv), exist_ok=True)
    if index_rows:
        with open(index_csv, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=index_rows[0].keys())
            w.writeheader()
            w.writerows(index_rows)
        print(f"\n[OK] Updated scan index: {index_csv}")

    return index_rows


def main():
    parser = argparse.ArgumentParser(description="ACSAR-E M1 Multi-Epoch Representative Sample Extractor")
    parser.add_argument("--bag", default="/mnt/d/glass-research/rosbag2_2026_10_01-16_49_33",
                        help="Path to rosbag2 directory")
    parser.add_argument("--mode", default="segments", choices=["segments", "first5"],
                        help="Extraction mode: 'segments' (early, mid, late) or 'first5'")
    parser.add_argument("--out", default="/mnt/d/glass-research/acsar_e_m1/exports",
                        help="Root output directory")
    args = parser.parse_args()

    print("=== ACSAR-E M1 Multi-Epoch Sample Extractor ===")
    print(f"Bag  : {args.bag}")
    print(f"Mode : {args.mode}")
    print(f"Out  : {args.out}")

    extract_representative_samples(args.bag, args.out, mode=args.mode)
    print("\nExtraction completed successfully.")


if __name__ == "__main__":
    main()
