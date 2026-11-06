#!/usr/bin/env python3
"""
Test fixed parse_pc2_metadata is_dense logic against official rclpy deserializer.
"""
import sqlite3, struct
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2

class CDRReader:
    def __init__(self, data: bytes):
        self.data = data
        self.pos = 4
    def _align(self, n: int):
        r = self.pos % n
        if r: self.pos += (n - r)
    def u8(self) -> int:
        v = struct.unpack_from('B', self.data, self.pos)[0]; self.pos += 1; return v
    def i32(self) -> int:
        self._align(4); v = struct.unpack_from('<i', self.data, self.pos)[0]; self.pos += 4; return v
    def u32(self) -> int:
        self._align(4); v = struct.unpack_from('<I', self.data, self.pos)[0]; self.pos += 4; return v
    def string(self) -> str:
        length = self.u32()
        if length == 0: return ""
        raw = self.data[self.pos: self.pos + length]; self.pos += length
        return raw.rstrip(b'\x00').decode('utf-8', errors='replace')
    def skip(self, n: int):
        self.pos += n

def parse_meta(raw):
    r = CDRReader(raw)
    sec = r.i32(); nsec = r.u32()
    frame_id = r.string()
    h = r.u32(); w = r.u32()
    n_f = r.u32()
    fields = []
    for _ in range(n_f):
        fn = r.string(); off = r.u32(); dt = r.u8(); r._align(4); cnt = r.u32()
        fields.append({'name': fn, 'offset': off, 'dtype_id': dt, 'count': cnt})
    is_big = r.u8(); r._align(4)
    pt_step = r.u32(); row_step = r.u32(); data_len = r.u32()
    data_start = r.pos
    r.skip(data_len)
    is_dense = bool(r.u8()) if r.pos < len(raw) else None
    return {
        'stamp_ns': sec * 1_000_000_000 + nsec,
        'frame_id': frame_id, 'h': h, 'w': w, 'is_big': bool(is_big),
        'pt_step': pt_step, 'row_step': row_step, 'data_len': data_len,
        'data_start': data_start, 'is_dense': is_dense, 'fields': fields
    }

DB3 = "/mnt/d/glass-research/rosbag2_2026_10_01-16_49_33/rosbag2_2026_10_01-16_49_33_0.db3"
conn = sqlite3.connect(f"file:{DB3}?mode=ro", uri=True)
cur = conn.cursor()

topics = ["/utlidar/cloud", "/utlidar/cloud_deskewed", "/lidar_points"]
for t in topics:
    cur.execute("SELECT id FROM topics WHERE name=?", (t,))
    tid = cur.fetchone()[0]
    cur.execute("SELECT data FROM messages WHERE topic_id=? ORDER BY timestamp ASC LIMIT 1", (tid,))
    raw = cur.fetchone()[0]
    
    msg = deserialize_message(raw, PointCloud2)
    meta = parse_meta(raw)
    
    match = (msg.is_dense == meta['is_dense'])
    status = "PASS" if match else "FAIL"
    print(f"Topic: {t}")
    print(f"  msg.is_dense: {msg.is_dense} | meta['is_dense']: {meta['is_dense']} -> [{status}]")
conn.close()
