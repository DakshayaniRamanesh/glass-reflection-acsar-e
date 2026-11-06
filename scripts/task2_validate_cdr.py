#!/usr/bin/env python3
"""
Task 2: CDR decoder validation.
- Verify our custom CDR decoder against expected values
- Attempt rosbag2_py import with correct ROS2 environment paths
- Cross-validate field layout, timestamps, point coordinates, NaN counts
- Sample multiple messages (not just first) to check consistency
"""
import sqlite3, struct, os, sys, math
import numpy as np

BAG_DIR = "/mnt/d/glass-research/rosbag2_2026_10_01-16_49_33"
DB3 = f"{BAG_DIR}/rosbag2_2026_10_01-16_49_33_0.db3"
OUT  = "/mnt/d/glass-research/acsar_e_m1/reports"
os.makedirs(OUT, exist_ok=True)

TOPICS = {
    "/utlidar/cloud":          "L1_raw",
    "/utlidar/cloud_deskewed": "L1_deskewed",
    "/lidar_points":           "Hesai_XT16",
}

# ── 1. Try rosbag2_py with explicit sys.path patching ────────────────────────
print("=" * 60)
print("Attempt 1: Import rosbag2_py with ROS2 Foxy sys.path")
print("=" * 60)
FOXY_SITE = "/opt/ros/foxy/lib/python3.8/site-packages"
if FOXY_SITE not in sys.path:
    sys.path.insert(0, FOXY_SITE)
try:
    import rosbag2_py
    print(f"[OK] rosbag2_py imported: {rosbag2_py.__file__}")
    HAS_ROSBAG2_PY = True
except ImportError as e:
    print(f"[FAIL] rosbag2_py not importable even with Foxy site-packages: {e}")
    HAS_ROSBAG2_PY = False

print()
print("Attempt 2: Check rclpy deserialization path")
try:
    import rclpy
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import PointCloud2
    print("[OK] rclpy + sensor_msgs.msg.PointCloud2 importable")
    HAS_RCLPY_DESER = True
except ImportError as e:
    print(f"[FAIL] {e}")
    HAS_RCLPY_DESER = False

# ── CDR decoder (same as extract_sample.py) ──────────────────────────────────
class CDRReader:
    def __init__(self, data):
        self.data = data
        self.pos  = 4
    def _align(self, n):
        r = self.pos % n
        if r: self.pos += (n - r)
    def u8(self):
        v = struct.unpack_from('B', self.data, self.pos)[0]; self.pos+=1; return v
    def i32(self):
        self._align(4); v = struct.unpack_from('<i', self.data, self.pos)[0]; self.pos+=4; return v
    def u32(self):
        self._align(4); v = struct.unpack_from('<I', self.data, self.pos)[0]; self.pos+=4; return v
    def string(self):
        length = self.u32()
        if length == 0: return ""
        raw = self.data[self.pos:self.pos+length]; self.pos+=length
        return raw.rstrip(b'\x00').decode('utf-8','replace')

DTYPE_NP   = {1:np.int8,2:np.uint8,3:np.int16,4:np.uint16,5:np.int32,6:np.uint32,7:np.float32,8:np.float64}
DTYPE_SIZE = {1:1,2:1,3:2,4:2,5:4,6:4,7:4,8:8}
DTYPE_NAME = {1:'INT8',2:'UINT8',3:'INT16',4:'UINT16',5:'INT32',6:'UINT32',7:'FLOAT32',8:'FLOAT64'}

def parse_pc2_full(raw):
    r = CDRReader(raw)
    stamp_sec  = r.i32()
    stamp_nsec = r.u32()
    frame_id   = r.string()
    height     = r.u32()
    width      = r.u32()
    n_fields   = r.u32()
    fields = []
    for _ in range(n_fields):
        fname    = r.string()
        offset   = r.u32()
        dtype_id = r.u8(); r._align(4)
        count    = r.u32()
        fields.append({"name":fname,"offset":offset,"dtype_id":dtype_id,"count":count})
    is_bigendian = r.u8(); r._align(4)
    point_step = r.u32()
    row_step   = r.u32()
    data_len   = r.u32()
    data_start = r.pos
    is_dense   = r.u8() if r.pos < len(raw) else None
    return {
        "stamp_ns": stamp_sec*1_000_000_000 + stamp_nsec,
        "frame_id": frame_id, "height": height, "width": width,
        "fields": fields, "is_bigendian": bool(is_bigendian),
        "point_step": point_step, "row_step": row_step,
        "data_len": data_len, "data_start": data_start,
        "is_dense": bool(is_dense) if is_dense is not None else None,
        "n_points": height*width,
    }

def pc2_to_array(raw, info):
    N = info["n_points"]; ps = info["point_step"]
    if N==0 or ps==0: return {}, []
    names=[]; formats=[]; offsets=[]
    for f in info["fields"]:
        np_dt = DTYPE_NP.get(f["dtype_id"], np.uint8)
        sz    = DTYPE_SIZE.get(f["dtype_id"],1)
        for i in range(f["count"]):
            names.append(f["name"] if f["count"]==1 else f"{f['name']}_{i}")
            formats.append(np_dt)
            offsets.append(f["offset"] + i*sz)
    dt = np.dtype({"names":names,"formats":formats,"offsets":offsets,"itemsize":ps})
    blob = raw[info["data_start"]: info["data_start"] + N*ps]
    actual_N = len(blob)//ps
    if actual_N < N:
        print(f"  [WARN] data blob shorter than expected: got {actual_N} pts, expected {N}")
    arr = np.frombuffer(blob[:actual_N*ps], dtype=dt)
    return {n: arr[n].astype(np.float64) for n in names}, names

# ── Validation per topic ──────────────────────────────────────────────────────
def validate_topic(conn, topic, label, n_check=5):
    print(f"\n{'='*60}")
    print(f"TOPIC: {topic}  [{label}]")
    cur = conn.cursor()
    cur.execute("SELECT id FROM topics WHERE name=?", (topic,))
    row = cur.fetchone()
    if not row:
        print("  [MISSING] topic not in this bag"); return {}
    tid = row[0]
    cur.execute("SELECT COUNT(*) FROM messages WHERE topic_id=?", (tid,))
    total = cur.fetchone()[0]
    print(f"  Total messages: {total}")
    if total == 0:
        print("  [EMPTY]"); return {}

    # Sample messages evenly across the bag
    cur.execute("""
        SELECT timestamp, data FROM messages WHERE topic_id=?
        ORDER BY timestamp ASC
    """, (tid,))
    all_rows = cur.fetchall()
    step  = max(1, len(all_rows)//n_check)
    sample= all_rows[::step][:n_check]

    results = []
    stamp_ns_list=[]; bag_ts_list=[]; npts_list=[]

    checks = {
        "frame_id_consistent": True,
        "point_step_consistent": True,
        "fields_consistent": True,
        "data_len_matches": True,
        "stamps_monotonic": True,
        "no_nan_xyz_majority": True,
        "coords_plausible": True,
        "intensity_in_range": True,
    }

    first_info = None
    for idx,(bag_ts, raw) in enumerate(sample):
        try:
            info = parse_pc2_full(raw)
        except Exception as e:
            print(f"  [PARSE ERROR seq={idx}]: {e}"); continue
        if first_info is None:
            first_info = info

        # Consistency checks
        if info["frame_id"] != first_info["frame_id"]:
            checks["frame_id_consistent"] = False
        if info["point_step"] != first_info["point_step"]:
            checks["point_step_consistent"] = False
        if [f["name"] for f in info["fields"]] != [f["name"] for f in first_info["fields"]]:
            checks["fields_consistent"] = False

        # Data length check
        expected_data = info["n_points"] * info["point_step"]
        if info["data_len"] != expected_data:
            checks["data_len_matches"] = False

        stamp_ns_list.append(info["stamp_ns"])
        bag_ts_list.append(bag_ts)
        npts_list.append(info["n_points"])

        # Decode points
        cols, names = pc2_to_array(raw, info)
        if not cols:
            continue

        x = cols.get("x", np.array([]))
        y = cols.get("y", np.array([]))
        z = cols.get("z", np.array([]))
        inten = cols.get("intensity", cols.get("intensity_0", np.array([])))

        nan_x = int(np.sum(np.isnan(x))) if len(x) else 0
        nan_y = int(np.sum(np.isnan(y))) if len(y) else 0
        nan_z = int(np.sum(np.isnan(z))) if len(z) else 0
        total_pts = len(x)
        nan_pct = 100*nan_x/total_pts if total_pts else 0

        # Coordinate plausibility (indoor robot: expect |xyz| < 100 m)
        if len(x) > 0:
            finite_x = x[np.isfinite(x)]
            if len(finite_x) > 0:
                rng = max(abs(finite_x.max()), abs(finite_x.min()))
                if rng > 500:
                    checks["coords_plausible"] = False

        nan_majority = nan_pct > 50
        if nan_majority:
            checks["no_nan_xyz_majority"] = False

        print(f"\n  --- Sample seq={idx} ---")
        print(f"    bag_ts     : {bag_ts}")
        print(f"    header_ns  : {info['stamp_ns']}")
        print(f"    lag_ms     : {(bag_ts - info['stamp_ns'])/1e6:+.3f}")
        print(f"    frame_id   : '{info['frame_id']}'")
        print(f"    n_points   : {info['n_points']}  (height={info['height']} × width={info['width']})")
        print(f"    point_step : {info['point_step']}  row_step={info['row_step']}")
        print(f"    data_len   : {info['data_len']}  expected={info['n_points']*info['point_step']}  match={info['data_len']==info['n_points']*info['point_step']}")
        print(f"    is_dense   : {info['is_dense']}  is_bigendian={info['is_bigendian']}")
        print(f"    NaN xyz    : {nan_x}/{nan_y}/{nan_z}  ({nan_pct:.1f}% NaN)")
        if len(x)>0 and np.any(np.isfinite(x)):
            fx=x[np.isfinite(x)]; fy=y[np.isfinite(y)]; fz=z[np.isfinite(z)]
            print(f"    x range    : [{fx.min():.3f}, {fx.max():.3f}] m")
            print(f"    y range    : [{fy.min():.3f}, {fy.max():.3f}] m")
            print(f"    z range    : [{fz.min():.3f}, {fz.max():.3f}] m")
        if len(inten)>0:
            fi=inten[np.isfinite(inten)]
            if len(fi)>0:
                print(f"    intensity  : min={fi.min():.2f}  max={fi.max():.2f}  mean={fi.mean():.2f}")
                if fi.max() > 1e6 or fi.min() < -1e6:
                    checks["intensity_in_range"] = False

        # Ring distribution
        if "ring" in cols:
            rings = cols["ring"].astype(int)
            unique_rings = np.unique(rings)
            print(f"    rings      : {unique_rings.min()}–{unique_rings.max()}  ({len(unique_rings)} unique)")

        # Per-point timestamp
        for ts_field in ["time","timestamp"]:
            if ts_field in cols:
                ts_arr = cols[ts_field]
                fi_ts = ts_arr[np.isfinite(ts_arr)]
                if len(fi_ts)>0:
                    print(f"    {ts_field:10s}: min={fi_ts.min():.6f}  max={fi_ts.max():.6f}  range={fi_ts.max()-fi_ts.min():.6f}")

    # Monotonicity check
    if len(stamp_ns_list) > 1:
        diffs = [stamp_ns_list[i+1]-stamp_ns_list[i] for i in range(len(stamp_ns_list)-1)]
        if any(d < 0 for d in diffs):
            checks["stamps_monotonic"] = False

    # Summary
    print(f"\n  === Validation Summary ===")
    for k,v in checks.items():
        status = "PASS" if v else "FAIL"
        print(f"    [{status}] {k}")

    if len(npts_list) > 1:
        print(f"\n  Point count range across {len(npts_list)} samples: {min(npts_list)} – {max(npts_list)}")
    if len(stamp_ns_list) > 1:
        intervals_ms = [(stamp_ns_list[i+1]-stamp_ns_list[i])/1e6
                        for i in range(len(stamp_ns_list)-1)]
        avg_int = sum(intervals_ms)/len(intervals_ms)
        # Note: these are sampled, not consecutive, so divide by step
        print(f"  Stamp intervals (sampled, not consecutive): {[f'{x:.1f}' for x in intervals_ms]} ms")

    return {"topic": topic, "label": label, "checks": checks,
            "first_info": first_info, "n_samples": len(npts_list)}

# ── rclpy cross-validation ────────────────────────────────────────────────────
def rclpy_crosscheck(raw, topic):
    if not HAS_RCLPY_DESER:
        print("  [SKIP] rclpy deserialization not available")
        return
    try:
        from rclpy.serialization import deserialize_message
        from sensor_msgs.msg import PointCloud2
        msg = deserialize_message(raw, PointCloud2)
        print(f"  [rclpy cross-check for {topic}]")
        print(f"    frame_id  : '{msg.header.frame_id}'")
        print(f"    stamp     : {msg.header.stamp.sec}s + {msg.header.stamp.nanosec}ns")
        print(f"    height    : {msg.height}  width={msg.width}")
        print(f"    point_step: {msg.point_step}  row_step={msg.row_step}")
        print(f"    is_dense  : {msg.is_dense}")
        print(f"    fields    : {[(f.name, f.offset, f.datatype, f.count) for f in msg.fields]}")
        print(f"    data len  : {len(msg.data)}")
    except Exception as e:
        print(f"  [rclpy cross-check failed]: {e}")

# ── Main ─────────────────────────────────────────────────────────────────────
print("\n" + "=" * 60)
print("TASK 2: CDR decoder validation — bag 16_49_33")
print("=" * 60)

conn = sqlite3.connect(f"file:{DB3}?mode=ro", uri=True)

validation_results = []
for topic, label in TOPICS.items():
    r = validate_topic(conn, topic, label, n_check=5)
    validation_results.append(r)

# rclpy cross-check on first L1 raw message
print("\n" + "=" * 60)
print("Cross-validation: rclpy deserializer vs our CDR parser")
print("=" * 60)
cur = conn.cursor()
cur.execute("SELECT id FROM topics WHERE name='/utlidar/cloud'")
row = cur.fetchone()
if row:
    cur.execute("SELECT data FROM messages WHERE topic_id=? LIMIT 1", (row[0],))
    raw_msg = cur.fetchone()
    if raw_msg:
        rclpy_crosscheck(raw_msg[0], "/utlidar/cloud")

conn.close()

# ── Write validation report ───────────────────────────────────────────────────
rpt_path = f"{OUT}/pointcloud_validation.md"
with open(rpt_path,"w") as f:
    f.write("# PointCloud2 CDR Decoder Validation Report\n\n")
    f.write(f"**Bag:** `rosbag2_2026_10_01-16_49_33`\n")
    f.write(f"**rosbag2_py available:** {'YES' if HAS_ROSBAG2_PY else 'NO'}\n")
    f.write(f"**rclpy deserialization available:** {'YES' if HAS_RCLPY_DESER else 'NO'}\n\n")
    for r in validation_results:
        if not r: continue
        f.write(f"## {r['label']} — `{r['topic']}`\n\n")
        for k,v in r.get("checks",{}).items():
            status = "✅ PASS" if v else "❌ FAIL"
            f.write(f"- {status}: {k}\n")
        f.write("\n")
print(f"\n[OK] Written: {rpt_path}")
print("\nTask 2 DONE.")

