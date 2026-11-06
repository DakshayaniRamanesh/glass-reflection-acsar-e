# ACSAR-E M1 — Independent PointCloud2 Decoder Validation Report

> [!IMPORTANT]
> Cross-validation performed by directly deserializing serialized byte buffers using **both** the official ROS2 `rclpy.serialization.deserialize_message(raw, PointCloud2)` and our custom precision-preserving CDR parser.

## 1. Official ROS2 vs. Custom CDR Verification Summary

| Sensor Topic | Label | Frame ID | Point Count | Point Step | `is_dense` | `is_bigendian` | Official `rclpy` Comparison |
|:---|:---|:---|:---:|:---:|:---:|:---:|:---:|
| `/utlidar/cloud` | Unitree L1 Raw | `utlidar_lidar` | 1,179 | 32 B | `True` | `False` | **100% PASS (Byte-for-Byte Match)** |
| `/utlidar/cloud_deskewed` | Unitree L1 Deskewed | `odom` | 10,412 | 32 B | `True` | `False` | **100% PASS (Byte-for-Byte Match)** |
| `/lidar_points` | Hesai XT-16 | `hesai_lidar` | 64,000 | 26 B | `False` | `False` | **100% PASS (Byte-for-Byte Match)** |

## 2. Granular Field-by-Field Verification Matrix

| Checked Attribute | Unitree L1 Raw | Unitree L1 Deskewed | Hesai XT-16 | Standard Expected |
|:---|:---:|:---:|:---:|:---|
| `frame_id_match` | ✅ PASS | ✅ PASS | ✅ PASS | Exact match with ROS2 standard |
| `stamp_sec_match` | ✅ PASS | ✅ PASS | ✅ PASS | Exact match with ROS2 standard |
| `stamp_nsec_match` | ✅ PASS | ✅ PASS | ✅ PASS | Exact match with ROS2 standard |
| `height_match` | ✅ PASS | ✅ PASS | ✅ PASS | Exact match with ROS2 standard |
| `width_match` | ✅ PASS | ✅ PASS | ✅ PASS | Exact match with ROS2 standard |
| `is_bigendian_match` | ✅ PASS | ✅ PASS | ✅ PASS | Exact match with ROS2 standard |
| `point_step_match` | ✅ PASS | ✅ PASS | ✅ PASS | Exact match with ROS2 standard |
| `row_step_match` | ✅ PASS | ✅ PASS | ✅ PASS | Exact match with ROS2 standard |
| `is_dense_match` | ✅ PASS | ✅ PASS | ✅ PASS | Exact match with ROS2 standard |
| `data_length_match` | ✅ PASS | ✅ PASS | ✅ PASS | Exact match with ROS2 standard |
| `fields_descriptor_match` | ✅ PASS | ✅ PASS | ✅ PASS | Exact match with ROS2 standard |
| `data_buffer_bytes_identical` | ✅ PASS | ✅ PASS | ✅ PASS | Exact match with ROS2 standard |

## 3. Byte Alignment & Data Buffer Integrity

- **Encapsulation Header:** All ROS2 CDR streams begin with 4-byte encapsulation header `00 01 00 00` (little-endian standard).
- **String Padding:** String lengths accurately account for 4-byte boundary padding.
- **`is_dense` Parsing:** Read immediately after skipping `data_len` bytes. Matches ROS2 `msg.is_dense` across all topics.
- **Endianness:** `is_bigendian == False` (little-endian) across all streams, matching host architecture.
- **Point Buffer Slicing:** Serialized point array offsets match the official `msg.data` byte-for-byte across all three topics.
- **Field Dtype Preservation:** NumPy structured arrays accurately bind IEEE 754 float64 (8B), float32 (4B), and uint16 (2B) without conversion or downcasting.
