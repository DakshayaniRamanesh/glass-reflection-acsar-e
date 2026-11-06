#!/usr/bin/env python3
"""
Inspect frames and initial poses of IMU, Robot Odom, and Robot Pose.
"""
import sqlite3
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

DB3 = "/mnt/d/glass-research/rosbag2_2026_10_01-16_49_33/rosbag2_2026_10_01-16_49_33_0.db3"
conn = sqlite3.connect(f"file:{DB3}?mode=ro", uri=True)
cur = conn.cursor()

# IMU
cur.execute("SELECT id FROM topics WHERE name='/utlidar/imu'")
row = cur.fetchone()
if row:
    tid = row[0]
    cur.execute("SELECT data FROM messages WHERE topic_id=? LIMIT 1", (tid,))
    raw = cur.fetchone()[0]
    imu = deserialize_message(raw, Imu)
    print("IMU:")
    print("  frame_id:", imu.header.frame_id)
    print(f"  linear_acc: ({imu.linear_acceleration.x:.3f}, {imu.linear_acceleration.y:.3f}, {imu.linear_acceleration.z:.3f})")
    print(f"  angular_vel: ({imu.angular_velocity.x:.3f}, {imu.angular_velocity.y:.3f}, {imu.angular_velocity.z:.3f})")

# Robot Odom
cur.execute("SELECT id FROM topics WHERE name='/utlidar/robot_odom'")
row = cur.fetchone()
if row:
    tid = row[0]
    cur.execute("SELECT data FROM messages WHERE topic_id=? LIMIT 1", (tid,))
    raw = cur.fetchone()[0]
    odom = deserialize_message(raw, Odometry)
    print("\nRobot Odom:")
    print("  header.frame_id:", odom.header.frame_id)
    print("  child_frame_id :", odom.child_frame_id)
    p = odom.pose.pose.position
    q = odom.pose.pose.orientation
    print(f"  position: ({p.x:.4f}, {p.y:.4f}, {p.z:.4f})")
    print(f"  orientation: ({q.x:.4f}, {q.y:.4f}, {q.z:.4f}, {q.w:.4f})")

# Robot Pose
cur.execute("SELECT id FROM topics WHERE name='/utlidar/robot_pose'")
row = cur.fetchone()
if row:
    tid = row[0]
    cur.execute("SELECT data FROM messages WHERE topic_id=? LIMIT 1", (tid,))
    raw = cur.fetchone()[0]
    pose = deserialize_message(raw, PoseStamped)
    print("\nRobot Pose:")
    print("  header.frame_id:", pose.header.frame_id)
    p = pose.pose.position
    q = pose.pose.orientation
    print(f"  position: ({p.x:.4f}, {p.y:.4f}, {p.z:.4f})")
    print(f"  orientation: ({q.x:.4f}, {q.y:.4f}, {q.z:.4f}, {q.w:.4f})")

conn.close()

