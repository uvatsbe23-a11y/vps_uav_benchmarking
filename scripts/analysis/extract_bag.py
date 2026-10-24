#!/usr/bin/env python3
"""Extract OpenVINS pose + camera IMU from a ROS 2 sqlite bag into an .npz.
usage: extract_bag.py <bag_folder> <out.npz> [time_offset_s]
time_offset_s is added to all stamps (used for test #4's 1970-clock bag)."""
import sys, glob, sqlite3, numpy as np
from rclpy.serialization import deserialize_message
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import Imu

bag, out = sys.argv[1], sys.argv[2]
off = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0
db = sqlite3.connect("file:" + glob.glob(bag + "/*.db3")[0] + "?mode=ro", uri=True)
tid = {n: i for i, n in db.execute("select id,name from topics")}

def rows(name):
    return db.execute("select timestamp,data from messages where topic_id=? order by timestamp", (tid[name],))

def stamp(h): return h.stamp.sec + h.stamp.nanosec * 1e-9 + off

P = []
for rx, d in rows("/ov_msckf/poseimu"):
    m = deserialize_message(d, PoseWithCovarianceStamped)
    p, q, c = m.pose.pose.position, m.pose.pose.orientation, m.pose.covariance
    P.append([stamp(m.header), rx * 1e-9 + off, p.x, p.y, p.z, q.x, q.y, q.z, q.w, c[0], c[7], c[14]])
I = []
for rx, d in rows("/camera/gyro_accel/sample"):
    m = deserialize_message(d, Imu)
    w, a = m.angular_velocity, m.linear_acceleration
    I.append([stamp(m.header), w.x, w.y, w.z, a.x, a.y, a.z])
IMG = np.array([r[0] * 1e-9 + off for r in rows("/camera/left_ir/image_raw")])
np.savez(out, pose=np.array(P), imu=np.array(I), img_rx=IMG)
P = np.array(P)
print(f"pose: {len(P)}  {P[0,0]:.2f}..{P[-1,0]:.2f}   imu: {len(I)}   images: {len(IMG)}")
