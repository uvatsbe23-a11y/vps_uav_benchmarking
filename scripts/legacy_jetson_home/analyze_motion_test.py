#!/usr/bin/env python3
"""
Analyze a cuVSLAM motion-test rosbag.

Usage:
    python3 analyze_motion_test.py <path_to_bag_folder>

Requires: rosbag2_py, rclpy (comes with ROS 2 humble install, no extra pip needed)
Run this WITH ros2 sourced:
    source /opt/ros/humble/setup.bash
    source ~/ros2_ws/install/setup.bash
    python3 analyze_motion_test.py motion_test_123456
"""
import sys
import math
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus

def main(bag_path):
    storage_options = StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = ConverterOptions('', '')
    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    topic_types = reader.get_all_topics_and_types()
    type_map = {t.name: t.type for t in topic_types}

    positions = []      # (t, x, y, z)
    speeds = []          # (t, linear_speed_mag)
    cov_trace = []       # (t, trace of position covariance)
    vo_states = []       # (t, vo_state)

    while reader.has_next():
        topic, data, t = reader.read_next()
        if topic == '/visual_slam/tracking/odometry':
            msg = deserialize_message(data, Odometry)
            p = msg.pose.pose.position
            positions.append((t, p.x, p.y, p.z))
            lv = msg.twist.twist.linear
            speed = math.sqrt(lv.x**2 + lv.y**2 + lv.z**2)
            speeds.append((t, speed))
            cov = msg.pose.covariance
            trace = cov[0] + cov[7] + cov[14]  # xx + yy + zz
            cov_trace.append((t, trace))
        elif topic == '/visual_slam/status':
            try:
                msg = deserialize_message(data, VisualSlamStatus)
                vo_states.append((t, msg.vo_state))
            except Exception:
                pass

    if not positions:
        print("No odometry messages found in bag. Check topic name / bag path.")
        return

    print(f"Total odometry samples: {len(positions)}")
    t0 = positions[0][0]
    tN = positions[-1][0]
    duration_s = (tN - t0) / 1e9
    print(f"Duration: {duration_s:.2f} s")

    # --- Return-to-origin drift check ---
    start = positions[0]
    end = positions[-1]
    drift = math.sqrt((end[1]-start[1])**2 + (end[2]-start[2])**2 + (end[3]-start[3])**2)
    print(f"\n--- Return-to-origin drift ---")
    print(f"Start pos: x={start[1]:.4f} y={start[2]:.4f} z={start[3]:.4f}")
    print(f"End pos:   x={end[1]:.4f} y={end[2]:.4f} z={end[3]:.4f}")
    print(f"Drift (straight-line distance start->end): {drift*100:.2f} cm")
    print("  (If you returned the camera to its exact start point, this number")
    print("   IS your absolute drift error over the test. Under ~2-5cm is solid")
    print("   for a short indoor test; growing drift over longer/repeated tests")
    print("   signals accumulating VIO error that will need loop closure/mapping.)")

    # --- Max displacement (sanity check the "1m forward" move actually registered) ---
    max_disp = 0.0
    for (t, x, y, z) in positions:
        d = math.sqrt((x-start[1])**2 + (y-start[2])**2 + (z-start[3])**2)
        max_disp = max(max_disp, d)
    print(f"\n--- Max displacement from start ---")
    print(f"Max distance reached: {max_disp*100:.2f} cm")
    print("  (Should roughly match however far you actually moved it, e.g. ~100cm")
    print("   if you paced out 1 meter. If this is wildly off, scale/tracking is suspect.)")

    # --- Speed sanity ---
    max_speed = max(s for _, s in speeds)
    mean_speed = sum(s for _, s in speeds) / len(speeds)
    print(f"\n--- Velocity sanity ---")
    print(f"Mean speed: {mean_speed:.3f} m/s")
    print(f"Max speed:  {max_speed:.3f} m/s")
    print("  (Hand-carried camera motion is typically under ~1.5 m/s. Sustained")
    print("   readings above ~3-4 m/s usually mean the tracker lost good features")
    print("   and is extrapolating/drifting rather than genuinely tracking motion.)")

    # --- Covariance growth ---
    cov_start = cov_trace[0][1]
    cov_end = cov_trace[-1][1]
    cov_max = max(c for _, c in cov_trace)
    print(f"\n--- Position covariance (uncertainty) ---")
    print(f"Start trace: {cov_start:.6e}")
    print(f"End trace:   {cov_end:.6e}")
    print(f"Max trace:   {cov_max:.6e}")
    print("  (Should stay small and roughly flat if tracking is confident.")
    print("   Steadily climbing values across the whole test = growing uncertainty,")
    print("   a warning sign even if position numbers still look plausible.)")

    # --- Spike detection: find WHEN the worst moment happened ---
    max_speed_idx = max(range(len(speeds)), key=lambda i: speeds[i][1])
    max_cov_idx = max(range(len(cov_trace)), key=lambda i: cov_trace[i][1])
    t_speed_offset = (speeds[max_speed_idx][0] - t0) / 1e9
    t_cov_offset = (cov_trace[max_cov_idx][0] - t0) / 1e9
    print(f"\n--- Spike location (for correlating with what you were doing) ---")
    print(f"Max speed ({max_speed:.2f} m/s) occurred at t={t_speed_offset:.2f}s into the recording")
    print(f"Max covariance ({cov_max:.3e}) occurred at t={t_cov_offset:.2f}s into the recording")
    print("  (If these two timestamps are close together, it's one bad event -")
    print("   e.g. fast rotation, occlusion, motion blur, or a reflective/low-texture")
    print("   surface causing a brief mistrack. Recall what the camera was doing")
    print("   at roughly that point in the recording.)")

    # --- VO state check ---
    if vo_states:
        bad_states = [s for _, s in vo_states if s != 1]
        print(f"\n--- Tracker state (/visual_slam/status) ---")
        print(f"Total status messages: {len(vo_states)}")
        print(f"Non-tracking (vo_state != 1) messages: {len(bad_states)}")
        if bad_states:
            print("  WARNING: tracker reported lost/degraded state at some point during the test.")
        else:
            print("  All samples report vo_state == 1 (tracking). Good.")
    else:
        print("\n--- Tracker state ---")
        print("No /visual_slam/status messages recorded (add it to the bag record command).")

if __name__ == '__main__':
    if len(sys.argv) != 2:
        print("Usage: python3 analyze_motion_test.py <bag_folder>")
        sys.exit(1)
    main(sys.argv[1])
