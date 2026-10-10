#!/usr/bin/env python3
"""
Analyze an OpenVINS (ov_msckf) flight-test rosbag.

Usage:
    python3 flight_analysis.py <path_to_bag_folder>

Run WITH ros2 sourced:
    source /opt/ros/humble/setup.bash
    source ~/ros2_ws/install/setup.bash
    python3 flight_analysis.py ~/flight_20260728_120000
"""
import sys
import math
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry

def main(bag_path):
    storage_options = StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = ConverterOptions('', '')
    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    topic_types = reader.get_all_topics_and_types()
    type_map = {t.name: t.type for t in topic_types}
    print("Topics found in bag:")
    for name, ttype in type_map.items():
        print(f"  {name}: {ttype}")
    print()

    positions = []  # (t, x, y, z)

    odom_topic = '/odomimu'
    if odom_topic not in type_map:
        print(f"WARNING: '{odom_topic}' not found in bag. Available topics listed above.")
        print("Edit this script's odom_topic variable to match, or check ros2 bag info output.")
        return

    while reader.has_next():
        topic, data, t = reader.read_next()
        if topic == odom_topic:
            msg = deserialize_message(data, Odometry)
            p = msg.pose.pose.position
            positions.append((t, p.x, p.y, p.z))

    if not positions:
        print(f"No messages found on {odom_topic}. Check the bag actually recorded data.")
        return

    n = len(positions)
    t0 = positions[0][0]
    tN = positions[-1][0]
    duration_s = (tN - t0) / 1e9
    avg_rate = n / duration_s if duration_s > 0 else 0.0

    start = positions[0]
    end = positions[-1]

    drift_3d = math.sqrt((end[1]-start[1])**2 + (end[2]-start[2])**2 + (end[3]-start[3])**2)
    drift_2d = math.sqrt((end[1]-start[1])**2 + (end[2]-start[2])**2)

    path_length = 0.0
    for i in range(1, n):
        x0, y0, z0 = positions[i-1][1], positions[i-1][2], positions[i-1][3]
        x1, y1, z1 = positions[i][1], positions[i][2], positions[i][3]
        path_length += math.sqrt((x1-x0)**2 + (y1-y0)**2 + (z1-z0)**2)

    drift_pct = (drift_3d / path_length * 100) if path_length > 0 else 0.0
    drift_rate = drift_3d / duration_s if duration_s > 0 else 0.0

    altitudes = [p[3] for p in positions]  # assuming z = altitude / up axis; verify this matches your frame convention
    min_alt = min(altitudes)
    max_alt = max(altitudes)
    avg_alt = sum(altitudes) / len(altitudes)

    # --- Divergence detection: find WHEN altitude first goes unphysical ---
    divergence_t = None
    divergence_idx = None
    for i in range(1, n):
        dz = abs(positions[i][3] - positions[i-1][3])
        if dz > 2.0 and divergence_t is None:
            divergence_t = (positions[i][0] - t0) / 1e9
            divergence_idx = i
            break

    print("\n--- Divergence check ---")
    if divergence_t is not None:
        print(f"First unphysical altitude jump (>2m in one sample) at t={divergence_t:.2f}s into flight")
        print(f"  Altitude before: {positions[divergence_idx-1][3]:.2f}m -> after: {positions[divergence_idx][3]:.2f}m")
        print("  Likely point where OpenVINS lost visual tracking and began diverging")
        print("  on IMU-only propagation. Check the OpenVINS terminal output around")
        print("  this timestamp for warnings, feature-count drops, or reset messages.")
    else:
        print("No single-sample altitude jump >2m detected - divergence (if any) was")
        print("gradual rather than one catastrophic event.")
    print()

    print("=" * 50)
    print("VPS FLIGHT ANALYSIS REPORT")
    print("=" * 50)
    print(f"Flight duration:     {duration_s:.1f} seconds")
    print(f"Pose messages:       {n}")
    print(f"Average rate:        {avg_rate:.1f} Hz")
    print()
    print(f"Start position (m):  ({start[1]:.3f}, {start[2]:.3f}, {start[3]:.3f})")
    print(f"End position (m):    ({end[1]:.3f}, {end[2]:.3f}, {end[3]:.3f})")
    print()
    print(f"Endpoint drift (3D): {drift_3d*100:.1f} cm")
    print(f"Endpoint drift (2D): {drift_2d*100:.1f} cm")
    print(f"Total path length:   {path_length:.2f} m")
    print(f"Drift / path length: {drift_pct:.2f}%")
    print(f"Drift rate:          {drift_rate*100:.3f} cm/s")
    print()
    print(f"Min altitude:        {min_alt:.2f} m")
    print(f"Max altitude:        {max_alt:.2f} m")
    print(f"Avg altitude:        {avg_alt:.2f} m")
    print()
    print("=" * 50)
    print("INTERPRETATION")
    print("=" * 50)
    if drift_pct < 1.0:
        rating = "EXCELLENT"
    elif drift_pct < 3.0:
        rating = "GOOD - Suitable for supervised flight operations"
    elif drift_pct < 7.0:
        rating = "MARGINAL - Usable with caution, verify against GPS/other reference"
    else:
        rating = "POOR - Not recommended for positioning-critical use without further tuning"
    print(f"Accuracy rating: {rating}")
    print(f"Drift percentage: {drift_pct:.2f}% of total distance")
    print("=" * 50)
    print()
    print("NOTE: 'altitude' above assumes the odom frame's Z axis is aligned with")
    print("real-world up. If your OpenVINS config uses a different world-frame")
    print("convention (e.g. IMU-forward as Z), these altitude numbers may not")
    print("mean what they appear to - verify against your estimator_config.yaml")
    print("world frame definition before reporting these numbers as altitude.")

if __name__ == '__main__':
    if len(sys.argv) != 2:
        print("Usage: python3 flight_analysis.py <bag_folder>")
        sys.exit(1)
    main(sys.argv[1])
