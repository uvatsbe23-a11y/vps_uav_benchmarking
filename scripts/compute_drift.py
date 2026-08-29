#!/usr/bin/env python3
"""
Compute static drift from an OpenVINS /ov_msckf/poseimu recording.

Usage:
    python3 compute_drift.py <bag_path>

Reports:
    - total duration
    - total 3D displacement from first to last pose (drift)
    - average drift rate (cm/s) over the whole recording
    - max instantaneous displacement from the start position (worst-case drift)
"""
import sys
import numpy as np
import rosbag2_py
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message


def get_reader(bag_path):
    # try mcap first (Jazzy default), fall back to sqlite3 (Humble default)
    for storage_id in ("mcap", "sqlite3"):
        try:
            storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id=storage_id)
            converter_options = rosbag2_py.ConverterOptions("", "")
            reader = rosbag2_py.SequentialReader()
            reader.open(storage_options, converter_options)
            return reader
        except RuntimeError:
            continue
    raise RuntimeError(f"Could not open bag at {bag_path} with mcap or sqlite3 storage plugins")


def main():
    if len(sys.argv) not in (2, 3):
        print("Usage: python3 compute_drift.py <bag_path> [edge_window_sec]")
        print("  edge_window_sec: average this many seconds at start and end")
        print("                   (use for closed-loop motion tests; default 0 = static mode)")
        sys.exit(1)

    bag_path = sys.argv[1]
    edge_window = float(sys.argv[2]) if len(sys.argv) == 3 else 0.0
    reader = get_reader(bag_path)
    topic_types = reader.get_all_topics_and_types()
    type_map = {t.name: t.type for t in topic_types}

    target_topic = "/ov_msckf/poseimu"
    if target_topic not in type_map:
        print(f"Topic {target_topic} not found in bag. Found: {list(type_map.keys())}")
        sys.exit(1)

    msg_type = get_message(type_map[target_topic])

    times = []
    positions = []

    while reader.has_next():
        topic, data, t = reader.read_next()
        if topic != target_topic:
            continue
        msg = deserialize_message(data, msg_type)
        # PoseWithCovarianceStamped (OpenVINS poseimu) layout
        p = msg.pose.pose.position
        positions.append([p.x, p.y, p.z])
        times.append(t * 1e-9)  # ns -> s

    if len(positions) < 2:
        print("Not enough messages to compute drift.")
        sys.exit(1)

    positions = np.array(positions)
    times = np.array(times)
    times -= times[0]
    duration = times[-1]

    if edge_window > 0:
        start_mask = times <= edge_window
        end_mask = times >= (duration - edge_window)
        start = positions[start_mask].mean(axis=0)
        end = positions[end_mask].mean(axis=0)
        print(f"Averaged {start_mask.sum()} samples for start, {end_mask.sum()} for end (window={edge_window}s)")
    else:
        start = positions[0]
        end = positions[-1]

    total_drift_m = np.linalg.norm(end - start)

    # total path length actually traveled (sum of consecutive deltas)
    deltas = np.diff(positions, axis=0)
    step_dists = np.linalg.norm(deltas, axis=1)
    path_length_m = step_dists.sum()

    drift_pct_of_path = (total_drift_m / path_length_m * 100.0) if path_length_m > 0 else float('nan')

    # distance from start at every timestep, to catch worst excursion (not just endpoint)
    dist_from_start = np.linalg.norm(positions - positions[0], axis=1)
    max_drift_m = np.max(dist_from_start)
    max_drift_t = times[np.argmax(dist_from_start)]

    avg_rate_cm_s = (total_drift_m * 100.0) / duration

    print(f"Messages processed:      {len(positions)}")
    print(f"Duration:                {duration:.2f} s")
    print(f"Start position (m):      {start}")
    print(f"End position (m):        {end}")
    print(f"Total path length:       {path_length_m:.4f} m")
    print(f"Endpoint drift:          {total_drift_m*100:.4f} cm")
    print(f"Drift as % of path:      {drift_pct_of_path:.3f} %")
    print(f"Avg drift rate:          {avg_rate_cm_s:.4f} cm/s")
    print(f"Max excursion from t=0:  {max_drift_m*100:.4f} cm at t={max_drift_t:.1f}s (informational only for motion tests)")


if __name__ == "__main__":
    main()
