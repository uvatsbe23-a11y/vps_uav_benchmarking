#!/usr/bin/env python3
"""
flight_analysis_strict.py - like flight_analysis.py, but REFUSES to report a
drift number for an obviously-invalid run.

Rejects a run when:
  - the odom topic carries any NaN / Inf position
  - the FIRST pose is already > MAX_START_OFFSET_M from the origin
    (OpenVINS starts the state at 0,0,0 - a first sample far from origin means
     the recording is polluted by a stale/zombie estimator node, or the fast
     propagator blew up before init)
  - fewer than 2 poses

On rejection it prints WHY and exits non-zero, so the calling script can mark
the run BAD instead of writing a meaningless "7000 m / nan %" row.
"""
import math
import os
import sys

import numpy as np

MAX_START_OFFSET_M = 5.0


def read_positions_txt(path):
    """OpenVINS save_total_state file: whitespace cols
       t  qx qy qz qw  px py pz  vx vy vz  bgx bgy bgz  bax bay baz  [calib...]"""
    times, xyz = [], []
    with open(path) as fh:
        for line in fh:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            c = line.split()
            if len(c) < 8:
                continue
            try:
                t = float(c[0])
                px, py, pz = float(c[5]), float(c[6]), float(c[7])
            except ValueError:
                continue
            times.append(t)
            xyz.append((px, py, pz))
    return np.asarray(times), np.asarray(xyz)


def read_positions(bag_dir, odom_topic):
    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    reader = SequentialReader()
    reader.open(StorageOptions(uri=bag_dir, storage_id=""),
                ConverterOptions("cdr", "cdr"))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if odom_topic not in types:
        sys.exit(f"REJECT: topic '{odom_topic}' not in bag. Have: {sorted(types)}")
    cls = get_message(types[odom_topic])
    times, xyz = [], []
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic != odom_topic:
            continue
        m = deserialize_message(data, cls)
        pose = getattr(m, "pose", None)
        if pose is not None and hasattr(pose, "pose"):
            p = pose.pose.position
        elif pose is not None and hasattr(pose, "position"):
            p = pose.position
        else:
            continue
        s = m.header.stamp
        times.append(s.sec + s.nanosec * 1e-9)
        xyz.append((p.x, p.y, p.z))
    return np.asarray(times), np.asarray(xyz)


def main():
    if len(sys.argv) < 2:
        sys.exit("usage: flight_analysis_strict.py <bag_dir> [--odom-topic T] [--path-length M] [--label L]")
    bag = sys.argv[1]
    odom_topic = "/ov_msckf/odomimu"
    measured_len = None
    label = bag.rstrip("/").split("/")[-1]
    args = sys.argv[2:]
    for i, a in enumerate(args):
        if a == "--odom-topic" and i + 1 < len(args):
            odom_topic = args[i + 1]
        elif a == "--path-length" and i + 1 < len(args):
            measured_len = float(args[i + 1])
        elif a == "--label" and i + 1 < len(args):
            label = args[i + 1]

    if bag.endswith(".txt") and os.path.isfile(bag):
        times, xyz = read_positions_txt(bag)
        src = "state_estimate.txt"
    else:
        times, xyz = read_positions(bag, odom_topic)
        src = odom_topic

    print(f"\n=== {label} ===")
    print(f"  samples from {src:<20}: {len(xyz)}")
    if len(xyz) < 2:
        print(f"  REJECT: only {len(xyz)} pose(s); need >= 2.")
        sys.exit(2)

    n_bad = int(np.sum(~np.isfinite(xyz)))
    start_off = float(np.linalg.norm(xyz[0]))
    dur = float(times[-1] - times[0])
    print(f"  duration                       : {dur:.1f} s")
    print(f"  first-pose offset from origin  : {start_off:.3f} m   (limit {MAX_START_OFFSET_M})")
    print(f"  NaN/Inf position samples       : {n_bad}")

    if n_bad > 0:
        print("  REJECT: trajectory contains NaN/Inf - filter diverged or stream polluted.")
        sys.exit(2)
    if start_off > MAX_START_OFFSET_M:
        print("  REJECT: first pose far from origin - stale/zombie estimator on the topic,")
        print("          or fast-propagator blow-up before init. Kill leftover nodes and re-run.")
        sys.exit(2)

    seg = np.linalg.norm(np.diff(xyz, axis=0), axis=1)
    integrated = float(seg.sum())
    denom = measured_len if measured_len else integrated
    err_vec = xyz[-1] - xyz[0]
    err = float(np.linalg.norm(err_vec))
    z_dev = float(np.max(np.abs(xyz[:, 2] - xyz[0, 2])))
    drift = 100.0 * err / denom if denom else float("nan")

    print(f"  integrated path length         : {integrated:.2f} m")
    if measured_len:
        print(f"  measured course length (used)  : {denom:.2f} m")
    print(f"  endpoint error |end - start|    : {err:.3f} m")
    print(f"      x / y / z                  : {err_vec[0]:+.3f} / {err_vec[1]:+.3f} / {err_vec[2]:+.3f} m")
    print(f"  max |z - z_start|              : {z_dev:.3f} m   <-- Z-divergence watch")
    print(f"  DRIFT                          : {drift:.2f} %  of path length")
    print("  ACCEPT")


if __name__ == "__main__":
    main()
