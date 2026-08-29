#!/usr/bin/env python3
"""
flight_analysis.py - trajectory drift analysis for VPS test bags.

Reads an odometry topic from a rosbag2 recording and reports:
  - duration and number of poses
  - integrated path length (sum of consecutive position deltas)
  - endpoint error: |last_position - first_position|
        (on a closed-loop test the estimate SHOULD return to the start, so this
         is pure accumulated error)
  - drift  = endpoint error / path length          <-- the headline number
  - per-axis final error, with Z called out separately (the known cam/IMU bug
    showed up as Z / altitude divergence - see docs/KNOWN_ISSUES.md #2)
  - max |Z - Z_start| over the whole run (catches a mid-run altitude swing that
    partially recovers by the end)

USAGE
  Single bag:
    python3 flight_analysis.py stage2_run1 --path-length 60 --plot

  Batch (build the summary table for the report):
    for d in stage2_run*/ ; do
      python3 flight_analysis.py "$d" --path-length 60 --csv stage2_summary.csv
    done
    python3 generate_briefing_graphs.py stage2_summary.csv

ARGS
  bag              rosbag2 directory (the folder that contains metadata.yaml)
  --odom-topic     default /ov_msckf/odomimu
  --path-length M  measured course length in metres; overrides the integrated
                   length as the drift denominator. Use: (loop perimeter) x (laps).
  --plot           save <label>_trajectory.png next to the bag
  --label NAME     row label (defaults to the bag folder name)
  --csv FILE       append one summary row (writes a header if FILE is new)
"""
import argparse
import csv
import os
import sys

import numpy as np

try:
    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
except ImportError as exc:  # pragma: no cover
    sys.exit(
        f"ROS 2 Python packages not importable ({exc}).\n"
        "Source your workspace first:  source ~/ros2_ws/install/setup.bash"
    )


def read_positions(bag_dir, odom_topic):
    storage = StorageOptions(uri=bag_dir, storage_id="")  # "" => auto-detect from metadata.yaml
    conv = ConverterOptions(
        input_serialization_format="cdr", output_serialization_format="cdr"
    )
    reader = SequentialReader()
    reader.open(storage, conv)

    topic_types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if odom_topic not in topic_types:
        sys.exit(
            f"Topic '{odom_topic}' not in bag.\nAvailable topics:\n  "
            + "\n  ".join(sorted(topic_types))
        )
    msg_cls = get_message(topic_types[odom_topic])

    times, xyz = [], []
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic != odom_topic:
            continue
        msg = deserialize_message(data, msg_cls)

        # Odometry / PoseWithCovarianceStamped -> msg.pose.pose.position
        # PoseStamped                          -> msg.pose.position
        pose = getattr(msg, "pose", None)
        if pose is not None and hasattr(pose, "pose"):
            pos = pose.pose.position
        elif pose is not None and hasattr(pose, "position"):
            pos = pose.position
        else:
            continue
        stamp = msg.header.stamp
        times.append(stamp.sec + stamp.nanosec * 1e-9)
        xyz.append((pos.x, pos.y, pos.z))

    if len(xyz) < 2:
        sys.exit(f"Only {len(xyz)} pose msgs on '{odom_topic}'; need >= 2.")
    return np.asarray(times), np.asarray(xyz)


def analyse(times, xyz, measured_len=None):
    seg = np.linalg.norm(np.diff(xyz, axis=0), axis=1)
    integrated = float(seg.sum())
    denom = float(measured_len) if measured_len else integrated

    start, end = xyz[0], xyz[-1]
    err_vec = end - start
    err = float(np.linalg.norm(err_vec))
    z_dev = float(np.max(np.abs(xyz[:, 2] - start[2])))

    return {
        "n_poses": len(xyz),
        "duration_s": float(times[-1] - times[0]),
        "integrated_path_m": integrated,
        "path_length_used_m": denom,
        "endpoint_err_m": err,
        "err_x_m": float(err_vec[0]),
        "err_y_m": float(err_vec[1]),
        "err_z_m": float(err_vec[2]),
        "max_abs_z_dev_m": z_dev,
        "drift_pct": 100.0 * err / denom if denom else float("nan"),
    }


def make_plot(times, xyz, out_png, label):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("  (matplotlib not installed - skipping plot)")
        return
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11, 4.5))
    ax1.plot(xyz[:, 0], xyz[:, 1], "-", lw=1)
    ax1.plot(xyz[0, 0], xyz[0, 1], "go", label="start")
    ax1.plot(xyz[-1, 0], xyz[-1, 1], "rx", label="end")
    ax1.set_aspect("equal", "datalim")
    ax1.set_xlabel("x [m]")
    ax1.set_ylabel("y [m]")
    ax1.set_title(f"{label} - top-down path")
    ax1.legend()
    ax1.grid(alpha=0.3)
    ax2.plot(times - times[0], xyz[:, 2] - xyz[0, 2], "-", lw=1)
    ax2.set_xlabel("t [s]")
    ax2.set_ylabel("z - z0 [m]")
    ax2.set_title("altitude vs time (should stay flat)")
    ax2.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(out_png, dpi=120)
    print(f"  wrote {out_png}")


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("bag", help="rosbag2 directory (contains metadata.yaml)")
    ap.add_argument("--odom-topic", default="/ov_msckf/odomimu")
    ap.add_argument("--path-length", type=float, default=None)
    ap.add_argument("--plot", action="store_true")
    ap.add_argument("--label", default=None)
    ap.add_argument("--csv", default=None)
    args = ap.parse_args()

    label = args.label or os.path.basename(os.path.normpath(args.bag))
    times, xyz = read_positions(args.bag, args.odom_topic)
    r = analyse(times, xyz, args.path_length)

    print(f"\n=== {label} ===")
    print(f"  poses on {args.odom_topic:<22}: {r['n_poses']}")
    print(f"  duration                       : {r['duration_s']:.1f} s")
    print(f"  integrated path length         : {r['integrated_path_m']:.2f} m")
    if args.path_length:
        print(f"  measured course length (used)  : {r['path_length_used_m']:.2f} m")
    print(f"  endpoint error |end - start|    : {r['endpoint_err_m']:.3f} m")
    print(
        f"      x / y / z                  : "
        f"{r['err_x_m']:+.3f} / {r['err_y_m']:+.3f} / {r['err_z_m']:+.3f} m"
    )
    print(f"  max |z - z_start|              : {r['max_abs_z_dev_m']:.3f} m   <-- Z-divergence watch")
    print(f"  DRIFT                          : {r['drift_pct']:.2f} %  of path length\n")

    # crude alarm for a re-run of the axis bug (107 m / -76 m altitude last time)
    if r["max_abs_z_dev_m"] > 2.0 and r["max_abs_z_dev_m"] > 0.5 * r["path_length_used_m"]:
        print("  !!  large Z excursion relative to distance travelled - possible")
        print("      recurrence of the cam/IMU axis bug. Keep this bag and report it.\n")

    if args.csv:
        is_new = not os.path.exists(args.csv)
        with open(args.csv, "a", newline="") as fh:
            w = csv.writer(fh)
            if is_new:
                w.writerow(
                    ["label", "n_poses", "duration_s", "integrated_path_m",
                     "path_length_used_m", "endpoint_err_m", "err_x_m", "err_y_m",
                     "err_z_m", "max_abs_z_dev_m", "drift_pct"]
                )
            w.writerow(
                [label, r["n_poses"], f"{r['duration_s']:.2f}",
                 f"{r['integrated_path_m']:.3f}", f"{r['path_length_used_m']:.3f}",
                 f"{r['endpoint_err_m']:.4f}", f"{r['err_x_m']:.4f}",
                 f"{r['err_y_m']:.4f}", f"{r['err_z_m']:.4f}",
                 f"{r['max_abs_z_dev_m']:.4f}", f"{r['drift_pct']:.3f}"]
            )
        print(f"  appended row to {args.csv}")

    if args.plot:
        out_dir = os.path.dirname(os.path.normpath(args.bag)) or "."
        make_plot(times, xyz, os.path.join(out_dir, f"{label}_trajectory.png"), label)


if __name__ == "__main__":
    main()
