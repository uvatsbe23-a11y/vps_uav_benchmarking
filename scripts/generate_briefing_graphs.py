#!/usr/bin/env python3
"""
generate_briefing_graphs.py <summary.csv>

Turns a CSV produced by `flight_analysis.py --csv` into one figure:
  top    - drift % per run (dashed line = 2% rough target)
  bottom - peak altitude excursion |z - z0| per run (Z-divergence watch)

Writes drift_summary.png next to the CSV.
"""
import csv
import os
import sys

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
except ImportError:
    sys.exit("matplotlib not installed:  pip install matplotlib")


def main():
    if len(sys.argv) < 2:
        sys.exit("usage: generate_briefing_graphs.py <summary.csv>")
    path = sys.argv[1]

    labels, drift, zdev = [], [], []
    with open(path) as fh:
        for row in csv.DictReader(fh):
            labels.append(row["label"])
            drift.append(float(row["drift_pct"]))
            zdev.append(float(row["max_abs_z_dev_m"]))

    if not labels:
        sys.exit(f"{path} has no data rows")

    x = range(len(labels))
    fig, (a1, a2) = plt.subplots(2, 1, figsize=(max(6, len(labels) * 0.8), 7))

    a1.bar(x, drift, color="tab:blue")
    a1.axhline(2.0, ls="--", lw=1, color="grey")
    a1.set_ylabel("drift [% of path]")
    a1.set_title("Drift per run")
    a1.set_xticks(list(x))
    a1.set_xticklabels(labels, rotation=45, ha="right")

    a2.bar(x, zdev, color="tab:orange")
    a2.set_ylabel("max |z - z0| [m]")
    a2.set_title("Peak altitude excursion per run")
    a2.set_xticks(list(x))
    a2.set_xticklabels(labels, rotation=45, ha="right")

    fig.tight_layout()
    out = os.path.join(os.path.dirname(os.path.abspath(path)), "drift_summary.png")
    fig.savefig(out, dpi=120)
    print("wrote", out)


if __name__ == "__main__":
    main()
