# Autonomous UAV Visual Positioning System (VPS) Benchmarking

This repository provides configurations and analysis tools for evaluating Visual-Inertial Odometry (VIO) and Visual SLAM systems on NVIDIA Jetson paired with Orbbec Gemini cameras.

## Hardware Prerequisites
Lock Jetson clocks before flight to prevent IMU timing errors:
`sudo nvpmodel -m 0`
`sudo jetson_clocks`


## Software Dependencies
Before building this package, ensure the following drivers and libraries are installed on the Jetson:

**1. External ROS 2 Packages:**
* [Orbbec SDK ROS 2](https://github.com/orbbec/OrbbecSDK_ROS2) (Camera Drivers)
* [OpenVINS](https://docs.openvins.com/) (Stereo-Inertial MSCKF)
* [Isaac ROS cuVSLAM](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_visual_slam/index.html) (Optional, for pure-vision benchmarking)

**2. Python Libraries:**
The post-flight diagnostic scripts require standard data science libraries:
`pip install numpy matplotlib`



## Installation
Clone into your ROS 2 workspace:
`cd ~/ros2_ws/src`
`git clone https://github.com/uvatsbe23-a11y/vps_uav_benchmarking.git`
`cd ~/ros2_ws && colcon build --symlink-install`

## Running the VPS (OpenVINS on Gemini 335L)
Lock the Jetson clocks first (`sudo nvpmodel -m 0 && sudo jetson_clocks`), then, in two
sourced terminals:

```
ros2 launch vps_uav_benchmarking gemini335l_vio.launch.py        # Orbbec camera, VIO profile
ros2 launch vps_uav_benchmarking openvins_gemini335l.launch.py   # OpenVINS -> /ov_msckf/odomimu
```

`config/estimator_config.yaml` is the OpenVINS entry point; it pulls calibration from
`config/kalibr_imucam_chain.yaml` (holds `T_cam_imu`) and `config/kalibr_imu_chain.yaml`.

## Drift Test
Full staged procedure (bring-up → axis check → handheld loop → flight) is in
[`docs/TEST_PLAN.md`](docs/TEST_PLAN.md); record results in
[`docs/REPORT_TEMPLATE.md`](docs/REPORT_TEMPLATE.md). Background on the bug being validated is
in [`docs/PIPELINE_OVERVIEW.md`](docs/PIPELINE_OVERVIEW.md) and `docs/KNOWN_ISSUES.md`.

* **Preflight check:** `ros2 run vps_uav_benchmarking check_setup.sh`
* **Record a bag (raw topics, always):** `ros2 run vps_uav_benchmarking record_bag.sh <name>`

## Post-Flight Diagnostics
* **Drift Analyzer (headline number):** `python3 scripts/compute_drift.py /path/to/flight_bag [edge_window_sec]`
  Reads `/ov_msckf/poseimu` from a rosbag2 recording (mcap or sqlite3) and reports duration, path length, endpoint drift, drift as % of path, and max excursion from the start position. Pass `edge_window_sec` to average that many seconds at the start/end (useful for closed-loop motion tests that return to the start); omit it for static/point-to-point drift.
* **Batch analyzer (per-run CSV + plots + Z-axis alarm):** `python3 scripts/flight_analysis.py <bag_dir> --path-length <m> --plot --csv summary.csv`
  Reprocesses one bag, appends a summary row to a CSV, and (with `--plot`) saves a top-down + altitude trajectory PNG. Built for batching the 5+ runs per stage in the test plan.
* **Report Graphs:** `python3 scripts/generate_briefing_graphs.py summary.csv`
