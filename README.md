# Autonomous UAV Visual Positioning System (VPS) Benchmarking

Configurations, field scripts and analysis tools for evaluating Visual-Inertial Odometry (VIO) on an
NVIDIA Jetson Orin Nano paired with an Orbbec Gemini 335L stereo IR camera (OpenVINS; cuVSLAM optional).

## Current status (24 Oct 2026)

First dynamic flight tests flown on **10 Oct 2026** (4 automatic GPS missions, camera facing down).
OpenVINS holds attitude and early scale, but its position drifts ~10 m after 50–100 m, and it
stopped mid-flight in 3 of 4 flights. **Start here:**

- [`docs/HANDOFF_2026-10-10.md`](docs/HANDOFF_2026-10-10.md): conclusions, what changed, and **what to do next** (incl. the next flight day checklist)
- [`docs/flight_tests/2026-10-10/`](docs/flight_tests/2026-10-10/): detailed flight analysis with figures
- [`docs/KNOWN_ISSUES.md`](docs/KNOWN_ISSUES.md) · [`docs/PIPELINE_OVERVIEW.md`](docs/PIPELINE_OVERVIEW.md) · [`docs/TEST_PLAN.md`](docs/TEST_PLAN.md)

## Hardware prerequisites (every boot)

```bash
sudo nvpmodel -m 2      # MAXN_SUPER on the Orin Nano Super. NOT -m 0: that is the throttled 15 W
                        # mode, under which OpenVINS could not keep up live and diverged.
sudo jetson_clocks
```

The Jetson's RTC has no battery: after power-on its clock is only the last saved time. Before
recording, set the real time from the ground PC with `scripts/flight/sync_jetson_clock.sh`.
`flight_start` (below) does the power checks and `jetson_clocks` for you.

## Software dependencies

- ROS 2 (vajra runs **Humble**), [Orbbec SDK ROS 2](https://github.com/orbbec/OrbbecSDK_ROS2),
  [OpenVINS](https://docs.openvins.com/) at upstream `6948812` **plus the local patches in
  [`third_party/open_vins/`](third_party/open_vins/)** (clean shutdown),
  optionally [Isaac ROS cuVSLAM](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_visual_slam/index.html).
- Analysis (ground PC): Python 3 with numpy, scipy, matplotlib, opencv-python, pymavlink, plus `rclpy`.

## Installation

```bash
cd ~/ros2_ws/src && git clone https://github.com/uvatsbe23-a11y/vps_uav_benchmarking.git
cd ~/ros2_ws && colcon build --symlink-install
```

## Running a test: one terminal, survives Wi-Fi drops

The `flight_*` commands ([`scripts/flight/`](scripts/flight/), linked into `/usr/local/bin` on vajra)
run everything detached from SSH, with per-flight log folders under `~/flights/<date>/`:

```bash
flight_start      # checks clock/disk/power, jetson_clocks, starts camera + OpenVINS
                  # → lift and slide the drone gently until flight_status says INITIALISED
flight_status     # (-w to refresh)
flight_record     # just before take-off
flight_stop       # after landing: stops recorder cleanly, then OpenVINS + camera; prints a summary
flight_logs       # follow the OpenVINS log (Ctrl+C stops only the viewer)
```

Manual alternative (two sourced terminals, **`export ROS_LOCALHOST_ONLY=1` in each**: the Jetson's
many network interfaces break DDS discovery otherwise):

```bash
ros2 launch vps_uav_benchmarking gemini335l_vio.launch.py        # Orbbec camera, VIO profile
ros2 launch vps_uav_benchmarking openvins_gemini335l.launch.py   # OpenVINS -> /ov_msckf/*
ros2 run vps_uav_benchmarking check_setup.sh                     # preflight check
ros2 run vps_uav_benchmarking record_bag.sh <name>               # raw topics, always
```

`config/estimator_config.yaml` is the OpenVINS entry point; calibration comes from
`config/kalibr_imucam_chain.yaml` (`T_cam_imu`: near-identity for the Orbbec built-in IMU) and
`config/kalibr_imu_chain.yaml` (IMU at 500 Hz).

## Post-flight analysis

- **Against flight-controller telemetry (recommended):** [`scripts/analysis/`](scripts/analysis/).
  Time-aligns the bag with the ArduPilot `.bin` log and reports ATE, true drift, scale, per-axis error,
  divergence distance, vibration and image quality. See its README.
- **Quick, VIO-only:** `python3 scripts/compute_drift.py <bag> [edge_window_sec]`. Endpoint drift for
  closed loops. Note: it divides by OpenVINS's *own* path length, which inflates when OpenVINS diverges
  (flight 1: 58–61 % vs a true 95 %).
- `scripts/flight_analysis.py` (per-run CSV + plots; pass `--odom-topic /ov_msckf/poseimu`) and
  `scripts/generate_briefing_graphs.py summary.csv`.

## Repository layout

| Path | Contents |
|---|---|
| `config/`, `launch/` | OpenVINS config + calibration; camera and OpenVINS launch files |
| `scripts/` | `check_setup.sh`, `record_bag.sh`, `compute_drift.py`, `flight_analysis.py`, `generate_briefing_graphs.py` |
| `scripts/flight/` | one-command flight routine + `sync_jetson_clock.sh` (ground PC) |
| `scripts/analysis/` | flight analysis vs FC telemetry |
| `scripts/jetson_system/` | Jetson system setup: clock restore, core dumps, `wifi-mode` |
| `scripts/replay_diag/`, `scripts/tools/` | bag-replay diagnostic harness and helpers (Jul–Sep) |
| `scripts/legacy_jetson_home/` | old unreviewed scripts, archived |
| `third_party/open_vins/` | OpenVINS local patches + extra configs |
| `ros2_packages/vps_bridge/` | OpenVINS → MAVROS bridge with pose sanity filter (not yet used in tests) |
| `docs/` | handoff, pipeline overview, known issues, test plan, report template, flight-test results |
