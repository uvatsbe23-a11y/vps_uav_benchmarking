# Flight analysis: OpenVINS vs flight-controller telemetry

Scripts used for the 10 Oct 2026 flight tests. Results and the full write-up:
[`docs/flight_tests/2026-10-10/`](../../docs/flight_tests/2026-10-10/).

## Pipeline

All scripts read and write files in the **current folder**, named `tN_*` per flight (t1, t2, …).

| Step | Script | Input → output |
|---|---|---|
| 1 | `extract_bag.py <bag> tN_bag.npz [clock_offset_s]` | ROS 2 bag → OpenVINS pose (`/ov_msckf/poseimu`, with covariance) + camera IMU (`/camera/gyro_accel/sample`) |
| 2 | `extract_bin.py <log.bin> tN_log.npz` | ArduPilot DataFlash log → EKF3 position/velocity (`XKF1`), GPS + accuracy, attitude, IMU, vibration, baro, events, messages |
| 3 | `analyze_flight.py tN` | → `tN_metrics.txt/.json`, `tN_report.png`, `tN_series.npz` (per-flight metrics, see below) |
| 4 | `image_metrics.py <bag> tN_img.npy [clock_offset_s] [step]` | → per-frame image quality (brightness, saturation, sharpness, FAST corners, optical flow, track ratio) |
| 5 | `diagnose_conditions.py` | all flights → 5 s windows: drift vs conditions (`conditions_report.txt`, `conditions_vs_drift.png`) |
| 6 | `extract_frames.py out.png name:bag:offset:t:label …` | camera frames from chosen windows (e.g. best vs worst) |
| 7 | `make_infographic.py` | 5 plain-language summary images (headless Firefox; output dir via `INFOGRAPHIC_OUT`) |

`clock_offset_s` is only for bags recorded while the Jetson clock was wrong (e.g. 1970):
real time = bag time + offset.

## What `analyze_flight.py` does

1. **Time sync**: cross-correlates the low-passed |gyro| of the camera IMU and the flight-controller IMU
   (rotation-invariant, so mounting doesn't matter). Searches ±3 s, so the bag clock must be roughly right.
2. **Mounting**: a Kabsch fit of the two gyro streams gives the camera-IMU → FC-body rotation.
3. **Frame link**: VIO world → ENU heading from FC attitude × mounting × VIO attitude, averaged before
   take-off. VIO is **anchored at arming** (its position at arming is set to the reference position).
   This is how it would be used live, so its error is the honest one.
4. **Metrics** (reference = EKF3 `XKF1`, GPS+IMU+baro):
   ATE anchored (and best-fit 4-DoF for comparison) · true drift % (end error / *reference* path length) ·
   scale (path ratio; early-segment sim3) · per-axis error · time/distance until error > 2/5/10 m ·
   vibration (VIBE-style RMS of > 5 Hz accel, FC and camera IMU) · heading drift · output rate · share of
   errors inside OpenVINS's own 3σ. Reports and clips to the part of the flight OpenVINS actually covered.

## Dependencies

Python 3 with numpy, scipy, matplotlib, opencv-python, pymavlink; plus ROS 2 `rclpy` + message
packages for steps 1, 4 and 6 (`source /opt/ros/<distro>/setup.bash`; Jazzy reads the Humble bags
fine). Bags are read directly from the sqlite `.db3` (no `ros2 bag` needed).

## Known limits

- Reference accuracy is that of the GPS (2–5 m here, worse during "GPS Glitch"), not cm-level.
- The ±3 s sync search assumes the Jetson clock was synced; otherwise pass `clock_offset_s`.
- `make_infographic.py` contains the 10 Oct wording and numbers; treat it as a template.
