# VPS Pipeline Overview — Handoff Doc

This document explains the whole Visual Positioning System (VPS) pipeline from the ground up: what problem it solves, how data flows through it, what has been built and tested, what is confirmed working, and what is still open. It's written for whoever picks this project up next, assuming no prior context beyond general ROS 2 familiarity.

If you only read one section before touching anything, read **"What still needs work"** — it lists the traps that already bit us once.

---

## 1. What problem is this solving?

A drone normally knows where it is using GPS. Indoors, under bridges, near buildings, or in any GPS-denied environment, that stops working. A **Visual Positioning System (VPS)** replaces GPS by estimating the drone's position and motion purely from a camera + IMU, and feeding that estimate into the flight controller instead of GPS.

This project benchmarks two ways of doing that on a **Jetson Orin Nano Super** paired with an **Orbbec Gemini 335L** stereo IR camera:

1. **OpenVINS** (`ov_msckf`) — a stereo-inertial MSCKF (an Extended Kalman Filter variant). This is the one that has actually been tested end-to-end so far.
2. **Isaac ROS cuVSLAM** — NVIDIA's GPU-accelerated visual SLAM. Wired up (launch file exists) but not benchmarked as thoroughly as OpenVINS.

Both approaches take the same raw inputs — stereo IR images + IMU — and produce the same kind of output: a continuously updated 3D pose (position + orientation) of the drone.

---

## 2. Hardware

| Component | Role |
|---|---|
| Jetson Orin Nano Super (JetPack 6.0) | Onboard compute, runs ROS 2 Jazzy |
| Orbbec Gemini 335L | Stereo IR camera + built-in accelerometer/gyro |
| Flight controller (ArduPilot, via MAVROS) | Consumes the final pose estimate as "vision position" input |

**Before every test/flight**, lock the Jetson's clock/power governor — this isn't optional, we've seen frame drops and scheduling jitter without it:
```bash
sudo nvpmodel -m 0
sudo jetson_clocks
```

---

## 3. The data flow, end to end

```
Gemini 335L camera driver (orbbec_camera)
   │
   ├── /camera/left_ir/image_raw  ──┐
   ├── /camera/right_ir/image_raw ──┤──► [optional] vps_bridge's ir_preprocessor.py
   │                                 │        (median blur + histogram equalization)
   └── /camera/gyro_accel/sample     │        → /camera/left_ir/image_filtered, /camera/right_ir/image_filtered
                                     │
                                     ▼
                    ┌────────────────────────────────┐
                    │   VIO / VSLAM estimator          │
                    │   (pick ONE of these two)         │
                    │                                    │
                    │  OpenVINS (ov_msckf)                │
                    │    → /ov_msckf/poseimu               │
                    │                                       │
                    │  Isaac ROS cuVSLAM                    │
                    │    → /visual_slam/tracking/odometry   │
                    └───────────────┬────────────────────┘
                                    ▼
                    vps_bridge_node (C++, ros2_ws/src/vps_bridge)
                    translates the pose into MAVROS's expected topic
                                    ▼
                    /mavros/vision_pose/pose
                                    ▼
                    ArduPilot flight controller EKF
                    (uses this instead of / alongside GPS)
```

**Important gap to know about:** `vps_bridge_node.cpp` currently only subscribes to cuVSLAM's `/visual_slam/tracking/odometry` topic. It does **not** yet subscribe to OpenVINS's `/ov_msckf/poseimu`. Since almost all of our real validation testing so far has been done with OpenVINS, **the pose estimates we've validated are not currently wired into MAVROS/the flight controller.** This is one of the first things that needs fixing before a real closed-loop flight test — see Section 6.

---

## 4. Calibration — what Kalibr produced and what each file means

We used [Kalibr](https://github.com/ethz-asl/kalibr) to calibrate the stereo camera and the camera-to-IMU relationship. Results live in `config/`:

- **`kalibr_imucam_chain.yaml`** — for each camera (cam0 = left, cam1 = right):
  - `intrinsics`, `distortion_coeffs` — standard lens calibration (focal length, distortion).
  - **`T_cam_imu`** — the 4x4 transform from camera frame to IMU frame. **This is the value that caused our big bug (Section 5).** It has to correctly express that the camera's optical frame (Z pointing forward, X right, Y down) and the IMU's body frame (X forward, Y left, Z up) are rotated 90° relative to each other. Get this wrong and the estimator confuses "flying forward" with "flying up/down."

- **`kalibr_imu_chain.yaml`** — IMU intrinsics: accelerometer/gyro noise density, random walk (used by the estimator's noise model), and `T_i_b` (IMU-to-body transform, currently identity — i.e. we're assuming the IMU is physically mounted aligned with the drone body frame). The `Tw`, `Ta`, `Tg`, `R_IMUtoGYRO`, `R_IMUtoACC` matrices are IMU axis-misalignment/scale corrections from Kalibr's IMU calibration — currently all identity/zero, meaning **no IMU intrinsic misalignment calibration has been applied yet**, only the nominal Kalibr defaults. This is a candidate area to revisit if drift shows up again after the camera-IMU fix.

- Raw calibration recordings live in `~/calib_recordings/`: `stereo_calib.bag` (the recording used), `stereo_calib-camchain.yaml` / `stereo_calib-report-cam.pdf` / `stereo_calib-results-cam.txt` (Kalibr's output report — worth reviewing for reprojection error before trusting the intrinsics), plus IMU-camera calibration folders `imu_ir_calib/`, `imu_ir_calib_full/`, `calib_stereo_ir/`.

---

## 5. The bug we hit, and the fix

### Symptom
The first real flight test (~90 seconds, ~150m flight path) produced a trajectory estimate that drifted **~107 meters from where it should have ended up — 72% of the total path length.** Altitude alone swung from +4m to -76m over the course of the flight. This wasn't a sudden glitch; it built up steadily over the whole flight.

### Root cause
`T_cam_imu` in `kalibr_imucam_chain.yaml` was effectively an **identity rotation** — i.e., it told the estimator "the camera and the IMU point the same direction." They don't: the camera's optical frame has Z pointing forward, while the IMU's body frame has X pointing forward. With the wrong transform, when the drone flew forward, the estimator's math interpreted that motion as happening along the wrong axis — effectively reading "forward motion" as "vertical motion," which is exactly the altitude-swing symptom we saw.

### The fix
A corrected rotation matrix was written into `T_cam_imu` for both cam0 and cam1 (see `config/kalibr_imucam_chain.yaml` — it's the version currently in the repo). Concretely, instead of an identity matrix, it maps camera axes onto IMU axes correctly, e.g. for cam0:
```yaml
T_cam_imu:
  - [0.0, -1.0,  0.0,  0.004181723256594681]
  - [0.0,  0.0, -1.0, -0.0015170671986489532]
  - [1.0,  0.0,  0.0, -0.005156202885411419]
  - [0.0,  0.0,  0.0,  1.0]
```

### Current validation status — read this carefully
The fix is **applied and looks empirically correct from bench testing**, but is **not yet formally validated with real flight data**, and the project's own `docs/KNOWN_ISSUES.md` deliberately says so. Two reasons:

1. The original 72%-drift flight bag only recorded OpenVINS's *output* topics (`/odomimu`, `/poseimu`, `/pathimu`), not the raw camera/IMU input. That means we **cannot reprocess the original bad flight** with the fix to directly prove the improvement on that exact flight — we can only test fresh.
2. All post-fix testing done so far (see numbers below) was **hand-carried motion on a laptop**, not an actual UAV flight, and it still only recorded `/ov_msckf/poseimu` rather than raw sensor topics.

So: strong evidence the fix works, but not yet the kind of proof the project standard requires. Don't mark this "resolved" in any doc without a real dynamic/flight test that also records raw topics.

---

## 6. What's confirmed working (with numbers)

These are real results from `~/calib_recordings/compute_drift.py`, run against post-fix bag recordings of `/ov_msckf/poseimu`:

| Test | Duration | Path length | Endpoint drift | Drift as % of path |
|---|---|---|---|---|
| `stereo_static_drift_test` (camera stationary) | 102.9s | ~0m | 0.013 cm | n/a (sanity check) |
| `openvins_laptop_final_test` (hand-carried motion) | 89.8s | 9.90 m | 12.92 cm | **1.31%** |
| `openvins_laptop_final_test2` (hand-carried motion) | 152.9s | 28.69 m | 34.56 cm | **1.21%** |

Compare that to the pre-fix flight's **72%** drift — this is the evidence the axis-mismatch fix is doing its job. The stationary test also confirms the estimator doesn't drift meaningfully when nothing is moving, which rules out a bias/noise problem in the estimator's rest state.

**Working / validated pieces of the pipeline:**
- Stereo camera + IMU driver bring-up (Orbbec Gemini 335L via `orbbec_camera`).
- Stereo intrinsic + camera-IMU extrinsic calibration via Kalibr (see calibration report in `calib_recordings/` before trusting a new calibration — check reprojection error).
- OpenVINS running stereo-inertial MSCKF against the corrected extrinsics, producing low-drift pose estimates in bench conditions.
- `compute_drift.py` as an analysis tool — it correctly parses `/ov_msckf/poseimu` from a bag, computes endpoint drift, path length, drift %, and max excursion.
- Jetson clock-lock procedure prevents frame-drop/jitter issues.

---

## 7. What still needs work — read before starting anything

This is the honest gap list. Don't assume any of these are done just because related pieces work.

1. **No real dynamic/flight validation yet.** Everything good above was bench-tested by hand-carrying the camera. The actual goal right now is a **real dynamic test** (ideally an actual flight, or at minimum realistic dynamic motion) that also **records raw topics** (`/camera/left_ir/image_raw`, `/camera/right_ir/image_raw`, `/camera/gyro_accel/sample`), not just the estimator's output — so that if something looks wrong, we can actually debug it by reprocessing, instead of getting stuck the way we did with the original 72%-drift flight.

2. ~~`vps_bridge_node` isn't wired to OpenVINS.~~ **Done.** It now subscribes to both `/ov_msckf/poseimu` (OpenVINS) and `/visual_slam/tracking/odometry` (cuVSLAM), forwarding whichever is active to `/mavros/vision_pose/pose`. It also now sanity-checks every pose before forwarding it (rejects implausible instantaneous velocity or a covariance spike, tracked separately per source) — this directly addresses the cuVSLAM tracking-spike risk in #4 below, since a spike now gets dropped instead of reaching the flight controller. Verified against a replayed real bag (clean data passes through untouched) and a synthetic ~495 m/s injected jump (correctly rejected, never reached the output topic). **This code lives in `~/ros2_ws/src/vps_bridge`, which is not yet under git — it needs to be added to this repo (or its own repo) before it counts as "in" the project for real.**

3. ~~`scripts/flight_analysis.py` is a stub, not a real analyzer.~~ **Done.** The stub has been removed and replaced with `scripts/compute_drift.py` (the real, working drift-analysis logic, ported in from `~/calib_recordings/compute_drift.py`). `CMakeLists.txt`, `package.xml`, and the README's Post-Flight Diagnostics section were updated to match. Verified by running it against `openvins_laptop_final_test` both from source and from the `colcon`-installed copy — output is identical (12.9194 cm / 1.305% drift), matching the numbers in Section 6.

4. **cuVSLAM has an unresolved tracking-spike issue.** During fast/jerky motion, cuVSLAM occasionally produces a single-frame catastrophic pose jump, and its `vo_state`/tracking-status flag does *not* reliably flag it. No mitigation is implemented yet — the plan on file is a downstream sanity check on instantaneous velocity/covariance before trusting a pose update. Don't treat cuVSLAM as flight-ready until this is addressed, if it's the path we go down instead of/alongside OpenVINS.

5. **IMU intrinsic calibration (`kalibr_imu_chain.yaml`) is still at defaults** — `Tw`, `Ta`, `Tg`, `R_IMUtoGYRO`, `R_IMUtoACC` are identity/zero, meaning no IMU axis-misalignment/scale correction from Kalibr has actually been applied. Worth a second calibration pass if drift reappears after the camera-IMU fix is validated, so we don't confuse a second bug with the first.

6. **Jetson clock resets to epoch (1970) on boot without NTP**, producing bag folders with bogus timestamps like `flight_19700101_*`. Install/enable `chrony` on the Jetson if this hasn't been done, especially before any test whose timestamps matter for analysis.

7. **Some calibration recordings are root-owned** (`stereo_calib.bag`, `stereo_calib-report-cam.pdf`, `stereo_calib-results-cam.txt` in `~/calib_recordings` are owned by `root`, likely from a `sudo`-launched camera driver). Not a pipeline bug, but worth normalizing so future scripted access doesn't hit permission errors.

8. **General hygiene gotchas already learned the hard way** (keep doing these):
   - Never run a manual `static_transform_publisher` for a frame the camera driver already publishes — creates a two-parent TF conflict that looks like a tracking bug but isn't.
   - Always fully restart both the camera driver and the VIO/VSLAM node between test runs — the pose graph persists in memory for the life of the node, so a "fresh" test without a restart can silently inherit corrupted state from the previous run.

---

## 8. How to reproduce things

**Analyze drift from a bag** (works against any bag containing `/ov_msckf/poseimu`):
```bash
python3 ~/calib_recordings/compute_drift.py <bag_path> [edge_window_sec]
# edge_window_sec: average this many seconds at start/end (use for closed-loop tests; 0/default = static mode)
```

**Build the repo:**
```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

**Launch cuVSLAM** (if going down that path):
```bash
ros2 launch vps_uav_benchmarking cuvslam_gemini335l.launch.py
```

---

## 9. Suggested order of work

1. ~~Wire OpenVINS's `/ov_msckf/poseimu` into `vps_bridge_node`.~~ **Done.** It now subscribes to both `/ov_msckf/poseimu` (OpenVINS) and `/visual_slam/tracking/odometry` (cuVSLAM), and both are sanity-checked (implausible velocity / covariance spike) before being forwarded to `/mavros/vision_pose/pose` — see `~/ros2_ws/src/vps_bridge`. This package is not yet in this git repo; it will be added separately.
2. ~~Port `compute_drift.py` into the repo.~~ **Done** — see `scripts/compute_drift.py`.
3. Set up a test that records **raw topics**, not just estimator output, so results are debuggable and reprocessable.
4. Run a real dynamic/flight test with raw topics recorded, then re-run `compute_drift.py` against it. Only then update `docs/KNOWN_ISSUES.md` to mark the axis-mismatch fix as validated.
5. Revisit IMU intrinsic calibration if drift shows up again after that test.
