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
  - **`T_cam_imu`** — the 4x4 transform from camera frame to IMU frame. **This is the value that caused our big bug (Section 5)** — twice: once by being wrong, and once by being "fixed" in the wrong direction. It expresses the rotation between the camera's optical frame and **whichever IMU OpenVINS is actually fed**. The pipeline feeds the Gemini 335L's *built-in* IMU (`/camera/gyro_accel/sample`), which is near-aligned with the IR optical frame by construction — so `T_cam_imu` here should be a **near-identity** rotation (see OpenVINS's stock `orbbec_335l_stereo_ir` config). A large rotation is only correct if you calibrate against a *separate* IMU mounted at some angle (e.g. the Pixhawk's `/mavros/imu/data`). Get this wrong and the estimator confuses "flying forward" with "flying up/down."
  - **Do not trust `~/ros2_ws/kalibr_imu-camchain-imucam.yaml`** — that Kalibr run used the color camera (640x480, fx~543, `/camera/image_raw`) and the Pixhawk IMU, neither of which the pipeline uses. The repo's `kalibr_imucam_chain.yaml` currently borrows the stock IR rotation + intrinsics; a real Kalibr pass on the IR-stereo + Orbbec-built-in-IMU rig is still owed.

- **`kalibr_imu_chain.yaml`** — IMU intrinsics: accelerometer/gyro noise density, random walk (used by the estimator's noise model), and `T_i_b` (IMU-to-body transform, currently identity — i.e. we're assuming the IMU is physically mounted aligned with the drone body frame). The `Tw`, `Ta`, `Tg`, `R_IMUtoGYRO`, `R_IMUtoACC` matrices are IMU axis-misalignment/scale corrections from Kalibr's IMU calibration — currently all identity/zero, meaning **no IMU intrinsic misalignment calibration has been applied yet**, only the nominal Kalibr defaults. This is a candidate area to revisit if drift shows up again after the camera-IMU fix.

- Raw calibration recordings live in `~/calib_recordings/`: `stereo_calib.bag` (the recording used), `stereo_calib-camchain.yaml` / `stereo_calib-report-cam.pdf` / `stereo_calib-results-cam.txt` (Kalibr's output report — worth reviewing for reprojection error before trusting the intrinsics), plus IMU-camera calibration folders `imu_ir_calib/`, `imu_ir_calib_full/`, `calib_stereo_ir/`.

---

## 5. The bug we hit, and the fix

### Symptom
The first real flight test (~90 seconds, ~150m flight path) produced a trajectory estimate that drifted **~107 meters from where it should have ended up — 72% of the total path length.** Altitude alone swung from +4m to -76m over the course of the flight. This wasn't a sudden glitch; it built up steadily over the whole flight.

### Root cause — corrected 2026-09-06 (the story has two layers)

**Layer 1 (what we thought):** `T_cam_imu` was an identity rotation and the
72 % flight needed a 90-degree correction. A hand-built 90-degree matrix was
written in as the fix (in the repo `78409db` .. `4559cb2^`):
```yaml
cam0.T_cam_imu:
  - [0.0, -1.0,  0.0, ...]
  - [0.0,  0.0, -1.0, ...]
  - [1.0,  0.0,  0.0, ...]
```

**Layer 2 (what replay diagnosis found):** that 90-degree matrix is *wrong*
for this pipeline and is the reason every subsequent Orbbec test drifted
20–50 %. OpenVINS is fed the Gemini 335L's **built-in IMU**
(`/camera/gyro_accel/sample`), which is physically near-aligned with the IR
optical frame — OpenVINS's stock `orbbec_335l_stereo_ir` config ships a
**near-identity** `T_cam_imu` for exactly this sensor. Feeding it a
90-degree rotation makes the filter interpret forward motion as vertical,
i.e. it re-creates the altitude-swing symptom. This is motion-independent:
even a tripod-static bag drifts with the bad matrix.

The original 90-degree value was derived from a Kalibr run
(`~/ros2_ws/kalibr_imu-camchain-imucam.yaml`) against the **color camera**
and the **Pixhawk IMU** (`/mavros/imu/data`) — a different camera and a
different IMU. A ~90-degree rotation is reasonable *there*; it does not
transfer to the IR + Orbbec-IMU pipeline.

### The fix (commit `4559cb2`, branch `drift-test-harness`)
Both cam0/cam1 `T_cam_imu` rotation blocks reverted to the near-identity
values from stock `orbbec_335l_stereo_ir` (translations, intrinsics,
`rostopic`, `timeshift_cam_imu` unchanged), e.g. cam0:
```yaml
T_cam_imu:
  - [ 0.99991, -0.00795, -0.01080,  0.004181723256594681]
  - [ 0.00793,  0.99997, -0.00142, -0.0015170671986489532]
  - [ 0.01081,  0.00133,  0.99994, -0.005156202885411419]
  - [ 0.0,      0.0,      0.0,       1.0]
```

### Evidence
Replaying the same `stage1_axes` bag through OpenVINS (drift from
`/ov_msckf/poseimu`):

| `T_cam_imu` rotation | drift | endpoint / path |
|---|---|---|
| hand-built 90 deg (old) | **43.7 %** | 4.2 m / 9.6 m |
| stock near-identity (new) | **1.15 %** | 0.10 m / 9.1 m |

EuRoC `V1_01_easy` on the same OpenVINS build + replay harness: 1.4 %.
So OpenVINS and the tooling are sound; the calibration was the fault.

### Current validation status — read this carefully
The new value is **replay-validated on `stage1_axes`**, not flight-validated.
Do not mark this "resolved" until:
1. A real dynamic/flight test through `docs/TEST_PLAN.md` Stages 1–3, with
   `scripts/record_bag.sh` capturing **raw** topics (so it can be reprocessed).
2. A proper Kalibr calibration of the actual IR-stereo + Orbbec-built-in-IMU
   rig replaces the current stock-borrowed values.
3. The camera mounting orientation is checked against a photo of the real rig.

---

## 6. What's confirmed working (with numbers)

**Replay-validated (2026-09-06), on this Jetson with the `4559cb2`
near-identity `T_cam_imu`:**

| Test | Source | Drift (`/ov_msckf/poseimu`) |
|---|---|---|
| `stage1_axes` (gentle 100 s handheld) | bag replay | **1.15 %** (0.10 m / 9.1 m) |
| EuRoC `V1_01_easy` (reference dataset) | bag replay | 1.4 % (sanity: build + harness OK) |
| same `stage1_axes` with the *old* 90-deg matrix | bag replay | 43.7 % — the bug |

**Historical bench numbers — treat as suspect.** These were run with
`~/calib_recordings/compute_drift.py` against hand-carried laptop bags and
were the basis for believing the 90-degree fix worked:

| Test | Duration | Path | Endpoint drift | % of path |
|---|---|---|---|---|
| `stereo_static_drift_test` (stationary) | 102.9 s | ~0 m | 0.013 cm | n/a |
| `openvins_laptop_final_test` (hand-carried) | 89.8 s | 9.90 m | 12.92 cm | 1.31 % |
| `openvins_laptop_final_test2` (hand-carried) | 152.9 s | 28.69 m | 34.56 cm | 1.21 % |

They almost certainly used the **Pixhawk IMU** (for which the 90-degree
matrix is reasonable), not the Orbbec built-in IMU the pipeline runs. They
do not validate the current pipeline. If those bags recorded raw topics,
re-run them through the fixed config to confirm.

**Working / validated pieces of the pipeline:**
- Stereo camera + IMU driver bring-up (Orbbec Gemini 335L via `orbbec_camera`).
- OpenVINS build (`v2.7-20-g6948812`) — proven correct on EuRoC (1.4 %) and on
  `stage1_axes` once the calibration was fixed.
- The bag-replay diagnostic harness on this Jetson (see `KNOWN_ISSUES.md` #4
  for the infra bugs it had to fix first).
- `compute_drift.py` / `flight_analysis.py --odom-topic /ov_msckf/poseimu` as
  analysis tools.
- Jetson clock-lock procedure prevents frame-drop/jitter issues.

**Not yet validated:** any camera-IMU extrinsic calibration for the *actual*
IR + Orbbec-IMU rig (the current one is stock-borrowed); any flight.

---

## 7. What still needs work — read before starting anything

This is the honest gap list. Don't assume any of these are done just because related pieces work.

1. **No real dynamic/flight validation yet.** The good result above (1.15 % on
   `stage1_axes`) is a **bag replay**, not a UAV flight. The actual goal is a
   **real dynamic test** (a flight, or at minimum realistic dynamic motion)
   that also **records raw topics** (`/camera/left_ir/image_raw`,
   `/camera/right_ir/image_raw`, `/camera/gyro_accel/sample`), not just the
   estimator's output — so a bad result can be debugged by reprocessing.

1b. **A proper Kalibr calibration is still owed.** `config/kalibr_imucam_chain.yaml`
   currently borrows the `T_cam_imu` rotation and IR intrinsics from OpenVINS's
   stock `orbbec_335l_stereo_ir` config (that's what commit `4559cb2` does). It
   works (~1 %), but it is not a calibration of *this* unit. Record a fresh
   Kalibr set — IR stereo + the Orbbec **built-in** IMU (`/camera/gyro_accel/sample`),
   not the Pixhawk — and check reprojection error before trusting it.

1c. **OpenVINS shutdown crash is patched locally, not upstream.** `ov_msckf`
   on this Jetson carries a local patch (`open_vins` branch `jetson-local-patches`,
   commit `da5e46a`) so `run_subscribe_msckf` exits cleanly instead of
   `SIGABRT`-ing on every SIGINT. A `colcon build` from a clean `open_vins`
   checkout loses it — see `KNOWN_ISSUES.md` #5.

2. ~~`vps_bridge_node` isn't wired to OpenVINS.~~ **Done.** It now subscribes to both `/ov_msckf/poseimu` (OpenVINS) and `/visual_slam/tracking/odometry` (cuVSLAM), forwarding whichever is active to `/mavros/vision_pose/pose`. It also now sanity-checks every pose before forwarding it (rejects implausible instantaneous velocity or a covariance spike, tracked separately per source) — this directly addresses the cuVSLAM tracking-spike risk in #4 below, since a spike now gets dropped instead of reaching the flight controller. Verified against a replayed real bag (clean data passes through untouched) and a synthetic ~495 m/s injected jump (correctly rejected, never reached the output topic). **This code lives in `~/ros2_ws/src/vps_bridge`, which is not yet under git — it needs to be added to this repo (or its own repo) before it counts as "in" the project for real.**

3. ~~`scripts/flight_analysis.py` is a stub, not a real analyzer.~~ **Done.** The stub has been removed and replaced with `scripts/compute_drift.py` (the real, working drift-analysis logic, ported in from `~/calib_recordings/compute_drift.py`). `CMakeLists.txt`, `package.xml`, and the README's Post-Flight Diagnostics section were updated to match. Verified by running it against `openvins_laptop_final_test` both from source and from the `colcon`-installed copy — output is identical (12.9194 cm / 1.305% drift), matching the numbers in Section 6.

4. **cuVSLAM has an unresolved tracking-spike issue.** During fast/jerky motion, cuVSLAM occasionally produces a single-frame catastrophic pose jump, and its `vo_state`/tracking-status flag does *not* reliably flag it. No mitigation is implemented yet — the plan on file is a downstream sanity check on instantaneous velocity/covariance before trusting a pose update. Don't treat cuVSLAM as flight-ready until this is addressed, if it's the path we go down instead of/alongside OpenVINS.

5. **IMU intrinsic calibration (`kalibr_imu_chain.yaml`) is still at defaults** — `Tw`, `Ta`, `Tg`, `R_IMUtoGYRO`, `R_IMUtoACC` are identity/zero, meaning no IMU axis-misalignment/scale correction from Kalibr has actually been applied. Fold this into the fresh calibration pass in item 1b — and note `rostopic` there must be `/camera/gyro_accel/sample` (the Orbbec built-in IMU), matching what OpenVINS is fed.

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
4. ~~Establish which `T_cam_imu` is correct.~~ **Done (2026-09-06)** — near-identity, for the Orbbec built-in IMU; commit `4559cb2`. `stage1_axes` replay drift 43.7 % -> 1.15 %. See `KNOWN_ISSUES.md` #2.
5. Record a fresh Kalibr calibration of the real IR-stereo + Orbbec-built-in-IMU rig (camera-IMU extrinsics **and** IMU intrinsics) to replace the stock-borrowed values.
6. Run a real dynamic/flight test with raw topics recorded, then re-run `compute_drift.py` against it. Only then update `docs/KNOWN_ISSUES.md` #2 to mark the axis fix flight-validated.
