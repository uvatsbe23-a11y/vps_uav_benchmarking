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
| Jetson Orin Nano Super (`vajra`, JetPack 6) | Onboard compute, runs ROS 2 **Humble** |
| Orbbec Gemini 335L | Stereo IR camera + built-in accelerometer/gyro |
| Flight controller: CubeOrange+, ArduCopter 4.6.3 | Flies the drone. In the 10 Oct tests it flew on GPS and OpenVINS was only recorded; later it is meant to take the VPS pose as "external navigation" via MAVROS |

**Before every test/flight**, set full power and lock the clocks. This isn't optional: in the
throttled mode OpenVINS could not keep up live and diverged (`KNOWN_ISSUES.md` #3):
```bash
sudo nvpmodel -m 2      # MAXN_SUPER. NOT -m 0 (= 15 W throttled mode on this board)
sudo jetson_clocks      # every boot; flight_start does this
```
Also sync the Jetson clock from the ground PC after every power-on (`scripts/flight/sync_jetson_clock.sh`;
the RTC has no battery, `KNOWN_ISSUES.md` #9).

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
                    vps_bridge_node (C++, repo: ros2_packages/vps_bridge)
                    translates the pose into MAVROS's expected topic
                                    ▼
                    /mavros/vision_pose/pose
                                    ▼
                    ArduPilot flight controller EKF
                    (uses this instead of / alongside GPS)
```

**Status of the last two boxes:** `vps_bridge_node` subscribes to both OpenVINS (`/ov_msckf/poseimu`)
and cuVSLAM, sanity-filters the pose and forwards it to `/mavros/vision_pose/pose` (see §7). It has
**not been used in any flight yet**: in the 10 Oct tests OpenVINS ran on the raw driver output (no
`ir_preprocessor`) and was only *recorded*; the drone flew on GPS. Feeding the flight controller
waits until OpenVINS is reliable (drift #6, stops #7 in `KNOWN_ISSUES.md`).

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

### Current validation status (updated 24 Oct 2026)
1. ~~Flight test with raw topics~~ **Done 10 Oct 2026** (4 flights). No frame/axis error: attitude
   agreed with the flight controller to ≤ 4.4° tilt and ≤ 15° heading, and early scale was
   0.85–1.09. The `T_cam_imu` fix holds. Position still drifts (a separate problem, §6–§7).
2. A proper Kalibr calibration of the actual IR-stereo + Orbbec-built-in-IMU rig is **still owed**.
3. ~~Mounting photo~~: mounting measured from flight data instead: camera facing down, 6–8° tilt.

---

## 6. What's confirmed working (with numbers)

### Flight-tested, 10 Oct 2026 (details: [`flight_tests/2026-10-10/`](flight_tests/2026-10-10/))

Four automatic GPS missions, camera facing down, compared against the flight controller's EKF
(GPS, accurate to ~2–5 m):

| Flight | Profile | Error passes 10 m after | Average error (anchored) | OpenVINS ran |
|---|---|---|---|---|
| 1 | 4.6 m high, up to 10.4 m/s | 58 m | 182 m (ended 387 m off, 95 %) | full flight |
| 2 | 9 m, ≤ 4 m/s, straight | 101 m | 7.8 m | stopped at 78 s of 213 |
| 3 | 9 m, ≤ 7 m/s, zig-zag + hovers | 95 m | 32.6 m | stopped at 91 s of 182 |
| 4 | 9 m, ≤ 7 m/s, out and back | 91 m | 7.6 m (15.5 % at landing) | full (last 4 s missing) |

What works: recording (30 Hz stereo, 493 Hz IMU, no gaps), attitude, early scale, real-time
performance on MAXN_SUPER. What doesn't: position drift after 50–100 m, height, over-confident
covariance, and mid-flight stops (`KNOWN_ISSUES.md` #6–#8).

### Replay-validated (2026-09-06), with the `4559cb2` near-identity `T_cam_imu`

| Test | Source | Drift (`/ov_msckf/poseimu`) |
|---|---|---|
| `stage1_axes` (gentle 100 s handheld) | bag replay | **1.15 %** (0.10 m / 9.1 m) |
| EuRoC `V1_01_easy` (reference dataset) | bag replay | 1.4 % (sanity: build + harness OK) |
| same `stage1_axes` with the *old* 90-deg matrix | bag replay | 43.7 %: the bug |

**Historical laptop bench numbers (1.21–1.31 %) are suspect**: they almost certainly used the
Pixhawk IMU, not the Orbbec built-in IMU the pipeline runs.

**Working pieces:** camera + IMU driver bring-up; OpenVINS build (`v2.7-20-g6948812` + the patches in
`third_party/open_vins/`); bag-replay harness (`scripts/replay_diag/`); the `flight_*` field routine;
the flight analysis pipeline (`scripts/analysis/`); Jetson clock restore and crash dumps.

---

## 7. What still needs work: read before starting anything

The ordered plan with checklists is in [`HANDOFF_2026-10-10.md`](HANDOFF_2026-10-10.md). In short:

1. **Position drift in flight** (`KNOWN_ISSUES.md` #6): image motion too large when fast and low, and
   low-texture ground. Try on the recorded bags first: process every frame (`track_frequency` 21 → ≥ 30,
   #11), contrast boost, wider KLT search. Then fly a controlled ladder (hover → slow line → faster →
   other heights).
2. **OpenVINS stopping mid-flight** (#7): replay flights 2/3/4 to see if it is deterministic; the
   `flight_*` scripts now log it and core dumps are on.
3. **Calibration** (#2, #8): Kalibr (IR stereo + Orbbec built-in IMU) and Allan variance for the IMU
   noise model. `kalibr_imu_chain.yaml` intrinsics (`Tw`, `Ta`, `Tg`, `R_IMUtoGYRO`, `R_IMUtoACC`)
   are still identity/zero, and `rostopic` must stay `/camera/gyro_accel/sample`.
4. **Hardware:** RTC battery, secure the Jetson power connector (it lost power once while handled),
   soft-mount the whole camera, downward rangefinder for height; RTK GPS as the test reference.
5. **Heading:** fuse the flight controller's compass heading (read via MAVROS) to correct OpenVINS
   yaw drift; later send the pose to the FC EKF as external navigation (via `vps_bridge`).
6. **`vps_bridge`** (now in `ros2_packages/`) is untested in flight, and its sanity filter relies on the
   covariance (#8).
7. **cuVSLAM tracking spikes** (#1): only relevant if cuVSLAM is pursued; it has not been flown.
8. **General hygiene gotchas already learned the hard way** (keep doing these):
   - `export ROS_LOCALHOST_ONLY=1` in every terminal on the Jetson (the `flight_*` scripts do).
   - Never run a manual `static_transform_publisher` for a frame the camera driver already publishes
     (two-parent TF conflict that looks like a tracking bug).
   - Fully restart the camera driver and OpenVINS between runs (`flight_stop` / `flight_start`); the
     filter state persists for the life of the node.
   - Some calibration recordings in `~/calib_recordings` are root-owned; normalise permissions.

---

## 8. How to reproduce things

**Run a test:** `flight_start` → initialise → `flight_record` → `flight_stop` (see the README).

**Analyse a flight against the flight-controller log:** see [`scripts/analysis/README.md`](../scripts/analysis/README.md).

**Quick drift from a bag** (any bag with `/ov_msckf/poseimu`; closed loops):
```bash
python3 scripts/compute_drift.py <bag_path> [edge_window_sec]
```

**Replay a bag through OpenVINS with a different config:** `scripts/replay_diag/` (see its README).

**Build the repo:**
```bash
cd ~/ros2_ws && colcon build --symlink-install && source install/setup.bash
```

**Launch cuVSLAM** (if going down that path): `ros2 launch vps_uav_benchmarking cuvslam_gemini335l.launch.py`

---

## 9. Suggested order of work

Superseded by the ordered checklist in [`HANDOFF_2026-10-10.md`](HANDOFF_2026-10-10.md) ("NEXT STEPS"
and "NEXT FLIGHT DAY CHECKLIST"). Done so far: `vps_bridge` wired to OpenVINS (now in the repo),
`compute_drift.py` ported, raw-topic recording, `T_cam_imu` corrected (2026-09-06) and confirmed in
flight (2026-10-10), field routine and clock fixes (2026-10-10), all code on GitHub (2026-10-24).
