# Known Issues & Debugging Log

## 1. cuVSLAM: occasional tracking-spike failures during fast motion

**Status:** Understood, not yet mitigated. Fast/jerky camera motion occasionally
triggers a single-frame catastrophic pose jump; `vo_state` does not reliably
flag it. Mitigation planned: downstream velocity/covariance sanity-check.

## 2. High closed-loop drift — the `T_cam_imu` "axis fix" was itself wrong

**Status: root cause corrected 2026-09-06; new fix applied on branch
`drift-test-harness` (commit `4559cb2`), replay-validated, not yet flight-validated.**

### History (what we believed before)
The first flight (~90 s, ~150 m) drifted ~107 m (72 % of path), altitude
swinging +4 m to -76 m. This was blamed on `T_cam_imu` in
`kalibr_imucam_chain.yaml` being "effectively identity", and a hand-built
90-degree rotation was written in as the fix:
```yaml
cam0.T_cam_imu:            # the WRONG value that shipped 78409db..4559cb2^
  - [0.0, -1.0,  0.0, ...]
  - [0.0,  0.0, -1.0, ...]
  - [1.0,  0.0,  0.0, ...]
```

### What's actually true (replay diagnosis, 2026-09-06)
That 90-degree rotation is **wrong for the pipeline as it runs**. OpenVINS
is fed the Gemini 335L's **built-in IMU** (`/camera/gyro_accel/sample`),
which sits inside the camera body and is **near-aligned with the IR
optical frame by construction**. OpenVINS's own stock config for this exact
sensor (`ov_msckf/config/orbbec_335l_stereo_ir`) ships a **near-identity**
`T_cam_imu` — that is the correct value here. Injecting 90 degrees makes the
filter read forward motion as vertical, i.e. it *creates* the Z-swing
symptom it was supposed to cure.

Replaying `stage1_axes` (gentle 100 s handheld bag) through OpenVINS:

| `T_cam_imu` rotation | drift (`/ov_msckf/poseimu`) |
|---|---|
| hand-built 90 deg (old repo value) | **43.7 %** (4.2 m / 9.6 m) |
| stock `orbbec_335l_stereo_ir` near-identity | **1.15 %** (0.10 m / 9.1 m) |

EuRoC `V1_01_easy` on the same OpenVINS build + replay harness gives 1.4 %,
so the build and the tooling are fine — the calibration was the problem.
The bad value is motion-independent: even a tripod-static bag drifts the
same way with the 90-degree rotation.

### Why the old "1.2–1.3 %" bench numbers still looked OK
The raw Kalibr result that the 90-degree matrix was derived from
(`~/ros2_ws/kalibr_imu-camchain-imucam.yaml`) was calibrated against the
**color camera** (640x480, fx~543, topic `/camera/image_raw`) and the
**Pixhawk IMU** (`/mavros/imu/data`) — a different camera *and* a different
IMU from what the pipeline runs (IR stereo 848x480 + Orbbec built-in IMU).
A ~90-degree camera/IMU rotation is plausible for the Pixhawk IMU, so the
laptop hand-carry bench tests (which reported 1.31 % / 1.21 %) most likely
used that IMU. Those numbers do not transfer to the Orbbec-IMU pipeline.

### The fix (commit `4559cb2`, branch `drift-test-harness`)
Both cam0/cam1 `T_cam_imu` rotation blocks replaced with the near-identity
values from stock `orbbec_335l_stereo_ir`. Translations, intrinsics,
`rostopic`, and `timeshift_cam_imu` unchanged.

### Still outstanding
* **A real Kalibr calibration of the actual rig** — IR stereo + the Orbbec
  *built-in* IMU. The file in the repo is now a mix of stock rotation +
  repo translations + IR intrinsics; it is a stopgap, not a proper
  calibration.
* Flight validation. The `stage1_axes` result is a bag replay, not a
  UAV flight. Run `docs/TEST_PLAN.md` Stages 1–3 with `scripts/record_bag.sh`
  (raw topics) before marking this resolved.
* Verify against a photo of the true camera mounting orientation.

## 3. Hardware Clock Drift

* **Symptom:** Frame drops and thread scheduling latencies during
  high-frequency execution.
* **Fix:** Lock Jetson performance governor:
  `sudo nvpmodel -m 0 && sudo jetson_clocks`

## 4. Bag-replay diagnosis was unreliable until these were fixed (2026-09-06)

Every replay drift number produced before 2026-09-06 is suspect. Four
independent problems, all in the *measurement*, not the estimator:

* **Zombie estimator nodes.** `kill -9` on the `ros2 run` wrapper leaves
  `run_subscribe_msckf` alive. Multiple diverged nodes then co-publish
  `/ov_msckf/odomimu`, and `ros2 bag record` captures the union — this is
  where the "7 km / NaN / 82 %" figures came from. Kill with
  `pkill -f run_subscribe_msckf` and verify with `pgrep` before each run.
* **No namespace when launched with bare `ros2 run`.** `run_subscribe_msckf`
  with no namespace publishes `/odomimu`, not `/ov_msckf/odomimu`, and it
  only publishes odometry at all when it already sees a subscriber. Result:
  the recorder captured nothing. Launch via
  `ros2 launch vps_uav_benchmarking openvins_gemini335l.launch.py`
  (which sets `namespace:=ov_msckf`).
* **DDS discovery on this multi-NIC Jetson.** The box has ~7 interfaces
  (`l4tbr0`, `wlP1p1s0`, `usb0/1`, `docker0`, ...). With
  `ROS_LOCALHOST_ONLY` unset, cross-process `ros2 node list` / graph
  discovery silently fails. Export `ROS_LOCALHOST_ONLY=1` for every replay
  process.
* **Wrong analyzer topic.** `scripts/flight_analysis.py` defaults to
  `/ov_msckf/odomimu`; the historical bench numbers and `compute_drift.py`
  use `/ov_msckf/poseimu`. Pass `--odom-topic /ov_msckf/poseimu` to
  `flight_analysis.py` for apples-to-apples.
* **Also don't chase these — they look like bugs but aren't:**
  `[init]: not enough feats to compute disp: 0,N` prints `0` for the first
  number even on a healthy run (EuRoC shows it too); and OpenVINS prints no
  per-update `[UPDATE-MSCKF]` lines at `verbosity: INFO`. Judge a run by the
  drift %, not those.

## 5. OpenVINS `run_subscribe_msckf` aborts on shutdown (aarch64)

* **Symptom:** `pthread_mutex_lock` assertion / `SIGABRT` (exit -6) every
  time the node is stopped with SIGINT. The estimate is already complete
  when it happens, so drift numbers are unaffected, but the process never
  exits cleanly and `visualize_final()` may not run.
* **Root cause:** `use_multi_threading_subs` is hardcoded `true`, so each
  IMU callback detaches a thread that runs the VIO update inside
  `VioManager`. On SIGINT the executor returns on the main thread, which
  then destroys the global `viz`/`sys` objects (and their mutexes) while a
  detached update thread is still mid-update.
* **Fix (local):** `ov_msckf` patched to `std::_Exit()` after
  `rclcpp::shutdown()` instead of unwinding — branch `jetson-local-patches`
  in the `open_vins` checkout, commit `da5e46a`, `ov_msckf` rebuilt.
  Verified: 3/3 clean exits, drift unchanged. This is a Jetson-local
  stopgap; the real fix upstream is joinable subscriber threads.
