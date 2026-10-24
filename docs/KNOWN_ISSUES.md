# Known Issues & Debugging Log

Updated 24 Oct 2026 after the first flight tests (10 Oct). Details of those flights:
[`flight_tests/2026-10-10/`](flight_tests/2026-10-10/). Next steps: [`HANDOFF_2026-10-10.md`](HANDOFF_2026-10-10.md).

| # | Issue | Status |
|---|---|---|
| 1 | cuVSLAM tracking spikes on fast motion | mitigation written (`vps_bridge` filter), untested in flight |
| 2 | `T_cam_imu` 90° "axis fix" was wrong | **fixed and flight-confirmed** (no frame error in flight) |
| 3 | Jetson power mode / clocks | fixed: use `nvpmodel -m 2` (**not** `-m 0`) |
| 4 | Bag-replay measurement bugs | fixed (2026-09-06) |
| 5 | OpenVINS crash/hang on shutdown | fixed (2nd patch, 2026-10-10); test under load pending |
| 6 | **Position drift in flight** (fast & low, low texture) | **open**: main problem |
| 7 | **OpenVINS stops publishing mid-flight** | **open**: cause unknown, now instrumented |
| 8 | OpenVINS covariance over-confident | **open** |
| 9 | Jetson clock resets to 1970 | fixed in software; RTC battery recommended |
| 10 | `check_setup.sh` false warnings | fixed |
| 11 | OpenVINS processes only every other frame (`track_frequency: 21`) | open (1-line change, test on replays first) |
| 12 | `compute_drift.py` understates drift when OpenVINS diverges | known; use `scripts/analysis/` |

---

## 1. cuVSLAM: occasional tracking-spike failures during fast motion

**Status:** Understood. Fast/jerky camera motion occasionally triggers a single-frame catastrophic
pose jump; `vo_state` does not reliably flag it. A downstream velocity/covariance sanity filter
exists in `ros2_packages/vps_bridge/` but has not been used in any flight. cuVSLAM itself has not
been flown. Note #8: the filter's covariance check relies on the estimator's covariance being honest.

## 2. High closed-loop drift: the `T_cam_imu` "axis fix" was itself wrong

**Status: fixed (commit `4559cb2` "Revert T_cam_imu…"), replay-validated 2026-09-06, and
confirmed in flight 2026-10-10:** OpenVINS's attitude agreed with the flight controller to
≤ 4.4° tilt and ≤ 15° heading in all 4 flights, and early scale was 0.85–1.09. That is no sign of a
frame/axis error. The remaining flight drift is a different problem (#6).

### History (what we believed before)
The first flight (~90 s, ~150 m) drifted ~107 m (72 % of path), altitude swinging +4 m to −76 m. This
was blamed on `T_cam_imu` in `kalibr_imucam_chain.yaml` being "effectively identity", and a
hand-built 90-degree rotation was written in as the fix:
```yaml
cam0.T_cam_imu:            # the WRONG value that shipped 78409db..4559cb2^
  - [0.0, -1.0,  0.0, ...]
  - [0.0,  0.0, -1.0, ...]
  - [1.0,  0.0,  0.0, ...]
```

### What's actually true (replay diagnosis, 2026-09-06)
OpenVINS is fed the Gemini 335L's **built-in IMU** (`/camera/gyro_accel/sample`), which sits inside
the camera body and is **near-aligned with the IR optical frame by construction**. OpenVINS's stock
config for this sensor (`orbbec_335l_stereo_ir`, copy in `third_party/open_vins/config/`) ships a
**near-identity** `T_cam_imu`, the correct value here. Injecting 90 degrees makes the filter read
forward motion as vertical, i.e. it *creates* the Z-swing symptom it was supposed to cure.

| `T_cam_imu` rotation (replay of `stage1_axes`) | drift (`/ov_msckf/poseimu`) |
|---|---|
| hand-built 90 deg (old repo value) | **43.7 %** (4.2 m / 9.6 m) |
| stock `orbbec_335l_stereo_ir` near-identity | **1.15 %** (0.10 m / 9.1 m) |

EuRoC `V1_01_easy` on the same build + harness gives 1.4 %, so the build and tooling are fine.

### Why the old "1.2–1.3 %" bench numbers looked OK
The raw Kalibr result behind the 90-degree matrix (`~/ros2_ws/kalibr_imu-camchain-imucam.yaml`)
was calibrated against the **color camera** and the **Pixhawk IMU** (`/mavros/imu/data`), a
different camera *and* IMU from the pipeline. Those bench numbers do not transfer.

### Still outstanding
* **A real Kalibr calibration of the actual rig** (IR stereo + Orbbec built-in IMU) plus Allan
  variance for the IMU noise (see #8). The current file is a stopgap mix of stock rotation, repo
  translations and IR intrinsics.
* ~~Mounting photo~~: mounting measured from the flights instead (gyro fit): camera **facing down,
  tilted 6–8°**. Tilting the whole camera does not change `T_cam_imu` (the IMU is inside it).

## 3. Jetson power mode and clocks

* **Symptom:** live OpenVINS diverged (~70 m) on handheld tests, while replaying the same bags
  offline gave 1.4–8.6 %. Cause: the board was in the **15 W mode (`nvpmodel -m 0`)**; OpenVINS
  could not keep up live.
* **Fix (every boot):** `sudo nvpmodel -m 2` (MAXN_SUPER; persists) and `sudo jetson_clocks` (does not
  persist). `flight_start` checks the mode and runs `jetson_clocks`. In flight, output latency stayed
  40–75 ms with no backlog, so compute is not a bottleneck in this mode.

## 4. Bag-replay diagnosis was unreliable until these were fixed (2026-09-06)

Every replay drift number produced before 2026-09-06 is suspect. All in the *measurement*:

* **Zombie estimator nodes.** `kill -9` on the `ros2 run` wrapper leaves `run_subscribe_msckf` alive;
  several diverged nodes then co-publish (the "7 km / NaN / 82 %" figures). Kill with
  `pkill -f run_subscribe_msckf` and check with `pgrep` before each run (the `flight_*` scripts track PIDs).
* **No namespace with bare `ros2 run`** → topics `/odomimu` instead of `/ov_msckf/odomimu`. Launch via
  `openvins_gemini335l.launch.py` (sets `namespace="ov_msckf"`).
* **DDS discovery on this multi-NIC Jetson** (~7 interfaces): export `ROS_LOCALHOST_ONLY=1` for every
  process (the `flight_*` scripts do).
* **Wrong analyzer topic.** `flight_analysis.py` defaults to `/ov_msckf/odomimu`; pass
  `--odom-topic /ov_msckf/poseimu` to match `compute_drift.py`.
* **Don't chase:** `[init]: not enough feats to compute disp: 0,N` and the absence of
  `[UPDATE-MSCKF]` lines at `verbosity: INFO` are normal.

## 5. OpenVINS `run_subscribe_msckf` crash/hang on shutdown (aarch64)

* **Symptom:** on SIGINT, `SIGABRT` (−6), segfault (−11), or hang past `ros2 launch`'s 5 s
  SIGINT→SIGTERM escalation.
* **Root cause:** `use_multi_threading_subs` is hardcoded `true`, so image/IMU callbacks run updates in
  detached threads, which race the main thread's teardown (`visualize_final`, `rclcpp::shutdown`).
* **Fix:** two local patches in [`third_party/open_vins/patches/`](../third_party/open_vins/patches/):
  `0001` `_Exit()` after shutdown (2026-09-06; launch logs later showed it was not enough), and
  `0002` **exit immediately on SIGINT/SIGTERM** (2026-10-10): stops in 0.1 s, 3/3 runs.
  **Still to test while OpenVINS is processing images.** A clean upstream checkout loses these patches.

## 6. Position drift in flight (open, the main problem)

10 Oct, 4 flights: about **10 m off after 50–100 m** (best 101 m, worst 58 m); flight 1 ended 387 m
off (95 % of a 406 m path). From 5 s window analysis:
* **Worst: image motion too large** (speed ÷ height too high). Flight 1 at 4.7 m / 9–10 m/s: features
  moved ~45 px between frames (10× the other flights), the only drop in tracking success.
* **Second: low-texture / low-contrast ground**, which drifts ~2.5× faster even when hovering.
* **No clear effect:** vibration at 9 m, brightness, compute.
* Provisional envelope: speed ÷ height ≤ ~0.35 s⁻¹. Height is the weakest axis.
* Next: process all frames (#11), contrast boost, wider KLT search, lidar height, compass heading,
  controlled flight ladder. See the handoff.

## 7. OpenVINS stops publishing mid-flight (open)

Flights 2, 3 and 4: all OpenVINS outputs stopped at the same instant (78 s, 91 s, 4 s before
disarm) while the camera and recorder continued. Latency was normal right up to the stop (not
overload). Hypotheses: (a) blocked console output to an SSH terminal whose Wi-Fi link dropped
(flights 2/3 stopped at their farthest points); (b) a crash: other runs in the launch logs show
spontaneous segfaults (−11), but every boot logged in "1970", so they could not be matched to flights.
**Now instrumented:** processes detached from SSH with per-flight logs (`flight_*`), core dumps to
`/var/crash/cores/`, clock fixed. Next step: replay flights 2/3/4 and see if it stops at the same moment.

## 8. OpenVINS covariance is over-confident (open)

True error was inside OpenVINS's own 3σ only 5–10 % of the time (≈99 % expected). Anything that weights
by that covariance (the `vps_bridge` sanity filter, an ArduPilot EKF fusing the vision pose) will trust
it too much. Likely cause: guessed IMU noise parameters and no proper calibration (#2). Fix: Allan
variance + Kalibr, then check consistency on replays.

## 9. Jetson clock resets to 1970 on every power-on

* **Cause:** the RTC has no backup battery; the `nvvrs-pseq-rtc` driver sets the clock to 1970 ~0.3 s
  *after* an early restore, and the old save timer then stored 1970.
* **Fix (2026-10-10):** [`scripts/jetson_system/`](../scripts/jetson_system/): re-apply the saved time after
  the RTC registers (udev → systemd service; udev `RUN+=` alone cannot set the clock), never save an older
  time, 20 s backstop. Verified across 2 reboots. The restored time lags by the downtime: run
  `scripts/flight/sync_jetson_clock.sh` on the ground PC before recording.
* **Recommended hardware fix:** an RTC coin cell on the carrier board.

## 10. `check_setup.sh` false warnings (fixed 2026-10-10)

The clock-offset check parsed `nanosec:` as seconds (`/sec:/` matched both) → offsets like
"1321243476 s"; and the topic check used the ROS daemon, whose stale list reported every topic
MISSING while all were streaming. Fixed (`--no-daemon`, anchored regex); a system-clock check was added.

## 11. OpenVINS uses only every other camera frame

`config/estimator_config.yaml` has `track_frequency: 21.0`. With the camera at 30 fps, frames are
accepted at ≤ 21 Hz → every other frame → the ~15 Hz output seen in all flights, and twice the image
motion between processed frames (#6). Raising it to ≥ 30 is a one-line change; a variant exists in
`scripts/replay_diag/cfg_klt_freq30/`. Test on the recorded flights before flying it (CPU load).

## 12. `compute_drift.py` understates drift when OpenVINS diverges

It divides the endpoint error by OpenVINS's *own* path length, which inflates when the estimate runs
away (flight 1: 58–61 % vs a true 95 % against the reference path). Fine for healthy closed loops;
for flights use `scripts/analysis/analyze_flight.py` with the flight-controller log.
