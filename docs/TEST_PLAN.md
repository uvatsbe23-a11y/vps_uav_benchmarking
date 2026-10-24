# VPS Drift Fix — Validation & Flight Test Plan

**System under test:** OpenVINS (stereo-inertial MSCKF) on Jetson + Orbbec Gemini 335L
**What we are validating:** the `T_cam_imu` correction in `config/kalibr_imucam_chain.yaml`
(see `KNOWN_ISSUES.md` #2, the ~107 m Z-axis divergence), then flight performance.

> **Status (24 Oct 2026):** Stage 3 was flown on **10 Oct 2026** (4 flights). The `T_cam_imu` fix
> is confirmed (no axis error in flight), but position drifts ~10 m after 50–100 m and OpenVINS
> stopped mid-flight in 3 of 4 flights. Results: [`flight_tests/2026-10-10/`](flight_tests/2026-10-10/).
> **The next flights follow the "NEXT FLIGHT DAY CHECKLIST" in
> [`HANDOFF_2026-10-10.md`](HANDOFF_2026-10-10.md)** (controlled ladder: hover → slow straight
> out-and-back → faster → other heights). Stages 0–2 below still apply to bench and handheld checks.
**Who runs it:** lab juniors. **Who signs off each gate:** project owner (currently remote).

Work through Stages 0 → 3 in order. Each stage is a **gate** — do not start the next stage
until the current one passes and (for Stage 2 → 3) the owner has seen the numbers.

---

## 0. Background — why this test exists

OpenVINS fuses the two IR cameras with the camera IMU to estimate position. In a forward-flight
test the altitude estimate diverged catastrophically: ~107 m endpoint error over 90 s, Z swinging
+4 m to −76 m. This was first blamed on a 90° camera/IMU frame mismatch, and a hand-built 90°
rotation was written into `T_cam_imu`. **That "fix" was itself wrong** (replay diagnosis
2026-09-06): OpenVINS uses the camera's *built-in* IMU, which is near-aligned with the IR cameras,
so `T_cam_imu` must be **near-identity** (OpenVINS's stock `orbbec_335l_stereo_ir` value, now in
`config/`). Replay drift on `stage1_axes`: 43.7 % with the 90° matrix, 1.15 % with near-identity.
See `KNOWN_ISSUES.md` #2. Flights on 10 Oct 2026 confirmed it (no axis error).

The original bag recorded only OpenVINS *output*, so it couldn't be replayed. Hence: **every bag
records raw topics** (`scripts/record_bag.sh` / `flight_record` enforce this).

Mounting (measured from the 10 Oct flights): camera facing **down**, tilted 6–8°.

---

## 1. Prerequisites

### 1.1 Software installed on the Jetson

- ROS 2 (the distro this workspace is built with)
- **Orbbec SDK ROS 2** (`orbbec_camera`) — https://github.com/orbbec/OrbbecSDK_ROS2
- **OpenVINS** (`ov_msckf`) — https://docs.openvins.com/
- Python: `pip install numpy matplotlib`
- This package built:
  ```bash
  cd ~/ros2_ws
  git -C src/vps_uav_benchmarking pull
  colcon build --symlink-install
  source install/setup.bash
  ```

If any external package is missing, install it **before** Stage 0 — that's setup, not part of
the test. The flight Jetson (`vajra`, JetPack 6 + ROS 2 **Humble**) is reachable at
`vajra@192.168.55.1` (USB) or `vajra@10.42.0.1` (`MOTHER_DRONE` Wi-Fi, `wifi-mode flight`).
OpenVINS must be built with the patches in `third_party/open_vins/`.

> **Analyzers:**
> - **Flights:** `scripts/analysis/` compares the bag with the flight controller's `.bin` log
>   (time-aligned; ATE, true drift, divergence distance, …). Use this for Stage 3.
> - `scripts/compute_drift.py <bag> [edge_window_sec]`: quick closed-loop endpoint drift from
>   `/ov_msckf/poseimu` (use `edge_window_sec` e.g. `2`). It divides by OpenVINS's *own* path
>   length, so it **understates drift when OpenVINS diverges** (`KNOWN_ISSUES.md` #12).
> - `scripts/flight_analysis.py <bag> --path-length <m> --plot --csv summary.csv` — batch
>   companion: per-run CSV rows, trajectory PNGs, and the Z-axis excursion alarm. Defaults to
>   `/ov_msckf/odomimu` (same position data); pass `--odom-topic /ov_msckf/poseimu` to match
>   `compute_drift.py` exactly.
>
> `record_bag.sh` captures `/ov_msckf/odomimu`, `/ov_msckf/poseimu` and `/ov_msckf/pathimu`,
> so either tool works on every bag.

> **`vps_bridge`** (IR pre-processor + OpenVINS→MAVROS bridge with pose sanity filter) is now in
> `ros2_packages/vps_bridge/` but is **not** part of the pipeline under test: OpenVINS runs directly on
> the raw driver output and is only recorded.

### 1.2 Physical

- Jetson + Gemini 335L, mounted on the airframe (rigid, correct orientation — see §3.1)
- A closed-loop course: taped rectangle or corridor loop, ~10–20 m perimeter, measured
- Masking tape, tape measure
- (Optional, better) access to the mocap volume

### 1.3 One-time confirmation of the Orbbec launch args

`launch/gemini335l_vio.launch.py` uses OrbbecSDK_ROS2 argument names for the Gemini 330 series.
Confirm them once against the installed driver:

```bash
ros2 launch orbbec_camera gemini_330_series.launch.py --show-args
```

If any key differs (`enable_laser`, `enable_sync_output_accel_gyro`, `accel_rate`, `time_domain`,
or the launch **file name** itself), fix it in `launch/gemini335l_vio.launch.py` and commit.

---

## 2. Stage 0 — Bring-up

### 2.1 Power, clocks and time (every boot: KNOWN_ISSUES #3, #9)

```bash
sudo nvpmodel -m 2      # MAXN_SUPER. NOT -m 0: that is the throttled 15 W mode
sudo jetson_clocks
```
and on the ground PC: `scripts/flight/sync_jetson_clock.sh` (the Jetson clock is wrong after power-on).

### 2.2 Start camera + VIO

Easiest: `flight_start` (does 2.1 except the clock sync, starts both, logs to `~/flights/<date>/…`;
check with `flight_status`). Manual alternative, two terminals, both sourced and with
`export ROS_LOCALHOST_ONLY=1`:

```bash
# terminal A
ros2 launch vps_uav_benchmarking gemini335l_vio.launch.py

# terminal B
ros2 launch vps_uav_benchmarking openvins_gemini335l.launch.py
```

Give OpenVINS a slow figure-8 / gentle shake for a few seconds so it initialises.

### 2.3 Run the automated preflight check

```bash
ros2 run vps_uav_benchmarking check_setup.sh
```

It verifies: the five required topics exist; left/right IR ≥ 15 Hz; IMU
(`/camera/gyro_accel/sample`) ≥ 300 Hz (target 500; ~493 measured); `/ov_msckf/odomimu`
publishing (only after OpenVINS initialises); left IR resolution 848×480; camera↔system clock
offset < 0.5 s; system clock synced this boot.

**Gate:** exit code 0 (no warnings). Fix any warning before Stage 1. Save the console output.

### 2.4 Confirm the bag recipe captures raw topics

```bash
ros2 run vps_uav_benchmarking record_bag.sh bringup_test
# let it run ~10 s, Ctrl-C
ros2 bag info bringup_test
```

**Gate:** `bag info` lists all of: `left_ir/image_raw`, `left_ir/camera_info`,
`right_ir/image_raw`, `right_ir/camera_info`, `gyro_accel/sample`, `ov_msckf/odomimu` — each
with a non-zero message count. Delete `bringup_test` afterwards.

---

## 3. Stage 1 — Verify the axis fix on THIS mounting

### 3.1 Mount

Camera on the airframe in the **exact orientation the calibration assumes** (match the mounting
reference photo). Rigid — it must not shift. (Handheld is acceptable for Stages 0–2 if you hold
that same orientation.)

### 3.2 Static test

```bash
ros2 run vps_uav_benchmarking record_bag.sh stage1_static
# leave the rig completely still for 5-10 min, then Ctrl-C
python3 $(ros2 pkg prefix vps_uav_benchmarking)/lib/vps_uav_benchmarking/flight_analysis.py \
    stage1_static --plot
```

**Gate:**
- `endpoint error` a few cm at most
- `max |z - z_start|` a few cm — **Z must stay flat**, no slow ramp
- the saved `stage1_static_trajectory.png` altitude plot is a flat line

### 3.3 Hand-translation test

With the pipeline running, move the rig a clean **1 m along one body axis at a time**, slowly,
set it down between moves. Do **+X (forward)**, **+Y (left)**, **+Z (up)** separately. Write
down which physical direction you are calling which.

```bash
ros2 run vps_uav_benchmarking record_bag.sh stage1_axes
```

Inspect the estimate (plot it, or `ros2 topic echo /ov_msckf/odomimu --field pose.pose.position`
while replaying).

**Gate:** each move shows up on the estimate in the **same direction**, magnitude ≈ 1 m (±0.2 m).

**Stop and message the owner if:** a move produces motion in the wrong direction, mostly on a
*different* axis, or with a badly wrong magnitude. That means the extrinsics are still wrong —
do not tune around it.

---

## 4. Stage 2 — Handheld closed-loop drift (pre-flight gate)

### 4.1 Course

Tape a start point, mark the facing direction. Walk a closed loop ~10–20 m perimeter. **Measure
the perimeter.** `path_length = perimeter × laps`.

### 4.2 Runs — at least 5

Per run:

```bash
ros2 run vps_uav_benchmarking record_bag.sh stage2_run1     # 2, 3, ...
```

- start recording on the mark
- walk the loop smoothly, **3–4 laps**, normal pace, camera looking at textured surroundings
- return the rig to the **exact same mark and orientation**
- Ctrl-C

### 4.3 Analyse the batch

```bash
FA=$(ros2 pkg prefix vps_uav_benchmarking)/lib/vps_uav_benchmarking/flight_analysis.py
for d in stage2_run*/ ; do
    python3 "$FA" "$d" --path-length <PERIMETER_x_LAPS> --plot --csv stage2_summary.csv
done
python3 $(ros2 pkg prefix vps_uav_benchmarking)/lib/vps_uav_benchmarking/generate_briefing_graphs.py \
    stage2_summary.csv
```

`stage2_summary.csv` + `drift_summary.png` are your deliverables.

### 4.4 Gate

- **drift consistently in the low single-digit %** across all 5+ runs (for reference, the
  `stage1_axes` replay with the fix gave **1.15 %**; the older 1.2–1.3 % bench numbers are suspect)
- **`max_abs_z_dev_m` stays small** on every run — no Z blow-up
- results are repeatable, not one good run among bad ones

Pass → send the owner the CSV + `drift_summary.png` + Stage 0 and Stage 1 evidence, and wait for
go-ahead. Fail → re-check Stage 1, send the numbers, do not proceed.

---

## 5. Stage 3 — Flight test

**Only after Stage 2 passes and the owner approves.**

### 5.1 Setup

Camera mounted as Stage 1, vibration isolation in place if the airframe has it. Jetson clock
synced from the ground PC (`sync_jetson_clock.sh`). Tape the takeoff point; textured ground; flight
profile per the handoff's **NEXT FLIGHT DAY CHECKLIST** (one variable at a time, speed ÷ height
≤ ~0.35 s⁻¹ to start). Normal flight-safety procedure; OpenVINS is only recorded, the drone flies on GPS.

### 5.2 Flights

```bash
flight_start        # then lift/slide the drone until flight_status says INITIALISED
flight_record       # just before take-off
# fly; sharp yaw or short hover right after take-off (time-sync marker); land on the mark
flight_stop         # after landing (from any SSH session)
```

Measure the landing offset from the mark; photograph the ground; download the flight
controller's `.bin` log for each flight.

### 5.3 Analyse

Copy the session folders (`~/flights/<date>/`) and the `.bin` logs to the ground PC, then follow
[`scripts/analysis/README.md`](../scripts/analysis/README.md) (`extract_bag.py`, `extract_bin.py`,
`analyze_flight.py`, optionally `image_metrics.py` + `diagnose_conditions.py`).

### 5.4 What we saw (10 Oct 2026) and expect next

10 Oct: no axis error, but ~10 m error after 50–100 m, 95 % drift on the fast/low flight, and
OpenVINS stopping mid-flight in 3 of 4 flights ([`flight_tests/2026-10-10/`](flight_tests/2026-10-10/)).
Next: within the speed/height envelope and over textured ground, the error should grow clearly
slower; record whatever it is, with the bags.

---

## 6. Deliverables back to the owner

Fill in `docs/REPORT_TEMPLATE.md` and attach:

1. Stage 0: `check_setup.sh` output; `ros2 bag info` of a raw-topic bag.
2. Stage 1: `stage1_static_trajectory.png`; one line per axis move with measured vs reported.
3. Stage 2: `stage2_summary.csv`, `drift_summary.png`, the per-run trajectory PNGs.
4. Stage 3 (after approval): `stage3_summary.csv`, `drift_summary.png`, trajectory PNGs, landing
   offsets.
5. **Keep every bag.** They can be replayed through OpenVINS if the config changes.

---

## 7. Troubleshooting

| Symptom | Likely cause | Action |
|---|---|---|
| `check_setup.sh`: IMU topic missing / low rate | `enable_sync_output_accel_gyro` not set, or rate arg wrong | Fix args in `gemini335l_vio.launch.py` (`--show-args` to get real names) |
| `check_setup.sh`: clock offset > 0.5 s | Driver publishing device time | Launch uses `time_domain: global` + `enable_sync_host_time: true` (as flown); check they are set |
| `check_setup.sh` / `flight_start`: system clock not synced, or year 1970 | Jetson RTC has no battery | Run `scripts/flight/sync_jetson_clock.sh` on the ground PC **before** starting |
| `check_setup.sh`: resolution not 848×480 | IR stream profile mismatch | Set the IR width/height/profile args to match `kalibr_imucam_chain.yaml` |
| OpenVINS never initialises | Not enough motion/texture at start, or bad `config_path` | Slow figure-8 to start; check the launch resolved `estimator_config.yaml` |
| Static test: estimate walks metres, Z ramps | `T_cam_imu` wrong for this mounting | Re-check mount orientation; verify the rotation block in `kalibr_imucam_chain.yaml`; report |
| Hand-move forward → estimate goes backward/sideways | Axis swap/flip — the original bug class | **Stop. Message the owner.** |
| X/Y fine, Z diverges under motion | Same 90° frame bug, partially corrected | Stop; keep the bag; report |
| Frame drops / latency warnings / OpenVINS diverges live but not on replay | Throttled power mode | `sudo nvpmodel -m 2 && sudo jetson_clocks` (not `-m 0`) |
| `flight_status`: OpenVINS "NO OUTPUT NOW (stalled?)" or "CRASHED" | Open issue (`KNOWN_ISSUES.md` #7) | Keep recording; note the time; check `openvins.log` and `/var/crash/cores/` after the flight |
| Feature tracking lost mid-run | Blank wall, motion blur, IR projector on | Keep texture in view, slow down, confirm `enable_laser:=false` |
| `flight_analysis.py`: "Topic ... not in bag" | Wrong `--odom-topic`, or bag recorded without it | `ros2 bag info <bag>`; use `record_bag.sh` next time |
| Single catastrophic pose jump on fast motion | Known cuVSLAM tracking spike (KNOWN_ISSUES #1) | Only affects cuVSLAM runs, not this OpenVINS test |

---

## 8. Glossary

- **VIO / VPS** — estimating position from cameras + IMU (here: OpenVINS).
- **MSCKF** — the filter OpenVINS runs.
- **`T_cam_imu`**: fixed transform between the camera and the IMU. A hand-built 90° rotation here
  was the bug; for the Orbbec built-in IMU it is near-identity. In `config/kalibr_imucam_chain.yaml`.
- **`timeshift_cam_imu`** — small calibrated camera↔IMU time offset (already in the config; not
  the same as gross clock drift, which `check_setup.sh` checks for).
- **Drift** — endpoint error as a percentage of distance travelled.
- **Closed-loop test** — start and end at the same physical point, so true displacement is 0 and
  any nonzero estimate is pure accumulated error.
- **Raw topics** — `image_raw` + `camera_info` + `gyro_accel/sample`: the sensor inputs.
  Recording these (not just the odom output) is what lets a bag be reprocessed. Enforced by
  `record_bag.sh`.
