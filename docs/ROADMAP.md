# VPS Roadmap

*Written 24 Oct 2026, after the first flight tests (10 Oct). Update it as phases complete.*

**Goal:** a camera-based positioning system good enough for the drone to fly GPS-denied: hold
position, fly missions and return home without GPS, with a **bounded** position error.

**Where we are:** OpenVINS (stereo IR + the camera's built-in IMU, on the Jetson) gets attitude and
early scale right, but drifts ~10 m after 50–100 m of flight, and stopped mid-flight in 3 of 4
flights. It is only recorded; the drone flies on GPS. Details: [`flight_tests/2026-10-10/`](flight_tests/2026-10-10/),
[`KNOWN_ISSUES.md`](KNOWN_ISSUES.md). Immediate task list: [`HANDOFF_2026-10-10.md`](HANDOFF_2026-10-10.md).

**What "good" looks like:** no camera-only system has zero drift. Visual-inertial odometry adds up
small motions, so its error grows with distance; good VIO today drifts ~0.5–2 % of distance flown.
Zero *growth* needs absolute references (compass for heading, lidar/baro for height, a map for
horizontal position), which bound the error the way GPS does. The phases below follow that order.

---

## Architecture decision: how the camera system and the flight controller work together

**Chosen ("option 2"): fuse at the flight controller.** OpenVINS runs on the Jetson with the
camera's **own** IMU (hardware-synced to the images, rigidly attached). Only its *result* (position,
velocity, covariance, ~15–30 Hz) goes over MAVLink to the CubeOrange's EKF3, as "external
navigation". EKF3 combines it with the Cube's IMUs (kHz, for control), the **compass** (heading),
the **barometer / lidar** (height), and falls back if vision stops.

**Rejected ("option 1"): feeding the Cube's raw IMU into OpenVINS.** The IMU data would cross a
serial link with millisecond jitter (VIO needs sub-ms IMU–camera timing), arrive filtered and
downsampled, and the Cube's IMUs are vibration-isolated, so not rigid relative to the camera. It
also doesn't address the actual cause of the drift (vision, not the IMU).

**Heading before full fusion:** the Cube's compass heading is a slow signal, so it *can* be sent to
the Jetson over MAVLink and used to correct OpenVINS's yaw drift (smoothed, updated while not
turning fast). Latency only matters for fast signals like raw IMU.

---

## Phase 0: Bench readiness *(next session, ~1 h)*

- [ ] Full `flight_start → flight_record → flight_stop` cycle with the camera connected.
- [ ] OpenVINS stopped several times **while processing images**: clean exit in < 1 s, no core dump.
- [ ] `scripts/flight/sync_jetson_clock.sh` tested interactively.
- [ ] Free Jetson disk (4 bags are archived on the ground PC).

**Exit:** the field routine works end to end on the bench.

## Phase 1: Improve OpenVINS offline, on the recorded flights *(no flying)*

- [ ] Replay all four 10 Oct bags at 1×. Check the live result reproduces.
- [ ] **Mid-flight stops:** does OpenVINS stop at the same moment on replay (flights 2/3/4)?
      Yes → deterministic bug, debug with core dumps. No → external cause (SSH / power).
- [ ] Try one change at a time; keep only what improves **all four** flights
      (score: `scripts/analysis/analyze_flight.py`):
  - [ ] process every frame: `track_frequency` 21 → 31 (`scripts/replay_diag/cfg_klt_freq30/`);
  - [ ] wider KLT search (pyramid levels / window);
  - [ ] contrast boost for plain ground (CLAHE; `vps_bridge/ir_preprocessor.py`), lower FAST threshold;
  - [ ] IMU noise inflation for a more honest covariance (properly: Phase 2 Allan variance);
  - [ ] mask the drone's own shadow.

**Exit:** a config that roughly doubles the distance before 10 m error on the recorded flights, with
no replay stops (or the stop cause identified).

## Phase 2: Hardware and calibration

- [ ] RTC backup battery on the Jetson carrier (permanent clock fix).
- [ ] Secure Jetson power: locking connector, strain relief, its own BEC.
- [ ] Soft-mount the **whole** camera (camera + IMU stay rigid together).
- [ ] Camera tilt **15–30° forward** in the new mount (start at 15–30°, go to 30–45° if needed). Keep
      props, legs and sky out of view, including the extra pitch in fast forward flight.
      `T_cam_imu` does **not** change (IMU inside the camera); re-measure the camera→drone mount
      angle (gyro fit, as in `scripts/analysis/`) and use it wherever camera→body is needed.
- [ ] Downward lidar rangefinder (height reference now, EKF input in Phase 5).
- [ ] Allan variance (~2 h static IMU recording) → real IMU noise parameters.
- [ ] Kalibr on the **final mount**: IR stereo camchain + IMU–camera (Orbbec built-in IMU).
- [ ] RTK GPS as the test reference, if one can be borrowed (today's GPS is only 2–5 m).

**Exit:** final mount and calibration in the repo; OpenVINS error inside its own 3σ most of the time
on replays (currently 5–10 %).

## Phase 3: Controlled flight ladder *(one variable at a time)*

Conditions from the 10 Oct analysis: diffuse light (no hard drone shadow), **textured ground**,
straight **out-and-back** legs with few turns, **higher and slower** (speed ÷ height ≤ ~0.35 s⁻¹ to
start), lidar for height. Use the handoff's **NEXT FLIGHT DAY CHECKLIST**.

1. Hover 60 s at 9 m. Expect within ~1 m.
2. Straight 100 m out and back at 2 m/s, 9 m.
3. Same at 4 and 6 m/s.
4. Repeat at 5 m and 15 m.
5. Tilted mount: repeat steps 1–2, then 3–4.
6. Full missions inside the envelope found.

**Exit:** a measured speed/height envelope inside which OpenVINS drifts **≤ 2–3 % of distance**, and
**no mid-flight stops** across ≥ 5 flights.

## Phase 4: Flight-controller link, observe only *(vision still does not fly the drone)*

- [ ] Physical link Cube TELEM2 ↔ Jetson UART (TX/RX/GND, no 5 V), e.g. `SERIAL2_PROTOCOL = 2`
      (MAVLink 2), `SERIAL2_BAUD = 921`, attitude stream ~50 Hz. MAVROS on the Jetson (`fcu_url`
      on the UART); its time sync puts FC data on the Jetson clock.
- [ ] Record FC attitude / compass heading / baro / lidar in every bag (`record_bag.sh`).
- [ ] **Heading correction:** first estimate the gain offline on the 10 Oct logs (compass-only
      heading); then a node (in `vps_bridge`) that tracks the OpenVINS→north yaw offset from the FC
      heading, smoothed over seconds and updated only below ~10°/s turn rate, applies it to each *new*
      motion step (no jumps), and publishes raw + corrected pose.
- [ ] `vps_bridge` → MAVROS **ODOMETRY** (position, orientation, velocity, covariance, reset counter)
      to EKF3, configured so EKF3 **uses GPS** and the vision data is only logged and compared
      (innovations, consistency).

**Exit:** heading drift removed in the corrected output; EKF3's view of the vision data is consistent
on GPS flights; compass healthy with everything mounted and powered.

## Phase 5: GPS-denied flights *(with GPS as fallback)*

- [ ] EKF3 sources: position/velocity from external navigation, height from baro/lidar, heading from
      compass (e.g. `VISO_TYPE`, `EK3_SRC*` parameters), with a second source set on GPS, switchable
      by an RC switch.
- [ ] Switch to vision at altitude over the take-off area: Loiter, then short Guided moves, then
      missions, then RTL. Always with a pilot ready to switch back to GPS.
- [ ] Failsafe behaviour when vision stops (Phase 1/3 stop cause must be fixed first).

**Exit:** position hold and RTL without GPS, with the error staying inside a stated bound over the
mission length.

## Phase 6: Bounded horizontal error *(the long-term "VPS")*

- [ ] **Loop closure** on revisited areas (out-and-back routes, home): snaps accumulated error back.
- [ ] **Map / satellite-image matching**: match the downward camera to orthophotos or a prior map
      for absolute position (the meaning of "VPS" in the wider field; how planetary helicopters and
      landers navigate). Needs recognisable terrain and handling of lighting/season changes.
- [ ] Optional absolute aids where available (UWB beacons, signals of opportunity).

**Exit:** error stays bounded (GPS-like, a few metres) regardless of distance flown.

---

## Cross-cutting

- **Every test records raw topics** and the FC `.bin` log; analyse with `scripts/analysis/`.
- **The repo is public:** never commit flight logs, GPS coordinates or passwords.
- **cuVSLAM:** parked; revisit only if OpenVINS can't meet Phase 3 (`KNOWN_ISSUES.md` #1).
- Keep `KNOWN_ISSUES.md`, this roadmap and a dated handoff updated after every test day.
