# VPS Roadmap

*Written 24 Oct 2026, after the first flight tests (10 Oct 2026) and the design discussion that
followed. Update it as phases complete.*

**Goal:** a camera-based positioning system good enough for the drone to fly GPS-denied: hold
position, fly missions and return home without GPS, with a **bounded** position error.

**Where we are:** OpenVINS (stereo IR + the camera's built-in IMU, on the Jetson) gets attitude and
early scale right, but drifts ~10 m after 50–100 m of flight, and stopped mid-flight in 3 of 4
flights. It is only recorded; the drone flies on GPS. Details: [`flight_tests/2026-10-10/`](flight_tests/2026-10-10/),
[`KNOWN_ISSUES.md`](KNOWN_ISSUES.md). Immediate task list: [`HANDOFF_2026-10-10.md`](HANDOFF_2026-10-10.md).

**Contents**
1. [What the 10 Oct flights taught us](#1-what-the-10-oct-flights-taught-us)
2. [Why a camera system can't simply match GPS](#2-why-a-camera-system-cant-simply-match-gps)
3. [Architecture: how the camera system and the flight controller work together](#3-architecture-how-the-camera-system-and-the-flight-controller-work-together)
4. [Heading correction from the Pixhawk compass](#4-heading-correction-from-the-pixhawk-compass)
5. [Camera tilt and calibration](#5-camera-tilt-and-calibration)
6. [Test conditions for the next flights](#6-test-conditions-for-the-next-flights)
7. [Hardware changes](#7-hardware-changes)
8. [Phases](#8-phases) (0 bench → 6 bounded error)
9. [Cross-cutting rules](#9-cross-cutting-rules)

---

## 1. What the 10 Oct flights taught us

Full analysis: [`flight_tests/2026-10-10/README.md`](flight_tests/2026-10-10/README.md).

| Finding | Evidence | Consequence for the plan |
|---|---|---|
| Calibration is not grossly wrong | Attitude within 4.4° tilt / 15° heading of the FC; early scale 0.85–1.09 | No axis bug: focus on tracking quality, not frames |
| Position drifts ~10 m after 50–100 m | All 4 flights (best 101 m, worst 58 m) | Need better VIO **and** absolute references |
| **Worst drift: ground moving too fast in the image** | Flight 1 (4.7 m, 9–10 m/s): features moved ~45 px between frames (10× other flights); tracking success dropped to ~80 % | Fly higher/slower; process every frame; consider tilting the camera |
| **Second: plain, low-texture ground** | < 300 corners/image or low sharpness → ~2.5× faster drift, even hovering (flight 3) | Choose textured ground; contrast boost |
| Best case | 9 m height, ≤ 3 m/s, little turning, tracks/objects in view | Starting point of the flight ladder |
| Provisional speed limit | Every good 5 s window had speed ÷ height ≤ ~0.35 s⁻¹ (≈ 1.7 m/s at 5 m, 3 m/s at 9 m, 5 m/s at 15 m) | Envelope to confirm in Phase 3 |
| Height is the weakest axis | −255 m (flight 1), 4 m dip and −10 m at landing (flight 4) | Lidar for height |
| Heading drifts 7–15° per flight | Most of flight 2's 13 m error was a ~6° rotation | Compass heading (§4) |
| OpenVINS is over-confident | True error inside its own 3σ only 5–10 % of the time (≈ 99 % expected) | Allan variance + Kalibr before fusing into the FC |
| **OpenVINS stopped mid-flight in 3 of 4 flights** | All outputs stop at once, normal latency until then; cause unknown | Must be solved before vision flies the drone |
| OpenVINS uses every other frame | `track_frequency: 21` with a 30 fps camera → 15 Hz | One-line change, test offline first |
| Not the problem | Vibration at 9 m, image brightness, compute (latency 40–75 ms) | Don't buy a faster computer |
| Flight 2 only *looks* best | OpenVINS stopped after its easiest stretch; at equal distance it is only moderately better | Compare flights at equal distance, change one variable at a time |

## 2. Why a camera system can't simply match GPS

- **GPS measures where you *are*** (against satellites at known positions), fresh several times a
  second. Each fix is independent, so errors never accumulate: always off by 2–5 m (cm with RTK),
  but it **doesn't drift**.
- **Visual-inertial odometry measures how much you *moved*** since the last frame and adds the moves
  up (dead reckoning). Each step has a small error, and adding steps adds their errors.
  Mathematically, VIO **cannot observe absolute position or absolute heading**: nothing in the images
  says where on Earth you are or where north is, so those errors grow without limit.
- **So zero drift from a camera alone is impossible.** The question is how small the drift *rate* is,
  and what resets it.

**Why even the best VIO drifts**

| Cause | Meaning | Seen in our flights? |
|---|---|---|
| IMU noise and slowly changing bias | small acceleration errors become position errors growing with time² | partly: velocity ran away (flights 1, 3) |
| Feature tracking errors | sub-pixel errors per point; worse with blur, fast motion, plain ground | **yes, the main cause** |
| Weak depth from height | stereo baseline ~10 cm; at 9 m depth is poor, scale leans on the IMU | likely contributes |
| Calibration / timing errors | small camera–IMU geometry or time-offset errors act as a constant bias | possible (calibration still pieced together) |
| Filter over-confidence | the filter thinks it is more accurate than it is and down-weights good corrections | **yes** (5–10 % within 3σ) |
| Scene problems | moving shadows (incl. the drone's own), people, water, glare, darkness, repetitive patterns | possibly (H-frame shadow visible) |

**Realistic numbers:** good VIO today drifts **~0.5–2 % of distance flown** (our clean replay: 1.4 %).
Our flights did ~10–20 %, mostly for the reasons above.

**How the field gets close to GPS: adding absolute information**

| Method | Pins down | Notes |
|---|---|---|
| Lidar / rangefinder / barometer | height only | cheap; fixes our worst axis |
| Magnetometer (compass) | heading only | removes heading drift; disturbed by motors/metal |
| Loop closure (recognising a place seen before) | snaps position back on revisits | helps out-and-back routes |
| Map / satellite-image matching ("visual terrain-relative navigation") | full absolute position, bounded error | how the Mars helicopter and landers navigate; needs a map and recognisable terrain; struggles with season/lighting/plain ground; active research. This is what "VPS" usually means in the wider field |
| Radio signals of opportunity (cellular, LEO satellites, Wi-Fi, UWB beacons) | absolute position | where those signals exist |
| Better VIO itself (learned features, event cameras, consistent filters, online calibration) | lowers the drift *rate* | does not remove drift |

With absolute aids the error becomes **bounded** ("as good as GPS"), not zero; GPS isn't zero either.

## 3. Architecture: how the camera system and the flight controller work together

The Cube Orange is an excellent flight controller (triple, heated, damped IMUs, compass, baro).
The question was whether to use its IMU and compass together with the camera on the Jetson.
There are two ways to combine them.

### Option 1 (rejected): the Cube's raw IMU inside OpenVINS
OpenVINS would use only the Pixhawk IMU, not the camera's own.

- *Tempting because:* the Cube's IMUs vibrate less (camera IMU measured 1.5–2× more), are
  temperature-stabilised and redundant.
- *Rejected because:*
  - **Timing (the main reason).** The camera IMU is hardware-synced to the camera exposures on the
    same clock. The Cube's IMU data would reach the Jetson over serial MAVLink with millisecond-level,
    *varying* delay. VIO uses every gyro reading to predict rotation between frames; the error is
    rotation rate × timing error: at 2 rad/s (~115°/s, normal for a drone) a 5 ms error is
    **~0.6° wrong on every frame**, so good features look wrong and errors pile up. Calibration can
    correct a *constant* offset, not jitter.
  - **What leaves the Cube is not the raw IMU.** Inside the Cube the IMUs are sampled at
    **1–8 kHz** and EKF3 runs at ~400 Hz; what is logged (`.bin`: 25 Hz) or streamed over MAVLink
    is downsampled and filtered (tens to a few hundred Hz), which adds delay.
    Bandwidth alone is *not* the blocker: 500 readings/s ≈ 40 KB/s fits a 921600-baud link.
  - **Not rigid:** the Cube's IMUs float on internal dampers; the camera is on the frame.
    VIO assumes camera + IMU are one rigid body.
  - **More calibration:** a new camera–Cube calibration after every mount change.
  - **It wouldn't fix our actual problem:** the drift came from vision (fast/plain ground), not the IMU.
- If ever tried: offline only, with the Cube's fast IMU logging enabled (the 25 Hz `.bin` data is too
  slow).

### Option 2 (chosen): OpenVINS → the Cube's EKF3 ("external navigation")
Both IMUs are used, **each inside its own computer, at full speed**; only the *result* crosses the cable.

```
JETSON (camera + camera IMU)              cable              PIXHAWK (its IMU + compass + baro + lidar)
  camera IMU: ~500 readings/s                                  Cube IMU: 1–8 kHz, EKF3 at ~400 Hz
  camera: 30 frames/s                                          EKF predicts with its own IMU
  OpenVINS: 15–30 positions/s ──── 15–30 small messages ────►  and corrects with each vision position
                                     (position + velocity)      (compass → heading, baro/lidar → height)
                                                               → flies the drone
```

- **What crosses the cable:** a MAVLink **`ODOMETRY`** message per OpenVINS output (via `vps_bridge`
  → MAVROS; older alternative: `VISION_POSITION_ESTIMATE` + `VISION_SPEED_ESTIMATE`):

  | Field | Content | Why the FC needs it |
  |---|---|---|
  | `time_usec` | when the camera frame was taken | apply the correction at the right moment despite ~50 ms delay |
  | position x, y, z | where OpenVINS thinks the drone is | main correction (takes GPS's place) |
  | orientation (quaternion) | attitude / heading | optional heading source |
  | velocity vx, vy, vz | speed | strong, fast correction |
  | covariance | OpenVINS's claimed uncertainty | weighting (**our over-confidence bites here**) |
  | `reset_counter` | increments on restart/jump | so a restart isn't treated as a sudden 50 m move |
  | quality / estimator type | health | lets the EKF ignore bad data |

  ~230 bytes × 15–30/s ≈ 5–7 KB/s, a small fraction of the link.
- **Why the Pixhawk needs it at all:** every autonomous mode (Loiter, Auto missions, RTL, Guided)
  runs on EKF3's position estimate, which today comes from GPS. Without GPS the drone can hold
  attitude and roughly height but **cannot hold position, fly a mission or come home**. Option 2
  makes the camera system **take GPS's place** (GPS also only delivers 5–10 fixes/s, and the same
  predict-and-correct scheme handles that).
- **What it improves:** GPS-denied autonomous flight; GPS backup/switching (RC switch);
  heading held by the compass; height by baro/lidar; smooth 400 Hz low-latency position from the
  Cube's IMU instead of jumpy 15 Hz updates; outlier rejection (innovation gating); a failsafe when
  OpenVINS stops.
- **What it does not improve:** OpenVINS itself (data flows one way). Horizontal position stays
  only as good as OpenVINS, so vision quality comes first.
- **Catches:** EKF3 trusts OpenVINS's covariance (currently over-confident); compass interference
  from motors/currents (calibrate with everything mounted and powered); vision affecting flight is a
  safety step, so log first, then fuse with GPS active, then GPS-denied.

## 4. Heading correction from the Pixhawk compass

Fixing heading needs data flowing **Pixhawk → Jetson**: the Pixhawk knows heading from its compass,
OpenVINS doesn't. The camera stays on the Jetson's USB.

```
 Camera ──USB──► Jetson (OpenVINS) ◄──── MAVLink serial ──── Pixhawk (compass + EKF heading)
                     └─► heading-corrected position (logged first; later sent to the Pixhawk = option 2)
```

**Why latency is fine here but not for raw IMU:** latency only hurts a signal that changes a lot
within the delay.
- OpenVINS's heading *drift* changes slowly (7–15° over minutes ≈ 0.1°/s): a 50 ms delay causes
  ≈ 0.005° error, and the offset is averaged over seconds anyway.
- The one place timing matters: comparing FC heading and OpenVINS heading **at the same instant**.
  During a 30°/s turn, a 20 ms mismatch gives a 0.6° spike. Handled by MAVROS time sync, averaging
  over seconds, and updating the offset only while turning slower than ~10°/s.

**Steps**
1. **Physical link:** Cube **TELEM2 → Jetson UART** on the 40-pin header (usually `/dev/ttyTHS1` on
   the Orin Nano dev kit): TX→RX, RX→TX, GND; **no 5 V**. Bench alternative: USB to the Cube
   (`/dev/ttyACM0`), less robust in flight. Pixhawk: `SERIAL2_PROTOCOL = 2` (MAVLink 2),
   `SERIAL2_BAUD = 921`, `SR2_EXTRA1 = 50` (attitude 50 Hz), `SR2_POSITION = 10`.
2. **MAVROS on the Jetson** (`fcu_url:=/dev/ttyTHS1:921600`; handles time sync). Topics:
   `/mavros/imu/data` (attitude incl. heading), `/mavros/global_position/compass_hdg`,
   `/mavros/imu/mag` (raw compass), `/mavros/local_position/pose`. Add them to `record_bag.sh`.
   (MAVROS is likely installed since `vps_bridge` uses it; verify.)
3. **Correction node** (extension of `vps_bridge`):
   - estimate the yaw offset OpenVINS-frame → north from FC attitude, OpenVINS attitude and the fixed
     camera→drone mount rotation (measured from the gyros: ~7° tilt today);
   - smooth it over seconds; update only below ~10°/s turn rate;
   - **apply it to each new motion step, not to the whole path**, so the path never jumps sideways;
   - publish raw and corrected pose. It only *reads* from the Pixhawk; no effect on flying.
4. **Estimate the gain offline first** on the 10 Oct logs (they contain FC attitude and raw compass).
   Use **compass-only** heading for an honest GPS-denied test (the FC's normal heading also uses GPS).
   Expect a few metres per 100 m on flights like #2/#4; it **won't fix** flights 1/3 (tracking failed).

**Watch out for:** compass interference (motors, battery current, Jetson and its cables); an accurate
mount angle (re-measure after any re-mount); the mid-flight stops are unaffected by this.

## 5. Camera tilt and calibration

Plan: tilt the camera **15–30° forward** (to 30–45° if needed). Goal: the far part of the image
moves slower than straight down (less of flight 1's problem) and the camera sees more texture
(horizon structures, objects).

- **No recalibration of OpenVINS is needed for the tilt.** OpenVINS doesn't assume the camera points
  down: it finds "down" from gravity with the IMU inside the camera, so tilting the whole camera just
  changes its measured starting orientation. Proof: the camera is already ~7° off vertical and
  OpenVINS's tilt agreed with the FC to 2–4°.
- **Calibration items unaffected:** lens intrinsics/distortion, stereo baseline, `T_cam_imu` (all
  inside the camera body). Feature depths are unaffected (stereo geometry inside the camera); points
  at the top of a tilted image are farther and less precise, which OpenVINS accounts for.
- **What must be updated: the camera→drone mount rotation**, used by the analysis scripts (measured
  automatically from the gyros), the heading-correction node (§4) and `vps_bridge` (camera frame →
  drone frame when talking to the Pixhawk). Re-measure once after re-mounting (gyro fit from a short
  flight or hand rotations).
- **Practical checks:** props, legs and frame out of view; no sky in the frame, including the extra
  10–20° of pitch in fast forward flight; a rigid mount whose angle cannot change in flight.
- **Proper Kalibr + Allan variance is still owed regardless** (the current file is a pieced-together
  stopgap, and the likely cause of the over-confidence).

## 6. Test conditions for the next flights

Agreed plan, with how strongly the 10 Oct data supports each point:

| Plan | Note from the data |
|---|---|
| Avoid direct sunlight (overcast / diffuse) | Brightness had no measurable effect and nothing was over-exposed. The benefit is no sharp **drone shadow** moving with the drone. Avoid dusk-level darkness. |
| Featureful ground | Strongly supported (low texture ~2.5× faster drift, even hovering). |
| Straight out-and-back legs, few turns | Supported (straightest stretch drifted least); return to the take-off mark. |
| Higher altitude, slower speed | Strongly supported: speed ÷ height ≤ ~0.35 s⁻¹ to start. |
| Lidar for altitude | OpenVINS can't use it directly: height **reference** now, FC input later. |
| Higher frame rate | First make OpenVINS use the frames it already gets (`track_frequency` → ≥ 30); raise camera fps only if needed. |
| Tilt camera 15–30° | See §5. |

**Additional rules:** bench-test the flight scripts with the camera first; try the software fixes
on the recorded bags first; check whether OpenVINS stops at the same moment on replay (none of the
above fixes the mid-flight stops); **change one thing at a time** (otherwise you can't tell which
change helped or where the limits are); `sync_jetson_clock.sh` after every power-on; a sharp yaw or
short hover after take-off (time-sync marker); measure the landing offset; photograph the ground;
RTK if available.

## 7. Hardware changes

| Tier | Change | Why |
|---|---|---|
| Before next flights (cheap) | RTC coin cell on the Jetson carrier | clock resets to 1970 on every power-off (software fix only restores the last saved time) |
| | Secure Jetson power (locking connector, strain relief, own BEC) | lost power once while handled; a cut in flight loses the flight |
| | Soft-mount the **whole** camera (never camera and IMU separately) | camera IMU shakes 1.5–2× more; do before calibrating |
| | Downward rangefinder (TF-series / LightWare) | height was the worst axis; also gives speed ÷ height |
| For evaluation | RTK GPS as ground truth | today's 2–5 m GPS can't measure 1–3 % drift; glitched in 2 flights |
| Only if software isn't enough | Tilt camera forward (§5) | less image motion, more texture |
| | Optical-flow sensor + rangefinder into ArduPilot (ARK Flow / H-Flow) | independent backup velocity when OpenVINS stalls |
| | Higher camera fps (if the 335L supports it on IR stereo) | halves image motion; CPU cost |
| Minor | Larger NVMe / USB SSD | 4–9 GB per flight, ~45 GB free |
| | External Wi-Fi antennas | `MOTHER_DRONE` drops at ~70–120 m; scripts already survive drops |
| **Don't buy** | A faster computer | OpenVINS kept up in every flight (no backlog) |

---

## 8. Phases

### Phase 0: Bench readiness *(next session, ~1 h)*
- [ ] Full `flight_start → flight_record → flight_stop` cycle with the camera connected.
- [ ] OpenVINS stopped several times **while processing images**: clean exit < 1 s, no core dump.
- [ ] `scripts/flight/sync_jetson_clock.sh` tested interactively.
- [ ] Free Jetson disk (the 4 bags are archived on the ground PC).

**Exit:** the field routine works end to end on the bench.

### Phase 1: Improve OpenVINS offline, on the recorded flights *(no flying)*
- [ ] Replay all four 10 Oct bags at 1×; check the live result reproduces.
- [ ] **Mid-flight stops:** same moment on replay (flights 2/3/4)? Yes → deterministic bug, debug
      with core dumps. No → external cause (SSH output blocking / power).
- [ ] One change at a time; keep only what improves **all four** flights (score with
      `scripts/analysis/analyze_flight.py`):
  - [ ] every frame: `track_frequency` 21 → 31 (`scripts/replay_diag/cfg_klt_freq30/`);
  - [ ] wider KLT search (pyramid levels / window);
  - [ ] contrast boost for plain ground (CLAHE; `vps_bridge/ir_preprocessor.py`), lower FAST threshold;
  - [ ] IMU noise inflation for a more honest covariance (properly in Phase 2);
  - [ ] mask the drone's own shadow.
- [ ] Offline estimate of the compass heading-correction gain (§4 step 4).

**Exit:** about double the distance before 10 m error on the recorded flights; stop cause identified.

### Phase 2: Hardware and calibration
- [ ] Tier-1 hardware from §7 (RTC battery, secure power, soft mount, rangefinder).
- [ ] New mount with **15–30° tilt** (§5); re-measure the camera→drone rotation.
- [ ] Allan variance (~2 h static IMU recording) → real IMU noise parameters.
- [ ] Kalibr on the **final mount**: IR stereo camchain + IMU–camera (Orbbec built-in IMU).
- [ ] RTK GPS reference if available.

**Exit:** final mount and calibration in the repo; OpenVINS error inside its own 3σ most of the time on replays.

### Phase 3: Controlled flight ladder *(one variable at a time; conditions §6)*
1. Hover 60 s at 9 m (expect within ~1 m).
2. Straight 100 m out and back at 2 m/s, 9 m.
3. Same at 4 and 6 m/s.
4. Repeat at 5 m and 15 m.
5. Tilted mount: repeat 1–2, then 3–4 (if the tilt is not already fitted for step 1, keep it as a separate variable).
6. Full missions inside the envelope found.

**Exit:** a measured speed/height envelope inside which OpenVINS drifts **≤ 2–3 % of distance**, and
**no mid-flight stops** across ≥ 5 flights.

### Phase 4: Flight-controller link, observe only *(vision does not fly the drone yet)*
- [ ] Cube ↔ Jetson link + MAVROS (§4 steps 1–2); FC attitude/compass/baro/lidar in every bag.
- [ ] Heading-correction node (§4 step 3); compare raw vs corrected against the reference.
- [ ] `vps_bridge` → MAVROS `ODOMETRY` (§3) into EKF3 configured so EKF3 **uses GPS** and the vision
      data is only logged/compared (innovations, consistency).

**Exit:** heading drift removed in the corrected output; EKF3's view of the vision data is consistent;
compass healthy with everything mounted and powered.

### Phase 5: GPS-denied flights *(GPS as fallback)*
- [ ] EKF3 sources: position/velocity from external navigation, height from baro/lidar, heading from
      compass (e.g. `VISO_TYPE`, `EK3_SRC*` parameters); a second source set on GPS, switchable by RC.
- [ ] Switch to vision at altitude over the take-off area: Loiter → short Guided moves → missions →
      RTL, pilot ready to switch back.
- [ ] Failsafe when vision stops (the Phase 1/3 stop cause must be fixed first).

**Exit:** position hold and RTL without GPS, error inside a stated bound over the mission length.

### Phase 6: Bounded horizontal error *(the long-term "VPS")*
- [ ] Loop closure on revisited areas (out-and-back, home).
- [ ] Map / satellite-image matching of the downward camera for absolute position.
- [ ] Optional absolute aids where available (UWB beacons, signals of opportunity).

**Exit:** error stays bounded (GPS-like, a few metres) regardless of distance flown.

---

## 9. Cross-cutting rules

- **Every test records raw topics** and the FC `.bin` log; analyse with `scripts/analysis/`.
- **Compare flights at equal distance flown**, and change one variable at a time.
- **The repo is public:** never commit flight logs, GPS coordinates or passwords.
- **cuVSLAM:** parked; revisit only if OpenVINS can't meet Phase 3 (`KNOWN_ISSUES.md` #1).
- After every test day: update `KNOWN_ISSUES.md`, this roadmap, and write a dated handoff.
