# Known Issues & Debugging Log

## 1. cuVSLAM: occasional tracking-spike failures during fast motion

**Status:** Understood, not yet architecturally mitigated.

* **Symptom:** During indoor motion tests, cuVSLAM occasionally produced a
  single-frame catastrophic pose jump (velocities up to ~90 m/s, covariance
  spiking from ~1e-9 to single digits for one instant), then recovering -
  but the absolute position offset introduced was never corrected afterward.
* **Key finding:** `/visual_slam/status` (`vo_state`) reported "tracking"
  continuously through every one of these events - this flag alone does
  NOT reliably detect the failure.
* **Correlation:** Strongly associated with fast/jerky camera motion; slow
  deliberate motion tests showed zero spikes.
* **Mitigation not yet implemented:** downstream sanity-check on
  instantaneous velocity/covariance before trusting a pose update.

## 2. OpenVINS: camera-IMU extrinsic axis mismatch causing severe drift

**Status: Root cause identified. Fix written. NOT YET VALIDATED with real flight/motion data.**

* **Symptom:** First live flight test (~90s, ~150m path) showed ~107m
  endpoint drift (72% of path length) and altitude swinging from +4m to
  -76m, gradually over the whole flight (not a single spike).
* **Root cause:** `T_cam_imu` in `kalibr_imucam_chain.yaml` was an identity
  rotation, implicitly assuming the camera optical frame (Z-forward,
  X-right, Y-down) and IMU body frame (X-forward, Y-left, Z-up) share the
  same axes. They don't - so forward flight was being interpreted by the
  estimator as vertical motion.
* **Fix applied:** corrected rotation block in `config/kalibr_imucam_chain.yaml`
  mapping camera axes to IMU axes correctly.
* **IMPORTANT:** the original flight bag only recorded OpenVINS *output*
  topics (`/odomimu`, `/poseimu`, `/pathimu`), not raw camera/IMU input, so
  it cannot be reprocessed with the fix. A NEW test run (ground hand-carry
  or flight) with raw topics recorded is required before treating any
  corrected drift numbers as confirmed. Do not report this as resolved
  until that validation run has been done.

## 3. Hardware / environment gotchas

* **Jetson clock/thermal governor:** lock performance state before testing
  to avoid frame drops and scheduling jitter:
  `sudo nvpmodel -m 0 && sudo jetson_clocks`
* **Jetson system clock resets to epoch (1970) on boot without NTP sync**,
  producing bag folders like `flight_19700101_*`. Install/enable `chrony`
  if accurate timestamps matter.
* **Never run a manual `static_transform_publisher` for a frame the camera
  driver already publishes** - creates a two-parent TF conflict that
  produces unstable pose estimates resembling a tracking bug.
* **Always fully restart both the camera driver and cuVSLAM/OpenVINS nodes
  between test runs** - cuVSLAM keeps building on the same pose graph for
  as long as the node is alive, so a "fresh" test without a restart will
  silently inherit corrupted state from the previous run.
