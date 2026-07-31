# Known Issues & Debugging Log

## 1. cuVSLAM: occasional tracking-spike failures during fast motion

**Status:** Understood, not yet mitigated. Fast/jerky camera motion occasionally
triggers a single-frame catastrophic pose jump; `vo_state` does not reliably
flag it. Mitigation planned: downstream velocity/covariance sanity-check.

## 2. Z-Axis Divergence (Camera-IMU Axis Mismatch)

**Status: Fix applied, NOT YET VALIDATED with real flight/motion data.**

* **Symptom:** VIO trajectory diverged aggressively on the Z-axis during
  forward flight (~107m endpoint drift, altitude swinging +4m to -76m
  over a 90s test).
* **Root cause:** 90-degree coordinate frame mismatch between the Camera
  Optical Frame (Z-forward) and IMU Body Frame (X-forward).
* **Fix:** corrected rotation matrix applied to `T_cam_imu` in
  `kalibr_imucam_chain.yaml`.
* **Outstanding:** original flight bag only recorded OpenVINS output
  topics, not raw sensor input, so it cannot be reprocessed to confirm
  the fix. A new validation test (raw topics recorded) is required
  before treating this as resolved.

## 3. Hardware Clock Drift

* **Symptom:** Frame drops and thread scheduling latencies during
  high-frequency execution.
* **Fix:** Lock Jetson performance governor:
  `sudo nvpmodel -m 0 && sudo jetson_clocks`
