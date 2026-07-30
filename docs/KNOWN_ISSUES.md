# Known Issues & Debugging Log

## 1. Z-Axis Divergence (Gravity Integration Error)
* **Symptom:** VIO trajectory diverges aggressively on the Z-axis during forward flight.
* **Root Cause:** 90-degree coordinate frame mismatch between the Camera Optical Frame (Z-forward) and ROS Body Frame (X-forward). EKF incorrectly integrates gravity along the forward axis.
* **Fix:** Apply 90-degree rotation matrix to `T_cam_imu` in `kalibr_imucam_chain.yaml`.

## 2. Hardware Clock Drift
* **Symptom:** Frame drops and thread scheduling latencies during high-frequency execution.
* **Fix:** Lock Jetson performance governor using `sudo nvpmodel -m 0` and `sudo jetson_clocks`.
