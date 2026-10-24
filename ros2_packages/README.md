# Companion ROS 2 packages

`vps_bridge/`: OpenVINS → MAVROS bridge (`vps_bridge_node`, with a pose sanity filter
that rejects implied-velocity / covariance outliers) and `ir_preprocessor.py` (median blur +
histogram equalisation on the IR images). Written Aug 2026; **not yet used in any test**.

This repo root is itself a ROS package, so colcon will not build packages nested inside it.
To build: `ln -s <repo>/ros2_packages/vps_bridge ~/ros2_ws/src/vps_bridge && colcon build --packages-select vps_bridge`.

Note: the sanity filter weights by OpenVINS's covariance, which the 10 Oct flights showed to be
badly over-confident; see docs/HANDOFF_2026-10-10.md before relying on it.
