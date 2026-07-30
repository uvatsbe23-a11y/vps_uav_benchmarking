# Autonomous UAV Visual Positioning System (VPS) Benchmarking

This repository provides configurations and analysis tools for evaluating Visual-Inertial Odometry (VIO) and Visual SLAM systems on NVIDIA Jetson paired with Orbbec Gemini cameras.

## Hardware Prerequisites
Lock Jetson clocks before flight to prevent IMU timing errors:
`sudo nvpmodel -m 0`
`sudo jetson_clocks`

## Installation
Clone into your ROS 2 workspace:
`cd ~/ros2_ws/src`
`git clone https://github.com/YourUsername/vps_uav_benchmarking.git`
`cd ~/ros2_ws && colcon build --symlink-install`

## Post-Flight Diagnostics
* **Flight Analyzer:** `python3 scripts/flight_analysis.py /path/to/flight_bag`
* **Report Generation:** `python3 scripts/generate_briefing_graphs.py`

## License & Access
**Proprietary and Confidential.** 
This repository is restricted for internal team use only. Unauthorized distribution is prohibited.
