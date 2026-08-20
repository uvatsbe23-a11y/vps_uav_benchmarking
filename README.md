# Autonomous UAV Visual Positioning System (VPS) Benchmarking

This repository provides configurations and analysis tools for evaluating Visual-Inertial Odometry (VIO) and Visual SLAM systems on NVIDIA Jetson paired with Orbbec Gemini cameras.

## Hardware Prerequisites
Lock Jetson clocks before flight to prevent IMU timing errors:
`sudo nvpmodel -m 0`
`sudo jetson_clocks`


## Software Dependencies
Before building this package, ensure the following drivers and libraries are installed on the Jetson:

**1. External ROS 2 Packages:**
* [Orbbec SDK ROS 2](https://github.com/orbbec/OrbbecSDK_ROS2) (Camera Drivers)
* [OpenVINS](https://docs.openvins.com/) (Stereo-Inertial MSCKF)
* [Isaac ROS cuVSLAM](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_visual_slam/index.html) (Optional, for pure-vision benchmarking)

**2. Python Libraries:**
The post-flight diagnostic scripts require standard data science libraries:
`pip install numpy matplotlib`



## Installation
Clone into your ROS 2 workspace:
`cd ~/ros2_ws/src`
`git clone https://github.com/uvatsbe23-a11y/vps_uav_benchmarking.git`
`cd ~/ros2_ws && colcon build --symlink-install`

## Post-Flight Diagnostics
* **Flight Analyzer:** `python3 scripts/flight_analysis.py /path/to/flight_bag`
* **Report Generation:** `python3 scripts/generate_briefing_graphs.py`


