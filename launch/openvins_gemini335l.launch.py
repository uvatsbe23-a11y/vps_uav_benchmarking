"""
Launch OpenVINS (ov_msckf) with the calibration in this package's config/ folder.

    ros2 launch vps_uav_benchmarking openvins_gemini335l.launch.py

Publishes the estimate on /ov_msckf/odomimu (nav_msgs/Odometry) - that is the
topic flight_analysis.py and the bag recipe use.

Start the camera first:  ros2 launch vps_uav_benchmarking gemini335l_vio.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("vps_uav_benchmarking")
    default_cfg = os.path.join(pkg_share, "config", "estimator_config.yaml")

    config_arg = DeclareLaunchArgument(
        "config_path",
        default_value=default_cfg,
        description="Path to the OpenVINS estimator_config.yaml",
    )

    ov_node = Node(
        package="ov_msckf",
        executable="run_subscribe_msckf",
        name="ov_msckf",
        output="screen",
        parameters=[{
            "config_path": LaunchConfiguration("config_path"),
            "use_sim_time": False,
        }],
    )

    return LaunchDescription([config_arg, ov_node])
