"""
Start the Orbbec Gemini 335L configured for visual-inertial odometry.

    ros2 launch vps_uav_benchmarking gemini335l_vio.launch.py

What this sets up and why:
  * stereo IR streams on  -> OpenVINS is a stereo-inertial filter
  * IR dot projector OFF  -> the projected pattern is fixed to the camera, so under
                             motion it slides across the scene and corrupts stereo
                             feature tracking. Turn it off for VIO.
  * synchronised accel+gyro on ONE topic (/camera/gyro_accel/sample) at 400 Hz ->
                             matches config/kalibr_imu_chain.yaml (update_rate: 400)
  * colour + depth off    -> not used by OpenVINS; keeps CPU/USB headroom.
                             (set enable_depth:=true if you also want depth in the bag)
  * header stamps in host/ROS time -> avoids a device-vs-host clock offset

>>> The Orbbec argument NAMES below follow OrbbecSDK_ROS2 (Gemini 330 series).
>>> Confirm them against YOUR installed version:
>>>     ros2 launch orbbec_camera gemini_330_series.launch.py --show-args
>>> and adjust the keys in `launch_arguments` if they differ.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    orbbec_launch = os.path.join(
        get_package_share_directory("orbbec_camera"),
        "launch",
        "gemini_330_series.launch.py",   # 335L is in the 330 series; confirm the filename
    )

    args = {
        "enable_left_ir": "true",
        "enable_right_ir": "true",
        "enable_ir_auto_exposure": "true",
        "enable_laser": "false",                 # IR projector OFF for VIO
        "enable_sync_output_accel_gyro": "true", # -> /camera/gyro_accel/sample
        "enable_accel": "true",
        "enable_gyro": "true",
        "accel_rate": "500hz",
        "gyro_rate": "500hz",
        "accel_range": "4g",
        "gyro_range": "1000dps",
        "enable_color": "false",
        "enable_depth": "false",
        "time_domain": "global",                 # publish ROS/host time on headers
        "enable_sync_host_time": "true"
    }

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(orbbec_launch),
            launch_arguments=args.items(),
        )
    ])
