from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    visual_slam_node = ComposableNode(
        name='visual_slam_node',
        package='isaac_ros_visual_slam',
        plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
        parameters=[{
            'denoise_input_images': False,
            'rectified_images': False,          # driver output is raw, not rectified
            'enable_slam_visualization': True,
            'enable_landmarks_view': True,
            'enable_observations_view': True,
            'map_frame': 'map',
            'odom_frame': 'odom',
            'base_frame': 'camera_link',         # confirm this matches your TF tree
            'input_left_camera_frame': 'camera_left_ir_optical_frame',
            'input_right_camera_frame': 'camera_right_ir_optical_frame',
            'input_imu_frame': 'camera_accel_gyro_optical_frame',
            'enable_imu_fusion': True,
            'gyro_noise_density': 0.000244,
            'gyro_random_walk': 0.000019393,
            'accel_noise_density': 0.001862,
            'accel_random_walk': 0.003,
            'calibration_frequency': 200.0,
            'img_jitter_threshold_ms': 34.00,
        }],
        remappings=[
            ('visual_slam/image_0', '/camera/left_ir/image_raw'),
            ('visual_slam/camera_info_0', '/camera/left_ir/camera_info'),
            ('visual_slam/image_1', '/camera/right_ir/image_raw'),
            ('visual_slam/camera_info_1', '/camera/right_ir/camera_info'),
            ('visual_slam/imu', '/camera/gyro_accel/sample'),
        ]
    )

    visual_slam_container = ComposableNodeContainer(
        name='visual_slam_launch_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[visual_slam_node],
        output='screen',
        arguments=['--ros-args', '--log-level', 'info'],
    )

    return LaunchDescription([visual_slam_container])
