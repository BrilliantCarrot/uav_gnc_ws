from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    start_fast_lio = LaunchConfiguration('start_fast_lio')
    gazebo_world_name = LaunchConfiguration('gazebo_world_name')
    gazebo_model_name = LaunchConfiguration('gazebo_model_name')
    trajectory_mode = LaunchConfiguration('trajectory_mode')
    line_speed_mps = LaunchConfiguration('line_speed_mps')
    yaw_follow_velocity = LaunchConfiguration('yaw_follow_velocity')
    max_yaw_rate_radps = LaunchConfiguration('max_yaw_rate_radps')

    fast_lio2_yaml = os.path.join(
        get_package_share_directory('uav_bringup'),
        'config',
        'fast_lio2_uav_gnc.yaml'
    )

    validation_traj = Node(
        package='uav_bringup',
        executable='lio_validation_trajectory_node',
        name='lio_validation_trajectory_node',
        output='screen',
        parameters=[{
            'odom_topic': '/lio_validation/odom_truth',
            'imu_topic': '/lio/imu',
            'trajectory_mode': trajectory_mode,
            'publish_rate_hz': 100.0,
            'hover_time_s': 3.0,
            'altitude_m': 1.8,
            'line_speed_mps': line_speed_mps,
            'radius_m': 1.5,
            'period_s': 20.0,
            'yaw_follow_velocity': yaw_follow_velocity,
            'max_yaw_rate_radps': max_yaw_rate_radps,
        }]
    )

    pose_sync = Node(
        package='uav_bringup',
        executable='sim_odom_to_gazebo_pose_node',
        name='lio_validation_gazebo_pose_sync',
        output='screen',
        parameters=[{
            'odom_topic': '/lio_validation/odom_truth',
            'world_name': gazebo_world_name,
            'model_name': gazebo_model_name,
            'publish_rate_hz': 100.0,
            'z_offset_m': 0.0,
        }]
    )

    gazebo_lidar_adapter = Node(
        package='uav_perception',
        executable='gazebo_lidar_fastlio_adapter_node',
        name='gazebo_lidar_fastlio_adapter_node',
        output='screen',
        parameters=[{
            'input_topic': '/gazebo/lidar/points_raw',
            'output_topic': '/lidar/points_raw',
            'frame_id': 'lidar',
            'scan_line': 32,
            'scan_rate_hz': 20.0,
            'restamp_with_ros_time': True,
        }]
    )

    fast_lio2 = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        name='fastlio_mapping',
        output='screen',
        condition=IfCondition(start_fast_lio),
        parameters=[fast_lio2_yaml],
        remappings=[
            ('/Odometry', '/lio/odom'),
            ('/path', '/lio/path'),
            ('/cloud_registered', '/lio/cloud_registered'),
            ('/cloud_registered_body', '/lio/cloud_registered_body'),
            ('/cloud_effected', '/lio/cloud_effected'),
            ('/Laser_map', '/lio/map'),
        ]
    )

    validation_error = Node(
        package='uav_bringup',
        executable='lio_validation_error_node',
        name='lio_validation_error_node',
        output='screen',
        parameters=[{
            'truth_topic': '/lio_validation/odom_truth',
            'lio_topic': '/lio/odom',
            'log_period_ms': 1000,
            'min_samples': 5,
            'compare_relative_motion': True,
        }]
    )

    base_to_lidar_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_lidar_tf',
        arguments=['0.0', '0.0', '0.05', '0.0', '0.0', '0.0', 'base_link', 'lidar']
    )

    base_to_imu_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_imu_tf',
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'base_link', 'imu']
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'start_fast_lio',
            default_value='true',
            description='Start fast_lio/fastlio_mapping from a sourced FAST_LIO_ROS2 workspace.'
        ),
        DeclareLaunchArgument(
            'gazebo_world_name',
            default_value='gps_denied_outdoor_lio_vio',
            description='Gazebo world name used by the /world/<name>/set_pose service.'
        ),
        DeclareLaunchArgument(
            'gazebo_model_name',
            default_value='f450_lio_vio_sensor_rig',
            description='Gazebo sensor rig model name to synchronize with validation odometry.'
        ),
        DeclareLaunchArgument(
            'trajectory_mode',
            default_value='line',
            description='LIO validation trajectory mode: line or figure8.'
        ),
        DeclareLaunchArgument(
            'line_speed_mps',
            default_value='0.15',
            description='Low-speed straight-line velocity for line trajectory mode.'
        ),
        DeclareLaunchArgument(
            'yaw_follow_velocity',
            default_value='false',
            description='Rotate the validation rig yaw to follow the horizontal velocity direction.'
        ),
        DeclareLaunchArgument(
            'max_yaw_rate_radps',
            default_value='0.25',
            description='Maximum yaw-rate used by yaw-follow validation trajectory.'
        ),
        validation_traj,
        pose_sync,
        gazebo_lidar_adapter,
        fast_lio2,
        validation_error,
        base_to_lidar_tf,
        base_to_imu_tf,
    ])
