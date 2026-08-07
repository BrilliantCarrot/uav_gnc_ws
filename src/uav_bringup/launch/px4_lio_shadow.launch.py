from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    start_fast_lio = LaunchConfiguration("start_fast_lio")
    publish_to_px4_ekf = LaunchConfiguration("publish_to_px4_ekf")
    publish_lio_velocity = LaunchConfiguration("publish_lio_velocity")
    bridge_config = os.path.join(
        get_package_share_directory("uav_bringup"),
        "config",
        "px4_lio_shadow_bridge.yaml",
    )
    imu_lio_adapter_yaml = os.path.join(
        get_package_share_directory("uav_perception"),
        "config",
        "imu_lio_adapter.yaml",
    )
    fast_lio2_yaml = os.path.join(
        get_package_share_directory("uav_bringup"),
        "config",
        "fast_lio2_uav_gnc.yaml",
    )

    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="px4_lio_gz_bridge",
        output="screen",
        parameters=[{"config_file": bridge_config}],
    )

    lidar_adapter = Node(
        package="uav_perception",
        executable="gazebo_lidar_fastlio_adapter_node",
        name="gazebo_lidar_fastlio_adapter_node",
        output="screen",
        parameters=[{
            "input_topic": "/gazebo/lidar/points_raw",
            "output_topic": "/lidar/points_raw",
            "frame_id": "lidar",
            "scan_line": 32,
            "scan_rate_hz": 10.0,
            "restamp_with_ros_time": False,
        }],
    )

    imu_adapter = Node(
        package="uav_perception",
        executable="imu_lio_adapter_node",
        name="imu_lio_adapter_node",
        output="screen",
        parameters=[imu_lio_adapter_yaml, {"input_topic": "/gazebo/imu_raw"}],
    )

    fast_lio2 = Node(
        package="fast_lio",
        executable="fastlio_mapping",
        name="fastlio_mapping",
        output="screen",
        condition=IfCondition(start_fast_lio),
        parameters=[fast_lio2_yaml],
        remappings=[
            ("/Odometry", "/lio/odom"),
            ("/path", "/lio/path"),
            ("/cloud_registered", "/lio/cloud_registered"),
            ("/cloud_registered_body", "/lio/cloud_registered_body"),
            ("/cloud_effected", "/lio/cloud_effected"),
            ("/Laser_map", "/lio/map"),
        ],
    )

    lio_vs_px4_error = Node(
        package="uav_bringup",
        executable="lio_validation_error_node",
        name="lio_vs_px4_odom_error_node",
        output="screen",
        parameters=[{
            "truth_topic": "/nav/odom",
            "lio_topic": "/lio/odom",
            "log_period_ms": 1000,
            "min_samples": 5,
            "compare_relative_motion": True,
        }],
    )

    lio_to_px4_ev = Node(
        package="uav_px4_bridge",
        executable="lio_to_px4_visual_odometry",
        name="lio_to_px4_visual_odometry",
        output="screen",
        condition=IfCondition(publish_to_px4_ekf),
        parameters=[{
            "input_topic": "/lio/odom",
            "output_topic": "/fmu/in/vehicle_visual_odometry",
            "publish_orientation": False,
            "publish_velocity": publish_lio_velocity,
            "position_variance": 0.04,
            "velocity_variance": 0.09,
            "quality": 100,
        }],
    )

    base_to_lidar_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_to_lidar_tf",
        arguments=["0.0", "0.0", "0.14", "0.0", "0.0", "0.0", "base_link", "lidar"],
    )

    base_to_imu_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_to_imu_tf",
        arguments=["0.0", "0.0", "0.02", "0.0", "0.0", "0.0", "base_link", "imu"],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "start_fast_lio",
            default_value="true",
            description="Start fast_lio/fastlio_mapping. Source external/fast_lio2_install/setup.bash first.",
        ),
        DeclareLaunchArgument(
            "publish_to_px4_ekf",
            default_value="false",
            description="Publish /lio/odom to PX4 EKF2 as /fmu/in/vehicle_visual_odometry.",
        ),
        DeclareLaunchArgument(
            "publish_lio_velocity",
            default_value="true",
            description="Include LIO velocity in the PX4 external vision message.",
        ),
        gz_bridge,
        lidar_adapter,
        imu_adapter,
        fast_lio2,
        lio_vs_px4_error,
        lio_to_px4_ev,
        base_to_lidar_tf,
        base_to_imu_tf,
    ])
