from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_bridge = LaunchConfiguration("use_bridge")
    world = PathJoinSubstitution([
        FindPackageShare("uav_bringup"),
        "worlds",
        "gps_denied_outdoor_lio_vio.world.sdf",
    ])
    model_path = PathJoinSubstitution([
        FindPackageShare("uav_bringup"),
        "models",
    ])
    bridge_config = PathJoinSubstitution([
        FindPackageShare("uav_bringup"),
        "config",
        "gz_lio_vio_bridge.yaml",
    ])

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py",
            ])
        ]),
        launch_arguments={
            "gz_args": ["-r ", world],
        }.items(),
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_lio_vio_bridge",
        output="screen",
        parameters=[{"config_file": bridge_config}],
        condition=IfCondition(use_bridge),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_bridge",
            default_value="true",
            description="Start ros_gz_bridge for LiDAR, IMU, stereo, and RGB-D topics.",
        ),
        SetEnvironmentVariable("IGN_GAZEBO_RESOURCE_PATH", model_path),
        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", model_path),
        gz_sim,
        bridge,
    ])
