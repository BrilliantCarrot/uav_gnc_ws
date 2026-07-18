from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    actuator_mode = LaunchConfiguration('actuator_mode')
    use_lio_test_source = LaunchConfiguration('use_lio_test_source')
    start_fast_lio = LaunchConfiguration('start_fast_lio')

    dynamics_pkg = get_package_share_directory('uav_dynamics')
    guidance_pkg = get_package_share_directory('uav_guidance')
    control_pkg = get_package_share_directory('uav_control')
    navigation_pkg = get_package_share_directory('uav_navigation')
    perception_pkg = get_package_share_directory('uav_perception')

    simulation_yaml = os.path.join(dynamics_pkg, 'config', 'simulation.yaml')
    guidance_yaml = os.path.join(guidance_pkg, 'config', 'guidance.yaml')
    control_yaml = os.path.join(control_pkg, 'config', 'control.yaml')
    raw_lidar_yaml = os.path.join(perception_pkg, 'config', 'raw_lidar_sim.yaml')
    imu_lio_adapter_yaml = os.path.join(perception_pkg, 'config', 'imu_lio_adapter.yaml')
    localization_manager_yaml = os.path.join(navigation_pkg, 'config', 'localization_manager.yaml')
    fast_lio2_yaml = os.path.join(
        get_package_share_directory('uav_bringup'),
        'config',
        'fast_lio2_uav_gnc.yaml'
    )

    simulation = Node(
        package='uav_dynamics',
        executable='simulation_node',
        name='simulation_node',
        output='screen',
        parameters=[simulation_yaml, {'actuator_mode': actuator_mode}]
    )

    raw_lidar = Node(
        package='uav_perception',
        executable='raw_lidar_sim_node',
        name='raw_lidar_sim_node',
        output='screen',
        parameters=[raw_lidar_yaml]
    )

    imu_lio_adapter = Node(
        package='uav_perception',
        executable='imu_lio_adapter_node',
        name='imu_lio_adapter_node',
        output='screen',
        parameters=[imu_lio_adapter_yaml]
    )

    # LIO backend is intentionally external. Run FAST-LIO/LIO-SAM separately and remap its odom to /lio/odom.
    localization_manager = Node(
        package='uav_navigation',
        executable='localization_manager_node',
        name='localization_manager_node',
        output='screen',
        parameters=[localization_manager_yaml]
    )

    lio_test_source = Node(
        package='uav_navigation',
        executable='lio_odom_test_source_node',
        name='lio_odom_test_source_node',
        output='screen',
        condition=IfCondition(use_lio_test_source),
        parameters=[{
            'truth_odom_topic': '/sim/odom',
            'output_odom_topic': '/lio/odom',
            'output_frame_id': 'world',
            'output_child_frame_id': 'base_link',
            'position_noise_std': 0.03,
            'velocity_noise_std': 0.02,
            'dropout_rate': 0.0,
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

    guidance = Node(
        package='uav_guidance',
        executable='guidance_node',
        name='guidance_node',
        output='screen',
        parameters=[guidance_yaml]
    )

    control = Node(
        package='uav_control',
        executable='control_node',
        name='control_node',
        output='screen',
        parameters=[control_yaml]
    )

    path_viz_sim = Node(
        package='uav_visualization',
        executable='path_viz_node',
        name='path_viz_sim',
        output='screen',
        parameters=[{
            'input_odom_topic': '/sim/odom',
            'output_path_topic': '/sim/path',
            'output_marker_topic': '/sim/marker',
            'publish_rate_hz': 10.0,
            'history_size': 5000
        }]
    )

    path_viz_nav = Node(
        package='uav_visualization',
        executable='path_viz_node',
        name='path_viz_nav',
        output='screen',
        parameters=[{
            'input_odom_topic': '/nav/odom',
            'output_path_topic': '/nav/path',
            'output_marker_topic': '/nav/marker',
            'publish_rate_hz': 10.0,
            'history_size': 5000
        }]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'actuator_mode',
            default_value='multirotor',
            description='Simulation actuator mode: direct_wrench or multirotor'
        ),
        DeclareLaunchArgument(
            'use_lio_test_source',
            default_value='false',
            description='Start a fake /lio/odom publisher for interface testing when FAST-LIO2 is not running.'
        ),
        DeclareLaunchArgument(
            'start_fast_lio',
            default_value='false',
            description='Start fast_lio/fastlio_mapping from a sourced FAST_LIO_ROS2 workspace.'
        ),
        simulation,
        raw_lidar,
        imu_lio_adapter,
        base_to_lidar_tf,
        base_to_imu_tf,
        localization_manager,
        lio_test_source,
        fast_lio2,
        guidance,
        control,
        path_viz_sim,
        path_viz_nav,
    ])
