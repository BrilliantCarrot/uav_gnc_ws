from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

# ======================================================================
# px4_bringup.launch.py
# PX4 SITL 연동용 launch 파일
# simulation_node 제외 (PX4 SITL이 시뮬레이션 담당)
# navigation_node가 /fmu/out/vehicle_odometry → /nav/odom 변환
# ======================================================================

def generate_launch_description():
    setpoint_topic = LaunchConfiguration('setpoint_topic')
    nav_odom_topic = LaunchConfiguration('nav_odom_topic')
    px4_odom_topic = LaunchConfiguration('px4_odom_topic')
    offboard_mode = LaunchConfiguration('offboard_mode')
    arm_on_start = LaunchConfiguration('arm_on_start')
    takeoff_z_enu = LaunchConfiguration('takeoff_z_enu')
    output_frame_id = LaunchConfiguration('output_frame_id')
    output_child_frame_id = LaunchConfiguration('output_child_frame_id')
    use_planner = LaunchConfiguration('use_planner')
    wait_for_start_waypoint = LaunchConfiguration('wait_for_start_waypoint')
    start_waypoint_accept_radius = LaunchConfiguration('start_waypoint_accept_radius')

    guidance_pkg = get_package_share_directory('uav_guidance')
    guidance_yaml = os.path.join(guidance_pkg, 'config', 'guidance.yaml')

    # guidance_node: 웨이포인트 → /guidance/setpoint 퍼블리시
    guidance = Node(
        package='uav_guidance',
        executable='guidance_node',
        name='guidance_node',
        output='screen',
        parameters=[guidance_yaml, {
            # PX4 bringup은 planner_node를 띄우지 않는 기본 구성이다.
            # 따라서 planner 입력을 기다리지 않고 guidance.yaml의 waypoint trajectory를 직접 사용한다.
            'use_planner': use_planner,
            # PX4는 arm 전에도 odom을 publish하므로, 이륙 전 trajectory 시간이 먼저 흐르지 않게 한다.
            'wait_for_start_waypoint': wait_for_start_waypoint,
            'start_waypoint_accept_radius': start_waypoint_accept_radius,
        }]
    )

    # PX4 odom → /nav/odom 변환 노드 (NED → ENU, 타입 변환 포함)
    px4_odom_converter = Node(
        package='uav_px4_bridge',
        executable='px4_odom_converter',
        name='px4_odom_converter',
        output='screen',
        parameters=[{
            'px4_odom_topic': px4_odom_topic,
            'nav_odom_topic': nav_odom_topic,
            'output_frame_id': output_frame_id,
            'output_child_frame_id': output_child_frame_id,
            'use_px4_timestamp': False,
        }]
    )

    # px4_bridge_node: /guidance/setpoint → /fmu/in/trajectory_setpoint 변환
    px4_bridge = Node(
        package='uav_px4_bridge',
        executable='px4_bridge_node',
        name='px4_bridge_node',
        output='screen',
        parameters=[{
            'setpoint_topic': setpoint_topic,
            'px4_odom_topic': px4_odom_topic,
            'offboard_mode': offboard_mode,
            'arm_on_start': arm_on_start,
            'takeoff_z_enu': takeoff_z_enu,
            'heartbeat_period_ms': 100,
            'offboard_start_count': 10,
        }]
    )

    # path_viz_node: /nav/odom → rviz 시각화
    path_viz_nav = Node(
        package='uav_visualization',
        executable='path_viz_node',
        name='path_viz_nav',
        output='screen',
        parameters=[{
            'input_odom_topic':   nav_odom_topic,
            'output_path_topic':  '/nav/path',
            'output_marker_topic': '/nav/marker',
            'publish_rate_hz':    10.0,
            'history_size':       5000
        }]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'setpoint_topic',
            default_value='/guidance/setpoint',
            description='ROS ENU guidance setpoint topic consumed by px4_bridge_node.'
        ),
        DeclareLaunchArgument(
            'nav_odom_topic',
            default_value='/nav/odom',
            description='ROS ENU odometry topic published from PX4 vehicle_odometry.'
        ),
        DeclareLaunchArgument(
            'px4_odom_topic',
            default_value='/fmu/out/vehicle_odometry',
            description='PX4 vehicle_odometry output topic.'
        ),
        DeclareLaunchArgument(
            'offboard_mode',
            default_value='position_velocity',
            description='PX4 offboard setpoint mode: position, velocity, or position_velocity.'
        ),
        DeclareLaunchArgument(
            'arm_on_start',
            default_value='false',
            description='If true, px4_bridge_node sends offboard mode and arm commands after heartbeat warmup.'
        ),
        DeclareLaunchArgument(
            'takeoff_z_enu',
            default_value='2.0',
            description='Fallback ENU hover altitude before guidance setpoints arrive.'
        ),
        DeclareLaunchArgument(
            'output_frame_id',
            default_value='world',
            description='frame_id used for converted /nav/odom.'
        ),
        DeclareLaunchArgument(
            'output_child_frame_id',
            default_value='base_link',
            description='child_frame_id used for converted /nav/odom.'
        ),
        DeclareLaunchArgument(
            'use_planner',
            default_value='false',
            description='If true, guidance_node consumes /planning/path. PX4 SITL default uses static waypoints.'
        ),
        DeclareLaunchArgument(
            'wait_for_start_waypoint',
            default_value='true',
            description='Delay time-parameterized mission trajectory until the vehicle reaches the first waypoint.'
        ),
        DeclareLaunchArgument(
            'start_waypoint_accept_radius',
            default_value='0.35',
            description='Radius around the first waypoint used to start the PX4 mission trajectory.'
        ),
        guidance,
        px4_odom_converter,
        px4_bridge,
        path_viz_nav,
    ])
