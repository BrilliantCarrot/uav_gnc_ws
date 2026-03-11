from launch import LaunchDescription
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
    pkg = get_package_share_directory('uav_gnc')
    guidance_yaml   = os.path.join(pkg, 'config', 'guidance.yaml')
    control_yaml    = os.path.join(pkg, 'config', 'control.yaml')
    navigation_yaml = os.path.join(pkg, 'config', 'navigation.yaml')

    # guidance_node: 웨이포인트 → /guidance/setpoint 퍼블리시
    guidance = Node(
        package='uav_gnc',
        executable='guidance_node',
        name='guidance_node',
        output='screen',
        parameters=[guidance_yaml]
    )

    # PX4 odom → /nav/odom 변환 노드 (NED → ENU, 타입 변환 포함)
    px4_odom_converter = Node(
        package='uav_gnc',
        executable='px4_odom_converter',
        name='px4_odom_converter',
        output='screen',
    )

    # px4_bridge_node: /guidance/setpoint → /fmu/in/trajectory_setpoint 변환
    px4_bridge = Node(
        package='uav_gnc',
        executable='px4_bridge_node',
        name='px4_bridge_node',
        output='screen',
        parameters=[{
            'arm_on_start': False,    # arming은 pxh>에서 수동으로
            'takeoff_z_enu': 2.0,     # 기본 호버링 고도 2m
        }]
    )

    # path_viz_node: /nav/odom → rviz 시각화
    path_viz_nav = Node(
        package='uav_gnc',
        executable='path_viz_node',
        name='path_viz_nav',
        output='screen',
        parameters=[{
            'input_odom_topic':   '/nav/odom',
            'output_path_topic':  '/nav/path',
            'output_marker_topic': '/nav/marker',
            'publish_rate_hz':    10.0,
            'history_size':       5000
        }]
    )

    return LaunchDescription([
        guidance,
        px4_odom_converter,
        px4_bridge,
        path_viz_nav,
    ])