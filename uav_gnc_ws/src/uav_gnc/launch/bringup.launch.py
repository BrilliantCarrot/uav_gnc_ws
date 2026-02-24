from launch import LaunchDescription
from launch.actions import RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = get_package_share_directory('uav_gnc')
    simulation_yaml = os.path.join(pkg, 'config', 'simulation.yaml')
    guidance_yaml = os.path.join(pkg, 'config', 'guidance.yaml')
    control_yaml = os.path.join(pkg, 'config', 'control.yaml')
    navigation_yaml = os.path.join(pkg, 'config', 'navigation.yaml')

    simulation = Node(
        package='uav_gnc',
        executable='simulation_node',   # CMake에서 만든 실행파일 이름
        name='simulation_node',
        output='screen',
        parameters=[simulation_yaml]
    )
    guidance = Node(
        package='uav_gnc',
        executable='guidance_node',
        name='guidance_node',
        output='screen',
        parameters=[guidance_yaml]
    )
    navigation = Node(
        package='uav_gnc',
        executable='navigation_node',
        name='navigation_node',
        output='screen',
        parameters=[navigation_yaml]
    )
    control = Node(
        package='uav_gnc',
        executable='control_node',
        name='control_node',
        output='screen',
        parameters=[control_yaml]
    )
    path_viz_sim = Node(
        package='uav_gnc',
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
        package='uav_gnc',
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
    eval_sim = Node(
    package='uav_gnc',
    executable='tracking_eval_node',
    name='eval_sim',
    output='screen',
    parameters=[{
        'input_odom_topic': '/sim/odom',
        'csv_path': 'sim_tracking_eval.csv',
        'rate_hz': 20.0,
        # guidance.yaml에서 웨이포인트를 바꾸면, 여기서도 수정 필요
        # 사각형
        # 'waypoints_x': [0.0, 5.0, 5.0, 0.0, 0.0],
        # 'waypoints_y': [0.0, 0.0, 5.0, 5.0, 0.0],
        # 칠각형
        'waypoints_x': [0.0, 4.0, 5.0, 5.0, 4.0, 1.0, 0.0, 0.0],
        'waypoints_y': [0.0, 0.0, 1.0, 4.0, 5.0, 5.0, 4.0, 0.0],
        'accept_radius': 2.0,
        }]
    )
    eval_nav = Node(
        package='uav_gnc',
        executable='tracking_eval_node',
        name='eval_nav',
        output='screen',
        parameters=[{
            'input_odom_topic': '/nav/odom',
            'csv_path': 'nav_tracking_eval.csv',
            'rate_hz': 20.0,
            # guidance.yaml에서 웨이포인트를 바꾸면, 여기서도 수정 필요
            # 'waypoints_x': [0.0, 5.0, 5.0, 0.0, 0.0],
            # 'waypoints_y': [0.0, 0.0, 5.0, 5.0, 0.0],
            'waypoints_x': [0.0, 4.0, 5.0, 5.0, 4.0, 1.0, 0.0, 0.0],
            'waypoints_y': [0.0, 0.0, 1.0, 4.0, 5.0, 5.0, 4.0, 0.0],
            'accept_radius': 2.0,
            'auto_exit_on_complete': True,
            'settle_time_sec': 1.0,   # 완주 후 1초만 더 기록하고 종료 (원하면 0으로)
        }]
    )
    # ✅ eval_nav 프로세스가 종료되면, 전체 bringup을 Shutdown
    shutdown_on_eval_done = RegisterEventHandler(
        OnProcessExit(
            target_action=eval_nav,
            on_exit=[EmitEvent(event=Shutdown(reason='Mission complete (eval_nav exited)'))]
        )
    )
    return LaunchDescription([
        simulation,
        guidance,
        navigation,
        control,
        path_viz_sim,
        path_viz_nav,
        eval_sim,
        eval_nav,
        shutdown_on_eval_done
    ])