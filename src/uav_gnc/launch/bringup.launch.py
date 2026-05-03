from launch import LaunchDescription
from launch.actions import RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def generate_launch_description():
    pkg = get_package_share_directory('uav_gnc')
    simulation_yaml = os.path.join(pkg, 'config', 'simulation.yaml')
    guidance_yaml = os.path.join(pkg, 'config', 'guidance.yaml')
    control_yaml = os.path.join(pkg, 'config', 'control.yaml')
    navigation_yaml = os.path.join(pkg, 'config', 'navigation.yaml')

    planner_yaml = os.path.join(pkg, 'config', 'planner.yaml')

    virtual_lidar_yaml = os.path.join(pkg, 'config', 'virtual_lidar.yaml')
    lidar_preprocess_yaml = os.path.join(pkg, 'config', 'lidar_preprocess.yaml')
    occupancy_projection_yaml = os.path.join(pkg, 'config', 'occupancy_projection.yaml')

    with open(planner_yaml, 'r') as f:
        planner_cfg = yaml.safe_load(f)

    planner_params = planner_cfg['path_planner_node']['ros__parameters']
    planner_goal_x = planner_params['goal_x']
    planner_goal_y = planner_params['goal_y']
    planner_fly_alt = planner_params['fly_altitude']

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
    # 노드 정의 추가
    planner = Node(
        package='uav_gnc',
        executable='path_planner_node',
        name='path_planner_node',
        output='screen',
        parameters=[planner_yaml]
    )
    virtual_lidar = Node(
    package='uav_gnc',
    executable='virtual_lidar_node',
    name='virtual_lidar_node',
    output='screen',
    parameters=[virtual_lidar_yaml]
    )
    lidar_preprocess = Node(
    package='uav_gnc',
    executable='lidar_preprocess_node',
    name='lidar_preprocess_node',
    output='screen',
    parameters=[lidar_preprocess_yaml]
    )
    occupancy_projection = Node(
        package='uav_gnc',
        executable='occupancy_projection_node',
        name='occupancy_projection_node',
        output='screen',
        parameters=[occupancy_projection_yaml]
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
    # eval_sim = Node(
    # package='uav_gnc',
    # executable='tracking_eval_node',
    # name='eval_sim',
    # output='screen',
    # parameters=[{
    #     'input_odom_topic': '/sim/odom',
    #     'csv_path': '/home/lyj/uav_gnc_ws/sim_tracking_eval.csv',
    #     'rate_hz': 20.0,
    #     # 3D 웨이포인트로 변경함
    #     'waypoints_x': [0.0, 4.0, 6.0, 6.0, 4.0, 0.0, -2.0, -2.0, 0.0],
    #     'waypoints_y': [0.0, 0.0, 2.0, 6.0, 8.0, 8.0, 6.0, 2.0, 0.0],
    #     'waypoints_z': [2.0, 1.5, 1.0, 1.5, 2.0, 1.5, 1.0, 1.5, 2.0],
    #     'accept_radius': 3.0, # 원래 1.0, MPC는 3.0
    #     }]
    # )
    
    # eval_nav = Node(
    #     package='uav_gnc',
    #     executable='tracking_eval_node',
    #     name='eval_nav',
    #     output='screen',
    #     parameters=[{
    #         'input_odom_topic': '/nav/odom',
    #         'csv_path': '/home/lyj/uav_gnc_ws/nav_tracking_eval.csv',
    #         'rate_hz': 20.0,
    #         # 3D 웨이포인트로 변경함
    #         'waypoints_x': [0.0, 4.0, 6.0, 6.0, 4.0, 0.0, -2.0, -2.0, 0.0],
    #         'waypoints_y': [0.0, 0.0, 2.0, 6.0, 8.0, 8.0, 6.0, 2.0, 0.0],
    #         'waypoints_z': [2.0, 1.5, 1.0, 1.5, 2.0, 1.5, 1.0, 1.5, 2.0],
    #         'accept_radius': 3.0, # 원래 1.0
    #         'auto_exit_on_complete': True,
    #         'settle_time_sec': 1.0, # MPC는 3.0
    #     }]
    # )
    eval_sim = Node(
        package='uav_gnc',
        executable='tracking_eval_node',
        name='eval_sim',
        output='screen',
        parameters=[{
            'input_odom_topic': '/sim/odom',
            'csv_path': '/home/lyj/uav_gnc_ws/sim_tracking_eval.csv',
            'rate_hz': 20.0,
            'waypoints_x': [0.0, planner_goal_x],
            'waypoints_y': [0.0, planner_goal_y],
            'waypoints_z': [planner_fly_alt, planner_fly_alt],
            'accept_radius': 1.0,
        }]
    )

    eval_nav = Node(
        package='uav_gnc',
        executable='tracking_eval_node',
        name='eval_nav',
        output='screen',
        parameters=[{
            'input_odom_topic': '/nav/odom',
            'csv_path': '/home/lyj/uav_gnc_ws/nav_tracking_eval.csv',
            'rate_hz': 20.0,
            'waypoints_x': [0.0, planner_goal_x],
            'waypoints_y': [0.0, planner_goal_y],
            'waypoints_z': [planner_fly_alt, planner_fly_alt],
            'accept_radius': 1.0,
            'auto_exit_on_complete': True,
            'settle_time_sec': 1.0,
        }]
    )
    
    # eval_nav 프로세스가 종료되면, 전체 bringup을 Shutdown
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
        virtual_lidar,
        lidar_preprocess,
        occupancy_projection,
        planner,
        path_viz_sim,
        path_viz_nav,
        eval_sim,
        eval_nav,
        shutdown_on_eval_done
    ])