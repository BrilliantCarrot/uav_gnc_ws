from launch import LaunchDescription
from launch.actions import RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml

"""
bringup_with_path_logger.launch.py

이 파일은 UAV GNC 고도화 프로젝트 Week 8 실험을 실행하기 위한 ROS2 launch 파일임.

기존 bringup.launch.py는 simulation, navigation, guidance, control, LiDAR perception,
occupancy projection, D* Lite planner, visualization, tracking evaluation 노드를 함께 실행했음.
하지만 기존 구성만으로는 path_planner_node가 실제로 퍼블리시하는 /planning/path를
CSV로 저장할 수 없었기 때문에, 나중에 시각화할 때 실제 D* Lite keypoints를 복원하기 어려웠음.

이 launch 파일은 기존 bringup 구성에 planning_path_logger_node를 추가한 버전임.
planning_path_logger_node는 /planning/path 토픽을 구독하고, D* Lite planner가 생성한
경로 안의 waypoint/keypoint들을 planning_path_log.csv로 저장함.

이를 통해 실험 종료 후 Python 시각화 코드에서 다음 정보를 함께 표시할 수 있음.

  - 실제 sim trajectory
  - navigation filter가 추정한 nav trajectory
  - 장애물 위치
  - D* Lite가 생성한 실제 keypoints/path

즉, 단순히 드론이 목표점까지 이동했다는 것뿐 아니라,
LiDAR 기반 occupancy update와 D* Lite planner가 장애물 회피 경로를 생성했고,
그 경로를 guidance/control이 따라갔다는 것을 README/포트폴리오용 그림으로 보여주기 위한 launch 파일임.

실행되는 주요 노드 역할:

  simulation_node:
    자체 6-DOF 시뮬레이터.
    /sim/odom, /sim/imu, /sim/gps/pos 등을 생성함.

  navigation_node:
    EKF/UKF 기반 항법 노드.
    IMU prediction과 GPS 또는 LiDAR pose correction을 이용해 /nav/odom을 생성함.

  guidance_node:
    planner 또는 waypoint 기반 경로를 받아 추종 setpoint/trajectory를 생성함.

  control_node:
    /nav/odom과 guidance setpoint를 이용해 드론 제어 명령을 생성함.

  virtual_lidar_node:
    /sim/odom 기반으로 가상 3D LiDAR point cloud를 생성함.

  lidar_preprocess_node:
    LiDAR point cloud를 필터링하고 전처리함.

  lidar_pose_correction_node:
    LiDAR 관측이 충분할 때 LiDAR-derived pose correction을 /lidar/pose_odom으로 발행함.
    Week 8의 GPS-denied LiDAR-aided navigation 실험에서 navigation_node의 보정 관측값으로 사용됨.

  occupancy_projection_node:
    3D point cloud를 비행 고도 기준으로 slice/filtering하여 2.5D occupancy grid로 투영함.

  path_planner_node:
    occupancy update와 현재 위치를 이용해 D* Lite 기반 경로를 생성하고 /planning/path를 발행함.

  planning_path_logger_node:
    /planning/path를 CSV로 저장하는 로깅 노드.
    실제 D* Lite keypoints를 나중에 시각화하기 위해 사용됨.

  path_viz_node:
    /sim/odom, /nav/odom 궤적을 RViz에서 path/marker로 시각화함.

  tracking_eval_node:
    경로 추종 오차, 완료 여부, 완료 시간 등을 CSV로 저장함.

주의사항:

  - 이 파일을 사용하려면 planning_path_logger_node가 CMakeLists.txt에 빌드 대상으로 등록되어 있어야 함.
  - config/planning_path_logger.yaml 파일이 존재해야 함.
  - config/lidar_pose_correction.yaml 파일도 포함되어 있어야 함.
  - 실험이 끝나면 planning_path_log.csv가 덮어쓰기될 수 있으므로,
    각 케이스 실행 후 planning_path_gps_ekf.csv 같은 이름으로 따로 저장하는 것이 좋음.

사용 예:

  ros2 launch uav_gnc bringup_with_path_logger.launch.py

실험 후 예시 저장:

  cp ~/uav_gnc_ws/planning_path_log.csv ~/uav_gnc_ws/planning_path_lidar_aided_ekf.csv
"""

def generate_launch_description():
    pkg = get_package_share_directory('uav_gnc')
    simulation_yaml = os.path.join(pkg, 'config', 'simulation.yaml')
    guidance_yaml = os.path.join(pkg, 'config', 'guidance.yaml')
    control_yaml = os.path.join(pkg, 'config', 'control.yaml')
    navigation_yaml = os.path.join(pkg, 'config', 'navigation.yaml')

    planner_yaml = os.path.join(pkg, 'config', 'planner.yaml')
    planning_path_logger_yaml = os.path.join(pkg, 'config', 'planning_path_logger.yaml')

    virtual_lidar_yaml = os.path.join(pkg, 'config', 'virtual_lidar.yaml')
    lidar_preprocess_yaml = os.path.join(pkg, 'config', 'lidar_preprocess.yaml')
    occupancy_projection_yaml = os.path.join(pkg, 'config', 'occupancy_projection.yaml')
    lidar_pose_correction_yaml = os.path.join(pkg, 'config', 'lidar_pose_correction.yaml')

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
    planning_path_logger = Node(
        package='uav_gnc',
        executable='planning_path_logger_node',
        name='planning_path_logger_node',
        output='screen',
        parameters=[planning_path_logger_yaml]
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
    lidar_pose_correction = Node(
    package='uav_gnc',
    executable='lidar_pose_correction_node',
    name='lidar_pose_correction_node',
    output='screen',
    parameters=[lidar_pose_correction_yaml]
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
        lidar_pose_correction,
        occupancy_projection,
        planner,
        planning_path_logger,
        path_viz_sim,
        path_viz_nav,
        eval_sim,
        eval_nav,
        shutdown_on_eval_done
    ])