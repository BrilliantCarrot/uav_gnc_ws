import os
import launch
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from datetime import datetime


def launch_setup(context, *args, **kwargs):
    # 실험 이름 가져옴 (기본값: 'exp')
    exp_name = LaunchConfiguration('exp_name').perform(context)

    # rosbag 저장 경로: bags/실험이름_날짜시간
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    bag_path = os.path.expanduser(
        f'~/master/amr_ws/bags/{exp_name}_{timestamp}'
    )

    # 메트릭 토픽 3개 자동 record
    rosbag_record = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record',
            '/metrics/control_latency_ms',
            '/metrics/pose_rmse',
            '/metrics/min_obstacle_distance',
            '-o', bag_path
        ],
        output='screen'
    )

    return [rosbag_record]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'exp_name',
            default_value='exp',
            description='실험 이름 (bag 파일명에 사용됨)'
        ),
        OpaqueFunction(function=launch_setup)
    ])