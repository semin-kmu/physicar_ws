"""
local path planner 발행.

    ros2 launch kau_local_path_planner_lane local_planner.launch.py

config/local_planner.yaml 을 그대로 쓰되, track_yaml_path 만
kau_object_detection 의 amet_2026_track.yaml 절대경로로 덮어쓴다
(파일을 복제하지 않고 그 패키지의 share 경로를 그대로 참조).

    use_sim_time:=false     실기에서 실행할 때 (기본 true)
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    own_share = Path(get_package_share_directory('kau_local_path_planner_lane'))
    config_file = own_share / 'config' / 'local_planner.yaml'

    object_detection_share = Path(
        get_package_share_directory('kau_object_detection'))
    default_track_yaml = (
        object_detection_share / 'config' / 'amet_2026_track.yaml')

    args = [
        DeclareLaunchArgument(
            'track_yaml_path', default_value=str(default_track_yaml),
            description='도로 outer/inner 경계 폴리곤 yaml '
                        '(kau_object_detection 과 동일 파일 재사용)'),
        # 시계 소스. 없으면 rclcpp 기본값 false 라 시뮬에서도 이 노드만
        # 벽시계로 돌아 plan_hz 타이머와 로그 시각이 다른 노드와 어긋난다.
        DeclareLaunchArgument(
            'use_sim_time', default_value='true', description='Gazebo 는 true, 실기는 false.'),
    ]

    node = Node(
        package='kau_local_path_planner_lane',
        executable='local_planner_node',
        name='local_planner_node',
        output='screen',
        parameters=[
            str(config_file),
            {'track_yaml_path': LaunchConfiguration('track_yaml_path'),
             'use_sim_time': ParameterValue(
                 LaunchConfiguration('use_sim_time'), value_type=bool)},
        ],
    )

    return LaunchDescription(args + [node])
