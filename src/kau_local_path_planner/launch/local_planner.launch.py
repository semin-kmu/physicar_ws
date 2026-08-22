"""
local path planner 발행.

    ros2 launch kau_local_path_planner local_planner.launch.py

config/local_planner.yaml 을 그대로 쓰되, track_yaml_path 만
kau_object_detection 의 amet_2026_track.yaml 절대경로로 덮어쓴다
(파일을 복제하지 않고 그 패키지의 share 경로를 그대로 참조).
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    own_share = Path(get_package_share_directory('kau_local_path_planner'))
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
    ]

    node = Node(
        package='kau_local_path_planner',
        executable='local_planner_node',
        name='local_planner_node',
        output='screen',
        parameters=[
            str(config_file),
            {'track_yaml_path': LaunchConfiguration('track_yaml_path')},
        ],
    )

    return LaunchDescription(args + [node])
