"""global path 발행.

    ros2 launch kau_global_path global_path.launch.py
    ros2 launch kau_global_path global_path.launch.py route:=center_loop
    ros2 launch kau_global_path global_path.launch.py path:=/tmp/other.yaml rate:=0.0

기본 경로는 `center_loop` 다 — 차로를 둘로 나눠 보지 않는다 (README 4.1.3).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument('route', default_value='center_loop',
                              description='lane_graph.yaml 의 routes 이름'),
        DeclareLaunchArgument('path', default_value='',
                              description='빈 값이면 패키지 share 의 config/lane_graph.yaml'),
        DeclareLaunchArgument('topic', default_value='/path/global'),
        DeclareLaunchArgument('viz_topic', default_value='/viz/path/global'),
        DeclareLaunchArgument('viz_spacing', default_value='0.05',
                              description='m. 시각화 리샘플 간격'),
        DeclareLaunchArgument('rate', default_value='1.0',
                              description='Hz. 0 이면 1 회만 발행 (latched 라 그래도 받는다)'),
        DeclareLaunchArgument('kappa_limit', default_value='1.8199',
                              description='1/m. 넘으면 경고만 낸다 (README 6.4)'),
    ]

    node = Node(
        package='kau_global_path',
        executable='global_path_publisher.py',
        name='global_path_publisher',
        output='screen',
        parameters=[{
            'route': LaunchConfiguration('route'),
            'path': LaunchConfiguration('path'),
            'topic': LaunchConfiguration('topic'),
            'viz_topic': LaunchConfiguration('viz_topic'),
            'viz_spacing': LaunchConfiguration('viz_spacing'),
            'rate': LaunchConfiguration('rate'),
            'kappa_limit': LaunchConfiguration('kappa_limit'),
        }],
    )

    return LaunchDescription(args + [node])
