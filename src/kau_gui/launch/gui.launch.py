"""주행 디버깅 GUI. 관측 전용.

    ros2 launch kau_gui gui.launch.py
    ros2 launch kau_gui gui.launch.py use_sim_time:=false   # 실기

`bringup.yaml` 에는 **넣지 않는다.** supervisor 가 spawn 하는 대상이 아니다
(kau_state_machine/docs/09 section 8). 그래서 이 파일이 config/gui.yaml 을
자동으로 물리는 유일한 경로다 -- 이게 없으면 `ros2 run` 에 매번
`--params-file` 을 손으로 붙여야 하고, 빠뜨리면 코드 기본값으로 조용히 돈다.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


PACKAGE = 'kau_gui'


def generate_launch_description():
    default_params = str(
        Path(get_package_share_directory(PACKAGE)) / 'config' / 'gui.yaml')

    return LaunchDescription([

        DeclareLaunchArgument(
            'params_file', default_value=default_params,
            description='GUI parameter yaml'),

        # 시계 소스. 다른 런치들과 같은 규칙이다.
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Gazebo 는 true, 실기는 false.'),

        Node(
            package=PACKAGE,
            executable='kau_gui',
            name='kau_gui',
            output='screen',
            parameters=[
                LaunchConfiguration('params_file'),
                {'use_sim_time': ParameterValue(
                    LaunchConfiguration('use_sim_time'), value_type=bool)},
            ],
        ),
    ])
