"""system_supervisor 1개만 기동. 나머지는 supervisor 가 spawn (01 section 2)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

PKG = 'kau_state_machine'


def generate_launch_description() -> LaunchDescription:
    cfg = os.path.join(get_package_share_directory(PKG), 'config')

    args = [
        DeclareLaunchArgument('bringup_yaml', default_value=os.path.join(cfg, 'bringup.yaml')),
        DeclareLaunchArgument('run_id', default_value='',
                              description='로그 디렉터리 이름. 비우면 기동 시각'),
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='전 노드 공통. /clock 을 쓰면 true, 실기는 false'),
    ]

    supervisor = Node(
        package=PKG,
        executable='system_supervisor',
        name='system_supervisor',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'bringup_yaml': LaunchConfiguration('bringup_yaml'),
            'run_id': LaunchConfiguration('run_id'),
            'use_sim_time': ParameterValue(
                LaunchConfiguration('use_sim_time'), value_type=bool),
        }],
    )

    return LaunchDescription([*args, supervisor])
