"""본 패키지 2노드만 기동. 타 노드는 system_supervisor 가 직접 spawn (01 section 3)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PKG = 'kau_state_machine'


def generate_launch_description() -> LaunchDescription:
    cfg = os.path.join(get_package_share_directory(PKG), 'config')

    args = [
        DeclareLaunchArgument('bringup_yaml', default_value=os.path.join(cfg, 'bringup.yaml')),
        DeclareLaunchArgument('recovery_yaml', default_value=os.path.join(cfg, 'recovery.yaml')),
        DeclareLaunchArgument(
            'mission_profile_yaml', default_value=os.path.join(cfg, 'mission_profile.yaml')
        ),
    ]

    supervisor = Node(
        package=PKG,
        executable='system_supervisor',
        name='system_supervisor',
        output='screen',
        emulate_tty=True,
        parameters=[
            LaunchConfiguration('bringup_yaml'),
            LaunchConfiguration('recovery_yaml'),
        ],
    )

    state_machine = Node(
        package=PKG,
        executable='state_machine',
        name='state_machine',
        output='screen',
        emulate_tty=True,
        parameters=[LaunchConfiguration('mission_profile_yaml')],
    )

    return LaunchDescription([*args, supervisor, state_machine])
