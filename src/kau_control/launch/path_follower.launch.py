#!/usr/bin/env python3
"""
경로 추종 제어 노드.

physicar_bringup 의 sim.launch.py / real.launch.py 와
kau_localization 의 localization.launch.py 가 떠 있는 상태에서 얹는다.
(map -> odom TF 가 없으면 이 노드는 계속 정지 상태로 남는다)

    ros2 launch kau_control path_follower.launch.py
    ros2 launch kau_control path_follower.launch.py use_sim_time:=false   # 실기
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PACKAGE = 'kau_control'


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory(PACKAGE), 'config', 'path_follower.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    log_level = LaunchConfiguration('log_level')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='시뮬이면 true. 실기는 false',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=config,
            description='parameter YAML 경로',
        ),
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='debug 로 두면 s / cte / Ld / steer 가 2 Hz 로 찍힌다',
        ),
        Node(
            package=PACKAGE,
            executable='path_follower_node',
            name='path_follower',
            output='screen',
            arguments=['--ros-args', '--log-level', log_level],
            parameters=[params_file, {'use_sim_time': use_sim_time}],
        ),
    ])
