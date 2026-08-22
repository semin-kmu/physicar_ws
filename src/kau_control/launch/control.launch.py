#!/usr/bin/env python3
"""제어 노드 2개 일괄 실행.

    speed_controller  --> /speed    [m/s]   곡률 기반 목표속도 + PID
    steer_controller  --> /steering [rad]   Pure Pursuit

두 노드는 서로를 구독하지 않는다. 각자 경로를 받고 각자 TF 를 본다
(steer 만 Ld 계산용으로 /speed 를 본다). 한쪽이 죽어도 다른 쪽은 계속
돈다 — 상호 감시는 state_machine 소관이다.

physicar_bringup 의 sim.launch.py / real.launch.py 와 kau_localization 의
localization.launch.py 가 떠 있는 상태에서 얹는다.
(map -> odom TF 가 없으면 두 노드 모두 계속 0 을 발행한다)

    ros2 launch kau_control control.launch.py
    ros2 launch kau_control control.launch.py use_sim_time:=false   # 실기
    ros2 launch kau_control control.launch.py log_level:=debug
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


PACKAGE = 'kau_control'


def config(name):
    return str(Path(get_package_share_directory(PACKAGE)) / 'config' / name)


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    log_level = LaunchConfiguration('log_level')
    speed_params = LaunchConfiguration('speed_params_file')
    steer_params = LaunchConfiguration('steer_params_file')

    common = ['--ros-args', '--log-level', log_level]

    return LaunchDescription([

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='시뮬이면 true. 실기는 false',
        ),

        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='debug 로 두면 제어 내부값이 2 Hz 로 찍힌다',
        ),

        DeclareLaunchArgument(
            'speed_params_file',
            default_value=config('speed_controller.yaml'),
            description='속도 제어 parameter yaml',
        ),

        DeclareLaunchArgument(
            'steer_params_file',
            default_value=config('steer_controller.yaml'),
            description='조향 제어 parameter yaml',
        ),

        Node(
            package=PACKAGE,
            executable='speed_controller_node',
            name='speed_controller',
            output='screen',
            arguments=common,
            parameters=[speed_params, {'use_sim_time': use_sim_time}],
        ),

        Node(
            package=PACKAGE,
            executable='steer_controller_node',
            name='steer_controller',
            output='screen',
            arguments=common,
            parameters=[steer_params, {'use_sim_time': use_sim_time}],
        ),
    ])
