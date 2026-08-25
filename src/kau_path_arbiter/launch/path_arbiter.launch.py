"""
경로 소스 중재 노드.

    ros2 launch kau_path_arbiter path_arbiter.launch.py

장애물이 없으면 /lane/center 를, 있으면 /path/local 을 골라 /path/drive
하나로 내보낸다. 제어기(kau_control_lane)는 /path/drive 만 보면 된다.

    kau_control_lane/config/lane_source.yaml 에서
        lane.topic: "/path/drive"
        lane.frame: "base_footprint"
    로 맞출 것 (topic 과 frame 은 짝이다).

인자:

    use_sim_time:=false   실기에서 실행할 때 (기본 true)
    release_sec:=2.0      장애물을 마지막으로 본 뒤 회피 모드를 유지할 시간
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory('kau_path_arbiter'))
    config_file = share / 'config' / 'path_arbiter.yaml'

    args = [
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Gazebo 는 true, 실기는 false.'),
        DeclareLaunchArgument(
            'release_sec', default_value='2.0',
            description='장애물을 마지막으로 본 뒤 회피 모드를 유지할 시간 [s]'),
    ]

    node = Node(
        package='kau_path_arbiter',
        executable='path_arbiter_node',
        name='path_arbiter_node',
        output='screen',
        parameters=[
            str(config_file),
            {'use_sim_time': ParameterValue(
                LaunchConfiguration('use_sim_time'), value_type=bool),
             'release_sec': ParameterValue(
                LaunchConfiguration('release_sec'), value_type=float)},
        ],
    )

    return LaunchDescription(args + [node])
