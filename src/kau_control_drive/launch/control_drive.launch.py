"""
/path/drive 기반 주행 제어.

    kau_lane_detection ──(/lane/center, /lane/left, /lane/right)──┐
    kau_object_detection_lane ──(/perception/obstacles)───────────┤
    kau_local_path_planner_lane ──(/path/local)───────────────────┤
                                                                  ▼
                                                        kau_path_arbiter
                                             (장애물 없으면 차선, 있으면 회피)
                                                                  │
                                                     (/path/drive, base_footprint)
                                                                  ▼
                                    drive_speed_controller / drive_steer_controller
                                                                  │
                                                          /speed, /steering

★ kau_control · kau_control_lane 과 **같은 topic(/speed, /steering)** 을 낸다.
  동시에 띄우면 발행자가 셋이 싸운다. 이걸 쓰는 동안 나머지 둘은 내릴 것.

실행:

    # 조향만 관찰 (차는 안 움직인다). 처음엔 반드시 이걸로 시작할 것
    ros2 launch kau_control_drive control_drive.launch.py speed:=0.0

    # 중재 노드까지 같이 띄운다
    ros2 launch kau_control_drive control_drive.launch.py arbiter:=true

주요 인자:

    speed:=0.4          상수 속도 [m/s]. v_min = v_max 로 덮는다.
                        안 주면 yaml 값이 그대로 산다. 0.0 이면 조향만
    arbiter:=false      kau_path_arbiter 도 같이 띄울지
    use_sim_time:=true  실기는 false

★ /path/drive 발행자(kau_path_arbiter)가 없으면 차는 안 움직인다.
  조용히가 아니라 2 초마다 사유를 WARN 으로 찍는다.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _nodes(context):
    share = Path(get_package_share_directory('kau_control_drive'))
    config_file = str(share / 'config' / 'drive_control.yaml')

    use_sim_time = ParameterValue(
        LaunchConfiguration('use_sim_time'), value_type=bool)

    # speed 인자가 있으면 v_min = v_max 로 덮어 상수 속도로 만든다.
    # 빈 문자열이면 yaml 값을 그대로 둔다.
    overrides = {'use_sim_time': use_sim_time}
    speed = context.perform_substitution(LaunchConfiguration('speed')).strip()
    speed_overrides = dict(overrides)
    if speed:
        v = float(speed)
        speed_overrides['speed.v_min'] = v
        speed_overrides['speed.v_max'] = v

    return [
        Node(
            package='kau_control_drive',
            executable='drive_steer_controller',
            name='drive_steer_controller',
            output='screen',
            parameters=[config_file, overrides],
        ),
        Node(
            package='kau_control_drive',
            executable='drive_speed_controller',
            name='drive_speed_controller',
            output='screen',
            parameters=[config_file, speed_overrides],
        ),
    ]


def generate_launch_description():
    arbiter_launch = (
        Path(get_package_share_directory('kau_path_arbiter'))
        / 'launch' / 'path_arbiter.launch.py')

    args = [
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Gazebo 는 true, 실기는 false.'),
        DeclareLaunchArgument(
            'speed', default_value='',
            description='상수 속도 [m/s]. 비우면 yaml 값을 쓴다. 0.0 이면 조향만.'),
        DeclareLaunchArgument(
            'arbiter', default_value='false',
            description='kau_path_arbiter 도 같이 띄울지.'),
    ]

    arbiter = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(arbiter_launch)),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('arbiter')),
    )

    return LaunchDescription(args + [arbiter, OpaqueFunction(function=_nodes)])
