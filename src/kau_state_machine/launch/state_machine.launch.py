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
        # bringup.yaml 의 option 스위치. 노드의 `option: lane_viewer` 와 짝이다.
        # run.sh 는 이걸 안 쓴다 -- 거기서는 KAU_NODES 표가 뷰어까지 다룬다.
        # launch 를 직접 부르는 쪽을 위해 남겨 둔다.
        DeclareLaunchArgument('lane_viewer', default_value='true',
                              description='차선 인지 BEV 웹 뷰어(포트 5000) 실행 여부'),
        # 이번 기동에서 뺄 노드 이름(bringup.yaml 의 name)을 쉼표로 잇는다.
        # option 과 달리 주행 필수 노드도 뺄 수 있다 -- supervisor 가 stderr 로
        # 경고를 남기고, 없는 이름을 주면 기동을 세운다.
        #   ros2 launch ... skip:=kau_lane_detection_node,kau_lane_detection_viewer
        DeclareLaunchArgument('skip', default_value='',
                              description='기동에서 뺄 노드 이름. 쉼표 구분'),
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
            'option.lane_viewer': ParameterValue(
                LaunchConfiguration('lane_viewer'), value_type=bool),
            'skip': ParameterValue(LaunchConfiguration('skip'), value_type=str),
        }],
    )

    return LaunchDescription([*args, supervisor])
