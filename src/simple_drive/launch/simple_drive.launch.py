"""
simple_drive 판단 + 제어 기동.

    ros2 launch simple_drive simple_drive.launch.py
    ros2 launch simple_drive simple_drive.launch.py use_sim_time:=false

인지는 띄우지 않는다. backup 스택 인지 노드를 그대로 쓴다:

    source run_backup.sh   # 단, BACKUP_NODES 에서 아래 셋을 false 로
        backup_path_planner       false
        backup_speed_controller   false
        backup_steer_controller   false

★ 위 셋을 끄지 않으면 /speed 와 /steering 을 두 곳에서 쏜다.
  값이 섞여 차량에 가므로 반드시 확인하고 띄운다.

이 launch 는 Node 를 직접 나열한다 -- 다른 launch 를 include 하지
않는다. include 사슬로 띄우면 Ctrl-C 때 고아 프로세스가 남는다.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('simple_drive'), 'config', 'simple_drive.yaml')

    use_sim_time = ParameterValue(
        LaunchConfiguration('use_sim_time'), value_type=bool)
    # 평가 러너 없이 그냥 띄우면 신호등이 계속 초록이라 빨강->초록
    # 전이가 영영 안 온다. 그때 WAIT_RED_FIRST 에서 안 나가므로
    # 주행 튜닝 중에는 이 인자로 끈다.
    red_to_green = ParameterValue(
        LaunchConfiguration('require_red_to_green'), value_type=bool)
    log_level = LaunchConfiguration('log_level')

    # 판단 -> 제어 순서로 띄운다. 제어가 먼저 뜨면 워치독이 곧바로
    # "지시 끊김" 경고를 뱉는데, 정상 기동과 실제 장애를 구분 못 하게
    # 된다.
    decision = Node(
        package='simple_drive',
        executable='simple_decision_node',
        name='simple_decision',
        output='screen',
        parameters=[params, {'use_sim_time': use_sim_time,
                             'require_red_to_green': red_to_green}],
        arguments=['--ros-args', '--log-level', log_level],
    )

    control = Node(
        package='simple_drive',
        executable='simple_control_node',
        name='simple_control',
        output='screen',
        parameters=[params, {'use_sim_time': use_sim_time}],
        arguments=['--ros-args', '--log-level', log_level],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Gazebo true / 실차 false'),
        DeclareLaunchArgument('log_level', default_value='info'),
        DeclareLaunchArgument(
            'require_red_to_green', default_value='true',
            description='빨강->초록 전이를 봐야 출발. 평가 없이 튜닝할 때만 false'),
        decision,
        control,
    ])
