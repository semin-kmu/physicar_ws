"""backup_steer_controller 단독 기동. /steering 을 발행한다 -- 기존 스택과 동시 실행 금지.

    ros2 launch backup_steer_controller steer_controller.launch.py

인자를 붙이지 않아도 떠야 한다. 모든 인자에 기본값이 있고
config/*.yaml 을 자동으로 얹는다.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

SHARE = get_package_share_directory('backup_steer_controller')


def generate_launch_description():
    use_sim = ParameterValue(
        LaunchConfiguration('use_sim_time'), value_type=bool)
    log_level = LaunchConfiguration('log_level')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Gazebo true / 실차 false'),
        DeclareLaunchArgument('log_level', default_value='info'),
        Node(package='backup_steer_controller', executable='steer_controller_node',
             name='backup_steer_controller', output='screen',
             parameters=[os.path.join(SHARE, 'config', 'steer_controller.yaml'),
                         {'use_sim_time': use_sim}],
             arguments=['--ros-args', '--log-level', log_level]),
    ])
