"""backup_path_planner 단독 기동. lane 콜백 구동이므로 차선이 없으면 발행하지 않는다.

    ros2 launch backup_path_planner path_planner.launch.py

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

SHARE = get_package_share_directory('backup_path_planner')


def generate_launch_description():
    use_sim = ParameterValue(
        LaunchConfiguration('use_sim_time'), value_type=bool)
    log_level = LaunchConfiguration('log_level')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Gazebo true / 실차 false'),
        DeclareLaunchArgument('log_level', default_value='info'),
        Node(package='backup_path_planner', executable='path_planner_node',
             name='backup_path_planner', output='screen',
             parameters=[os.path.join(SHARE, 'config', 'path_planner.yaml'),
                         {'use_sim_time': use_sim}],
             arguments=['--ros-args', '--log-level', log_level]),
    ])
