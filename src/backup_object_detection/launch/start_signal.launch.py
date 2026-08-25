"""출발 신호 검출 단독 기동. latch 후 구독을 해제한다.

    ros2 launch backup_object_detection start_signal.launch.py
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

SHARE = get_package_share_directory('backup_object_detection')


def generate_launch_description():
    use_sim = ParameterValue(
        LaunchConfiguration('use_sim_time'), value_type=bool)
    log_level = LaunchConfiguration('log_level')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Gazebo true / 실차 false'),
        DeclareLaunchArgument('log_level', default_value='info'),
        Node(package='backup_object_detection', executable='start_signal_node',
             name='backup_start_signal_detector', output='screen',
             parameters=[os.path.join(SHARE, 'config', 'start_signal.yaml'),
                         {'use_sim_time': use_sim}],
             arguments=['--ros-args', '--log-level', log_level]),
    ])
