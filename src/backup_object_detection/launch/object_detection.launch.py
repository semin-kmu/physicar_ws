"""backup_object_detection 두 노드 동시 기동.

    ros2 launch backup_object_detection object_detection.launch.py

개별 실행은 obstacle_detector.launch.py / start_signal.launch.py 를 쓴다.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

SHARE = get_package_share_directory('backup_object_detection')


def generate_launch_description():
    args = {'use_sim_time': LaunchConfiguration('use_sim_time'),
            'log_level': LaunchConfiguration('log_level')}
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Gazebo true / 실차 false'),
        DeclareLaunchArgument('log_level', default_value='info'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(SHARE, 'launch', 'start_signal.launch.py')),
            launch_arguments=args.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(SHARE, 'launch', 'obstacle_detector.launch.py')),
            launch_arguments=args.items()),
    ])
