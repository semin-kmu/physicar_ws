"""backup_gui 단독 기동. 관측 전용.

    ros2 launch backup_gui gui.launch.py
    ros2 launch backup_gui gui.launch.py use_sim_time:=false   # 실차

인자를 붙이지 않아도 떠야 한다. 모든 인자에 기본값이 있고
config/gui.yaml 을 자동으로 얹는다 -- 빠뜨리면 코드 기본값으로 조용히 돈다.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

SHARE = get_package_share_directory('backup_gui')


def generate_launch_description():
    use_sim = ParameterValue(
        LaunchConfiguration('use_sim_time'), value_type=bool)
    log_level = LaunchConfiguration('log_level')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Gazebo true / 실차 false'),
        DeclareLaunchArgument('log_level', default_value='info'),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(SHARE, 'config', 'gui.yaml'),
            description='GUI parameter yaml'),
        Node(package='backup_gui', executable='backup_gui',
             name='backup_gui', output='screen',
             parameters=[LaunchConfiguration('params_file'),
                         {'use_sim_time': use_sim}],
             arguments=['--ros-args', '--log-level', log_level]),
    ])
