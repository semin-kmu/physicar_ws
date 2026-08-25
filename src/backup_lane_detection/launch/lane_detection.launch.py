"""backup_lane_detector 단독 기동.

    ros2 launch backup_lane_detection lane_detection.launch.py

인자를 붙이지 않아도 떠야 한다. 모든 인자에 기본값이 있고
config/lane_detector.yaml + platform/{sim,real}.yaml 을 자동으로 얹는다.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def _setup(context, *args, **kwargs):
    share = get_package_share_directory('backup_lane_detection')
    use_sim = LaunchConfiguration('use_sim_time').perform(context)
    platform = 'sim' if use_sim.lower() == 'true' else 'real'

    params = [
        os.path.join(share, 'config', 'lane_detector.yaml'),
        os.path.join(share, 'config', 'platform', f'{platform}.yaml'),
        {'use_sim_time': use_sim.lower() == 'true'},
    ]
    return [Node(
        package='backup_lane_detection',
        executable='lane_detector_node',
        name='backup_lane_detector',
        output='screen',
        parameters=params,
        arguments=['--ros-args', '--log-level',
                   LaunchConfiguration('log_level').perform(context)],
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Gazebo true / 실차 false'),
        DeclareLaunchArgument('log_level', default_value='info'),
        OpaqueFunction(function=_setup),
    ])
