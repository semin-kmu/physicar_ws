"""
local path planner 발행 (lane-only).

    ros2 launch kau_local_path_planner_lane local_planner.launch.py

/lane/center, /lane/left, /lane/right (KauPath, base_link) 와
/perception/obstacles, /odom 만으로 /path/local 을 만든다.
map / global path / localization 은 쓰지 않는다.

    use_sim_time:=false     실기에서 실행할 때 (기본 true)
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    own_share = Path(get_package_share_directory('kau_local_path_planner_lane'))
    config_file = own_share / 'config' / 'local_planner.yaml'

    args = [
        # 시계 소스. 없으면 rclcpp 기본값 false 라 시뮬에서도 이 노드만
        # 벽시계로 돌아 plan_hz 타이머와 로그 시각이 다른 노드와 어긋난다.
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Gazebo 는 true, 실기는 false.'),
    ]

    node = Node(
        package='kau_local_path_planner_lane',
        executable='local_planner_node',
        name='local_planner_node',
        output='screen',
        parameters=[
            str(config_file),
            {'use_sim_time': ParameterValue(
                LaunchConfiguration('use_sim_time'), value_type=bool)},
        ],
    )

    return LaunchDescription(args + [node])
