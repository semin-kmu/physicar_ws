# Copyright 2026 KAU AMET Team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = PathJoinSubstitution(
        [FindPackageShare('kau_object_detection'), 'config', 'laser_scan_validator.yaml']
    )

    # 시계 소스는 launch 인자로만 정한다. 예전에는 yaml 에 use_sim_time: true 가
    # 박혀 있었는데, 그러면 실기에서 끌 방법이 없다 (/clock 이 없어 now() 가
    # 0 에 멈춘다). 다른 런치들(ekf / amcl / global_path)과 같은 규칙이다.
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true', description='Gazebo 는 true, 실기는 false.')

    return LaunchDescription(
        [
            use_sim_time,
            Node(
                package='kau_object_detection',
                executable='laser_scan_validator_node',
                name='laser_scan_validator',
                output='screen',
                parameters=[config_file, {'use_sim_time': ParameterValue(
                    LaunchConfiguration('use_sim_time'), value_type=bool)}],
            )
        ]
    )
