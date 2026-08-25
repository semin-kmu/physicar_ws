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

"""Launch the start signal detection node on its own."""

#     ros2 launch kau_object_detection_lane start_signal_detector.launch.py
#
# Subscribes /camera/image_raw/compressed and publishes
# /perception/start_permission (std_msgs/Bool). The topic contract is unchanged
# from kau_object_detection, so the speed controllers need no reconfiguration.
#
# Camera only: no TF, no map, no odom. This node is independent of localisation
# by construction.
#
# Arguments:
#
#     use_sim_time:=false     on the real vehicle (default true for Gazebo)
#     params_file:=/path.yaml replace the parameter file
#
# DO NOT run this at the same time as kau_object_detection's
# start_signal_detector: both publish /perception/start_permission.

from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


PACKAGE = 'kau_object_detection_lane'


def _default_params():
    return str(
        Path(get_package_share_directory(PACKAGE)) /
        'config' /
        'start_signal_detector.yaml'
    )


def generate_launch_description():
    use_sim_time = ParameterValue(
        LaunchConfiguration('use_sim_time'), value_type=bool)

    return LaunchDescription([

        DeclareLaunchArgument(
            'params_file',
            default_value=_default_params(),
            description='Start signal parameter yaml.',
        ),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='true in Gazebo, false on the real vehicle.',
        ),

        Node(
            package=PACKAGE,
            executable='start_signal_detector_node',
            name='start_signal_detector',
            output='screen',
            parameters=[
                LaunchConfiguration('params_file'),
                {'use_sim_time': use_sim_time},
            ],
        ),
    ])
