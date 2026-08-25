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

"""Launch both perception nodes of the Localization-free Lane stack."""

#     ros2 launch kau_object_detection_lane perception_lane.launch.py
#
# Starts, and starts nothing else:
#
#     /lane_laser_object_detector   /perception/obstacles       (base_link)
#     /start_signal_detector        /perception/start_permission
#
# No localisation, no map server, no global path, no planner and no controller
# are started here. That is the point of the package: this launch file brings up
# everything Object Detection owes the Lane-only stack and nothing it does not.
#
# Arguments:
#
#     use_sim_time:=false          on the real vehicle (default true for Gazebo)
#     lidar:=false                 skip the LiDAR detector
#     start_signal:=false          skip the start signal detector
#     object_list_frame_id:=base_footprint
#                                  override the Object List frame
#     markers:=false               stop advertising the RViz MarkerArray
#     detector_params_file:=/path.yaml
#     start_signal_params_file:=/path.yaml
#
# DO NOT run this at the same time as kau_object_detection's launch files: both
# packages publish /perception/obstacles and /perception/start_permission. This
# launch file will not kill anything for you; stop the other stack yourself. See
# README.md "Never run alongside the map-frame node".

from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


PACKAGE = 'kau_object_detection_lane'


def _config(name):
    return str(Path(get_package_share_directory(PACKAGE)) / 'config' / name)


def _detector_node(context, *unused):
    """Build the LiDAR detector with only the overrides that were given."""
    overrides = {
        'use_sim_time': ParameterValue(
            LaunchConfiguration('use_sim_time'), value_type=bool),
    }

    frame_id = LaunchConfiguration('object_list_frame_id').perform(context)
    if frame_id:
        overrides['object_list_frame_id'] = frame_id

    markers = LaunchConfiguration('markers').perform(context)
    if markers:
        overrides['obstacle_marker_enabled'] = markers.lower() == 'true'

    return [Node(
        package=PACKAGE,
        executable='lane_laser_object_detector_node',
        name='lane_laser_object_detector',
        output='screen',
        parameters=[LaunchConfiguration('detector_params_file'), overrides],
        condition=IfCondition(LaunchConfiguration('lidar')),
    )]


def generate_launch_description():
    use_sim_time = ParameterValue(
        LaunchConfiguration('use_sim_time'), value_type=bool)

    return LaunchDescription([

        DeclareLaunchArgument(
            'detector_params_file',
            default_value=_config('lane_object_detector.yaml'),
            description='LiDAR detector parameter yaml.',
        ),

        DeclareLaunchArgument(
            'start_signal_params_file',
            default_value=_config('start_signal_detector.yaml'),
            description='Start signal parameter yaml.',
        ),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='true in Gazebo, false on the real vehicle.',
        ),

        DeclareLaunchArgument(
            'lidar',
            default_value='true',
            description='Run the LiDAR Object Detection node.',
        ),

        DeclareLaunchArgument(
            'start_signal',
            default_value='true',
            description='Run the start signal detection node.',
        ),

        DeclareLaunchArgument(
            'object_list_frame_id',
            default_value='',
            description=(
                'Object List output frame. Empty keeps the yaml value '
                '(base_link).'
            ),
        ),

        DeclareLaunchArgument(
            'markers',
            default_value='',
            description=(
                'Advertise /perception/obstacle_markers. Empty keeps the '
                'yaml value (true). Visualisation only.'
            ),
        ),

        OpaqueFunction(function=_detector_node),

        Node(
            package=PACKAGE,
            executable='start_signal_detector_node',
            name='start_signal_detector',
            output='screen',
            parameters=[
                LaunchConfiguration('start_signal_params_file'),
                {'use_sim_time': use_sim_time},
            ],
            condition=IfCondition(LaunchConfiguration('start_signal')),
        ),
    ])
