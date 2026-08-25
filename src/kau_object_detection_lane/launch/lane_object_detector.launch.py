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

"""Launch the Localization-free LiDAR Object Detection node on its own."""

#     ros2 launch kau_object_detection_lane lane_object_detector.launch.py
#
# Publishes /perception/obstacles in base_link. No localisation node, no global
# path and no map are started or needed: the only transform this node reads is
# the static base_link <- lidar_link from robot_state_publisher.
#
# Arguments:
#
#     use_sim_time:=false     on the real vehicle (default true for Gazebo)
#     params_file:=/path.yaml replace the parameter file
#     object_list_frame_id:=base_footprint
#                             override the output frame (default base_link)
#     markers:=false          stop advertising the RViz MarkerArray
#
# DO NOT run this at the same time as kau_object_detection's
# laser_scan_clusterer: both publish /perception/obstacles, in different frames.
# See README.md "Never run alongside the map-frame node".

from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


PACKAGE = 'kau_object_detection_lane'


def _default_params():
    return str(
        Path(get_package_share_directory(PACKAGE)) /
        'config' /
        'lane_object_detector.yaml'
    )


def _detector_node(context, *unused):
    """Build the detector node from the yaml plus any explicit overrides."""
    # The parameter file owns the values. A launch argument overrides one only
    # when it was given explicitly; an empty default means "not given", so the
    # yaml keeps winning by default.
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
        parameters=[LaunchConfiguration('params_file'), overrides],
    )]


def generate_launch_description():
    return LaunchDescription([

        DeclareLaunchArgument(
            'params_file',
            default_value=_default_params(),
            description='Detector parameter yaml.',
        ),

        # Gazebo drives /clock; the real vehicle does not. Pinning this in the
        # yaml would make it impossible to turn off on the real vehicle.
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='true in Gazebo, false on the real vehicle.',
        ),

        DeclareLaunchArgument(
            'object_list_frame_id',
            default_value='',
            description=(
                'Object List output frame. Empty keeps the yaml value '
                '(base_link). base_footprint is the only other sensible '
                'value on this vehicle; map is not supported here.'
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
    ])
