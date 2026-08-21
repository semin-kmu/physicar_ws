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

"""Launch the camera-based start signal detector."""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_parameter_file = (
        Path(__file__).resolve().parents[1] / 'config' / 'start_signal_detector.yaml'
    )

    parameter_file_argument = DeclareLaunchArgument(
        'params_file',
        default_value=str(default_parameter_file),
        description='YAML holding the ROI, HSV and shape gates of the green lamp.',
    )
    input_topic_argument = DeclareLaunchArgument(
        'input_topic',
        default_value='/camera/image_raw/compressed',
        description='Compressed camera stream the start signal is read from.',
    )
    output_topic_argument = DeclareLaunchArgument(
        'output_topic',
        default_value='/perception/start_permission',
        description='std_msgs/Bool topic carrying the latched start permission.',
    )
    log_level_argument = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logger level of the detector node.',
    )

    return LaunchDescription([
        parameter_file_argument,
        input_topic_argument,
        output_topic_argument,
        log_level_argument,
        Node(
            package='kau_object_detection',
            executable='start_signal_detector_node',
            name='start_signal_detector',
            output='screen',
            parameters=[
                LaunchConfiguration('params_file'),
                {
                    'input_topic': LaunchConfiguration('input_topic'),
                    'output_topic': LaunchConfiguration('output_topic'),
                },
            ],
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        ),
    ])
