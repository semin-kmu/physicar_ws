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
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def detector_node(context, *unused):
    """검출 노드 하나를 만든다.

    값의 주인은 params_file(기본 config/start_signal_detector.yaml)이다.
    토픽 인자는 명시적으로 준 것만 덮는다 -- 빈 값은 "안 줬다"는 뜻이다.
    """
    overrides = {
        'use_sim_time': ParameterValue(
            LaunchConfiguration('use_sim_time'), value_type=bool),
    }
    for key in ('input_topic', 'output_topic'):
        raw = LaunchConfiguration(key).perform(context)
        if raw:
            overrides[key] = raw

    return [Node(
        package='kau_object_detection',
        executable='start_signal_detector_node',
        name='start_signal_detector',
        output='screen',
        parameters=[LaunchConfiguration('params_file'), overrides],
        arguments=['--ros-args', '--log-level',
                   LaunchConfiguration('log_level')],
    )]


def generate_launch_description():
    default_parameter_file = (
        Path(__file__).resolve().parents[1] / 'config' / 'start_signal_detector.yaml'
    )

    parameter_file_argument = DeclareLaunchArgument(
        'params_file',
        default_value=str(default_parameter_file),
        description='YAML holding the ROI, HSV and shape gates of the green lamp.',
    )
    # 아래 둘은 yaml 에 값이 있다. 빈 값이 기본이고, 주면 그때만 yaml 을 덮는다.
    input_topic_argument = DeclareLaunchArgument(
        'input_topic',
        default_value='',
        description='Compressed camera stream the start signal is read from '
                    '(default: yaml).',
    )
    output_topic_argument = DeclareLaunchArgument(
        'output_topic',
        default_value='',
        description='std_msgs/Bool topic carrying the latched start permission '
                    '(default: yaml).',
    )
    log_level_argument = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logger level of the detector node.',
    )
    # 시계 소스는 launch 인자로만 정한다 (laser_scan_validator.launch.py 주석 참고).
    use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time', default_value='true', description='Gazebo 는 true, 실기는 false.')

    return LaunchDescription([
        parameter_file_argument,
        input_topic_argument,
        output_topic_argument,
        log_level_argument,
        use_sim_time_argument,
        OpaqueFunction(function=detector_node),
    ])
