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

"""차선 인지 일괄 실행.

터미널을 따로 열어 ros_gz_bridge 를 띄울 필요가 없도록,
CameraInfo bridge + 인지 노드 + 웹 뷰어를 한 번에 올린다.

    ros2 launch kau_lane_detection lane_detection.launch.py

Gazebo 는 /camera/image_raw 와 /camera/camera_info 를 모두 내보내지만,
ROS 쪽에는 image_raw 만 bridge 되어 있어서 camera_info 가 비어 있었다.
노드는 CameraInfo 가 와야 undistort 를 시작하므로
("Waiting for CameraInfo..." 로 멈춘다) 이 bridge 가 필수다.

주요 인자:

    use_sim_time:=false         실기에서 실행할 때 (기본 true)
    camera_info_bridge:=false   다른 곳에서 이미 bridge 중일 때
    image_bridge:=true          image_raw 도 직접 bridge 해야 할 때
    viewer:=false               웹 뷰어(포트 5000) 없이 노드만
    params_file:=/path/to.yaml  다른 파라미터 파일로 교체
    pan_search:=false           카메라 pan 탐색 끄기 (기본 켜짐)
    pan_aim:=false              카메라 pan 조준 끄기 (기본 켜짐)

pan_search 는 노드가 생성자에서 한 번만 읽는 값이라
ros2 param set 으로는 바꿀 수 없다. 기동 시점에 넣어야 한다.

기본 켜짐이다. 탐색/유지/복귀 동안에는 카메라 자세가 BEV 의
전제(pan=0)와 어긋나므로 그 사이 /lane/center 를 새로 짓지 않고
건너뛴다 — 하류는 직전 값을 그대로 들고 간다. 주행 중 한쪽 흰선을
오래 놓치면 경로가 그만큼 낡는다는 뜻이므로, 그게 곤란하면
pan_search:=false 로 끄거나 pan_hold_timeout_s 에 상한을 준다.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


PACKAGE = 'kau_lane_detection'

# ros_gz_bridge parameter_bridge 인자 문법:
#   <topic>@<ROS 타입>@<GZ 타입>
CAMERA_INFO_BRIDGE = (
    '/camera/camera_info'
    '@sensor_msgs/msg/CameraInfo'
    '@gz.msgs.CameraInfo'
)

IMAGE_BRIDGE = (
    '/camera/image_raw'
    '@sensor_msgs/msg/Image'
    '@gz.msgs.Image'
)


def generate_launch_description():
    default_params = str(
        Path(get_package_share_directory(PACKAGE)) /
        'config' /
        'lane_detection.yaml'
    )

    params_file = LaunchConfiguration('params_file')
    camera_info_bridge = LaunchConfiguration('camera_info_bridge')
    image_bridge = LaunchConfiguration('image_bridge')
    viewer = LaunchConfiguration('viewer')
    pan_search = LaunchConfiguration('pan_search')
    pan_aim = LaunchConfiguration('pan_aim')
    use_sim_time = ParameterValue(
        LaunchConfiguration('use_sim_time'), value_type=bool)

    return LaunchDescription([

        # ------------------------------------------------------------
        # Launch Arguments
        # ------------------------------------------------------------

        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='차선 인지 노드 파라미터 yaml',
        ),

        DeclareLaunchArgument(
            'camera_info_bridge',
            default_value='true',
            description='Gazebo -> ROS CameraInfo bridge 실행 여부',
        ),

        DeclareLaunchArgument(
            'image_bridge',
            default_value='false',
            description=(
                'image_raw 도 bridge 할지. '
                '이미 다른 곳에서 bridge 중이면 false 로 둔다'
            ),
        ),

        DeclareLaunchArgument(
            'viewer',
            default_value='true',
            description='웹 뷰어(http://localhost:5000) 실행 여부',
        ),

        # 시계 소스. 없으면 rclcpp 기본값 false 라 시뮬에서도 이 노드만
        # 벽시계로 돈다. 다른 런치들(ekf / amcl / global_path)과 같은 규칙이다.
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Gazebo 는 true, 실기는 false.',
        ),

        DeclareLaunchArgument(
            'pan_search',
            default_value='true',
            description=(
                '차선 소실 시 카메라 pan 탐색 (기본 켜짐). '
                '탐색/유지/복귀 동안 /lane/center 가 갱신되지 않고 '
                '직전 값이 유지된다'
            ),
        ),

        DeclareLaunchArgument(
            'pan_aim',
            default_value='true',
            description=(
                '전역경로 룩어헤드로 카메라를 미리 돌리는 조준 '
                '(기본 켜짐). 켜지면 pan_search 상태기계 대신 '
                '동작하고, 측위/전역경로가 없으면 자동으로 '
                '상태기계로 넘어간다'
            ),
        ),

        # ------------------------------------------------------------
        # Gazebo -> ROS Bridge
        #
        # 노드 두 개로 분리한 이유: image_raw 가 이미 bridge 되어
        # 있는 환경에서 한 프로세스로 묶으면 중복 발행이 된다.
        # ------------------------------------------------------------

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='camera_info_bridge',
            arguments=[CAMERA_INFO_BRIDGE],
            output='screen',
            condition=IfCondition(camera_info_bridge),
        ),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='camera_image_bridge',
            arguments=[IMAGE_BRIDGE],
            output='screen',
            condition=IfCondition(image_bridge),
        ),

        # ------------------------------------------------------------
        # Lane Detection
        # ------------------------------------------------------------

        Node(
            package=PACKAGE,
            executable='kau_lane_detection_node',
            name='kau_lane_detection_node',
            output='screen',
            parameters=[
                params_file,
                {
                    # yaml 을 덮는다. 노드가 생성자에서 한 번만 읽으므로
                    # 여기서 넣지 않으면 나중에 켤 방법이 없다.
                    'pan_search_enable': ParameterValue(
                        pan_search, value_type=bool),

                    # 이쪽은 노드가 매 프레임 다시 읽으므로 기동 뒤
                    # ros2 param set 으로도 바꿀 수 있다.
                    'pan_aim_enable': ParameterValue(
                        pan_aim, value_type=bool),
                    'use_sim_time': use_sim_time,
                },
            ],
        ),

        # ------------------------------------------------------------
        # Web Viewer
        # ------------------------------------------------------------

        Node(
            package=PACKAGE,
            executable='lane_viewer.py',
            name='kau_lane_detection_viewer',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(viewer),
        ),
    ])
