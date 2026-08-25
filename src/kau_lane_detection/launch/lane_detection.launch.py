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
    pan_search:=false           카메라 pan 탐색 끄기 (기본: yaml)
    pan_aim:=false              카메라 pan 조준 끄기 (기본: yaml)

pan_search / pan_aim 은 안 주면 yaml 값(pan_search_enable ·
pan_aim_enable)이 그대로 산다. 줄 때만 덮는다.

pan_search 는 노드가 생성자에서 한 번만 읽는 값이라
ros2 param set 으로는 바꿀 수 없다. 기동 시점에 넣어야 한다.

yaml 기본은 켜짐이다. 탐색/유지/복귀 동안에는 카메라 자세가 BEV 의
전제(pan=0)와 어긋나므로 그 사이 /lane/center 를 새로 짓지 않고
건너뛴다 — 하류는 직전 값을 그대로 들고 간다. 주행 중 한쪽 흰선을
오래 놓치면 경로가 그만큼 낡는다는 뜻이므로, 그게 곤란하면
pan_search:=false 로 끄거나 pan_hold_timeout_s 에 상한을 준다.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
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


# 인자 이름 -> 노드 파라미터 이름. yaml 의 같은 키를 덮는 자리다.
PAN_OVERRIDES = (
    ('pan_search', 'pan_search_enable'),
    ('pan_aim', 'pan_aim_enable'),
)


def lane_detection_node(context, *unused):
    """인지 노드 하나를 만든다.

    값의 주인은 params_file(기본 config/lane_detection.yaml)이다. pan 인자는
    **명시적으로 준 것만** 덮는다 -- 기본값 ''(빈 값)은 "안 줬다"는 뜻이다.
    예전에는 기본값이 true 라 yaml 의 pan_search_enable 을 항상 이겼다.
    """
    overrides = {
        'use_sim_time': ParameterValue(
            LaunchConfiguration('use_sim_time'), value_type=bool),
    }
    for arg, param in PAN_OVERRIDES:
        raw = LaunchConfiguration(arg).perform(context)
        if raw:
            overrides[param] = raw.lower() == 'true'

    return [Node(
        package=PACKAGE,
        executable='kau_lane_detection_node',
        name='kau_lane_detection_node',
        output='screen',
        parameters=[LaunchConfiguration('params_file'), overrides],
    )]


def generate_launch_description():
    default_params = str(
        Path(get_package_share_directory(PACKAGE)) /
        'config' /
        'lane_detection.yaml'
    )

    camera_info_bridge = LaunchConfiguration('camera_info_bridge')
    image_bridge = LaunchConfiguration('image_bridge')
    viewer = LaunchConfiguration('viewer')
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

        # 아래 둘은 yaml 에 값이 있다(pan_search_enable · pan_aim_enable).
        # 빈 값이 기본이고, 주면 그때만 yaml 을 덮는다.
        DeclareLaunchArgument(
            'pan_search',
            default_value='',
            description=(
                '차선 소실 시 카메라 pan 탐색 (기본: yaml). '
                '탐색/유지/복귀 동안 /lane/center 가 갱신되지 않고 '
                '직전 값이 유지된다'
            ),
        ),

        DeclareLaunchArgument(
            'pan_aim',
            default_value='',
            description=(
                '전역경로 룩어헤드로 카메라를 미리 돌리는 조준 '
                '(기본: yaml). 켜지면 pan_search 상태기계 대신 '
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

        OpaqueFunction(function=lane_detection_node),

        # ------------------------------------------------------------
        # Web Viewer
        # ------------------------------------------------------------

        # 같은 yaml 의 kau_lane_detection_viewer 섹션을 읽는다 (port).
        Node(
            package=PACKAGE,
            executable='lane_viewer.py',
            name='kau_lane_detection_viewer',
            output='screen',
            parameters=[
                LaunchConfiguration('params_file'),
                {'use_sim_time': use_sim_time},
            ],
            condition=IfCondition(viewer),
        ),
    ])
