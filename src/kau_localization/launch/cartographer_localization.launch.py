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

"""Cartographer 2D pure localization (저장된 지도 위에서 위치 추정).

    ros2 launch kau_localization cartographer_localization.launch.py \
        pbstream:=kau_v3

slam.launch.py 로 만들어 /write_state 로 저장한 .pbstream 이 필요하다.
입출력과 TF 소유권은 slam.launch.py 와 동일하다.
지도를 키우지 않고 frozen submap 에 붙어 map -> odom 만 갱신한다.

AMCL 을 쓰려면 amcl.launch.py 를 대신 띄운다. **둘은 택일이다** —
둘 다 map -> odom 을 발행한다.

지도 지정
    폴더는 항상 같으므로 이름만 준다. pbstream:=kau_v3 / kau_v3.pbstream /
    latest 가 모두 된다. 다른 곳의 지도를 쓰려면 경로를 그대로 주면 된다.
    자세한 규칙은 map_arg.py 참고.

초기 위치
    재부팅하면 /odom 이 0 에서 다시 시작하므로 Cartographer 는 로봇이
    지도 어디에 있는지 모른 채 전역 재탐색을 한다. RViz 의
    "2D Pose Estimate" 로 /initialpose 를 찍으면 initial_pose_relay 가
    현재 trajectory 를 끝내고 그 pose 로 새 trajectory 를 시작해 즉시 수렴시킨다.

주요 인자
    pbstream:=kau_v3               (필수) 이름 / latest / 경로
    maps_dir:=/다른/폴더            이름만 줬을 때 뒤질 폴더
    use_sim_time:=false            실기에서 실행할 때
    initial_pose:=false            /initialpose 릴레이 없이 전역 재탐색만
    start_pose:="0.23,-0.02,0.0"   시작 위치를 알 때 (m, m, rad). 기동 직후
                                   그 자리에서 재측위한다 — 전역 재탐색을 안 기다린다
    publish_map:=false             /map 을 안 띄울 때 (nav2 map_server 를 따로 쓸 때)
    rviz:=true                     RViz 동시 실행
"""

import sys
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

sys.path.insert(0, str(Path(__file__).resolve().parent))
from map_arg import resolve_map  # noqa: E402  (launch 폴더의 공용 헬퍼)


PACKAGE = 'kau_localization'


def launch_setup(context, *_args, **_kwargs):
    share = Path(get_package_share_directory(PACKAGE))
    config_dir = str(share / 'config')
    rviz_config = str(share / 'rviz' / 'localization.rviz')

    maps_dir = Path(LaunchConfiguration('maps_dir').perform(context)).expanduser()
    pbstream = resolve_map(
        LaunchConfiguration('pbstream').perform(context),
        maps_dir, '.pbstream', 'pbstream')

    use_sim_time = LaunchConfiguration('use_sim_time')
    config_file = LaunchConfiguration('config_file')

    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        arguments=[
            '-configuration_directory', config_dir,
            '-configuration_basename', config_file,
            '-load_state_filename', pbstream,
            # frozen 으로 올려야 저장된 지도가 재최적화로 변형되지 않는다.
            '-load_frozen_state', 'true',
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[
            ('scan', LaunchConfiguration('scan_topic')),
            ('odom', LaunchConfiguration('odom_topic')),
        ],
    )

    # 값의 주인은 params_file(기본 config/cartographer_localization.yaml)이다.
    # 뒤에 오는 dict 가 같은 키를 덮으므로 **명시적으로 준 인자만** 넣는다.
    # 빈 값이 기본인 인자는 "안 줬다"는 뜻이라 yaml 값이 그대로 산다.
    params_file = LaunchConfiguration('params_file').perform(context)

    grid_overrides = {'use_sim_time': use_sim_time}
    for key in ('resolution', 'publish_period_sec'):
        raw = LaunchConfiguration(key).perform(context)
        if raw:
            grid_overrides[key] = float(raw)

    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='log',
        parameters=[params_file, grid_overrides],
        condition=IfCondition(LaunchConfiguration('publish_map')),
    )

    # lua 경로 둘은 설치 트리마다 달라 yaml 에 못 적는다. 여기서 채운다.
    relay_overrides = {
        'use_sim_time': use_sim_time,
        'configuration_directory': config_dir,
        'configuration_basename': config_file,
    }
    start_pose = LaunchConfiguration('start_pose').perform(context)
    if start_pose:
        relay_overrides['start_pose'] = start_pose

    initial_pose_relay = Node(
        package=PACKAGE,
        executable='initial_pose_relay',
        name='initial_pose_relay',
        output='screen',
        parameters=[params_file, relay_overrides],
        condition=IfCondition(LaunchConfiguration('initial_pose')),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return [cartographer_node, occupancy_grid_node, initial_pose_relay, rviz_node]


def generate_launch_description():
    share = Path(get_package_share_directory(PACKAGE))

    args = [
        # default_value 를 주지 않으면 인자 없이 실행할 때 launch 가
        # 바로 에러를 낸다. 지도 없이 localization 은 의미가 없으므로 의도된 동작.
        DeclareLaunchArgument(
            'pbstream',
            description='지도 이름 (kau_v3 / kau_v3.pbstream / latest) 또는 경로. '
                        'slam.launch.py + /write_state 로 만든 것.'),
        DeclareLaunchArgument(
            'maps_dir', default_value=str(share / 'maps'),
            description='이름만 줬을 때 지도를 찾을 폴더.'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Gazebo 는 true, 실기는 false.'),
        DeclareLaunchArgument(
            'scan_topic', default_value='/scan',
            description='SLAM 때와 동일한 토픽을 써야 매칭 특성이 일치한다.'),
        DeclareLaunchArgument(
            'odom_topic', default_value='/odom',
            description='EKF 융합 출력.'),
        DeclareLaunchArgument(
            'config_file', default_value='physicar_2d_localization.lua',
            description='config/ 안의 lua 파일명.'),
        DeclareLaunchArgument(
            'params_file',
            default_value=str(share / 'config' / 'cartographer_localization.yaml'),
            description='initial_pose_relay / occupancy_grid_node 파라미터.'),
        # 아래 둘은 위 yaml 에 값이 있다. 주면 그때만 덮는다.
        DeclareLaunchArgument(
            'resolution', default_value='',
            description='/map 격자 해상도 [m] (기본: yaml).'),
        DeclareLaunchArgument(
            'publish_period_sec', default_value='',
            description='/map 발행 주기 [s] (기본: yaml).'),
        DeclareLaunchArgument(
            'initial_pose', default_value='true',
            description='RViz 2D Pose Estimate(/initialpose) 릴레이 사용 여부.'),
        DeclareLaunchArgument(
            'start_pose', default_value='',
            description='시작 위치를 알 때 "x,y,yaw" (m, m, rad). 기동 직후 '
                        '릴레이가 스스로 재측위한다. 기본은 yaml (빈 값 = '
                        '/initialpose 대기).'),
        DeclareLaunchArgument(
            'publish_map', default_value='true',
            description='occupancy_grid_node 로 /map 을 발행할지.'),
        DeclareLaunchArgument(
            'rviz', default_value='false',
            description='RViz 동시 실행 여부.'),
    ]

    return LaunchDescription(args + [OpaqueFunction(function=launch_setup)])
