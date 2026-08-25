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

"""AMCL 파티클 필터 측위 (Cartographer 의 대안).

    ros2 launch kau_localization amcl.launch.py \
        map:=kau_v3 start_pose:="0.23,-0.02,0.0"

cartographer_localization.launch.py 와 **동시에 띄우면 안 된다.**
둘 다 map -> odom 을 발행해서 tf2 가 깨진다.

Cartographer 와 비교:
    추정 정확도는 Cartographer 가 낫다. 대신 대회장에서 관계자가 차를 들어
    이전 지점으로 되돌리면 Cartographer 는 pose graph 가 어긋나 복구가
    어렵고, AMCL 은 /initialpose 한 방이면 그 자리에서 다시 잡는다.

지도 지정
    폴더는 항상 같으므로 이름만 준다. map:=kau_v3 / kau_v3.yaml / latest 가
    모두 된다. 다른 곳의 지도는 경로를 그대로 주면 된다 (map_arg.py 참고).

입출력은 Cartographer 쪽과 같다. 지도만 .pbstream 대신 .yaml + .pgm 이다.
없으면 pbstream 에서 만든다:
    ros2 run cartographer_ros cartographer_pbstream_to_ros_map \
        -pbstream_filename <...>.pbstream -map_filestem <...> -resolution 0.05

초기 위치
    start_pose 를 주면 그 자리에서 시작한다. 안 주면 RViz 의
    "2D Pose Estimate"(/initialpose) 를 기다린다 — AMCL 이 직접 구독하므로
    Cartographer 때와 달리 릴레이 노드가 필요 없다.

    지도 origin 이 [-3.79, -1.76] 이라 (0,0,0) 은 방 한가운데가 아니다.
    start_pose 를 생략하고 아무 것도 안 찍으면 파티클이 초기 분산 그대로
    떠 있어서 수렴하지 않는다.

주요 인자
    map:=kau_v3                    (필수) 이름 / latest / 경로
    maps_dir:=/다른/폴더            이름만 줬을 때 뒤질 폴더
    start_pose:="x,y,yaw"          시작 위치 (m, m, rad). 비우면 /initialpose 대기
    use_sim_time:=false            실기에서 실행할 때
    scan_topic:=/scan              기본은 yaml. raw /scan 을 써야 한다 (config/amcl.yaml)
    params_file:=/path/to.yaml     기본은 config/amcl.yaml
    nomotion_update:=true          정지 중 강제 갱신을 켤 때 (기본 꺼짐, 권장 안 함)
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


def parse_start_pose(text):
    """Parse "x,y,yaw" 문자열 -> AMCL 초기 pose 파라미터. 비어 있으면 None."""
    text = text.strip()
    if not text:
        return None
    parts = [p for p in text.replace(' ', ',').split(',') if p]
    if len(parts) != 3:
        raise RuntimeError(
            f'start_pose 는 "x,y,yaw" 세 값이어야 한다 (받은 값: {text!r})')
    x, y, yaw = (float(p) for p in parts)
    return {
        'set_initial_pose': True,
        'initial_pose.x': x,
        'initial_pose.y': y,
        'initial_pose.z': 0.0,
        'initial_pose.yaw': yaw,
    }


def launch_setup(context, *_args, **_kwargs):
    share = Path(get_package_share_directory(PACKAGE))

    params_file = LaunchConfiguration('params_file').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    scan_topic = LaunchConfiguration('scan_topic').perform(context)

    maps_dir = Path(LaunchConfiguration('maps_dir').perform(context)).expanduser()
    map_yaml = resolve_map(
        LaunchConfiguration('map').perform(context), maps_dir, '.yaml', 'map')

    initial_pose = parse_start_pose(LaunchConfiguration('start_pose').perform(context))

    # 값의 주인은 params_file(기본 config/amcl.yaml)이다. params_file 뒤에
    # 오는 dict 가 같은 키를 덮으므로, **명시적으로 준 인자만** 여기 넣는다.
    # 빈 값이 기본인 인자는 "안 줬다"는 뜻이라 yaml 값이 그대로 산다.
    amcl_overrides = {'use_sim_time': use_sim_time}
    if scan_topic:
        amcl_overrides['scan_topic'] = scan_topic
    if initial_pose:
        amcl_overrides.update(initial_pose)

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[params_file, {
            'use_sim_time': use_sim_time,
            'yaml_filename': map_yaml,
        }],
    )

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[params_file, amcl_overrides],
    )

    # 정지 중 /request_nomotion_update 를 대신 불러 주는 노드. **기본은 꺼져 있다.**
    #
    # 원래는 "회전하다 멈추면 틀어진 채 유지" 증상을 잡으려고 만들었는데, 실측
    # 결과 처방이 틀렸다. 강제 갱신은 odom 델타가 0 이라 모션 노이즈가 안 들어가서
    # 파티클 다양성만 소멸시킨다 (2 Hz 무제한 시 100 초만에 발산). 같은 증상은
    # update_min_a 를 0.004 로 낮추는 쪽이 훨씬 잘 잡는다 — 이 플랫폼은 정지
    # 중에도 EKF 드리프트 때문에 odom 이 움직여서 자연 갱신이 계속 트리거된다.
    #
    # 그래도 남겨 둔 이유: EKF 가 나중에 고쳐져 정지 중 odom 이 진짜로 멈추면
    # 자연 갱신도 멈춘다. 그때는 이 노드가 다시 필요해진다.
    nomotion_updater = Node(
        package=PACKAGE,
        executable='nomotion_updater',
        name='nomotion_updater',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        condition=IfCondition(LaunchConfiguration('nomotion_update')),
    )

    # 두 노드 모두 lifecycle 노드라 configure/activate 를 해줘야 뜬다.
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='map_amcl_lifecycle_manager',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        # Cartographer 와 같은 설정을 쓴다. 두 방식을 같은 뷰에서 번갈아 봐야
        # 비교가 되기 때문이다. carto 전용 디스플레이는 기본으로 꺼져 있다.
        arguments=['-d', str(share / 'rviz' / 'localization.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return [map_server, amcl, lifecycle_manager, nomotion_updater, rviz_node]


def generate_launch_description():
    share = Path(get_package_share_directory(PACKAGE))

    args = [
        # pbstream 과 마찬가지로 기본값을 주지 않는다. 지도 없이 측위는 의미가 없다.
        DeclareLaunchArgument(
            'map',
            description='지도 이름 (kau_v3 / kau_v3.yaml / latest) 또는 경로. '
                        'pgm 과 짝인 yaml 이다 — pbstream 이 아니다.'),
        DeclareLaunchArgument(
            'maps_dir', default_value=str(share / 'maps'),
            description='이름만 줬을 때 지도를 찾을 폴더.'),
        DeclareLaunchArgument(
            'params_file',
            default_value=str(share / 'config' / 'amcl.yaml'),
            description='AMCL / map_server / lifecycle_manager 파라미터.'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Gazebo 는 true, 실기는 false.'),
        DeclareLaunchArgument(
            'scan_topic', default_value='',
            description='기본은 yaml (raw /scan). /scan_filtered 는 무효값을 '
                        '0.0 으로 바꿔서 AMCL 가중치를 망가뜨리므로 쓰지 말 것.'),
        DeclareLaunchArgument(
            'start_pose', default_value='',
            description='시작 위치 "x,y,yaw" (m, m, rad). 비우면 /initialpose 대기.'),
        DeclareLaunchArgument(
            'nomotion_update', default_value='false',
            description='정지 중 AMCL 필터를 강제로 갱신할지. 기본은 꺼짐 — '
                        'update_min_a 0.004 로 자연 갱신이 계속 일어나므로 '
                        '불필요하고, 켜면 파티클 다양성이 소멸한다.'),
        DeclareLaunchArgument(
            'rviz', default_value='false',
            description='RViz 동시 실행 여부.'),
    ]

    return LaunchDescription(args + [OpaqueFunction(function=launch_setup)])
