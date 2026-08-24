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

"""Cartographer 2D SLAM (지도 작성).

    ros2 launch kau_localization slam.launch.py

physicar_bringup 의 sim.launch.py / real.launch.py 가 이미 떠 있는 상태에서
얹어 쓴다. Cartographer 는 map -> odom 만 발행하고,
odom -> base_footprint 는 기존 ekf_filter_node 가 계속 소유한다.

입력
    /scan  LaserScan     관측값 (submap 삽입 + loop closure)
    /odom  Odometry      EKF 융합 출력. 스캔 매칭 prior
    TF     base_footprint -> lidar_link (robot_state_publisher, URDF)

출력
    TF        map -> odom
    /map      OccupancyGrid (cartographer_occupancy_grid_node)
    /submap_list, /constraint_list, /trajectory_node_list, /scan_matched_points2

지도 저장 (주행이 끝난 뒤)
    ../scripts/save_map.py

    최종 최적화 -> maps/kau_vN.pbstream 저장 -> pgm/yaml 변환 -> 품질 검사까지
    한 번에 한다. 버전 N 은 자동으로 증가하므로 기존 지도를 덮어쓰지 않는다.

    주의: save_state:= 를 주지 않고 띄웠다면 노드를 그냥 종료해도 저장되지
    않는다. 반드시 위 스크립트로 저장한 뒤에 끌 것.

주요 인자
    use_sim_time:=false        실기에서 실행할 때
    scan_topic:=/scan_filtered 필터된 스캔을 쓸 때. 권장하지 않는다.
                               scan_filter 는 무효값을 0.0 으로 바꾸는데
                               0.0 은 range_min(0.1) 미만이라 통째로 버려진다.
    save_state:=/path/x.pbstream   종료 시 자동 저장 (save_map.py 를 쓰면 불필요)
    rviz:=true                 RViz 동시 실행. 매핑 중에는 끄는 편이 낫다.
                               4 코어에서 rviz2 가 CPU 136% 를 먹고
                               (cartographer 는 35%) 자신도 메시지를 버린다.
                               지도 확인은 save_map.py 의 검사로 하는 게 정확하다.

품질 진단
    ../scripts/check_constraints.py   루프 클로저 constraint 길이 분포.
                                      6 m 초과가 거의 0 이어야 정상이다.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


PACKAGE = 'kau_localization'


def generate_launch_description():
    share = Path(get_package_share_directory(PACKAGE))
    config_dir = str(share / 'config')
    rviz_config = str(share / 'rviz' / 'localization.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')
    scan_topic = LaunchConfiguration('scan_topic')
    odom_topic = LaunchConfiguration('odom_topic')
    config_file = LaunchConfiguration('config_file')
    resolution = LaunchConfiguration('resolution')
    publish_period = LaunchConfiguration('publish_period_sec')
    save_state = LaunchConfiguration('save_state')

    args = [
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Gazebo 는 true, 실기는 false.'),
        DeclareLaunchArgument(
            'scan_topic', default_value='/scan',
            description=(
                'raw /scan 권장. scan_filter 는 무효값을 0.0 으로 바꾸는데 '
                '0.0 은 range_min 미만이라 Cartographer 가 버린다.'),),
        DeclareLaunchArgument(
            'odom_topic', default_value='/odom',
            description='EKF 융합 출력. /odom/laser 원본이 아니다.'),
        DeclareLaunchArgument(
            'config_file', default_value='physicar_2d.lua',
            description='config/ 안의 lua 파일명.'),
        DeclareLaunchArgument(
            'resolution', default_value='0.05',
            description='/map 격자 해상도 [m]. lua 의 submap 해상도와 맞춘다.'),
        DeclareLaunchArgument(
            'publish_period_sec', default_value='1.0',
            description='/map 발행 주기 [s].'),
        DeclareLaunchArgument(
            'save_state', default_value='',
            description='비우면 저장 안 함. 경로를 주면 종료 시 pbstream 저장.'),
        DeclareLaunchArgument(
            'rviz', default_value='false',
            description=(
                'RViz 동시 실행 여부. 매핑 중에는 끄는 편이 낫다. 4 코어에서 '
                'rviz2 가 CPU 136% 를 먹어 cartographer(35%) 를 굶긴다.'),),
    ]

    # gflags 인자는 반드시 --ros-args 앞에 와야 한다. launch_ros 는
    # arguments 를 실행 파일 바로 뒤, --ros-args 앞에 붙여준다.
    # save_state 가 빈 문자열이면 플래그 자체를 넘기지 않는다
    # (-save_state_filename "" 은 gflags 가 빈 값으로 받아 무시하지만
    #  의도를 분명히 하려고 조건부로 구성한다).
    cartographer_args = [
        '-configuration_directory', config_dir,
        '-configuration_basename', config_file,
        '-save_state_filename', save_state,
    ]

    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        arguments=cartographer_args,
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[
            ('scan', scan_topic),
            ('odom', odom_topic),
        ],
    )

    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='log',
        parameters=[{
            'use_sim_time': use_sim_time,
            'resolution': ParameterValue(resolution, value_type=float),
            'publish_period_sec': ParameterValue(publish_period, value_type=float),
        }],
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

    return LaunchDescription(
        args + [cartographer_node, occupancy_grid_node, rviz_node])
