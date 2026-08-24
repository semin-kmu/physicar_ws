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

"""플랫폼 EKF 를 우리 설정으로 교체한다.

    ros2 launch kau_localization ekf.launch.py

측위(AMCL / Cartographer)와 **별개로** 띄운다. 먼저 이걸 띄우고 그 다음에
amcl.launch.py 든 cartographer_localization.launch.py 든 올리면 된다.
둘 다 이 EKF 가 만든 odom -> base_footprint 위에서 돈다.

왜 필요한가
    플랫폼 기본 EKF 설정이 절대 방위를 전부 버려서 odom yaw 가 랜덤워크한다
    (정지 7.9 deg/min, 주행 중 최대 19 도). 자세한 실측은 config/ekf.yaml 과
    README 참고. 19 도는 AMCL 모션 모델이 흡수할 수 있는 범위 밖이라
    측위 튜닝으로는 못 덮는다.

    /opt/physicar 아래 플랫폼 소스는 **건드리지 않는다.** 플랫폼이 초기화되면
    같이 날아가고, 다른 팀원 환경과도 어긋나기 때문이다. 대신 프로세스만
    잠시 재우고 우리 설정으로 하나 더 띄운다.

플랫폼 EKF 를 왜 kill 이 아니라 -STOP 으로 재우나
    physicar_bringup 의 sim.launch.py / real.launch.py 가 ekf_node 를
    respawn=True, respawn_delay=2.0 으로 띄운다. 그냥 죽이면 2 초 뒤 되살아나서
    odom -> base_footprint 를 우리 노드와 **동시에** 발행한다.

    tf2 의 소유권은 (부모, 자식) 쌍으로만 결정된다. 노드 이름은 아무 상관이
    없다. /tf 는 중재자 없는 단일 토픽이라 두 발행자의 값이 번갈아 섞여
    들어가고, 증상이 "가끔 튄다" 로 나타나서 원인 찾기가 최악이다.

    SIGSTOP 은 프로세스를 살려 둔 채 멈추기만 한다. launch 는 살아 있다고
    보므로 respawn 이 안 걸리고, 멈춘 동안은 아무것도 발행하지 않는다.

    재우기/깨우기는 platform_ekf_pause.py 가 자기 생명주기에 묶어서 한다.
    처음에는 launch 의 OnShutdown 에 깨우기를 걸었는데 **안 돌았다** -- launch
    는 종료 중에 새 프로세스를 못 띄운다. 자세한 사정은 그 파일 참고.

    이 launch 를 Ctrl-C 로 내리면 감시 프로세스가 SIGINT 를 받고 종료 처리에서
    반드시 SIGCONT 를 보낸다. 멈춰 있던 사이 SIGTERM 이 밀려 있었으면 깨어나는
    순간 죽기도 하는데, 그때는 플랫폼 launch 의 respawn 이 2 초 뒤 새로 띄운다.
    어느 쪽이든 /odom 은 돌아온다 (실측 확인).

맵 리로드 / sim 재시작을 하면
    아무것도 안 해도 된다. 감시자가 알아서 처리한다.

      - sim.launch.py 는 안 죽고 자식 노드만 다시 뜬다. 플랫폼 ekf_node 가
        respawn=True 라 **새 PID 로** 살아나는데, 감시자가 1 Hz 로 다시 확인해서
        새로 뜬 것도 재운다. 안 그러면 TF 를 이중 발행한다.
      - 시계가 거꾸로 뛰면 robot_localization 이 조용히 발행을 멈춘다. 감시자가
        "입력은 오는데 출력이 없다" 로 감지해서 우리 EKF 를 재시작한다.
      - 세 번 재시작해도 안 되면 플랫폼 EKF 를 깨우고 전체를 내린다. 최소한
        플랫폼 기본 상태로는 돌아간다.

주요 인자
    use_sim_time:=false               실기에서 실행할 때
    params_file:=/path/to.yaml        기본은 config/ekf.yaml
    odom_topic:=/odom                 융합 결과를 낼 토픽 (플랫폼과 같아야 한다)
    freeze_platform_ekf:=false        플랫폼 EKF 를 안 재울 때. **tf 가 깨진다.**
                                      이미 손으로 재웠거나 죽였을 때만 쓸 것.

손으로 할 때 (launch 없이)
    재우기:  pkill -STOP -f physicar_bringup/config/ekf_params.yaml
    깨우기:  pkill -CONT -f physicar_bringup/config/ekf_params.yaml
    확인:    ps -eo pid,stat,cmd | grep ekf_node    # STAT 가 T 면 멈춘 것
"""

import sys
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    Shutdown,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


PACKAGE = 'kau_localization'

# 플랫폼 EKF 프로세스만 골라내는 패턴. 우리 노드도 같은 실행 파일
# (robot_localization/ekf_node) 이라 실행 파일 이름으로 잡으면 우리가 우리를
# 재운다. 플랫폼 쪽 명령줄에만 있는 params 파일 경로로 구분한다.
PLATFORM_EKF_PATTERN = 'physicar_bringup/config/ekf_params.yaml'


def launch_setup(context, *_args, **_kwargs):
    """Build the node list once launch arguments are resolvable."""
    share = Path(get_package_share_directory(PACKAGE))

    params_file = LaunchConfiguration('params_file').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    odom_topic = LaunchConfiguration('odom_topic').perform(context)

    # 이 프로세스가 살아 있는 동안만 플랫폼 EKF 가 멈춰 있다. 죽으면 -- 정상
    # 종료든 사고든 -- 반드시 깨운다. 그 외에 새로 뜬 플랫폼 EKF 를 다시 재우고,
    # 우리 EKF 가 멈추면 되살리는 일도 한다. 자세한 사정은 그 파일 참고.
    #
    # 우리 EKF 를 가려내는 패턴으로는 params 파일 경로를 그대로 넘긴다. 우리
    # 노드의 명령줄에만 들어 있는 문자열이라 플랫폼 것과 확실히 구분된다.
    pause = ExecuteProcess(
        cmd=[sys.executable, str(share / 'launch' / 'platform_ekf_pause.py'),
             PLATFORM_EKF_PATTERN, params_file],
        name='platform_ekf_pause',
        output='screen',
        shell=False,
        # 감시 프로세스가 먼저 죽으면 플랫폼 EKF 가 깨어나서 우리 노드와 같이
        # odom -> base_footprint 를 쏜다. 그런 상태로 계속 도느니 전부 내린다.
        on_exit=Shutdown(reason='platform_ekf_pause 가 종료됨'),
        condition=IfCondition(LaunchConfiguration('freeze_platform_ekf')),
    )

    # 이름은 tf 소유권과 무관하지만, ros2 node list / ps 에서 플랫폼 것과
    # 구별되도록 다르게 준다. config/ekf.yaml 최상위 키가 /** 라서 이름을
    # 바꿔도 파라미터는 정상적으로 붙는다.
    #
    # respawn=True 가 감시자와 짝을 이룬다. robot_localization 은 시계가 거꾸로
    # 뛰면 조용히 발행을 멈추는데, 프로세스는 살아 있어서 respawn 만으로는 안
    # 걸린다. 감시자가 그걸 감지해 SIGINT 를 보내고, 그때 respawn 이 새 시계로
    # 처음부터 시작하는 프로세스를 띄운다.
    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='kau_ekf',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        remappings=[('odometry/filtered', odom_topic)],
        respawn=True,
        respawn_delay=2.0,
    )

    # SIGSTOP 이 실제로 걸린 뒤에 우리 노드를 띄운다. 겹치는 순간에 두
    # 발행자가 같이 쏘는 것을 피하려는 것이다.
    return [pause, TimerAction(period=1.0, actions=[ekf])]


def generate_launch_description():
    """Declare arguments and wire the platform-EKF freeze/thaw pair."""
    share = Path(get_package_share_directory(PACKAGE))

    args = [
        DeclareLaunchArgument(
            'params_file',
            default_value=str(share / 'config' / 'ekf.yaml'),
            description='EKF 파라미터. 기본은 우리 설정 config/ekf.yaml.'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Gazebo 는 true, 실기는 false.'),
        DeclareLaunchArgument(
            'odom_topic', default_value='/odom',
            description='융합 결과를 낼 토픽. 플랫폼과 같아야 제어 쪽이 받는다.'),
        DeclareLaunchArgument(
            'freeze_platform_ekf', default_value='true',
            description='플랫폼 EKF 를 SIGSTOP 으로 재울지. 끄면 odom -> '
                        'base_footprint 를 둘이 동시에 발행해 tf2 가 깨진다.'),
    ]

    return LaunchDescription(args + [OpaqueFunction(function=launch_setup)])
