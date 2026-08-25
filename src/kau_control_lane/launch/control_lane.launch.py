#!/usr/bin/env python3
r"""차선 기반 주행 제어 — 측위(map) 없이 카메라·라이다만으로 돈다.

    kau_lane_detection ──(/lane/center, /lane/left, /lane/right)──┐
    kau_object_detection_lane ──(/perception/obstacles)───────────┤
                                                                  ▼
                                              kau_local_path_planner_lane
                                                (장애물 회피 경로를 그린다)
                                                                  │
                                                (/path/local, base_footprint)
                                                                  ▼
                                          lane_speed_controller / lane_steer_controller
                                                                  │
                                                        /speed, /steering

kau_control 의 **대안**이다. 알고리즘(Pure Pursuit · 곡률 기반 속도 + PID)은
같고, 경로가 전역경로·측위를 타지 않는다는 것만 다르다.

★ kau_control 의 두 제어기와 **같은 topic(/speed, /steering)** 을 낸다.
  동시에 띄우면 두 발행자가 싸운다. 이걸 쓰는 동안 run.sh 의 KAU_NODES 표에서
  steer_controller / speed_controller 를 false 로 내릴 것.

전제 (/speed, /steering 소비자):

    ros2 launch physicar_bringup sim.launch.py     # 또는 real.launch.py

실행:

    # 조향만 관찰 (차는 안 움직인다). 처음엔 반드시 이걸로 시작할 것
    ros2 launch kau_control_lane control_lane.launch.py speed:=0.0

    # 인지 + 회피 계획까지 한 번에 (장애물 인지는 별도로 띄울 것)
    ros2 launch kau_control_lane control_lane.launch.py \
        lane_detection:=true planner:=true speed:=0.3 start_gate:=false

    # 계획을 빼고 중앙선만 따라간다 (planner 를 의심할 때)
    ros2 launch kau_control_lane control_lane.launch.py source:=center

    # s / cte / Ld / steer 를 2 Hz 로 본다
    ros2 launch kau_control_lane control_lane.launch.py log_level:=debug

주요 인자:

    source:=local           local(기본) = 회피 경로 /path/local (base_footprint)
                            center      = 중앙선 /lane/center (base_link)
                            topic·frame·timeout 셋을 함께 바꾼다
    speed:=0.4              상수 속도 [m/s]. v_min = v_max 로 덮는다.
                            안 주면 yaml 값이 그대로 산다. 0.0 이면 조향만
    planner:=false          회피 계획도 같이 띄울지
    lane_detection:=false   차선 인지도 같이 띄울지
    viewer:=true            웹 뷰어(포트 5000). lane_detection:=true 일 때만
    start_gate:=            빈 값이면 yaml. false 면 신호등 없이 출발
    pose_source:=identity   identity(측위 불필요) | tf(map 프레임)
    use_sim_time:=true      실기는 false

★ 경로 발행자가 없으면 차는 안 움직인다 (조용히가 아니라 2 초마다 사유를
  WARN 으로 찍는다).

    source:=local   -> kau_local_path_planner_lane 이 떠 있어야 한다
    source:=center  -> kau_lane_detection 의 path_frame_id 가 base_link 여야 한다

안전:

    두 노드 모두 어떤 경로로 빠져나가든 0 을 계속 발행한다 (driver 의
    cmd_timeout 이 1 초라, 조용히 멈추면 그동안 직전 명령으로 계속
    굴러가기 때문). 경로가 끊기면 스스로 선다.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


PACKAGE = 'kau_control_lane'

DETECTION_PACKAGE = 'kau_lane_detection'

PLANNER_PACKAGE = 'kau_local_path_planner_lane'


# source 인자 -> 경로 입력 세 값. topic · frame · timeout 은 짝이라
# 따로 놀 수 없다 (lane_source.yaml 의 같은 표를 코드로 옮긴 것).
SOURCES = {
    # 회피 경로. kau_local_path_planner_lane 이 차선 + 장애물을 보고 그린다
    'local': {
        'lane.topic': '/path/local',
        'lane.frame': 'base_footprint',
        'lane.timeout': 1.0,          # 5 Hz -> 5 주기 여유
    },
    # 중앙선 그 자체. 장애물을 모른다 (planner 를 의심할 때 여기로 내린다)
    'center': {
        'lane.topic': '/lane/center',
        'lane.frame': 'base_link',
        'lane.timeout': 0.5,          # 14 Hz -> 최대 7 프레임
    },
}


def config(name):
    return str(Path(get_package_share_directory(PACKAGE)) / 'config' / name)


def controller_nodes(context, *unused):
    """lane_speed / lane_steer 컨트롤러 두 개를 만든다.

    값의 주인은 yaml 이다. launch 인자는 **명시적으로 준 것만** 덮는다 --
    기본값 ''(빈 값)은 "안 줬다" 는 뜻이라 yaml 이 그대로 산다.
    """
    lane_params = LaunchConfiguration('lane_params_file')
    speed_params = LaunchConfiguration('speed_params_file')
    steer_params = LaunchConfiguration('steer_params_file')

    common = ['--ros-args', '--log-level', LaunchConfiguration('log_level')]

    use_sim_time = ParameterValue(
        LaunchConfiguration('use_sim_time'), value_type=bool)

    # 두 노드 공통 override. 조향과 속도가 서로 다른 경로/프레임을 보면 안
    # 되므로 source 와 pose_source 는 반드시 둘 다에 같이 건다.
    shared = {'use_sim_time': use_sim_time}

    # topic · frame · timeout 은 **짝이다.** 하나만 바꾸면 경로가 전부
    # 버려지므로 인자 하나로 셋을 함께 넘긴다 (lane_source.yaml 의 표와 동일).
    raw_source = LaunchConfiguration('source').perform(context)
    if raw_source:
        if raw_source not in SOURCES:
            raise RuntimeError(
                "source 는 %s 중 하나여야 한다 (받은 값: '%s')"
                % (' | '.join(SOURCES), raw_source))
        shared.update(SOURCES[raw_source])

    raw_pose = LaunchConfiguration('pose_source').perform(context)
    if raw_pose:
        shared['lane.pose_source'] = raw_pose

    speed_overrides = dict(shared)

    raw_speed = LaunchConfiguration('speed').perform(context)
    if raw_speed:
        speed_overrides['speed.v_min'] = float(raw_speed)
        speed_overrides['speed.v_max'] = float(raw_speed)

    raw_gate = LaunchConfiguration('start_gate').perform(context)
    if raw_gate:
        speed_overrides['start_gate.required'] = (raw_gate.lower() == 'true')

    return [
        Node(
            package=PACKAGE,
            executable='lane_speed_controller_node',
            name='lane_speed_controller',
            output='screen',
            arguments=common,
            parameters=[lane_params, speed_params, speed_overrides],
        ),
        Node(
            package=PACKAGE,
            executable='lane_steer_controller_node',
            name='lane_steer_controller',
            output='screen',
            arguments=common,
            parameters=[lane_params, steer_params, shared],
        ),
    ]


def generate_launch_description():
    detection_launch = str(
        Path(get_package_share_directory(DETECTION_PACKAGE)) /
        'launch' /
        'lane_detection.launch.py'
    )

    planner_launch = str(
        Path(get_package_share_directory(PLANNER_PACKAGE)) /
        'launch' /
        'local_planner.launch.py'
    )

    lane_detection = LaunchConfiguration('lane_detection')
    planner = LaunchConfiguration('planner')
    viewer = LaunchConfiguration('viewer')

    return LaunchDescription([

        # ------------------------------------------------------------
        # Launch Arguments
        # ------------------------------------------------------------

        DeclareLaunchArgument(
            'lane_params_file',
            default_value=config('lane_source.yaml'),
            description='차선 경로 입력 parameter yaml (두 노드 공용). '
                        '좌표계는 그 파일의 lane.pose_source 한 줄이다',
        ),

        DeclareLaunchArgument(
            'speed_params_file',
            default_value=config('lane_speed_controller.yaml'),
            description='속도 제어 parameter yaml',
        ),

        DeclareLaunchArgument(
            'steer_params_file',
            default_value=config('lane_steer_controller.yaml'),
            description='조향 제어 parameter yaml',
        ),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='시뮬이면 true. 실기는 false',
        ),

        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='debug 로 두면 s / cte / Ld / steer 가 2 Hz 로 찍힌다',
        ),

        DeclareLaunchArgument(
            'speed',
            default_value='',
            description=(
                '상수 속도 [m/s]. v_min = v_max 로 덮는다 (기본: yaml). '
                '0.0 이면 조향만 관찰한다'
            ),
        ),

        DeclareLaunchArgument(
            'source',
            default_value='',
            description=(
                'local | center. 비우면 lane_source.yaml 값(local). '
                'local = 회피 경로(/path/local), '
                'center = 중앙선 그 자체(/lane/center). '
                'topic·frame·timeout 셋을 함께 바꾼다'
            ),
        ),

        DeclareLaunchArgument(
            'pose_source',
            default_value='',
            description=(
                'identity | tf. 비우면 lane_source.yaml 값. '
                'kau_lane_detection 의 path_frame_id 와 짝이 맞아야 한다'
            ),
        ),

        DeclareLaunchArgument(
            'start_gate',
            default_value='',
            description=(
                '빈 값이면 yaml. false 로 두면 신호등 허가 없이 출발한다 '
                '(주행부만 시험할 때만)'
            ),
        ),

        DeclareLaunchArgument(
            'lane_detection',
            default_value='false',
            description='차선 인지도 같이 띄울지. 이미 돌고 있으면 false',
        ),

        DeclareLaunchArgument(
            'planner',
            default_value='false',
            description=(
                '회피 계획(kau_local_path_planner_lane)도 같이 띄울지. '
                'source:=local 인데 이게 안 떠 있으면 /path/local 발행자가 '
                '없어 차가 안 움직인다. 장애물 인지는 별도로 띄울 것'
            ),
        ),

        DeclareLaunchArgument(
            'viewer',
            default_value='true',
            description='웹 뷰어(http://localhost:5000) 실행 여부',
        ),

        # ------------------------------------------------------------
        # 차선 인지 — /lane/center 발행자
        #
        # 이 launch 가 camera_info bridge 까지 챙긴다. CameraInfo 가
        # 없으면 인지 노드는 undistort 를 시작하지 못하고
        # "Waiting for CameraInfo..." 에서 멈춘다.
        # ------------------------------------------------------------

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(detection_launch),
            launch_arguments={'viewer': viewer}.items(),
            condition=IfCondition(lane_detection),
        ),

        # ------------------------------------------------------------
        # 회피 계획 — /path/local 발행자
        #
        # 차선(중앙선 + 좌/우 흰선)과 장애물을 보고 회피 경로를 그린다.
        # 장애물은 별도 패키지(kau_object_detection_lane)가 낸다 --
        # 그게 없으면 계획은 돌지만 피할 것을 모른다.
        # ------------------------------------------------------------

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(planner_launch),
            condition=IfCondition(planner),
        ),

        # ------------------------------------------------------------
        # 제어
        #
        # parameter 는 yaml 이 값의 주인이다. launch 인자는 명시적으로
        # 준 것만 덮는다 (controller_nodes 주석 참고).
        # ------------------------------------------------------------

        OpaqueFunction(function=controller_nodes),
    ])
