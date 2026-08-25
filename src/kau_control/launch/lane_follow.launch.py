#!/usr/bin/env python3
"""차선 추종 일괄 실행 — 노란 중앙선을 보고 그대로 따라간다.

    kau_lane_detection --(/lane/center, KauPath, base_link)--> speed_controller
                                                               steer_controller
                                                                     |
                                                       /speed, /steering

측위(Cartographer / map -> odom TF)가 **필요 없다.** lane detection 이
경로를 차량 프레임(base_link)으로 발행하고, lane_follow.yaml 이 두 제어
노드를 path.mode: lane_only + sources.lane.pose_source: identity 로 두면
차량은 그 프레임의 원점이므로 TF 를 볼 일이 없다. 측위 오차가 결과에
섞이지 않는 것이 이 조합의 장점이다.

사용 전에 physicar 쪽이 떠 있어야 한다 (/speed, /steering 소비자):

    ros2 launch physicar_bringup sim.launch.py     # 또는 real.launch.py

실행:

    # 조향만 관찰 (차는 안 움직인다). 처음엔 반드시 이걸로 시작할 것
    ros2 launch kau_control lane_follow.launch.py speed:=0.0

    # 상수 속도 주행
    ros2 launch kau_control lane_follow.launch.py speed:=0.4

    # 인지는 이미 다른 터미널에서 돌고 있을 때
    ros2 launch kau_control lane_follow.launch.py lane_detection:=false

    # s / cte / Ld / steer 를 2 Hz 로 본다
    ros2 launch kau_control lane_follow.launch.py log_level:=debug

주요 인자:

    speed:=0.4            상수 속도 [m/s]. v_min = v_max 로 덮는다.
                          안 주면 yaml 값이 그대로 산다.
                          0.0 이면 조향만 (차는 정지)
    lane_detection:=true  차선 인지도 같이 띄울지
    viewer:=true          웹 뷰어(포트 5000). lane_detection:=true 일 때만
    use_sim_time:=true    실기는 false
    params_file:=...      config/lane_follow.yaml 대체

곡률 기반 가감속을 켜려면 speed 인자를 주지 말고 lane_follow.yaml 의
v_min / v_max 를 서로 다르게 둔다 (예: 0.3 / 0.8). lane 경로는 앞 80 cm
짜리 토막이라 코너를 미리 못 보므로 아직 미검증이다.

안전:

    두 노드 모두 어떤 경로로 빠져나가든 0 을 계속 발행한다 (driver 의
    cmd_timeout 이 1 초라, 조용히 멈추면 그동안 직전 명령으로 계속
    굴러가기 때문). 차선을 놓치면(0.5 s 무발행) 스스로 선다.
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


PACKAGE = 'kau_control'

DETECTION_PACKAGE = 'kau_lane_detection'


def controller_nodes(context, *unused):
    """speed / steer 컨트롤러 두 개를 만든다.

    값의 주인은 params_file(기본 config/lane_follow.yaml)이다. speed 인자는
    **명시적으로 줄 때만** v_min · v_max 를 덮는다 -- 기본값 ''(빈 값)은
    "안 줬다"는 뜻이라 yaml 의 곡률 기반 가감속 설정이 그대로 산다.
    예전에는 기본값이 0.4 라 yaml 을 항상 상수 속도로 눌러 버렸다.
    """
    params_file = LaunchConfiguration('params_file')
    common = ['--ros-args', '--log-level', LaunchConfiguration('log_level')]
    use_sim_time = ParameterValue(
        LaunchConfiguration('use_sim_time'), value_type=bool)

    speed_overrides = {'use_sim_time': use_sim_time}
    raw_speed = LaunchConfiguration('speed').perform(context)
    if raw_speed:
        speed_overrides['speed.v_min'] = float(raw_speed)
        speed_overrides['speed.v_max'] = float(raw_speed)

    return [
        Node(
            package=PACKAGE,
            executable='speed_controller_node',
            name='speed_controller',
            output='screen',
            arguments=common,
            parameters=[params_file, speed_overrides],
        ),
        Node(
            package=PACKAGE,
            executable='steer_controller_node',
            name='steer_controller',
            output='screen',
            arguments=common,
            parameters=[params_file, {'use_sim_time': use_sim_time}],
        ),
    ]


def generate_launch_description():
    default_params = str(
        Path(get_package_share_directory(PACKAGE)) /
        'config' /
        'lane_follow.yaml'
    )

    detection_launch = str(
        Path(get_package_share_directory(DETECTION_PACKAGE)) /
        'launch' /
        'lane_detection.launch.py'
    )

    lane_detection = LaunchConfiguration('lane_detection')
    viewer = LaunchConfiguration('viewer')

    return LaunchDescription([

        # ------------------------------------------------------------
        # Launch Arguments
        # ------------------------------------------------------------

        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='제어 parameter yaml (두 노드 섹션이 다 들어 있다)',
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

        # yaml 에 speed.v_min · speed.v_max 가 있다. 빈 값이 기본이고,
        # 주면 그때만 둘을 같은 값으로 덮어 상수 속도로 만든다.
        DeclareLaunchArgument(
            'speed',
            default_value='',
            description=(
                '상수 속도 [m/s]. v_min = v_max 로 덮는다 (기본: yaml). '
                '0.0 이면 조향만 관찰한다'
            ),
        ),

        DeclareLaunchArgument(
            'lane_detection',
            default_value='true',
            description='차선 인지도 같이 띄울지. 이미 돌고 있으면 false',
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
        # 제어
        #
        # parameter 는 yaml 이 값의 주인이다. launch 인자는 명시적으로
        # 준 것만 덮는다 (controller_nodes 주석 참고).
        # ------------------------------------------------------------

        OpaqueFunction(function=controller_nodes),
    ])
