"""백업 스택 전체 기동.

    ros2 launch backup_bringup backup.launch.py
    ros2 launch backup_bringup backup.launch.py use_sim_time:=false
    ros2 launch backup_bringup backup.launch.py skip:=backup_gui,backup_path_planner

각 패키지 launch 를 IncludeLaunchDescription 으로 조립한다.
파라미터 정의가 패키지 한 곳에만 있게 하기 위함이다.

★ /speed, /steering 을 발행한다. 기존 kau_* 스택과 절대 동시에 띄우지 않는다.
"""

import os
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction, LogInfo)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

# (노드 이름, 패키지, launch 파일) -- 기동 순서.
# 노드마다 launch 가 하나씩이라 skip 이 노드 단위로 먹는다.
STACK = [
    ('backup_start_signal_detector', 'backup_object_detection', 'start_signal.launch.py'),
    ('backup_obstacle_detector',     'backup_object_detection', 'obstacle_detector.launch.py'),
    ('backup_lane_detector',         'backup_lane_detection',   'lane_detection.launch.py'),
    ('backup_path_planner',          'backup_path_planner',     'path_planner.launch.py'),
    ('backup_speed_controller',      'backup_speed_controller', 'speed_controller.launch.py'),
    ('backup_steer_controller',      'backup_steer_controller', 'steer_controller.launch.py'),
    ('backup_gui',                   'backup_gui',              'gui.launch.py'),
]


def _setup(context, *args, **kwargs):
    # launch_arguments 로 넘기므로 문자열이어야 한다.
    # bool 변환은 각 패키지 launch 가 ParameterValue 로 처리한다.
    use_sim = LaunchConfiguration('use_sim_time').perform(context)
    log_level = LaunchConfiguration('log_level').perform(context)
    skip = {s.strip() for s in
            LaunchConfiguration('skip').perform(context).split(',') if s.strip()}

    known = {name for name, _, _ in STACK}
    unknown = skip - known
    if unknown:
        # 오타를 조용히 넘기면 끈 줄 알았던 노드가 그대로 뜬다.
        return [LogInfo(msg=f'[backup.launch] 모르는 노드 이름: {sorted(unknown)}. '
                            f'가능한 값: {sorted(known)}')]

    actions = []
    for name, pkg, launch_file in STACK:
        if name in skip:
            continue
        try:
            share = get_package_share_directory(pkg)
        except Exception:
            actions.append(LogInfo(msg=f'[backup.launch] {pkg} 없음 -- 건너뛴다'))
            continue
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', launch_file)),
            launch_arguments={'use_sim_time': use_sim, 'log_level': log_level}.items(),
        ))
    if skip:
        actions.insert(0, LogInfo(msg=f'[backup.launch] 제외: {sorted(skip)}'))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Gazebo true / 실차 false'),
        DeclareLaunchArgument('log_level', default_value='info'),
        DeclareLaunchArgument('skip', default_value='',
                              description='제외할 노드 이름 쉼표 구분'),
        OpaqueFunction(function=_setup),
    ])
