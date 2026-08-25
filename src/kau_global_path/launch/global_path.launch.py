"""global path 발행.

    ros2 launch kau_global_path global_path.launch.py                 # 원본
    ros2 launch kau_global_path global_path.launch.py lane:=right_bias
    ros2 launch kau_global_path global_path.launch.py lane:=left_bias
    ros2 launch kau_global_path global_path.launch.py lane:=last_obstacle
    ros2 launch kau_global_path global_path.launch.py lane:=every_obstacle
    ros2 launch kau_global_path global_path.launch.py path:=/tmp/other.yaml rate:=0.0
    ros2 launch kau_global_path global_path.launch.py use_sim_time:=false   # 실기

`lane` 은 share 의 config/ 에서 파일을 고르는 이름표일 뿐이다 (4.7). `path` 를
직접 주면 그쪽이 이긴다 — 생성기로 막 뽑은 파일을 rebuild 없이 시험할 때 쓴다.

`route` 등 노드 파라미터의 값은 **config/global_path.yaml 에 있다**. 여기 인자는
빈 값이 기본이고, 주면 그때만 yaml 을 덮는다. 기본 경로는 `center_loop` 다 —
차로를 둘로 나눠 보지 않는다 (README 4.1.3). 변형 파일들도 레이어 이름이
`center` 라 `route` 는 그대로 두면 된다.

`use_sim_time` 은 다른 런치들(ekf / amcl / cartographer)과 같은 규칙이다 —
Gazebo 는 true, 실기는 false. **대회장에서는 네 런치 모두 false 를 준다.**

원래 이 런치에는 이 인자가 아예 없었다. 없으면 rclpy 기본값 false 라
시뮬에서도 두 노드만 벽시계를 봤다. 실제 피해는 크지 않았는데,
/path/global 을 받는 path_tracker 가 경로 stamp 가 아니라 TimePointZero
(최신 TF) 로 조회하기 때문이다. 다만 global_avoidance_publisher 의
obstacle_hold_s 타임아웃은 자기 시계로 재므로, Gazebo real-time factor 가
1.0 에서 벗어나면 장애물 유지 시간이 어긋난다. 그래서 붙였다.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# lane 이름 -> config/ 의 파일 이름.
LANES = {
    '': 'lane_graph.yaml',
    'center': 'lane_graph.yaml',
    'lane_graph': 'lane_graph.yaml',
    'right_bias': 'right_bias_lane.yaml',
    'left_bias': 'left_bias_lane.yaml',
    'last_obstacle': 'last_obstacle_lane.yaml',
    'every_obstacle': 'every_obstacle_lane.yaml',
}


def resolve_lane(lane, path):
    """(lane, path) -> 실제로 읽을 yaml 경로.

    path 가 있으면 그대로 쓴다. 없으면 lane 으로 고른다. 이름이 틀리면 조용히
    원본을 발행하지 않고 **바로 멈춘다** — 엉뚱한 경로로 도는 것보다 낫다.
    """
    if path:
        return path
    if lane not in LANES:
        raise RuntimeError(
            f'lane "{lane}" 을 모른다. 있는 것: {", ".join(k for k in LANES if k)} '
            '(또는 path:= 로 파일을 직접 준다)')

    share = Path(get_package_share_directory('kau_global_path')) / 'config'
    yaml_path = share / LANES[lane]
    if not yaml_path.is_file():
        raise RuntimeError(
            f'{yaml_path} 이 없다. 생성기를 돌리고 colcon build 를 다시 할 것 — '
            'python3 src/kau_global_path/scripts/gen_lane_variants.py')
    return str(yaml_path)


# launch 인자 이름 -> 형변환. yaml 의 같은 키를 덮는 자리다.
OVERRIDABLE = (
    ('route', str),
    ('topic', str),
    ('viz_topic', str),
    ('viz_spacing', float),
    ('rate', float),
    ('kappa_limit', float),
)


def launch_setup(context, *unused):
    share = Path(get_package_share_directory('kau_global_path')) / 'config'
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'

    # 값의 주인은 config/global_path.yaml 이다. launch 인자는 **명시적으로 준
    # 것만** 덮는다 -- 기본값 ''(빈 값)은 "안 줬다"는 뜻이라 dict 에 안 넣는다.
    # 예전에는 인자 기본값으로 dict 를 통째로 만들어 넘겨서, yaml 을 고쳐도
    # launch 기동에는 반영되지 않았고 bringup 경로와 값이 갈렸다.
    overrides = {'use_sim_time': use_sim_time}
    for key, cast in OVERRIDABLE:
        raw = LaunchConfiguration(key).perform(context)
        if raw:
            overrides[key] = cast(raw)

    # path 만 예외다. 설치 트리마다 절대경로가 달라 정적 yaml 에 못 적는다
    # (local_planner.yaml 의 track_yaml_path 와 같은 자리).
    overrides['path'] = resolve_lane(
        LaunchConfiguration('lane').perform(context),
        LaunchConfiguration('path').perform(context))

    node = Node(
        package='kau_global_path',
        executable='global_path_publisher.py',
        name='global_path_publisher',
        output='screen',
        parameters=[str(share / 'global_path.yaml'), overrides],
    )
    avoidance = Node(
        package='kau_global_path',
        executable='global_avoidance_publisher.py',
        name='global_avoidance_publisher',
        output='screen',
        parameters=[str(share / 'global_avoidance.yaml'),
                    {'use_sim_time': use_sim_time}],
        condition=IfCondition(LaunchConfiguration('avoidance')),
    )
    return [node, avoidance]


def generate_launch_description():
    args = [
        DeclareLaunchArgument('lane', default_value='',
                              description='빈 값(원본) · right_bias · left_bias · '
                                          'last_obstacle · every_obstacle (README 4.7)'),
        DeclareLaunchArgument('path', default_value='',
                              description='yaml 을 직접 지정한다. 주면 lane 보다 우선한다'),

        # 아래 다섯은 config/global_path.yaml 에 값이 있다. 빈 값이 기본이고,
        # 주면 그때만 yaml 을 덮는다.
        DeclareLaunchArgument('route', default_value='',
                              description='lane_graph.yaml 의 routes 이름 (기본: yaml)'),
        DeclareLaunchArgument('topic', default_value='',
                              description='기본: yaml'),
        DeclareLaunchArgument('viz_topic', default_value='',
                              description='기본: yaml'),
        DeclareLaunchArgument('viz_spacing', default_value='',
                              description='m. 시각화 리샘플 간격 (기본: yaml)'),
        DeclareLaunchArgument('rate', default_value='',
                              description='Hz. 0 이면 1 회만 발행 (기본: yaml)'),
        DeclareLaunchArgument('kappa_limit', default_value='',
                              description='1/m. 넘으면 경고만 낸다 (기본: yaml)'),
        DeclareLaunchArgument('avoidance', default_value='true',
                              description='/path/global_avoidance 노드도 함께 실행'),
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Gazebo 는 true, 실기는 false.'),
    ]

    return LaunchDescription(args + [OpaqueFunction(function=launch_setup)])
