"""global path 발행.

    ros2 launch kau_global_path global_path.launch.py                 # 원본
    ros2 launch kau_global_path global_path.launch.py lane:=right_bias
    ros2 launch kau_global_path global_path.launch.py lane:=last_obstacle
    ros2 launch kau_global_path global_path.launch.py lane:=every_obstacle
    ros2 launch kau_global_path global_path.launch.py path:=/tmp/other.yaml rate:=0.0

`lane` 은 share 의 config/ 에서 파일을 고르는 이름표일 뿐이다 (4.7). `path` 를
직접 주면 그쪽이 이긴다 — 생성기로 막 뽑은 파일을 rebuild 없이 시험할 때 쓴다.

기본 경로는 `center_loop` 다 — 차로를 둘로 나눠 보지 않는다 (README 4.1.3).
변형 파일들도 레이어 이름이 `center` 라 `route` 는 그대로 두면 된다.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# lane 이름 -> config/ 의 파일 이름.
LANES = {
    '': 'lane_graph.yaml',
    'center': 'lane_graph.yaml',
    'lane_graph': 'lane_graph.yaml',
    'right_bias': 'right_bias_lane.yaml',
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


def launch_setup(context, *unused):
    cfg = {k: LaunchConfiguration(k).perform(context)
           for k in ('lane', 'path', 'route', 'topic', 'viz_topic',
                     'viz_spacing', 'rate', 'kappa_limit')}

    node = Node(
        package='kau_global_path',
        executable='global_path_publisher.py',
        name='global_path_publisher',
        output='screen',
        parameters=[{
            'route': cfg['route'],
            'path': resolve_lane(cfg['lane'], cfg['path']),
            'topic': cfg['topic'],
            'viz_topic': cfg['viz_topic'],
            'viz_spacing': float(cfg['viz_spacing']),
            'rate': float(cfg['rate']),
            'kappa_limit': float(cfg['kappa_limit']),
        }],
    )
    return [node]


def generate_launch_description():
    args = [
        DeclareLaunchArgument('lane', default_value='',
                              description='빈 값(원본) · right_bias · '
                                          'last_obstacle · every_obstacle (README 4.7)'),
        DeclareLaunchArgument('route', default_value='center_loop',
                              description='lane_graph.yaml 의 routes 이름'),
        DeclareLaunchArgument('path', default_value='',
                              description='yaml 을 직접 지정한다. 주면 lane 보다 우선한다'),
        DeclareLaunchArgument('topic', default_value='/path/global'),
        DeclareLaunchArgument('viz_topic', default_value='/viz/path/global'),
        DeclareLaunchArgument('viz_spacing', default_value='0.05',
                              description='m. 시각화 리샘플 간격'),
        DeclareLaunchArgument('rate', default_value='1.0',
                              description='Hz. 0 이면 1 회만 발행 (latched 라 그래도 받는다)'),
        DeclareLaunchArgument('kappa_limit', default_value='1.8199',
                              description='1/m. 넘으면 경고만 낸다 (README 6.4)'),
    ]

    return LaunchDescription(args + [OpaqueFunction(function=launch_setup)])
