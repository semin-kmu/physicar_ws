#!/usr/bin/env python3
"""lane_graph.yaml 을 읽어 `/path/global` 로 발행한다.

`config/lane_graph.yaml` 의 `bezier:` 블록이 곧 발행 대상이다. 제어점을 그대로
싣는다 — **다시 피팅하지 않는다.** 편집기가 이음새에서 맞춰 둔 C2 가 깨진다
(README 4.4.1).

    ros2 run kau_global_path global_path_publisher.py
    ros2 run kau_global_path global_path_publisher.py --ros-args -p route:=center_loop
    ros2 run kau_global_path global_path_publisher.py --ros-args -p path:=/tmp/other.yaml

단위 — **KauPath 는 전부 cm 다.** yaml 은 map 프레임 미터다.

    ctrl_x, ctrl_y, seg_length, s_offset, total_length     m   -> cm   (x100)
    seg_kappa_max                                        1/m  -> 1/cm  (/100)

`seg_kappa_max` 만 방향이 반대다. 곡률은 길이의 역수라 그렇고, 이게 이 노드에서
제일 틀리기 쉬운 한 줄이다. 발행값은 `scripts/check_path.py` 의 "KauPath 환산"
출력과 그대로 대조된다.

발행되는 것

    /path/global      kau_msgs/KauPath   RELIABLE · TRANSIENT_LOCAL · depth 1 (latched)
    /viz/path/global  nav_msgs/Path      BEST_EFFORT   RViz 용. 호길이 등간격 리샘플
"""

import math
import sys
from pathlib import Path

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

import yaml
from ament_index_python.packages import get_package_share_directory

from geometry_msgs.msg import PoseStamped
from kau_msgs.msg import KauPath
from nav_msgs.msg import Path as NavPath

M_TO_CM = 100.0
DEGREE = 5
NCTRL = DEGREE + 1


# ---------------------------------------------------------------- Bezier

# check_path.py / lane_editor.js 와 같은 식이어야 한다. 여기서 갈라지면
# 검사를 통과한 파일이 다른 곡선으로 발행된다.

def de_casteljau(ctrl, u):
    p = [list(q) for q in ctrl]
    for r in range(len(p) - 1):
        for i in range(len(p) - 1 - r):
            p[i] = [(1 - u) * p[i][0] + u * p[i + 1][0],
                    (1 - u) * p[i][1] + u * p[i + 1][1]]
    return p[0]


def hodograph(ctrl):
    n = len(ctrl) - 1
    return [[n * (ctrl[i + 1][0] - ctrl[i][0]),
             n * (ctrl[i + 1][1] - ctrl[i][1])] for i in range(n)]


def theta_at(ctrl, u):
    v = de_casteljau(hodograph(ctrl), u)
    return math.atan2(v[1], v[0])


# ---------------------------------------------------------------- 파일

class LaneGraphError(RuntimeError):
    pass


def resolve_path(param):
    """빈 문자열이면 패키지 share 에서 찾는다."""
    if param:
        return Path(param).expanduser()
    return Path(get_package_share_directory('kau_global_path')) / 'config' / 'lane_graph.yaml'


def load_segments(path: Path, route: str):
    """(레이어, 조각 목록, 폐곡선, frame_id, map_source) 를 준다.

    route 이름으로 레이어를 고른다. `center_loop` -> `center`.
    """
    if not path.is_file():
        raise LaneGraphError(f'{path} 이 없다')

    doc = yaml.safe_load(path.read_text())
    if not isinstance(doc, dict) or 'lane_graph' not in doc:
        raise LaneGraphError(f'{path}: lane_graph 블록이 없다')

    g = doc['lane_graph']
    bez = g.get('bezier') or {}
    if not bez:
        raise LaneGraphError(
            f'{path}: bezier 블록이 없다 — 편집기에서 다시 내보낼 것 '
            '(예전 형식은 노드만 들고 있어 발행할 수 없다)')

    routes = g.get('routes') or {}
    layer = None

    if route in routes:
        # center_loop -> center, inner_loop -> lane_inner
        stem = route.rsplit('_', 1)[0]
        for cand in (stem, f'lane_{stem}'):
            if cand in bez:
                layer = cand
                break
    if layer is None and route in bez:
        layer = route

    if layer is None:
        raise LaneGraphError(
            f'route "{route}" 에 맞는 레이어가 없다. '
            f'routes {list(routes)} · bezier {list(bez)}')

    segs = bez[layer]
    if not segs:
        raise LaneGraphError(f'레이어 "{layer}" 의 조각이 비었다')

    for i, s in enumerate(segs):
        ctrl = s.get('ctrl')
        if not ctrl or len(ctrl) != NCTRL:
            raise LaneGraphError(
                f'조각 #{i}: 제어점이 {len(ctrl or [])} 개다. '
                f'{NCTRL} 개여야 한다 ({DEGREE}차 Bezier)')
        for q in ctrl:
            if not (isinstance(q, list) and len(q) == 2):
                raise LaneGraphError(f'조각 #{i}: 제어점이 [x, y] 쌍이 아니다')

    closed = bool((g.get('closed') or {}).get(layer, True))
    return layer, segs, closed, g.get('frame_id', 'map'), g.get('map_source', '?')


# ---------------------------------------------------------------- 메시지

def to_kaupath(segs, closed, frame_id, stamp):
    """조각 목록 -> KauPath. **여기서 m 을 cm 로 바꾼다.**"""
    msg = KauPath()
    msg.header.frame_id = frame_id
    msg.header.stamp = stamp

    msg.source = KauPath.SRC_GLOBAL
    msg.degree = DEGREE
    msg.is_closed = closed
    msg.s_offset = 0.0

    for s in segs:
        for x, y in s['ctrl']:
            msg.ctrl_x.append(float(x) * M_TO_CM)
            msg.ctrl_y.append(float(y) * M_TO_CM)

        msg.seg_length.append(float(s['length']) * M_TO_CM)

        # 곡률만 나눈다. [1/m] -> [1/cm]
        msg.seg_kappa_max.append(float(s['kappa_max']) / M_TO_CM)

    msg.total_length = float(sum(msg.seg_length))
    msg.confidence = 1.0
    msg.valid_length = 0.0        # Lane Detection 전용. Global 은 0
    return msg


def to_navpath(segs, closed, frame_id, stamp, spacing_m):
    """RViz 용. **미터 그대로** 다 (nav_msgs 는 SI 다).

    t 를 등분하면 급한 구간에서 점이 성겨지므로 조각마다 호길이에 비례해
    개수를 잡는다. 발행 경로가 아니라 그림용이라 이 정도면 충분하다.
    """
    p = NavPath()
    p.header.frame_id = frame_id
    p.header.stamp = stamp

    for s in segs:
        ctrl = s['ctrl']
        n = max(2, int(math.ceil(float(s['length']) / spacing_m)))
        for i in range(n):
            u = i / n
            q = de_casteljau(ctrl, u)
            th = theta_at(ctrl, u)

            ps = PoseStamped()
            ps.header.frame_id = frame_id
            ps.header.stamp = stamp
            ps.pose.position.x = q[0]
            ps.pose.position.y = q[1]
            ps.pose.orientation.z = math.sin(th / 2.0)
            ps.pose.orientation.w = math.cos(th / 2.0)
            p.poses.append(ps)

    if closed and p.poses:
        p.poses.append(p.poses[0])
    else:
        ctrl = segs[-1]['ctrl']
        q = de_casteljau(ctrl, 1.0)
        th = theta_at(ctrl, 1.0)
        ps = PoseStamped()
        ps.header.frame_id = frame_id
        ps.header.stamp = stamp
        ps.pose.position.x = q[0]
        ps.pose.position.y = q[1]
        ps.pose.orientation.z = math.sin(th / 2.0)
        ps.pose.orientation.w = math.cos(th / 2.0)
        p.poses.append(ps)

    return p


# ---------------------------------------------------------------- 노드

class GlobalPathPublisher(Node):

    def __init__(self):
        super().__init__('global_path_publisher')

        self.declare_parameter('path', '')
        self.declare_parameter('route', 'center_loop')
        self.declare_parameter('topic', '/path/global')
        self.declare_parameter('viz_topic', '/viz/path/global')
        self.declare_parameter('viz_spacing', 0.05)
        self.declare_parameter('rate', 1.0)
        self.declare_parameter('kappa_limit', 1.8199)

        p = lambda k: self.get_parameter(k).value

        yaml_path = resolve_path(p('path'))
        route = p('route')

        layer, segs, closed, frame_id, map_source = load_segments(yaml_path, route)
        stamp = self.get_clock().now().to_msg()

        self.msg = to_kaupath(segs, closed, frame_id, stamp)
        self.viz = to_navpath(segs, closed, frame_id, stamp, float(p('viz_spacing')))

        # latched. 늦게 뜬 구독자도 마지막 값을 받는다.
        qos = QoSProfile(depth=1,
                         reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(KauPath, p('topic'), qos)

        # 시각화는 놓쳐도 되는 데이터다. RViz 기본과 맞춰 BEST_EFFORT.
        self.viz_pub = self.create_publisher(NavPath, p('viz_topic'), 1)

        self.pub.publish(self.msg)
        self.viz_pub.publish(self.viz)

        nseg = len(self.msg.seg_length)
        kmax_cm = max(self.msg.seg_kappa_max)
        kmax_m = kmax_cm * M_TO_CM
        limit = float(p('kappa_limit'))

        self.get_logger().info(
            f'{yaml_path.name} [{map_source}] · 레이어 {layer} · route {route}\n'
            f'  조각 {nseg} · 제어점 {len(self.msg.ctrl_x)} '
            f'(= {nseg} x {NCTRL}) · 폐곡선 {closed}\n'
            f'  총 길이 {self.msg.total_length:.2f} cm ({self.msg.total_length / M_TO_CM:.3f} m)\n'
            f'  max|kappa| {kmax_cm:.6f} 1/cm ({kmax_m:.4f} 1/m · R {1 / kmax_m:.3f} m)\n'
            f'  -> {p("topic")} (KauPath, latched) · '
            f'{p("viz_topic")} (nav_msgs/Path, {len(self.viz.poses)} 점)')

        if kmax_m > limit:
            self.get_logger().error(
                f'곡률 한계 초과다. max|kappa| {kmax_m:.4f} > {limit:.4f} 1/m. '
                '그대로 발행했지만 차가 못 도는 구간이 있다 — '
                'check_path.py 로 어느 조각인지 확인할 것')

        # latched 라 한 번이면 되지만, VOLATILE 로 구독하는 노드와
        # RViz 재시작을 위해 주기 재발행한다. rate 0 이면 1 회만.
        rate = float(p('rate'))
        if rate > 0.0:
            self.create_timer(1.0 / rate, self._republish)

    def _republish(self):
        stamp = self.get_clock().now().to_msg()
        self.msg.header.stamp = stamp
        self.viz.header.stamp = stamp
        self.pub.publish(self.msg)
        self.viz_pub.publish(self.viz)


def main():
    rclpy.init()
    try:
        node = GlobalPathPublisher()
    except LaneGraphError as e:
        rclpy.logging.get_logger('global_path_publisher').fatal(str(e))
        rclpy.try_shutdown()
        return 1

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
