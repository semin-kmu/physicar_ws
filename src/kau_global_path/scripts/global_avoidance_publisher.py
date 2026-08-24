#!/usr/bin/env python3
"""
Publish a slowly varying, obstacle-aware global reference path.

The static /path/global contract is intentionally untouched.  This node consumes
that path and the map-frame Object List, then publishes /path/global_avoidance.
All geometry in this file is in KauPath units (centimetres).

RViz has no display for KauPath, so the same curve also goes out as
/viz/path/global_avoidance (nav_msgs/Path, metres) for visualisation only.
"""

import copy
import math
import signal
import sys

from geometry_msgs.msg import PoseStamped

from kau_msgs.msg import KauPath, ObstacleCircleArray

from nav_msgs.msg import Path as NavPath

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions

CM_TO_M = 0.01


def eval_bezier(ctrl, u):
    pts = [list(p) for p in ctrl]
    for level in range(1, len(pts)):
        for i in range(len(pts) - level):
            pts[i][0] = (1.0 - u) * pts[i][0] + u * pts[i + 1][0]
            pts[i][1] = (1.0 - u) * pts[i][1] + u * pts[i + 1][1]
    return tuple(pts[0])


def derivative_ctrl(ctrl):
    n = len(ctrl) - 1
    return [(n * (ctrl[i + 1][0] - ctrl[i][0]),
             n * (ctrl[i + 1][1] - ctrl[i][1])) for i in range(n)]


def segment_frame(ctrl, u):
    d1 = eval_bezier(derivative_ctrl(ctrl), u)
    d2 = eval_bezier(derivative_ctrl(derivative_ctrl(ctrl)), u)
    p = eval_bezier(ctrl, u)
    speed2 = d1[0] * d1[0] + d1[1] * d1[1]
    theta = math.atan2(d1[1], d1[0])
    kappa = 0.0 if speed2 < 1e-12 else (
        d1[0] * d2[1] - d1[1] * d2[0]) / (speed2 ** 1.5)
    return p, theta, kappa


def _hermite_to_bezier(a, b, sigma_ratio):
    p0, th0, k0 = a
    p1, th1, k1 = b
    chord = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
    sigma = sigma_ratio * chord
    t0 = (math.cos(th0), math.sin(th0))
    t1 = (math.cos(th1), math.sin(th1))
    n0 = (-t0[1], t0[0])
    n1 = (-t1[1], t1[0])
    d10 = (sigma * t0[0], sigma * t0[1])
    d11 = (sigma * t1[0], sigma * t1[1])
    d20 = (sigma * sigma * k0 * n0[0], sigma * sigma * k0 * n0[1])
    d21 = (sigma * sigma * k1 * n1[0], sigma * sigma * k1 * n1[1])
    return [
        p0,
        (p0[0] + .2 * d10[0], p0[1] + .2 * d10[1]),
        (p0[0] + .4 * d10[0] + .05 * d20[0],
         p0[1] + .4 * d10[1] + .05 * d20[1]),
        (p1[0] - .4 * d11[0] + .05 * d21[0],
         p1[1] - .4 * d11[1] + .05 * d21[1]),
        (p1[0] - .2 * d11[0], p1[1] - .2 * d11[1]),
        p1,
    ]


def hermite_to_bezier(a, b):
    """Use the small sigma search from the Local Planner's quintic fitter."""
    candidates = [_hermite_to_bezier(a, b, ratio) for ratio in (.85, 1.05, 1.30)]
    return min(candidates, key=lambda ctrl: segment_metrics(ctrl, samples=24)[1])


def segment_metrics(ctrl, samples=40):
    length = 0.0
    kmax = 0.0
    prev = eval_bezier(ctrl, 0.0)
    for i in range(samples + 1):
        u = i / samples
        p, _, kappa = segment_frame(ctrl, u)
        if i:
            length += math.hypot(p[0] - prev[0], p[1] - prev[1])
        prev = p
        kmax = max(kmax, abs(kappa))
    return length, kmax


def to_navpath(msg, spacing_cm):
    """Convert a KauPath to nav_msgs/Path for RViz.  Metres, because nav_msgs is SI.

    Point count is proportional to arc length per segment; splitting u evenly
    thins the samples out exactly where curvature is highest.
    """
    out = NavPath()
    out.header = copy.deepcopy(msg.header)

    def pose(point, theta):
        ps = PoseStamped()
        ps.header = out.header
        ps.pose.position.x = point[0] * CM_TO_M
        ps.pose.position.y = point[1] * CM_TO_M
        ps.pose.orientation.z = math.sin(theta / 2.0)
        ps.pose.orientation.w = math.cos(theta / 2.0)
        return ps

    nctrl = msg.degree + 1
    ctrl = []
    for k, length in enumerate(msg.seg_length):
        start = k * nctrl
        ctrl = list(zip(msg.ctrl_x[start:start + nctrl],
                        msg.ctrl_y[start:start + nctrl]))
        count = max(2, int(math.ceil(length / spacing_cm)))
        for i in range(count):
            point, theta, _ = segment_frame(ctrl, i / count)
            out.poses.append(pose(point, theta))

    # u = 1.0 of each segment is the next one's u = 0.0, so it is skipped above.
    # Only the very end needs closing: back to the start, or the last endpoint.
    if msg.is_closed and out.poses:
        out.poses.append(out.poses[0])
    elif ctrl:
        point, theta, _ = segment_frame(ctrl, 1.0)
        out.poses.append(pose(point, theta))
    return out


class ReferencePath:
    def __init__(self, msg, sample_step_cm=5.0):
        self.closed = msg.is_closed
        self.ctrl = []
        nctrl = msg.degree + 1
        for i in range(len(msg.seg_length)):
            start = i * nctrl
            self.ctrl.append(list(zip(msg.ctrl_x[start:start + nctrl],
                                      msg.ctrl_y[start:start + nctrl])))
        self.lengths = list(msg.seg_length)
        self.total = sum(self.lengths)
        self.samples = []
        s0 = 0.0
        for ctrl, length in zip(self.ctrl, self.lengths):
            count = max(2, int(math.ceil(length / sample_step_cm)))
            for j in range(count):
                u = j / count
                p, theta, kappa = segment_frame(ctrl, u)
                self.samples.append((s0 + length * u, p, theta, kappa))
            s0 += length

    def frame(self, s):
        if self.closed:
            s %= self.total
        else:
            s = min(max(s, 0.0), self.total)
        acc = 0.0
        for ctrl, length in zip(self.ctrl, self.lengths):
            if s <= acc + length or ctrl is self.ctrl[-1]:
                return segment_frame(ctrl, min(1.0, max(0.0, (s - acc) / length)))
            acc += length
        return segment_frame(self.ctrl[-1], 1.0)

    def station(self, point):
        s, p, theta, _ = min(
            self.samples,
            key=lambda q: (q[1][0] - point[0]) ** 2 + (q[1][1] - point[1]) ** 2)
        lateral = -math.sin(theta) * (point[0] - p[0]) + math.cos(theta) * (point[1] - p[1])
        return s, lateral

    def delta(self, a, b):
        d = b - a
        if self.closed:
            d = (d + .5 * self.total) % self.total - .5 * self.total
        return d


def smooth_bump(distance, half_length):
    x = abs(distance) / half_length
    if x >= 1.0:
        return 0.0
    # cos^4 has zero first derivative at the ends and a broad, smooth apex.
    return math.cos(.5 * math.pi * x) ** 4


def plan_avoidance(msg, obstacles, params, invert=False):
    ref = ReferencePath(msg, params['projection_step_cm'])
    active = []
    clearance_extra = params['body_radius_cm'] + params['safety_margin_cm']
    for x, y, radius in obstacles:
        station, lateral = ref.station((x, y))
        clearance = radius + clearance_extra
        if abs(lateral) >= clearance + params['activation_margin_cm']:
            continue
        targets = (lateral - clearance, lateral + clearance)
        target = min(targets, key=abs)
        if invert:
            target = max(targets, key=abs)
        target = max(-params['max_offset_cm'], min(params['max_offset_cm'], target))
        active.append((station, target))

    if not active:
        return copy.deepcopy(msg), 0, True, 0.0

    half_length = params['avoidance_half_length_cm']

    def offset(s):
        weighted = [(smooth_bump(ref.delta(station, s), half_length), target)
                    for station, target in active]
        denom = sum(w for w, _ in weighted)
        if denom <= 1e-12:
            return 0.0
        # Weighted average prevents nearby cones asking for twice the offset.
        return sum(w * target for w, target in weighted) / max(1.0, denom)

    def displaced(s):
        p, theta, _ = ref.frame(s)
        d = offset(s)
        return (p[0] - math.sin(theta) * d, p[1] + math.cos(theta) * d)

    eps = params['derivative_step_cm']

    def knot(s):
        pm = displaced(s - eps)
        p = displaced(s)
        pp = displaced(s + eps)
        dx = (pp[0] - pm[0]) / (2.0 * eps)
        dy = (pp[1] - pm[1]) / (2.0 * eps)
        ddx = (pp[0] - 2.0 * p[0] + pm[0]) / (eps * eps)
        ddy = (pp[1] - 2.0 * p[1] + pm[1]) / (eps * eps)
        speed2 = dx * dx + dy * dy
        kappa = 0.0 if speed2 < 1e-12 else (dx * ddy - dy * ddx) / (speed2 ** 1.5)
        return p, math.atan2(dy, dx), kappa

    # Preserve every original Bezier segment outside an avoidance window.  A
    # whole-loop refit slightly inflates the already tight hairpin curvature.
    controls = []
    spacing = params['fit_spacing_cm']
    segment_start = 0.0
    for original, length in zip(ref.ctrl, ref.lengths):
        probes = [segment_start + length * i / 8.0 for i in range(9)]
        if max(abs(offset(s)) for s in probes) < 1e-6:
            controls.append(original)
        else:
            count = max(1, int(math.ceil(length / spacing)))
            stations = [segment_start + length * i / count for i in range(count + 1)]
            knots = [knot(s) for s in stations]
            controls.extend(hermite_to_bezier(a, b) for a, b in zip(knots, knots[1:]))
        segment_start += length
    metrics = [segment_metrics(ctrl) for ctrl in controls]
    kmax = max(k for _, k in metrics)

    safe = kmax <= params['kappa_max_vehicle'] * params['kappa_margin']
    for x, y, radius in obstacles:
        required = radius + clearance_extra
        best = min(math.hypot(p[0] - x, p[1] - y)
                   for ctrl in controls
                   for i in range(21)
                   for p in [eval_bezier(ctrl, i / 20.0)])
        safe = safe and best >= required - params['collision_tolerance_cm']

    out = KauPath()
    out.header = copy.deepcopy(msg.header)
    out.source = KauPath.SRC_GLOBAL
    out.degree = 5
    out.is_closed = msg.is_closed
    out.s_offset = msg.s_offset
    for ctrl, (length, kappa) in zip(controls, metrics):
        for x, y in ctrl:
            out.ctrl_x.append(x)
            out.ctrl_y.append(y)
        out.seg_length.append(length)
        out.seg_kappa_max.append(kappa)
    out.total_length = sum(out.seg_length)
    out.confidence = 1.0 if safe else 0.0
    out.valid_length = 0.0
    return out, len(active), safe, kmax


class GlobalAvoidancePublisher(Node):
    def __init__(self):
        super().__init__('global_avoidance_publisher')
        defaults = {
            'publish_hz': 1.0, 'obstacle_hold_s': 1.0,
            'body_radius_cm': 11.0353, 'safety_margin_cm': 2.0,
            'activation_margin_cm': 3.0, 'max_offset_cm': 35.0,
            'avoidance_half_length_cm': 120.0, 'fit_spacing_cm': 35.0,
            'projection_step_cm': 5.0, 'derivative_step_cm': 1.0,
            'kappa_max_vehicle': 0.020221, 'kappa_margin': 0.95,
            'collision_tolerance_cm': 0.5,
            'global_topic': '/path/global',
            'obstacle_topic': '/perception/obstacles',
            'output_topic': '/path/global_avoidance',
            'viz_topic': '/viz/path/global_avoidance',
            'viz_spacing_cm': 5.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.params = {name: self.get_parameter(name).value for name in defaults}
        self.global_path = None
        self.obstacles = []
        self.last_ok_time = None

        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        live = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                          durability=DurabilityPolicy.VOLATILE)
        self.global_sub = self.create_subscription(
            KauPath, self.params['global_topic'], self.on_global, latched)
        self.obstacle_sub = self.create_subscription(
            ObstacleCircleArray, self.params['obstacle_topic'], self.on_obstacles, live)
        self.publisher = self.create_publisher(KauPath, self.params['output_topic'], latched)
        # RViz subscribes RELIABLE/VOLATILE by default; match it so the
        # display fills in without touching the topic's QoS in the GUI.
        self.viz_pub = self.create_publisher(NavPath, self.params['viz_topic'], 1)
        self.timer = self.create_timer(1.0 / self.params['publish_hz'], self.publish_path)

    def on_global(self, msg):
        if msg.degree != 5 or not msg.seg_length:
            self.get_logger().error('지원하지 않는 /path/global: degree=5, non-empty 필요')
            return
        self.global_path = msg

    def on_obstacles(self, msg):
        if msg.header.frame_id != 'map':
            self.get_logger().warning('장애물 frame_id가 map이 아니어서 무시: ' + msg.header.frame_id)
            return
        if msg.status != ObstacleCircleArray.STATUS_OK:
            return
        self.obstacles = [(o.center_x * 100.0, o.center_y * 100.0, o.radius * 100.0)
                          for o in msg.obstacles]
        self.last_ok_time = self.get_clock().now()

    def publish_path(self):
        if self.global_path is None:
            return
        obstacles = self.obstacles
        age_s = math.inf if self.last_ok_time is None else (
            self.get_clock().now() - self.last_ok_time).nanoseconds * 1e-9
        if age_s > self.params['obstacle_hold_s']:
            obstacles = []
        candidates = [plan_avoidance(self.global_path, obstacles, self.params, invert=False)]
        if obstacles and not candidates[0][2]:
            candidates.append(plan_avoidance(
                self.global_path, obstacles, self.params, invert=True))
        valid = [c for c in candidates if c[2]]
        if valid:
            path, count, safe, kmax = min(valid, key=lambda c: c[3])
        else:
            path = copy.deepcopy(self.global_path)
            count, safe, kmax = candidates[0][1], False, candidates[0][3]
            path.confidence = 0.0
            self.get_logger().error('안전한 global avoidance 후보 없음; 원본 경로를 confidence=0으로 발행')
        path.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(path)
        self.viz_pub.publish(to_navpath(path, self.params['viz_spacing_cm']))
        self.get_logger().info(
            f'global avoidance 발행: 활성 장애물={count}, safe={safe}, max|kappa|={kmax:.6f} 1/cm',
            throttle_duration_sec=5.0)


def main():
    """SIGINT/SIGTERM 을 직접 받아 곱게 내려온다.

    rclpy 기본 신호 처리는 컨텍스트를 **비동기로** 내린다. 신호가 온 직후
    타이머 콜백이 한 번 더 돌면 이미 죽은 발행자에 publish 하게 되어
    RCLError("publisher's context is invalid") 로 터진다. ExternalShutdownException
    이 아니라서 안 잡히고, 종료 코드가 0 이 아니게 되어 kau_state_machine
    supervisor 의 사망 판정을 오염시킨다 (실측 2026-08-24: 종료 시 rc=1).

    신호를 직접 받아 루프를 빠져나오면 그 창 자체가 없다.
    """
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = GlobalAvoidancePublisher()

    alive = [True]

    def stop(_signum, _frame):
        alive[0] = False

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    try:
        while alive[0] and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
