#!/usr/bin/env python3
"""
Publish a slowly varying, obstacle-aware global reference path.

The static /path/global contract is intentionally untouched.  This node consumes
that path and the map-frame Object List, then publishes /path/global_avoidance.
All geometry in this file is in KauPath units (centimetres).

RViz has no display for KauPath, so the same curve also goes out as
/viz/path/global_avoidance (nav_msgs/Path, metres) for visualisation only.
"""

import bisect
import copy
import math
import signal
import sys

import numpy

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


# ====================================================================
# 경로_형식.md section 8 — 최근접점 / 호길이
#
# 이 절은 `경로_형식.md` 의 "연산별 알고리즘 지정" 을 그대로 구현한다.
# kau_control/bezier.hpp 와 **같은 알고리즘·같은 상수**이며, 그 패키지를
# import 하지 않고 여기에 둔다 (kau_lane_detection 이 bezier.hpp 사본을
# 갖는 것과 같은 이유 -- 패키지 간 의존을 늘리지 않는다).
#
# 2026-08-25 이전에는 5 cm 간격 표본 중 최근접 **표본**을 골랐다. 곡선 위로
# 투영하지 않으므로 규격 8.1 이 아니고, 표본 간격의 절반(2.5 cm)이 계통
# 오차로 남았다. 회피 판정이 lateral 을 cm 단위로 쓰므로 무시할 수 없다.
# ====================================================================

# 8.5 Gauss-Legendre 10점. [-1,1] 구간. 100 cm 당 절대오차 27 um.
_GL_X = (
    -0.9739065285171717, -0.8650633666889845,
    -0.6794095682990244, -0.4333953941292472,
    -0.1488743389816312, 0.1488743389816312,
    0.4333953941292472, 0.6794095682990244,
    0.8650633666889845, 0.9739065285171717,
)
_GL_W = (
    0.0666713443086881, 0.1494513491505806,
    0.2190863625159820, 0.2692667193099963,
    0.2955242247147529, 0.2955242247147529,
    0.2692667193099963, 0.2190863625159820,
    0.1494513491505806, 0.0666713443086881,
)


def _binom(n, k):
    return math.comb(n, k)


def to_power(ctrl):
    """Bezier 제어점 -> 거듭제곱 기저 계수 [(x, y)]. 낮은 차수부터."""
    n = len(ctrl) - 1
    coef = []
    for j in range(n + 1):
        cx = cy = 0.0
        for i in range(j + 1):
            m = _binom(n, j) * _binom(j, i) * (1.0 if (j - i) % 2 == 0 else -1.0)
            cx += m * ctrl[i][0]
            cy += m * ctrl[i][1]
        coef.append((cx, cy))
    return coef


def _poly_der(c):
    return [i * c[i] for i in range(1, len(c))] or [0.0]


def _poly_mul(a, b):
    out = [0.0] * (len(a) + len(b) - 1)
    for i, ai in enumerate(a):
        if ai == 0.0:
            continue
        for j, bj in enumerate(b):
            out[i + j] += ai * bj
    return out


def _poly_add(a, b):
    n = max(len(a), len(b))
    return [(a[i] if i < len(a) else 0.0) + (b[i] if i < len(b) else 0.0)
            for i in range(n)]


def real_roots_in_unit(coef, tol=1e-9):
    """8.2 동반행렬 고유값. (0,1) 안의 실근만. coef 는 낮은 차수부터.

    규격이 지정한 Python 구현 수단이 numpy.roots 이고, 그것이 곧 동반행렬
    고유값이다. numpy 는 계수를 높은 차수부터 받으므로 뒤집어 넘긴다.
    """
    c = list(coef)
    while len(c) > 1 and c[-1] == 0.0:
        c.pop()
    if len(c) <= 1:
        return []
    ev = numpy.roots(list(reversed(c)))
    scale = max([1.0] + [abs(v.real) for v in ev])
    return [v.real for v in ev
            if abs(v.imag) < tol * scale and 0.0 < v.real < 1.0]


def nearest_on_seg(ctrl, p):
    """8.1 최근접점. (u, dist). 반복법을 쓰지 않아 전역 최소가 보장된다.

    g(t) = (r(t) - p) . r'(t) 는 quintic 에서 정확히 9차.
    후보 = { g=0 의 (0,1) 실근 } U {0, 1}.
    """
    c = to_power(ctrl)
    cx = [q[0] for q in c]
    cy = [q[1] for q in c]
    gx = list(cx)
    gy = list(cy)
    gx[0] -= p[0]
    gy[0] -= p[1]
    g = _poly_add(_poly_mul(gx, _poly_der(cx)), _poly_mul(gy, _poly_der(cy)))

    best_u, best_d = 0.0, float('inf')
    for u in real_roots_in_unit(g) + [0.0, 1.0]:
        q = eval_bezier(ctrl, u)
        d = math.hypot(q[0] - p[0], q[1] - p[1])
        if d < best_d:
            best_u, best_d = u, d
    return best_u, best_d


def seg_length(ctrl, u0=0.0, u1=1.0):
    """8.5 호길이. Gauss-Legendre 10점."""
    half = 0.5 * (u1 - u0)
    mid = 0.5 * (u1 + u0)
    d1 = derivative_ctrl(ctrl)
    total = 0.0
    for x, w in zip(_GL_X, _GL_W):
        v = eval_bezier(d1, mid + half * x)
        total += w * math.hypot(v[0], v[1])
    return total * half


def invert_arclen(ctrl, s_local, length):
    """8.5 역산. safeguarded Newton (bracket 유지 + bisection fallback).

    u 를 s/length 로 두는 선형 근사는 quintic 이 호길이 매개변수가 아니라서
    곡률이 큰 구간에서 어긋난다. 2026-08-25 이전에는 그 근사를 썼다.
    """
    if length < 1e-12:
        return 0.0
    s_local = min(max(s_local, 0.0), length)
    lo, hi = 0.0, 1.0
    u = s_local / length
    d1 = derivative_ctrl(ctrl)
    for _ in range(5):
        h = seg_length(ctrl, 0.0, u) - s_local
        if abs(h) < 1e-9:
            break
        if h > 0.0:
            hi = u
        else:
            lo = u
        v = eval_bezier(d1, u)
        sp = math.hypot(v[0], v[1])
        u = (u - h / sp) if sp > 1e-12 else 0.5 * (lo + hi)
        if not (lo < u < hi):
            u = 0.5 * (lo + hi)
    return min(max(u, 0.0), 1.0)


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
    """KauPath 를 호길이로 다루는 뷰. 연산은 전부 `경로_형식.md` section 8.

    호길이는 msg.seg_length 가 아니라 제어점에서 다시 잰다 (8.5 GL10).
    kau_control 의 Curve 도 같게 하며, 그래야 station() 이 돌려준 s 를
    frame(s) 에 다시 넣었을 때 같은 점으로 돌아온다.
    """

    def __init__(self, msg):
        self.closed = msg.is_closed
        self.ctrl = []
        nctrl = msg.degree + 1
        for i in range(len(msg.seg_length)):
            start = i * nctrl
            self.ctrl.append(list(zip(msg.ctrl_x[start:start + nctrl],
                                      msg.ctrl_y[start:start + nctrl])))

        self.lengths = [seg_length(c) for c in self.ctrl]
        self.total = sum(self.lengths)

        self.cum = [0.0]
        for length in self.lengths:
            self.cum.append(self.cum[-1] + length)

        # 8.3 전역 탐색용 AABB. 제어점 볼록포가 곡선을 감싸므로
        # 제어점의 AABB 까지의 거리는 곡선까지 거리의 진하한이다.
        self.aabb = []
        for c in self.ctrl:
            xs = [q[0] for q in c]
            ys = [q[1] for q in c]
            self.aabb.append((min(xs), min(ys), max(xs), max(ys)))

    def _wrap_s(self, s):
        if self.total <= 0.0:
            return 0.0
        if self.closed:
            return s % self.total
        return min(max(s, 0.0), self.total)

    def frame(self, s):
        """호길이 -> (점, 접선각, 곡률). 역산은 8.5 safeguarded Newton."""
        s = self._wrap_s(s)
        i = bisect.bisect_right(self.cum, s) - 1
        i = min(max(i, 0), len(self.ctrl) - 1)
        u = invert_arclen(self.ctrl[i], s - self.cum[i], self.lengths[i])
        return segment_frame(self.ctrl[i], u)

    def _aabb_lower_bound(self, i, point):
        lo_x, lo_y, hi_x, hi_y = self.aabb[i]
        dx = max(lo_x - point[0], point[0] - hi_x, 0.0)
        dy = max(lo_y - point[1], point[1] - hi_y, 0.0)
        return math.hypot(dx, dy)

    def station(self, point):
        """8.3 전역 최근접점 -> (호길이 s, 횡거리). 좌측 +.

        AABB 하한 오름차순으로 훑고 하한이 현재 최소를 넘으면 중단한다.
        하한이 진하한이라 배제한 segment 가 최적일 가능성은 없다.
        """
        order = sorted(
            range(len(self.ctrl)),
            key=lambda i: self._aabb_lower_bound(i, point))

        best = (float('inf'), 0, 0.0)      # (dist, seg, u)
        for i in order:
            if self._aabb_lower_bound(i, point) >= best[0]:
                break
            u, dist = nearest_on_seg(self.ctrl[i], point)
            if dist < best[0]:
                best = (dist, i, u)

        _, i, u = best
        p, theta, _ = segment_frame(self.ctrl[i], u)
        s = self.cum[i] + seg_length(self.ctrl[i], 0.0, u)
        lateral = (-math.sin(theta) * (point[0] - p[0]) +
                   math.cos(theta) * (point[1] - p[1]))
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
    ref = ReferencePath(msg)
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
            'derivative_step_cm': 1.0,
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
