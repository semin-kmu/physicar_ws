#!/usr/bin/env python3
"""
kau_local_path_planner_lane 단독 검증용 가짜 상류 발행자 (lane-only).

현재 노드가 실제로 구독하는 것만 낸다:

    /lane/center /lane/left /lane/right   KauPath, base_link, 14Hz
    /perception/obstacles                 ObstacleCircleArray, base_link, 10Hz
    /odom                                 nav_msgs/Odometry, 50Hz

    ros2 run kau_local_path_planner_lane fake_upstream.py --shape circle --radius 100

★ /odom 이 핵심이다. 없으면 previous_path_ 가 재정렬되지 않아 kappa0 가 0 에
  고정되고(코너를 직진으로 계획한다) continuity 항이 옛 frame 기준으로
  채점돼 곡선에서 횡오차가 발산한다. 노드가 5 초마다 WARN 을 찍는다.
  옛 버전은 이걸 안 냈다 -- 그래서 이 스크립트로는 그 버그를 못 봤다.

차선은 **자차 좌표계에서 고정된 등곡률 원호** 다 (반지름 --radius). 차는
--speed 로 그 위를 따라간다고 보고 /odom 을 적분해 낸다. 즉 "완벽 추종
가정" 이라, 플래너가 곡선을 어떻게 계획하는지만 본다.

장애물은 --obstacle X,Y (m, **기동 시점의 자차 기준**) 로 주면 그 자리에
말뚝처럼 서 있고, 차가 다가갔다가 지나쳐 간다.

    --shape straight                  직선 (기본은 circle)
    --radius 100                      곡률 반경 [cm], + 가 좌회전
    --half-width 35                   좌/우 edge 를 중앙선에서 얼마나 띄울지 [cm]
    --observed 168                    lane detection 관측 길이 [cm]
    --speed 0.5                       주행 속도 [m/s]
    --obstacle 0.8,0.0                장애물 1 개 [m]
    --no-edges                        /lane/left,/lane/right 를 안 냄
                                      (한쪽 edge 폴백 경로 확인용)

2026-08-26: /path/global 과 /steering 발행을 제거했다. 노드가 둘 다 구독하지
않는다 (global path 는 lane-only 재설계로, /steering 은 폐루프 방지로 끊겼다).
"""

import argparse
import math

from kau_msgs.msg import KauPath, ObstacleCircle, ObstacleCircleArray

from nav_msgs.msg import Odometry

from std_msgs.msg import Float64

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

DEGREE = 5


def state_derivs(theta, kappa, sigma, dsigma=0.0):
    tx, ty = math.cos(theta), math.sin(theta)
    nx, ny = -math.sin(theta), math.cos(theta)
    d1 = (sigma * tx, sigma * ty)
    d2 = (dsigma * tx + sigma * sigma * kappa * nx,
          dsigma * ty + sigma * sigma * kappa * ny)
    return d1, d2


def hermite_to_bezier(p0, th0, k0, p1, th1, k1):
    chord = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
    (ax, ay), (bx, by) = state_derivs(th0, k0, chord)
    (cx, cy), (dx, dy) = state_derivs(th1, k1, chord)
    return [
        (p0[0], p0[1]),
        (p0[0] + ax / 5.0, p0[1] + ay / 5.0),
        (p0[0] + 2.0 * ax / 5.0 + bx / 20.0, p0[1] + 2.0 * ay / 5.0 + by / 20.0),
        (p1[0] - 2.0 * cx / 5.0 + dx / 20.0, p1[1] - 2.0 * cy / 5.0 + dy / 20.0),
        (p1[0] - cx / 5.0, p1[1] - cy / 5.0),
        (p1[0], p1[1]),
    ]


def _gauss_legendre_10():
    x = [-0.9739065285171717, -0.8650633666889845, -0.6794095682990244,
         -0.4333953941292472, -0.1488743389816312, 0.1488743389816312,
         0.4333953941292472, 0.6794095682990244, 0.8650633666889845,
         0.9739065285171717]
    w = [0.0666713443086881, 0.1494513491505806, 0.2190863625159820,
         0.2692667193099963, 0.2955242247147529, 0.2955242247147529,
         0.2692667193099963, 0.2190863625159820, 0.1494513491505806,
         0.0666713443086881]
    return x, w


def _eval(ctrl, u):
    p = list(ctrl)
    n = len(p) - 1
    for r in range(n):
        for i in range(n - r):
            p[i] = ((1.0 - u) * p[i][0] + u * p[i + 1][0],
                    (1.0 - u) * p[i][1] + u * p[i + 1][1])
    return p[0]


def _hodograph(ctrl):
    n = len(ctrl) - 1
    return [(n * (ctrl[i + 1][0] - ctrl[i][0]),
             n * (ctrl[i + 1][1] - ctrl[i][1])) for i in range(n)]


def seg_length(ctrl):
    xs, ws = _gauss_legendre_10()
    d1 = _hodograph(ctrl)
    total = 0.0
    for x, w in zip(xs, ws):
        vx, vy = _eval(d1, 0.5 * (x + 1.0))
        total += w * math.hypot(vx, vy)
    return total * 0.5


def seg_kappa_max(ctrl, samples=200):
    d1 = _hodograph(ctrl)
    d2 = _hodograph(d1)
    best = 0.0
    for i in range(samples + 1):
        u = i / samples
        vx, vy = _eval(d1, u)
        ax, ay = _eval(d2, u)
        sp = math.hypot(vx, vy)
        if sp < 1e-9:
            continue
        best = max(best, abs(vx * ay - vy * ax) / (sp ** 3))
    return best


def knots_arc(length_cm, kappa, nseg, offset_cm=0.0):
    """
    자차 원점에서 +x 방향으로 시작하는 등곡률 원호의 knot 열.

    offset_cm 만큼 법선(+가 좌) 방향으로 민다 -- 좌/우 lane edge 용.
    오프셋 곡선의 곡률은 kappa/(1 - d*kappa) 다.
    """
    knots = []
    for i in range(nseg + 1):
        s_arc = length_cm * i / nseg
        th = kappa * s_arc
        if abs(kappa) < 1e-12:
            px, py = s_arc, 0.0
            k_off = 0.0
        else:
            px = math.sin(th) / kappa
            py = (1.0 - math.cos(th)) / kappa
            k_off = kappa / (1.0 - offset_cm * kappa)
        nx, ny = -math.sin(th), math.cos(th)
        knots.append(((px + offset_cm * nx, py + offset_cm * ny), th, k_off))
    return knots, False


def build_path(knots, closed):
    pairs = ([(i, (i + 1) % len(knots)) for i in range(len(knots))] if closed
             else [(i, i + 1) for i in range(len(knots) - 1)])
    segs = []
    for a, b in pairs:
        (pa, tha, ka), (pb, thb, kb) = knots[a], knots[b]
        segs.append(hermite_to_bezier(pa, tha, ka, pb, thb, kb))
    return segs


def to_kaupath(segs, closed, source, frame_id, stamp, confidence=1.0,
               valid_length=0.0):
    msg = KauPath()
    msg.header.frame_id = frame_id
    msg.header.stamp = stamp
    msg.source = source
    msg.degree = DEGREE
    msg.is_closed = closed
    msg.s_offset = 0.0
    for ctrl in segs:
        for (x, y) in ctrl:
            msg.ctrl_x.append(x)
            msg.ctrl_y.append(y)
        msg.seg_length.append(seg_length(ctrl))
        msg.seg_kappa_max.append(seg_kappa_max(ctrl))
    msg.total_length = float(sum(msg.seg_length))
    msg.confidence = confidence
    msg.valid_length = valid_length
    return msg


class FakeUpstream(Node):
    """자차 좌표계 lane 3종 + 장애물 + odom 발행. 개루프/폐루프 겸용.

    world(=odom) 프레임에 등곡률 원호를 하나 박아 두고, 자차 pose 를 그
    위에서 굴린다. 매 틱 자차 최근접점 s_car 를 구해
    [s_car + near, s_car + far] 구간만 잘라 **자차 기준으로 변환해** 낸다
    -- 카메라가 보는 창(window)을 그대로 흉내낸 것이다.

    개루프: 자차가 원호 위를 정확히 따라간다 (조향 무시).
    폐루프: /steering 을 받아 자전거 모델로 적분한다. 경로를 못 따라가면
            자차가 원호에서 벗어나고, 그 횡오차(cte)를 /debug/cte 로 낸다.
    """

    def __init__(self, args):
        super().__init__('fake_upstream')
        self.args = args
        self.kappa = 0.0 if args.shape == 'straight' else 1.0 / args.radius
        self.L = args.wheelbase          # cm
        self.steer = 0.0                 # rad

        reliable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        best_effort = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.center_pub = self.create_publisher(KauPath, '/lane/center', reliable)
        self.left_pub = self.create_publisher(KauPath, '/lane/left', reliable)
        self.right_pub = self.create_publisher(KauPath, '/lane/right', reliable)
        self.obstacle_pub = self.create_publisher(
            ObstacleCircleArray, '/perception/obstacles', best_effort)
        self.odom_pub = self.create_publisher(Odometry, '/odom', best_effort)
        self.cte_pub = self.create_publisher(Float64, '/debug/cte', best_effort)

        if args.closed_loop:
            self.create_subscription(Float64, '/steering', self._on_steer, 10)

        # 자차 pose (world=odom frame, cm/rad). 원호 위 s=0 에서 시작.
        self.x, self.y, self.yaw = 0.0, 0.0, 0.0

        self.obstacle_world = None
        if args.obstacle:
            ox, oy = (float(v) * 100.0 for v in args.obstacle.split(','))
            self.obstacle_world = (ox, oy)

        self.create_timer(1.0 / 50.0, self._tick_50hz)
        self.create_timer(1.0 / 14.0, self._lane_tick)
        self.create_timer(1.0 / 10.0, self._obstacle_tick)

        self.get_logger().info(
            f'fake_upstream: {"폐루프" if args.closed_loop else "개루프"} '
            f'shape={args.shape} '
            f'R={"inf" if self.kappa == 0.0 else f"{1.0 / self.kappa:.0f}cm"} '
            f'관측창 [{args.near:.1f}, {args.far:.1f}]cm '
            f'반폭 {args.half_width:.0f}cm v={args.speed:.2f}m/s')

    def _on_steer(self, msg):
        self.steer = float(msg.data)

    # ---- world 원호 기하 -------------------------------------------------
    def _arc_point(self, s):
        """호길이 s 지점의 (x, y, theta)."""
        if abs(self.kappa) < 1e-12:
            return s, 0.0, 0.0
        th = self.kappa * s
        return math.sin(th) / self.kappa, (1.0 - math.cos(th)) / self.kappa, th

    def _project(self):
        """자차의 (원호 호길이 s, 부호 있는 횡오차 cte[cm])."""
        if abs(self.kappa) < 1e-12:
            return self.x, self.y
        # 원호 P(s) - C = r*(sin u, -cos u),  u = kappa*s,  C = (0, r).
        # => u = atan2(vx*sgn, -vy*sgn),  cte = sgn*(|r| - |v|)
        #    (cte 부호: + 가 진행방향 왼쪽)
        r = 1.0 / self.kappa                      # 부호 있는 반경
        sgn = 1.0 if r > 0.0 else -1.0
        vx, vy = self.x - 0.0, self.y - r
        u = math.atan2(vx * sgn, -vy * sgn)
        s = u / self.kappa
        cte = sgn * (abs(r) - math.hypot(vx, vy))
        return s, cte

    # ---- 50Hz: pose 적분 + /odom + /debug/cte ----------------------------
    def _tick_50hz(self):
        dt = 1.0 / 50.0
        ds = self.args.speed * 100.0 * dt
        if self.args.closed_loop:
            # 자전거 모델. 조향각 -> yaw rate.
            dth = ds * math.tan(self.steer) / self.L
        else:
            dth = self.kappa * ds                  # 원호를 정확히 따라간다
        mid = self.yaw + 0.5 * dth
        self.x += ds * math.cos(mid)
        self.y += ds * math.sin(mid)
        self.yaw = math.atan2(math.sin(self.yaw + dth), math.cos(self.yaw + dth))

        now = self.get_clock().now().to_msg()
        m = Odometry()
        m.header.stamp = now
        m.header.frame_id = 'odom'
        m.child_frame_id = 'base_footprint'
        m.pose.pose.position.x = self.x / 100.0
        m.pose.pose.position.y = self.y / 100.0
        m.pose.pose.orientation.z = math.sin(0.5 * self.yaw)
        m.pose.pose.orientation.w = math.cos(0.5 * self.yaw)
        m.twist.twist.linear.x = self.args.speed
        self.odom_pub.publish(m)

        c = Float64()
        c.data = self._project()[1]
        self.cte_pub.publish(c)

    # ---- 14Hz: 관측창을 자차 기준으로 잘라 발행 --------------------------
    def _lane_tick(self):
        stamp = self.get_clock().now().to_msg()
        a = self.args
        s_car, _ = self._project()
        cy, sy = math.cos(-self.yaw), math.sin(-self.yaw)

        def emit(pub, offset_cm):
            knots = []
            n = a.segments
            for i in range(n + 1):
                s = s_car + a.near + (a.far - a.near) * i / n
                px, py, th = self._arc_point(s)
                nx, ny = -math.sin(th), math.cos(th)
                px += offset_cm * nx
                py += offset_cm * ny
                # world -> ego
                dx, dy = px - self.x, py - self.y
                ex = dx * cy - dy * sy
                ey = dx * sy + dy * cy
                k = (self.kappa / (1.0 - offset_cm * self.kappa)
                     if abs(self.kappa) > 1e-12 else 0.0)
                knots.append(((ex, ey), th - self.yaw, k))
            msg = to_kaupath(build_path(knots, False), False, KauPath.SRC_LANE,
                             a.frame, stamp, confidence=1.0,
                             valid_length=a.far - a.near)
            pub.publish(msg)

        emit(self.center_pub, 0.0)
        if not a.no_edges:
            emit(self.left_pub, a.half_width)
            emit(self.right_pub, -a.half_width)

    # ---- 10Hz: 장애물 ----------------------------------------------------
    def _obstacle_tick(self):
        msg = ObstacleCircleArray()
        msg.header.frame_id = self.args.frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.status = ObstacleCircleArray.STATUS_OK
        if self.obstacle_world is not None:
            dx = self.obstacle_world[0] - self.x
            dy = self.obstacle_world[1] - self.y
            c, s_ = math.cos(-self.yaw), math.sin(-self.yaw)
            circle = ObstacleCircle()
            circle.center_x = (dx * c - dy * s_) / 100.0
            circle.center_y = (dx * s_ + dy * c) / 100.0
            circle.radius = 0.09
            circle.confidence = 1.0
            msg.obstacles = [circle]
        self.obstacle_pub.publish(msg)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--shape', default='circle', choices=['circle', 'straight'])
    p.add_argument('--radius', type=float, default=100.0, help='cm, + 가 좌회전')
    p.add_argument('--near', type=float, default=42.9,
                   help='cm, 관측창 근거리 (base_link 기준). 이 안쪽은 안 보인다')
    p.add_argument('--far', type=float, default=128.0,
                   help='cm, 관측창 원거리. 근거가 닿는 상한')
    p.add_argument('--half-width', dest='half_width', type=float, default=35.0)
    p.add_argument('--segments', type=int, default=4)
    p.add_argument('--speed', type=float, default=0.5, help='m/s')
    p.add_argument('--wheelbase', type=float, default=18.0, help='cm')
    p.add_argument('--closed-loop', dest='closed_loop', action='store_true',
                   help='/steering 을 받아 자전거 모델로 적분 (횡오차 측정용)')
    p.add_argument('--frame', default='base_link')
    p.add_argument('--obstacle', default=None, help='m, "X,Y" 기동 시점 자차 기준')
    p.add_argument('--no-edges', dest='no_edges', action='store_true')
    args, ros_args = p.parse_known_args()

    rclpy.init(args=ros_args)
    node = FakeUpstream(args)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
