#!/usr/bin/env python3
"""
kau_local_path_planner 단독 검증용 가짜 상류 발행자.

kau_control/scripts/fake_path.py 와 같은 방식(quintic Hermite -> Bezier,
numpy 없이)으로 /path/global, /lane/center, /perception/obstacles 를
한 번에 발행한다. 다른 팀원 노드 없이 local_planner_node 하나만 켜놓고
테스트할 때 쓴다 (KAU_AMET_Test 세션의 sources.py 대응).

/steering 도 같이 내지만 2026-08-25 부터 local_planner_node 는 이걸
구독하지 않는다 (P0 곡률은 이전 계획 경로에서 온다). steer_controller
쪽 확인용으로만 남겨둔 것이라 --steer-deg 는 플래너 결과에 영향이 없다.

    ros2 run kau_local_path_planner fake_upstream.py --shape circle --radius 300

TF(map -> base_footprint) 는 이 스크립트가 만들지 않는다 -- static_transform_publisher
나 별도 localization mock 을 같이 띄울 것. 예:

    ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map base_footprint
"""

import argparse
import math

from kau_msgs.msg import KauPath, ObstacleCircle, ObstacleCircleArray

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64

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


def knots_circle(radius_cm, nseg=12):
    r = radius_cm
    k = 1.0 / r
    return [((r * math.sin(2.0 * math.pi * i / nseg),
              r * (1.0 - math.cos(2.0 * math.pi * i / nseg))),
             2.0 * math.pi * i / nseg, k) for i in range(nseg)], True


def knots_straight(length_cm, nseg=2):
    step = length_cm / nseg
    return [((i * step, 0.0), 0.0, 0.0) for i in range(nseg + 1)], False


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
    def __init__(self, args):
        super().__init__('fake_upstream')

        if args.shape == 'circle':
            knots, closed = knots_circle(args.radius, max(6, args.segments))
        else:
            knots, closed = knots_straight(args.length, max(2, args.segments))
        self.global_msg = to_kaupath(
            build_path(knots, closed), closed, KauPath.SRC_GLOBAL, args.frame,
            self.get_clock().now().to_msg())

        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        reliable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        best_effort = QoSProfile(depth=1,
                                 reliability=ReliabilityPolicy.BEST_EFFORT)

        self.global_pub = self.create_publisher(KauPath, '/path/global', latched)
        self.global_pub.publish(self.global_msg)
        self.get_logger().info(
            f'/path/global 발행: {args.shape} nseg={len(self.global_msg.seg_length)} '
            f'길이={self.global_msg.total_length:.1f}cm')

        self.lane_pub = self.create_publisher(KauPath, '/lane/center', reliable)
        self.obstacle_pub = self.create_publisher(
            ObstacleCircleArray, '/perception/obstacles', best_effort)
        self.steering_pub = self.create_publisher(Float64, '/steering', 10)

        self.args = args
        self.t0 = self.get_clock().now()
        self.create_timer(0.1, self._tick)   # 10Hz, /lane/center 규약과 동일

    def _tick(self):
        now = self.get_clock().now()
        stamp = now.to_msg()

        # 짧은 직선 lane (관측 30~100cm 앞이라고 가정 -- 실제 카메라 모델
        # 대신 아주 단순한 고정 형태).
        lane_knots = [((30.0, 0.0), 0.0, 0.0), ((100.0, 0.0), 0.0, 0.0)]
        lane_msg = to_kaupath(
            build_path(lane_knots, False), False, KauPath.SRC_LANE,
            self.args.frame, stamp, confidence=1.0, valid_length=70.0)
        self.lane_pub.publish(lane_msg)

        obstacles = ObstacleCircleArray()
        obstacles.header.frame_id = self.args.frame
        obstacles.header.stamp = stamp
        obstacles.status = ObstacleCircleArray.STATUS_OK
        if self.args.obstacle:
            ox, oy = (float(v) for v in self.args.obstacle.split(','))
            circle = ObstacleCircle()
            circle.center_x = ox
            circle.center_y = oy
            circle.radius = 0.09      # m, 실제 콘 반경 (kau_object_detection 기준)
            circle.confidence = 1.0
            obstacles.obstacles = [circle]
        self.obstacle_pub.publish(obstacles)

        steer = Float64()
        steer.data = math.radians(self.args.steer_deg)
        self.steering_pub.publish(steer)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--shape', default='circle', choices=['circle', 'straight'])
    p.add_argument('--radius', type=float, default=300.0, help='cm, circle')
    p.add_argument('--length', type=float, default=1000.0, help='cm, straight')
    p.add_argument('--segments', type=int, default=12)
    p.add_argument('--frame', default='map')
    p.add_argument('--steer-deg', type=float, default=0.0,
                   help='/steering 고정값 [deg], 좌회전 + (플래너는 안 쓴다)')
    p.add_argument('--obstacle', default=None,
                   help='m, "X,Y" map frame -- 지정 시 장애물 1개 계속 발행')
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
