#!/usr/bin/env python3
"""
테스트용 KauPath 발행자.

kau_global_path / local path 가 아직 구현 전이라 경로를 발행하는 노드가 없다.
제어기 단독 검증용으로 `제어기_시뮬레이션_가이드.md` 의 시나리오와 같은 기하를
발행한다. **경로 발행자가 생기면 이 스크립트는 버린다.**

    ros2 run kau_control fake_path.py --shape straight --length 500
    ros2 run kau_control fake_path.py --shape curve --radius 150
    ros2 run kau_control fake_path.py --shape circle --radius 200

단위: cm (KauPath 규약). 좌표는 map 프레임.

제어점은 quintic Hermite -> Bezier 정확 기저변환으로 만든다
(sim_common/curve.py 의 hermite_to_bezier 와 동일식, numpy 없이).
인접 segment 가 경계 knot 의 (theta, kappa) 를 공유하므로 G2 가 자동 성립한다.
"""

import argparse
import math

from kau_msgs.msg import KauPath

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

DEGREE = 5
NCTRL = DEGREE + 1


def state_derivs(theta, kappa, sigma, dsigma=0.0):
    """
    (theta, kappa) -> (P', P'').

    T = (cos t, sin t),  N = (-sin t, cos t)
    P'  = sigma * T
    P'' = dsigma * T + sigma^2 * kappa * N
    """
    tx, ty = math.cos(theta), math.sin(theta)
    nx, ny = -math.sin(theta), math.cos(theta)
    d1 = (sigma * tx, sigma * ty)
    d2 = (dsigma * tx + sigma * sigma * kappa * nx,
          dsigma * ty + sigma * sigma * kappa * ny)
    return d1, d2


def hermite_to_bezier(p0, th0, k0, p1, th1, k1):
    """Quintic Hermite -> Bezier 제어점 6개. sigma 는 양단 직선거리."""
    chord = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
    (ax, ay), (bx, by) = state_derivs(th0, k0, chord)
    (cx, cy), (dx, dy) = state_derivs(th1, k1, chord)
    return [
        (p0[0], p0[1]),
        (p0[0] + ax / 5.0, p0[1] + ay / 5.0),
        (p0[0] + 2.0 * ax / 5.0 + bx / 20.0,
         p0[1] + 2.0 * ay / 5.0 + by / 20.0),
        (p1[0] - 2.0 * cx / 5.0 + dx / 20.0,
         p1[1] - 2.0 * cy / 5.0 + dy / 20.0),
        (p1[0] - cx / 5.0, p1[1] - cy / 5.0),
        (p1[0], p1[1]),
    ]


def _gauss_legendre_10():
    """[-1,1] 10점. curve.py 와 같은 값 (100 cm 당 절대오차 27 um)."""
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
    """
    표본 상한. 발행측 사전계산값이므로 근사로 충분하다.

    (제어기는 이 값을 쓰지 않고 kappaMaxOver 로 직접 정확값을 구한다)
    """
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


# --------------------------------------------------------------------------
# 시나리오 기하 (가이드 §제어기 시뮬레이션 구체화)
# --------------------------------------------------------------------------

def knots_straight(length_cm, nseg):
    """시나리오 1: 직진. 원점에서 +x 로."""
    step = length_cm / nseg
    return [((i * step, 0.0), 0.0, 0.0) for i in range(nseg + 1)], False


def knots_curve(radius_cm, entry=100.0, exit_=200.0):
    """시나리오 2: 직진 -> 90도 좌턴 -> 직진. 사분원호."""
    r = radius_cm
    k = 1.0 / r
    # 진입 직선 끝 = 원호 시작. 원호 중심은 (entry, r)
    knots = [
        ((0.0, 0.0), 0.0, 0.0),
        ((entry, 0.0), 0.0, 0.0),
    ]
    # 사분원을 3 조각으로 나눠 quintic 이 무리하지 않게 한다
    pieces = 3
    for i in range(1, pieces + 1):
        a = (math.pi / 2.0) * i / pieces
        px = entry + r * math.sin(a)
        py = r * (1.0 - math.cos(a))
        knots.append(((px, py), a, k))
    tip = knots[-1][0]
    knots.append(((tip[0], tip[1] + exit_), math.pi / 2.0, 0.0))
    return knots, False


def knots_circle(radius_cm, nseg=8):
    """폐곡선. window 추적(8.4) 과 wrap 처리 확인용. nseg >= 3 필수."""
    r = radius_cm
    k = 1.0 / r
    knots = []
    for i in range(nseg):
        a = 2.0 * math.pi * i / nseg
        knots.append(((r * math.sin(a), r * (1.0 - math.cos(a))),
                      a, k))
    return knots, True


def transform_ctrl(segs, theta, tx, ty):
    """
    제어점 강체변환. Bernstein 분할단위성으로 곡선 변환과 정확히 동치.

    curve.py 의 transform_ctrl 과 같은 식 (문서 7.1 아핀 불변성).
    """
    c, sn = math.cos(theta), math.sin(theta)
    return [[(c * x - sn * y + tx, sn * x + c * y + ty) for (x, y) in ctrl]
            for ctrl in segs]


def build_path(knots, closed):
    pairs = ([(i, (i + 1) % len(knots)) for i in range(len(knots))] if closed
             else [(i, i + 1) for i in range(len(knots) - 1)])
    segs = []
    for a, b in pairs:
        (pa, tha, ka), (pb, thb, kb) = knots[a], knots[b]
        segs.append(hermite_to_bezier(pa, tha, ka, pb, thb, kb))
    return segs


def to_msg(segs, closed, frame_id, stamp):
    msg = KauPath()
    msg.header.frame_id = frame_id
    msg.header.stamp = stamp
    msg.source = KauPath.SRC_GLOBAL
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
    msg.confidence = 1.0
    msg.valid_length = 0.0
    return msg


class FakePath(Node):
    def __init__(self, args):
        super().__init__('fake_path')

        if args.shape == 'straight':
            knots, closed = knots_straight(args.length, args.segments)
        elif args.shape == 'curve':
            knots, closed = knots_curve(args.radius)
        else:
            knots, closed = knots_circle(args.radius, max(3, args.segments))

        segs = build_path(knots, closed)
        if args.origin or args.yaw:
            ox, oy = (0.0, 0.0)
            if args.origin:
                ox, oy = (float(v) for v in args.origin.split(','))
            segs = transform_ctrl(segs, math.radians(args.yaw), ox, oy)
        self.msg = to_msg(segs, closed, args.frame,
                          self.get_clock().now().to_msg())

        qos = QoSProfile(depth=1,
                         reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(KauPath, args.topic, qos)
        self.pub.publish(self.msg)

        self.get_logger().info(
            f'{args.shape} 발행: nseg={len(self.msg.seg_length)} '
            f'길이={self.msg.total_length:.1f} cm '
            f'max|kappa|={max(self.msg.seg_kappa_max):.5f} 1/cm '
            f'-> {args.topic} ({args.frame})')

        # latched 이므로 한 번이면 되지만, volatile 구독자를 위해 주기 재발행
        if args.rate > 0.0:
            self.create_timer(1.0 / args.rate, self._republish)

    def _republish(self):
        self.msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.msg)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--shape', default='straight',
                   choices=['straight', 'curve', 'circle'])
    p.add_argument('--length', type=float, default=500.0, help='cm, straight')
    p.add_argument('--radius', type=float, default=150.0, help='cm')
    p.add_argument('--segments', type=int, default=4)
    p.add_argument('--topic', default='/path/local')
    p.add_argument('--frame', default='map')
    p.add_argument('--rate', type=float, default=1.0,
                   help='Hz, 0 이면 1회만 발행')
    p.add_argument('--origin', default=None,
                   help='cm, 경로 시작점 "X,Y" (차량 현재 위치에 맞출 때)')
    p.add_argument('--yaw', type=float, default=0.0,
                   help='deg, 경로 시작 방위')
    args, ros_args = p.parse_known_args()

    rclpy.init(args=ros_args)
    node = FakePath(args)
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
