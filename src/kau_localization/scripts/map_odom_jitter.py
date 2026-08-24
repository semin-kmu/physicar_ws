#!/usr/bin/env python3
"""map -> odom 보정량이 얼마나 흔들리는지 측정.

map -> odom 은 "누적 드리프트 보정량"이므로 이상적으로는 아주 천천히만
변해야 한다. 이게 빠르게 흔들리면 RViz(fixed frame=map)에서 지도가 차를
따라 움직이는 것처럼 보인다.

같은 시간의 차량 속도(odom -> base_footprint 미분)와 같이 찍어서
"정지 중 노이즈 바닥" vs "주행/회전 중 흔들림" 을 비교한다.
"""
import argparse
import math
import time

import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class Jitter(Node):
    def __init__(self):
        super().__init__('map_odom_jitter')
        qos = QoSProfile(depth=200, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE,
                         history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(TFMessage, '/tf', self.cb, qos)
        self.mo = []   # (t, x, y, yaw)
        self.ob = []

    def cb(self, msg):
        for tr in msg.transforms:
            t = tr.header.stamp.sec + tr.header.stamp.nanosec * 1e-9
            p, c = tr.header.frame_id, tr.child_frame_id
            rec = (t, tr.transform.translation.x, tr.transform.translation.y,
                   yaw_of(tr.transform.rotation))
            if (p, c) == ('map', 'odom'):
                self.mo.append(rec)
            elif (p, c) == ('odom', 'base_footprint'):
                self.ob.append(rec)


def report(name, data, secs):
    if len(data) < 3:
        print(f'{name}: 데이터 부족 ({len(data)})')
        return
    t0 = data[0][0]
    xs = [d[1] for d in data]
    ys = [d[2] for d in data]
    yaws = [d[3] for d in data]
    # 전체 변동폭
    span_xy = math.hypot(max(xs) - min(xs), max(ys) - min(ys))
    span_yaw = math.degrees(max(yaws) - min(yaws))
    # 샘플 간 변화율 (초당)
    dxy, dyaw = [], []
    for a, b in zip(data, data[1:]):
        dt = b[0] - a[0]
        if dt <= 0:
            continue
        dxy.append(math.hypot(b[1] - a[1], b[2] - a[2]) / dt)
        dyaw.append(abs(wrap(b[3] - a[3])) / dt)
    dxy.sort()
    dyaw.sort()
    n = len(dxy)
    print(f'{name}  (샘플 {len(data)}, {data[-1][0]-t0:.1f} s)')
    print(f'   전체 변동폭      : xy {span_xy*100:7.2f} cm   yaw {span_yaw:7.3f} deg')
    print(f'   변화율 평균      : xy {sum(dxy)/n*100:7.2f} cm/s  '
          f'yaw {math.degrees(sum(dyaw)/n):7.3f} deg/s')
    print(f'   변화율 p99 / max : xy {dxy[int(0.99*(n-1))]*100:7.2f} / {dxy[-1]*100:.2f} cm/s  '
          f'yaw {math.degrees(dyaw[int(0.99*(n-1))]):7.3f} / {math.degrees(dyaw[-1]):.3f} deg/s')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--secs', type=float, default=15.0)
    a = ap.parse_args()
    rclpy.init()
    node = Jitter()
    end = time.time() + a.secs
    while rclpy.ok() and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
    print()
    report('map -> odom  (보정량. 느리게 변해야 정상)', node.mo, a.secs)
    print()
    report('odom -> base_footprint (차량 실제 움직임)', node.ob, a.secs)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
