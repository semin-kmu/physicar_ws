#!/usr/bin/env python3
"""TF 링크별 발행 주기 측정.

/tf 는 여러 노드가 섞여서 쏘므로 `ros2 topic hz /tf` 로는 링크별 주기를
알 수 없다. 이 스크립트는 TransformStamped 하나하나를 (parent, child) 로
갈라서 각각의 주기를 잰다.

두 가지 주기를 같이 낸다.
  wall : 메시지가 도착한 벽시계 간격 (= ros2 topic hz 가 재는 것)
  stamp: 메시지 header.stamp 간격 (sim 시계. 배속이 걸리면 wall 과 달라진다)

사용법:
  ./tf_rate.py                 # 10 초 측정, 모든 링크
  ./tf_rate.py --secs 20
  ./tf_rate.py --link map odom # 특정 링크만
"""
import argparse
import time
from collections import defaultdict

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from tf2_msgs.msg import TFMessage
from rosgraph_msgs.msg import Clock


class TFRate(Node):
    def __init__(self, only):
        super().__init__('tf_rate')
        self.only = only
        self.wall = defaultdict(list)   # (p,c) -> [wall_t]
        self.stamp = defaultdict(list)  # (p,c) -> [stamp_t]
        qos = QoSProfile(depth=200, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE,
                         history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(TFMessage, '/tf', self.cb, qos)
        self.clock_wall = []
        self.clock_sim = []
        self.create_subscription(Clock, '/clock', self.clock_cb, 50)

    def clock_cb(self, msg):
        self.clock_wall.append(time.time())
        self.clock_sim.append(msg.clock.sec + msg.clock.nanosec * 1e-9)

    def cb(self, msg):
        now = time.time()
        for tr in msg.transforms:
            key = (tr.header.frame_id.lstrip('/'), tr.child_frame_id.lstrip('/'))
            if self.only and key != self.only:
                continue
            self.wall[key].append(now)
            self.stamp[key].append(tr.header.stamp.sec + tr.header.stamp.nanosec * 1e-9)


def stats(ts):
    if len(ts) < 2:
        return None
    d = [b - a for a, b in zip(ts, ts[1:])]
    d_sorted = sorted(d)
    n = len(d)
    mean = sum(d) / n
    var = sum((x - mean) ** 2 for x in d) / n
    return {
        'n': n + 1,
        'hz': (1.0 / mean) if mean > 0 else float('inf'),
        'mean_ms': mean * 1e3,
        'min_ms': d_sorted[0] * 1e3,
        'max_ms': d_sorted[-1] * 1e3,
        'p99_ms': d_sorted[int(0.99 * (n - 1))] * 1e3,
        'std_ms': var ** 0.5 * 1e3,
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--secs', type=float, default=10.0)
    ap.add_argument('--link', nargs=2, metavar=('PARENT', 'CHILD'), default=None)
    a = ap.parse_args()

    rclpy.init()
    node = TFRate(tuple(a.link) if a.link else None)
    t_end = time.time() + a.secs
    while rclpy.ok() and time.time() < t_end:
        rclpy.spin_once(node, timeout_sec=0.05)

    # sim 배속
    speed = None
    if len(node.clock_sim) > 10:
        dw = node.clock_wall[-1] - node.clock_wall[0]
        ds = node.clock_sim[-1] - node.clock_sim[0]
        if dw > 0:
            speed = ds / dw

    print(f'\n측정 {a.secs:.0f} s (벽시계)')
    if speed is not None:
        print(f'sim 시계 배속: {speed:.3f} x  (sim 초 / 벽시계 초)')
    print()
    hdr = (f'{"link":<34}{"wall Hz":>9}{"stamp Hz":>10}'
           f'{"mean ms":>9}{"min":>8}{"max":>8}{"p99":>8}{"std":>8}   n')
    print(hdr)
    print('-' * len(hdr))
    for key in sorted(node.wall, key=lambda k: -len(node.wall[k])):
        w = stats(node.wall[key])
        s = stats(node.stamp[key])
        if not w:
            continue
        name = f'{key[0]} -> {key[1]}'
        print(f'{name:<34}{w["hz"]:>9.2f}{(s["hz"] if s else 0):>10.2f}'
              f'{w["mean_ms"]:>9.2f}{w["min_ms"]:>8.2f}{w["max_ms"]:>8.2f}'
              f'{w["p99_ms"]:>8.2f}{w["std_ms"]:>8.2f}{w["n"]:>6}')
    print('\n(stamp Hz 는 sim 시계 기준 = 노드가 자기 시계로 의도한 주기)')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
