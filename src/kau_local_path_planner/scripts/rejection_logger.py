#!/usr/bin/env python3
"""
/path/local 상태 모니터. KAU_AMET_Test 세션의 diagnose_local_failure.py 대응.

지금 KauPath 에는 candidate 별 탈락 사유(cusp/kappa_bound/road_boundary/
obstacle 등)를 실어보낼 필드가 없다 -- 그건 local_planner_node 내부에서만
계산되고 끝난다. 그래서 이 스크립트는 그 상세 사유까지는 못 보여주고,
와이어에 실제로 나가는 신호(confidence, valid_length, 발행 주기 끊김)만
본다.

    confidence < 1.0  -> local_planner_node 가 degraded 상태로 발행함
                         (Python 의 status="degraded" 대응. hard constraint
                         일부 위반한 최소위반 후보를 그대로 내보낸 것)
    발행 간격이 1/plan_hz 보다 훨씬 길어짐 -> status=no_feasible/degenerate
                         라 이번 사이클 아예 발행 안 함 (path=None)

candidate 별 정확한 탈락 사유가 필요하면 local_planner_node 에 진단 전용
토픽(비표준, 대회 메시지 계약 밖)을 추가하는 걸 별도로 검토할 것.
"""

import argparse

from kau_msgs.msg import KauPath

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy


class RejectionLogger(Node):
    def __init__(self, args):
        super().__init__('rejection_logger')
        self.args = args
        self.last_stamp = None
        self.degraded_count = 0
        self.total_count = 0

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.sub = self.create_subscription(
            KauPath, '/path/local', self._on_path, qos)
        self.create_timer(5.0, self._summary)

    def _on_path(self, msg: KauPath):
        now = self.get_clock().now()
        self.total_count += 1

        if self.last_stamp is not None:
            gap = (now - self.last_stamp).nanoseconds / 1e9
            expected = 1.0 / self.args.plan_hz
            if gap > expected * self.args.gap_factor:
                self.get_logger().warn(
                    f'/path/local 발행 간격 {gap:.3f}s (기대 {expected:.3f}s) '
                    '-- 그 사이 최소 1 사이클은 no_feasible/degenerate 였을 가능성')
        self.last_stamp = now

        if msg.confidence < 1.0:
            self.degraded_count += 1
            self.get_logger().warn(
                f'degraded 발행 (confidence={msg.confidence:.2f}) '
                f'nseg={len(msg.seg_length)} length={msg.total_length:.1f}cm')

    def _summary(self):
        if self.total_count == 0:
            return
        rate = 100.0 * self.degraded_count / self.total_count
        self.get_logger().info(
            f'최근 요약: 수신 {self.total_count}회, degraded {self.degraded_count}회 '
            f'({rate:.1f}%)')
        self.degraded_count = 0
        self.total_count = 0


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--plan-hz', type=float, default=5.0,
                   help='local_planner_node 의 plan_hz 와 맞출 것')
    p.add_argument('--gap-factor', type=float, default=2.5,
                   help='이 배수보다 발행 간격이 길면 경고')
    args, ros_args = p.parse_known_args()

    rclpy.init(args=ros_args)
    node = RejectionLogger(args)
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
