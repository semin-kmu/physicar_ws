#!/usr/bin/env python3
"""EKF/odom 의 누적 오차를 재부팅 없이 0 으로 되돌린다.

  ros2 run kau_localization odom_reset.py
  ros2 run kau_localization odom_reset.py --ekf skip   (EKF 를 곧 새로 띄울 때)

── 왜 재부팅해야만 초기화됐나 ──────────────────────────────────────
누적 오차는 EKF 가 아니라 **physicar_laser_odom 안에 있다.**

  laser_odom_node.cpp:120   pose_ = pose_.compose(result.delta);

pose_ 는 ICP 델타를 계속 더하는 멤버인데, 이 노드에는
리셋 서비스도, /set_pose 구독도, 초기화 파라미터도 **하나도 없다.**
프로세스를 새로 띄우는 것 말고는 0 으로 만들 방법이 없다.

시뮬레이션에서 월드 리로드로 오차가 사라졌던 이유가 이것이다.
리로드가 sim.launch.py 의 자식들을 다시 띄우면서 laser_odom 도 새로
시작해 pose_ 가 0 이 됐다. 실차는 physicar.service 가 부팅 때
real.launch.py 를 **한 번만** 띄우므로 laser_odom 이 전원을 끌 때까지
살아 있다. 그래서 재부팅 말고는 방법이 없었던 것이다.

── EKF 만 다시 켜면 왜 안 되나 ─────────────────────────────────────
ekf.yaml 에서 odom0 는 절대 pose 로 융합된다 (_differential, _relative
둘 다 false). 새 EKF 는 첫 /odom/laser 값에서 시작하므로 이미 흘러간
값을 그대로 물려받는다.

/set_pose 만 쏘는 것도 안 된다. 실측으로 확인했다 -- yaw 는 0 이
됐지만 x 가 곧바로 +0.142 로 되돌아왔다. 절대 pose 측정이 계속
들어오니 필터가 다시 그쪽으로 끌려간다.

그래서 **순서가 중요하다**: laser_odom 을 먼저 새로 띄워 소스를 0 으로
만들고, 그 다음에 EKF 상태를 0 으로 밀어 넣는다.

── 안전 ────────────────────────────────────────────────────────────
laser_odom 은 real.launch.py 에서 respawn=True, respawn_delay=2.0 으로
떠 있다. SIGTERM 을 보내면 launch 가 2 초 뒤 알아서 다시 띄운다.
이 스크립트는 죽이기만 하고 **직접 띄우지 않는다** -- 직접 띄우면
launch 가 나중에 하나를 더 띄워 두 개가 된다.

프로세스 탐색에 pgrep 을 쓰지 않는다. pgrep -f 는 패턴이 자기
명령줄에도 들어 있어서 **자기 자신을 찾아 죽인다.** /proc 를 직접
읽고 자기 PID 와 이 스크립트 이름이 든 명령줄을 걸러 낸다.
"""

import argparse
import math
import os
import signal
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

LASER_PATTERN = 'physicar_laser_odom/laser_odom_node'
RESPAWN_TIMEOUT = 20.0        # launch 의 respawn_delay 2.0 보다 넉넉히
NEAR_ZERO_M = 0.30            # 이 안이면 "새로 시작했다" 로 본다
NEAR_ZERO_DEG = 15.0
SELF_NAME = os.path.basename(__file__)

SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)


def targets(pattern):
    """pattern 이 명령줄에 든 PID 목록. 자기 자신은 절대 포함하지 않는다."""
    me = os.getpid()
    found = []
    for entry in os.listdir('/proc'):
        if not entry.isdigit():
            continue
        pid = int(entry)
        if pid == me:
            continue
        try:
            with open(f'/proc/{pid}/cmdline', 'rb') as fh:
                cmd = fh.read().replace(b'\0', b' ').decode('utf-8', 'replace')
        except OSError:
            continue                      # 이미 죽었거나 권한 없음
        if pattern in cmd and SELF_NAME not in cmd:
            found.append(pid)
    return found


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Watcher(Node):
    def __init__(self):
        super().__init__('odom_reset')
        self.last = None
        self.count = 0
        self.create_subscription(Odometry, '/odom/laser', self._on, SENSOR_QOS)

        # /set_pose 는 **글로벌 이름**이다 (노드 이름이 안 붙는다).
        # 서비스가 아니라 토픽을 쓴다: 서비스 쪽은 플랫폼 EKF 와 우리
        # EKF 두 개가 서버로 붙어 있어 어느 쪽으로 갈지 정해지지 않는다.
        self.set_pose = self.create_publisher(
            PoseWithCovarianceStamped, '/set_pose', 10)

    def _on(self, msg):
        self.last = msg
        self.count += 1

    def pose(self):
        if self.last is None:
            return None
        p = self.last.pose.pose
        return p.position.x, p.position.y, math.degrees(yaw_of(p.orientation))

    def spin(self, secs):
        end = time.monotonic() + secs
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--ekf', choices=['auto', 'skip'], default='auto',
                    help='auto: laser_odom 재시작 후 /set_pose 로 EKF 도 0 으로. '
                         'skip: laser_odom 만 재시작 (EKF 를 곧 새로 띄울 때).')
    args = ap.parse_args()

    rclpy.init()
    node = Watcher()

    node.spin(2.0)
    before = node.pose()
    if before is None:
        print('/odom/laser 가 안 온다. laser_odom 이 떠 있는지 확인할 것.',
              flush=True)
        rclpy.shutdown()
        return 1
    print(f'현재 /odom/laser: x {before[0]:+.3f}  y {before[1]:+.3f}  '
          f'yaw {before[2]:+.2f} deg', flush=True)

    old = targets(LASER_PATTERN)
    if not old:
        print(f'laser_odom 프로세스를 못 찾았다 (패턴 {LASER_PATTERN}).',
              flush=True)
        rclpy.shutdown()
        return 1
    print(f'laser_odom PID {old} 에 SIGTERM. launch 가 다시 띄운다.', flush=True)
    for pid in old:
        try:
            os.kill(pid, signal.SIGTERM)
        except OSError as ex:
            print(f'  PID {pid} 종료 실패: {ex}', flush=True)

    # 새 PID 가 뜨고, 그 PID 가 낸 pose 가 0 근처가 될 때까지 기다린다.
    node.count = 0
    node.last = None
    deadline = time.monotonic() + RESPAWN_TIMEOUT
    fresh = None
    while time.monotonic() < deadline:
        node.spin(0.5)
        now = targets(LASER_PATTERN)
        new = [p for p in now if p not in old]
        if not new or node.last is None:
            continue
        cand = node.pose()
        if (math.hypot(cand[0], cand[1]) < NEAR_ZERO_M
                and abs(cand[2]) < NEAR_ZERO_DEG):
            fresh = (new, cand)
            break

    if fresh is None:
        print(f'{RESPAWN_TIMEOUT:.0f} 초 안에 laser_odom 이 0 에서 다시 뜨지 '
              f'않았다. respawn 이 꺼져 있거나 launch 가 안 돌고 있다.',
              flush=True)
        rclpy.shutdown()
        return 1

    new_pids, after = fresh
    print(f'laser_odom 재시작 완료 PID {new_pids}: '
          f'x {after[0]:+.3f}  y {after[1]:+.3f}  yaw {after[2]:+.2f} deg',
          flush=True)

    if args.ekf == 'skip':
        print('EKF 는 건드리지 않는다 (--ekf skip). 곧 새로 뜨는 EKF 가 '
              '0 에서 시작한다.', flush=True)
        rclpy.shutdown()
        return 0

    msg = PoseWithCovarianceStamped()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.header.frame_id = 'odom'
    msg.pose.pose.orientation.w = 1.0
    for i in range(6):
        msg.pose.covariance[i * 7] = 1e-9
    node.set_pose.publish(msg)
    node.spin(1.0)
    node.set_pose.publish(msg)     # 구독 연결이 늦게 붙는 경우를 대비
    node.spin(1.0)
    print('/set_pose 에 0 을 발행했다. `ros2 topic echo /odom --once` 로 '
          '확인할 것.', flush=True)

    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
