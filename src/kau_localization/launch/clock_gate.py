#!/usr/bin/env python3
# Copyright 2026 KAU AMET Team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""sim 시계가 실제로 흐르기 시작할 때까지 막아 두는 문지기.

    python3 clock_gate.py <타임아웃초>

ekf.launch.py 가 use_sim_time:=true 일 때만 띄운다. 직접 부를 일은 없다.
시계가 흐르면 0 으로, 타임아웃이면 1 로 빠진다. launch 는 0 을 받았을 때만
EKF 를 올린다.

왜 필요한가
    robot_localization 의 RosFilter::initialize() 첫 줄이 이렇다.

        if (!this->get_clock()->started()) {
          RCLCPP_INFO(get_logger(), "Waiting for clock to start...");
          this->get_clock()->wait_until_started();   // 타임아웃 없음
        }

    started() 는 "ROS 시간이 0 보다 큰가" 다. 그리고 이 대기는 rclcpp::spin()
    **앞에서** 돈다. 걸리면 노드가 통째로 멈춘 채 아무 로그도 더 안 찍는다.
    ekf_node 는 정상일 때도 조용해서, 화면만 보면 죽은 것과 구별이 안 된다.

    2026-08-24 실측 (sim 정상 동작 중, ekf_node 10 회 반복 기동):
        첫 /odom 발행까지   10/10 전부 0.09 ~ 0.24 초
        "Waiting for clock" 출력   4/10  (그중 최속이 0.09 초)

    즉 /clock 만 살아 있으면 저 메시지를 찍어도 0.1 초면 통과한다. 메시지를
    봤다는 것 자체는 고장이 아니다. 진짜로 걸리는 건 시계가 안 흐를 때다.

맵 리로드가 왜 시계를 멈추나
    physicar 의 sim_api.py 가 맵을 갈 때 gz 를 **정지 상태로** 띄운다
    (/opt/physicar/src/physicar-sim/sim_api.py:1566).

        subprocess.Popen(["gz", "sim", "-s", "--headless-rendering", path])
        #                              -r 가 없다 = paused 로 시작

    그리고 별도 스레드가 뒤늦게 풀어 준다 (같은 파일 1586 행부터).

        for i in range(120):                       # ① 최대 60 초 준비 폴링
            time.sleep(0.5)
            gz topic -l ...                        #    호출당 ~340 ms
        gz service ... --req "pause: false"        # ② 여기서야 unpause
        gz service ... /create                     #    차량 spawn
        pkill -f ros_gz_bridge/parameter_bridge    # ③ 브릿지를 죽인다

    구멍이 둘이다.

      ① -> ② 사이: gz 도 /clock 도 살아 있는데 sim time 이 0 에 멈춰 있다.
                   started() 가 계속 false 라 EKF 가 무한 대기한다.
      ③        : ROS 쪽 /clock 발행자인 parameter_bridge 를 죽였다가 ROS
                   launch 가 ~2 초 뒤 respawn 시킨다. 그 사이엔 발행자가 없다.

    이 창이 열려 있는 동안 launch 를 치면 걸리고, 닫힌 뒤에 치면 정상이다.
    사람 손 타이밍이라 체감상 절반쯤 걸린다. world 가 클수록 창이 길어진다.

왜 "0 보다 큰가" 가 아니라 "커지고 있나" 인가
    돌던 sim 을 손으로 일시정지하면 sim time 은 0 이 아닌 값에 멈춘다. 그때
    started() 는 true 라서 EKF 가 뜨긴 하는데 측정이 하나도 안 들어온다.
    어차피 쓸 수 없는 상태이므로 여기서 같이 막는다. 두 표본이 벽시계로
    MIN_GAP_SEC 이상 떨어진 채 sim time 이 증가했을 때만 통과시킨다.

왜 무한정 안 기다리나
    무한 대기는 지금 증상과 똑같아진다. 타임아웃이 나면 1 로 빠지고, launch
    가 그걸 받아 "왜 못 떴는지" 를 적고 전체를 내린다. 조용히 멈춰 있는 것
    보다 낫다. 기다리는 동안에도 PROGRESS_SEC 마다 지금 무엇을 기다리는지
    (발행자가 없는지 / 시계가 멈춰 있는지) 찍는다.
"""

import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from rosgraph_msgs.msg import Clock


CLOCK_TOPIC = '/clock'

# 두 표본 사이 최소 벽시계 간격. 이만큼 떨어진 표본에서 sim time 이 늘었으면
# "흐르고 있다" 로 본다.
MIN_GAP_SEC = 0.2

# 현재 상태를 찍는 주기. 화면이 멈춘 것처럼 보이는 게 이 문제의 절반이었다.
PROGRESS_SEC = 2.0

# 발행자가 BEST_EFFORT 여도 받을 수 있도록 구독자를 BEST_EFFORT 로 둔다.
# (RELIABLE 구독자는 BEST_EFFORT 발행자에게서 못 받는다. 반대는 된다.)
# rclcpp 의 TimeSource 가 /clock 에 쓰는 ClockQoS 와 같은 모양이다.
CLOCK_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


class Gate(Node):
    """Watch /clock and decide whether simulation time is actually running."""

    def __init__(self):
        """Subscribe to /clock and start with no samples seen."""
        super().__init__('clock_gate')
        self.first = None       # (벽시계, sim ns) 비교 기준 표본
        self.latest = None      # 가장 최근 sim ns
        self.samples = 0
        self.running = False
        self.create_subscription(Clock, CLOCK_TOPIC, self.on_clock, CLOCK_QOS)

    def on_clock(self, msg):
        """Mark the clock as running once sim time advances over a real gap."""
        now = time.monotonic()
        sim = msg.clock.sec * 1_000_000_000 + msg.clock.nanosec
        self.samples += 1
        self.latest = sim

        if sim <= 0:
            # 리로드 직후다. 기준 표본을 잡아 두면 0 을 "증가 전" 으로 오해한다.
            self.first = None
            return

        if self.first is None:
            self.first = (now, sim)
            return

        base_wall, base_sim = self.first
        if now - base_wall >= MIN_GAP_SEC:
            if sim > base_sim:
                self.running = True
            else:
                # 멈춘 채로 시간만 흘렀다. 기준을 당겨서 다시 잰다.
                self.first = (now, sim)

    def status(self):
        """One-line description of what we are currently waiting on."""
        if self.samples == 0:
            return (f'{CLOCK_TOPIC} 에 아무것도 안 온다. 발행자가 없다 '
                    '(ros_gz_bridge 재시작 중이거나 sim 이 안 떠 있다).')
        if self.latest == 0:
            return (f'{CLOCK_TOPIC} 은 오는데 sim time 이 0 이다. '
                    'gz 가 paused 로 떠 있고 아직 unpause 가 안 됐다.')
        return (f'sim time 이 {self.latest / 1e9:.2f} 초에 멈춰 있다. '
                'sim 이 일시정지 상태다.')


def main():
    """Block until simulation time advances, or give up after the timeout."""
    if len(sys.argv) != 2:
        print('usage: clock_gate.py <timeout-seconds>', file=sys.stderr)
        return 2
    timeout = float(sys.argv[1])

    rclpy.init()
    node = Gate()
    started = time.monotonic()
    next_report = started + PROGRESS_SEC

    # 성공 여부는 반드시 정리 **전에** 확정한다. rclpy.shutdown() 뒤에는
    # rclpy.ok() 가 무조건 False 라, 그걸로 판정하면 통과해도 실패로 빠진다.
    passed = False
    try:
        while rclpy.ok():
            if node.running:
                passed = True
                break
            if time.monotonic() - started > timeout:
                print(f'{timeout:.0f} 초 동안 sim 시계가 안 흘렀다. '
                      f'{node.status()}', flush=True)
                break
            rclpy.spin_once(node, timeout_sec=0.1)
            if time.monotonic() >= next_report:
                next_report = time.monotonic() + PROGRESS_SEC
                waited = time.monotonic() - started
                print(f'sim 시계 대기 {waited:.0f}초 -- {node.status()}', flush=True)
        if passed:
            print(f'sim 시계가 흐른다 (sim time {node.latest / 1e9:.2f} 초). '
                  'EKF 를 올린다.', flush=True)
    except KeyboardInterrupt:
        passed = False
    finally:
        try:
            node.destroy_node()
            rclpy.shutdown()
        except Exception:
            # 종료 경로다. 여기서 나는 예외로 반환값을 바꾸면 안 된다.
            pass

    return 0 if passed else 1


if __name__ == '__main__':
    sys.exit(main())
