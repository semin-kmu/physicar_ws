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

"""플랫폼 EKF 를 재워 두고, 그 자리를 대신하는 우리 EKF 를 지켜보는 감시자.

    python3 platform_ekf_pause.py <플랫폼 EKF 패턴> <우리 EKF 패턴>

ekf.launch.py 가 띄운다. 직접 부를 일은 없다.

하는 일 세 가지
    1. 플랫폼 EKF 를 계속 재워 둔다. 한 번만이 아니라 1 Hz 로 다시 확인한다.
    2. 우리 EKF 가 죽거나 멈추면 죽여서 launch 가 새로 띄우게 한다.
    3. 그래도 복구가 안 되면 플랫폼 EKF 를 깨우고 스스로 빠진다.

왜 1 번이 "계속" 이어야 하나
    sim.launch.py 는 맵 리로드 때 죽지 않는다. 자식 노드만 다시 뜬다. 그리고
    플랫폼 ekf_node 는 respawn=True 라 **새 PID 로** 살아난다. 시작할 때 한
    번만 얼리면 새로 뜬 놈은 그대로 돌아서 odom -> base_footprint 를 우리
    노드와 동시에 발행한다.

    이게 제일 위험한 고장이다. tf2 소유권은 (부모, 자식) 쌍으로만 결정되고
    /tf 는 중재자 없는 단일 토픽이라, 두 값이 번갈아 섞여 들어간다. "아예 안
    되는" 게 아니라 "가끔 튀는" 걸로 나타나서 원인 추적이 최악이다.

왜 2 번이 필요한가
    robot_localization 은 시간이 거꾸로 뛰면 (sim 재시작, 맵 리로드) 과거
    타임스탬프 측정을 버리고 조용히 발행을 멈춘다. 실측 2026-08-24: 맵
    리로드 후 /clock 이 1300 초대에서 74 초로 떨어지자 우리 EKF 가
    "Detected jump back in time" 만 찍고 /odom 발행을 멈췄다.

    플랫폼 EKF 는 sim.launch.py 의 자식이라 sim 과 같이 재시작돼서 이 문제를
    안 겪는다. 우리 것은 별도 launch 라 sim 보다 오래 살아남는다.

왜 3 번이 필요한가
    2 번이 실패하면 odom -> base_footprint 를 내는 노드가 하나도 없다. 플랫폼
    EKF 는 얼려 놨으니 대신할 것도 없다. **플랫폼 기본 상태보다 나쁜 실패
    모드**다. 최악이라도 원래대로 돌아가게 만든다.

건강 판정
    "입력은 살아 있는데 출력이 없다" 로 본다. sim 을 일시정지하면 입력도 같이
    멈추므로 오탐이 안 난다. 판정 시각은 sim 시간이 아니라 monotonic 벽시계로
    잰다 -- sim 시간이 얼어붙는 것 자체가 잡으려는 고장이기 때문이다.

왜 kill 이 아니라 SIGSTOP 인가 (플랫폼 EKF)
    respawn=True, respawn_delay=2.0 이라 죽이면 2 초 뒤 되살아난다. SIGSTOP 은
    프로세스를 살려 둔 채 멈추기만 해서 launch 가 보기엔 살아 있다.

대상을 어떻게 고르나
    pkill -f 는 **자기 자신의 argv 도 매치한다.** 패턴을 인자로 받는 이
    프로세스의 명령줄에 그 패턴이 그대로 들어 있어서, 그냥 pkill 을 쓰면
    자기 자신을 재워 버린다. 그래서 pgrep 으로 후보를 뽑은 뒤 자기 pid 를
    빼고, 명령줄에 robot_localization/ekf_node 가 실제로 들어 있는지 확인한
    다음에만 신호를 보낸다.

SIGKILL 로 이 프로세스가 죽으면 깨우기가 못 돈다. 그때는 손으로:
    pkill -CONT -f <플랫폼 패턴>
"""

import os
import signal
import subprocess
import sys
import time

from nav_msgs.msg import Odometry

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


# 신호를 보내기 전에 명령줄에 반드시 들어 있어야 하는 문자열. 패턴이 헐거워도
# 엉뚱한 프로세스를 멈추지 않게 하는 안전장치다.
REQUIRED_IN_CMDLINE = 'robot_localization/ekf_node'

INPUT_TOPIC = '/odom/laser'   # 우리 EKF 의 입력. 살아 있는지 본다
OUTPUT_TOPIC = '/odom'        # 우리 EKF 의 출력. 끊겼는지 본다

INPUT_FRESH_SEC = 3.0    # 이 안에 입력이 왔으면 "입력 살아 있음"
OUTPUT_DEAD_SEC = 5.0    # 이만큼 출력이 없으면 "출력 끊김"
GRACE_SEC = 15.0         # 기동 직후/재시작 직후 이만큼은 판정하지 않는다
MAX_RESTARTS = 3         # 이 횟수를 넘게 되살려도 안 되면 포기하고 원복한다

# 발행자가 BEST_EFFORT 여도 받을 수 있도록 구독자를 BEST_EFFORT 로 둔다.
# (RELIABLE 구독자는 BEST_EFFORT 발행자에게서 못 받는다. 반대는 된다.)
SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


def cmdline(pid):
    """Return the full command line of `pid`, or '' if it is gone."""
    try:
        with open(f'/proc/{pid}/cmdline', 'rb') as f:
            return f.read().replace(b'\0', b' ').decode('utf-8', 'replace')
    except OSError:
        return ''


def state(pid):
    """Return the single-letter process state of `pid` ('T' = stopped), or ''."""
    try:
        with open(f'/proc/{pid}/stat', 'rb') as f:
            # comm 에 공백이나 괄호가 들어갈 수 있으므로 마지막 ')' 뒤부터 읽는다.
            raw = f.read().decode('utf-8', 'replace')
        return raw[raw.rindex(')') + 2]
    except (OSError, ValueError, IndexError):
        return ''


def targets(pattern):
    """Find PIDs matching `pattern` that really are ekf_node processes."""
    found = subprocess.run(
        ['pgrep', '-f', pattern], capture_output=True, text=True, check=False)
    me = os.getpid()
    return [
        pid for pid in (int(t) for t in found.stdout.split())
        if pid != me and REQUIRED_IN_CMDLINE in cmdline(pid)
    ]


def send(sig, pattern, skip_state=None):
    """Signal every real target. `skip_state` skips PIDs already in that state."""
    sent = 0
    for pid in targets(pattern):
        if skip_state and state(pid) == skip_state:
            continue
        try:
            os.kill(pid, sig)
            sent += 1
        except OSError:
            # 그 사이에 죽었다. 대상이 사라진 것이니 문제될 게 없다.
            pass
    return sent


class Guard(Node):
    """Keep the platform EKF asleep and our EKF alive."""

    def __init__(self, platform_pattern, ours_pattern):
        """Subscribe to the input/output topics and start the 1 Hz check."""
        super().__init__('platform_ekf_guard')
        self.platform = platform_pattern
        self.ours = ours_pattern
        self.restarts = 0
        self.gave_up = False

        now = time.monotonic()
        self.last_input = 0.0
        self.last_output = 0.0
        self.grace_until = now + GRACE_SEC

        self.create_subscription(
            Odometry, INPUT_TOPIC, lambda _m: self._stamp('last_input'), SENSOR_QOS)
        self.create_subscription(
            Odometry, OUTPUT_TOPIC, lambda _m: self._stamp('last_output'), SENSOR_QOS)
        self.create_timer(1.0, self.tick)

    def _stamp(self, which):
        setattr(self, which, time.monotonic())

    def tick(self):
        """Re-freeze any new platform EKF, then check our EKF's health."""
        # 1. 새로 뜬 플랫폼 EKF 를 다시 재운다. 이미 멈춰 있는 놈은 건너뛴다.
        froze = send(signal.SIGSTOP, self.platform, skip_state='T')
        if froze:
            self.get_logger().warn(
                f'플랫폼 EKF {froze} 개가 새로 떠 있어서 다시 재웠다. '
                'sim 이 맵 리로드로 자식 노드를 다시 띄운 것으로 보인다.')

        if self.gave_up:
            return

        now = time.monotonic()
        if now < self.grace_until:
            return

        input_alive = (now - self.last_input) < INPUT_FRESH_SEC
        output_dead = (now - self.last_output) > OUTPUT_DEAD_SEC
        if not (input_alive and output_dead):
            return

        # 2. 입력은 오는데 출력이 없다. 우리 EKF 를 죽여서 launch 가 새로
        #    띄우게 한다 (Node(respawn=True)). 새 프로세스는 현재 시계로
        #    처음부터 시작하므로 시간 역행 문제가 사라진다.
        if self.restarts < MAX_RESTARTS:
            self.restarts += 1
            # 첫 시도는 SIGINT 로 곱게 내린다. 그래도 안 되면 SIGKILL 로 올린다 --
            # 프로세스가 멎어(T) 있거나 걸려 있으면 SIGINT 는 전달만 되고 처리가
            # 안 되기 때문이다. SIGKILL 은 멈춘 프로세스에도 먹는다.
            sig = signal.SIGINT if self.restarts == 1 else signal.SIGKILL
            killed = send(sig, self.ours)
            self.get_logger().error(
                f'{INPUT_TOPIC} 은 오는데 {OUTPUT_TOPIC} 이 '
                f'{now - self.last_output:.1f} 초째 없다. 우리 EKF {killed} 개를 '
                f'{sig.name} 으로 재시작한다 ({self.restarts}/{MAX_RESTARTS}). '
                'sim 재시작으로 시계가 거꾸로 뛰었을 때 생기는 증상이다.')
            self.grace_until = now + GRACE_SEC
            return

        # 3. 여러 번 되살려도 안 된다. 플랫폼 EKF 를 깨워서 최소한 원래 상태로
        #    돌려놓는다. 우리 EKF 는 그대로 두면 TF 를 이중 발행하므로 같이
        #    내린다 -- launch 쪽에서 이 프로세스가 죽으면 전체를 내리게 해뒀다.
        self.gave_up = True
        self.get_logger().fatal(
            f'{MAX_RESTARTS} 번 재시작해도 {OUTPUT_TOPIC} 이 안 돌아온다. '
            '플랫폼 EKF 를 깨우고 물러난다. 이후로는 플랫폼 기본 설정으로 돈다.')
        raise SystemExit(1)


def main():
    """Freeze the platform EKF, guard our EKF, and always thaw on the way out."""
    if len(sys.argv) != 3:
        print('usage: platform_ekf_pause.py <platform-pattern> <ours-pattern>',
              file=sys.stderr)
        return 2
    platform_pattern, ours_pattern = sys.argv[1], sys.argv[2]

    frozen = send(signal.SIGSTOP, platform_pattern, skip_state='T')
    if frozen:
        print(f'플랫폼 EKF {frozen} 개를 재웠다 (패턴: {platform_pattern})', flush=True)
    else:
        # 플랫폼 EKF 가 안 떠 있는 경우다. sim 을 아직 안 띄웠거나 이미 손으로
        # 죽인 상황이라 정상이다. 아래 tick() 이 나중에 뜨는 것도 잡아 준다.
        print(f'재울 플랫폼 EKF 가 없다 (패턴: {platform_pattern}). 그대로 진행한다.',
              flush=True)

    rclpy.init()
    node = Guard(platform_pattern, ours_pattern)

    alive = [True]

    def stop(_signum, _frame):
        alive[0] = False

    # rclpy 는 SIGINT 만 처리한다. SIGTERM 은 기본 동작이 즉시 종료라 finally 가
    # 안 돌고, 그러면 플랫폼 EKF 가 얼어붙은 채 남는다. 직접 잡는다.
    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGHUP, stop)

    try:
        while alive[0] and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        woke = send(signal.SIGCONT, platform_pattern)
        if woke:
            print(f'플랫폼 EKF {woke} 개를 깨웠다.', flush=True)
        else:
            print('깨울 플랫폼 EKF 가 없다 (이미 죽었으면 respawn 이 띄운다).',
                  flush=True)
        try:
            node.destroy_node()
            rclpy.shutdown()
        except Exception:
            # 종료 경로다. 여기서 나는 예외로 깨우기를 놓치면 안 된다.
            pass
    return 0


if __name__ == '__main__':
    sys.exit(main())
