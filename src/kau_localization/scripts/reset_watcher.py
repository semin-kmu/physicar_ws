#!/usr/bin/env python3
"""
PhysiCar 앱의 "리셋" / "재시작" 버튼을 감지해서 localization 을 자동으로 이어붙인다.

장기 주행에서 localization 이 계속 살아있는지 보려고 만든 것이다. 앱에서 버튼을
누를 때마다 손으로 launch 를 다시 칠 필요가 없다.

두 버튼은 성격이 달라서 대응도 다르다.

  리셋 (light reset)
      차량 pose 만 스폰 지점으로 순간이동한다. sim 시계는 그대로 흐른다.
      여기서 재런치하면 장기 검증 자체가 끊기므로 (trajectory · 드리프트 이력이
      날아간다) cartographer 를 죽이지 않는다. 스폰 좌표를 map 프레임으로 바꿔
      /initialpose 로 쏘면 initial_pose_relay 가 trajectory 를 갈아끼운다.
      -> cartographer 도 RViz 도 끊기지 않고 그대로 이어진다.

  재시작 (respawn)
      gz 월드가 통째로 다시 뜨고 /clock 이 0 으로 되감긴다. 이 상태로
      cartographer 를 살려두면 TF 버퍼에 미래 stamp 가 남아 스스로 복구하지
      못한다. 그래서 반드시 재런치한다 (RViz 도 새로 뜬다). 시계가 다시
      정상적으로 흐르고 /scan · /odom 이 돌아온 뒤에 띄우고, 저장된 오프셋으로
      /initialpose 까지 자동으로 넣어 바로 수렴시킨다.

감지 방법
    버튼은 원격 앱(sim.physicar.ai)에 있고 눌리면 sim_api 로 HTTP 요청만 간다.
    ROS 쪽으로는 토픽도 이벤트도 나오지 않는다 (/events SSE 에도 없다).
    로컬에 남는 유일한 흔적이 sim_api 로그라 그걸 tail 한다.

        "light reset done"                     -> 리셋
        "starting sim:"                        -> 재시작 시작 (여기서 즉시 죽인다)
        "physicar spawned, starting gz-launch" -> 재시작 준비 완료 (여기서 띄운다)

    sim_api.py 는 updater 가 덮어쓰는 벤더 파일이라 거기에 훅을 심지 않았다.

시계가 어긋나도 버티는 법
    이 스크립트 자체는 벽시계로 돈다 (use_sim_time=false). sim 시계가 멈추거나
    되감겨도 감시 로직은 얼지 않는다. TF 는 stamp 를 지정하지 않고 항상 latest
    로만 조회하고, 되감김을 감지하면 TF 버퍼를 비운다. /initialpose 의 header
    stamp 는 initial_pose_relay 가 아예 보지 않으므로 문제가 되지 않는다.

좌표 보정
    스폰 좌표는 gz 월드 좌표계로 나오고 /initialpose 는 map 프레임이라 둘
    사이의 고정 변환이 필요하다. 처음 한 번, localization 이 수렴된 상태에서
    Enter 를 누르면 계산해서 저장하고 (~/.ros/kau_reset_watcher_offset.json)
    다음 실행부터는 그대로 재사용한다. --offset / --spawn-pose 로 직접 줄 수도
    있다.

사용법
    source /opt/ros/jazzy/setup.bash
    source install/setup.bash
    ./src/kau_localization/scripts/reset_watcher.py

    ./src/kau_localization/scripts/reset_watcher.py --no-rviz
    ./src/kau_localization/scripts/reset_watcher.py --recalibrate
    ./src/kau_localization/scripts/reset_watcher.py --relaunch-on-reset
    ./src/kau_localization/scripts/reset_watcher.py --spawn-pose 0,0,0

    Ctrl-C 로 끄면 자기가 띄운 localization 도 같이 정리한다.
"""

import argparse
import json
import math
import os
import queue
import re
import signal
import subprocess
import sys
import threading
import time
import urllib.request
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

import tf2_ros
from geometry_msgs.msg import PoseWithCovarianceStamped
from rosgraph_msgs.msg import Clock


PKG_DIR = Path(__file__).resolve().parent.parent
DEFAULT_SIM_LOG = '/tmp/sim_api.log'
DEFAULT_SIM_API = 'http://127.0.0.1:9003'
OFFSET_FILE = Path.home() / '.ros' / 'kau_reset_watcher_offset.json'

MAP_FRAME = 'map'
# cartographer 의 tracking_frame (physicar_2d.lua) 과 맞춰야 한다.
BASE_FRAME = 'base_footprint'

# sim_api 로그에서 잡아낼 줄. 문자열이 바뀌면 여기만 고치면 된다.
PAT_RESET = re.compile(r'light reset done')
PAT_SIM_STARTING = re.compile(r'starting sim:')
PAT_SIM_READY = re.compile(r'physicar spawned, starting gz-launch')

EV_RESET = 'reset'
EV_SIM_STARTING = 'sim_starting'
EV_SIM_READY = 'sim_ready'


def wrap(angle):
    """[-pi, pi) 로 접는다."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def yaw_of(qx, qy, qz, qw):
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


def stamp_now():
    return time.strftime('%H:%M:%S')


def say(msg):
    print(f'[{stamp_now()}] {msg}', flush=True)


def latest_pbstream(maps_dir):
    """maps/ 에서 가장 번호가 큰 kau_vN.pbstream."""
    best, best_n = None, -1
    for path in maps_dir.glob('kau_v*.pbstream'):
        m = re.fullmatch(r'kau_v(\d+)\.pbstream', path.name)
        if m and int(m.group(1)) > best_n:
            best, best_n = path, int(m.group(1))
    return best


def parse_triplet(text, what):
    try:
        parts = [float(v) for v in text.replace(' ', '').split(',')]
    except ValueError:
        parts = []
    if len(parts) != 3:
        sys.exit(f'{what} 는 "x,y,yaw" 형식이어야 한다: {text!r}')
    return parts[0], parts[1], parts[2]


class SimApi:
    """sim_api (127.0.0.1:9003) 조회. 실패하면 None 을 준다 — 감시는 계속 돈다."""

    def __init__(self, base):
        self.base = base.rstrip('/')

    def get(self, path, timeout=3.0):
        try:
            with urllib.request.urlopen(self.base + path, timeout=timeout) as res:
                return json.load(res)
        except Exception:
            return None

    def pose(self):
        """차량의 gz 월드 좌표 {x, y, z, yaw}."""
        return self.get('/pose')

    def running(self):
        st = self.get('/status')
        return bool(st and st.get('running') and not st.get('switching'))


def tail_log(path, out_queue, stop_event):
    """sim_api 로그를 tail 해서 이벤트를 큐에 넣는다. 로테이트/트렁케이트 대응."""
    handle, inode = None, None
    while not stop_event.is_set():
        try:
            if handle is None:
                handle = open(path, 'r', errors='replace')
                handle.seek(0, os.SEEK_END)
                inode = os.fstat(handle.fileno()).st_ino
            line = handle.readline()
            if line:
                if PAT_RESET.search(line):
                    out_queue.put(EV_RESET)
                elif PAT_SIM_STARTING.search(line):
                    out_queue.put(EV_SIM_STARTING)
                elif PAT_SIM_READY.search(line):
                    out_queue.put(EV_SIM_READY)
                continue
            # EOF. supervisord 가 1MB 에서 잘라내므로 크기가 줄면 다시 연다.
            try:
                st = os.stat(path)
                if st.st_ino != inode or st.st_size < handle.tell():
                    handle.close()
                    handle = None
                    continue
            except OSError:
                handle.close()
                handle = None
            stop_event.wait(0.25)
        except Exception as exc:            # 로그 문제로 감시가 죽으면 안 된다
            say(f'로그 tail 오류 ({exc}). 2초 뒤 재시도.')
            if handle is not None:
                handle.close()
                handle = None
            stop_event.wait(2.0)
    if handle is not None:
        handle.close()


class ResetWatcher(Node):
    def __init__(self, args):
        # 벽시계로 돈다. sim 시계가 되감겨도 이 노드의 판단은 흔들리지 않는다.
        super().__init__('kau_reset_watcher',
                         parameter_overrides=[
                             rclpy.parameter.Parameter('use_sim_time', value=False)])
        self.args = args
        self.sim = SimApi(args.sim_api)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self, spin_thread=False)

        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', QoSProfile(depth=1))
        self.create_subscription(Clock, '/clock', self._on_clock, 10)

        self.sim_time = None            # 마지막으로 본 /clock [s]
        self.sim_time_wall = 0.0        # 그걸 본 벽시계 시각

        self.proc = None                # 우리가 띄운 launch 프로세스 그룹
        self.owns_launch = False        # 이미 떠 있던 걸 물려받았으면 False
        self.offset = None              # (x, y, yaw): map <- gz world
        self.csv = None
        self.stop_event = threading.Event()
        self.events = queue.Queue()
        self.last_event_at = {}

    # ────────────────────────── ROS 입출력 ──────────────────────────

    def _on_clock(self, msg):
        now = msg.clock.sec + msg.clock.nanosec * 1e-9
        prev = self.sim_time
        self.sim_time = now
        self.sim_time_wall = time.monotonic()
        # 되감김 감지 — 재시작 로그를 놓쳤더라도 TF 버퍼는 반드시 비운다.
        if prev is not None and now < prev - 1.0:
            self.clear_tf()
            say(f'sim 시계 되감김 감지 ({prev:.1f}s -> {now:.1f}s). TF 버퍼를 비웠다.')

    def clear_tf(self):
        try:
            self.tf_buffer.clear()
        except AttributeError:          # 구버전 tf2_ros 대비
            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(
                self.tf_buffer, self, spin_thread=False)

    def map_pose(self):
        """map -> base_footprint 를 latest 로 조회. stamp 에 의존하지 않는다."""
        try:
            tr = self.tf_buffer.lookup_transform(
                MAP_FRAME, BASE_FRAME, rclpy.time.Time()).transform
        except Exception:
            return None
        q = tr.rotation
        return (tr.translation.x, tr.translation.y, yaw_of(q.x, q.y, q.z, q.w))

    def publish_initialpose(self, x, y, yaw):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = MAP_FRAME
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        cov = [0.0] * 36
        cov[0] = cov[7] = 0.25          # RViz 2D Pose Estimate 기본값과 동일
        cov[35] = 0.06853891945200942
        msg.pose.covariance = cov
        self.pose_pub.publish(msg)

    # ────────────────────────── 좌표 변환 ──────────────────────────

    def world_to_map(self, world):
        """gz 월드 좌표 -> map 프레임."""
        ox, oy, oyaw = self.offset
        wx, wy, wyaw = world['x'], world['y'], world['yaw']
        c, s = math.cos(oyaw), math.sin(oyaw)
        return (ox + c * wx - s * wy, oy + s * wx + c * wy, wrap(oyaw + wyaw))

    def load_offset(self):
        if self.args.spawn_pose:
            return None                 # 스폰 pose 를 직접 받았으면 변환이 필요 없다
        if self.args.offset:
            off = parse_triplet(self.args.offset, '--offset')
            say(f'오프셋을 인자로 받았다: x={off[0]:.3f} y={off[1]:.3f} '
                f'yaw={math.degrees(off[2]):.1f}deg')
            return off
        if self.args.recalibrate or not OFFSET_FILE.exists():
            return None
        try:
            saved = json.loads(OFFSET_FILE.read_text())
            off = (float(saved['x']), float(saved['y']), float(saved['yaw']))
        except Exception as exc:
            say(f'저장된 오프셋을 읽지 못했다 ({exc}). 새로 잡는다.')
            return None
        if saved.get('pbstream') and saved['pbstream'] != str(self.args.pbstream):
            say(f"저장된 오프셋은 다른 지도({saved['pbstream']}) 기준이다. 새로 잡는다.")
            return None
        say(f'저장된 오프셋 사용: x={off[0]:.3f} y={off[1]:.3f} '
            f'yaw={math.degrees(off[2]):.1f}deg  ({OFFSET_FILE})')
        return off

    def save_offset(self, off):
        OFFSET_FILE.parent.mkdir(parents=True, exist_ok=True)
        OFFSET_FILE.write_text(json.dumps({
            'x': off[0], 'y': off[1], 'yaw': off[2],
            'pbstream': str(self.args.pbstream),
            'saved_at': time.strftime('%Y-%m-%d %H:%M:%S'),
        }, indent=2))
        say(f'오프셋을 저장했다: {OFFSET_FILE}')

    def calibrate(self):
        """map <- gz world 고정 변환을 지금 상태에서 역산한다.

        localization 이 수렴해 있어야 의미가 있다. 그래서 사람이 확인하고
        Enter 를 치게 한다 — TF 는 수렴 전에도 나오기 때문에 자동으로 하면
        틀린 오프셋을 그대로 굳혀버린다.
        """
        print()
        print('─' * 70)
        print(' 좌표 보정 (처음 한 번만)')
        print()
        print(' RViz 에서 스캔이 지도에 제대로 겹쳤는지 확인한다.')
        print(' 안 맞으면 2D Pose Estimate 로 맞춘 뒤 진행할 것.')
        print(' 차량은 세워둔 상태여야 한다.')
        print('─' * 70)
        try:
            input(' 수렴됐으면 Enter: ')
        except EOFError:
            say('대화형 입력이 없다. --offset 또는 --spawn-pose 로 직접 줘야 한다.')
            return None

        samples = []
        for _ in range(self.args.calib_samples):
            world = self.sim.pose()
            here = self.map_pose()
            if world and here:
                samples.append((here, world))
            time.sleep(0.3)

        if len(samples) < 3:
            say('보정 실패: TF 또는 sim_api pose 를 못 읽었다.')
            return None

        xs = [w['x'] for _, w in samples]
        ys = [w['y'] for _, w in samples]
        if max(xs) - min(xs) > 0.02 or max(ys) - min(ys) > 0.02:
            say('보정 실패: 보정 중 차량이 움직였다. 세우고 다시 실행할 것.')
            return None

        # yaw 는 원형 평균으로, 평행이동은 그 yaw 로 회전시킨 뒤 평균낸다.
        sin_sum = sum(math.sin(m[2] - w['yaw']) for m, w in samples)
        cos_sum = sum(math.cos(m[2] - w['yaw']) for m, w in samples)
        oyaw = math.atan2(sin_sum, cos_sum)
        c, s = math.cos(oyaw), math.sin(oyaw)
        ox = sum(m[0] - (c * w['x'] - s * w['y']) for m, w in samples) / len(samples)
        oy = sum(m[1] - (s * w['x'] + c * w['y']) for m, w in samples) / len(samples)

        off = (ox, oy, oyaw)
        say(f'보정 완료: x={ox:.3f} y={oy:.3f} yaw={math.degrees(oyaw):.1f}deg '
            f'({len(samples)} 샘플)')
        self.save_offset(off)
        return off

    def spawn_in_map(self):
        """리셋 직후 차량이 서 있는 위치를 map 프레임으로."""
        if self.args.spawn_pose:
            x, y, yaw = parse_triplet(self.args.spawn_pose, '--spawn-pose')
            return (x, y, yaw)
        world = self.settled_world_pose()
        if world is None:
            say('sim_api 에서 차량 pose 를 못 읽었다. /initialpose 를 건너뛴다.')
            return None
        if self.offset is None:
            say('오프셋이 없어서 map 좌표를 못 만든다. --recalibrate 로 잡을 것.')
            return None
        return self.world_to_map(world)

    def settled_world_pose(self, timeout=6.0):
        """pose 가 더 안 움직일 때까지 기다렸다가 읽는다 (순간이동 직후용)."""
        deadline = time.monotonic() + timeout
        prev = None
        while time.monotonic() < deadline:
            now = self.sim.pose()
            if now and prev and abs(now['x'] - prev['x']) < 0.005 \
                    and abs(now['y'] - prev['y']) < 0.005:
                return now
            prev = now
            time.sleep(0.3)
        return prev

    # ────────────────────────── launch 관리 ──────────────────────────

    def cartographer_running(self):
        """우리가 띄웠든 남이 띄웠든 cartographer 가 살아있는지."""
        return subprocess.run(['pgrep', '-f', 'cartographer_node'],
                              stdout=subprocess.DEVNULL,
                              check=False).returncode == 0

    def localization_alive(self):
        if self.proc is not None and self.proc.poll() is None:
            return True
        # 이미 떠 있던 걸 물려받았을 수 있다. 이걸 안 보면 리셋 때 두 번째
        # 인스턴스를 띄워서 노드 이름이 충돌한다.
        return self.cartographer_running()

    def start_localization(self):
        if self.localization_alive():
            return
        cmd = [
            'ros2', 'launch', 'kau_localization', 'cartographer_localization.launch.py',
            f'pbstream:={self.args.pbstream}',
            'use_sim_time:=true',
            f"rviz:={'false' if self.args.no_rviz else 'true'}",
        ] + list(self.args.launch_arg)
        say('localization 기동: ' + ' '.join(cmd[2:]))
        # 자체 세션으로 띄워야 프로세스 그룹째 정리할 수 있다.
        self.proc = subprocess.Popen(cmd, start_new_session=True)
        self.owns_launch = True

    def stop_localization(self):
        if not self.localization_alive():
            self.proc = None
            return
        say('localization 종료 중...')
        if self.proc is not None and self.proc.poll() is None:
            pgid = os.getpgid(self.proc.pid)
            for sig, wait_s in ((signal.SIGINT, 10.0), (signal.SIGTERM, 5.0),
                                (signal.SIGKILL, 3.0)):
                try:
                    os.killpg(pgid, sig)
                except ProcessLookupError:
                    break
                try:
                    self.proc.wait(timeout=wait_s)
                    break
                except subprocess.TimeoutExpired:
                    continue
        self.proc = None
        # launch 가 죽어도 노드가 남으면 다음 기동이 이름 충돌로 깨진다.
        for name in ('cartographer_node', 'cartographer_occupancy_grid_node',
                     'initial_pose_relay'):
            subprocess.run(['pkill', '-f', name], check=False,
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        self.clear_tf()

    def wait_for_relay(self, timeout=60.0):
        """initial_pose_relay 가 /initialpose 를 구독할 때까지."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and not self.stop_event.is_set():
            if self.count_subscribers('/initialpose') > 0:
                return True
            time.sleep(0.5)
        return False

    def wait_for_sim(self, timeout=180.0):
        """gz 가 돌고 시계가 앞으로 흐르고 센서 토픽이 살아난 상태."""
        deadline = time.monotonic() + timeout
        first_seen = None
        while time.monotonic() < deadline and not self.stop_event.is_set():
            ok_sim = self.sim.running()
            fresh_clock = (self.sim_time is not None
                           and time.monotonic() - self.sim_time_wall < 2.0)
            ok_topics = (self.count_publishers('/scan') > 0
                         and self.count_publishers('/odom') > 0)
            if ok_sim and fresh_clock and ok_topics:
                # 시계가 실제로 "앞으로" 가는지 2초 지켜본다.
                if first_seen is None:
                    first_seen = (self.sim_time, time.monotonic())
                elif time.monotonic() - first_seen[1] >= 2.0:
                    if self.sim_time > first_seen[0]:
                        return True
                    first_seen = None
            else:
                first_seen = None
            time.sleep(0.5)
        return False

    # ────────────────────────── 이벤트 처리 ──────────────────────────

    def on_reset(self):
        say('리셋 감지.')
        self.log_csv(event='reset')
        time.sleep(self.args.settle)

        if self.args.relaunch_on_reset:
            self.stop_localization()
            self.start_localization()
            self.inject_after_start()
            return

        if not self.localization_alive():
            say('localization 이 떠 있지 않다. 새로 띄운다.')
            self.start_localization()
            self.inject_after_start()
            return

        target = self.spawn_in_map()
        if target is None:
            return
        if not self.wait_for_relay(timeout=15.0):
            say('initial_pose_relay 가 안 보인다. /initialpose 를 건너뛴다.')
            return
        self.publish_initialpose(*target)
        say(f'/initialpose 주입: x={target[0]:.3f} y={target[1]:.3f} '
            f'yaw={math.degrees(target[2]):.1f}deg  (cartographer · RViz 는 유지)')
        self.log_csv(event='initialpose')

    def on_sim_starting(self):
        say('재시작 감지. sim 시계가 되감기므로 localization 을 먼저 내린다.')
        self.log_csv(event='respawn_start')
        self.stop_localization()

    def on_sim_ready(self):
        if self.localization_alive():
            return                      # 부팅 직후 등 — 이미 정상이면 둘 것
        say('sim 준비 완료 신호. 시계와 센서 토픽을 확인한다.')
        if not self.wait_for_sim():
            say('sim 이 제때 안 올라왔다. 다음 이벤트를 기다린다.')
            return
        self.start_localization()
        self.inject_after_start()
        self.log_csv(event='respawn_done')

    def inject_after_start(self):
        """새로 띄운 cartographer 에 스폰 pose 를 넣어 바로 수렴시킨다."""
        if self.offset is None and not self.args.spawn_pose:
            return
        if not self.wait_for_relay():
            say('initial_pose_relay 가 안 떴다. /initialpose 를 건너뛴다.')
            return
        # 서비스 3개(get/finish/start_trajectory)가 붙을 여유를 준다.
        time.sleep(self.args.inject_delay)
        target = self.spawn_in_map()
        if target is None:
            return
        self.publish_initialpose(*target)
        say(f'/initialpose 주입: x={target[0]:.3f} y={target[1]:.3f} '
            f'yaw={math.degrees(target[2]):.1f}deg')

    # ────────────────────────── 드리프트 기록 ──────────────────────────

    def open_csv(self):
        if self.args.no_monitor:
            return
        path = Path(self.args.csv) if self.args.csv else (
            Path.home() / 'physicar_logs' /
            time.strftime('localization_drift_%Y%m%d_%H%M%S.csv'))
        path.parent.mkdir(parents=True, exist_ok=True)
        self.csv = path.open('a', buffering=1)
        self.csv.write('wall,sim_time,map_x,map_y,map_yaw_deg,'
                       'truth_x,truth_y,truth_yaw_deg,err_xy_m,err_yaw_deg,event\n')
        say(f'드리프트 기록: {path}')

    def log_csv(self, row=None, event=''):
        if self.csv is None:
            return
        sim_t = '' if self.sim_time is None else f'{self.sim_time:.3f}'
        cells = row if row else [''] * 8
        self.csv.write(','.join(
            [time.strftime('%Y-%m-%d %H:%M:%S'), sim_t] +
            [str(c) for c in cells] + [event]) + '\n')

    def monitor_loop(self):
        """실측(gz) 대비 추정(map) 오차를 1Hz 로 남긴다. 장기 드리프트 확인용."""
        last_print = 0.0
        while not self.stop_event.is_set():
            time.sleep(1.0)
            if self.offset is None or not self.localization_alive():
                continue
            here = self.map_pose()
            world = self.sim.pose()
            if not here or not world:
                continue
            ex, ey, eyaw = self.world_to_map(world)
            err_xy = math.hypot(here[0] - ex, here[1] - ey)
            err_yaw = math.degrees(wrap(here[2] - eyaw))
            self.log_csv([f'{here[0]:.3f}', f'{here[1]:.3f}',
                          f'{math.degrees(here[2]):.2f}',
                          f'{ex:.3f}', f'{ey:.3f}', f'{math.degrees(eyaw):.2f}',
                          f'{err_xy:.3f}', f'{err_yaw:.2f}'])
            if time.monotonic() - last_print >= self.args.report_period:
                last_print = time.monotonic()
                say(f'드리프트: {err_xy * 100:.1f}cm / {err_yaw:+.1f}deg')

    # ────────────────────────── 메인 루프 ──────────────────────────

    def startup(self):
        if self.cartographer_running():
            say('cartographer 가 이미 떠 있다. 그대로 물려받는다 '
                '(재시작 때는 이 스크립트가 다시 띄운다).')
            self.owns_launch = False
        else:
            self.start_localization()

        self.offset = self.load_offset()
        if self.offset is None and not self.args.spawn_pose:
            deadline = time.monotonic() + 60.0
            while self.map_pose() is None and time.monotonic() < deadline \
                    and not self.stop_event.is_set():
                time.sleep(0.5)
            if self.map_pose() is None:
                say('map -> base_footprint TF 가 안 나온다. 보정을 건너뛴다 '
                    '(--offset 으로 줄 수 있다).')
            else:
                self.offset = self.calibrate()
        say('감시 시작. 앱에서 리셋 / 재시작을 누르면 알아서 처리한다. (Ctrl-C 종료)')

    def worker(self):
        self.startup()
        while not self.stop_event.is_set():
            try:
                event = self.events.get(timeout=0.5)
            except queue.Empty:
                continue
            # 버튼 연타 / 같은 이벤트 중복은 흘려보낸다.
            now = time.monotonic()
            if now - self.last_event_at.get(event, 0.0) < 2.0:
                continue
            self.last_event_at[event] = now
            try:
                if event == EV_RESET:
                    self.on_reset()
                elif event == EV_SIM_STARTING:
                    self.on_sim_starting()
                elif event == EV_SIM_READY:
                    self.on_sim_ready()
            except Exception as exc:
                say(f'이벤트 처리 중 오류 ({event}): {exc}')
            self.last_event_at[event] = time.monotonic()

    def shutdown(self):
        self.stop_event.set()
        if self.owns_launch:
            self.stop_localization()
        if self.csv is not None:
            self.csv.close()


def main():
    maps_dir = PKG_DIR / 'maps'
    default_pbstream = latest_pbstream(maps_dir)

    parser = argparse.ArgumentParser(
        description='PhysiCar 리셋/재시작 버튼을 감지해 localization 을 이어붙인다.',
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--pbstream', default=str(default_pbstream) if default_pbstream else None,
                        help='cartographer_localization.launch.py 에 넘길 지도 '
                             f'(기본: {default_pbstream})')
    parser.add_argument('--sim-log', default=DEFAULT_SIM_LOG, help='sim_api 로그 경로')
    parser.add_argument('--sim-api', default=DEFAULT_SIM_API, help='sim_api 주소')
    parser.add_argument('--no-rviz', action='store_true', help='RViz 를 띄우지 않는다')
    parser.add_argument('--relaunch-on-reset', action='store_true',
                        help='리셋에도 /initialpose 대신 통째로 재런치한다')
    parser.add_argument('--settle', type=float, default=1.5,
                        help='리셋 뒤 pose 가 안정될 때까지 기다릴 시간 [s]')
    parser.add_argument('--inject-delay', type=float, default=3.0,
                        help='재런치 뒤 /initialpose 를 쏘기까지 기다릴 시간 [s]')
    parser.add_argument('--offset', help='map <- gz world 오프셋을 "x,y,yaw" 로 직접 지정')
    parser.add_argument('--spawn-pose',
                        help='map 프레임 스폰 pose 를 "x,y,yaw" 로 직접 지정 (보정 생략)')
    parser.add_argument('--recalibrate', action='store_true', help='저장된 오프셋을 무시하고 다시 잡는다')
    parser.add_argument('--calib-samples', type=int, default=10, help='보정 샘플 수')
    parser.add_argument('--no-monitor', action='store_true', help='드리프트 CSV 기록을 끈다')
    parser.add_argument('--csv', help='드리프트 CSV 경로')
    parser.add_argument('--report-period', type=float, default=10.0,
                        help='드리프트를 콘솔에 찍는 주기 [s]')
    parser.add_argument('--launch-arg', action='append', default=[],
                        help='cartographer_localization.launch.py 에 추가로 넘길 인자 (여러 번 가능)')
    args = parser.parse_args()

    if not args.pbstream:
        sys.exit(f'{maps_dir} 에 kau_vN.pbstream 이 없다. --pbstream 으로 지정할 것.')
    if not Path(args.pbstream).is_file():
        sys.exit(f'지도를 찾을 수 없다: {args.pbstream}')
    if not Path(args.sim_log).exists():
        sys.exit(f'sim_api 로그가 없다: {args.sim_log}  (sim 이 떠 있는지 확인할 것)')

    rclpy.init()
    node = ResetWatcher(args)
    node.open_csv()

    tail = threading.Thread(target=tail_log,
                            args=(args.sim_log, node.events, node.stop_event),
                            daemon=True)
    worker = threading.Thread(target=node.worker, daemon=True)
    monitor = threading.Thread(target=node.monitor_loop, daemon=True)
    for thread in (tail, worker, monitor):
        thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print()
        say('종료 요청.')
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
