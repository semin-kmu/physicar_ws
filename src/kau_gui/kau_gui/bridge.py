"""구독 전담 rclpy 노드. Qt 를 전혀 모른다.

**관측 전용.** publisher 도 service client 도 만들지 않는다.
여기에 발행 코드가 생기면 kau_state_machine/docs/09 계약 104·128 위반이다.

수신을 두 계열로 나눈다. 이 구분이 설계의 핵심이다.

    latch  맵 계열 (스캔 · 경로 · 장애물 · TF). 최신 한 장만 그리므로 덮어쓴다.
    ring   플롯 계열. 콜백에서 **전량** 쌓는다. 렌더가 10 Hz 라고 50 Hz 신호를
           10 Hz 로 샘플링하면 조향 떨림 · cte 스파이크처럼 정작 봐야 할
           순간 변화가 화면에서 사라진다.

스레드: rclpy spin 은 별도 스레드, Qt 메인 스레드는 snapshot() 만 부른다.
"""

from __future__ import annotations

import math
import threading
from collections import deque

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.node import Node
from rosidl_runtime_py.utilities import get_message
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from kau_msgs.msg import SteerDebug

CM_PER_M = 100.0
RAD2DEG = 180.0 / math.pi


def observe_qos():
    """관측 전용 구독. RELIABLE 발행자와도 호환되고 재전송 비용을
    발행측에 지우지 않는다 (docs/09 section 10-1)."""
    return QoSProfile(depth=1,
                      history=HistoryPolicy.KEEP_LAST,
                      reliability=ReliabilityPolicy.BEST_EFFORT,
                      durability=DurabilityPolicy.VOLATILE)


def latched_qos():
    """latched(TRANSIENT_LOCAL) 발행자를 받으려면 구독도 맞춰야 한다."""
    return QoSProfile(depth=1,
                      history=HistoryPolicy.KEEP_LAST,
                      reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL)


class Series:
    """시계열 링버퍼. 시간축은 GUI 노드 clock 기준 초.

    메시지 stamp 를 쓰지 않는 이유: 발행 노드마다 sim/system clock 이
    섞일 수 있어 축이 어긋난다.
    """

    def __init__(self, window: float):
        self.window = window
        self.t = deque()
        self.v = deque()

    def push(self, t: float, v: float):
        self.t.append(t)
        self.v.append(v)
        while self.t and (t - self.t[0]) > self.window:
            self.t.popleft()
            self.v.popleft()

    def arrays(self):
        return np.fromiter(self.t, float), np.fromiter(self.v, float)

    @property
    def last(self):
        return self.v[-1] if self.v else None

    @property
    def stamp(self):
        return self.t[-1] if self.t else None


class RateMeter:
    """토픽 수신 주기 실측. 노드가 '제 주기로 도는가' 를 본다.

    /diag/heartbeat 이 아직 없으므로 이것이 유일한 근거다.
    """

    N = 24

    def __init__(self):
        self._t = deque(maxlen=self.N)

    def mark(self, t):
        self._t.append(t)

    @property
    def last(self):
        return self._t[-1] if self._t else None

    @property
    def hz(self):
        # 표본 2 개 미만이면 '모름'. 0 Hz 와 구분해야 한다.
        if len(self._t) < 2:
            return None
        span = self._t[-1] - self._t[0]
        return (len(self._t) - 1) / span if span > 1e-6 else None


class Latest:
    """최신 한 장만 유지하는 표시물."""

    __slots__ = ("value", "stamp")

    def __init__(self):
        self.value = None
        self.stamp = None

    def set(self, v, t):
        self.value = v
        self.stamp = t

    def age(self, now):
        return None if self.stamp is None else (now - self.stamp)


class Bridge(Node):

    def __init__(self):
        super().__init__("kau_gui")

        self._lock = threading.Lock()

        self._declare()

        w = self.history_s
        self.speed_target = Series(w)
        self.speed_real = Series(w)
        self.steer_raw = Series(w)
        self.steer_cmd = Series(w)
        self.lookahead = Series(w)
        self.heading_err = Series(w)
        self.cross_track = Series(w)

        self.scan = Latest()
        self.path_global = Latest()
        self.path_local = Latest()
        self.path_lane = Latest()
        self.obstacles = Latest()
        self.grid = Latest()

        self.tf_poses = [None, None, None]      # map / odom / base_link
        self.tf_ages = [None, None, None]

        self.tracking_ok = False
        self.steer_debug_alive = False

        self.nodes = []
        self._rates = {}

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self._make_display_subs()
        self._make_watch_subs()

        self.create_timer(0.05, self._on_tf)          # 20 Hz
        self.create_timer(1.0, self._on_graph)        # 1 Hz

        self.get_logger().info(
            f"[kau_gui] 관측 전용 기동. render={self.render_hz:g} Hz "
            f"history={self.history_s:g} s 감시노드={len(self.watch_names)}")

    # ------------------------------------------------------------------
    # 파라미터
    # ------------------------------------------------------------------

    def _declare(self):
        d = self.declare_parameter

        self.render_hz = d("render_hz", 10.0).value
        self.history_s = d("history_s", 30.0).value

        self.map_frame = d("frames.map", "map").value
        self.odom_frame = d("frames.odom", "odom").value
        self.base_frame = d("frames.base", "base_link").value
        self.tf_timeout = d("frames.tf_timeout_s", 0.5).value

        self.map_source = d("map.source", "auto").value
        self.map_topic = d("map.topic", "/map").value
        self.map_yaml = d("map.yaml_path", "").value
        self.map_wait = d("map.wait_s", 5.0).value

        self.topics = {
            k: d(f"topics.{k}", v).value for k, v in (
                ("scan", "/scan_filtered"),
                ("obstacles", "/perception/obstacle_markers"),
                ("path_global", "/viz/path/global"),
                ("path_local", "/viz/path/local"),
                ("path_lane", "/viz/path/lane"),
                ("speed_cmd", "/speed"),
                ("odom", "/odom"),
                ("steering", "/steering"),
                ("steer_debug", "/debug/steer"))}

        self.tf_axis_cm = d("tf_axis_cm", 20.0).value

        self.stale_ratio = d("watch.stale_ratio", 3.0).value
        self.min_stale_s = d("watch.min_stale_s", 0.3).value
        self.watch_names = list(d("watch.names", []).value or [])
        self.watch_topics = list(d("watch.topics", []).value or [])
        self.watch_types = list(d("watch.types", []).value or [])
        self.watch_rates = list(d("watch.rates", []).value or [])

        n = len(self.watch_names)
        if not (len(self.watch_topics) == len(self.watch_types)
                == len(self.watch_rates) == n):
            # 길이가 어긋나면 엉뚱한 토픽으로 등급을 매긴다. 조용히 넘어가지 않는다.
            raise RuntimeError(
                "watch.names / topics / types / rates 의 길이가 서로 다르다")

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # ------------------------------------------------------------------
    # 구독
    # ------------------------------------------------------------------

    def _make_display_subs(self):
        t = self.topics
        self.create_subscription(
            LaserScan, t["scan"], self._on_scan, observe_qos())
        self.create_subscription(
            MarkerArray, t["obstacles"], self._on_markers, observe_qos())

        # 발행측이 latched 다. 맞춰야 GUI 를 나중에 띄워도 전역 경로가 온다.
        self.create_subscription(
            Path, t["path_global"],
            lambda m: self._on_path(m, self.path_global), latched_qos())
        self.create_subscription(
            Path, t["path_local"],
            lambda m: self._on_path(m, self.path_local), observe_qos())
        self.create_subscription(
            Path, t["path_lane"],
            lambda m: self._on_path(m, self.path_lane), observe_qos())

        self.create_subscription(
            Odometry, t["odom"], self._on_odom, observe_qos())
        self.create_subscription(
            Float64, t["speed_cmd"], self._on_speed, observe_qos())
        self.create_subscription(
            Float64, t["steering"], self._on_steering, observe_qos())
        self.create_subscription(
            SteerDebug, t["steer_debug"], self._on_debug, observe_qos())

        if self.map_source in ("topic", "auto"):
            self.create_subscription(
                OccupancyGrid, self.map_topic, self._on_grid, latched_qos())

    def _make_watch_subs(self):
        """감시 토픽마다 raw 구독 하나. 역직렬화하지 않고 수신 시각만 찍는다.

        주기 판정을 하지 않는 노드(rate <= 0)는 구독을 만들지 않는다.
        graph 존재 여부만으로 판정하기 때문이다.
        """
        for name, topic, typ, rate in zip(
                self.watch_names, self.watch_topics,
                self.watch_types, self.watch_rates):
            if not topic or not typ or rate <= 0.0:
                continue
            if topic in self._rates:
                continue
            self._rates[topic] = RateMeter()
            try:
                # rclpy 는 타입 문자열을 못 받는다. 클래스로 풀어 준다.
                self.create_subscription(
                    get_message(typ), topic,
                    (lambda _m, tp=topic: self._rates[tp].mark(self._now())),
                    observe_qos(), raw=True)
            except Exception as e:       # 타입 이름 오타 등
                # 감시 하나가 실패했다고 GUI 전체를 죽이지 않는다.
                # 해당 노드는 STALE 로 남아 눈에 띈다.
                self.get_logger().error(
                    f"[kau_gui] 감시 구독 실패 {topic} ({typ}): {e}")

    # ------------------------------------------------------------------
    # 표시용 콜백
    # ------------------------------------------------------------------

    def _on_scan(self, m: LaserScan):
        # 스캔은 센서 프레임이다. map 으로 옮겨야 배경 지도 위에 얹힌다.
        tf = self._lookup(self.map_frame, m.header.frame_id)
        if tf is None:
            # 측위가 없으면 절대좌표를 만들 수 없다. 갱신하지 않고 늙게 둔다.
            return
        tx, ty, yaw = tf

        r = np.asarray(m.ranges, dtype=float)
        ok = np.isfinite(r) & (r >= m.range_min) & (r <= m.range_max)
        if not ok.any():
            return
        idx = np.nonzero(ok)[0]
        a = m.angle_min + m.angle_increment * idx
        lx, ly = r[idx] * np.cos(a), r[idx] * np.sin(a)
        c, s = math.cos(yaw), math.sin(yaw)
        xs = (tx + c * lx - s * ly) * CM_PER_M
        ys = (ty + s * lx + c * ly) * CM_PER_M

        with self._lock:
            self.scan.set((xs, ys), self._now())

    def _on_markers(self, m: MarkerArray):
        out = []
        for mk in m.markers:
            # 발행측이 사라진 장애물을 DELETE 로 지운다. 그릴 것이 없다.
            if mk.action != Marker.ADD:
                continue
            # obstacle_markers 는 원판을 CYLINDER 로 낸다. scale.x = 지름.
            out.append((mk.pose.position.x * CM_PER_M,
                        mk.pose.position.y * CM_PER_M,
                        mk.scale.x * 0.5 * CM_PER_M))
        with self._lock:
            self.obstacles.set(out, self._now())

    def _on_path(self, m: Path, dst: Latest):
        if not m.poses:
            with self._lock:
                dst.set((np.empty(0), np.empty(0)), self._now())
            return
        xs = np.fromiter((p.pose.position.x for p in m.poses), float)
        ys = np.fromiter((p.pose.position.y for p in m.poses), float)
        with self._lock:
            dst.set((xs * CM_PER_M, ys * CM_PER_M), self._now())

    def _on_grid(self, m: OccupancyGrid):
        w, h = m.info.width, m.info.height
        if w <= 0 or h <= 0:
            return
        g = np.asarray(m.data, dtype=np.int16).reshape(h, w)
        with self._lock:
            self.grid.set(
                (g, m.info.resolution,
                 m.info.origin.position.x, m.info.origin.position.y),
                self._now())

    def _on_odom(self, m: Odometry):
        with self._lock:
            self.speed_real.push(self._now(), m.twist.twist.linear.x)

    def _on_speed(self, m: Float64):
        with self._lock:
            self.speed_target.push(self._now(), m.data)

    def _on_steering(self, m: Float64):
        # /steering 은 rad. 플롯은 deg 로 통일한다. max_steer_deg(20) 와
        # 눈으로 바로 대조하려면 deg 여야 한다.
        with self._lock:
            self.steer_cmd.push(self._now(), m.data * RAD2DEG)

    def _on_debug(self, m: SteerDebug):
        t = self._now()
        with self._lock:
            self.steer_debug_alive = True
            self.tracking_ok = bool(m.tracking_ok)
            # 추종 실패 tick 의 0 을 쌓으면 플롯이 0 으로 끌려 내려가
            # 오차가 사라진 것처럼 보인다. 값을 넣지 않고 선을 끊는다.
            if not m.tracking_ok:
                return
            self.steer_raw.push(t, m.raw_steer_deg)
            self.lookahead.push(t, m.lookahead_cm)
            self.heading_err.push(t, m.heading_error_rad * RAD2DEG)
            self.cross_track.push(t, m.cross_track_cm)

    # ------------------------------------------------------------------
    # 주기 작업
    # ------------------------------------------------------------------

    def _lookup(self, parent: str, child: str):
        """(x_m, y_m, yaw) 또는 None."""
        try:
            tf = self.tf_buffer.lookup_transform(
                parent, child, rclpy.time.Time())
        except Exception:
            return None
        q = tf.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return (tf.transform.translation.x, tf.transform.translation.y, yaw)

    def _link_age(self, parent: str, child: str):
        try:
            tf = self.tf_buffer.lookup_transform(
                parent, child, rclpy.time.Time())
        except Exception:
            return None
        stamp = (tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9)
        return self._now() - stamp

    def _on_tf(self):
        """map 기준 세 프레임의 자세. map 은 항상 원점이다."""
        odom = self._lookup(self.map_frame, self.odom_frame)
        base = self._lookup(self.map_frame, self.base_frame)

        poses = [(0.0, 0.0, 0.0), None, None]
        if odom is not None:
            poses[1] = (odom[0] * CM_PER_M, odom[1] * CM_PER_M, odom[2])
        if base is not None:
            poses[2] = (base[0] * CM_PER_M, base[1] * CM_PER_M, base[2])

        # HUD 용. map->odom 은 AMCL, odom->base 는 EKF 소관이라 나눠 봐야
        # 어느 쪽이 끊겼는지 짚인다 (kau_localization README).
        ages = [0.0,
                self._link_age(self.map_frame, self.odom_frame),
                self._link_age(self.odom_frame, self.base_frame)]

        with self._lock:
            self.tf_poses = poses
            self.tf_ages = ages

    def _on_graph(self):
        # 노드 이름공간은 쓰지 않는 것이 팀 규약이라(docs/07 section 2)
        # 이름만 비교한다.
        alive = {n for n, _ns in self.get_node_names_and_namespaces()}
        t = self._now()

        out = []
        with self._lock:
            for name, topic, rate in zip(
                    self.watch_names, self.watch_topics, self.watch_rates):
                if name not in alive:
                    out.append((name, "absent", None))
                    continue
                if not topic or rate <= 0.0:
                    # 대표 토픽이 없거나(진단 로그 전용) latched 라 주기 판정
                    # 불가. 존재만으로 정상 처리하고 Hz 칸은 비운다.
                    out.append((name, "ok", None))
                    continue
                rm = self._rates.get(topic)
                if rm is None or rm.last is None:
                    out.append((name, "stale", None))
                    continue
                # 50 Hz 토픽은 3 배가 60 ms 라 무선 지터만으로도 깜빡인다.
                # 하한을 둔다.
                limit = max(self.stale_ratio / rate, self.min_stale_s)
                ok = (t - rm.last) <= limit
                out.append((name, "ok" if ok else "stale", rm.hz))
            self.nodes = out

    # ------------------------------------------------------------------
    # 스냅샷
    # ------------------------------------------------------------------

    def snapshot(self) -> dict:
        """Qt 메인 스레드가 렌더 직전에 한 번 부른다."""
        with self._lock:
            return {
                "now": self._now(),
                "scan": (self.scan.value, self.scan.stamp),
                "grid": (self.grid.value, self.grid.stamp),
                "path_global": (self.path_global.value,
                                self.path_global.stamp),
                "path_local": (self.path_local.value, self.path_local.stamp),
                "path_lane": (self.path_lane.value, self.path_lane.stamp),
                "obstacles": (self.obstacles.value, self.obstacles.stamp),
                "tf_poses": list(self.tf_poses),
                "tf_ages": list(self.tf_ages),
                "series": {
                    "speed_target": self.speed_target.arrays(),
                    "speed_real": self.speed_real.arrays(),
                    "steer_raw": self.steer_raw.arrays(),
                    "steer_cmd": self.steer_cmd.arrays(),
                    "lookahead": self.lookahead.arrays(),
                    "heading_err": self.heading_err.arrays(),
                    "cross_track": self.cross_track.arrays(),
                },
                "last": {
                    "speed_target": self.speed_target.last,
                    "speed_real": self.speed_real.last,
                    "steer_raw": self.steer_raw.last,
                    "steer_cmd": self.steer_cmd.last,
                    "lookahead": self.lookahead.last,
                    "heading_err": self.heading_err.last,
                    "cross_track": self.cross_track.last,
                },
                "tracking_ok": self.tracking_ok,
                "steer_debug_alive": self.steer_debug_alive,
                "nodes": list(self.nodes),
            }
