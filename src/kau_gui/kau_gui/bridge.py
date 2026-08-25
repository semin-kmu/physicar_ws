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
import pathlib

import numpy as np
import rclpy
from ament_index_python.packages import (PackageNotFoundError,
                                          get_package_share_directory)
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from kau_msgs.msg import SteerDebug

from . import manifest

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


class RateMeter:
    """도착 시각만 찍어 주기를 낸다. 메시지 내용은 보지 않는다."""

    N = 30

    __slots__ = ("_t",)

    def __init__(self):
        self._t = deque(maxlen=self.N)

    def mark(self, t):
        self._t.append(t)

    @property
    def last(self):
        return self._t[-1] if self._t else None

    @property
    def hz(self):
        # 표본 2 개 미만은 '모름'. 0 Hz 와 구분해야 한다.
        if len(self._t) < 2:
            return None
        span = self._t[-1] - self._t[0]
        return (len(self._t) - 1) / span if span > 1e-6 else None


class Bridge(Node):

    def __init__(self):
        super().__init__("kau_gui")

        self._lock = threading.Lock()

        # 플롯 시간축의 원점. 절대 시각을 그대로 쓰면 축이 못 읽는 값이 된다 --
        # 벽시계면 1.78e9 라 30 초 창을 구분하려고 소수점 8 자리가 붙고,
        # sim 시계면 sim 이 켜져 있던 시간이 그대로 나온다. 기동 시점을 0 으로
        # 두면 "몇 초째" 가 되어 그대로 읽힌다.
        #
        # None 인 동안은 아직 시계를 못 잡은 것이다. use_sim_time 이면 첫
        # /clock 이 오기 전까지 시각이 0 이라, 그때 원점을 잡으면 이후 값이
        # sim 절대시각이 되어 버린다. 0 이 아닌 첫 값에서 잡는다.
        self._t0 = None

        self._declare()

        w = self.history_s
        self.speed_target = Series(w)
        self.speed_real = Series(w)
        self.steer_cmd = Series(w)
        self.heading_err = Series(w)
        self.cross_track = Series(w)

        self.scan = Latest()
        self.path_global = Latest()
        self.path_local = Latest()
        self.path_lane = Latest()
        self.obstacles = Latest()

        self.tf_poses = [None, None, None]      # map / odom / base_link

        # 최상단 상태 띠. [(이름, ok|stale|absent, hz|None)]
        #
        # 첫 그래프 조회(1 초 뒤) 전까지도 목록은 보여야 한다. 비워 두면
        # 그 1 초 동안 "감시 대상 없음" 이 떠서 설정이 틀린 것처럼 보인다.
        self.nodes = [(n, "absent", None) for n in self.watch_names]
        self._rates = {}                        # topic -> RateMeter

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self._make_display_subs()
        self._make_status_subs()

        self.create_timer(0.05, self._on_tf)          # 20 Hz
        if self.watch_names:
            self.create_timer(1.0, self._on_graph)    # 1 Hz

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

        # 맵 배경. 기본은 kau_localization 이 share 에 설치하는 maps/ 에서
        # 이름으로 찾는다 (bringup.yaml 의 "@kau_localization:maps/..." 와
        # 같은 지도를 본다). yaml_path 를 주면 그쪽이 우선이다.
        self.map_pkg = d("map.package", "kau_localization").value
        self.map_name = d("map.name", "kau_v3").value
        self.map_yaml = self._resolve_map(d("map.yaml_path", "").value)

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

        # 점군 거리색의 양 끝 [m]. 이 밖은 끝 색으로 뭉갠다.
        # 고정값이라 차가 움직여도 같은 거리가 같은 색으로 남는다.
        self.scan_near_m = d("scan.near_m", 0.0).value
        self.scan_far_m = d("scan.far_m", 12.0).value
        self.scan_size = d("scan.point_size", 3.0).value

        self._declare_status()

    # ------------------------------------------------------------------
    # 노드 상태 (최상단 띠)
    # ------------------------------------------------------------------

    def _declare_status(self):
        """감시 대상은 bringup.yaml 이 정한다. GUI 는 목록을 갖지 않는다."""
        d = self.declare_parameter

        self.stale_ratio = d("status.stale_ratio", 3.0).value
        self.min_stale_s = d("status.min_stale_s", 0.3).value

        # 노드 -> (대표 topic, 타입, 기대 Hz). 없는 노드는 그래프 존재만으로
        # 판정하고 Hz 칸을 비운다.
        #
        # 기본값이 빈 배열이 아니라 [""] 인 이유: rclpy 는 기본값에서 타입을
        # 추론하는데 빈 배열은 BYTE_ARRAY 로 잡힌다 (all([]) 이 참이라 첫
        # 분기에 걸린다). 그러면 yaml 의 문자열 배열과 타입이 안 맞아
        # InvalidParameterTypeException 으로 GUI 가 기동하지 못한다.
        self.watch_spec = self._parse_watch(
            d("status.watch", [""]).value or [])

        self.watch_names = []
        if not d("status.enabled", True).value:
            return

        path = (d("status.bringup_yaml", "").value
                or manifest.default_path())
        if not path:
            self.get_logger().warn(
                "[kau_gui] bringup.yaml 을 못 찾았다. 상태 띠는 빈 채로 뜬다")
            return

        try:
            data = manifest.load(path)
            # manifest 가 쓰는 스위치마다 파라미터를 연다. 기본은 켬이라
            # 아무것도 안 주면 supervisor 기본 기동과 같은 목록이 나온다.
            options = {name: d(f"status.options.{name}", True).value
                       for name in manifest.option_names(data)}
            self.watch_names = manifest.watch_names(
                data, bool(self.get_parameter("use_sim_time").value), options)
        except Exception as e:      # 오탈자 · 스키마 변경 등
            # 상태 띠 하나 때문에 GUI 전체를 죽이지 않는다.
            self.get_logger().error(
                f"[kau_gui] bringup.yaml 해석 실패 ({path}): {e}")
            return

        unknown = sorted(set(self.watch_spec) - set(self.watch_names))
        if unknown:
            # 이름이 틀리면 그 노드는 영영 Hz 가 안 나온다. 조용히 두면
            # "왜 저것만 Hz 가 없지" 로 시간을 날린다.
            self.get_logger().warn(
                f"[kau_gui] status.watch 에 manifest 에 없는 노드: {unknown}")

    def _parse_watch(self, lines) -> dict:
        """"<노드> | <topic> | <타입> | <기대 Hz>" 목록 -> dict.

        ROS 파라미터 배열은 원소 타입이 같아야 해서 한 줄 문자열로 받는다.
        예전처럼 names/topics/types/rates 를 평행 배열로 두면 길이가
        어긋나는 순간 전체가 한 칸씩 밀린다.
        """
        out = {}
        for line in lines:
            if not str(line).strip():       # 기본값 [""] 의 빈 칸
                continue
            parts = [f.strip() for f in str(line).split("|")]
            if len(parts) != 4:
                self.get_logger().error(
                    f"[kau_gui] status.watch 형식 오류 (칸 4 개여야 한다): {line!r}")
                continue
            name, topic, typ, rate = parts
            try:
                out[name] = (topic, typ, float(rate))
            except ValueError:
                self.get_logger().error(
                    f"[kau_gui] status.watch 의 기대 Hz 가 수가 아니다: {line!r}")
        return out

    def _make_status_subs(self):
        """감시 토픽마다 raw 구독 하나. 역직렬화하지 않고 수신 시각만 찍는다."""
        for name in self.watch_names:
            spec = self.watch_spec.get(name)
            if spec is None:
                continue
            topic, typ, rate = spec
            if not topic or not typ or rate <= 0.0 or topic in self._rates:
                continue

            self._rates[topic] = RateMeter()
            try:
                # rclpy 는 타입 문자열을 못 받는다. 클래스로 풀어 준다.
                self.create_subscription(
                    get_message(typ), topic,
                    (lambda _m, tp=topic: self._rates[tp].mark(self._now())),
                    observe_qos(), raw=True)
            except Exception as e:      # 타입 이름 오타 등
                # 감시 하나가 실패했다고 GUI 를 죽이지 않는다. 해당 노드는
                # stale 로 남아 눈에 띈다.
                self.get_logger().error(
                    f"[kau_gui] 감시 구독 실패 {topic} ({typ}): {e}")

    def _on_graph(self):
        # 노드 이름공간은 쓰지 않는 것이 팀 규약이라(docs/07 section 2)
        # 이름만 비교한다.
        alive = {n for n, _ns in self.get_node_names_and_namespaces()}
        t = self._now()

        out = []
        for name in self.watch_names:
            if name not in alive:
                out.append((name, "absent", None))
                continue

            spec = self.watch_spec.get(name)
            if spec is None or not spec[0] or spec[2] <= 0.0:
                # 대표 토픽이 없거나(진단 로그 전용) latched 라 주기 판정
                # 불가. 존재만으로 정상 처리하고 Hz 칸은 비운다.
                out.append((name, "ok", None))
                continue

            meter = self._rates.get(spec[0])
            if meter is None or meter.last is None:
                out.append((name, "stale", None))
                continue

            # 50 Hz 토픽은 기대주기의 3 배가 60 ms 라 지터만으로도 깜빡인다.
            # 하한을 둔다.
            limit = max(self.stale_ratio / spec[2], self.min_stale_s)
            ok = (t - meter.last) <= limit
            out.append((name, "ok" if ok else "stale", meter.hz))

        with self._lock:
            self.nodes = out

    def _resolve_map(self, override: str) -> str:
        if override:
            return override
        if not self.map_pkg or not self.map_name:
            return ""
        try:
            share = get_package_share_directory(self.map_pkg)
        except PackageNotFoundError:
            self.get_logger().warn(
                f"[kau_gui] 패키지를 못 찾음: {self.map_pkg}. 맵 배경 없이 뜬다")
            return ""
        path = pathlib.Path(share) / "maps" / f"{self.map_name}.yaml"
        if not path.is_file():
            self.get_logger().warn(
                f"[kau_gui] 맵 yaml 없음: {path}. 맵 배경 없이 뜬다")
            return ""
        return str(path)

    def _now(self):
        """기동 시점 기준 경과 초. 표시·신선도 판정에 쓰는 유일한 시각이다.

        차이만 쓰는 곳(신선도·롤링 창)은 원점을 옮겨도 값이 안 변한다.
        """
        t = self.get_clock().now().nanoseconds * 1e-9
        if self._t0 is None:
            if t <= 0.0:
                return 0.0
            self._t0 = t
        return t - self._t0

    # ------------------------------------------------------------------
    # 구독
    # ------------------------------------------------------------------

    def _make_display_subs(self):
        t = self.topics
        self.create_subscription(
            LaserScan, t["scan"], self._on_scan, observe_qos())
        self.create_subscription(
            MarkerArray, t["obstacles"], self._on_markers, observe_qos())

        # latched 인 것은 /path/global (KauPath) 이고, 여기서 보는
        # /viz/path/global (nav_msgs/Path) 은 아니다. 발행측이 RViz 기본에
        # 맞춰 VOLATILE 로 낸다 (global_path_publisher.py 의 viz_pub 주석).
        # TRANSIENT_LOCAL 로 구독하면 durability 가 안 맞아 **한 건도 안 온다**
        # ("requesting incompatible QoS. No messages will be sent to it").
        # 나중에 띄우면 다음 재발행까지 비지만 rate 1 Hz 라 최대 1 초다.
        self.create_subscription(
            Path, t["path_global"],
            lambda m: self._on_path(m, self.path_global), observe_qos())
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
        rr = r[idx]
        lx, ly = rr * np.cos(a), rr * np.sin(a)
        c, s = math.cos(yaw), math.sin(yaw)
        xs = (tx + c * lx - s * ly) * CM_PER_M
        ys = (ty + s * lx + c * ly) * CM_PER_M

        # 거리는 변환 전 원본 range [m] 다. 색 기준이 화면 좌표가 아니라
        # 센서로부터의 실제 거리여야 차가 움직여도 색이 안 흔들린다.
        with self._lock:
            self.scan.set((xs, ys, rr), self._now())

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

    def _on_odom(self, m: Odometry):
        with self._lock:
            self.speed_real.push(self._now(), m.twist.twist.linear.x)

    def _on_speed(self, m: Float64):
        with self._lock:
            self.speed_target.push(self._now(), m.data)

    def _on_steering(self, m: Float64):
        # /steering 은 rad. 플롯은 deg 로 통일한다. max_steer_deg(20) 와
        # 눈으로 바로 대조하려면 deg 여야 한다.
        #
        # 이것이 조향 패널이 그리는 유일한 계열이다. 추종 실패 tick 에
        # steer_controller 가 내는 0 도 그대로 들어온다 -- 실제로 차량에
        # 나간 값이므로 감추지 않는다 (SteerDebug 계열과 다른 점이다).
        with self._lock:
            self.steer_cmd.push(self._now(), m.data * RAD2DEG)

    def _on_debug(self, m: SteerDebug):
        t = self._now()
        with self._lock:
            # 추종 실패 tick 의 0 을 쌓으면 플롯이 0 으로 끌려 내려가
            # 오차가 사라진 것처럼 보인다. 값을 넣지 않고 선을 끊는다.
            if not m.tracking_ok:
                return
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

    def _on_tf(self):
        """map 기준 세 프레임의 자세. map 은 항상 원점이다."""
        odom = self._lookup(self.map_frame, self.odom_frame)
        base = self._lookup(self.map_frame, self.base_frame)

        poses = [(0.0, 0.0, 0.0), None, None]
        if odom is not None:
            poses[1] = (odom[0] * CM_PER_M, odom[1] * CM_PER_M, odom[2])
        if base is not None:
            poses[2] = (base[0] * CM_PER_M, base[1] * CM_PER_M, base[2])

        with self._lock:
            self.tf_poses = poses

    # ------------------------------------------------------------------
    # 스냅샷
    # ------------------------------------------------------------------

    def snapshot(self) -> dict:
        """Qt 메인 스레드가 렌더 직전에 한 번 부른다."""
        with self._lock:
            return {
                "now": self._now(),
                "scan": (self.scan.value, self.scan.stamp),
                "path_global": (self.path_global.value,
                                self.path_global.stamp),
                "path_local": (self.path_local.value, self.path_local.stamp),
                "path_lane": (self.path_lane.value, self.path_lane.stamp),
                "obstacles": (self.obstacles.value, self.obstacles.stamp),
                "tf_poses": list(self.tf_poses),
                "nodes": list(self.nodes),
                "series": {
                    "speed_target": self.speed_target.arrays(),
                    "speed_real": self.speed_real.arrays(),
                    "steer_cmd": self.steer_cmd.arrays(),
                    "heading_err": self.heading_err.arrays(),
                    "cross_track": self.cross_track.arrays(),
                },
            }
