"""구독 전담 rclpy 노드. Qt 를 전혀 모른다.

**관측 전용.** publisher 도 service client 도 만들지 않는다.

수신을 두 계열로 나눈다. 이 구분이 설계의 핵심이다.

    latch  맵 계열 (스캔 · 경로 · 차선 · 장애물). 최신 한 장만 그린다.
    ring   플롯 계열. 콜백에서 **전량** 쌓는다. 렌더가 3 Hz 라고 50 Hz 신호를
           3 Hz 로 샘플링하면 조향 떨림 · CTE 스파이크가 화면에서 사라진다.

렌더가 3 Hz 라 12 Hz 경로의 짧은 플래그는 latch 만으로도 놓친다. 그래서
`kappa_saturated` · `stop_request` 는 **발생 시각과 횟수를 따로 기록**한다.
"이 트랙을 통과할 수 있는가" 를 이 화면 하나로 판정해야 하기 때문이다.

좌표는 전부 `base_link` 상대 **m** 다. 백업 스택은 측위 비의존이라 map/odom
TF 가 없다 -- frames 가 전부 base_link 면 TF listener 자체를 만들지 않는다.

스레드: rclpy spin 은 별도 스레드, Qt 메인 스레드는 snapshot() 만 부른다.
"""

from __future__ import annotations

import math
import pathlib
import threading
from collections import deque

import numpy as np
import rclpy
from ament_index_python.packages import (PackageNotFoundError,
                                         get_package_share_directory)
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64

from backup_msgs.msg import BackupPath, ControlDebug, LaneGeometry
from backup_msgs.msg import ObstacleCircleArray

RAD2DEG = 180.0 / math.pi

# BackupPath 한 조각을 몇 점으로 찍을지. u 등분이라 곡률이 큰 구간이 성기게
# 찍히지만 16 점이면 화면 1 px 아래다.
SAMPLES_PER_SEG = 16

# 플래그가 켜졌던 것을 화면에 붙들어 두는 시간 [s].
# 12 Hz 경로의 1 프레임 이벤트를 3 Hz 렌더로 보려면 반드시 필요하다.
FLAG_HOLD_S = 3.0


def observe_qos():
    """관측 전용 구독. RELIABLE 발행자와도 호환되고 재전송 비용을
    발행측에 지우지 않는다."""
    return QoSProfile(depth=1,
                      history=HistoryPolicy.KEEP_LAST,
                      reliability=ReliabilityPolicy.BEST_EFFORT,
                      durability=DurabilityPolicy.VOLATILE)


def sample_bezier(ctrl_x, ctrl_y, degree: int, nseg: int,
                  per_seg: int = SAMPLES_PER_SEG):
    """flat 제어점 배열 -> (xs, ys, knot_x, knot_y). 실패하면 None.

    Bernstein 기저를 한 번 만들어 segment 마다 재사용한다. quintic 고정이
    아니라 degree 를 그대로 쓰므로 메시지가 바뀌어도 따라간다.
    """
    n = degree + 1
    if nseg <= 0 or n <= 1 or len(ctrl_x) < nseg * n:
        return None

    u = np.linspace(0.0, 1.0, per_seg + 1)
    k = np.arange(n)
    coef = np.array([math.comb(degree, int(i)) for i in k], dtype=float)
    basis = coef * (u[:, None] ** k) * ((1.0 - u)[:, None] ** (degree - k))

    xs, ys, kx, ky = [], [], [], []
    for s in range(nseg):
        px = np.asarray(ctrl_x[s * n:(s + 1) * n], dtype=float)
        py = np.asarray(ctrl_y[s * n:(s + 1) * n], dtype=float)
        # 조각 끝 u=1.0 은 다음 조각의 u=0.0 이다. 마지막만 끝점을 남긴다.
        cut = None if s == nseg - 1 else -1
        xs.append((basis @ px)[:cut])
        ys.append((basis @ py)[:cut])
        kx.append(px[0])
        ky.append(py[0])

    kx.append(float(ctrl_x[nseg * n - 1]))
    ky.append(float(ctrl_y[nseg * n - 1]))
    return (np.concatenate(xs), np.concatenate(ys),
            np.asarray(kx), np.asarray(ky))


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


class Latest:
    """최신 한 장만 유지하는 표시물."""

    __slots__ = ("value", "stamp")

    def __init__(self):
        self.value = None
        self.stamp = None

    def set(self, v, t):
        self.value = v
        self.stamp = t


class Flag:
    """짧은 이벤트의 발생 시각과 횟수. 렌더 주기보다 빠른 신호용."""

    __slots__ = ("last", "count", "_on")

    def __init__(self):
        self.last = None
        self.count = 0
        self._on = False

    def mark(self, on: bool, t: float):
        if on:
            self.last = t
            if not self._on:            # 상승 에지만 센다
                self.count += 1
        self._on = on

    def state(self, now: float):
        """(현재 켜짐, 최근 발생 경과 s|None, 누적 횟수)."""
        age = None if self.last is None else (now - self.last)
        return self._on, age, self.count


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
        super().__init__("backup_gui")

        self._lock = threading.Lock()

        # 플롯 시간축의 원점. 절대 시각을 그대로 쓰면 축이 못 읽는 값이 된다.
        # use_sim_time 이면 첫 /clock 전까지 시각이 0 이라 0 이 아닌 첫 값에서
        # 원점을 잡는다.
        self._t0 = None

        self._declare()

        w = self.history_s
        self.speed_target = Series(w)
        self.speed_real = Series(w)
        self.steer_cmd = Series(w)
        self.heading_err = Series(w)
        self.cross_track = Series(w)
        self.kappa_max = Series(w)

        self.scan = Latest()
        self.path = Latest()
        self.lane = Latest()
        self.obstacles = Latest()

        self.f_kappa = Flag()
        self.f_stop = Flag()
        self.f_avoid = Flag()

        self.tf_poses = [None, None, None]      # map / odom / base

        # 최상단 상태 띠. [(이름, ok|stale|absent, hz|None)]
        # 첫 그래프 조회(1 초 뒤) 전까지도 목록은 보여야 한다.
        self.nodes = [(n, "absent", None) for n in self.watch_names]
        self._rates = {}                        # topic -> RateMeter
        self._subscribed = set()                # 이미 구독한 토픽

        self.tf_buffer = None
        self.tf_listener = None
        self._tf_warned = False
        if self.tf_enabled:
            from tf2_ros import Buffer, TransformListener
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)

        # 순서가 중요하다. meter 를 먼저 만들어야 표시용 구독이 같은 토픽을
        # 잡을 때 주기를 거기서 같이 잰다.
        self._prepare_rates()
        self._make_display_subs()
        self._make_status_subs()

        if self.tf_enabled:
            # TF 를 렌더보다 자주 볼 이유가 없다.
            self.create_timer(1.0 / max(1.0, self.render_hz), self._on_tf)
        if self.watch_names:
            self.create_timer(1.0, self._on_graph)    # 1 Hz

        self.get_logger().info(
            f"[기동][gui] 관측 전용. render {self.render_hz:g} Hz · "
            f"history {self.history_s:g} s · 감시 {len(self.watch_names)} 노드 · "
            f"TF {'표시' if self.tf_enabled else '없음'}")

    # ------------------------------------------------------------------
    # 파라미터
    # ------------------------------------------------------------------

    def _declare(self):
        d = self.declare_parameter

        self.render_hz = d("render_hz", 3.0).value
        self.history_s = d("history_s", 15.0).value

        # 통과 가능 곡률 상한 [1/m]. backup_common VehicleParams::kappaMax().
        self.kappa_ref = d("kappa_max_ref", 2.0221).value

        # /steering 은 rad (kau_control/src/steer_controller_node.cpp:66 확인).
        # ControlDebug.cmd_steer_deg 만 deg 다.
        self.steering_rad = d("steering_rad", True).value

        self.map_frame = d("frames.map", "base_link").value
        self.odom_frame = d("frames.odom", "base_link").value
        self.base_frame = d("frames.base", "base_link").value
        # 전부 base_link 면 그릴 축이 없다. 조회 자체를 하지 않는다.
        self.tf_enabled = (self.map_frame != self.base_frame
                           or self.odom_frame != self.base_frame)

        self.tf_axis_m = d("tf_axis_m", 0.2).value

        self.map_pkg = d("map.package", "").value
        self.map_name = d("map.name", "").value
        self.map_yaml = self._resolve_map(d("map.yaml_path", "").value)

        self.track_enabled = d("track.enabled", False).value
        self.track_pkg = d("track.package", "").value
        self.track_name = d("track.name", "").value
        self.track_tick_m = d("track.s_tick_m", 5.0).value
        self.track_labels = d("track.labels", False).value
        self.track_yaml = self._resolve_track(d("track.yaml_path", "").value)

        self.topics = {
            k: d(f"topics.{k}", v).value for k, v in (
                ("scan", "/scan_filtered"),
                ("obstacles", "/backup/perception/obstacles"),
                ("path_local", "/backup/path"),
                ("path_lane", "/backup/lane/geometry"),
                ("speed_cmd", "/speed"),
                ("odom", "/odom"),
                ("steering", "/steering"),
                ("steer_debug", "/backup/debug/steer"))}

        # 점군 거리색의 양 끝 [m]. 고정값이라 차가 움직여도 같은 거리가
        # 같은 색으로 남는다.
        self.scan_near_m = d("scan.near_m", 0.0).value
        self.scan_far_m = d("scan.far_m", 3.0).value
        self.scan_size = d("scan.point_size", 3.0).value

        # lidar_link -> 후륜축 정적 변환. TF 를 안 쓰므로 상수다.
        # 값은 backup_object_detection/config 와 같아야 장애물과 점군이
        # 같은 자리에 겹친다.
        self.lidar_x_m = d("scan.lidar_x_m", -0.027).value
        self.lidar_y_m = d("scan.lidar_y_m", 0.0).value
        self.lidar_yaw = d("scan.lidar_yaw_rad", 0.0).value
        self.rear_axle_offset_m = d("scan.rear_axle_offset_m", -0.090).value

        self._declare_status()

    # ------------------------------------------------------------------
    # 노드 상태 (최상단 띠)
    # ------------------------------------------------------------------

    def _declare_status(self):
        """감시 대상은 gui.yaml 의 status.watch 하나로 정한다.

        백업 스택에는 supervisor 도 bringup manifest 도 없다.
        """
        d = self.declare_parameter

        self.stale_ratio = d("status.stale_ratio", 3.0).value
        self.min_stale_s = d("status.min_stale_s", 0.3).value

        # 기본값이 빈 배열이 아니라 [""] 인 이유: rclpy 는 기본값에서 타입을
        # 추론하는데 빈 배열은 BYTE_ARRAY 로 잡힌다. 그러면 yaml 의 문자열
        # 배열과 타입이 안 맞아 GUI 가 기동하지 못한다.
        self.watch_spec = self._parse_watch(
            d("status.watch", [""]).value or [])

        # dict 는 삽입 순서를 지킨다 = yaml 에 적은 순서 그대로 표시된다.
        self.watch_names = (list(self.watch_spec)
                            if d("status.enabled", True).value else [])

    def _parse_watch(self, lines) -> dict:
        """"<노드> | <topic> | <타입> | <기대 Hz>" 목록 -> dict.

        ROS 파라미터 배열은 원소 타입이 같아야 해서 한 줄 문자열로 받는다.
        평행 배열로 두면 길이가 어긋나는 순간 전체가 한 칸씩 밀린다.
        """
        out = {}
        for line in lines:
            if not str(line).strip():       # 기본값 [""] 의 빈 칸
                continue
            parts = [f.strip() for f in str(line).split("|")]
            if len(parts) != 4:
                self.get_logger().error(
                    f"[설정][gui] status.watch 형식 오류 (칸 4 개): {line!r}")
                continue
            name, topic, typ, rate = parts
            try:
                out[name] = (topic, typ, float(rate))
            except ValueError:
                self.get_logger().error(
                    f"[설정][gui] status.watch 의 기대 Hz 가 수가 아니다: {line!r}")
        return out

    def _prepare_rates(self):
        """감시 토픽마다 RateMeter 하나. 구독보다 먼저 만든다."""
        for topic, typ, rate in self.watch_spec.values():
            if not topic or not typ or rate <= 0.0:
                continue
            self._rates.setdefault(topic, RateMeter())

    def _make_status_subs(self):
        """표시용으로 안 잡는 감시 토픽만 raw 구독. 역직렬화하지 않고
        수신 시각만 찍는다."""
        for name in self.watch_names:
            topic, typ, _rate = self.watch_spec[name]
            if topic not in self._rates or topic in self._subscribed:
                continue

            self._subscribed.add(topic)
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
                    f"[설정][gui] 감시 구독 실패 {topic} ({typ}): {e}")

    def _on_graph(self):
        alive = {n for n, _ns in self.get_node_names_and_namespaces()}
        t = self._now()

        out = []
        for name in self.watch_names:
            if name not in alive:
                out.append((name, "absent", None))
                continue

            topic, _typ, rate = self.watch_spec[name]
            if not topic or rate <= 0.0:
                # 대표 토픽이 없거나 latched 라 주기 판정 불가. 존재만으로
                # 정상 처리하고 Hz 칸은 비운다.
                out.append((name, "ok", None))
                continue

            meter = self._rates.get(topic)
            if meter is None or meter.last is None:
                out.append((name, "stale", None))
                continue

            # 50 Hz 토픽은 기대주기의 3 배가 60 ms 라 지터만으로 깜빡인다.
            limit = max(self.stale_ratio / rate, self.min_stale_s)
            out.append((name, "ok" if (t - meter.last) <= limit else "stale",
                        meter.hz))

        with self._lock:
            self.nodes = out

    # ------------------------------------------------------------------
    # 선택 자원 (없어도 기동한다. 경고는 기동 시 1 회뿐)
    # ------------------------------------------------------------------

    def _resolve_map(self, override: str) -> str:
        if override:
            return override
        if not self.map_pkg or not self.map_name:
            self.get_logger().info("[설정][gui] 맵 배경 없음 (map.package/name 비었음)")
            return ""
        try:
            share = get_package_share_directory(self.map_pkg)
        except (PackageNotFoundError, ValueError):
            self.get_logger().warn(
                f"[설정][gui] 패키지를 못 찾음: {self.map_pkg}. 맵 배경 없이 뜬다")
            return ""
        path = pathlib.Path(share) / "maps" / f"{self.map_name}.yaml"
        if not path.is_file():
            self.get_logger().warn(
                f"[설정][gui] 맵 yaml 없음: {path}. 맵 배경 없이 뜬다")
            return ""
        return str(path)

    def _resolve_track(self, override: str) -> str:
        if not self.track_enabled:
            return ""
        if override:
            return override
        if not self.track_pkg or not self.track_name:
            self.get_logger().warn(
                "[설정][gui] track.enabled 인데 package/name 이 비었다. 트랙 없이 뜬다")
            return ""
        try:
            share = get_package_share_directory(self.track_pkg)
        except (PackageNotFoundError, ValueError):
            self.get_logger().warn(
                f"[설정][gui] 패키지를 못 찾음: {self.track_pkg}. 참값 트랙 없이 뜬다")
            return ""
        path = pathlib.Path(share) / "config" / f"{self.track_name}.yaml"
        if not path.is_file():
            self.get_logger().warn(
                f"[설정][gui] 트랙 yaml 없음: {path}. 참값 트랙 없이 뜬다")
            return ""
        return str(path)

    def _now(self):
        """기동 시점 기준 경과 초. 표시·신선도 판정에 쓰는 유일한 시각이다."""
        t = self.get_clock().now().nanoseconds * 1e-9
        if self._t0 is None:
            if t <= 0.0:
                return 0.0
            self._t0 = t
        return t - self._t0

    # ------------------------------------------------------------------
    # 구독
    # ------------------------------------------------------------------

    def _sub(self, msg_type, topic, cb):
        """표시용 구독 하나. 빈 토픽 이름이면 건너뛴다.

        감시 목록에 같은 토픽이 있으면 주기도 여기서 잰다. 따로 구독하면
        같은 메시지를 두 번 받는다. mark 는 콜백 본체보다 먼저다 --
        중간에 return 하는 콜백이 있어 뒤에 두면 주기가 끊긴 것으로 읽힌다.
        """
        if not topic:
            return

        meter = self._rates.get(topic)
        if meter is not None:
            body = cb

            def cb(m, _body=body, _meter=meter):
                _meter.mark(self._now())
                _body(m)

        self._subscribed.add(topic)
        self.create_subscription(msg_type, topic, cb, observe_qos())

    def _make_display_subs(self):
        t = self.topics
        self._sub(LaserScan, t["scan"], self._on_scan)
        self._sub(ObstacleCircleArray, t["obstacles"], self._on_obstacles)
        self._sub(BackupPath, t["path_local"], self._on_path)
        self._sub(LaneGeometry, t["path_lane"], self._on_lane)

        self._sub(Odometry, t["odom"], self._on_odom)
        self._sub(Float64, t["speed_cmd"], self._on_speed)
        self._sub(Float64, t["steering"], self._on_steering)
        self._sub(ControlDebug, t["steer_debug"], self._on_debug)

    # ------------------------------------------------------------------
    # 표시용 콜백 (예외를 밖으로 던지지 않는다)
    # ------------------------------------------------------------------

    def _on_scan(self, m: LaserScan):
        r = np.asarray(m.ranges, dtype=float)
        ok = np.isfinite(r) & (r >= m.range_min) & (r <= m.range_max)
        if not ok.any():
            return
        idx = np.nonzero(ok)[0]
        a = m.angle_min + m.angle_increment * idx
        rr = r[idx]
        lx, ly = rr * np.cos(a), rr * np.sin(a)

        # lidar_link -> 후륜축. TF 가 아니라 yaml 상수다.
        c, s = math.cos(self.lidar_yaw), math.sin(self.lidar_yaw)
        xs = self.lidar_x_m + c * lx - s * ly - self.rear_axle_offset_m
        ys = self.lidar_y_m + s * lx + c * ly

        # 색 기준은 변환 전 원본 range [m] 다.
        with self._lock:
            self.scan.set((xs, ys, rr), self._now())

    def _on_obstacles(self, m: ObstacleCircleArray):
        # status 가 OK 가 아니면 배열이 비어 온다 (재사용 금지 규약).
        out = [(o.center_x, o.center_y, o.radius) for o in m.obstacles]
        with self._lock:
            self.obstacles.set((out, int(m.status)), self._now())

    def _on_path(self, m: BackupPath):
        t = self._now()
        nseg = len(m.seg_length)
        pts = sample_bezier(m.ctrl_x, m.ctrl_y, int(m.degree), nseg)

        kmax = float(max(m.seg_kappa_max)) if len(m.seg_kappa_max) else 0.0

        with self._lock:
            self.kappa_max.push(t, kmax)
            self.f_kappa.mark(bool(m.kappa_saturated), t)
            self.f_stop.mark(bool(m.stop_request), t)
            self.f_avoid.mark(bool(m.avoiding), t)

            if pts is None:
                self.path.set(None, t)
                return

            xs, ys, kx, ky = pts
            # 외삽 시작점. 호길이는 seg_length 가 아니라 찍은 점의 현길이로
            # 잰다 -- 화면에 그린 선과 잘린 지점이 반드시 일치해야 한다.
            cum = np.concatenate(
                ([0.0], np.cumsum(np.hypot(np.diff(xs), np.diff(ys)))))
            split = int(np.clip(
                np.searchsorted(cum, float(m.valid_length), side="right") - 1,
                0, len(xs) - 1))

            self.path.set({
                "xs": xs, "ys": ys, "split": split,
                "knot_x": kx, "knot_y": ky,
                "source": int(m.source),
                "confidence": float(m.confidence),
                "total_length": float(m.total_length),
                "valid_length": float(m.valid_length),
                "corner_entry_s": float(m.corner_entry_s),
                "corner_kappa_max": float(m.corner_kappa_max),
                "kappa_max": kmax,
                "kappa_saturated": bool(m.kappa_saturated),
                "avoiding": bool(m.avoiding),
                "stop_request": bool(m.stop_request),
                "lost_state": int(m.lost_state),
                "lost_first": int(m.lost_first),
            }, t)

    def _on_lane(self, m: LaneGeometry):
        def line(ln, valid):
            if not valid:
                return None
            return ((ln.x0, ln.x1), (ln.y0, ln.y1))

        infl = None
        if m.inflection_valid:
            infl = (float(m.inflection_x), float(m.inflection_y),
                    float(m.delta_psi) * RAD2DEG)

        with self._lock:
            self.lane.set({
                "left": line(m.left, m.left_valid),
                "right": line(m.right, m.right_valid),
                "center": line(m.center, m.center_valid),
                "inflection": infl,
                "arc_length": float(m.arc_length),
                "second_inflection": bool(m.second_inflection),
                "lost_state": int(m.lost_state),
                "lost_first": int(m.lost_first),
                "valid_length": float(m.valid_length),
                "conf": (float(m.left.confidence), float(m.right.confidence),
                         float(m.center.confidence)),
            }, self._now())

    def _on_odom(self, m: Odometry):
        with self._lock:
            self.speed_real.push(self._now(), m.twist.twist.linear.x)

    def _on_speed(self, m: Float64):
        with self._lock:
            self.speed_target.push(self._now(), m.data)

    def _on_steering(self, m: Float64):
        # 플롯은 deg 로 통일한다. max_steer_deg(20) 와 눈으로 대조하려면
        # deg 여야 한다. 추종 실패 tick 의 0 도 그대로 넣는다 -- 실제로
        # 차량에 나간 값이므로 감추지 않는다 (ControlDebug 계열과 다른 점).
        v = m.data * RAD2DEG if self.steering_rad else m.data
        with self._lock:
            self.steer_cmd.push(self._now(), v)

    def _on_debug(self, m: ControlDebug):
        t = self._now()
        with self._lock:
            # 추종 실패 tick 의 0 을 쌓으면 플롯이 0 으로 끌려 내려가
            # 오차가 사라진 것처럼 보인다. 값을 넣지 않고 선을 끊는다.
            if not m.tracking_ok:
                return
            self.heading_err.push(t, m.heading_error_rad * RAD2DEG)
            self.cross_track.push(t, m.cross_track_m)

    # ------------------------------------------------------------------
    # TF (frames 가 전부 base_link 면 아예 돌지 않는다)
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

        if odom is None and base is None and not self._tf_warned:
            # 매 프레임 찍으면 화면이 이 로그로 덮인다. 한 번만 알린다.
            self._tf_warned = True
            self.get_logger().warn(
                f"[설정][gui] TF 조회 실패 ({self.map_frame} -> "
                f"{self.odom_frame}/{self.base_frame}). 축 마커 없이 계속한다")

        with self._lock:
            self.tf_poses = [(0.0, 0.0, 0.0), odom, base]

    # ------------------------------------------------------------------
    # 스냅샷
    # ------------------------------------------------------------------

    def snapshot(self) -> dict:
        """Qt 메인 스레드가 렌더 직전에 한 번 부른다."""
        with self._lock:
            now = self._now()
            return {
                "now": now,
                "scan": (self.scan.value, self.scan.stamp),
                "path": (self.path.value, self.path.stamp),
                "lane": (self.lane.value, self.lane.stamp),
                "obstacles": (self.obstacles.value, self.obstacles.stamp),
                "tf_poses": list(self.tf_poses),
                "nodes": list(self.nodes),
                "flags": {
                    "kappa": self.f_kappa.state(now),
                    "stop": self.f_stop.state(now),
                    "avoid": self.f_avoid.state(now),
                },
                "series": {
                    "speed_target": self.speed_target.arrays(),
                    "speed_real": self.speed_real.arrays(),
                    "steer_cmd": self.steer_cmd.arrays(),
                    "heading_err": self.heading_err.arrays(),
                    "cross_track": self.cross_track.arrays(),
                    "kappa_max": self.kappa_max.arrays(),
                },
            }
