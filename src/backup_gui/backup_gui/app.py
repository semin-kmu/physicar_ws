"""백업 스택 디버깅 GUI. 관측 전용.

    최상단   노드 상태 띠 (status_bar). 가로 전체
    좌열     지표 패널 5 개
    우열     map (base_link 고정 탑뷰, rowspan 5)
    열 비율  95 : 125

차량은 항상 원점이다. 백업 스택은 측위가 없어 절대좌표 자체가 없다.

**κ_max 패널과 map 좌상단의 KAPPA SAT 표시가 이 화면의 핵심이다.**
"이 트랙을 조향으로 통과할 수 있는가" 를 판정할 유일한 신호다.

조작
    space  화면 갱신 일시정지 (수신은 계속. 순간 관찰용)
    r      뷰 리셋
"""

from __future__ import annotations

from . import _venv

_venv.ensure("pyqtgraph")            # noqa: E402  (import 순서보다 먼저다)

import threading                     # noqa: E402

import numpy as np                   # noqa: E402
import pyqtgraph as pg               # noqa: E402
import rclpy                         # noqa: E402
from pyqtgraph.Qt import QtCore, QtWidgets   # noqa: E402

from . import viz                    # noqa: E402
from .bridge import FLAG_HOLD_S, Bridge      # noqa: E402
from .status_bar import StatusBar    # noqa: E402

PANELS = [
    ("speed_target", "speed [m/s]"),
    ("steer_cmd", "steer [deg]"),
    ("cross_track", "CTE [m]"),
    ("heading_err", "θ_err [deg]"),
    # 축 라벨은 기본 폰트라 κ 가 tofu 로 뜨는 시스템이 있다. 여기만 ascii.
    ("kappa_max", "kappa [1/m]"),
]

COL_REF, COL_EMA = "#1f77b4", "#2ca02c"
COL_TEXT = "#404040"

# 같은 패널에 겹쳐 그리는 보조 계열. key -> (보조 key, 색, 점선 여부)
OVERLAY = {
    "speed_target": ("speed_real", COL_EMA, False),
}

STALE_S = 1.0

# 선 굵기. 겹쳐 그리는 것이 많아 얇게 둔다. 1.0 이하로 내리지 말 것
# (pyqtgraph drawLines 빠른 경로 조건).
W_PLOT = 1.5
W_PATH = 2.0
W_PATH_ALERT = 3.0

EMPTY = (np.empty(0), np.empty(0))

LOST_NAME = {0: "NONE", 1: "LEFT", 2: "RIGHT", 3: "BOTH"}
LOST_TURN = {1: "좌", 2: "우"}


def _path_lines(d, flags):
    """map 좌상단 텍스트. [(글자, 색)]."""
    kappa_on, kappa_age, kappa_n = flags["kappa"]
    stop_on, stop_age, stop_n = flags["stop"]
    avoid_on, _avoid_age, _avoid_n = flags["avoid"]

    out = []
    if d is None:
        out.append(("path --", COL_TEXT))
    else:
        src = viz.SRC_NAME.get(d["source"], f"? {d['source']}")
        col = (viz.COL_ALERT if d["kappa_saturated"]
               else viz.SRC_COLOR.get(d["source"], viz.COL_PATH_UNK))
        out.append((f"{src}  conf {d['confidence']:.2f}", col))
        out.append((f"valid {d['valid_length']:.2f} / "
                    f"{d['total_length']:.2f} m", COL_TEXT))
        corner = ("코너 없음" if d["corner_entry_s"] < 0.0
                  else f"코너 {d['corner_entry_s']:.2f} m "
                       f"κ {d['corner_kappa_max']:.2f}")
        out.append((corner, COL_TEXT))

    # 12 Hz 경로의 1 프레임 이벤트를 3 Hz 렌더로 보려면 지나간 것도 붙든다.
    def flag(label, on, age, n):
        if on:
            return (f"{label} ●  x{n}", viz.COL_ALERT)
        if age is not None and age <= FLAG_HOLD_S:
            return (f"{label} ({age:.1f}s 전) x{n}", viz.COL_ALERT)
        if n:
            return (f"{label} x{n}", COL_TEXT)
        return None

    for line in (flag("KAPPA SAT", kappa_on, kappa_age, kappa_n),
                 flag("STOP", stop_on, stop_age, stop_n)):
        if line:
            out.append(line)

    if avoid_on:
        out.append(("AVOIDING", viz.SRC_COLOR[2]))
    return out


def _lane_lines(d, obs_status):
    """map 우상단 텍스트. [(글자, 색)]."""
    out = []
    if d is None:
        out.append(("lane --", COL_TEXT))
    else:
        lost = d["lost_state"]
        name = LOST_NAME.get(lost, str(lost))
        if lost == 3:
            # fallback 이 먼저 끊긴 쪽으로 최대 조향한다. 방향까지 보여야
            # 화면의 조향 명령과 대조가 된다.
            turn = LOST_TURN.get(d["lost_first"], "?")
            out.append((f"lost BOTH · first {LOST_NAME.get(d['lost_first'], '?')}"
                        f" → {turn} 전타", viz.COL_ALERT))
        elif lost:
            out.append((f"lost {name}", "#b8860b"))
        else:
            out.append(("lost NONE", COL_TEXT))

        if d["inflection"] is not None:
            out.append((f"Δψ {d['inflection'][2]:+.1f}°  "
                        f"arc {d['arc_length']:.2f} m", viz.COL_INFL))
        if d["second_inflection"]:
            out.append(("연속 코너", "#b8860b"))
        out.append((f"lane valid {d['valid_length']:.2f} m", COL_TEXT))

    if obs_status:      # ObstacleCircleArray.STATUS_OK = 0
        out.append((f"obstacle status {obs_status}", viz.COL_ALERT))
    return out


def _run(bridge: Bridge, fps: float) -> None:
    viz.init("backup_gui")

    # 상태 띠는 pyqtgraph 그리드 밖이다. 좌우 열 비율과 무관하게 가로 전체를
    # 써야 하므로 GraphicsLayoutWidget 위에 얹지 않는다.
    root = QtWidgets.QWidget()
    layout = QtWidgets.QVBoxLayout(root)
    layout.setContentsMargins(0, 0, 0, 0)
    layout.setSpacing(0)

    status = StatusBar(root)
    layout.addWidget(status, 0)

    win = pg.GraphicsLayoutWidget()
    layout.addWidget(win, 1)

    panels = viz.stack_plots(win, PANELS, col=0, row0=0, xlabel="t [s]")
    curves = {}
    for key, p in panels.items():
        if key in OVERLAY:
            alt, col, dashed = OVERLAY[key]
            style = QtCore.Qt.DotLine if dashed else QtCore.Qt.SolidLine
            curves[alt] = p.plot([], [], pen=pg.mkPen(col, width=W_PLOT,
                                                      style=style))
        curves[key] = p.plot([], [], pen=pg.mkPen(COL_REF, width=W_PLOT))

    # 통과 불가 경계. 경로 곡률이 이 선을 넘으면 kappa_saturated 가 뜬다.
    viz.limit_line(panels["kappa_max"], bridge.kappa_ref)

    m = viz.map_plot(win, row=0, col=1, rowspan=len(PANELS))

    if bridge.map_yaml:
        try:
            viz.draw_map_image(m, *viz.load_map(bridge.map_yaml))
        except Exception as e:
            # 배경이 없다고 GUI 를 죽이지 않는다.
            print(f"[설정][gui] 맵 배경 로드 실패 ({bridge.map_yaml}): {e}")
    else:
        viz.vehicle_view(m, bridge.scan_far_m)

    # 참값 트랙. 정적이라 여기서 한 번만 그린다 (update 에 안 들어간다).
    if bridge.track_yaml:
        try:
            viz.draw_track(m, viz.load_track(bridge.track_yaml),
                           s_tick_m=bridge.track_tick_m,
                           labels=bridge.track_labels)
        except Exception as e:
            print(f"[설정][gui] 참값 트랙 로드 실패 ({bridge.track_yaml}): {e}")

    # 추가 순서가 곧 겹침 순서다. 근거(차선) -> 관측(점군·장애물) -> 판단(경로).
    lane = viz.LaneView(m)
    scan = viz.ScanCloud(m, size=bridge.scan_size,
                         near_m=bridge.scan_near_m, far_m=bridge.scan_far_m)
    obstacles = viz.Obstacles(m, color=viz.COL_OBS)
    path = viz.BezierPath(m, width=W_PATH, width_alert=W_PATH_ALERT)

    tf = (viz.TfChain(m, ("map", "odom", "base_link"),
                      length_m=bridge.tf_axis_m) if bridge.tf_enabled else None)

    # 차체는 원점 고정이다 (base_link 상대좌표). 한 번만 그린다.
    m.plot(*viz.car_shape(), pen=pg.mkPen(viz.COL_CAR, width=W_PATH))
    m.disableAutoRange()

    ov_path = viz.Overlay(m, corner="tl")
    ov_lane = viz.Overlay(m, corner="tr")

    state = {"paused": False}
    first = next(iter(panels.values()))

    def update():
        if state["paused"]:
            return

        s = bridge.snapshot()
        now = s["now"]

        status.set_state(s["nodes"])

        for name, c in curves.items():
            c.setData(*s["series"][name])
        # 패널이 setXLink 로 묶여 있으므로 하나만 잡으면 전부 따라온다.
        first.setXRange(now - bridge.history_s, now, padding=0.0)

        def fresh(stamp, timeout=STALE_S):
            return stamp is not None and (now - stamp) <= timeout

        pts, st = s["scan"]
        if pts is not None and fresh(st):
            scan.update(*pts)
        else:
            scan.clear()

        pd, st = s["path"]
        pd = pd if fresh(st) else None
        path.update(pd)

        ld, st = s["lane"]
        ld = ld if fresh(st) else None
        lane.update(ld)

        obs, st = s["obstacles"]
        circles, obs_status = obs if obs is not None else ([], 0)
        if not fresh(st):
            circles, obs_status = [], 0
        obstacles.update(circles)

        ov_path.set(_path_lines(pd, s["flags"]))
        ov_lane.set(_lane_lines(ld, obs_status))

        if tf is not None:
            tf.update(s["tf_poses"])

    def key(ev):
        k = ev.key()
        if k == QtCore.Qt.Key_Space:
            state["paused"] = not state["paused"]
        elif k == QtCore.Qt.Key_R:
            if bridge.map_yaml:
                m.autoRange()
            else:
                viz.vehicle_view(m, bridge.scan_far_m)

    root.setFocusPolicy(QtCore.Qt.StrongFocus)
    root.keyPressEvent = key
    win.ci.layout.setColumnStretchFactor(0, 95)
    win.ci.layout.setColumnStretchFactor(1, 125)

    # 첫 tick 전에 한 번 그린다. 안 그러면 창이 뜬 뒤 1/fps 동안 상태 띠가
    # "감시 대상 없음" 으로 보여 설정이 틀린 것처럼 읽힌다.
    update()

    timer = QtCore.QTimer(root)
    timer.timeout.connect(update)
    timer.start(int(round(1000.0 / fps)))
    viz.run(root, "backup_gui")


def main(args=None):
    rclpy.init(args=args)

    try:
        bridge = Bridge()
    except Exception as e:
        # 설정 오류는 여기서 걸린다. 창을 띄우고 나서 죽는 것보다
        # 이유를 남기고 즉시 끝내는 편이 낫다.
        print(f"[기동][gui] 기동 실패: {e}")
        rclpy.shutdown()
        return 1

    stop = threading.Event()

    def spin():
        while rclpy.ok() and not stop.is_set():
            rclpy.spin_once(bridge, timeout_sec=0.05)

    thread = threading.Thread(target=spin, daemon=True)
    thread.start()

    try:
        _run(bridge, max(1.0, bridge.render_hz))
    finally:
        stop.set()
        thread.join(timeout=2.0)
        bridge.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
