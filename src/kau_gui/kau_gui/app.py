"""주행 디버깅 GUI. 관측 전용.

화면 구성은 KAU_AMET_Test/src/full_simulation.py 의 show_realtime 을 그대로
따른다. 시뮬과 실차를 나란히 놓고 보므로 같은 자리에 같은 것이 있어야 한다.

    row 0        제목 줄 (colspan 2, 11pt)
    row 1..5     왼쪽 지표 패널 5 개 / 오른쪽 map (rowspan 5)
    열 비율      95 : 125

시뮬과 다를 수밖에 없는 것은 하나뿐이다. 시뮬은 배치 재생이라 전체 로그를
옅은 회색으로 미리 깔고 커서를 옮기지만, 실시간에는 "미래" 가 없다.
그래서 회색 밑그림과 커서 InfiniteLine 이 빠진다.

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
from pyqtgraph.Qt import QtCore      # noqa: E402

from . import viz                    # noqa: E402
from .bridge import Bridge           # noqa: E402

# full_simulation.PANELS 와 같은 구성 · 같은 순서 · 같은 라벨
PANELS = [
    ("speed_target", "target speed [m/s]"),
    ("steer_raw", "target steer [deg]"),
    ("lookahead", "lookahead distance [cm]"),
    ("cross_track", "cross track error [cm]"),
    ("heading_err", "heading error [deg]"),
]

COL_REF, COL_TRUTH_ERR, COL_EMA = "#1f77b4", "#d62728", "#2ca02c"

# full_simulation.OVERLAY 와 같은 규약. key -> (보조 key, 색, 설명, 점선 여부)
OVERLAY = {
    "speed_target": ("speed_real", COL_EMA,
                     "blue = target (/speed),  green = measured (/odom)",
                     False),
    "steer_raw": ("steer_cmd", COL_TRUTH_ERR,
                  "blue = raw,  red dotted = commanded (/steering)", True),
}

STALE_S = 1.0

EMPTY = (np.empty(0), np.empty(0))


def _run(bridge: Bridge, fps: float) -> None:
    viz.init("KAU AMET GUI")
    win = pg.GraphicsLayoutWidget()
    win.addLabel(f"map={bridge.map_frame}  odom={bridge.odom_frame}  "
                 f"base={bridge.base_frame}  |  render={fps:g} Hz  |  "
                 f"history={bridge.history_s:g} s",
                 row=0, col=0, colspan=2, size="11pt")

    panels = viz.stack_plots(win, PANELS, col=0, row0=1, xlabel="t [s]")
    curves = {}
    for key, p in panels.items():
        if key in OVERLAY:
            alt, col, note, dashed = OVERLAY[key]
            p.setTitle(note, size="8pt")
            style = QtCore.Qt.DotLine if dashed else QtCore.Qt.SolidLine
            curves[alt] = p.plot([], [], pen=pg.mkPen(col, width=2,
                                                      style=style))
        curves[key] = p.plot([], [], pen=pg.mkPen(COL_REF, width=2))

    m = viz.map_plot(win, row=1, col=1, rowspan=len(PANELS), title="map")
    m.addLegend(offset=(-10, 10), labelTextSize="8pt")

    if bridge.map_yaml:
        try:
            viz.draw_map_image(m, *viz.load_map(bridge.map_yaml))
        except Exception as e:
            # 배경이 없다고 GUI 를 죽이지 않는다. 나머지는 그대로 보인다.
            print(f"[kau_gui] 맵 배경 로드 실패 ({bridge.map_yaml}): {e}")

    scan_pts = m.plot([], [], pen=None, symbol="o", symbolSize=2,
                      symbolBrush=viz.COL_SCAN, symbolPen=None, name="scan")
    global_line = m.plot([], [], pen=pg.mkPen(viz.COL_GLOBAL, width=2),
                         name="global path")
    lane_line = m.plot([], [], pen=pg.mkPen(viz.COL_LANE, width=3),
                       name="lane detection")
    local_line = m.plot([], [], pen=pg.mkPen(viz.COL_LOCAL, width=3),
                        name="local path")
    obstacles = viz.Obstacles(m, color=viz.COL_OBS)
    m.plot([], [], pen=None, symbol="o", symbolSize=7,
           symbolBrush=viz.COL_OBS, symbolPen=None, name="obstacle")
    tf = viz.TfChain(m, ("map", "odom", "base_link"),
                     length_cm=bridge.tf_axis_cm)
    body = m.plot([], [], pen=pg.mkPen(viz.COL_CAR, width=2))
    m.disableAutoRange()

    hud = viz.hud(m)
    state = {"paused": False}

    first = next(iter(panels.values()))

    def update():
        if state["paused"]:
            return

        s = bridge.snapshot()
        now = s["now"]

        for name, c in curves.items():
            c.setData(*s["series"][name])
        # 패널이 setXLink 로 묶여 있으므로 하나만 잡으면 전부 따라온다.
        first.setXRange(now - bridge.history_s, now, padding=0.0)

        def fresh(stamp, timeout=STALE_S):
            return stamp is not None and (now - stamp) <= timeout

        xy, st = s["scan"]
        scan_pts.setData(*(xy if xy is not None and fresh(st) else EMPTY))

        # 전역 경로는 latched 라 한 번만 온다. 다른 것과 같은 잣대로 재면 안 된다.
        xy, _ = s["path_global"]
        global_line.setData(*(xy if xy is not None else EMPTY))

        for key, line in (("path_lane", lane_line), ("path_local", local_line)):
            xy, st = s[key]
            line.setData(*(xy if xy is not None and fresh(st) else EMPTY))

        obs, st = s["obstacles"]
        obstacles.update(obs if obs is not None and fresh(st) else [])

        tf.update(s["tf_poses"])

        base = s["tf_poses"][2]
        body.setData(*(viz.car_shape(*base) if base is not None else EMPTY))

        hud.setText(_hud_text(s, now, bridge.tf_timeout))

    def key(ev):
        k = ev.key()
        if k == QtCore.Qt.Key_Space:
            state["paused"] = not state["paused"]
        elif k == QtCore.Qt.Key_R:
            m.autoRange()

    win.setFocusPolicy(QtCore.Qt.StrongFocus)
    win.keyPressEvent = key
    win.ci.layout.setColumnStretchFactor(0, 95)
    win.ci.layout.setColumnStretchFactor(1, 125)

    timer = QtCore.QTimer(win)
    timer.timeout.connect(update)
    timer.start(int(round(1000.0 / fps)))
    viz.run(win, "KAU AMET GUI")


def _hud_text(s, now, tf_timeout) -> str:
    """맵에 그려지는 것들의 수신 상태. 제어 수치는 왼쪽 패널 소관이다."""
    lines = []

    def age(stamp):
        return "      " if stamp is None else f"{now - stamp:5.1f}s"

    def path(label, key, timeout=STALE_S):
        xy, st = s[key]
        if xy is None:
            lines.append(f"{label:<13}      --")
            return
        ok = st is not None and (now - st) <= timeout
        lines.append(f"{label:<13} {len(xy[0]):4d} pt {age(st)}"
                     f"{'' if ok else '  STALE'}")

    path("global path", "path_global", timeout=1e9)
    path("local path", "path_local")
    path("lane center", "path_lane")

    for name, a in zip(("tf map", "tf odom", "tf base_link"), s["tf_ages"]):
        if a is None:
            lines.append(f"{name:<13}      --")
        else:
            lines.append(f"{name:<13}      {a:5.1f}s"
                         f"{'' if a <= tf_timeout else '  STALE'}")

    obs, st = s["obstacles"]
    if obs is None:
        lines.append(f"{'obstacles':<13}      --")
    else:
        ok = st is not None and (now - st) <= STALE_S
        lines.append(f"{'obstacles':<13} {len(obs):4d} EA {age(st)}"
                     f"{'' if ok else '  STALE'}")

    return "\n".join(lines)


def main(args=None):
    rclpy.init(args=args)

    try:
        bridge = Bridge()
    except Exception as e:
        # 설정 오류는 여기서 걸린다. 창을 띄우고 나서 죽는 것보다
        # 이유를 남기고 즉시 끝내는 편이 낫다.
        print(f"[kau_gui] 기동 실패: {e}")
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
