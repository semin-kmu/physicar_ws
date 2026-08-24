"""주행 디버깅 GUI. 관측 전용.

화면 규약은 KAU_AMET_Test/src/full_simulation.py 의 show_realtime 과 같다.
왼쪽 = 지표 패널 5 개, 오른쪽 = map, 열 비율 95:125. 위에 노드 상태등이 붙는다.

    +----------------------------------------------+
    | 노드 상태등 + Hz            (가로 전체)        |
    +---------------------+------------------------+
    | target speed        |  map                   |
    | target steer        |   scan · TF · obstacle |
    | lookahead distance  |   global/local/lane    |
    | cross track error   |            [HUD][범례] |
    | heading error       |                        |
    +---------------------+------------------------+

렌더는 QTimer (기본 10 Hz). ROS 콜백 도착 주기와 독립이다.
플롯 데이터는 콜백에서 전량 쌓이므로 50 Hz 신호도 누락되지 않는다.

조작
    space  화면 갱신 일시정지 (수신은 계속. 순간 관찰용)
    r      뷰 리셋
    f      차량 추종 토글
"""

from __future__ import annotations

from . import _venv

_venv.ensure("pyqtgraph")            # noqa: E402  (import 순서보다 먼저다)

import math                          # noqa: E402
import threading                     # noqa: E402

import numpy as np                   # noqa: E402
import pyqtgraph as pg               # noqa: E402
import rclpy                         # noqa: E402
from pyqtgraph.Qt import QtCore, QtWidgets   # noqa: E402

from . import viz                    # noqa: E402
from .bridge import Bridge           # noqa: E402
from .status_bar import StatusBar    # noqa: E402

# full_simulation.PANELS 와 같은 구성 · 같은 순서
PANELS = [
    ("speed", "target speed [m/s]"),
    ("steer", "target steer [deg]"),
    ("lookahead", "lookahead distance [cm]"),
    ("cross_track", "cross track error [cm]"),
    ("heading_err", "heading error [deg]"),
]

# 패널 -> (주 계열, 보조 계열 | None). 보조는 주황으로 겹쳐 그린다.
PANEL_SERIES = {
    "speed": ("speed_target", "speed_real"),
    "steer": ("steer_raw", "steer_cmd"),
    "lookahead": ("lookahead", None),
    "cross_track": ("cross_track", None),
    "heading_err": ("heading_err", None),
}

COL_A = "#1f77b4"      # full_simulation 의 COL_REF
COL_B = "#ff7f0e"

STALE_S = 1.0
TF_NAMES = ("map", "odom", "base_link")


class Gui(QtWidgets.QWidget):

    def __init__(self, bridge: Bridge):
        super().__init__()
        self.bridge = bridge
        self.paused = False
        self.follow = False
        self.window_s = bridge.history_s
        self._grid_done = False

        root = QtWidgets.QVBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(0)

        self.status = StatusBar(self)
        root.addWidget(self.status, 0)

        self.win = pg.GraphicsLayoutWidget()
        root.addWidget(self.win, 1)

        self._build_plots()
        self._build_map()

        # full_simulation 의 setColumnStretchFactor(0, 95) / (1, 125)
        self.win.ci.layout.setColumnStretchFactor(0, 95)
        self.win.ci.layout.setColumnStretchFactor(1, 125)

        self.setFocusPolicy(QtCore.Qt.StrongFocus)

        hz = max(1.0, bridge.render_hz)
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self._tick)
        self.timer.start(int(round(1000.0 / hz)))

    # ------------------------------------------------------------------
    # 구성
    # ------------------------------------------------------------------

    def _build_plots(self):
        self.panels = viz.stack_plots(self.win, PANELS, col=0, row0=0,
                                      xlabel="t [s]")
        self.curves = {}
        self.notices = {}
        for key, p in self.panels.items():
            a, b = PANEL_SERIES[key]
            self.curves[a] = p.plot([], [], pen=pg.mkPen(COL_A, width=2))
            if b is not None:
                self.curves[b] = p.plot([], [], pen=pg.mkPen(COL_B, width=2))
            # 값이 안 들어올 때 왜 비었는지 화면에 남긴다. 안 그러면
            # GUI 버그로 오해한다.
            note = pg.TextItem("", color="#d62728", anchor=(0.5, 0.5))
            note.setParentItem(p.getViewBox())
            self.notices[key] = note

    def _build_map(self):
        m = viz.map_plot(self.win, row=0, col=1, rowspan=len(PANELS),
                         title="map")
        m.addLegend(offset=(-10, 10), labelTextSize="8pt")
        self.map = m

        self.map_img = viz.map_image_item(m)

        # 뒤에서 앞으로. 겹칠 때 무엇이 위에 오는지가 곧 우선순위다.
        self.scan_item = m.plot(
            [], [], pen=None, symbol="o", symbolSize=2,
            symbolBrush=viz.COL_SCAN, symbolPen=None, name="scan")
        self.global_item = m.plot(
            [], [], pen=pg.mkPen(viz.COL_GLOBAL, width=2), name="global path")
        self.lane_item = m.plot(
            [], [], pen=pg.mkPen(viz.COL_LANE, width=3), name="lane center")
        self.local_item = m.plot(
            [], [], pen=pg.mkPen(viz.COL_LOCAL, width=3), name="local path")

        self.obstacles = viz.Obstacles(m, color=viz.COL_OBS)
        m.plot([], [], pen=None, symbol="o", symbolSize=7,
               symbolBrush=viz.COL_OBS, symbolPen=None, name="obstacle")

        self.tf = viz.TfChain(m, TF_NAMES, length_cm=self.bridge.tf_axis_cm)

        self.car = m.plot([], [], pen=pg.mkPen(viz.COL_CAR, width=2),
                          name="vehicle")

        self.hud = viz.hud(m)

    # ------------------------------------------------------------------
    # 갱신
    # ------------------------------------------------------------------

    def _tick(self):
        # 일시정지는 표시만 멈춘다. 수신은 계속되므로 재개하면 그동안의
        # 이력이 그대로 남아 있다.
        if self.paused:
            return

        s = self.bridge.snapshot()
        now = s["now"]

        self._update_status(s)
        self._update_plots(s, now)
        self._update_map(s, now)
        self._update_hud(s, now)

    def _update_status(self, s):
        nodes = s["nodes"]
        banner = ""
        if nodes and all(h == "absent" for _n, h, _z in nodes):
            banner = ("연결 끊김 · 감시 대상 노드가 하나도 보이지 않는다 "
                      "(ROS_DOMAIN_ID / zenoh 라우터 확인)")
        elif s["tf_poses"][2] is None:
            banner = "측위 없음 · map -> base_link TF 미수신"
        self.status.set_state(nodes, banner)

    def _update_plots(self, s, now):
        t0 = now - self.window_s
        for name, curve in self.curves.items():
            t, v = s["series"][name]
            curve.setData(t, v)
        # 패널들이 setXLink 로 묶여 있으므로 하나만 잡으면 전부 따라온다.
        next(iter(self.panels.values())).setXRange(t0, now, padding=0.0)

        # /debug/steer 가 없으면 네 패널이 통째로 빈다. 이유를 적어 둔다.
        if not s["steer_debug_alive"]:
            msg = "/debug/steer 미수신 · steer_controller debug.enabled 확인"
        elif not s["tracking_ok"]:
            msg = "추종 불가 · 조향 0 발행 중"
        else:
            msg = ""
        for key, note in self.notices.items():
            text = msg if key != "speed" else ""
            note.setText(text)
            vb = self.panels[key].getViewBox()
            r = vb.boundingRect()
            note.setPos(r.width() * 0.5, r.height() * 0.5)

    def _update_map(self, s, now):
        grid, gstamp = s["grid"]
        if grid is not None and gstamp is not None and not self._grid_done:
            viz.set_map_image(self.map_img, *grid)
            self._grid_done = True
            self.map.autoRange()

        def fresh(stamp, timeout=STALE_S):
            return stamp is not None and (now - stamp) <= timeout

        xy, st = s["scan"]
        self.scan_item.setData(*(xy if fresh(st) else (np.empty(0),
                                                       np.empty(0))))

        # 전역 경로는 latched 라 한 번만 온다. 다른 것과 같은 잣대로 재면 안 된다.
        xy, st = s["path_global"]
        self.global_item.setData(*(xy if xy is not None else (np.empty(0),
                                                              np.empty(0))))

        for key, item in (("path_lane", self.lane_item),
                          ("path_local", self.local_item)):
            xy, st = s[key]
            item.setData(*(xy if (xy is not None and fresh(st))
                           else (np.empty(0), np.empty(0))))

        obs, st = s["obstacles"]
        self.obstacles.update(obs if (obs is not None and fresh(st)) else [])

        self.tf.update(s["tf_poses"])

        base = s["tf_poses"][2]
        if base is None:
            self.car.setData([], [])
        else:
            self.car.setData(*viz.car_shape(base[0], base[1], base[2]))
            if self.follow:
                self._center_on(base[0], base[1])

    def _update_hud(self, s, now):
        def age_txt(stamp):
            return "  --  " if stamp is None else f"{now - stamp:5.1f}s"

        def mark(ok):
            return "O" if ok else "X"

        def path_line(label, key, timeout=STALE_S):
            xy, st = s[key]
            if xy is None:
                return f"{mark(False)} {label:<12}      미수신"
            ok = st is not None and (now - st) <= timeout
            return (f"{mark(ok)} {label:<12} {len(xy[0]):4d} pt "
                    f"{age_txt(st)}")

        obs, ost = s["obstacles"]
        if obs is None:
            obs_line = f"{mark(False)} {'obstacles':<12}      미수신"
        else:
            ok = ost is not None and (now - ost) <= STALE_S
            obs_line = (f"{mark(ok)} {'obstacles':<12} {len(obs):4d} EA "
                        f"{age_txt(ost)}")

        ages = s["tf_ages"]
        tol = self.bridge.tf_timeout

        def tf_line(label, age):
            if age is None:
                return f"{mark(False)} {label:<12}      미수신"
            return f"{mark(age <= tol)} {label:<12}      {age:5.1f}s"

        self.hud.setText("\n".join([
            path_line("global path", "path_global", timeout=1e9),
            path_line("local path", "path_local"),
            path_line("lane center", "path_lane"),
            tf_line("tf map>odom", ages[1]),
            tf_line("tf odom>base", ages[2]),
            obs_line,
        ]))

    # ------------------------------------------------------------------
    # 조작
    # ------------------------------------------------------------------

    def _center_on(self, x, y):
        vb = self.map.getViewBox()
        r = vb.viewRect()
        vb.setRange(xRange=(x - r.width() * 0.5, x + r.width() * 0.5),
                    yRange=(y - r.height() * 0.5, y + r.height() * 0.5),
                    padding=0.0)

    def keyPressEvent(self, ev):
        k = ev.key()
        if k == QtCore.Qt.Key_Space:
            self.paused = not self.paused
            self.setWindowTitle(
                "KAU AMET GUI — 일시정지" if self.paused else "KAU AMET GUI")
        elif k == QtCore.Qt.Key_R:
            self.follow = False
            self.map.autoRange()
        elif k == QtCore.Qt.Key_F:
            self.follow = not self.follow
        else:
            super().keyPressEvent(ev)


def main(args=None):
    rclpy.init(args=args)

    viz.init("KAU AMET GUI")

    try:
        bridge = Bridge()
    except Exception as e:
        # 설정 오류(watch 배열 길이 불일치 등)는 여기서 걸린다. 창을 띄우고
        # 나서 죽는 것보다 이유를 남기고 즉시 끝내는 편이 낫다.
        print(f"[kau_gui] 기동 실패: {e}")
        rclpy.shutdown()
        return 1

    stop = threading.Event()

    def spin():
        while rclpy.ok() and not stop.is_set():
            rclpy.spin_once(bridge, timeout_sec=0.05)

    thread = threading.Thread(target=spin, daemon=True)
    thread.start()

    gui = Gui(bridge)
    try:
        viz.run(gui, "KAU AMET GUI")
    finally:
        stop.set()
        thread.join(timeout=2.0)
        bridge.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
