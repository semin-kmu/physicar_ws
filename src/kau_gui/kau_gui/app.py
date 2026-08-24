"""주행 디버깅 GUI. 관측 전용.

화면 구성은 KAU_AMET_Test/src/full_simulation.py 의 show_realtime 을 그대로
따른다. 시뮬과 실차를 나란히 놓고 보므로 같은 자리에 같은 것이 있어야 한다.

    row 0..4     왼쪽 지표 패널 5 개 / 오른쪽 map (rowspan 5)
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
#
# steer 는 /steering 을 그대로 그린다 -- 차량에 실제로 나간 값이다.
# 예전에는 clamp 전 원출력(SteerDebug.raw_steer_deg)을 같이 겹쳐 그렸는데,
# 실제로 나가지 않은 수치가 나란히 보여 어느 쪽이 명령인지 헷갈렸다.
# 조향 포화는 선이 +-max_steer_deg(20) 에 붙는 것으로 그대로 보인다.
PANELS = [
    ("speed_target", "speed [m/s]"),
    ("steer_cmd", "steer [deg]"),
    ("cross_track", "CTE [cm]"),
    ("heading_err", "\u03b8_err [deg]"),
]

COL_REF, COL_EMA = "#1f77b4", "#2ca02c"

# 같은 패널에 겹쳐 그리는 보조 계열. key -> (보조 key, 색, 점선 여부)
OVERLAY = {
    "speed_target": ("speed_real", COL_EMA, False),
}

STALE_S = 1.0

# 선 굵기. 겹쳐 그리는 것이 많아 얇게 둔다 -- 굵으면 경로 세 개가
# 서로를 가려 어느 것이 위에 있는지 안 보인다.
W_PLOT = 1.5        # 좌측 지표 그래프
W_PATH = 1.2        # 전역 경로 (배경)
W_PATH_HI = 2.0     # lane · local (주목 대상)

EMPTY = (np.empty(0), np.empty(0))


def _run(bridge: Bridge, fps: float) -> None:
    viz.init("KAU AMET GUI")
    win = pg.GraphicsLayoutWidget()

    panels = viz.stack_plots(win, PANELS, col=0, row0=0, xlabel="t [s]")
    curves = {}
    for key, p in panels.items():
        if key in OVERLAY:
            alt, col, dashed = OVERLAY[key]
            style = QtCore.Qt.DotLine if dashed else QtCore.Qt.SolidLine
            curves[alt] = p.plot([], [], pen=pg.mkPen(col, width=W_PLOT,
                                                      style=style))
        curves[key] = p.plot([], [], pen=pg.mkPen(COL_REF, width=W_PLOT))

    m = viz.map_plot(win, row=0, col=1, rowspan=len(PANELS), title="map")

    if bridge.map_yaml:
        try:
            viz.draw_map_image(m, *viz.load_map(bridge.map_yaml))
        except Exception as e:
            # 배경이 없다고 GUI 를 죽이지 않는다. 나머지는 그대로 보인다.
            print(f"[kau_gui] 맵 배경 로드 실패 ({bridge.map_yaml}): {e}")

    scan = viz.ScanCloud(m, size=bridge.scan_size,
                         near_m=bridge.scan_near_m, far_m=bridge.scan_far_m)
    global_line = m.plot([], [], pen=pg.mkPen(viz.COL_GLOBAL, width=W_PATH))
    lane_line = m.plot([], [], pen=pg.mkPen(viz.COL_LANE, width=W_PATH_HI))
    local_line = m.plot([], [], pen=pg.mkPen(viz.COL_LOCAL, width=W_PATH_HI))
    obstacles = viz.Obstacles(m, color=viz.COL_OBS)
    tf = viz.TfChain(m, ("map", "odom", "base_link"),
                     length_cm=bridge.tf_axis_cm)
    body = m.plot([], [], pen=pg.mkPen(viz.COL_CAR, width=W_PATH_HI))
    m.disableAutoRange()

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

        pts, st = s["scan"]
        if pts is not None and fresh(st):
            scan.update(*pts)
        else:
            scan.clear()

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
