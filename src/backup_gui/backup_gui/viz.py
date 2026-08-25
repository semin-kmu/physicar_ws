"""PyQtGraph 표시 helper. 백업 스택 전용.

단위는 **m** 이다. 백업 스택은 전 토픽이 `base_link` 상대 m 라 화면도 m 로
그린다. kau_gui 는 cm 였는데, 변환을 두면 화면에서 읽은 값과 yaml 에 적는
값이 달라져 튜닝 때마다 100 을 곱해야 한다.

차량은 항상 원점이다 (측위 없음). map plot 은 차량 고정 탑뷰다.

색 규약
    채도 높은 색  경로 = 판단 결과 (source 별로 다름)
    회청색        차선 기하 = 근거
    주황          장애물
    **빨강은 kappa_saturated / stop_request 전용.** 다른 것에 쓰지 않는다.
"""

from __future__ import annotations

import math
import pathlib

import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui

# --- 경로 (BackupPath.source) ---
SRC_COLOR = {0: "#2ca02c", 1: "#1f77b4", 2: "#9467bd", 3: "#b8860b"}
SRC_NAME = {0: "LANE_CENTER", 1: "RACE_LINE", 2: "AVOID", 3: "DEGRADED"}
COL_PATH_UNK = "#606060"

# 통과 불가 · 정지 요구. 이 화면에서 빨강은 이 둘뿐이다.
COL_ALERT = "#d62728"

# --- 차선 기하 (LaneGeometry) ---
COL_LANE_SIDE = "#7d94ab"
COL_LANE_MID = "#4f6577"
COL_INFL = "#111111"

# --- 그 외 ---
COL_TRUTH = "#8c8c8c"
COL_KNOT = "#111111"
COL_CAR = "#d62728"
COL_OBS = "#ff7f0e"

# TF 축 색은 ROS 표준. x=빨강 y=초록 (z 는 2D 라 생략)
COL_TF_X = "#e04040"
COL_TF_Y = "#40b040"

# 차체 solid 280 x 200 mm. 후륜축 기준 [-rear_overhang, +body_front] [m]
CAR_REAR = -0.05
CAR_FRONT = 0.23
CAR_HALF_W = 0.10


def init(title: str = "backup_gui"):
    # antialias 가 렌더 비용의 거의 전부다 (측정 1610 -> 80 ms/frame).
    # pyqtgraph 는 aa 를 켜면 곡선을 drawLines 가 아니라 drawPath 로 그린다
    # (PlotCurveItem._shouldUseDrawLineSegments). 같은 조건에 선 굵기 > 1.0 도
    # 있으므로 선 굵기를 1.0 이하로 내리지 말 것.
    pg.setConfigOptions(antialias=False, background="w", foreground="k")
    return pg.mkQApp(title)


def map_plot(win, row: int = 0, col: int = 0, rowspan: int = 1,
             colspan: int = 1, title: str = "map (base_link)"):
    """등축 탑뷰. 원점 = 후륜축."""
    p = win.addPlot(row=row, col=col, rowspan=rowspan, colspan=colspan)
    p.showGrid(x=True, y=True, alpha=0.25)
    p.setLabel("bottom", "x [m]")
    p.setLabel("left", "y [m]")
    p.setTitle(title)
    p.setAspectLocked(True)
    return p


def vehicle_view(plot, far_m: float, back_m: float = 0.6):
    """차량 고정 뷰 범위. 전방은 센서 유효거리, 후방은 차체가 보일 만큼."""
    plot.setXRange(-back_m, far_m, padding=0.0)
    plot.setYRange(-far_m * 0.5, far_m * 0.5, padding=0.0)


class SecondAxis(pg.AxisItem):
    """시간축을 "12초" 처럼 읽는다.

    기본 AxisItem 은 눈금값의 자릿수를 축 범위가 아니라 절대값으로 정한다.
    bridge 가 시각을 기동 시점 기준 상대초로 주므로(Bridge._now) 값 자체는
    이미 작지만, 단위를 붙여 두면 축이 무엇인지 바로 보인다.
    """

    def tickStrings(self, values, scale, spacing):
        if spacing >= 1.0:
            return [f"{v:.0f}초" for v in values]
        digits = max(0, int(math.ceil(-math.log10(spacing))))
        return [f"{v:.{digits}f}초" for v in values]


def stack_plots(win, panels, col: int = 0, row0: int = 0,
                xlabel: str = "t"):
    """(key, ylabel) 목록 -> 세로로 쌓인 plot dict. x 축 연동.

    x 눈금 값은 맨 아래 하나만 보인다. 전 plot 이 같은 시간창을 쓴다.
    """
    out, first = {}, None
    for i, (key, ylabel) in enumerate(panels):
        p = win.addPlot(row=row0 + i, col=col,
                        axisItems={"bottom": SecondAxis(orientation="bottom")})
        p.showGrid(x=True, y=True, alpha=0.3)
        p.setLabel("left", ylabel)
        # SI 접두어 자동 변환을 끈다. 켜 두면 CTE 0.05 m 가 "50 (x0.001)" 로
        # 찍혀 라벨의 [m] 와 어긋나고 축 이름까지 밀어낸다.
        p.getAxis("left").enableAutoSIPrefix(False)
        p.addItem(pg.InfiniteLine(angle=0, pos=0.0,
                                  pen=pg.mkPen("#cccccc", width=1)),
                  ignoreBounds=True)
        if first is None:
            first = p
        else:
            p.setXLink(first)
        if i < len(panels) - 1:
            p.getAxis("bottom").setStyle(showValues=False)
        else:
            p.setLabel("bottom", xlabel)
        out[key] = p
    return out


def limit_line(plot, value: float, color: str = COL_ALERT):
    """넘으면 안 되는 값의 수평선. kappa_max 처럼 판정 기준이 있는 패널용."""
    line = pg.InfiniteLine(angle=0, pos=value,
                           pen=pg.mkPen(color, width=1,
                                        style=QtCore.Qt.DashLine))
    plot.addItem(line, ignoreBounds=True)
    return line


def car_shape(x: float = 0.0, y: float = 0.0, yaw: float = 0.0):
    """차체 외곽 폴리라인 [m]."""
    pts = np.array([[CAR_REAR, -CAR_HALF_W], [CAR_FRONT, -CAR_HALF_W],
                    [CAR_FRONT, CAR_HALF_W], [CAR_REAR, CAR_HALF_W],
                    [CAR_REAR, -CAR_HALF_W]])
    c, s = math.cos(yaw), math.sin(yaw)
    return (x + pts[:, 0] * c - pts[:, 1] * s,
            y + pts[:, 0] * s + pts[:, 1] * c)


# --------------------------------------------------------------------------
# 화면 고정 텍스트
# --------------------------------------------------------------------------

class Overlay:
    """뷰 모서리에 **픽셀로** 고정되는 텍스트.

    ViewBox 의 자식으로 두면 좌표가 데이터가 아니라 픽셀이다. 데이터
    좌표에 두면 3 Hz 렌더에서 팬/줌 할 때 글자가 화면을 늦게 따라온다.

    상태 플래그는 플롯이 아니라 글자로 낸다 -- kappa_saturated 처럼
    "지금 켜졌나" 만 보면 되는 신호는 선으로 그리면 오히려 안 보인다.
    """

    FONT = "font-family:monospace; font-size:9pt; line-height:118%"
    MARGIN = 10                                     # px

    def __init__(self, plot, corner: str = "tl"):
        self._vb = plot.getViewBox()
        self._right = corner.endswith("r")
        self._item = pg.TextItem("", anchor=(1.0 if self._right else 0.0, 0.0))
        self._item.setParentItem(self._vb)
        self._item.setZValue(100)
        # 창 크기가 바뀌면 바로 따라간다. 다음 프레임(3 Hz)까지 기다리면
        # 드래그로 창을 늘리는 내내 글자가 엉뚱한 자리에 남는다.
        self._vb.sigResized.connect(self._place)
        self._place()

    def _place(self, *_):
        w = self._vb.boundingRect().width()
        self._item.setPos(w - self.MARGIN if self._right else self.MARGIN,
                          self.MARGIN)

    def set(self, lines):
        """lines: [(텍스트, 색)]. 빈 목록이면 지운다."""
        if not lines:
            self._item.setHtml("")
            return
        body = "<br>".join(
            f'<span style="color:{c}">{t}</span>' for t, c in lines)
        self._item.setHtml(f'<div style="{self.FONT}">{body}</div>')
        self._place()


# --------------------------------------------------------------------------
# TF 표시 (ROS 표준 축 마커)
# --------------------------------------------------------------------------

class TfAxes:
    """좌표계 하나를 rviz TF display 규약으로. x 빨강 / y 초록 + 이름."""

    def __init__(self, plot, name: str, length_m: float = 0.2,
                 width: int = 2):
        self.name = name
        self.length = length_m
        self.x_line = plot.plot([], [], pen=pg.mkPen(COL_TF_X, width=width))
        self.y_line = plot.plot([], [], pen=pg.mkPen(COL_TF_Y, width=width))
        self.label = pg.TextItem(name, color="#404040", anchor=(0, 1))
        self.label.setFont(QtGui.QFont("monospace", 8))
        plot.addItem(self.label, ignoreBounds=True)
        self.hide()

    def set_pose(self, x: float, y: float, yaw: float):
        """m / rad."""
        c, s = math.cos(yaw), math.sin(yaw)
        n = self.length
        self.x_line.setData([x, x + n * c], [y, y + n * s])
        self.y_line.setData([x, x - n * s], [y, y + n * c])
        self.label.setPos(x + 0.04, y + 0.04)
        self.label.setVisible(True)

    def hide(self):
        self.x_line.setData([], [])
        self.y_line.setData([], [])
        self.label.setVisible(False)


class TfChain:
    """축 마커 묶음 + 부모·자식 연결선."""

    def __init__(self, plot, names, length_m: float = 0.2):
        self.link = plot.plot(
            [], [], pen=pg.mkPen("#b0b0b0", width=1,
                                 style=QtCore.Qt.DashLine))
        self.axes = [TfAxes(plot, n, length_m) for n in names]

    def update(self, poses):
        """poses: [(x_m, y_m, yaw) | None] · names 와 같은 길이."""
        xs, ys = [], []
        for ax, pose in zip(self.axes, poses):
            if pose is None:
                ax.hide()
                continue
            ax.set_pose(*pose)
            xs.append(pose[0])
            ys.append(pose[1])
        self.link.setData(xs, ys)


# --------------------------------------------------------------------------
# LiDAR 점군 (거리색)
# --------------------------------------------------------------------------

# 팔레트를 이 개수로 잘라 QBrush 를 미리 만들어 둔다. 점마다 QBrush 를 새로
# 만들면 720 점 x 10 Hz = 초당 7200 개가 생겼다 사라진다.
SCAN_LUT_N = 64


def rainbow_rgb(t: float):
    """0..1 -> (r, g, b) 0..255. rviz 의 getRainbowColor 와 같은 식이다.

    보라(가까움) -> 파랑 -> 청록 -> 초록 -> 노랑 -> 빨강(멀리).
    """
    t = 0.0 if t < 0.0 else (1.0 if t > 1.0 else t)
    h = t * 5.0 + 1.0
    i = int(h)
    f = h - i
    if not (i & 1):                      # 짝수 구간은 진행 방향이 반대다
        f = 1.0 - f
    n = 1.0 - f
    if i <= 1:
        r, g, b = n, 0.0, 1.0
    elif i == 2:
        r, g, b = 0.0, n, 1.0
    elif i == 3:
        r, g, b = 0.0, 1.0, n
    elif i == 4:
        r, g, b = n, 1.0, 0.0
    else:
        r, g, b = 1.0, n, 0.0
    return int(r * 255), int(g * 255), int(b * 255)


class ScanCloud:
    """LiDAR 점군을 **측정 거리(range)** 로 색칠한다.

    화면 좌표가 아니라 센서로부터의 거리를 쓰므로 차가 움직여도 같은 거리의
    벽이 같은 색으로 남는다. 범위는 고정이다 -- 자동이면 벽에 다가갈 때마다
    온 화면 색이 흔들려 거리를 못 읽는다.
    """

    def __init__(self, plot, size: float = 3.0,
                 near_m: float = 0.0, far_m: float = 3.0):
        self.near = float(near_m)
        self.far = float(far_m)
        self._span = max(self.far - self.near, 1e-6)
        self._lut = [pg.mkBrush(*rainbow_rgb(i / (SCAN_LUT_N - 1)))
                     for i in range(SCAN_LUT_N)]
        self._item = pg.ScatterPlotItem(pen=None, size=size, pxMode=True)
        plot.addItem(self._item)

    def update(self, xs, ys, dists):
        """xs·ys 는 화면 m, dists 는 센서로부터의 거리 m."""
        if xs is None or len(xs) == 0:
            self._item.setData([], [])
            return
        idx = np.clip((np.asarray(dists) - self.near) / self._span, 0.0, 1.0)
        idx = (idx * (SCAN_LUT_N - 1)).astype(np.intp)
        lut = self._lut
        self._item.setData(x=xs, y=ys, brush=[lut[i] for i in idx])

    def clear(self):
        self._item.setData([], [])


# --------------------------------------------------------------------------
# 장애물
# --------------------------------------------------------------------------

class Obstacles:
    """원판 묶음. 개수가 매 프레임 바뀌므로 item 을 재사용한다.

    반지름은 발행값 그대로다. 안전 margin 은 플래너 소관이라 부풀리지 않는다.
    """

    def __init__(self, plot, color: str = COL_OBS, alpha: int = 90):
        self._plot = plot
        self._items = []
        col = pg.mkColor(color)
        self._pen = pg.mkPen(color, width=2)
        self._brush = pg.mkBrush(col.red(), col.green(), col.blue(), alpha)

    def _grow(self, n):
        while len(self._items) < n:
            it = pg.QtWidgets.QGraphicsEllipseItem(0, 0, 0, 0)
            it.setPen(self._pen)
            it.setBrush(self._brush)
            self._plot.addItem(it)
            self._items.append(it)

    def update(self, circles):
        """circles: [(x_m, y_m, r_m)]."""
        self._grow(len(circles))
        for it, (x, y, r) in zip(self._items, circles):
            it.setRect(x - r, y - r, 2.0 * r, 2.0 * r)
            it.setVisible(True)
        for it in self._items[len(circles):]:
            it.setVisible(False)


# --------------------------------------------------------------------------
# BackupPath (quintic Bezier)
# --------------------------------------------------------------------------

class BezierPath:
    """경로 하나. bridge 가 샘플링한 폴리라인을 받아 그린다.

    실선 = `valid_length` 안 (근거가 닿는 구간), 점선 = 그 너머 외삽.
    두 구간을 같은 선으로 그리면 어디까지가 관측이고 어디부터가 추정인지
    화면에서 사라진다.
    """

    def __init__(self, plot, width: float = 2.0, width_alert: float = 3.0):
        self.width = width
        self.width_alert = width_alert
        self._pens = {}
        self.solid = plot.plot([], [], pen=pg.mkPen(COL_PATH_UNK,
                                                    width=width))
        self.dashed = plot.plot([], [], pen=pg.mkPen(COL_PATH_UNK, width=width,
                                                     style=QtCore.Qt.DashLine))
        self.knots = pg.ScatterPlotItem(pen=pg.mkPen(COL_KNOT, width=1),
                                        brush=None, symbol="o", size=5)
        self.knots.setZValue(12)
        plot.addItem(self.knots)

    def _pen(self, color, width, dashed):
        key = (color, width, dashed)
        pen = self._pens.get(key)
        if pen is None:
            pen = pg.mkPen(color, width=width,
                           style=(QtCore.Qt.DashLine if dashed
                                  else QtCore.Qt.SolidLine))
            self._pens[key] = pen
        return pen

    def clear(self):
        self.solid.setData([], [])
        self.dashed.setData([], [])
        self.knots.setData(x=[], y=[])

    def update(self, d):
        """d: bridge.snapshot()["path"] 값 또는 None."""
        if not d:
            self.clear()
            return

        # kappa_saturated 는 "조향으로 통과 불가" 다. source 색을 덮어쓴다 --
        # 이 화면에서 트랙 통과 가능 여부를 판정할 유일한 신호다.
        if d["kappa_saturated"]:
            color, width = COL_ALERT, self.width_alert
        else:
            color = SRC_COLOR.get(d["source"], COL_PATH_UNK)
            width = self.width

        xs, ys, k = d["xs"], d["ys"], d["split"]
        self.solid.setPen(self._pen(color, width, False))
        self.dashed.setPen(self._pen(color, width, True))
        # 점선 구간은 실선 끝점부터 시작한다. k 에서 끊으면 한 칸 빈다.
        self.solid.setData(xs[:k + 1], ys[:k + 1])
        self.dashed.setData(xs[k:], ys[k:])
        self.knots.setData(x=d["knot_x"], y=d["knot_y"])


# --------------------------------------------------------------------------
# LaneGeometry
# --------------------------------------------------------------------------

class LaneView:
    """좌/우/중앙 직선 + 변곡점 마커.

    좌우를 같은 색으로 둔다. 어느 쪽인지는 y 부호로 이미 갈리므로 색까지
    나누면 경로 색과 섞여 읽기만 나빠진다.
    """

    def __init__(self, plot, width: float = 2.0):
        pen_side = pg.mkPen(COL_LANE_SIDE, width=width)
        self.left = plot.plot([], [], pen=pen_side)
        self.right = plot.plot([], [], pen=pen_side)
        self.center = plot.plot(
            [], [], pen=pg.mkPen(COL_LANE_MID, width=max(1.0, width - 0.5),
                                 style=QtCore.Qt.DashLine))
        self.infl = pg.ScatterPlotItem(pen=pg.mkPen(COL_INFL, width=2),
                                       brush=None, symbol="x", size=12)
        self.infl.setZValue(6)
        plot.addItem(self.infl)
        self.label = pg.TextItem("", color=COL_INFL, anchor=(0.0, 1.0))
        self.label.setFont(QtGui.QFont("monospace", 8))
        plot.addItem(self.label, ignoreBounds=True)
        self.label.setVisible(False)

    def clear(self):
        for c in (self.left, self.right, self.center):
            c.setData([], [])
        self.infl.setData(x=[], y=[])
        self.label.setVisible(False)

    def update(self, d):
        """d: bridge.snapshot()["lane"] 값 또는 None."""
        if not d:
            self.clear()
            return

        for curve, key in ((self.left, "left"), (self.right, "right"),
                           (self.center, "center")):
            line = d[key]
            curve.setData(*(line if line else ([], [])))

        if d["inflection"] is None:
            self.infl.setData(x=[], y=[])
            self.label.setVisible(False)
            return

        x, y, dpsi_deg = d["inflection"]
        self.infl.setData(x=[x], y=[y])
        self.label.setText(f"Δψ {dpsi_deg:+.1f}°")
        self.label.setPos(x + 0.03, y - 0.02)
        self.label.setVisible(True)


# --------------------------------------------------------------------------
# 참값 트랙 (선택. 백업 스택 기본은 track.enabled=false)
# --------------------------------------------------------------------------

def load_track(yaml_path: str):
    """track yaml -> dict. centerline / corners 는 ndarray 로 만든다."""
    import yaml as _yaml

    try:                                    # C 로더가 있으면 10 배쯤 빠르다
        loader = _yaml.CSafeLoader
    except AttributeError:
        loader = _yaml.SafeLoader

    doc = _yaml.load(pathlib.Path(yaml_path).expanduser().read_text(),
                     Loader=loader)["track"]
    doc["centerline"] = np.asarray(doc["centerline"], dtype=float)
    doc["corners"] = np.asarray(doc["corners"], dtype=float)
    return doc


def draw_track(plot, track, s_tick_m: float = 5.0, labels: bool = True):
    """참값 트랙을 표시물 뒤에 깐다. 정적이라 기동 시 1 회만 그린다."""
    cl, co = track["centerline"], track["corners"]
    xs, ys = cl[:, 1], cl[:, 2]

    line = plot.plot(np.append(xs, xs[0]), np.append(ys, ys[0]),
                     pen=pg.mkPen(COL_TRUTH, width=1,
                                  style=QtCore.Qt.DashLine))
    line.setZValue(-60)

    # 글자는 진행 방향 법선으로 민다. 코너는 +n 쪽, s 눈금은 -n 쪽이라
    # 두 무리가 겹치지 않는다.
    def normal(i):
        th = cl[i, 3]
        return -math.sin(th), math.cos(th)

    def put(i, text, color, off_m, size):
        nx, ny = normal(i)
        t = pg.TextItem(text, color=color, anchor=(0.5, 0.5))
        t.setFont(QtGui.QFont("", size))
        t.setPos(xs[i] + nx * off_m, ys[i] + ny * off_m)
        t.setZValue(-55)
        plot.addItem(t)

    if s_tick_m > 0.0:
        step = max(1, int(round(s_tick_m / float(track["ds_m"]))))
        seg_x, seg_y = [], []
        for i in range(0, len(cl), step):
            nx, ny = normal(i)
            seg_x += [xs[i], xs[i] - nx * 0.08, np.nan]
            seg_y += [ys[i], ys[i] - ny * 0.08, np.nan]
            if labels:
                put(i, f"{cl[i, 0]:.0f}m", COL_TRUTH, -0.15, 6)
        tick = plot.plot(seg_x, seg_y,
                         pen=pg.mkPen(COL_TRUTH, width=1), connect="finite")
        tick.setZValue(-58)

    idx = np.clip((co[:, 0] / float(track["ds_m"])).astype(int), 0, len(cl) - 1)
    marks = pg.ScatterPlotItem(x=xs[idx], y=ys[idx], symbol="t1", size=9,
                               pen=pg.mkPen(COL_KNOT, width=1),
                               brush=pg.mkBrush(COL_KNOT))
    marks.setZValue(-55)
    plot.addItem(marks)

    if labels:
        for i, j in enumerate(idx):
            put(j, f"{i} {co[i, 1]:+.0f}°", COL_KNOT, 0.16, 7)

    return line


# --------------------------------------------------------------------------
# 맵 배경 (.pgm + .yaml). 백업 스택 기본은 map.package/name 이 빈 문자열이다
# --------------------------------------------------------------------------

def load_map(yaml_path: str):
    """map_server 형식의 yaml + pgm 을 읽는다. (img, res_m, ox_m, oy_m).

    P5 (binary graymap) 만 다룬다. yaml 의 image 경로는 지도를 만든 PC 기준
    절대경로인 경우가 많아, 없으면 yaml 옆에서 같은 파일명을 찾는다.
    """
    yp = pathlib.Path(yaml_path).expanduser()
    image, res, ox, oy = "", 0.05, 0.0, 0.0
    for line in yp.read_text().splitlines():
        k, _, v = line.partition(":")
        k, v = k.strip(), v.strip().strip("\"'")
        if k == "image":
            image = v
        elif k == "resolution":
            res = float(v)
        elif k == "origin":
            nums = v.strip("[]").split(",")
            ox, oy = float(nums[0]), float(nums[1])

    ip = pathlib.Path(image).expanduser()
    if not ip.is_file():
        ip = yp.parent / ip.name

    return _read_pgm(ip), res, ox, oy


def _read_pgm(path):
    """P5 PGM -> (h, w) uint8. row 0 이 이미지 위쪽 = 월드 y 최대."""
    data = pathlib.Path(path).read_bytes()

    fields, pos = [], 0
    while len(fields) < 4:
        while pos < len(data) and data[pos:pos + 1].isspace():
            pos += 1
        if data[pos:pos + 1] == b"#":                  # 주석 줄 통째로 건너뜀
            while pos < len(data) and data[pos:pos + 1] not in b"\r\n":
                pos += 1
            continue
        start = pos
        while pos < len(data) and not data[pos:pos + 1].isspace():
            pos += 1
        fields.append(data[start:pos])
    pos += 1                                            # 헤더 뒤 공백 1 개

    if fields[0] != b"P5":
        raise ValueError(f"P5 가 아니다: {fields[0]!r}")
    w, h = int(fields[1]), int(fields[2])

    return np.frombuffer(data, dtype=np.uint8, count=w * h,
                         offset=pos).reshape(h, w)


def draw_map_image(plot, img, res_m: float, ox_m: float, oy_m: float):
    """맵을 표시물 뒤에 깔고, 뷰 범위를 맵 전체에 맞춘다."""
    h, w = img.shape
    x0, y0 = ox_m, oy_m

    it = pg.ImageItem()
    it.setZValue(-100)
    # pgm row 0 은 월드 y 최대다. ImageItem 은 y 가 위로 증가하므로 뒤집는다.
    it.setImage(np.flipud(img).T, levels=(0, 255))
    it.setRect(QtCore.QRectF(x0, y0, w * res_m, h * res_m))
    # 배경은 안 바뀌는데 매 프레임 다시 확대돼 그려진다 (측정 20 ms/frame).
    it.setCacheMode(pg.QtWidgets.QGraphicsItem.DeviceCoordinateCache)
    plot.addItem(it)

    plot.plot([x0, x0 + w * res_m, x0 + w * res_m, x0, x0],
              [y0, y0, y0 + h * res_m, y0 + h * res_m, y0],
              pen=pg.mkPen("#000000", width=1))

    m = 0.2
    plot.setXRange(x0 - m, x0 + w * res_m + m, padding=0.0)
    plot.setYRange(y0 - m, y0 + h * res_m + m, padding=0.0)
    return it


# 창 크기. 페인트 비용이 픽셀 수를 따라간다 (1600x950 -> 1280x720 에서 -13 %).
def run(widget, title: str, size=(1280, 720)) -> None:
    widget.setWindowTitle(title)
    widget.resize(*size)
    widget.show()
    pg.exec()
