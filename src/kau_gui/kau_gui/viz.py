"""PyQtGraph 표시 helper.

KAU_AMET_Test/src/sim_common/viz.py 의 화면 규약을 실차용으로 이식한 것이다.
시뮬과 실차 GUI 를 나란히 놓고 보는 일이 많으므로 **같은 색이 같은 것을
가리켜야 한다.** 색 상수와 함수 이름을 원본 그대로 둔 이유다.

빠진 것: 참값 세계(draw_track · 차선 경계 · 참 차선중앙) 는 시뮬 전용이라
실차엔 없다. 대신 점유격자 맵과 TF 표시가 들어간다.

단위: ROS 는 m 로 주고 화면은 cm 로 그린다 (팀 규약 · 시뮬과 동일).
      변환은 이 module 의 경계에서만 한다.
"""

from __future__ import annotations

import math
import pathlib

import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui

CM_PER_M = 100.0

# --- 공통 색 (sim_common/viz.py 와 동일) ---
COL_TRUTH = "#8c8c8c"
COL_BOUND = "#c8c8c8"
COL_GLOBAL = "#1f77b4"     # global path
COL_LANE = "#2ca02c"       # lane detection
COL_LOCAL = "#d62728"      # local path
COL_CAR = "#d62728"
COL_TRACE = "#9467bd"
COL_KNOT = "#111111"
COL_CTRL = "#bbbbbb"

# --- 실차 전용 ---
COL_SCAN = "#8c8c8c"       # LiDAR 점군 단색 (거리색을 끌 때만 쓴다)
COL_OBS = "#ff7f0e"        # object detection. 주황

# TF 축 색은 ROS 표준을 따른다. x=빨강 y=초록 (z 는 2D 라 생략)
COL_TF_X = "#e04040"
COL_TF_Y = "#40b040"

# 차체 solid 280 x 200 mm. 후륜축 기준 [-rear_overhang, +body_front]
CAR_REAR = -5.0
CAR_FRONT = 23.0
CAR_HALF_W = 10.0


def init(title: str = "KAU AMET GUI"):
    # antialias 가 렌더 비용의 거의 전부다 (측정 1610 -> 80 ms/frame).
    # pyqtgraph 는 aa 를 켜면 곡선을 drawLines 가 아니라 drawPath 로 그린다
    # (PlotCurveItem._shouldUseDrawLineSegments). 같은 조건에 선 굵기 > 1.0 도
    # 있으므로 W_PLOT · W_PATH 를 1.0 이하로 내리지 말 것.
    pg.setConfigOptions(antialias=False, background="w", foreground="k")
    return pg.mkQApp(title)


def map_plot(win, row: int = 0, col: int = 0, rowspan: int = 1,
             colspan: int = 1, title: str = "map"):
    """맵 전체를 담는 등축 plot."""
    p = win.addPlot(row=row, col=col, rowspan=rowspan, colspan=colspan)
    p.showGrid(x=True, y=True, alpha=0.25)
    p.setLabel("bottom", "x [cm]")
    p.setLabel("left", "y [cm]")
    p.setTitle(title)
    p.setAspectLocked(True)
    return p


class SecondAxis(pg.AxisItem):
    """시간축을 "12초" 처럼 읽는다.

    기본 AxisItem 은 눈금값의 자릿수를 축 범위가 아니라 절대값으로 정한다.
    GUI 가 벽시계로 돌면 t 가 1.78e9 라 30 초 창을 구분하려고 소수점 8 자리를
    찍는다. bridge 가 시각을 기동 시점 기준 상대초로 주므로(Bridge._now)
    값 자체는 이미 작지만, 단위를 붙여 두면 축이 무엇인지 바로 보인다.
    """

    def tickStrings(self, values, scale, spacing):
        if spacing >= 1.0:
            return [f"{v:.0f}초" for v in values]
        digits = max(0, int(math.ceil(-math.log10(spacing))))
        return [f"{v:.{digits}f}초" for v in values]


def stack_plots(win, panels, col: int = 0, row0: int = 0,
                xlabel: str = "t"):
    """(key, ylabel) 목록 -> 세로로 쌓인 plot dict. x 축 연동.

    x 눈금 값은 맨 아래 하나만 보인다. 전 plot 이 같은 시간창을 쓰므로
    축을 반복할 이유가 없다.
    """
    out, first = {}, None
    for i, (key, ylabel) in enumerate(panels):
        p = win.addPlot(row=row0 + i, col=col,
                        axisItems={"bottom": SecondAxis(orientation="bottom")})
        p.showGrid(x=True, y=True, alpha=0.3)
        p.setLabel("left", ylabel)
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


def car_shape(x: float, y: float, yaw: float):
    """차체 외곽 폴리라인 [cm]. x, y 도 cm."""
    pts = np.array([[CAR_REAR, -CAR_HALF_W], [CAR_FRONT, -CAR_HALF_W],
                    [CAR_FRONT, CAR_HALF_W], [CAR_REAR, CAR_HALF_W],
                    [CAR_REAR, -CAR_HALF_W]])
    c, s = math.cos(yaw), math.sin(yaw)
    return (x + pts[:, 0] * c - pts[:, 1] * s,
            y + pts[:, 0] * s + pts[:, 1] * c)


# --------------------------------------------------------------------------
# TF 표시 (ROS 표준 축 마커)
# --------------------------------------------------------------------------

class TfAxes:
    """좌표계 하나를 ROS 표준 모양으로 그린다.

    rviz 의 TF display 와 같은 규약: x 축 빨강 / y 축 초록 + 프레임 이름.
    2D 탑뷰라 z 축(파랑)은 그리지 않는다.
    """

    def __init__(self, plot, name: str, length_cm: float = 20.0,
                 width: int = 2):
        self.name = name
        self.length = length_cm
        self.x_line = plot.plot([], [], pen=pg.mkPen(COL_TF_X, width=width))
        self.y_line = plot.plot([], [], pen=pg.mkPen(COL_TF_Y, width=width))
        self.label = pg.TextItem(name, color="#404040", anchor=(0, 1))
        self.label.setFont(QtGui.QFont("monospace", 8))
        plot.addItem(self.label, ignoreBounds=True)
        self.hide()

    def set_pose(self, x: float, y: float, yaw: float):
        """cm / rad."""
        c, s = math.cos(yaw), math.sin(yaw)
        n = self.length
        self.x_line.setData([x, x + n * c], [y, y + n * s])
        self.y_line.setData([x, x - n * s], [y, y + n * c])
        self.label.setPos(x + 4.0, y + 4.0)
        self.label.setVisible(True)

    def hide(self):
        self.x_line.setData([], [])
        self.y_line.setData([], [])
        self.label.setVisible(False)


class TfChain:
    """map -> odom -> base_link 를 잇는 축 마커 묶음 + 연결선."""

    def __init__(self, plot, names, length_cm: float = 20.0):
        # 부모->자식 연결선. 체인이 어떻게 이어졌는지 눈으로 보이게 한다.
        self.link = plot.plot(
            [], [], pen=pg.mkPen("#b0b0b0", width=1,
                                 style=QtCore.Qt.DashLine))
        self.axes = [TfAxes(plot, n, length_cm) for n in names]

    def update(self, poses):
        """poses: [(x_cm, y_cm, yaw) | None] · names 와 같은 길이."""
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
# 만들면 720 점 x 10 Hz = 초당 7200 개가 생겼다 사라진다. 64 단계면 12 m
# 범위에서 한 칸이 19 cm 라 눈으로는 연속으로 보인다.
SCAN_LUT_N = 64


def rainbow_rgb(t: float):
    """0..1 -> (r, g, b) 0..255. rviz 의 getRainbowColor 와 같은 식이다.

    보라(가까움) -> 파랑 -> 청록 -> 초록 -> 노랑 -> 빨강(멀리). rviz 에서
    PointCloud2 를 rainbow 로 놓고 본 것과 같은 색이 같은 거리를 가리키도록
    구현을 그대로 옮겼다 (rviz_common/src/rviz_common/properties/color_map.cpp).
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
    """LiDAR 점군을 센서로부터의 거리로 색칠해 그린다.

    색 기준은 **화면 좌표가 아니라 측정 거리(range)** 다. 차가 움직여도 같은
    거리의 벽은 같은 색으로 남아, 어느 반사가 가까운 것인지 한눈에 보인다.

    거리 범위는 고정이다. rviz 는 기본이 매 프레임 min/max 자동인데, 그러면
    차가 벽에 다가갈 때마다 온 화면 색이 같이 흔들려 거리를 못 읽는다.
    """

    def __init__(self, plot, size: float = 3.0,
                 near_m: float = 0.0, far_m: float = 12.0):
        self.near = float(near_m)
        self.far = float(far_m)
        self._span = max(self.far - self.near, 1e-6)
        self._lut = [pg.mkBrush(*rainbow_rgb(i / (SCAN_LUT_N - 1)))
                     for i in range(SCAN_LUT_N)]
        self._item = pg.ScatterPlotItem(pen=None, size=size, pxMode=True)
        plot.addItem(self._item)

    def update(self, xs, ys, dists):
        """xs·ys 는 화면 cm, dists 는 센서로부터의 거리 m."""
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
# 장애물 (object detection)
# --------------------------------------------------------------------------

class Obstacles:
    """원판 묶음. 개수와 반지름이 매 프레임 바뀌므로 item 을 재사용한다.

    반지름은 발행된 값을 그대로 쓴다 (크기 동적 반영). 안전 margin 은
    Local Planner 소관이라 여기서 부풀리지 않는다.
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
        """circles: [(x_cm, y_cm, r_cm)]."""
        self._grow(len(circles))
        for it, (x, y, r) in zip(self._items, circles):
            it.setRect(x - r, y - r, 2.0 * r, 2.0 * r)
            it.setVisible(True)
        for it in self._items[len(circles):]:
            it.setVisible(False)


# --------------------------------------------------------------------------
# 맵 배경 (.pgm + .yaml)
# --------------------------------------------------------------------------

def load_map(yaml_path: str):
    """map_server 형식의 yaml + pgm 을 읽는다. (img, res_m, ox_m, oy_m).

    P5 (binary graymap) 만 다룬다. map_server 가 내는 것이 그 형식이다.
    yaml 의 image 경로는 지도를 만든 PC 기준 절대경로인 경우가 많아,
    없으면 yaml 옆에서 같은 파일명을 찾는다.
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
    res = res_m * CM_PER_M
    x0, y0 = ox_m * CM_PER_M, oy_m * CM_PER_M

    it = pg.ImageItem()
    it.setZValue(-100)
    # pgm row 0 은 월드 y 최대다. ImageItem 은 y 가 위로 증가하므로 뒤집는다.
    it.setImage(np.flipud(img).T, levels=(0, 255))
    it.setRect(QtCore.QRectF(x0, y0, w * res, h * res))
    # 배경은 안 바뀌는데 매 프레임 다시 확대돼 그려진다 (측정 20 ms/frame).
    # 캐시해 두면 뷰 변환이 바뀔 때만 다시 만든다.
    it.setCacheMode(pg.QtWidgets.QGraphicsItem.DeviceCoordinateCache)
    plot.addItem(it)

    # 맵 경계 (viz.draw_map_border 와 같은 규약)
    plot.plot([x0, x0 + w * res, x0 + w * res, x0, x0],
              [y0, y0, y0 + h * res, y0 + h * res, y0],
              pen=pg.mkPen("#000000", width=1))

    m = 20.0
    plot.setXRange(x0 - m, x0 + w * res + m, padding=0.0)
    plot.setYRange(y0 - m, y0 + h * res + m, padding=0.0)
    return it


# 창 크기. 페인트 비용이 픽셀 수를 따라간다 (1600x950 -> 1280x720 에서 -13 %).
def run(widget, title: str, size=(1280, 720)) -> None:
    widget.setWindowTitle(title)
    widget.resize(*size)
    widget.show()
    pg.exec()
