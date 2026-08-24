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
COL_SCAN = "#8c8c8c"       # LiDAR 점군. 배경으로 물러나야 하므로 회색
COL_OBS = "#ff7f0e"        # object detection. 주황

# TF 축 색은 ROS 표준을 따른다. x=빨강 y=초록 (z 는 2D 라 생략)
COL_TF_X = "#e04040"
COL_TF_Y = "#40b040"

# 차체 solid 280 x 200 mm. 후륜축 기준 [-rear_overhang, +body_front]
CAR_REAR = -5.0
CAR_FRONT = 23.0
CAR_HALF_W = 10.0


def init(title: str = "KAU AMET GUI"):
    pg.setConfigOptions(antialias=True, background="w", foreground="k")
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


def stack_plots(win, panels, col: int = 0, row0: int = 0,
                xlabel: str = "t [s]"):
    """(key, ylabel) 목록 -> 세로로 쌓인 plot dict. x 축 연동.

    x 눈금 값은 맨 아래 하나만 보인다. 전 plot 이 같은 시간창을 쓰므로
    축을 반복할 이유가 없다.
    """
    out, first = {}, None
    for i, (key, ylabel) in enumerate(panels):
        p = win.addPlot(row=row0 + i, col=col)
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


def hud(plot, x: int = 8, y: int = 8) -> pg.TextItem:
    """ViewBox 픽셀 좌표에 고정되는 텍스트 상자."""
    t = pg.TextItem(anchor=(0, 0), color="k",
                    fill=pg.mkBrush(255, 255, 255, 220),
                    border=pg.mkPen("#bbbbbb"))
    t.textItem.setFont(QtGui.QFont("monospace", 9))
    t.setParentItem(plot.getViewBox())
    t.setPos(x, y)
    return t


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
# 점유격자 맵 배경
# --------------------------------------------------------------------------

def map_image_item(plot):
    """맵을 깔 ImageItem. 표시물보다 뒤에 있어야 한다."""
    img = pg.ImageItem()
    img.setZValue(-100)
    plot.addItem(img)
    return img


def set_map_image(item, grid, resolution_m, origin_x_m, origin_y_m):
    """OccupancyGrid 값 배열(-1/0..100) 을 회색조로 깔아 놓는다.

    grid 는 (h, w) numpy. row 0 이 월드 y 최소 (ROS 규약) 라
    pyqtgraph 의 축 방향과 그대로 맞는다.
    """
    g = np.asarray(grid, dtype=np.int16)
    shade = np.where(g < 0, 128, 255 - (g.astype(np.int32) * 255) // 100)
    item.setImage(shade.astype(np.uint8).T, levels=(0, 255))
    res_cm = resolution_m * CM_PER_M
    item.setRect(QtCore.QRectF(
        origin_x_m * CM_PER_M, origin_y_m * CM_PER_M,
        g.shape[1] * res_cm, g.shape[0] * res_cm))


def run(widget, title: str, size=(1600, 950)) -> None:
    widget.setWindowTitle(title)
    widget.resize(*size)
    widget.show()
    pg.exec()
