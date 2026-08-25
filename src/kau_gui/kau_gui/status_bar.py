"""최상단 노드 상태 띠. 좌우 분할과 무관하게 가로 전체를 쓴다.

셀 하나 = 노드 하나. 좌측부터 이름 · 상태등 · Hz 한 줄이다.

    steer_controller  ●  49.7Hz
    map_server        ●  -

Hz 칸의 "-" 는 **잴 대표 토픽이 없다** 는 뜻이다 (status.watch 에 없는 노드).
측정에 실패한 것이 아니다 -- 그쪽은 상태등이 빨강으로 간다.

색
    초록  정상 (그래프에 있고 대표 토픽이 기대 주기대로 온다)
    빨강  노드는 살아 있는데 토픽이 끊겼다
    회색  그래프에 없다 (안 떴거나 죽었다)

글자는 작다. 이 띠는 훑어보는 것이지 읽는 것이 아니다 -- 눈에 걸려야 하는
것은 빨강·회색 램프고, 이름과 Hz 는 그 램프가 무엇인지 확인할 때만 본다.
"""

from __future__ import annotations

from pyqtgraph.Qt import QtCore, QtGui, QtWidgets

COL_OK = QtGui.QColor("#2ca02c")
COL_STALE = QtGui.QColor("#d62728")
COL_ABSENT = QtGui.QColor("#b0b0b0")
COL_TEXT = QtGui.QColor("#141414")
COL_DIM = QtGui.QColor("#787878")
COL_BG = QtGui.QColor("#ffffff")
COL_BORDER = QtGui.QColor("#cccccc")

LAMP = {"ok": COL_OK, "stale": COL_STALE, "absent": COL_ABSENT}

# 셀 최소 폭. 남는 폭은 셀에 고루 나눠 준다.
# 가장 긴 이름(map_amcl_lifecycle_manager)이 7.5 pt 에서 147 px 이므로
# 147 + 램프 12 + Hz 38 = 197 이 하한이다. 이보다 좁게 잡으면 이름이 잘려
# kau_lane_detection_node 와 _viewer 가 같은 글자로 보인다.
CELL_W = 200
CELL_H = 15
PAD = 6

LAMP_R = 3.0        # 상태등 반지름
LAMP_W = 12.0       # 상태등이 차지하는 폭
HZ_W = 38.0         # Hz 칸. "129.9Hz" 가 7 pt 에서 37 px

FONT_NAME = 7.5
FONT_HZ = 7.0

# 잴 대표 토픽이 없는 노드의 Hz 칸. 빈칸으로 두면 "측정 실패" 와 구분이
# 안 된다. en dash 다 -- 하이픈보다 길어 훑을 때 숫자 자리로 읽힌다.
HZ_NONE = "\u2013"


class StatusBar(QtWidgets.QWidget):
    """set_state 로만 갱신한다. 자체 타이머도 ROS 도 모른다."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self._nodes = []
        self.setSizePolicy(QtWidgets.QSizePolicy.Expanding,
                           QtWidgets.QSizePolicy.Fixed)
        self.setFixedHeight(PAD * 2 + CELL_H)

    def set_state(self, nodes):
        """nodes: [(name, 'ok'|'stale'|'absent', hz|None)].

        값이 같으면 다시 그리지 않는다. 갱신은 1 Hz 인데 렌더는 그보다
        빨라, 그냥 두면 같은 그림을 몇 번씩 다시 칠한다.
        """
        if nodes == self._nodes:
            return
        self._nodes = list(nodes)
        self._fit()
        self.update()

    def resizeEvent(self, ev):
        # 폭이 바뀌면 열 수가 바뀐다. 높이도 같이 따라가야 한다.
        super().resizeEvent(ev)
        self._fit()

    def _fit(self):
        # 창을 좁히면 열이 줄어 행이 늘어난다. 높이를 같이 안 늘리면
        # 아래 행이 잘려 노드가 사라진 것처럼 보인다.
        want = PAD * 2 + self._rows() * CELL_H
        if self.height() != want:
            self.setFixedHeight(want)

    def _cols(self):
        return max(1, max(1, self.width() - PAD * 2) // CELL_W)

    def _rows(self):
        if not self._nodes:
            return 1
        cols = self._cols()
        return (len(self._nodes) + cols - 1) // cols

    def paintEvent(self, _ev):
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.Antialiasing, True)
        p.fillRect(self.rect(), COL_BG)
        p.setPen(QtGui.QPen(COL_BORDER, 1))
        p.drawLine(0, self.height() - 1, self.width(), self.height() - 1)

        name_f = p.font()
        name_f.setPointSizeF(FONT_NAME)
        hz_f = p.font()
        hz_f.setPointSizeF(FONT_HZ)

        if not self._nodes:
            p.setFont(name_f)
            p.setPen(COL_DIM)
            p.drawText(
                QtCore.QRectF(PAD, PAD, self.width() - PAD * 2, CELL_H),
                QtCore.Qt.AlignVCenter | QtCore.Qt.AlignLeft,
                "감시 대상 없음 (status.enabled=false 이거나 bringup.yaml 을 못 읽었다)")
            return

        cols = self._cols()
        cw = (self.width() - PAD * 2) / cols
        metrics = QtGui.QFontMetrics(name_f)

        for i, (name, health, hz) in enumerate(self._nodes):
            x0 = PAD + (i % cols) * cw
            y0 = PAD + (i // cols) * CELL_H

            # 이름. 셀보다 길면 뒤를 줄인다 (노드명은 앞이 더 구분된다).
            nw = cw - LAMP_W - HZ_W
            p.setFont(name_f)
            p.setPen(COL_DIM if health == "absent" else COL_TEXT)
            p.drawText(
                QtCore.QRectF(x0, y0, nw, CELL_H),
                QtCore.Qt.AlignVCenter | QtCore.Qt.AlignLeft,
                metrics.elidedText(name, QtCore.Qt.ElideRight, int(nw)))

            # 상태등
            p.setPen(QtCore.Qt.NoPen)
            p.setBrush(LAMP.get(health, COL_ABSENT))
            p.drawEllipse(
                QtCore.QPointF(x0 + nw + LAMP_W * 0.5, y0 + CELL_H * 0.5),
                LAMP_R, LAMP_R)

            # Hz. 대표 토픽이 없거나 아직 표본이 모자라면 못 잰다.
            # 0.0 Hz 를 찍으면 "안 온다" 로 읽히는데 그건 사실이 아니다.
            # 그렇다고 비워 두면 **잴 것이 없는 것과 측정에 실패한 것이
            # 구분되지 않는다** -- 둘 다 빈칸이면 상태등만 보고 판단해야 한다.
            # 그래서 대시를 찍는다. 색이 흐려 훑을 때 눈에 걸리지 않는다.
            p.setFont(hz_f)
            p.setPen(COL_DIM)
            p.drawText(
                QtCore.QRectF(x0 + nw + LAMP_W, y0, HZ_W, CELL_H),
                QtCore.Qt.AlignVCenter | QtCore.Qt.AlignLeft,
                HZ_NONE if hz is None else f"{hz:.1f}Hz")
