"""최상단 노드 상태 패널. 좌우 분할과 무관하게 가로 전체를 쓴다.

셀 하나 = 한 노드.
    노드명            ●
              12.4 Hz

감시 대상은 bringup.yaml 이 기동하는 노드 그대로다. 목록을 config 에
고정해 두는 이유는, 자동 탐색으로는 **떠 있지 않은 노드**를 보여줄 수
없어 정작 필요한 빨간불이 안 켜지기 때문이다.

색: 초록 정상 / 빨강 노드는 살아있으나 토픽 끊김 / 회색 미기동.
등급 색은 신호등과 무관한 별도 배색이다 (docs/09 section 11-2).
"""

from __future__ import annotations

from pyqtgraph.Qt import QtCore, QtGui, QtWidgets

COL_OK = QtGui.QColor("#2ca02c")
COL_STALE = QtGui.QColor("#d62728")
COL_ABSENT = QtGui.QColor("#b0b0b0")
COL_TEXT = QtGui.QColor("#141414")
COL_DIM = QtGui.QColor("#787878")
COL_BG = QtGui.QColor("#ffffff")
COL_BORDER = QtGui.QColor("#aaaaaa")

CELL_W = 176
CELL_H = 34
PAD = 8
BANNER_H = 22
LAMP_R = 5

_LAMP = {"ok": COL_OK, "stale": COL_STALE, "absent": COL_ABSENT}


class StatusBar(QtWidgets.QWidget):

    def __init__(self, parent=None):
        super().__init__(parent)
        self._nodes = []
        self._banner = ""
        self.setSizePolicy(QtWidgets.QSizePolicy.Expanding,
                           QtWidgets.QSizePolicy.Fixed)
        self.setMinimumHeight(BANNER_H + PAD * 2 + CELL_H)

    def set_state(self, nodes, banner: str = ""):
        """nodes: [(name, 'ok'|'stale'|'absent', hz|None)]."""
        self._nodes = nodes
        self._banner = banner

        # 창을 좁히면 열이 줄어 행이 늘어난다. 높이를 안 늘리면 아래 행이
        # 통째로 잘려 노드가 사라진 것처럼 보인다.
        want = ((BANNER_H + 2 if banner else PAD)
                + self._rows() * CELL_H + PAD)
        if self.height() != want:
            self.setFixedHeight(want)
        self.update()

    def _cols(self):
        return max(1, max(1, self.width() - PAD * 2) // CELL_W)

    def _rows(self):
        if not self._nodes:
            return 1
        c = self._cols()
        return (len(self._nodes) + c - 1) // c

    def paintEvent(self, _ev):
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.Antialiasing, True)
        p.fillRect(self.rect(), COL_BG)
        p.setPen(QtGui.QPen(COL_BORDER, 1))
        p.drawLine(0, self.height() - 1, self.width(), self.height() - 1)

        top = PAD
        if self._banner:
            box = QtCore.QRectF(PAD, 2, self.width() - PAD * 2, BANNER_H - 4)
            p.setPen(QtCore.Qt.NoPen)
            bg = QtGui.QColor(COL_STALE)
            bg.setAlpha(40)
            p.setBrush(bg)
            p.drawRoundedRect(box, 3, 3)
            f = p.font()
            f.setPointSizeF(9.0)
            f.setBold(True)
            p.setFont(f)
            p.setPen(COL_STALE)
            p.drawText(box, QtCore.Qt.AlignCenter, self._banner)
            top = BANNER_H + 2

        if not self._nodes:
            f = p.font()
            f.setPointSizeF(9.0)
            f.setBold(False)
            p.setFont(f)
            p.setPen(COL_DIM)
            p.drawText(QtCore.QRectF(PAD, top, self.width() - PAD * 2, CELL_H),
                       QtCore.Qt.AlignVCenter | QtCore.Qt.AlignLeft,
                       "감시 대상 없음 (watch.names 미설정)")
            return

        cols = self._cols()
        # 남는 폭을 셀에 고루 나눠 준다. 고정폭이면 오른쪽이 비어 보인다.
        cw = (self.width() - PAD * 2) / cols

        name_f = p.font()
        name_f.setPointSizeF(9.0)
        name_f.setBold(False)
        hz_f = p.font()
        hz_f.setPointSizeF(8.5)
        hz_f.setBold(False)

        for i, (name, health, hz) in enumerate(self._nodes):
            col, row = i % cols, i // cols
            x0 = PAD + col * cw
            y0 = top + row * CELL_H

            p.setFont(name_f)
            p.setPen(COL_DIM if health == "absent" else COL_TEXT)
            nbox = QtCore.QRectF(x0, y0 + 2, cw - 18, 15)
            # 이름이 길면 잘라 준다. 넘치면 옆 노드와 붙어 읽을 수 없다.
            elided = p.fontMetrics().elidedText(
                name, QtCore.Qt.ElideRight, int(nbox.width()))
            p.drawText(nbox, QtCore.Qt.AlignVCenter | QtCore.Qt.AlignLeft,
                       elided)

            lamp = _LAMP.get(health, COL_ABSENT)
            cx, cy = x0 + cw - 12, nbox.center().y()
            p.setPen(QtCore.Qt.NoPen)
            # 바깥 옅은 링 + 안쪽 원. 작은 점만으로는 색 구분이 잘 안 된다.
            halo = QtGui.QColor(lamp)
            halo.setAlpha(45)
            p.setBrush(halo)
            p.drawEllipse(QtCore.QPointF(cx, cy), LAMP_R + 3, LAMP_R + 3)
            p.setBrush(lamp)
            p.drawEllipse(QtCore.QPointF(cx, cy), LAMP_R, LAMP_R)

            p.setFont(hz_f)
            p.setPen(COL_DIM)
            # 주기 판정 대상이 아니면 '-'. 0 Hz 로 오해하면 안 된다.
            txt = "-" if hz is None else f"{hz:.1f} Hz"
            p.drawText(QtCore.QRectF(x0, y0 + 17, cw - 18, 14),
                       QtCore.Qt.AlignVCenter | QtCore.Qt.AlignRight, txt)
