#!/usr/bin/env python3
r"""`lane_graph.yaml` 의 center 레이어에서 실험용 변형 경로를 만든다.

`lane_graph.yaml` 은 건드리지 않는다. 결과는 config/ 에 별도 파일로 떨어지고,
발행 노드의 `path` 인자만 갈아끼우면 그대로 실험된다.

    python3 src/kau_global_path/scripts/gen_lane_variants.py --all
    ros2 launch kau_global_path global_path.launch.py \\
        path:=/.../config/right_bias_lane.yaml

만드는 것

    right_bias_lane.yaml      중심선을 진행 방향 오른쪽으로 민 것
    last_obstacle_lane.yaml   마지막 콘 하나만 피하는 것 (예선)
    every_obstacle_lane.yaml  콘 6 개를 전부 피하는 것 (예선)

레이어 이름은 전부 `center`, route 는 `center_loop` 다. 원본과 같으므로
`route` 인자는 그대로 두고 `path` 만 바꾸면 된다.

방식 — 오프셋 곡선을 해석적으로 세운다

    원본 곡선을 등호길이로 샘플해 (P, θ, κ) 를 얻고, 횡오프셋 d(s) 를 얹는다
    (오른쪽이 +). 오프셋 곡선의 접선·곡률은 근사가 아니라 식으로 나온다.

        Q  = P - d N              N 은 왼쪽 법선
        Q' = (1+dκ) T - d' N
        κQ = [(1+dκ)(κ + dκ² - d'') + d'(2d'κ + dκ')] / |Q'|³

    이 (θQ, κQ) 를 노드마다 quintic Hermite 로 물려 Bezier 제어점 6 개를 뽑는다.
    노드에서 양쪽이 같은 (θ, κ) 를 쓰므로 C² 는 구성상 정확히 성립한다
    (README 4.4). 노드는 제어점이 아니다 — 곡선이 지나는 점이다.

d(s) 를 정하는 규칙

    1. 하고 싶은 값 (bias 목표치 / 콘 회피 요구치)
    2. 곡률 상한  |κ/(1+dκ)| <= 한계        우회전에서 오른쪽으로 밀면 반경이
                                            줄어 한계를 넘는다. 여기서 잘린다
    3. 코리도 상한  주행면 테두리까지 법선 레이 - 차체 반경 - 여유
    4. 기울기 제한 |d'| <= lam              옆으로 갈아타는 속도
    5. 가우시안 평활                        d'' 가 튀면 κQ 가 그만큼 튄다
    6. 실제 κQ 를 재서 넘으면 그 자리 상한을 깎고 1 로 되돌아간다

콘 좌표는 `kau_localization/scripts/cones/cone*.sdf` 의 sim pose 를 map 으로
옮긴 것이다 (map_x = ox - sim_y, map_y = sim_x - oy). 지도에는 콘을 넣지
않는다 (README 6.2) — 경로 생성 입력으로만 쓴다.
"""

import argparse
import json
import math
import re
import sys
from pathlib import Path

import numpy as np

import yaml

PKG = Path(__file__).resolve().parents[1]
CONE_DIR = PKG.parents[0] / 'kau_localization' / 'scripts' / 'cones'

DEGREE = 5
NCTRL = DEGREE + 1
DS = 0.005          # m. 프로파일 격자. 원본의 2 cm 짜리 조각까지 담아야 한다
# 제어점 소수 자리. 원본은 7 자리다. 이 파일들은 조각이 원본보다 짧아
# (0.1 m 대 0.6 m) 같은 반올림이 이음새 κ 를 0.006 1/m 씩 흔든다 —
# check_path.py 의 G2 허용오차 0.005 를 넘긴다. 두 자리 더 적으면 그 몫이
# 100 배 줄어든다. 곡선 자체는 구성상 정확히 C² 다 (같은 (θ,κ) 를 양쪽이 쓴다).
ROUND = 9


# ---------------------------------------------------------------- Bezier
# check_path.py / lane_editor.js 와 같은 식이어야 한다.

def de_casteljau(ctrl, u):
    p = [list(q) for q in ctrl]
    for r in range(len(p) - 1):
        for i in range(len(p) - 1 - r):
            p[i] = [(1 - u) * p[i][0] + u * p[i + 1][0],
                    (1 - u) * p[i][1] + u * p[i + 1][1]]
    return p[0]


def hodograph(ctrl):
    n = len(ctrl) - 1
    return [[n * (ctrl[i + 1][0] - ctrl[i][0]),
             n * (ctrl[i + 1][1] - ctrl[i][1])] for i in range(n)]


def seg_length(ctrl, samples=400):
    prev = de_casteljau(ctrl, 0.0)
    total = 0.0
    for i in range(1, samples + 1):
        q = de_casteljau(ctrl, i / samples)
        total += math.hypot(q[0] - prev[0], q[1] - prev[1])
        prev = q
    return total


def kappa_at(ctrl, u):
    d1 = de_casteljau(hodograph(ctrl), u)
    d2 = de_casteljau(hodograph(hodograph(ctrl)), u)
    sp = math.hypot(d1[0], d1[1])
    if sp < 1e-12:
        return 0.0
    return (d1[0] * d2[1] - d1[1] * d2[0]) / (sp ** 3)


def seg_kappa_max(ctrl, samples=200):
    return max(abs(kappa_at(ctrl, i / samples)) for i in range(samples + 1))


def frame_at(ctrl, u):
    """(점, θ, κ, |P'|)."""
    d1 = de_casteljau(hodograph(ctrl), u)
    d2 = de_casteljau(hodograph(hodograph(ctrl)), u)
    sp = math.hypot(d1[0], d1[1])
    k = 0.0 if sp < 1e-12 else (d1[0] * d2[1] - d1[1] * d2[0]) / (sp ** 3)
    return de_casteljau(ctrl, u), math.atan2(d1[1], d1[0]), k, sp


def hermite_to_bezier(p0, th0, k0, p1, th1, k1):
    """양 끝 (점, θ, κ) -> quintic Bezier 제어점 6 개.

    lane_editor.js 의 hermiteToBezier 와 같은 식이다. σ 를 현 길이로 잡으면
    양 끝의 (θ, κ) 는 σ 에 무관하게 보존된다 — κ = |P'xP''|/|P'|³ 에서
    약분되기 때문이다. 그래서 이음새의 C² 는 구성상 정확하다.
    """
    chord = math.hypot(p1[0] - p0[0], p1[1] - p0[1])

    def derivs(th, k):
        t = (math.cos(th), math.sin(th))
        n = (-math.sin(th), math.cos(th))
        return ((chord * t[0], chord * t[1]),
                (chord * chord * k * n[0], chord * chord * k * n[1]))

    (A, B) = derivs(th0, k0)
    (C, D) = derivs(th1, k1)
    return [
        [p0[0], p0[1]],
        [p0[0] + A[0] / 5, p0[1] + A[1] / 5],
        [p0[0] + 2 * A[0] / 5 + B[0] / 20, p0[1] + 2 * A[1] / 5 + B[1] / 20],
        [p1[0] - 2 * C[0] / 5 + D[0] / 20, p1[1] - 2 * C[1] / 5 + D[1] / 20],
        [p1[0] - C[0] / 5, p1[1] - C[1] / 5],
        [p1[0], p1[1]],
    ]


# ---------------------------------------------------------------- 입력

class Base:
    """원본 center 곡선을 등호길이 격자로 편 것."""

    def __init__(self, segs, ds=DS, short_seg=0.06, pad=0.10):
        # 조각마다 u <-> 호길이 표를 만들고, 전체를 균일 격자로 다시 잡는다.
        tables, lengths = [], []
        for s in segs:
            ctrl = s['ctrl']
            us = np.linspace(0.0, 1.0, 601)
            pts = np.array([de_casteljau(ctrl, float(u)) for u in us])
            cum = np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(pts, axis=0).T))])
            tables.append((ctrl, us, cum))
            lengths.append(cum[-1])

        self.seg_s0 = np.concatenate([[0.0], np.cumsum(lengths)])
        self.total = float(self.seg_s0[-1])
        self.n = int(round(self.total / ds))
        self.ds = self.total / self.n
        self.s = np.arange(self.n) * self.ds

        P, TH, K, SEG = [], [], [], []
        for si in range(self.n):
            s = self.s[si]
            j = min(int(np.searchsorted(self.seg_s0, s, 'right') - 1), len(segs) - 1)
            ctrl, us, cum = tables[j]
            u = float(np.interp(s - self.seg_s0[j], cum, us))
            p, th, k, _ = frame_at(ctrl, u)
            P.append(p)
            TH.append(th)
            K.append(k)
            SEG.append(j)

        self.P = np.array(P)
        self.TH = np.array(TH)
        self.K = np.array(K)
        self.seg_of = np.array(SEG)
        self.seg_len = np.array(lengths)
        self.seg_type = [str(s.get('type', '?')) for s in segs]

        # 짧은 조각은 자기 안에서 θ 와 κ 가 서로 안 맞는다.
        #
        # 제어점이 소수 7 자리다. 2 cm 짜리 조각(제어점 간격 4 mm)에서는 그
        # 반올림이 2차 미분에 증폭돼 κ 가 ±0.9 1/m 씩 흔들리고, 실제 도는 각도도
        # κ 가 말하는 값과 2° 쯤 어긋난다 (check_path.py 의 tol-kappa 주석이
        # 말하는 것과 같은 현상인데, 조각이 짧을수록 훨씬 크다).
        #
        # d=0 이면 이게 안 보인다. 오프셋을 얹는 순간 θ 오차가 d 배로 위치에
        # 실려서, 그 2 cm 안에서 곡률이 2.8 1/m 까지 튄다.
        #
        # 그래서 **그 구간만 다시 그린다.** 양옆 긴 조각에서 (θ, κ) 를 받아
        # quintic 하나로 잇고, 그 곡선의 값으로 격자를 덮어쓴다. 원본과 달라지는
        # 양은 결함 자체의 크기(1 mm 안쪽)를 넘지 않는다.
        self.repaired = self._repair_short(lengths, short_seg, pad)

        w = max(1, int(round(0.03 / self.ds)))
        ker = np.ones(2 * w + 1) / (2 * w + 1)
        self.K_smooth = np.convolve(np.concatenate([self.K[-w:], self.K, self.K[:w]]),
                                    ker, mode='valid')

        # 오른쪽 법선. 진행 방향에서 -90°.
        self.NR = np.stack([np.sin(self.TH), -np.cos(self.TH)], axis=1)
        # 원본 조각 경계의 station. 노드 자리로 그대로 물려받는다.
        self.breaks = np.array(sorted({int(round(v / self.ds)) % self.n
                                       for v in self.seg_s0[:-1]}))

    def _repair_short(self, lengths, short_seg, pad):
        """짧은 조각 주변을 quintic 하나로 다시 그린다. 고친 샘플 수를 준다."""
        fixed = 0
        for j, L in enumerate(lengths):
            if L >= short_seg:
                continue
            a = int(round((self.seg_s0[j] - pad) / self.ds)) % self.n
            b = int(round((self.seg_s0[j + 1] + pad) / self.ds)) % self.n
            span = (b - a) % self.n
            if span < 4:
                continue

            ctrl = hermite_to_bezier(self.P[a], self.TH[a], self.K[a],
                                     self.P[b], self.TH[b], self.K[b])
            us = np.linspace(0.0, 1.0, 201)
            pts = np.array([de_casteljau(ctrl, float(u)) for u in us])
            cum = np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(pts, axis=0).T))])

            for m in range(1, span):
                u = float(np.interp(cum[-1] * m / span, cum, us))
                q, th, k, _ = frame_at(ctrl, u)
                i = (a + m) % self.n
                self.P[i] = q
                self.TH[i] = th
                self.K[i] = k
                fixed += 1
        return fixed

    def station_of(self, xy):
        d = np.hypot(self.P[:, 0] - xy[0], self.P[:, 1] - xy[1])
        i = int(np.argmin(d))
        lat = float(np.dot([xy[0] - self.P[i, 0], xy[1] - self.P[i, 1]], self.NR[i]))
        return i, lat, float(d[i])


def load_layer(path: Path, layer='center'):
    doc = yaml.safe_load(path.read_text())
    g = doc['lane_graph']
    segs = (g.get('bezier') or {}).get(layer)
    if not segs:
        raise SystemExit(f'{path}: bezier.{layer} 가 없다')
    return g, segs


def load_cones(directory: Path, ox, oy):
    """cone*.sdf 의 sim pose -> map. (이름, x, y) 목록."""
    out = []
    for f in sorted(directory.glob('cone*.sdf')):
        m = re.search(r'<pose>([-\d.eE+ ]+)</pose>', f.read_text())
        if not m:
            continue
        v = [float(q) for q in m.group(1).split()]
        out.append((f.stem, ox - v[1], v[0] - oy))
    if not out:
        raise SystemExit(f'{directory} 에서 콘을 못 찾았다')
    return out


def load_road_rings(track_json: Path, ox, oy):
    doc = json.loads(track_json.read_text())
    rings = (doc.get('layers', {}).get('road', {}) or {}).get('rings') or []
    return [np.array([[ox - q[1], q[0] - oy] for q in r]) for r in rings]


def start_station(base: Base, track_json: Path, ox, oy):
    """출발선 중심에 제일 가까운 격자 index. 없으면 0."""
    doc = json.loads(track_json.read_text())
    ring = (doc.get('layers', {}).get('start_line', {}) or {}).get('rings')
    if not ring:
        return 0
    pts = np.array([[ox - q[1], q[0] - oy] for q in ring[0]])
    i, _, _ = base.station_of(pts.mean(axis=0))
    return i


# ---------------------------------------------------------------- 코리도

def ray_room(origins, dirs, rings):
    """각 점에서 dirs 방향으로 링에 닿을 때까지의 거리."""
    out = np.full(len(origins), np.inf)
    for ring in rings:
        A = ring
        B = np.roll(ring, -1, axis=0)
        E = B - A
        for i, (o, d) in enumerate(zip(origins, dirs)):
            den = d[0] * E[:, 1] - d[1] * E[:, 0]
            ok = np.abs(den) > 1e-12
            den = np.where(ok, den, 1.0)
            W = A - o
            t = (W[:, 0] * E[:, 1] - W[:, 1] * E[:, 0]) / den
            u = (W[:, 0] * d[1] - W[:, 1] * d[0]) / den
            m = ok & (t > 0) & (u >= 0.0) & (u <= 1.0)
            if m.any():
                out[i] = min(out[i], float(t[m].min()))
    return out


# ---------------------------------------------------------------- 프로파일
#
# d(s) 는 반드시 매끄러워야 한다. κQ 에 d'' 가 그대로 들어가므로 (offset_geometry
# 참고), 격자 5 mm 에서 0.1 mm 짜리 리플만 남아도 곡률이 4 1/m 씩 튄다.
# 그래서 "클립 -> 평활" 을 반복하지 않는다. 상한을 커널 반경만큼 **침식**해 두고
# 클립한 뒤, 평활은 마지막에 딱 한 번 건다. 이러면
#
#   d_raw(s) <= hi_erode(s) = min_{|u|<=r} hi(s+u)   이고 가중치 합이 1 이므로
#   (d_raw * w)(s) = Σ w(u) d_raw(s+u) <= Σ w(u) hi(s) = hi(s)
#
# 가 성립한다 — 평활 결과도 원래 상한 안에 있다. 침식 반경과 커널 반경이 같아야
# 이 논증이 성립하므로 둘 다 r 로 묶어 둔다.

def circ_deriv(x, ds):
    return (np.roll(x, -1) - np.roll(x, 1)) / (2.0 * ds)


def offset_geometry(base, d, dp=None, dpp=None):
    """오프셋 곡선의 점 / θ / κ. 근사가 아니라 해석해다.

    Q  = P - d N,   Q' = (1+dκ) T - d' N
    κQ = [(1+dκ)(κ + dκ² - d'') + d'(2d'κ + dκ')] / |Q'|³
    """
    if dp is None:
        dp = circ_deriv(d, base.ds)
    if dpp is None:
        dpp = circ_deriv(dp, base.ds)
    k = base.K
    # κ' 는 **평활한 κ** 에서 뽑는다. 짧은 조각 보정(Base 참고)의 이음매에서
    # 원시 κ 가 한 칸에 0.9 씩 뛰는데, 그대로 미분하면 kp 가 90 1/m² 이 되고
    # d·kp 항이 κQ 를 0.7 씩 흔든다. 이 항 자체는 d·d' 에 곱해져 작으므로
    # 평활해도 정확도 손해가 없다.
    kp = circ_deriv(base.K_smooth, base.ds)

    Q = base.P + d[:, None] * base.NR
    a = 1.0 + d * k
    b = -dp
    sp = np.hypot(a, b)
    cross = a * (k + d * k * k - dpp) + dp * (2.0 * dp * k + d * kp)
    kq = cross / np.maximum(sp ** 3, 1e-12)
    thq = base.TH + np.arctan2(b, a)
    return Q, thq, kq


def kappa_target(base, klim, safety):
    """해석 곡률에 걸 처음 상한.

    원본은 가장 급한 코너에서 |κ| 1.8088 로 이미 한계(1.8199)의 99 % 다.
    한계에 안전율만 곱해 들이대면 **d=0 자체가 위반**이 되어, 최적화가 콘도
    없는 코너를 벌리기 시작한다 (실측: 관계없는 구간에서 ±8 cm 요동).
    그래서 "원본보다 나빠지지 않는다" 를 처음 상한으로 잡고, 실제로 만들어진
    조각이 한계를 넘으면 그 자리만 make_variant 의 바깥 고리가 조인다.
    """
    return np.maximum(klim * safety, np.abs(base.K))


def corridor_caps(base, rings, args):
    """주행면 테두리까지 법선 레이 - 차체 반경 - 여유."""
    room_r = ray_room(base.P, base.NR, rings) - args.body_radius - args.edge_margin
    room_l = ray_room(base.P, -base.NR, rings) - args.body_radius - args.edge_margin
    hi = np.minimum(room_r, args.max_offset)
    lo = np.maximum(-room_l, -args.max_offset)
    return hi, lo


# ---------------------------------------------------------------- 최적화
#
# d(s) 를 격자 점마다 자유롭게 두면 d'' 가 통제가 안 된다 (κQ 에 그대로 들어가
# 곡률이 튄다). **주기 3차 B-spline 계수**로 두면 매듭 간격이 곧 d 가 만들 수
# 있는 가장 급한 굴곡이 되고, 변수도 백여 개로 줄어 그냥 최적화할 수 있다.
#
# 목적함수 하나에 전부 넣는다 — 하고 싶은 값, 곡률 한계, 코리도, 콘 여유.
# 한계 항은 위반량의 제곱이라, 못 지키면 "얼마나 못 지켰는지" 가 그대로 남는다.

def bspline_basis(n, ds, total, m):
    """주기 3차 B-spline. (B, B1, B2) 를 격자에서 평가해 둔다."""
    h = total / m
    s = np.arange(n) * ds
    B = np.zeros((n, m))
    B1 = np.zeros((n, m))
    B2 = np.zeros((n, m))

    def w(u):                      # 3차 B-spline 과 그 미분 (u 는 h 단위)
        au = np.abs(u)
        far = np.where(au < 2, (2.0 - au) ** 3 / 6.0, 0.0)
        v = np.where(au < 1, 2.0 / 3.0 - au ** 2 + au ** 3 / 2.0, far)
        far1 = np.where(au < 2, -(2.0 - au) ** 2 / 2.0 * np.sign(u), 0.0)
        d1 = np.where(au < 1, (-2.0 * au + 1.5 * au ** 2) * np.sign(u), far1)
        far2 = np.where(au < 2, 2.0 - au, 0.0)
        d2 = np.where(au < 1, -2.0 + 3.0 * au, far2)
        return v, d1, d2

    for j in range(m):
        u = (s - j * h) / h
        u = (u + m / 2.0) % m - m / 2.0        # 주기
        v, d1, d2 = w(u)
        B[:, j] = v
        B1[:, j] = d1 / h
        B2[:, j] = d2 / (h * h)
    return B, B1, B2


def solve_offset(base, args, hi_cap, lo_cap, ktgt, target, cone_req, verbose=False):
    """오프셋 프로파일을 푼다. 돌려주는 것은 (d, d', d'', 진단)."""
    from scipy.optimize import minimize

    m = max(16, int(round(base.total / args.knot)))
    B, B1, B2 = bspline_basis(base.n, base.ds, base.total, m)
    wds = base.ds

    def unpack(c):
        return B @ c, B1 @ c, B2 @ c

    def parts(c):
        d, dp, dpp = unpack(c)
        _, _, kq = offset_geometry(base, d, dp, dpp)

        e_want = np.sum((d - target) ** 2) * wds
        e_kap = np.sum(np.maximum(np.abs(kq) - ktgt, 0.0) ** 2) * wds
        e_cap = (np.sum(np.maximum(d - hi_cap, 0.0) ** 2) +
                 np.sum(np.maximum(lo_cap - d, 0.0) ** 2)) * wds
        e_sm = np.sum(dpp ** 2) * wds
        e_dd = np.sum(np.maximum(np.abs(dpp) - args.ddmax, 0.0) ** 2) * wds

        e_cone = 0.0
        for i, lo_i, hi_i in cone_req:
            e_cone += max(0.0, d[i] - hi_i) ** 2 + max(0.0, lo_i - d[i]) ** 2
        return d, dp, dpp, kq, e_want, e_kap, e_cap, e_sm, e_dd, e_cone

    def cost(c):
        _, _, _, _, e_want, e_kap, e_cap, e_sm, e_dd, e_cone = parts(c)
        return (args.w_want * e_want + args.w_kappa * e_kap + args.w_cap * e_cap +
                args.w_smooth * e_sm + args.w_dd * e_dd + args.w_cone * e_cone)

    c0 = np.zeros(m)
    # 초기값: 하고 싶은 값을 캡 안에 넣고 계수로 투영한다 (최소제곱)
    d0 = np.clip(target, lo_cap, hi_cap)
    c0 = np.linalg.lstsq(B.T @ B + 1e-6 * np.eye(m), B.T @ d0, rcond=None)[0]

    res = minimize(cost, c0, method='L-BFGS-B',
                   options={'maxiter': args.maxiter, 'maxfun': args.maxiter * (m + 2)})
    d, dp, dpp, kq, e_want, e_kap, e_cap, e_sm, e_dd, e_cone = parts(res.x)

    diag = {
        'm': m, 'nit': res.nit, 'ok': bool(res.success),
        'kappa_over': float(np.maximum(np.abs(kq) - ktgt, 0.0).max()),
        'cap_over': float(max(np.maximum(d - hi_cap, 0.0).max(),
                              np.maximum(lo_cap - d, 0.0).max())),
        'cone_over': math.sqrt(e_cone) if cone_req else 0.0,
        'ddmax': float(np.abs(dpp).max()),
    }
    if verbose:
        print(f'    B-spline 계수 {m} · 반복 {res.nit} · '
              f'κ 초과 {diag["kappa_over"]:.4f} · 캡 초과 {diag["cap_over"] * 100:.2f} cm '
              f'· |d\'\'|max {diag["ddmax"]:.2f}')
    return d, dp, dpp, diag


# ---------------------------------------------------------------- 노드 / 조각

def pick_nodes(base, thq, kq, d, args):
    """노드 자리를 고른다.

    두 가지를 지킨다.

      원본 조각 경계는 전부 노드로 물려받는다. 경계는 원본이 직선/원호/전이를
      나눠 둔 자리라 그 안에서는 κ 가 얌전하다.

      **짧은 조각은 절대 쪼개지 않는다.** 전이(15 cm)를 반으로 자르면 한
      조각이 κ 1.0 -> 1.8 을 11 cm 안에서 이어야 하고, quintic 은 그걸
      반드시 넘겨서 그린다 (실측 +10 %). 원본이 전이를 통째로 한 조각에
      담는 이유가 이것이다.

    긴 조각만 나눈다. 나누는 기준은 방향 변화 (원호에서 quintic 이 부푸는
    양은 조각당 회전각으로 정해진다 — 8° 에서 +0.1 %, 13.5° 에서 +0.4 %) 와
    오프셋 변화다.
    """
    dth = math.radians(args.dtheta)
    br = [int(b) for b in base.breaks]
    idx = []

    for a in range(len(br)):
        i, j = br[a], br[(a + 1) % len(br)]
        idx.append(i)
        span = (j - i) % base.n
        if span == 0:
            continue
        seg_len = span * base.ds
        span_idx = np.arange(i, i + span + 1) % base.n

        # 짧은 조각만 통째로 둔다.
        #
        # 전이(link)도 나눈다. 안 나누면 오히려 나빠진다 — 0.27 m 짜리 전이를
        # quintic 하나로 덮으면 원본에서 14 mm 벗어나고 κ 가 2.04 까지 간다
        # (실측). 나누면 0.14 mm / 1.8169 다. 원본이 전이를 한 조각에 담은 건
        # 편집기가 노드를 그렇게 찍었기 때문이지, 나누면 안 되어서가 아니다.
        if seg_len <= args.keep_whole:
            continue

        turn = 0.0
        for t_ in range(1, span + 1):
            a0 = thq[(i + t_ - 1) % base.n]
            a1 = thq[(i + t_) % base.n]
            turn += abs(math.atan2(math.sin(a1 - a0), math.cos(a1 - a0)))
        dspan = float(np.abs(d[span_idx] - d[i]).max())
        # 조각 안에서 κ 가 얼마나 움직이는가. 한 조각이 감당할 Δκ 를 넘으면
        # quintic 이 그 차이를 넘겨서 그린다 — 오프셋을 얹으면 여기가 먼저 터진다.
        kspan = float(np.abs(np.diff(kq[span_idx])).sum())

        m = max(1,
                int(math.ceil(seg_len / args.max_step)),
                int(math.ceil(turn / dth)) if dth > 0 else 1,
                int(math.ceil(kspan / args.dkappa)),
                int(math.ceil(dspan / 0.02)))
        for k in range(1, m):
            q = (i + int(round(k * span / m))) % base.n
            if q != i and q != j:
                idx.append(q)

    idx = sorted(set(idx))

    # 너무 짧은 조각은 만들지 않는다.
    #
    # 원본의 2 cm 짜리 link 는 자기 안에서 θ 와 κ 가 서로 안 맞는다 (제어점
    # 반올림 때문이며, Base 에서 κ 만 보정했다). 그 위에 그대로 조각을 세우면
    # 2 cm 안에서 모순을 흡수하느라 곡률이 1 1/m 씩 튄다. 경계를 하나 건너뛰어
    # 더 긴 조각에 담으면 같은 모순이 길이에 반비례해 묽어진다.
    out = [idx[0]]
    for q in idx[1:]:
        if ((q - out[-1]) % base.n) * base.ds >= args.min_out:
            out.append(q)
    if ((out[0] - out[-1]) % base.n) * base.ds < args.min_out and len(out) > 2:
        out.pop()
    return out


def build_segments(Q, thq, kq, idx):
    """노드 목록 -> 조각. 이음새는 같은 (θ, κ) 를 쓰므로 C² 다."""
    segs = []
    n = len(idx)
    for a in range(n):
        i, j = idx[a], idx[(a + 1) % n]
        ctrl = hermite_to_bezier(Q[i], thq[i], kq[i], Q[j], thq[j], kq[j])
        ctrl = [[round(x, ROUND), round(y, ROUND)] for x, y in ctrl]
        segs.append({'type': 'free', 'from': a, 'to': (a + 1) % n,
                     'length': seg_length(ctrl), 'kappa_max': seg_kappa_max(ctrl),
                     'ctrl': ctrl})
    return segs


def curve_points(segs, step=0.01):
    out = []
    for s in segs:
        n = max(2, int(s['length'] / step))
        for i in range(n):
            out.append(de_casteljau(s['ctrl'], i / n))
    return np.array(out)


# ---------------------------------------------------------------- 출력

def write_yaml(path: Path, g, segs, meta_lines):
    nodes = [(i, s['ctrl'][0][0], s['ctrl'][0][1]) for i, s in enumerate(segs)]
    L = []
    L.append('# kau_global_path lane_graph — gen_lane_variants.py 로 생성.')
    L.append('# 손으로 고치지 말 것. 원본은 config/lane_graph.yaml 이고,')
    L.append('# 이 파일은 거기서 횡오프셋을 얹어 다시 뽑은 것이다.')
    L.append('#')
    for t in meta_lines:
        L.append(f'# {t}')
    L.append('')
    L.append('lane_graph:')
    L.append(f'  frame_id: {g.get("frame_id", "map")}')
    L.append(f'  map_source: {g.get("map_source", "?")}')
    L.append(f'  map_resolution: {g.get("map_resolution", 0.05)}')
    origin = g.get('map_origin') or [0.0, 0.0, 0.0]
    L.append(f'  map_origin: [{origin[0]}, {origin[1]}, {origin[2]}]')
    L.append('')
    L.append('  nodes:')
    for i, x, y in nodes:
        L.append(f'    - {{id: {i}, x: {x:.4f}, y: {y:.4f}}}')
    L.append('')
    L.append('  routes:')
    seq = ', '.join(str(i) for i, _, _ in nodes) + ', 0'
    L.append(f'    center_loop: [{seq}]')
    L.append('')
    L.append('  closed:')
    L.append('    center: true')
    L.append('')
    L.append('  bezier:')
    L.append('    center:')
    for s in segs:
        c = ', '.join(f'[{x:.9f}, {y:.9f}]' for x, y in s['ctrl'])
        L.append(f'      - {{type: {s["type"]}, from: {s["from"]}, to: {s["to"]}, '
                 f'length: {s["length"]:.6f}, kappa_max: {s["kappa_max"]:.6f},')
        L.append(f'         ctrl: [{c}]}}')
    L.append('')
    path.write_text('\n'.join(L))


def plot_variant(path, name, base, d, segs, cones, rings, args):
    """원본 · 변형 · 콘 · 오프셋 프로파일을 한 장에 그린다."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    pts = curve_points(segs, 0.01)
    fig, (ax, bx) = plt.subplots(2, 1, figsize=(7.5, 12),
                                 gridspec_kw={'height_ratios': [3, 1]})

    for r in rings:
        ax.fill(r[:, 0], r[:, 1], color='#b8c4d0', zorder=0)
    ax.plot(base.P[:, 0], base.P[:, 1], '--', color='#c8a000', lw=1.0,
            label='원본 lane_graph', zorder=2)
    ax.plot(pts[:, 0], pts[:, 1], '-', color='#1f6fd0', lw=1.6,
            label=name, zorder=3)
    for nm, cx, cy in cones:
        ax.add_patch(plt.Circle((cx, cy), args.cone_radius, color='#d02020',
                                alpha=0.85, zorder=4))
        ax.add_patch(plt.Circle((cx, cy), args.obs_clear, color='#d02020',
                                fill=False, ls=':', lw=0.8, zorder=4))
        ax.annotate(nm, (cx, cy), textcoords='offset points', xytext=(6, 6),
                    fontsize=7, color='#802020', zorder=5)
    ax.set_aspect('equal')
    ax.grid(alpha=0.3)
    ax.legend(loc='upper right', fontsize=8)
    ax.set_title(f'{name}  ·  길이 {sum(s["length"] for s in segs):.3f} m  ·  '
                 f'|k|max {max(s["kappa_max"] for s in segs):.4f} 1/m')

    _, _, kq = offset_geometry(base, d)
    bx.axhline(0, color='#888', lw=0.8)
    bx.plot(base.s, d * 100, color='#1f6fd0', lw=1.2, label='offset d [cm] (우 +)')
    bx.plot(base.s, kq, color='#c04000', lw=0.8, alpha=0.8, label='kappa [1/m]')
    bx.plot(base.s, base.K, '--', color='#c8a000', lw=0.8, alpha=0.8,
            label='원본 kappa')
    for nm, cx, cy in cones:
        i, _, _ = base.station_of((cx, cy))
        bx.axvline(base.s[i], color='#d02020', lw=0.8, alpha=0.5)
    bx.set_xlabel('station [m]')
    bx.grid(alpha=0.3)
    bx.legend(fontsize=8, loc='upper right')

    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


# ---------------------------------------------------------------- 변형

def report(name, base, d, segs, cones, klim, rings, args):
    pts = curve_points(segs, 0.01)
    kmax = max(s['kappa_max'] for s in segs)
    total = sum(s['length'] for s in segs)

    print(f'\n[{name}]')
    print(f'  노드/조각 {len(segs)}  ·  길이 {total:.3f} m '
          f'(원본 {base.total:.3f} m, {total - base.total:+.3f})')
    print(f'  |κ|max {kmax:.4f} 1/m  (한계 {klim:.4f}, {kmax / klim * 100:.1f} %)' +
          ('   <- 초과' if kmax > klim else ''))
    print(f'  오프셋 d  최대 우 {d.max() * 100:+.1f} cm · 최대 좌 {d.min() * 100:+.1f} cm '
          f'· 평균 {d.mean() * 100:+.1f} cm')

    # 주행면 테두리 여유
    worst = np.inf
    for ring in rings:
        A, B = ring, np.roll(ring, -1, axis=0)
        E = B - A
        l2 = np.maximum((E ** 2).sum(axis=1), 1e-18)
        for p in pts[::5]:
            t = np.clip(((p - A) * E).sum(axis=1) / l2, 0.0, 1.0)
            worst = min(worst, float(np.hypot(*(p - (A + t[:, None] * E)).T).min()))
    print(f'  주행면 테두리 여유 최소 {worst * 100:.1f} cm '
          f'(차체 반경 {args.body_radius * 100:.1f} cm)' +
          ('   <- 차체가 닿는다' if worst < args.body_radius else ''))

    if cones:
        print('  콘 여유 (중심-경로 거리, 필요 '
              f'{args.obs_clear * 100:.1f} cm)')
        for nm, cx, cy in cones:
            dmin = float(np.hypot(pts[:, 0] - cx, pts[:, 1] - cy).min())
            mark = '' if dmin >= args.obs_clear - 1e-3 else '   <- 부족'
            print(f'    {nm}  {dmin * 100:6.2f} cm{mark}')


def cone_constraints(base, cones, args, boost=None):
    """콘마다 '이 station 에서 오프셋은 이 범위' 라는 제약으로 편다.

    콘이 오른쪽에 있으면 왼쪽으로 지난다 (상한), 왼쪽에 있으면 오른쪽 (하한).
    필요 거리는 콘 반경(대각) + 차체 반경 + 여유이고, `--obs-clear` 하나로 묶어
    두었다. 실제 판정은 2D 거리로 다시 하며(make_variant), 평활 때문에 모자라면
    boost 로 요구치를 올린다.
    """
    req, info = [], []
    for j, (nm, cx, cy) in enumerate(cones):
        i, lat, _ = base.station_of((cx, cy))
        need = args.obs_clear + (0.0 if boost is None else float(boost[j]))
        if lat >= 0.0:
            lo_i, hi_i = -np.inf, lat - need
        else:
            lo_i, hi_i = lat + need, np.inf
        req.append((i, lo_i, hi_i))
        info.append((nm, i, lat, lo_i, hi_i))
    return req, info


def make_variant(name, base, args, rings, cones, klim, verbose=False):
    """d(s) 를 풀고 조각을 만든다.

    바깥 고리가 두 가지를 실물 기준으로 확인한다.

      곡률 — 해석 κQ 가 아니라 **만들어진 Bezier 조각의 kappa_max** 를 본다.
             노드에서 (θ,κ) 를 물려 만든 quintic 은 조각 안쪽에서 조금 부푼다.
             넘친 조각이 있으면 그 구간의 상한만 넘친 만큼 깎고 다시 푼다.
      여유 — 콘까지의 실제 2D 거리를 잰다. 요구는 그 station 의 오프셋으로만
             걸리므로 한 번에 안 맞는 게 정상이다. 모자라면 그 콘만 올린다.
    """
    ktgt = kappa_target(base, klim, args.kappa_safety)
    hi_cap, lo_cap = corridor_caps(base, rings, args)
    target = np.zeros(base.n) if cones else np.full(base.n, args.bias)
    kmax_ok = klim * args.kappa_final

    boost = np.zeros(len(cones)) if cones else None
    best = None
    warn = []

    for outer in range(args.max_iter):
        req, _ = cone_constraints(base, cones, args, boost) if cones else ([], [])
        d, dp, dpp, diag = solve_offset(base, args, hi_cap, lo_cap, ktgt, target,
                                        req, verbose)
        Q, thq, kq = offset_geometry(base, d, dp, dpp)
        idx = pick_nodes(base, thq, kq, d, args)
        segs = build_segments(Q, thq, kq, idx)

        kbuilt = max(s['kappa_max'] for s in segs)
        short = []
        for j, (nm, cx, cy) in enumerate(cones):
            dmin = float(np.hypot(Q[:, 0] - cx, Q[:, 1] - cy).min())
            if args.obs_clear - dmin > 1e-3:
                short.append((j, nm, args.obs_clear - dmin, dmin))

        score = (max(0.0, kbuilt - kmax_ok), sum(s[2] for s in short))
        if best is None or score < best[0]:
            best = (score, d, segs, diag, kbuilt)

        if kbuilt <= kmax_ok and not short:
            break

        if kbuilt > kmax_ok:
            # 넘친 조각의 구간에서만 상한을 깎는다
            tightened = 0
            for a, s in enumerate(segs):
                if s['kappa_max'] <= kmax_ok:
                    continue
                i, j = idx[a], idx[(a + 1) % len(idx)]
                span = np.arange(i - 2, i + ((j - i) % base.n) + 3) % base.n
                cut = s['kappa_max'] - kmax_ok
                ktgt[span] = np.maximum(ktgt[span] - cut * 1.2, 0.2)
                tightened += 1
            if verbose:
                print(f'    [{outer}] 만들어진 κmax {kbuilt:.4f} > {kmax_ok:.4f} '
                      f'-> 조각 {tightened} 곳 상한 축소')
            continue

        for j, nm, gap, dmin in short:
            if boost[j] >= args.boost_max - 1e-9:
                warn.append(f'{nm}: 요구 여유 {args.obs_clear * 100:.1f} cm 를 '
                            f'못 만든다 (최대 {dmin * 100:.1f} cm)')
                continue
            boost[j] = min(boost[j] + max(gap * 1.1, 0.005), args.boost_max)
        if verbose:
            print(f'    [{outer}] 2D 여유 부족 {[s[1] for s in short]} -> boost '
                  f'{np.round(boost * 100, 1)} cm')

    (kover, cover), d, segs, diag, kbuilt = best
    if kover > 1e-6:
        warn.append(f'만들어진 곡률이 {kbuilt:.4f} 1/m 다 (목표 {kmax_ok:.4f})')
    if diag.get('cap_over', 0.0) > 0.005:
        warn.append(f'코리도 한계를 {diag["cap_over"] * 100:.1f} cm 넘긴 자리가 있다')
    for w in dict.fromkeys(warn):
        print(f'  ! {name}: {w}')
    return d, segs, diag


def selftest(base, args, rings, klim):
    """d=0 으로 다시 뽑는다. 재구성 자체가 원본을 얼마나 흔드는지 보는 것이다.

    여기서 어긋나면 오프셋을 얹기 전에 이미 틀린 것이다.
    """
    d = np.zeros(base.n)
    Q, thq, kq = offset_geometry(base, d)
    idx = pick_nodes(base, thq, kq, d, args)
    segs = build_segments(Q, thq, kq, idx)

    pts = curve_points(segs, 0.005)
    dev = [float(np.hypot(base.P[:, 0] - p[0], base.P[:, 1] - p[1]).min()) for p in pts]
    kmax = max(s['kappa_max'] for s in segs)
    total = sum(s['length'] for s in segs)

    print('\n[selftest] d=0 재구성')
    print(f'  노드/조각 {len(segs)} (원본 50)')
    print(f'  길이 {total:.4f} m (원본 {base.total:.4f}, {total - base.total:+.4f})')
    print(f'  |κ|max {kmax:.4f} (원본 {np.abs(base.K).max():.4f}, 한계 {klim:.4f})')
    print(f'  원본 곡선에서 벗어난 정도  중앙값 {np.median(dev) * 1000:.2f} mm · '
          f'최대 {max(dev) * 1000:.2f} mm')
    return 0


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--src', type=Path, default=PKG / 'config' / 'lane_graph.yaml')
    ap.add_argument('--track', type=Path, default=PKG / 'config' / 'amet2026_track.json')
    ap.add_argument('--cones', type=Path, default=CONE_DIR)
    ap.add_argument('--out-dir', type=Path, default=PKG / 'config')
    ap.add_argument('--only', default=None,
                    help='right_bias / last_obstacle / every_obstacle 중 하나만')
    ap.add_argument('--all', action='store_true', help='(기본) 세 개 다 만든다')
    ap.add_argument('--dry-run', action='store_true', help='파일을 쓰지 않는다')
    ap.add_argument('--plot', type=Path, default=None,
                    help='이 디렉터리에 변형마다 PNG 를 그린다')
    ap.add_argument('--selftest', action='store_true',
                    help='d=0 으로 다시 뽑아 원본이 그대로 나오는지만 본다')
    ap.add_argument('-v', '--verbose', action='store_true')

    ap.add_argument('--bias', type=float, default=0.10, help='m. 오른쪽 목표 offset')
    ap.add_argument('--max-offset', type=float, default=0.22, help='m. 횡오프셋 절대 상한')
    ap.add_argument('--cone-radius', type=float, default=0.09,
                    help='m. 콘 반경. 시뮬 콘은 0.18 m 각 박스라 반경 0.09 로 본다. '
                         '실물 라바콘 반경은 아직 실측 전이다')
    ap.add_argument('--obs-margin', type=float, default=0.04,
                    help='m. 콘 여유. 측위 오차를 여기서 흡수한다')
    ap.add_argument('--body-radius', type=float, default=0.110353, help='m')
    ap.add_argument('--edge-margin', type=float, default=0.02,
                    help='m. 차체 반경 위에 더 두는 테두리 여유')
    ap.add_argument('--knot', type=float, default=0.45,
                    help='m. B-spline 매듭 간격. d 가 만들 수 있는 가장 급한 굴곡')
    ap.add_argument('--boost-max', type=float, default=0.15,
                    help='m. 콘 요구 여유를 이만큼까지만 올려 본다')
    ap.add_argument('--kappa-tol', type=float, default=0.005,
                    help='1/m. 곡률 판정 여유 (반올림 몫)')
    ap.add_argument('--maxiter', type=int, default=250)
    ap.add_argument('--ddmax', type=float, default=0.25,
                    help="1/m. |d''| 예산. κQ 에 그대로 더해지므로 이게 곧 여유다")
    ap.add_argument('--dkappa', type=float, default=1.0,
                    help='1/m. 한 조각이 감당할 κ 변화량. 작게 잡아 잘게 나눌수록 '
                         '반드시 좋아지지는 않는다 (조각당 κ 차이가 커진다)')
    ap.add_argument('--w-want', type=float, default=1.0, help='하고 싶은 값 가중치')
    ap.add_argument('--w-kappa', type=float, default=1.0e5,
                    help='곡률은 물리적 한계다 — 콘 여유와 부딪치면 이쪽이 이긴다')
    ap.add_argument('--w-cap', type=float, default=3.0e4)
    ap.add_argument('--w-cone', type=float, default=3.0e4)
    ap.add_argument('--w-smooth', type=float, default=1.0e-3)
    ap.add_argument('--w-dd', type=float, default=3.0e3)
    ap.add_argument('--kappa-final', type=float, default=0.995,
                    help='만들어진 조각의 κ 가 한계의 이 배 안에 들어와야 한다')
    ap.add_argument('--kappa-safety', type=float, default=0.985,
                    help='곡률 한계에 곱하는 안전율 (Bezier 내부 부풀림 몫)')
    ap.add_argument('--max-iter', type=int, default=8,
                    help='바깥 고리 반복. 한 번마다 최적화를 다시 푼다 (약 15 초)')

    ap.add_argument('--dtheta', type=float, default=12.0, help='deg. 노드 간 방향 변화')
    ap.add_argument('--max-step', type=float, default=0.50, help='m')
    ap.add_argument('--min-out', type=float, default=0.02,
                    help='m. 만들어 낼 조각의 최소 길이')
    ap.add_argument('--keep-whole', type=float, default=0.16,
                    help='m. 이보다 짧은 원본 조각은 쪼개지 않는다')

    ap.add_argument('--wheelbase', type=float, default=0.18)
    ap.add_argument('--max-steer', type=float, default=20.0, help='deg')
    ap.add_argument('--margin', type=float, default=0.90)
    ap.add_argument('--track-ox', type=float, default=3.68)
    ap.add_argument('--track-oy', type=float, default=1.39)

    args = ap.parse_args(argv)
    # 콘 중심에서 경로까지 필요한 거리. 콘 + 차체 + 여유.
    args.obs_clear = args.cone_radius + args.body_radius + args.obs_margin
    klim = math.tan(math.radians(args.max_steer)) / args.wheelbase * args.margin

    g, segs = load_layer(args.src)
    base = Base(segs)
    rings = load_road_rings(args.track, args.track_ox, args.track_oy)
    cones = load_cones(args.cones, args.track_ox, args.track_oy)
    i_start = start_station(base, args.track, args.track_ox, args.track_oy)

    print(f'원본 {args.src}')
    print(f'  조각 {len(segs)} · 길이 {base.total:.3f} m · '
          f'|κ|max {np.abs(base.K).max():.4f} (한계 {klim:.4f})')
    print(f'  격자 {base.n} 점 · {base.ds * 1000:.1f} mm')
    print(f'  출발선 station {base.s[i_start]:.3f} m')

    # 주행 순서대로 콘을 세운다. 마지막이 곧 "마지막 장애물" 이다.
    order = []
    for nm, cx, cy in cones:
        i, lat, dd = base.station_of((cx, cy))
        order.append((float((base.s[i] - base.s[i_start]) % base.total), nm, cx, cy, lat))
    order.sort()
    print('  콘 (출발선 기준 주행 순)')
    for run, nm, cx, cy, lat in order:
        print(f'    {nm}  {run:6.3f} m  map ({cx:+.3f}, {cy:+.3f})  '
              f'중심선에서 {"우" if lat > 0 else "좌"} {abs(lat) * 100:.1f} cm')
    last = [(order[-1][1], order[-1][2], order[-1][3])]
    print(f'  -> 마지막 장애물 = {last[0][0]}')

    todo = [args.only] if args.only else ['right_bias', 'last_obstacle', 'every_obstacle']

    if args.selftest:
        return selftest(base, args, rings, klim)

    for name in todo:
        if name == 'right_bias':
            used = []
            d, out, _ = make_variant(name, base, args, rings, used, klim,
                                     args.verbose)
            meta = [f'variant: right_bias  ·  목표 오른쪽 offset {args.bias * 100:.0f} cm',
                    '곡률/코리도 한계에 걸리는 자리에서는 자동으로 줄어든다']
        else:
            used = last if name == 'last_obstacle' else \
                [(nm, cx, cy) for _, nm, cx, cy, _ in order]
            d, out, _ = make_variant(name, base, args, rings, used, klim,
                                     args.verbose)
            meta = [f'variant: {name}  ·  콘 {len(used)} 개 회피 '
                    f'(필요 여유 {args.obs_clear * 100:.1f} cm)',
                    '콘: ' + ', '.join(f'{nm}@({cx:.3f},{cy:.3f})' for nm, cx, cy in used)]

        report(name, base, d, out, used, klim, rings, args)

        if args.plot:
            args.plot.mkdir(parents=True, exist_ok=True)
            png = args.plot / f'{name}_lane.png'
            plot_variant(png, name, base, d, out, used, rings, args)
            print(f'  -> {png}')

        path = args.out_dir / f'{name}_lane.yaml'
        if args.dry_run:
            print(f'  (dry-run) {path} 안 씀')
            continue
        write_yaml(path, g, out, meta + [
            f'원본 {args.src.name} · 노드 {len(out)} · 길이 '
            f'{sum(s["length"] for s in out):.3f} m · |κ|max '
            f'{max(s["kappa_max"] for s in out):.4f} 1/m'])
        print(f'  -> {path}')

    return 0


if __name__ == '__main__':
    sys.exit(main())
