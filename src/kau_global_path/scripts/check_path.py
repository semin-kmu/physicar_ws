#!/usr/bin/env python3
"""lane_graph.yaml 이 발행해도 되는 파일인지 ROS 없이 판정한다.

편집기 화면으로만 판단하면, 발행 노드가 yaml 을 KauPath 로 바꾸는 단계에서
생기는 실수(cm 환산, 제어점 순서, 이음새)를 아무도 못 잡는다. 이 스크립트는
**파일만 보고** 다음을 다시 계산해서 대조한다.

    - 조각마다 제어점이 6 개인가 (팀 약속: 전부 5차 Bezier)
    - 적어 둔 seg_length / kappa_max 가 제어점에서 다시 계산한 값과 맞는가
    - 이음새가 G0 / G1 / G2 로 이어지는가 (폐곡선이면 마지막->처음까지)
    - 곡률이 차량 한계 안에 있는가 (kinematic bicycle, 후륜축 중심)
    - 트랙 밖으로 나가지 않는가 (amet2026_track.json 이 있을 때)
    - KauPath 로 환산했을 때의 값 (cm 단위. seg_kappa_max 만 나누기다)

쓰는 법

    python3 src/kau_global_path/scripts/check_path.py
    python3 src/kau_global_path/scripts/check_path.py --plot /tmp/path.png
    python3 src/kau_global_path/scripts/check_path.py --route center_loop -v

통과하면 0, 하나라도 실패하면 1 을 반환한다. CI 에 그대로 걸 수 있다.
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

try:
    import yaml
except ImportError:                                     # pragma: no cover
    sys.exit('PyYAML 이 없다:  pip install pyyaml')


# ---------------------------------------------------------------- Bezier

# 편집기(`web/lane_editor.js`)와 같은 식이어야 한다. 여기서 다르면 이 검사가
# 검사 자체를 못 한다.

GL10_X = [-0.9739065285171717, -0.8650633666889845, -0.6794095682990244,
          -0.4333953941292472, -0.1488743389816312, 0.1488743389816312,
          0.4333953941292472, 0.6794095682990244, 0.8650633666889845,
          0.9739065285171717]
GL10_W = [0.0666713443086881, 0.1494513491505806, 0.2190863625159820,
          0.2692667193099963, 0.2955242247147529, 0.2955242247147529,
          0.2692667193099963, 0.2190863625159820, 0.1494513491505806,
          0.0666713443086881]


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


def seg_length(ctrl):
    d1 = hodograph(ctrl)
    total = 0.0
    for x, w in zip(GL10_X, GL10_W):
        v = de_casteljau(d1, 0.5 * (x + 1))
        total += w * math.hypot(v[0], v[1])
    return total * 0.5


def kappa_at(ctrl, u):
    d1 = hodograph(ctrl)
    d2 = hodograph(d1)
    v = de_casteljau(d1, u)
    a = de_casteljau(d2, u)
    sp = math.hypot(v[0], v[1])
    if sp < 1e-12:
        return 0.0
    return (v[0] * a[1] - v[1] * a[0]) / (sp ** 3)


def seg_kappa_max(ctrl, samples=200):
    return max(abs(kappa_at(ctrl, i / samples)) for i in range(samples + 1))


def theta_at(ctrl, u):
    v = de_casteljau(hodograph(ctrl), u)
    return math.atan2(v[1], v[0])


def dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def wrap_pi(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


# ---------------------------------------------------------------- 리포트

class Report:
    """통과/실패를 세면서 그대로 찍는다. 요약만 내면 어디가 틀렸는지 못 찾는다."""

    OK, BAD, INFO = '통과', '실패', '    '

    def __init__(self, verbose=False):
        self.failed = 0
        self.verbose = verbose

    def head(self, text):
        print(f'\n{text}')

    def line(self, text):
        print(f'    {text}')

    def check(self, name, ok, detail=''):
        if not ok:
            self.failed += 1
        mark = self.OK if ok else self.BAD
        print(f'  {mark}  {name}' + (f'   {detail}' if detail else ''))


# ---------------------------------------------------------------- 검사

def load(path: Path):
    doc = yaml.safe_load(path.read_text())
    if not isinstance(doc, dict) or 'lane_graph' not in doc:
        raise SystemExit(f'{path}: lane_graph 블록이 없다')
    return doc['lane_graph']


def pick_route(g, want):
    """어느 레이어를 검사할지 고른다. 기본은 center_loop."""
    routes = g.get('routes') or {}
    bez = g.get('bezier') or {}

    if want:
        for layer, segs in bez.items():
            if want in routes and want.startswith(layer.split('_')[0]):
                return layer, want, segs
        if want in bez:
            return want, want, bez[want]
        raise SystemExit(f'route/레이어 "{want}" 가 파일에 없다. '
                         f'있는 것: routes {list(routes)} · bezier {list(bez)}')

    for prefer in ('center', 'lane_inner', 'lane_outer'):
        if prefer in bez and bez[prefer]:
            route = next((r for r in routes if r.startswith(prefer.split('_')[0])), None)
            return prefer, route, bez[prefer]

    raise SystemExit(f'bezier 블록이 비었다. 있는 키: {list(bez)}')


def check_shape(rep, segs, degree):
    """조각마다 제어점이 degree+1 개인가."""
    nctrl = degree + 1
    bad = [i for i, s in enumerate(segs) if len(s.get('ctrl') or []) != nctrl]
    rep.check(f'모든 조각의 제어점이 {nctrl} 개 ({degree}차 Bezier)',
              not bad,
              f'{len(segs)} 조각 · 제어점 {len(segs) * nctrl} 개'
              if not bad else f'어긋난 조각 {bad[:8]}')

    flat = [q for s in segs for q in (s.get('ctrl') or [])]
    bad2 = [i for i, q in enumerate(flat) if not (isinstance(q, list) and len(q) == 2)]
    rep.check('제어점이 전부 [x, y] 쌍', not bad2,
              f'{len(flat)} 개' if not bad2 else f'어긋난 자리 {bad2[:8]}')


def check_precomputed(rep, segs, tol_len, tol_k):
    """적어 둔 length / kappa_max 를 제어점에서 다시 계산해 대조한다.

    발행 노드는 이 값을 그대로 KauPath 에 싣는다. 여기가 틀리면 수신측이
    호길이로 하는 계산(추종 지점 찾기 등)이 전부 어긋난다.
    """
    worst_len = worst_k = 0.0
    at_len = at_k = -1

    for i, s in enumerate(segs):
        ctrl = s['ctrl']
        dl = abs(seg_length(ctrl) - float(s.get('length', 0.0)))
        dk = abs(seg_kappa_max(ctrl) - float(s.get('kappa_max', 0.0)))
        if dl > worst_len:
            worst_len, at_len = dl, i
        if dk > worst_k:
            worst_k, at_k = dk, i

    rep.check('적어 둔 length 가 제어점과 맞는다', worst_len < tol_len,
              f'최대 차이 {worst_len * 1000:.3f} mm (조각 #{at_len})')
    rep.check('적어 둔 kappa_max 가 제어점과 맞는다', worst_k < tol_k,
              f'최대 차이 {worst_k:.6f} 1/m (조각 #{at_k})')


def check_continuity(rep, segs, closed, tol_g0, tol_g1_deg, tol_g2):
    """이음새. G0 는 위치, G1 은 접선, G2 는 곡률이다.

    폐곡선이면 마지막 조각 -> 첫 조각까지 본다. 여기를 빼먹으면 시작점에서
    heading 이 튀는 걸 놓친다 (README 6.5).
    """
    n = len(segs)
    last = n if closed else n - 1

    worst = {'g0': (-1.0, 0), 'g1': (-1.0, 0), 'g2': (-1.0, 0)}

    for i in range(last):
        a = segs[i]['ctrl']
        b = segs[(i + 1) % n]['ctrl']

        g0 = dist(a[-1], b[0])
        g1 = abs(math.degrees(wrap_pi(theta_at(b, 0.0) - theta_at(a, 1.0))))
        g2 = abs(kappa_at(b, 0.0) - kappa_at(a, 1.0))

        for key, v in (('g0', g0), ('g1', g1), ('g2', g2)):
            if v > worst[key][0]:
                worst[key] = (v, i)

    rep.check('G0 — 이음새가 붙어 있다', worst['g0'][0] < tol_g0,
              f'최대 {max(0.0, worst["g0"][0]) * 1000:.4f} mm (조각 #{worst["g0"][1]} 뒤)')
    rep.check('G1 — 접선이 이어진다', worst['g1'][0] < tol_g1_deg,
              f'최대 Δθ {worst["g1"][0]:.4f}° (조각 #{worst["g1"][1]} 뒤)')
    rep.check('G2 — 곡률이 이어진다', worst['g2'][0] < tol_g2,
              f'최대 Δκ {worst["g2"][0]:.6f} 1/m (조각 #{worst["g2"][1]} 뒤)')

    if closed:
        rep.line(f'폐곡선이라 마지막 조각 #{n - 1} -> 첫 조각 #0 까지 봤다')
    else:
        rep.line('열린 곡선 — 마지막->처음 이음새는 검사 대상이 아니다')


def check_curvature(rep, segs, limit, kmax_geom, wheelbase):
    kmax = 0.0
    at = -1
    over = []

    for i, s in enumerate(segs):
        k = seg_kappa_max(s['ctrl'])
        if k > kmax:
            kmax, at = k, i
        if k > limit:
            over.append((i, s.get('type', '?'), k))

    steer = math.degrees(math.atan(kmax * wheelbase))
    rep.check(f'곡률이 한계 {limit:.4f} 1/m 안에 있다', not over,
              f'|κ|max {kmax:.4f} (R {1 / kmax if kmax else float("inf"):.3f} m · '
              f'δ {steer:.2f}° · 한계의 {kmax / limit * 100:.0f}%) · 조각 #{at}')

    for i, t, k in over[:10]:
        rep.line(f'  초과: 조각 #{i} ({t}) κ {k:.4f} → δ {math.degrees(math.atan(k * wheelbase)):.2f}°')
    if len(over) > 10:
        rep.line(f'  … 외 {len(over) - 10} 개')

    rep.line(f'기하학적 한계는 κ {kmax_geom:.4f} (R {1 / kmax_geom:.4f} m), '
             f'실제 마진 {100 * (1 - kmax / kmax_geom):.1f} %')
    return kmax


def sample_curve(segs, step=0.02):
    """호길이 대략 step 간격으로 곡선 위 점을 깐다. 트랙 판정과 그림용."""
    out = []
    for s in segs:
        ctrl = s['ctrl']
        n = max(2, int(seg_length(ctrl) / step))
        for i in range(n):
            out.append(de_casteljau(ctrl, i / n))
    return out


def nearest_dist(poly, p):
    best = float('inf')
    n = len(poly)
    for i in range(n):
        ax, ay = poly[i]
        bx, by = poly[(i + 1) % n]
        dx, dy = bx - ax, by - ay
        l2 = dx * dx + dy * dy
        t = 0.0 if l2 < 1e-18 else max(0.0, min(1.0, ((p[0] - ax) * dx + (p[1] - ay) * dy) / l2))
        best = min(best, math.hypot(p[0] - (ax + t * dx), p[1] - (ay + t * dy)))
    return best


def check_corridor(rep, pts, track_path: Path, ox, oy):
    """CAD 주행면 안에 있는지. 트랙 파일이 없으면 조용히 건너뛴다."""
    if not track_path.is_file():
        rep.line(f'{track_path.name} 이 없어 트랙 판정은 건너뛴다')
        return

    import json
    doc = json.loads(track_path.read_text())
    rings = (doc.get('layers', {}).get('road', {}) or {}).get('rings')
    if not rings:
        rep.line('트랙 파일에 road 고리가 없어 건너뛴다')
        return

    # sim -> map. extract_sim_track.py 와 같은 변환이다.
    to_map = lambda q: (ox - q[1], q[0] - oy)
    edges = [[to_map(q) for q in r] for r in rings]

    worst = float('inf')
    at = None
    for p in pts:
        d = min(nearest_dist(e, p) for e in edges)
        if d < worst:
            worst, at = d, p

    rep.check('주행면 테두리에 닿지 않는다', worst > 0.05,
              f'최소 여유 {worst * 100:.1f} cm (map {at[0]:.2f}, {at[1]:.2f})')


def kaupath_preview(rep, segs, closed, degree):
    """KauPath 로 실을 값. 단위 실수를 여기서 한 번 보여 준다."""
    total_cm = sum(seg_length(s['ctrl']) for s in segs) * 100.0
    kmax_per_cm = max(seg_kappa_max(s['ctrl']) for s in segs) / 100.0
    nseg = len(segs)

    rep.head('KauPath 환산 (kau_msgs/KauPath 는 전부 cm 다)')
    rep.line(f'source        SRC_GLOBAL (0)')
    rep.line(f'degree        {degree}')
    rep.line(f'is_closed     {str(closed).lower()}')
    rep.line(f's_offset      0.0')
    rep.line(f'ctrl_x/ctrl_y 길이 {nseg * (degree + 1)}  = nseg {nseg} x {degree + 1}   [m x 100]')
    rep.line(f'seg_length    길이 {nseg}                        [m x 100]')
    rep.line(f'seg_kappa_max 길이 {nseg}, 최대 {kmax_per_cm:.6f}  [1/m ÷ 100]  <- 나누기다')
    rep.line(f'total_length  {total_cm:.2f} cm')
    rep.line(f'confidence    1.0        valid_length  0.0')


def plot(path, segs, nodes, track_path: Path, ox, oy, out: Path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon

    fig, ax = plt.subplots(figsize=(7.5, 11.5))

    if track_path.is_file():
        import json
        doc = json.loads(track_path.read_text())
        to_map = lambda q: (ox - q[1], q[0] - oy)
        for name, col, z in [('road', '#d8dce4', 2), ('outer_line', '#8899aa', 3),
                             ('inner_line', '#8899aa', 3), ('center_line', '#e0b000', 4)]:
            layer = doc.get('layers', {}).get(name)
            if not layer:
                continue
            for ring in layer['rings']:
                ax.add_patch(Polygon([to_map(q) for q in ring], closed=True,
                                     fc=col, ec='none', zorder=z))

    pts = sample_curve(segs, 0.01)
    ax.plot([p[0] for p in pts] + [pts[0][0]], [p[1] for p in pts] + [pts[0][1]],
            '-', color='#1a7fd4', lw=2, zorder=6, label='global path')
    ax.plot([n['x'] for n in nodes], [n['y'] for n in nodes], 'o', ms=3,
            color='#d41a5a', zorder=7, label=f'nodes ({len(nodes)})')
    for s in segs:
        c = s['ctrl']
        ax.plot([q[0] for q in c], [q[1] for q in c], '-', color='#ffd166',
                lw=0.6, alpha=0.7, zorder=5)

    ax.set_aspect('equal')
    ax.grid(alpha=0.25)
    ax.legend(fontsize=8)
    ax.set_title(path.name, fontsize=9)
    fig.tight_layout()
    fig.savefig(out, dpi=110)
    print(f'\n  그림 -> {out}')


# ---------------------------------------------------------------- main

def main(argv=None):
    pkg = Path(__file__).resolve().parents[1]

    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('yaml', nargs='?', type=Path,
                    default=pkg / 'config' / 'lane_graph.yaml')
    ap.add_argument('--track', type=Path, default=pkg / 'config' / 'amet2026_track.json')
    ap.add_argument('--route', default=None, help='검사할 route 또는 레이어 이름')
    ap.add_argument('--plot', type=Path, default=None, help='PNG 로 그려서 저장')
    ap.add_argument('-v', '--verbose', action='store_true', help='조각을 전부 나열')

    ap.add_argument('--wheelbase', type=float, default=0.18, help='m')
    ap.add_argument('--max-steer', type=float, default=20.0, help='deg')
    ap.add_argument('--margin', type=float, default=0.90)

    # 허용오차는 **파일 자체의 반올림 바닥보다 위**여야 한다. 제어점을 소수 7 자리로
    # 적으면 짧은 조각(전이 2.4 cm, 제어점 간격 5 mm)에서 kappa 가 0.002 1/m 쯤
    # 흔들린다 — 이건 못 없앤다. 한계 1.82 대비 0.1 % 라 물리적으로 무의미하다.
    # 대신 0.005 (한계의 0.3 %) 를 넘으면 진짜 문제이므로 거기서 자른다.
    ap.add_argument('--tol-len', type=float, default=1e-3, help='length 대조 허용 m (기본 1 mm)')
    ap.add_argument('--tol-kappa', type=float, default=5e-3, help='kappa 대조 허용 1/m')
    ap.add_argument('--tol-g0', type=float, default=1e-4, help='이음새 허용 m (기본 0.1 mm)')
    ap.add_argument('--tol-g1', type=float, default=0.05, help='접선 허용 deg')
    ap.add_argument('--tol-g2', type=float, default=5e-3, help='곡률 허용 1/m')
    ap.add_argument('--track-ox', type=float, default=3.68)
    ap.add_argument('--track-oy', type=float, default=1.39)

    args = ap.parse_args(argv)

    if not args.yaml.is_file():
        raise SystemExit(f'{args.yaml} 이 없다')

    g = load(args.yaml)
    layer, route, segs = pick_route(g, args.route)
    closed = bool((g.get('closed') or {}).get(layer, True))
    nodes = [n for n in (g.get('nodes') or []) if isinstance(n, dict)]
    degree = len((segs[0].get('ctrl') or [])) - 1 if segs else 5

    kmax_geom = math.tan(math.radians(args.max_steer)) / args.wheelbase
    limit = kmax_geom * args.margin

    rep = Report(args.verbose)

    print(f'{args.yaml}')
    print(f'  레이어 {layer}  ·  route {route or "—"}  ·  '
          f'폐곡선 {closed}  ·  지도 {g.get("map_source", "?")}')
    print(f'  노드 {len(nodes)} 개  ·  조각 {len(segs)} 개  ·  '
          f'총 길이 {sum(seg_length(s["ctrl"]) for s in segs):.3f} m')
    print(f'  차량 L {args.wheelbase} m · δ {args.max_steer}° · 마진 {args.margin} '
          f'→ 한계 κ {limit:.4f} 1/m (R {1 / limit:.4f} m)')
    print(f'  허용오차  길이 {args.tol_len * 1000:g} mm · κ {args.tol_kappa:g} 1/m '
          f'(한계의 {args.tol_kappa / limit * 100:.2f}%) · '
          f'G0 {args.tol_g0 * 1000:g} mm · G1 {args.tol_g1:g}° · G2 {args.tol_g2:g} 1/m')

    rep.head('[1] 형식 — 팀 약속은 제어점 6 개짜리 5차 Bezier 다')
    check_shape(rep, segs, degree)

    rep.head('[2] 사전계산값 — 발행 노드가 그대로 싣는 값이다')
    check_precomputed(rep, segs, args.tol_len, args.tol_kappa)

    rep.head('[3] 연속성 — 이음새에서 위치·접선·곡률')
    check_continuity(rep, segs, closed, args.tol_g0, args.tol_g1, args.tol_g2)

    rep.head('[4] 주행 가능성')
    check_curvature(rep, segs, limit, kmax_geom, args.wheelbase)
    check_corridor(rep, sample_curve(segs, 0.02), args.track, args.track_ox, args.track_oy)

    if args.verbose:
        rep.head('[조각 목록]')
        for i, s in enumerate(segs):
            k = seg_kappa_max(s['ctrl'])
            rep.line(f'#{i:3d} {s.get("type", "?"):5s} n{s.get("from")}->n{s.get("to")}  '
                     f'{seg_length(s["ctrl"]):6.3f} m  κ {k:7.4f}'
                     + ('  <- 한계 초과' if k > limit else ''))

    kaupath_preview(rep, segs, closed, degree)

    if args.plot:
        plot(args.yaml, segs, nodes, args.track, args.track_ox, args.track_oy, args.plot)

    print()
    if rep.failed:
        print(f'실패 {rep.failed} 건 — 이 파일은 발행하면 안 된다\n')
        return 1
    print('전부 통과 — 발행해도 되는 파일이다\n')
    return 0


if __name__ == '__main__':
    sys.exit(main())
