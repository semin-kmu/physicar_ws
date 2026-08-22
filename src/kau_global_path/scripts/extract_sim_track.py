#!/usr/bin/env python3
"""physicar-sim 의 AMET 2026 world 에서 트랙 도형을 뽑아 map 프레임으로 옮긴다.

편집기(`web/lane_editor.html`)가 배경으로 깔 참조 도형을 만든다. 지도
(`kau_v3.pgm`)에는 벽밖에 안 찍힌다 — 차선은 바닥 테이프라 LiDAR 평면에 안
걸린다. 그래서 노드를 찍을 기준이 주행 기록밖에 없었는데, 시뮬레이터 월드에
**같은 트랙의 CAD 가 통째로 들어 있다.**

    /opt/physicar/src/physicar-sim/share/
      worlds/custom_<id>.pcpub.json          {"name": "AMET 2026", "size": [12, 7]}
      meshes/custom_<id>/custom_<id>.dae     road / ol / il / cl / start_line 지오메트리

메시는 삼각형 수프라 그대로는 못 쓴다. **한 번만 등장하는 변**(= 경계 변)을
모아 고리로 이어 붙이면 각 도형의 윤곽선이 나온다. 채워 그리면 CAD 그대로다.

좌표계
    sim 은 방 구석이 원점이고 12 m 쪽이 x, 7 m 쪽이 y 다.
    map 은 Cartographer 가 잡은 프레임이라 축이 90도 돌아 있다.

        map_x = ox - sim_y
        map_y = sim_x - oy          (기본 ox = 3.68, oy = 1.39)

    이 값은 kau_v3 의 벽 경계상자(x [-3.49, 3.66] · y [-1.41, 10.74],
    7.15 x 12.15 m)와 sim 벽(12.08 x 7.08 m)을 맞춘 것이다. 잔차가 10 cm
    안쪽이라 편집기에서 미세조정만 하면 된다 — 그래서 이 파일에는 **sim 원좌표**
    를 담고 변환은 편집기가 건다. 나중에 재매핑해도 이 파일은 안 바뀐다 (6.3).

쓰는 법

    python3 src/kau_global_path/scripts/extract_sim_track.py

    기본값으로 src/kau_global_path/config/amet2026_track.json 을 쓴다.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import xml.etree.ElementTree as ET
from collections import defaultdict
from pathlib import Path

SIM_ROOT = Path('/opt/physicar/src/physicar-sim/share')
COLLADA = '{http://www.collada.org/2005/11/COLLADASchema}'

# world 이름이 "AMET 2026" 인 것을 고른다. id 는 언제든 바뀔 수 있다.
WORLD_NAME = 'AMET 2026'

# 뽑을 지오메트리와 편집기에서 쓸 이름. dae 안의 id 는 Blender 가 붙인 것이라
# 그대로 쓰면 나중에 못 알아본다.
WANTED = [
    ('road', 'road-mesh', '주행면'),
    ('outer_line', 'ol_001-mesh', '바깥 차선'),
    ('inner_line', 'il_001-mesh', '안쪽 차선'),
    ('center_line', 'cl_001-mesh', '중앙선 (점선)'),
    ('start_line', 'start_line-mesh', '출발선'),
]

DEFAULT_OX = 3.68
DEFAULT_OY = 1.39


# ---------------------------------------------------------------- COLLADA

def find_world(root: Path) -> tuple[str, dict]:
    """이름으로 world 를 찾아 (id, 메타) 를 준다."""
    for js in sorted((root / 'worlds').glob('*.pcpub.json')):
        try:
            meta = json.loads(js.read_text())
        except (OSError, ValueError):
            continue
        if meta.get('name') == WORLD_NAME:
            return meta.get('world_id', js.name.split('.')[0]), meta
    raise SystemExit(f'{root}/worlds 에 "{WORLD_NAME}" world 가 없다')


def triangles(mesh_root: ET.Element, gid: str):
    """geometry 하나의 삼각형 목록. [(x,y), (x,y), (x,y)] 의 리스트."""
    for geom in mesh_root.iter(COLLADA + 'geometry'):
        if geom.get('id') != gid:
            continue

        mesh = geom.find(COLLADA + 'mesh')
        src_id = None
        for inp in mesh.find(COLLADA + 'vertices').findall(COLLADA + 'input'):
            if inp.get('semantic') == 'POSITION':
                src_id = inp.get('source')[1:]

        pos = None
        for src in mesh.findall(COLLADA + 'source'):
            if src.get('id') == src_id:
                flat = [float(v) for v in src.find(COLLADA + 'float_array').text.split()]
                pos = [(flat[i], flat[i + 1]) for i in range(0, len(flat), 3)]

        if pos is None:
            return []

        out = []
        for tag in ('triangles', 'polylist', 'polygons'):
            for prim in mesh.findall(COLLADA + tag):
                inputs = prim.findall(COLLADA + 'input')
                stride = len(inputs)
                offset = 0
                for inp in inputs:
                    if inp.get('semantic') == 'VERTEX':
                        offset = int(inp.get('offset'))

                raw = [int(v) for v in prim.find(COLLADA + 'p').text.split()]
                idx = raw[offset::stride]

                if tag == 'polylist':
                    counts = [int(v) for v in prim.find(COLLADA + 'vcount').text.split()]
                    k = 0
                    for n in counts:
                        face = idx[k:k + n]
                        k += n
                        for j in range(1, n - 1):
                            out.append([pos[face[0]], pos[face[j]], pos[face[j + 1]]])
                else:
                    for j in range(0, len(idx) - 2, 3):
                        out.append([pos[idx[j]], pos[idx[j + 1]], pos[idx[j + 2]]])

        return out

    return []


def outlines(tris, weld=1e-4):
    """삼각형 수프의 윤곽 고리들.

    한 번만 등장하는 변이 경계 변이다. 그 변들을 이어 붙이면 도형의 테두리가
    나온다. 정점은 weld 만큼 반올림해서 같은 점으로 묶는다 — 부동소수 오차로
    갈라지면 고리가 안 닫힌다.
    """
    key = lambda p: (round(p[0] / weld), round(p[1] / weld))

    pts = {}
    count = defaultdict(int)
    ends = {}

    for tri in tris:
        ks = [key(p) for p in tri]
        for p, k in zip(tri, ks):
            pts[k] = p
        for i in range(3):
            a, b = ks[i], ks[(i + 1) % 3]
            e = (a, b) if a < b else (b, a)
            count[e] += 1
            ends[e] = (a, b)

    adj = defaultdict(list)
    for e, c in count.items():
        if c != 1:
            continue
        a, b = ends[e]
        adj[a].append(b)
        adj[b].append(a)

    seen = set()
    rings = []

    for start in adj:
        if start in seen:
            continue
        loop = [start]
        seen.add(start)
        cur, prev = start, None
        while True:
            nxt = [v for v in adj[cur] if v != prev and v not in seen]
            if not nxt:
                break
            prev, cur = cur, nxt[0]
            seen.add(cur)
            loop.append(cur)
        if len(loop) >= 3:
            rings.append([pts[k] for k in loop])

    return rings


# ---------------------------------------------------------------- 단순화

def rdp(pts, tol):
    """Douglas-Peucker. 메시 테셀레이션이 남긴 일직선 위 점들을 걷어낸다."""
    if len(pts) < 3:
        return list(pts)

    keep = [False] * len(pts)
    keep[0] = keep[-1] = True
    stack = [(0, len(pts) - 1)]

    while stack:
        i, j = stack.pop()
        if j <= i + 1:
            continue
        ax, ay = pts[i]
        bx, by = pts[j]
        dx, dy = bx - ax, by - ay
        norm = math.hypot(dx, dy)
        best, at = -1.0, -1
        for k in range(i + 1, j):
            px, py = pts[k]
            if norm < 1e-12:
                d = math.hypot(px - ax, py - ay)
            else:
                d = abs(dx * (ay - py) - (ax - px) * dy) / norm
            if d > best:
                best, at = d, k
        if best > tol:
            keep[at] = True
            stack.append((i, at))
            stack.append((at, j))

    return [p for p, k in zip(pts, keep) if k]


# ---------------------------------------------------------------- 차로 중심선

def poly_length(pts, closed=True):
    total = 0.0
    n = len(pts)
    last = n if closed else n - 1
    for i in range(last):
        a, b = pts[i], pts[(i + 1) % n]
        total += math.hypot(b[0] - a[0], b[1] - a[1])
    return total


def order_dashes(rings):
    """중앙선 점선 조각들의 무게중심을 고리 순서로 늘어놓는다.

    조각 간격(10 cm)이 트랙이 자기 자신과 가까워지는 거리(>40 cm)보다 훨씬
    작아서 최근접 체인으로 안전하게 이어진다. 간격이 튀면 건너뛴 것이니
    호출한 쪽에서 알 수 있게 최대 간격도 같이 준다.
    """
    cen = []
    for r in rings:
        cen.append((sum(p[0] for p in r) / len(r), sum(p[1] for p in r) / len(r)))

    n = len(cen)
    used = [False] * n
    order = [0]
    used[0] = True
    gap = 0.0

    for _ in range(n - 1):
        cx, cy = cen[order[-1]]
        best, bd = -1, 1e18
        for j in range(n):
            if used[j]:
                continue
            d = math.hypot(cen[j][0] - cx, cen[j][1] - cy)
            if d < bd:
                bd, best = d, j
        gap = max(gap, bd)
        order.append(best)
        used[best] = True

    return [cen[i] for i in order], gap


def project(poly, p):
    """닫힌 폴리라인 위에서 p 에 가장 가까운 점과 거리."""
    best, bd = poly[0], 1e18
    n = len(poly)
    for i in range(n):
        ax, ay = poly[i]
        bx, by = poly[(i + 1) % n]
        dx, dy = bx - ax, by - ay
        l2 = dx * dx + dy * dy
        t = 0.0 if l2 < 1e-18 else max(0.0, min(1.0, ((p[0] - ax) * dx + (p[1] - ay) * dy) / l2))
        qx, qy = ax + t * dx, ay + t * dy
        d = math.hypot(p[0] - qx, p[1] - qy)
        if d < bd:
            bd, best = d, (qx, qy)
    return best, bd


def lane_center(mid, edge):
    """중앙선과 주행면 테두리의 중간 = 차로 중심선.

    법선 오프셋 대신 **테두리로 투영한 점과의 중간**을 쓴다. 오프셋은 오목한
    모서리에서 자기 자신과 교차해서 고리를 만들고, 그걸 걷어내는 코드가 이
    스크립트의 절반을 차지하게 된다. 투영 방식은 직선 구간에서 오차가 mm 미만이고
    모서리에서만 1~2 cm 어긋나는데, 그 모서리들은 어차피 차가 못 도는 각이라
    (아래 kappa 참고) 사람이 다시 다듬을 자리다.
    """
    out = []
    width = []
    for p in mid:
        q, d = project(edge, p)
        out.append(((p[0] + q[0]) / 2, (p[1] + q[1]) / 2))
        width.append(d)
    return out, width


def fit_line_res(pts):
    """최소제곱 직선까지의 최대 거리."""
    n = len(pts)
    mx = sum(p[0] for p in pts) / n
    my = sum(p[1] for p in pts) / n
    sxx = syy = sxy = 0.0
    for x, y in pts:
        dx, dy = x - mx, y - my
        sxx += dx * dx
        syy += dy * dy
        sxy += dx * dy
    # 주축 방향 = 공분산 행렬의 최대 고윳값 벡터
    theta = 0.5 * math.atan2(2 * sxy, sxx - syy)
    ux, uy = math.cos(theta), math.sin(theta)
    return max(abs((x - mx) * uy - (y - my) * ux) for x, y in pts)


def fit_circle_res(pts):
    """대수적 원 맞춤. (중심, 반지름, 최대 잔차) 를 준다."""
    n = len(pts)
    sx = sy = sxx = syy = sxy = sz = szx = szy = 0.0
    for x, y in pts:
        z = x * x + y * y
        sx += x; sy += y; sxx += x * x; syy += y * y; sxy += x * y
        sz += z; szx += z * x; szy += z * y
    a = [[sxx, sxy, sx], [sxy, syy, sy], [sx, sy, float(n)]]
    b = [szx, szy, sz]

    # 3x3 가우스 소거
    for i in range(3):
        piv = max(range(i, 3), key=lambda r: abs(a[r][i]))
        if abs(a[piv][i]) < 1e-14:
            return None, float('inf'), float('inf')
        a[i], a[piv] = a[piv], a[i]
        b[i], b[piv] = b[piv], b[i]
        for r in range(i + 1, 3):
            f = a[r][i] / a[i][i]
            for c in range(i, 3):
                a[r][c] -= f * a[i][c]
            b[r] -= f * b[i]
    sol = [0.0] * 3
    for i in (2, 1, 0):
        sol[i] = (b[i] - sum(a[i][c] * sol[c] for c in range(i + 1, 3))) / a[i][i]

    cx, cy = sol[0] / 2, sol[1] / 2
    r2 = sol[2] + cx * cx + cy * cy
    if r2 <= 0:
        return None, float('inf'), float('inf')
    R = math.sqrt(r2)
    res = max(abs(math.hypot(x - cx, y - cy) - R) for x, y in pts)
    return (cx, cy), R, res


def primitives(ring, tol):
    """닫힌 점열을 직선과 원호로 쪼갠다.

    탐욕적으로 간다 — 자리마다 직선을 최대한 늘려 보고, 원호도 최대한 늘려 보고,
    더 멀리 가는 쪽을 택한다. 잔차 한계 tol 을 넘지 않는 한 계속 늘린다.
    곡률을 미분해서 문턱값으로 자르는 방식보다 튼튼하다 — 10 cm 간격에서
    2차 미분은 mm 단위 잡음에도 통째로 무너진다.
    """
    n = len(ring)
    get = lambda i: ring[i % n]

    # 가장 긴 직선의 한가운데에서 시작한다. 호 한복판에서 시작하면 그 호가
    # 두 조각으로 갈라진다.
    best_i, best_run = 0, 0
    i = 0
    while i < n:
        j = i + 2
        while j - i < n and fit_line_res([get(t) for t in range(i, j + 1)]) <= tol:
            j += 1
        if j - i > best_run:
            best_run, best_i = j - i, i
        i += max(1, (j - i) // 2)
    start = (best_i + best_run // 2) % n

    out = []
    pos = 0
    while pos < n:
        i = start + pos

        j = i + 1
        while (j - i) < (n - pos) and fit_line_res([get(t) for t in range(i, j + 2)]) <= tol:
            j += 1

        k = i + 2
        if (k - i) <= (n - pos):
            while (k - i) < (n - pos) and fit_circle_res([get(t) for t in range(i, k + 2)])[2] <= tol:
                k += 1
        else:
            k = i

        if (j - i) >= (k - i):
            span = j - i
            out.append({'type': 'line', 'i0': i % n, 'n': span + 1})
        else:
            span = k - i
            c, R, _ = fit_circle_res([get(t) for t in range(i, k + 1)])
            # 부호: 왼쪽으로 돌면 +
            a, b, d = get(i), get(i + span // 2), get(i + span)
            cr = ((b[0] - a[0]) * (d[1] - b[1]) - (b[1] - a[1]) * (d[0] - b[0]))
            out.append({'type': 'arc', 'i0': i % n, 'n': span + 1,
                        'R': round(R, 4), 'kappa': round((1 if cr >= 0 else -1) / R, 4)})

        pos += span

    return out


# ---------------------------------------------------------------- 골격

# 중심선은 매끈한 곡선이 아니다. **곧은 구간과 뾰족한 꼭짓점으로 된 다각형**에
# 완만한 원호 몇 개가 섞인 모양이다. 실측한 꼭짓점 꺾임각은 17 ~ 90 도다.
# 차(최소회전반경 0.494 m)는 그 꼭짓점을 그대로 못 돈다 — 반드시 둥글려야 한다.
#
# 그래서 조각 목록을 그대로 내보내지 않고 **골격**으로 정리한다.
#   corner : 뾰족한 꼭짓점. 위치와 꺾임각만 준다. 둥글리는 반지름은 소비자가 정한다.
#   arc    : 진짜 원호 구간. 양 끝과 가운데 점, 반지름.
# 둘 사이는 직선이다 (따로 안 적는다).

CORNER_MIN_R = 0.25       # 이보다 반지름이 작은 '원호' 는 사실 꼭짓점이다
CORNER_MIN_DEG = 2.0      # 이보다 작은 꺾임은 무시한다


def line_fit(pts):
    """최소제곱 직선. (지나는 점, 단위 방향)."""
    n = len(pts)
    mx = sum(q[0] for q in pts) / n
    my = sum(q[1] for q in pts) / n
    sxx = syy = sxy = 0.0
    for x, y in pts:
        dx, dy = x - mx, y - my
        sxx += dx * dx
        syy += dy * dy
        sxy += dx * dy
    th = 0.5 * math.atan2(2 * sxy, sxx - syy)
    ux, uy = math.cos(th), math.sin(th)
    # 진행 방향과 같은 쪽을 보게 뒤집는다
    if (pts[-1][0] - pts[0][0]) * ux + (pts[-1][1] - pts[0][1]) * uy < 0:
        ux, uy = -ux, -uy
    return (mx, my), (ux, uy)


def line_isect(a, ua, b, ub):
    """두 직선의 교점. 평행이면 None."""
    den = ua[0] * ub[1] - ua[1] * ub[0]
    if abs(den) < 1e-9:
        return None
    t = ((b[0] - a[0]) * ub[1] - (b[1] - a[1]) * ub[0]) / den
    return (a[0] + t * ua[0], a[1] + t * ua[1])


def project_line(a, u, p):
    t = (p[0] - a[0]) * u[0] + (p[1] - a[1]) * u[1]
    return (a[0] + t * u[0], a[1] + t * u[1])


def skeleton(ring, prims):
    """조각 목록 -> 꼭짓점 나열 (다각형).

    **중심선은 곡선이 아니라 다각형이다.** 조각 맞춤이 뱉는 '원호' 는 거의 다
    10 cm 간격으로 샘플된 꼭짓점을 원으로 오인한 것이다 — 실제로 이웃 직선에
    접하지도 않는다 (중심-직선 거리가 R 보다 20 ~ 125 cm 작다). 그래서 원호는
    버리고 **직선 조각들의 교점**만 남긴다. 직선 맞춤은 잔차가 1 mm 아래라
    믿을 만하고, 교점이 곧 진짜 꼭짓점이다.

    소비자는 이 꼭짓점을 원하는 반지름으로 둥글리면 된다. 그러면 직선과 원호가
    **접선 연속으로** 이어진다 — 여기서 원호를 억지로 남기면 그게 안 된다.
    """
    n = len(ring)
    get = lambda i: ring[i % n]

    lines = []
    for pr in prims:
        if pr['type'] != 'line':
            continue
        pts = [get(pr['i0'] + t) for t in range(pr['n'])]
        a, u = line_fit(pts)
        lines.append((a, u))

    if len(lines) < 3:
        return []

    verts = []
    m = len(lines)
    for i in range(m):
        a, u = lines[i]
        b, w = lines[(i + 1) % m]
        turn = math.degrees(math.atan2(u[0] * w[1] - u[1] * w[0], u[0] * w[0] + u[1] * w[1]))
        if abs(turn) < CORNER_MIN_DEG:
            continue
        v = line_isect(a, u, b, w)
        if v is None:
            continue
        verts.append({'kind': 'corner', 'p': v, 'turn_deg': round(turn, 3)})

    # 이웃 꼭짓점까지의 거리. 둥글릴 때 접선길이를 얼마나 쓸 수 있는지가 여기서 정해진다.
    q = len(verts)
    for i, it in enumerate(verts):
        nxt = verts[(i + 1) % q]['p']
        prv = verts[(i - 1) % q]['p']
        it['run_out_m'] = round(math.hypot(nxt[0] - it['p'][0], nxt[1] - it['p'][1]), 4)
        it['run_in_m'] = round(math.hypot(it['p'][0] - prv[0], it['p'][1] - prv[1]), 4)
        it['p'] = [round(it['p'][0], 4), round(it['p'][1], 4)]

    return verts


def ring_length(ring):
    total = 0.0
    for i in range(len(ring)):
        a, b = ring[i], ring[(i + 1) % len(ring)]
        total += math.hypot(b[0] - a[0], b[1] - a[1])
    return total


# ---------------------------------------------------------------- main

def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--sim-root', type=Path, default=SIM_ROOT)
    ap.add_argument('--out', type=Path, default=None,
                    help='기본: <패키지>/config/amet2026_track.json')
    ap.add_argument('--tol', type=float, default=0.002,
                    help='윤곽 단순화 허용오차 m (기본 2 mm)')
    ap.add_argument('--prim-tol', type=float, default=0.010,
                    help='직선/원호 쪼개기 허용오차 m (기본 10 mm)')
    ap.add_argument('--ox', type=float, default=DEFAULT_OX,
                    help='map_x = ox - sim_y 의 ox')
    ap.add_argument('--oy', type=float, default=DEFAULT_OY,
                    help='map_y = sim_x - oy 의 oy')
    args = ap.parse_args(argv)

    pkg = Path(__file__).resolve().parents[1]
    out_path = args.out or (pkg / 'config' / 'amet2026_track.json')

    if not args.sim_root.is_dir():
        raise SystemExit(f'{args.sim_root} 가 없다 — 시뮬레이터가 설치된 기기에서 돌려야 한다')

    world_id, meta = find_world(args.sim_root)
    mesh_dir = args.sim_root / 'meshes' / f'custom_{world_id}'
    dae = mesh_dir / f'custom_{world_id}.dae'
    if not dae.is_file():
        raise SystemExit(f'{dae} 가 없다')

    print(f'world "{meta.get("name")}"  id {world_id}  크기 {meta.get("size")} m')
    root = ET.parse(dae).getroot()

    layers = {}
    for name, gid, label in WANTED:
        tris = triangles(root, gid)
        if not tris:
            print(f'  {name:12s} 지오메트리 {gid} 없음 — 건너뛴다')
            continue
        rings = [rdp(r, args.tol) for r in outlines(tris)]
        rings = [r for r in rings if len(r) >= 3]
        rings.sort(key=ring_length, reverse=True)
        layers[name] = {
            'label': label,
            'rings': [[[round(x, 4), round(y, 4)] for x, y in r] for r in rings],
        }
        pts = sum(len(r) for r in rings)
        print(f'  {name:12s} 고리 {len(rings):4d} 개  점 {pts:5d}  '
              f'가장 긴 고리 {ring_length(rings[0]):6.2f} m')

    # 주행면 중심선. **이게 global path 다** (2026-08-21 결정).
    #
    # 트랙을 차로 둘로 나눠 보지 않는다. 흰색 차선 두 개의 한가운데 = 노란
    # 중앙 점선이 그대로 주행선이다. 차로별 중심선(lane_outer / lane_inner)도
    # 같이 뽑아 두지만 지금 경로에는 안 쓴다 — 나중에 차로 개념이 필요해지면
    # 다시 꺼내 쓰라고 남긴 것이다.
    if 'center_line' in layers and 'road' in layers:
        cl_rings = [[tuple(p) for p in r] for r in layers['center_line']['rings']]
        mid, gap = order_dashes(cl_rings)
        road = [[tuple(p) for p in r] for r in layers['road']['rings']]
        road.sort(key=ring_length, reverse=True)
        outer_edge, inner_edge = road[0], road[1]

        print(f'\n중앙선 점선 {len(mid)} 개 -> 고리 {poly_length(mid):.3f} m '
              f'(조각 간격 최대 {gap * 100:.1f} cm)')

        centers = {}
        _, w_out = lane_center(mid, outer_edge)
        _, w_in = lane_center(mid, inner_edge)
        half = (sum(w_out) + sum(w_in)) / (len(w_out) + len(w_in))

        specs = [('road_center', '주행면 중심선 — global path', mid, 2 * half)]
        for key, label, edge in [('lane_outer', '바깥 차로 중심선 (참고)', outer_edge),
                                 ('lane_inner', '안쪽 차로 중심선 (참고)', inner_edge)]:
            pts, width = lane_center(mid, edge)
            specs.append((key, label, pts, sum(width) / len(width)))

        for key, label, pts, width in specs:
            prims = primitives(pts, args.prim_tol)
            kmax = max((abs(p.get('kappa', 0.0)) for p in prims), default=0.0)
            centers[key] = {
                'label': label,
                'pts': [[round(x, 4), round(y, 4)] for x, y in pts],
                'length_m': round(poly_length(pts), 4),
                'corridor_half_width_m': round(width / 2, 4),
                'primitives': prims,
            }
            centers[key]['skeleton'] = skeleton(pts, prims)
            nline = sum(1 for q in prims if q['type'] == 'line')
            print(f'  {key:11s} {poly_length(pts):6.3f} m  반폭 {width/2:.3f} m  '
                  f'직선 {nline} + 원호 {len(prims) - nline}  '
                  f'|k|max {kmax:.3f} (R {1 / kmax if kmax else float("inf"):.3f} m)')
            sk = centers[key]['skeleton']
            ncor = sum(1 for it in sk if it['kind'] == 'corner')
            narc = len(sk) - ncor
            turns = sorted((abs(it['turn_deg']) for it in sk if it['kind'] == 'corner'),
                           reverse=True)
            print(f'{"":14s}골격: 꼭짓점 {ncor} + 원호 {narc}  '
                  f'꺾임각 {turns[0]:.1f}° ~ {turns[-1]:.1f}°' if turns else '')

        centers['primary'] = 'road_center'
    else:
        centers = {}

    doc = {
        'source': f'physicar-sim world {world_id} ("{meta.get("name")}")',
        'world_size_m': meta.get('size'),
        'units': 'm',
        'coords': 'sim',
        'note': ('좌표는 시뮬레이터 world 프레임이다. map 프레임으로 옮기려면 '
                 'map_x = ox - sim_y, map_y = sim_x - oy 를 건다. '
                 '편집기가 이 변환을 걸고, 미세조정은 화면에서 한다.'),
        'sim_to_map': {'ox': args.ox, 'oy': args.oy, 'rot_deg': 0.0},
        'simplify_tol_m': args.tol,
        'primitive_tol_m': args.prim_tol,
        'layers': layers,
        'centerlines': centers,
    }

    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(json.dumps(doc, ensure_ascii=False, separators=(',', ':')))
    print(f'\n-> {out_path}  ({out_path.stat().st_size / 1024:.1f} KB)')
    return 0


if __name__ == '__main__':
    sys.exit(main())
