#!/usr/bin/env python3
"""amet2026_track.json -> track_amet2026.yaml

트랙 기하를 map 프레임 · 주행 방향으로 정규화해 측위용 정적 테이블을 만든다.
생성물은 커밋한다. 런타임에 /sim/api 를 호출하지 않는다.

출력:
  centerline  s 등간격 [s, x, y, theta, kappa]   황색 중앙선 기준
  corners     [s, turn_deg, run_in_m]            시그니처 매칭용
"""

import argparse
import json
import math
import pathlib
import sys

# ── 좌표 변환 ────────────────────────────────────────────────────────
# amet2026_track.json 의 note:
#   map_x = ox - sim_y,  map_y = sim_x - oy
# 야코비안 [[0,-1],[1,0]] -> det +1, +90 deg 회전. 손대칭이 보존되므로
# 회전각 부호는 그대로다.
SIM_TO_MAP_ROT_DEG = 90.0


def sim_to_map(pt, ox, oy):
    return (ox - pt[1], pt[0] - oy)


def wrap_pi(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def signed_area(pts):
    """양수면 반시계. 진행 방향 판정에 쓴다."""
    a = 0.0
    for i in range(len(pts)):
        x0, y0 = pts[i]
        x1, y1 = pts[(i + 1) % len(pts)]
        a += x0 * y1 - x1 * y0
    return 0.5 * a


# ── 주행 방향 판정 ───────────────────────────────────────────────────
def driving_direction(center_map, lane_graph_nodes):
    """lane_graph(주행 경로, map 프레임)와 대조해 진행 방향을 정한다.

    json 의 점 순서가 주행 방향과 같으면 +1, 반대면 -1.
    두 폐곡선의 회전 방향(signed area 부호)을 비교한다.
    """
    a_center = signed_area(center_map)
    a_lane = signed_area(lane_graph_nodes)
    same = (a_center > 0.0) == (a_lane > 0.0)
    return (+1 if same else -1), a_center, a_lane


# ── 리샘플 · 기하량 ──────────────────────────────────────────────────
def resample_closed(pts, ds):
    """폐곡선을 호길이 ds 등간격으로 다시 뽑는다."""
    ring = list(pts) + [pts[0]]
    seg = [math.dist(ring[i], ring[i + 1]) for i in range(len(ring) - 1)]
    total = sum(seg)
    n = int(round(total / ds))
    step = total / n

    out, j, run = [], 0, 0.0
    for i in range(n):
        target = i * step
        while j < len(seg) - 1 and run + seg[j] < target:
            run += seg[j]
            j += 1
        t = 0.0 if seg[j] < 1e-12 else (target - run) / seg[j]
        out.append(
            (
                ring[j][0] + t * (ring[j + 1][0] - ring[j][0]),
                ring[j][1] + t * (ring[j + 1][1] - ring[j][1]),
            )
        )
    return out, step, total


def heading_curvature(pts, ds, smooth_m):
    """heading 과 곡률. 적분 보존이 최우선이다.

    primitives 의 kappa 를 쓰면 값은 정확하지만 적분이 안 맞는다 — 원본이
    10 cm 간격인데 코너 반경은 6 cm 라, 호의 kappa 를 현 길이에 곱하면
    실제 호보다 긴 구간에 얹힌다 (실측 +483 deg / 기대 +360).

    그래서 이웃 방향각 차분으로 구한다. 폐곡선에서 sum(d_theta) 는 정확히
    2*pi 로 망원합이 되고, 이동평균은 적분을 보존하므로 평활 뒤에도
    유지된다. 코너 첨두값은 뭉개지지만 원본 해상도가 애초에 6 cm 반경을
    담지 못하므로 그게 정직하다. Frenet 보정·pan 피드포워드 모두
    첨두보다 적분이 중요하다.
    """
    n = len(pts)
    seg = [
        math.atan2(pts[(i + 1) % n][1] - pts[i][1], pts[(i + 1) % n][0] - pts[i][0])
        for i in range(n)
    ]
    dth = [wrap_pi(seg[(i + 1) % n] - seg[i]) for i in range(n)]

    w = max(1, int(round(smooth_m / ds)))
    if w % 2 == 0:
        w += 1
    half = w // 2
    ka = []
    for i in range(n):
        acc = sum(dth[(i + o) % n] for o in range(-half, half + 1))
        ka.append(acc / (w * ds))

    # theta 는 세그먼트 방향을 같은 창으로 평활한다 (각도라 벡터 평균).
    th = []
    for i in range(n):
        cs = sum(math.cos(seg[(i + o) % n]) for o in range(-half, half + 1))
        sn = sum(math.sin(seg[(i + o) % n]) for o in range(-half, half + 1))
        th.append(math.atan2(sn, cs))
    return th, ka


# ── 코너 테이블 ──────────────────────────────────────────────────────
def build_corners(skeleton, ox, oy, direction, center_map, s_of):
    """skeleton -> 주행 방향 기준 [s, turn_deg, run_in_m].

    skeleton 은 json 점 순서 기준이다. direction == -1 이면 순서를 뒤집고
    turn_deg 부호를 뒤집는다. run_in 은 뒤집으면 이웃의 run_out 이 된다.
    """
    corners = []
    for c in skeleton:
        p = sim_to_map(c["p"], ox, oy)
        corners.append(
            {
                "p": p,
                "turn_deg": float(c["turn_deg"]),
                "run_in_m": float(c["run_in_m"]),
                "run_out_m": float(c["run_out_m"]),
            }
        )

    if direction < 0:
        corners.reverse()
        for c in corners:
            c["turn_deg"] = -c["turn_deg"]
            c["run_in_m"], c["run_out_m"] = c["run_out_m"], c["run_in_m"]

    for c in corners:
        c["s"] = s_of(c["p"])
    return corners


def nearest_s_factory(pts, step):
    def s_of(p):
        best_i, best_d = 0, float("inf")
        for i, q in enumerate(pts):
            d = (q[0] - p[0]) ** 2 + (q[1] - p[1]) ** 2
            if d < best_d:
                best_d, best_i = d, i
        return best_i * step

    return s_of


# ── dash 기하 ────────────────────────────────────────────────────────
def dash_geometry(rings):
    """center_line 폴리곤(황색 점선 한 칸씩) -> 길이 · 폭 · pitch."""
    cents, lens, wids = [], [], []
    for r in rings:
        xs = [p[0] for p in r]
        ys = [p[1] for p in r]
        cents.append((sum(xs) / len(xs), sum(ys) / len(ys)))
        e = sorted(math.dist(r[i], r[(i + 1) % len(r)]) for i in range(len(r)))
        wids.append(e[1])
        lens.append(e[-1])

    pitch = []
    for i, c in enumerate(cents):
        pitch.append(min(math.dist(c, o) for j, o in enumerate(cents) if j != i))
    pitch.sort()
    med = pitch[len(pitch) // 2]
    return {
        "count": len(rings),
        "length_m": round(sorted(lens)[len(lens) // 2], 4),
        "width_m": round(sorted(wids)[len(wids) // 2], 4),
        "pitch_m": round(med, 4),
    }


# ── 유일성 검사 ──────────────────────────────────────────────────────
def uniqueness(corners, tol_len, tol_deg, max_k=4):
    n = len(corners)
    sig = [(c["run_in_m"], c["turn_deg"]) for c in corners]

    def win(i, k):
        return [sig[(i + j) % n] for j in range(k)]

    for k in range(1, max_k + 1):
        bad = []
        for i in range(n):
            for j in range(i + 1, n):
                if all(
                    abs(a[0] - b[0]) <= tol_len and abs(a[1] - b[1]) <= tol_deg
                    for a, b in zip(win(i, k), win(j, k))
                ):
                    bad.append((i, j))
        if not bad:
            return k, []
    return None, bad


# ── 출력 ─────────────────────────────────────────────────────────────
def emit(path, meta, center, corners, dash, ds):
    with open(path, "w", encoding="utf-8") as f:
        w = f.write
        w("# kau_lane_localization 트랙 테이블 — gen_track_table.py 로 생성.\n")
        w("# 손으로 고치지 말 것. 원본은 kau_global_path/config/amet2026_track.json.\n")
        w("#\n")
        w("# 좌표: map 프레임, 미터. s 는 주행 방향 누적 호길이.\n")
        w("# theta [rad], kappa [1/m] (좌회전 +). centerline 은 황색 중앙선.\n")
        w("#\n")
        for k, v in meta.items():
            w(f"#   {k}: {v}\n")
        w("\ntrack:\n")
        w("  frame_id: map\n")
        w("  source: '%s'\n" % str(meta["source"]).replace("'", "''"))
        w(f"  length_m: {meta['length_m']}\n")
        w(f"  ds_m: {round(ds, 6)}\n")
        w("  closed: true\n")
        w("\n  dash:\n")
        for k, v in dash.items():
            w(f"    {k}: {v}\n")
        w("\n  # [s, turn_deg, run_in_m]\n  corners:\n")
        for c in corners:
            w(
                "    - [%8.4f, %+9.4f, %8.4f]\n"
                % (c["s"], c["turn_deg"], c["run_in_m"])
            )
        w("\n  # [s, x, y, theta, kappa]\n  centerline:\n")
        for row in center:
            w(
                "    - [%8.4f, %9.5f, %9.5f, %+9.6f, %+10.5f]\n"
                % row
            )


def main():
    ap = argparse.ArgumentParser()
    here = pathlib.Path(__file__).resolve().parents[1]
    ws = here.parent
    ap.add_argument("--track", default=ws / "kau_global_path/config/amet2026_track.json")
    ap.add_argument("--lane-graph", default=ws / "kau_global_path/config/lane_graph.yaml")
    ap.add_argument("--out", default=here / "config/track_amet2026.yaml")
    ap.add_argument("--ds", type=float, default=0.02)
    ap.add_argument("--kappa-smooth", type=float, default=0.20,
                    help="곡률 이동평균 창 [m]. 적분은 창 크기와 무관하게 보존된다")
    ap.add_argument("--tol-len", type=float, default=0.15)
    ap.add_argument("--tol-deg", type=float, default=10.0)
    a = ap.parse_args()

    print("[1/6][gen] 원본 로드")
    d = json.loads(pathlib.Path(a.track).read_text())
    ox = d["sim_to_map"]["ox"]
    oy = d["sim_to_map"]["oy"]
    if abs(d["sim_to_map"]["rot_deg"]) > 1e-9:
        sys.exit("[gen] rot_deg 가 0 이 아니다. 변환식을 다시 확인할 것")
    rc = d["centerlines"]["road_center"]
    print(f"        source={d['source']}")
    print(f"        road_center pts={len(rc['pts'])} length={rc['length_m']} m")
    print(f"        sim_to_map ox={ox} oy={oy}")

    print("[2/6][gen] sim -> map 변환")
    center_map = [sim_to_map(p, ox, oy) for p in rc["pts"]]

    print("[3/6][gen] 주행 방향 판정 (lane_graph 대조)")
    import yaml

    lg = yaml.safe_load(pathlib.Path(a.lane_graph).read_text())["lane_graph"]
    lg_pts = [(n["x"], n["y"]) for n in lg["nodes"]]
    direction, a_c, a_l = driving_direction(center_map, lg_pts)
    print(f"        signed_area  center={a_c:+.3f}  lane_graph={a_l:+.3f}")
    print(
        "        direction=%+d (%s)"
        % (direction, "json 순서와 동일" if direction > 0 else "json 순서의 역방향")
    )

    if direction < 0:
        center_map.reverse()

    print(f"[4/6][gen] 리샘플 ds={a.ds} m")
    pts, step, total = resample_closed(center_map, a.ds)
    th, ka = heading_curvature(pts, step, a.kappa_smooth)
    center_rows = [
        (i * step, pts[i][0], pts[i][1], th[i], ka[i]) for i in range(len(pts))
    ]
    print(f"        {len(pts)} 점  총길이 {total:.4f} m  step {step:.5f} m")
    print(f"        |kappa| max {max(abs(k) for k in ka):.3f} 1/m")

    # 전역 검증: 폐곡선이면 반드시 +-360 deg 다. 여기서 어긋나면 기하가 깨진 것.
    turn_int = math.degrees(sum(ka) * step)
    print(f"        적분 kappa ds = {turn_int:+.3f} deg  (기대 +360)")
    if abs(abs(turn_int) - 360.0) > 1.0:
        sys.exit("[gen] 곡률 적분이 360 deg 와 어긋난다")

    print("[5/6][gen] 코너 테이블 + dash 기하")
    s_of = nearest_s_factory(pts, step)
    corners = build_corners(rc["skeleton"], ox, oy, direction, center_map, s_of)
    corners.sort(key=lambda c: c["s"])
    dash = dash_geometry(d["layers"]["center_line"]["rings"])
    turn_sum = sum(c["turn_deg"] for c in corners)
    print(f"        코너 {len(corners)}개  turn_deg 합 {turn_sum:+.1f}")
    print(f"        dash {dash}")

    k, bad = uniqueness(corners, a.tol_len, a.tol_deg)
    if k is None:
        print(f"        [경고] 유일성 실패 — 4개까지 봐도 모호 {len(bad)}쌍 {bad}")
    else:
        print(
            f"        유일성 OK — 코너 {k}개면 확정 "
            f"(허용 +-{a.tol_len} m / +-{a.tol_deg} deg)"
        )

    print("[6/6][gen] 출력")
    meta = {
        "source": d["source"],
        "length_m": round(total, 4),
        "direction": "json 역방향" if direction < 0 else "json 정방향",
        "turn_deg_sum": round(turn_sum, 1),
        "corners": len(corners),
        "unique_after_corners": k,
    }
    emit(a.out, meta, center_rows, corners, dash, step)
    print(f"        {a.out}")

    print("\n=== 코너 테이블 (주행 방향) ===")
    print("  #      s[m]   turn[deg]   run_in[m]      x        y")
    for i, c in enumerate(corners):
        print(
            "  %2d  %8.3f  %+9.3f  %9.3f  %8.3f %8.3f"
            % (i, c["s"], c["turn_deg"], c["run_in_m"], c["p"][0], c["p"][1])
        )


if __name__ == "__main__":
    main()
