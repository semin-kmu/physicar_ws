#!/usr/bin/env python3
"""track_amet2026.yaml 시각 확인.

생성 테이블이 map 프레임 · 주행 방향으로 제대로 나왔는지 눈으로 본다.
lane_graph(주행 경로)를 겹쳐 그려 좌표계 일치를 확인한다.
"""

import argparse
import math
import pathlib

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import yaml  # noqa: E402


def main():
    here = pathlib.Path(__file__).resolve().parents[1]
    ws = here.parent
    ap = argparse.ArgumentParser()
    ap.add_argument("--track", default=here / "config/track_amet2026.yaml")
    ap.add_argument("--lane-graph", default=ws / "kau_global_path/config/lane_graph.yaml")
    ap.add_argument("--out", default="/tmp/track_check.png")
    a = ap.parse_args()

    tr = yaml.safe_load(pathlib.Path(a.track).read_text())["track"]
    cl = tr["centerline"]
    co = tr["corners"]
    lg = yaml.safe_load(pathlib.Path(a.lane_graph).read_text())["lane_graph"]

    s = [r[0] for r in cl]
    x = [r[1] for r in cl]
    y = [r[2] for r in cl]
    th = [r[3] for r in cl]
    ka = [r[4] for r in cl]

    fig = plt.figure(figsize=(15, 8))
    gs = fig.add_gridspec(2, 2, width_ratios=[1.15, 1])

    # ── 트랙 평면도 ──
    ax = fig.add_subplot(gs[:, 0])
    ax.plot([p["x"] for p in lg["nodes"]], [p["y"] for p in lg["nodes"]],
            "o-", ms=3, lw=1, color="#bbb", label="lane_graph (global path)")
    sc = ax.scatter(x, y, c=ka, s=6, cmap="coolwarm", vmin=-4, vmax=4,
                    label="centerline (color = kappa)")
    fig.colorbar(sc, ax=ax, label="kappa [1/m]", shrink=0.6)

    for i, c in enumerate(co):
        j = min(range(len(s)), key=lambda k: abs(s[k] - c[0]))
        ax.plot(x[j], y[j], "k^", ms=9)
        ax.annotate(f"{i}\n{c[1]:+.0f}", (x[j], y[j]), fontsize=7,
                    xytext=(6, 6), textcoords="offset points")

    # 진행 방향 화살표
    for k in range(0, len(s), max(1, len(s) // 26)):
        ax.arrow(x[k], y[k], 0.22 * math.cos(th[k]), 0.22 * math.sin(th[k]),
                 head_width=0.09, color="#2a7", lw=0.8, zorder=5)

    ax.plot(x[0], y[0], "g*", ms=20, label="s = 0")
    ax.set_aspect("equal")
    ax.set_title("map frame | %.2f m | %d corners | arrows = driving dir" % (tr["length_m"], len(co)))
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.legend(loc="upper right", fontsize=8)
    ax.grid(alpha=0.3)

    # ── kappa(s) ──
    ax2 = fig.add_subplot(gs[0, 1])
    ax2.plot(s, ka, lw=0.8)
    for i, c in enumerate(co):
        ax2.axvline(c[0], color="k", lw=0.5, alpha=0.4)
        ax2.annotate(str(i), (c[0], 0), fontsize=6, color="k")
    ax2.set_title("kappa(s) - source of the corner signature")
    ax2.set_xlabel("s [m]")
    ax2.set_ylabel("kappa [1/m]")
    ax2.grid(alpha=0.3)

    # ── 직선 길이 시퀀스 ──
    ax3 = fig.add_subplot(gs[1, 1])
    idx = range(len(co))
    ax3.bar(idx, [c[2] for c in co], color="#69c")
    ax3.set_ylabel("run_in [m]", color="#69c")
    ax3.set_xlabel("corner index")
    ax3.set_xticks(list(idx))
    ax3.grid(alpha=0.3, axis="y")
    ax4 = ax3.twinx()
    ax4.plot(idx, [c[1] for c in co], "r.-", ms=8)
    ax4.set_ylabel("turn [deg]", color="r")
    ax4.axhline(0, color="r", lw=0.5, alpha=0.4)
    ax3.set_title("signature (run_in, turn) - 2 corners = unique")

    fig.tight_layout()
    fig.savefig(a.out, dpi=110)
    print(f"[plot] {a.out}")


if __name__ == "__main__":
    main()
