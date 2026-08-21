#!/usr/bin/env python3
# Copyright 2026 KAU AMET Team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""record_trajectory.py 로 딴 한 바퀴를 지도 위에 그려서 눈으로 검수한다.

    python3 src/kau_global_path/scripts/review_trajectory.py data/inner_1.csv
    python3 src/kau_global_path/scripts/review_trajectory.py data/*.csv --map kau_v3

ROS 가 필요 없다. matplotlib + numpy 만 있으면 된다.

왜 리샘플하는가
    측위 지터가 min_dist(기본 0.02 m) 보다 커서, 원본 점열의 호길이와 곡률은
    둘 다 부풀어 있다. 한 바퀴 길이는 --resample 을 키우면 수렴하고,
    곡률도 마찬가지다. 그래서 "원본" 과 "리샘플" 을 같이 찍는다.
    수렴한 쪽이 참값이다.

무엇을 보는가
    1. 지도 위 궤적 + heading  — 벽을 넘지 않는지, 한 바퀴가 닫혔는지
    2. 곡률 vs 차량 한계 kappa_max — 조향 포화 구간이 어디인지
    3. 속도                    — 텔레옵이라 들쭉날쭉한 게 정상
"""

import argparse
import csv
import math
from pathlib import Path

import numpy as np

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.collections import LineCollection  # noqa: E402


# kau_control/include/kau_control/params.hpp 의 VehicleParams 와 같은 값.
# 대회측 회신으로 확정된 제원이라 여기 하드코딩해도 된다.
WHEELBASE_M = 0.18
MAX_STEER_DEG = 20.0
R_MIN = WHEELBASE_M / math.tan(math.radians(MAX_STEER_DEG))
KAPPA_MAX = 1.0 / R_MIN


def read_pgm(path):
    """P5 (binary) PGM 을 읽는다. cartographer 출력은 헤더에 주석이 한 줄 있다."""
    data = Path(path).read_bytes()
    tok, i = [], 2
    while len(tok) < 3:
        while data[i:i + 1].isspace():
            i += 1
        if data[i:i + 1] == b'#':
            while data[i:i + 1] != b'\n':
                i += 1
            continue
        j = i
        while not data[j:j + 1].isspace():
            j += 1
        tok.append(int(data[i:j]))
        i = j
    i += 1
    w, h, _ = tok
    return np.frombuffer(data[i:i + w * h], dtype=np.uint8).reshape(h, w)


def read_map(stem, maps_dir):
    """yaml 을 읽어서 (rgb 이미지, extent) 를 준다. 값을 하드코딩하지 않는다 (README 2.3)."""
    import yaml
    meta = yaml.safe_load((maps_dir / f'{stem}.yaml').read_text())
    img = read_pgm(maps_dir / Path(meta['image']).name)
    h, w = img.shape
    res = float(meta['resolution'])
    ox, oy = float(meta['origin'][0]), float(meta['origin'][1])

    # 흰=free, 회=unknown, 검=occupied. 회색조 그대로 쓰면 벽이 안 보인다.
    rgb = np.full((h, w, 3), 0.97)
    rgb[img < 100] = (0.10, 0.10, 0.10)
    rgb[(img >= 100) & (img < 250)] = (0.86, 0.86, 0.86)
    return rgb, [ox, ox + w * res, oy, oy + h * res]


def load_csv(path):
    rows = list(csv.DictReader(Path(path).open()))
    return {k: np.array([float(r[k]) for r in rows]) for k in ('t', 'x', 'y', 'yaw')}


def resample(x, y, step):
    """직전 채택점에서 step 이상 떨어진 점만 남긴다. 지터를 걷어내는 가장 싼 방법."""
    keep = [0]
    for i in range(1, len(x)):
        if math.hypot(x[i] - x[keep[-1]], y[i] - y[keep[-1]]) >= step:
            keep.append(i)
    return np.array(keep)


def curvature(x, y):
    """이웃 3점의 외접원 반지름의 역수. README 6.4 가 말하는 싼 sanity check."""
    n = len(x)
    k = np.zeros(n)
    for i in range(1, n - 1):
        a, b, c = (x[i - 1], y[i - 1]), (x[i], y[i]), (x[i + 1], y[i + 1])
        la, lb, lc = math.dist(a, b), math.dist(b, c), math.dist(a, c)
        area = abs((b[0] - a[0]) * (c[1] - a[1]) - (c[0] - a[0]) * (b[1] - a[1])) / 2
        k[i] = 4 * area / (la * lb * lc) if la * lb * lc > 1e-9 else 0.0
    if n > 2:
        k[0], k[-1] = k[1], k[-2]
    return k


def arclen(x, y):
    return float(np.hypot(np.diff(x), np.diff(y)).sum())


def review(csv_path, rgb, extent, resample_m, out_path):
    d = load_csv(csv_path)
    t, x, y, yaw = d['t'], d['x'], d['y'], d['yaw']

    step = np.hypot(np.diff(x), np.diff(y))
    s_raw = np.r_[0.0, np.cumsum(step)]
    gap = math.dist((x[0], y[0]), (x[-1], y[-1]))

    idx = resample(x, y, resample_m)
    rx, ry = x[idx], y[idx]
    s = np.r_[0.0, np.cumsum(np.hypot(np.diff(rx), np.diff(ry)))]
    k = curvature(rx, ry)
    # 양 끝 2점은 폐곡선 이음새라 곡률이 의미 없다
    kk = k[2:-2] if len(k) > 6 else k
    over = float((kk > KAPPA_MAX).mean() * 100) if len(kk) else 0.0

    dt = np.diff(t)
    v = np.where(dt > 0, step / np.maximum(dt, 1e-6), 0.0)

    fig = plt.figure(figsize=(16, 8.5))
    gs = fig.add_gridspec(2, 2, width_ratios=[1, 1.15], wspace=0.18, hspace=0.30)

    ax = fig.add_subplot(gs[:, 0])
    if rgb is not None:
        ax.imshow(rgb, extent=extent, origin='upper', interpolation='nearest')
        ax.set_xlim(extent[0], extent[1])
        ax.set_ylim(extent[2], extent[3])
    seg = np.stack([np.c_[x[:-1], y[:-1]], np.c_[x[1:], y[1:]]], axis=1)
    lc = LineCollection(seg, cmap='viridis', linewidth=3)
    lc.set_array(t[:-1])
    ax.add_collection(lc)
    fig.colorbar(lc, ax=ax, label='elapsed t [s]', fraction=0.045, pad=0.02)
    st = max(1, len(x) // 40)
    ax.quiver(x[::st], y[::st], np.cos(yaw[::st]), np.sin(yaw[::st]),
              color='orangered', scale=26, width=0.004, alpha=0.9, zorder=4)
    ax.plot(x[0], y[0], 'o', ms=13, mfc='lime', mec='k', zorder=6, label='start')
    ax.plot(x[-1], y[-1], 's', ms=11, mfc='red', mec='k', zorder=6,
            label=f'end (gap {gap * 100:.1f} cm)')
    ax.set_aspect('equal')
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.set_title(f'{Path(csv_path).name}   lap {s[-1]:.2f} m '
                 f'(raw {s_raw[-1]:.2f} m)   {t[-1]:.1f} s   {len(x)} pts', fontsize=12)
    ax.legend(loc='lower right', fontsize=9)
    ax.grid(alpha=0.15)

    a = fig.add_subplot(gs[0, 1])
    a.plot(s, k, lw=1.5, color='tab:red', label=f'|k| @ {resample_m:.2f} m resample')
    a.axhline(KAPPA_MAX, ls='--', c='k',
              label=f'vehicle limit {KAPPA_MAX:.2f} /m  (R_min {R_MIN:.2f} m)')
    a.axhline(0.8 * KAPPA_MAX, ls=':', c='gray', label='80 % (design margin)')
    a.fill_between(s, KAPPA_MAX, np.maximum(k, KAPPA_MAX), color='red', alpha=0.25)
    a.set_ylabel('curvature |k| [1/m]')
    a.set_xlim(0, s[-1])
    a.grid(alpha=0.25)
    a.legend(fontsize=8, loc='upper right')
    a.set_title(f'max|k| = {kk.max():.2f} /m  ->  R = {1 / max(kk.max(), 1e-9):.2f} m'
                f'   |   over limit: {over:.1f} % of points', fontsize=11)

    b = fig.add_subplot(gs[1, 1])
    b.plot(s_raw[:-1], v, lw=0.6, color='tab:blue', alpha=0.30, label='raw')
    win = max(1, int(0.5 / max(np.median(step), 1e-3)))
    if win > 1 and len(v) > win:
        sm = np.convolve(v, np.ones(win) / win, mode='same')
        b.plot(s_raw[:-1], sm, lw=1.8, color='tab:blue', label='~0.5 m moving avg')
    b.axhline(v.mean(), ls='--', c='k', alpha=0.5, label=f'mean {v.mean():.2f} m/s')
    b.set_xlabel('arc length s [m]')
    b.set_ylabel('speed [m/s]')
    b.set_xlim(0, s_raw[-1])
    b.grid(alpha=0.25)
    b.legend(fontsize=8, loc='upper right')
    b.set_title('speed  (teleop: stop-and-go is expected)', fontsize=11)

    fig.savefig(out_path, dpi=125, bbox_inches='tight')
    plt.close(fig)

    print(f'{Path(csv_path).name}')
    print(f'  lap length   {s[-1]:.2f} m   (raw {s_raw[-1]:.2f} m, '
          f'jitter inflation x{s_raw[-1] / max(s[-1], 1e-9):.2f})')
    print(f'  duration     {t[-1]:.1f} s,  {len(x)} pts -> {len(idx)} after resample')
    print(f'  closure gap  {gap * 100:.1f} cm')
    print(f'  max |k|      {kk.max():.2f} /m  (R = {1 / max(kk.max(), 1e-9):.2f} m)  '
          f'vs limit {KAPPA_MAX:.2f} /m (R_min {R_MIN:.2f} m)')
    print(f'  over limit   {over:.1f} % of resampled points')
    print(f'  -> {out_path}')


def main():
    here = Path(__file__).resolve().parent.parent
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('csv', nargs='+', help='record_trajectory.py 가 만든 csv')
    p.add_argument('--map', default='kau_v3', help='kau_localization/maps 의 지도 이름')
    p.add_argument('--maps-dir', default=None)
    p.add_argument('--resample', type=float, default=0.40,
                   help='m. 곡률/길이를 재기 전에 걷어낼 지터 크기. '
                        '값을 키워도 결과가 안 변하면 수렴한 것이다')
    p.add_argument('--out-dir', default=None, help='기본: csv 와 같은 폴더')
    args = p.parse_args()

    maps_dir = Path(args.maps_dir) if args.maps_dir \
        else here.parent / 'kau_localization' / 'maps'
    try:
        rgb, extent = read_map(args.map, maps_dir)
    except Exception as exc:                                  # noqa: BLE001
        print(f'지도를 못 읽었다 ({exc}). 배경 없이 그린다.')
        rgb, extent = None, None

    print(f'vehicle: L={WHEELBASE_M:.2f} m, max_steer={MAX_STEER_DEG:.0f} deg'
          f'  ->  R_min={R_MIN:.3f} m, kappa_max={KAPPA_MAX:.3f} /m')
    for c in args.csv:
        c = Path(c)
        out = (Path(args.out_dir) if args.out_dir else c.parent) / f'{c.stem}_review.png'
        review(c, rgb, extent, args.resample, out)


if __name__ == '__main__':
    main()
