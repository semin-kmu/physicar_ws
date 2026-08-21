#!/usr/bin/env python3
"""
저장된 지도(pgm)를 눈으로 확인한다. ROS 가 필요 없다. 언제든 실행 가능.

  ./view_map.py                 최신 kau_vN 을 자동으로 골라 검사 + ASCII 출력
  ./view_map.py --all           maps/ 안의 모든 버전을 한 줄씩 비교
  ./view_map.py kau_v2          특정 버전
  ./view_map.py --png           PNG 로도 뽑는다 (VNC 이미지 뷰어로 크게 보기)
  ./view_map.py --walls         벽 위치를 미터 단위로 정밀 측정해서 출력

검사 항목과 판정 기준은 save_map.py 와 같다.
"""

import argparse
import importlib.util
import re
import sys
from pathlib import Path

MAPS = Path(__file__).resolve().parent.parent / 'maps'

# 검사/파싱 로직은 save_map.py 것을 그대로 쓴다. 두 벌 관리하지 않는다.
_spec = importlib.util.spec_from_file_location(
    'save_map', Path(__file__).resolve().parent / 'save_map.py')
_sm = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_sm)
read_pgm, inspect, sketch = _sm.read_pgm, _sm.inspect, _sm.sketch


def versions(maps_dir: Path):
    """maps/ 안의 kau_vN.pgm 을 번호순으로."""
    pat = re.compile(r'^kau_v(\d+)\.pgm$')
    out = [(int(m.group(1)), f)
           for f in maps_dir.glob('kau_v*.pgm') if (m := pat.match(f.name))]
    return [f for _, f in sorted(out)]


def read_origin(pgm: Path):
    """짝이 되는 yaml 에서 origin 과 resolution 을 읽는다."""
    y = pgm.with_suffix('.yaml')
    res, ox, oy = 0.05, 0.0, 0.0
    if y.exists():
        for line in y.read_text().splitlines():
            if line.startswith('resolution:'):
                res = float(line.split(':')[1])
            elif line.startswith('origin:'):
                nums = re.findall(r'-?\d+\.?\d*', line.split(':', 1)[1])
                ox, oy = float(nums[0]), float(nums[1])
    return res, ox, oy


def walls(pgm: Path):
    """벽이 몇 개인지, 어디 있는지 미터로 찍는다. 이중 벽 찾기용."""
    w, h, d = read_pgm(pgm)
    res, ox, oy = read_origin(pgm)
    col = [0] * w
    row = [0] * h
    for i, v in enumerate(d):
        if v < 100:
            col[i % w] += 1
            row[i // w] += 1

    def peaks(hist, n, to_world, label, unit):
        # 전체 길이의 20% 이상 채워진 줄만 "벽" 으로 본다
        thr = max(8, n * 0.20)
        cand = [(i, c) for i, c in enumerate(hist) if c >= thr]
        groups, cur = [], []
        for i, c in cand:                       # 인접한 픽셀은 한 벽으로 묶는다
            if cur and i - cur[-1][0] > 2:
                groups.append(cur)
                cur = []
            cur.append((i, c))
        if cur:
            groups.append(cur)
        print(f'\n  {label} ({len(groups)}개)')
        for g in groups:
            best = max(g, key=lambda t: t[1])
            lo, hi = to_world(g[0][0]), to_world(g[-1][0])
            span = abs(hi - lo) + res
            flag = '  <- 이중/번짐' if span > 0.22 else ''
            print(f'    {unit} = {min(lo, hi):7.2f} ~ {max(lo, hi):7.2f} m'
                  f'  (두께 {span:.2f} m, 최대 {best[1]}셀){flag}')
        return groups

    print(f'\n=== 벽 정밀 측정  {pgm.name} ===')
    vg = peaks(col, h, lambda px: ox + px * res, '세로벽 (좌/우)', 'x')
    hg = peaks(row, w, lambda py: oy + (h - py) * res, '가로벽 (위/아래)', 'y')
    if len(vg) >= 2:
        a = ox + max(vg[0], key=lambda t: t[1])[0] * res
        b = ox + max(vg[-1], key=lambda t: t[1])[0] * res
        print(f'\n  -> 좌우 벽 중심 간격 {abs(b - a):.2f} m')
    if len(hg) >= 2:
        a = oy + (h - max(hg[0], key=lambda t: t[1])[0]) * res
        b = oy + (h - max(hg[-1], key=lambda t: t[1])[0]) * res
        print(f'  -> 상하 벽 중심 간격 {abs(b - a):.2f} m')
    if len(vg) > 2 or len(hg) > 2:
        print('\n  경고: 벽이 2개보다 많다. 유령 벽이나 이중 벽이 있다.')


def to_png(pgm: Path, scale: int = 3):
    """PNG 로 변환. 점유=검정, 자유=흰색, 미탐색=회색. 확대해서 보기 좋게."""
    try:
        from PIL import Image
    except ImportError:
        print('PIL 이 없어 PNG 변환을 건너뛴다.')
        return None
    w, h, d = read_pgm(pgm)
    img = Image.new('RGB', (w, h))
    px = img.load()
    for i, v in enumerate(d):
        if v < 100:
            c = (0, 0, 0)              # 벽
        elif v > 250:
            c = (255, 255, 255)        # 빈 공간
        else:
            c = (128, 128, 128)        # 미탐색
        px[i % w, i // w] = c
    img = img.resize((w * scale, h * scale), Image.NEAREST)
    out = pgm.with_suffix('.png')
    img.save(out)
    return out


def main():
    ap = argparse.ArgumentParser(description='저장된 지도 확인 (ROS 불필요)')
    ap.add_argument('name', nargs='?', help='kau_v2 처럼. 생략하면 최신 버전')
    ap.add_argument('--all', action='store_true', help='모든 버전을 한 줄씩 비교')
    ap.add_argument('--png', action='store_true', help='PNG 로도 저장')
    ap.add_argument('--walls', action='store_true', help='벽 위치 정밀 측정')
    ap.add_argument('--expect', default='12x7', help='실제 공간 치수 WxH [m]')
    ap.add_argument('--step', type=int, default=4, help='ASCII 축소 비율')
    ap.add_argument('--maps-dir', type=Path, default=MAPS)
    args = ap.parse_args()

    exp = None
    if args.expect.lower() != 'off':
        a, b = args.expect.lower().split('x')
        exp = (float(a), float(b))

    vs = versions(args.maps_dir)
    if not vs:
        sys.exit(f'{args.maps_dir} 에 kau_vN.pgm 이 없다.')

    if args.all:
        print(f'{"버전":<10}{"이미지":>16}{"내부 free":>16}{"벽두께":>9}{"판정":>7}')
        print('-' * 60)
        for f in vs:
            res, _, _ = read_origin(f)
            w, h, _d = read_pgm(f)
            ok, lines = inspect(f, res, exp)
            def pick(key, default='-'):
                for ln in lines:
                    if key in ln:
                        return ln.split(':')[1].strip().split('(')[0].strip()
                return default
            print(f'{f.stem:<10}{w}x{h} px{"":>4}{pick("내부 free 범위"):>16}'
                  f'{pick("벽 두께 최대"):>9}{"통과" if ok else "실패":>7}')
        return

    target = args.maps_dir / f'{args.name}.pgm' if args.name else vs[-1]
    if not target.exists():
        sys.exit(f'{target} 없음. 있는 것: {", ".join(f.stem for f in vs)}')
    res, ox, oy = read_origin(target)

    print('=' * 62)
    print(f' {target.name}   (origin [{ox:.3f}, {oy:.3f}], {res} m/px)')
    print('=' * 62)
    ok, lines = inspect(target, res, exp)
    print('\n'.join(lines))
    if args.walls:
        walls(target)
    print()
    print('\n'.join(sketch(target, step=args.step)))
    print()
    print('  # = 벽    . = 빈 공간    (공백) = 미탐색')
    print('=' * 62)
    print(' 판정: ' + ('통과' if ok else '실패 — 위 [실패] 항목 확인'))
    print('=' * 62)

    if args.png:
        out = to_png(target)
        if out:
            print(f'\n PNG 저장: {out}')
            print(f'   보기:  DISPLAY=:1 xdg-open {out}')
    return 0 if ok else 1


if __name__ == '__main__':
    raise SystemExit(main())
