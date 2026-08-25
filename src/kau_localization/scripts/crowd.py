#!/usr/bin/env python3
"""사람처럼 찍히는 라이다 노이즈를 Gazebo 월드에 뿌리기 위한 SDF 생성기.

    ./crowd.py > /tmp/crowd.sdf         기본 배치 43명
                                        (관중 30 + 인필드 8 + 도로 5)
    ./crowd.py --outside 70 --stand-off 0.18    더 붐비게
    ./crowd.py --list-only              좌표만 출력 (sim / map 양쪽)

crowd.sh 가 이걸 불러서 월드에 띄운다. world 파일은 건드리지 않는다.

── 왜 원기둥 두 개가 한 사람인가 ────────────────────────────────────
라이다 빔 평면이 지면 18.2 cm 다 (physicar model.sdf 의 lidar_sensor
pose z=0.182). 그 높이에서 사람은 몸통이 아니라 **정강이 두 개**로
찍힌다. 그래서 한 사람을 반지름 5 cm 원기둥 두 개(간격 22 cm)로 만든다.
덩어리 하나로 두면 실제와 다른 깨끗한 원호가 찍혀서, 대회장에서 본
"포인트가 분산되어 찍힌다" 가 재현되지 않는다.

── 왜 collision 이 없는가 ───────────────────────────────────────────
gz-sim 의 gpu_lidar 는 렌더링된 **visual** 을 상대로 레이캐스팅한다.
collision 은 물리 엔진 쪽이라 별개다. 그래서 visual 만 두면
**라이다에는 보이는데 차는 통과한다.** 측위 저하만 떼어내서 보려면
이게 맞다 -- 차가 사람에 걸려 멈추면 그건 측위 실험이 아니라 충돌
실험이 된다. 실제로 막고 싶으면 --collide 를 준다.

── 좌표계 ───────────────────────────────────────────────────────────
Gazebo world 좌표로 낸다. map 프레임과의 관계는
kau_global_path/config/amet2026_track.json 에 적힌 그대로다:

    map_x = ox - sim_y,   map_y = sim_x - oy     (ox=3.68, oy=1.39)

--list-only 는 두 좌표를 같이 찍어 rviz 에서 대조할 수 있게 한다.

── 밀도별 실측 (2026-08-25, 차량 정차, /scan 720빔) ─────────────────
얼마나 뿌려야 대회장처럼 되는지의 기준표다. ">7m" 가 "먼 공간을 못
본다" 에 해당한다.

    배치                              평균거리   >3m   >5m   >7m
    사람 없음                          4.51 m    62%   25%   16%
    19명 (기본, stand-off 0.45)        4.15 m    55%   23%   14%
    42명                               4.09 m    54%   21%   12%
    68명 (stand-off 0.18, 트랙 바로 옆) 3.82 m    51%   18%    9%
    107명                              3.60 m    44%   18%    9%

stand-off 가 사람 수보다 세다. 관중이 트랙에서 45 cm 떨어져 있으면
차 근처 빔을 거의 안 가리는데, 18 cm 로 붙이면 같은 인원으로도 원거리
빔이 절반 가까이 잘린다. 대회장 사진처럼 사람이 트랙에 붙어 있으면
--stand-off 를 먼저 줄일 것.

★ 이 표는 차가 한 자리에 서 있을 때의 값이다. 주행 중에는 차 위치에
  따라 가림 정도가 크게 달라진다 -- 한 지점 수치로 판단하지 말 것.
"""

import argparse
import json
import math
import os
import random
import sys


HERE = os.path.dirname(os.path.abspath(__file__))

TRACK_JSON = os.path.normpath(os.path.join(
    HERE, '..', '..', 'kau_global_path', 'config', 'amet2026_track.json'))

# physicar-sim world 크기 [m]. amet2026_track.json 의 world_size_m.
WORLD_W = 12.0
WORLD_H = 7.0

# 벽에서 이만큼은 띄운다 [m]. 벽에 박힌 사람은 벽과 구별이 안 돼서
# 노이즈 실험이 안 된다.
WALL_MARGIN = 0.35

LEG_RADIUS = 0.05
LEG_GAP = 0.22
PERSON_HEIGHT = 1.70


def load_rings():
    with open(TRACK_JSON, encoding='utf-8') as fp:
        doc = json.load(fp)

    rings = doc['layers']['road']['rings']
    sim_to_map = doc.get('sim_to_map', {'ox': 3.68, 'oy': 1.39})

    # ring[0] 이 바깥 경계다 (bbox 가 전체 범위를 덮는 쪽).
    outer, inner = rings[0], rings[1]
    return outer, inner, sim_to_map['ox'], sim_to_map['oy']


def centroid(ring):
    return (sum(p[0] for p in ring) / len(ring),
            sum(p[1] for p in ring) / len(ring))


def offset_points(ring, distance, outward, count, rng, jitter):
    """ring 위를 고르게 훑으며 법선 방향으로 distance 만큼 민 점들."""
    cx, cy = centroid(ring)
    out = []

    for i in range(count):
        # 고르게 퍼뜨리되 약간 흔든다. 정확히 등간격이면 사람이 아니라
        # 울타리처럼 찍혀서 오히려 지도와 잘 맞아버린다.
        t = (i + rng.uniform(-jitter, jitter)) / count
        idx = int(t * len(ring)) % len(ring)
        px, py = ring[idx]

        nx, ny = px - cx, py - cy
        norm = math.hypot(nx, ny)

        if norm < 1e-6:
            continue

        nx, ny = nx / norm, ny / norm
        sign = 1.0 if outward else -1.0
        d = distance * rng.uniform(0.7, 1.3)

        x = px + sign * nx * d
        y = py + sign * ny * d

        x = min(max(x, WALL_MARGIN), WORLD_W - WALL_MARGIN)
        y = min(max(y, WALL_MARGIN), WORLD_H - WALL_MARGIN)

        out.append((x, y))

    return out


def on_ring_points(ring, count, rng):
    """도로 위 (바깥 경계와 안쪽 경계 사이) 점들."""
    out = []

    for i in range(count):
        t = (i + rng.uniform(0.0, 1.0)) / count
        idx = int(t * len(ring)) % len(ring)
        px, py = ring[idx]
        out.append((px, py))

    return out


def person_links(name, x, y, yaw, collide):
    """사람 하나 = 정강이 원기둥 두 개."""
    half = LEG_GAP / 2.0
    dx, dy = -math.sin(yaw) * half, math.cos(yaw) * half

    parts = []

    for side, (lx, ly) in (('l', (x + dx, y + dy)), ('r', (x - dx, y - dy))):
        geom = (
            f'        <geometry>\n'
            f'          <cylinder>\n'
            f'            <radius>{LEG_RADIUS}</radius>\n'
            f'            <length>{PERSON_HEIGHT}</length>\n'
            f'          </cylinder>\n'
            f'        </geometry>\n'
        )

        collision = ''
        if collide:
            collision = (
                f'      <collision name="{name}_{side}_col">\n'
                f'{geom}'
                f'      </collision>\n'
            )

        parts.append(
            f'    <link name="{name}_{side}">\n'
            f'      <pose>{lx:.3f} {ly:.3f} {PERSON_HEIGHT / 2.0:.3f} 0 0 0</pose>\n'
            f'{collision}'
            f'      <visual name="{name}_{side}_vis">\n'
            f'{geom}'
            f'        <material>\n'
            f'          <ambient>0.15 0.15 0.18 1</ambient>\n'
            f'          <diffuse>0.20 0.22 0.28 1</diffuse>\n'
            f'        </material>\n'
            f'      </visual>\n'
            f'    </link>\n'
        )

    return ''.join(parts)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--model-name', default='crowd_noise')
    ap.add_argument('--seed', type=int, default=1,
                    help='같은 seed 면 같은 배치가 나온다 (재현용)')
    ap.add_argument('--outside', type=int, default=30,
                    help='바깥 경계 밖 관중 수. 펜스/벽을 가린다')
    ap.add_argument('--infield', type=int, default=8,
                    help='안쪽 경계 안 인필드 인원. 반대편 트랙을 가린다')
    ap.add_argument('--onroad', type=int, default=5,
                    help='도로 위 인원. 스캔을 직접 오염시킨다')
    ap.add_argument('--stand-off', type=float, default=0.25,
                    help='경계에서 떨어지는 거리 [m]')
    ap.add_argument('--collide', action='store_true',
                    help='충돌체도 넣는다 (기본은 라이다에만 보인다)')
    ap.add_argument('--list-only', action='store_true',
                    help='SDF 대신 좌표표만 출력')
    args = ap.parse_args()

    rng = random.Random(args.seed)
    outer, inner, ox, oy = load_rings()

    spots = []
    spots += [('관중', p) for p in offset_points(
        outer, args.stand_off, True, args.outside, rng, 0.35)]
    spots += [('인필드', p) for p in offset_points(
        inner, args.stand_off, False, args.infield, rng, 0.35)]
    spots += [('도로위', p) for p in on_ring_points(inner, args.onroad, rng)]

    if args.list_only:
        print(f'{"#":>3}  {"구역":<8} {"sim_x":>7} {"sim_y":>7}   '
              f'{"map_x":>7} {"map_y":>7}')
        for i, (zone, (x, y)) in enumerate(spots):
            print(f'{i:>3}  {zone:<8} {x:>7.2f} {y:>7.2f}   '
                  f'{ox - y:>7.2f} {x - oy:>7.2f}')
        print(f'\n총 {len(spots)} 명 (원기둥 {len(spots) * 2} 개), seed={args.seed}',
              file=sys.stderr)
        return 0

    links = ''.join(
        person_links(f'p{i}', x, y, rng.uniform(0.0, math.pi), args.collide)
        for i, (_zone, (x, y)) in enumerate(spots))

    print(
        '<?xml version="1.0"?>\n'
        '<sdf version="1.9">\n'
        f'  <model name="{args.model_name}">\n'
        '    <static>true</static>\n'
        f'{links}'
        '  </model>\n'
        '</sdf>'
    )

    print(f'{len(spots)} 명 생성 (seed={args.seed}, '
          f'collision={"있음" if args.collide else "없음"})', file=sys.stderr)

    return 0


if __name__ == '__main__':
    sys.exit(main())
