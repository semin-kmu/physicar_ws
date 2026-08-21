#!/usr/bin/env python3
"""
Cartographer 지도 최종 최적화 + 저장 + Nav2 정적 지도 변환.

매핑 주행이 끝난 뒤 이 스크립트 하나만 실행하면 된다.

  1. 살아있는 trajectory 를 모두 finish  -> pose graph 최적화가 돌아간다
  2. 최적화가 끝날 때까지 대기
  3. kau_vN.pbstream 으로 저장          -> N 은 자동으로 붙는다
  4. kau_vN.pgm / kau_vN.yaml 로 변환
  5. 지도 품질 검사 결과 출력

버전은 maps/ 안을 훑어서 다음 번호를 자동으로 고른다.
아무것도 없으면 v1, kau_v1 이 있으면 v2, ... 기존 파일은 절대 덮어쓰지 않는다.

사용법:
    ./save_map.py                    # 기본값으로 저장
    ./save_map.py --settle 15        # 최적화 대기를 더 길게
    ./save_map.py --expect 12x7      # 기대 치수를 주면 검사에 쓴다
    ./save_map.py --no-convert       # pbstream 만 저장, pgm 변환 생략

주의:
    slam.launch.py 를 save_state:= 없이 띄웠다면 종료해도 자동 저장되지 않는다.
    반드시 이 스크립트로 저장한 뒤에 끄는 것.
"""

import argparse
import re
import subprocess
import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node

from cartographer_ros_msgs.srv import (
    FinishTrajectory,
    GetTrajectoryStates,
    WriteState,
)

DEFAULT_MAPS_DIR = Path(__file__).resolve().parent.parent / 'maps'
PREFIX = 'kau_v'
STATE_NAME = {0: 'ACTIVE', 1: 'FINISHED', 2: 'FROZEN', 3: 'DELETED'}


# ---------------------------------------------------------------- 버전 결정

def next_version(maps_dir: Path) -> int:
    """maps/ 안의 kau_vN.* 을 전부 훑어 다음 번호를 고른다.

    pbstream 뿐 아니라 pgm / yaml 까지 본다. 변환만 남아 있는 반쪽 버전이
    있어도 그 번호를 재사용해 덮어쓰지 않기 위해서다.
    """
    pat = re.compile(rf'^{re.escape(PREFIX)}(\d+)\.(pbstream|pgm|yaml)$')
    used = [
        int(m.group(1))
        for f in maps_dir.glob(f'{PREFIX}*')
        if (m := pat.match(f.name))
    ]
    return max(used) + 1 if used else 1


# ---------------------------------------------------------------- ROS 통신

class MapSaver(Node):

    def __init__(self):
        super().__init__('kau_map_saver')
        self.states = self.create_client(GetTrajectoryStates, '/get_trajectory_states')
        self.finish = self.create_client(FinishTrajectory, '/finish_trajectory')
        self.write = self.create_client(WriteState, '/write_state')

    def wait_services(self, timeout=10.0) -> bool:
        for name, cli in (('/get_trajectory_states', self.states),
                          ('/finish_trajectory', self.finish),
                          ('/write_state', self.write)):
            if not cli.wait_for_service(timeout_sec=timeout):
                self.get_logger().error(f'{name} 서비스가 없다.')
                return False
        return True

    def call(self, client, req, timeout=120.0):
        fut = client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        if not fut.done():
            raise TimeoutError('서비스 응답 없음')
        return fut.result()

    def trajectory_states(self):
        res = self.call(self.states, GetTrajectoryStates.Request())
        ts = res.trajectory_states
        return list(zip(ts.trajectory_id, ts.trajectory_state))


# ---------------------------------------------------------------- 지도 검사

def read_pgm(path: Path):
    """P5 PGM 파서. 주석(#) 을 건너뛴다."""
    with open(path, 'rb') as f:
        def tok():
            t = b''
            while True:
                c = f.read(1)
                if c == b'':
                    raise ValueError('PGM 헤더가 잘렸다')
                if c == b'#':
                    while f.read(1) not in (b'\n', b''):
                        pass
                    continue
                if c.isspace():
                    if t:
                        return t
                    continue
                t += c
        magic = tok()
        if magic != b'P5':
            raise ValueError(f'P5 가 아니다: {magic!r}')
        w, h = int(tok()), int(tok())
        tok()                                   # maxval
        return w, h, f.read(w * h)


def inspect(pgm: Path, res: float, expect):
    """유령 벽 / 벽 번짐 / 내부 치수를 검사한다. (pass, 리포트 줄 목록)"""
    w, h, d = read_pgm(pgm)
    occ = [(i % w, i // w) for i, v in enumerate(d) if v < 100]
    free = [(i % w, i // w) for i, v in enumerate(d) if v > 250]
    lines, ok = [], True

    if not occ or not free:
        return False, ['  점유/자유 셀이 비었다. 매핑이 거의 안 됐다.']

    def box(pts):
        xs = [a for a, _ in pts]
        ys = [b for _, b in pts]
        return ((max(xs) - min(xs) + 1) * res, (max(ys) - min(ys) + 1) * res)

    fw, fh = box(free)
    ow, oh = box(occ)
    lines.append(f'  이미지          : {w} x {h} px  ({w * res:.2f} x {h * res:.2f} m)')
    lines.append(f'  내부 free 범위  : {fw:.2f} x {fh:.2f} m')
    lines.append(f'  occupied 범위   : {ow:.2f} x {oh:.2f} m')

    # 유령 벽: 벽 바깥으로 크게 삐져나온 점유 셀
    slack_w, slack_h = ow - fw, oh - fh
    if max(slack_w, slack_h) > 0.60:
        ok = False
        lines.append(f'  [실패] occupied 가 free 보다 {max(slack_w, slack_h):.2f} m 크다.'
                     ' 방 밖에 유령 구조물이 있다.')
    else:
        lines.append(f'  [통과] 벽 바깥 여유 {max(slack_w, slack_h):.2f} m (0.60 이하)')

    # 벽 두께 / 내부 폭: 가로줄을 훑는다
    thick, widths = [], []
    for py in range(0, h, max(1, h // 40)):
        row = d[py * w:(py + 1) * w]
        idx = [i for i, v in enumerate(row) if v < 100]
        if len(idx) < 2:
            continue
        L = [i for i in idx if i < w // 2]
        R = [i for i in idx if i >= w // 2]
        if not L or not R:
            continue
        # 좌우 벽 사이에 실제로 빈 공간이 있는 줄만 본다. 끝벽이나 유령 벽처럼
        # 가로로 꽉 찬 줄을 섞으면 벽 두께가 방 절반으로 잡혀 엉뚱한 값이 나온다.
        if not any(v > 250 for v in row[max(L) + 1:min(R)]):
            continue
        thick.append(max((max(L) - min(L) + 1), (max(R) - min(R) + 1)) * res)
        widths.append((min(R) - max(L) - 1) * res)

    if thick:
        tmax = max(thick)
        lines.append(f'  벽 두께 최대    : {tmax:.2f} m')
        if tmax > 0.25:
            ok = False
            lines.append('  [실패] 벽이 0.25 m 넘게 번졌다. 드리프트 구간이 있다.')
        else:
            lines.append('  [통과] 벽 두께 정상 (0.25 이하)')

    if widths:
        lo, hi = min(widths), max(widths)
        lines.append(f'  내부 폭         : {lo:.2f} ~ {hi:.2f} m  (편차 {hi - lo:.2f})')
        if hi - lo > 0.30:
            ok = False
            lines.append('  [실패] 내부 폭 편차 0.30 m 초과. 지도가 휘었다.')

    if expect:
        ew, eh = expect
        got = sorted([fw, fh])
        exp = sorted([ew, eh])
        err = max(abs(got[0] - exp[0]), abs(got[1] - exp[1]))
        lines.append(f'  기대 치수 대비  : 오차 {err:.2f} m  (기대 {ew} x {eh} m)')
        if err > 0.50:
            ok = False
            lines.append('  [실패] 실제 공간 치수와 0.50 m 넘게 다르다.')
    return ok, lines


def sketch(pgm: Path, step=4):
    """터미널용 ASCII 축소도. 유령 벽은 눈으로 보는 게 제일 빠르다."""
    w, h, d = read_pgm(pgm)
    out = []
    for py in range(0, h, step):
        row = ''
        for px in range(0, w, step):
            blk = [d[(py + j) * w + (px + i)]
                   for j in range(step) if py + j < h
                   for i in range(step) if px + i < w]
            row += '#' if min(blk) < 100 else ('.' if max(blk) > 250 else ' ')
        out.append('  ' + row)
    return out


# ---------------------------------------------------------------- 메인

def main():
    ap = argparse.ArgumentParser(description='Cartographer 지도 최적화 + 버전 저장')
    ap.add_argument('--maps-dir', type=Path, default=DEFAULT_MAPS_DIR)
    ap.add_argument('--settle', type=float, default=8.0,
                    help='trajectory finish 후 최적화 대기 [s] (기본 8)')
    ap.add_argument('--resolution', type=float, default=0.05,
                    help='pgm 격자 해상도 [m] (기본 0.05)')
    ap.add_argument('--expect', default='12x7',
                    help='실제 공간 치수 WxH [m]. "off" 면 검사 안 함')
    ap.add_argument('--no-convert', action='store_true', help='pgm 변환 생략')
    ap.add_argument('--no-sketch', action='store_true', help='ASCII 축소도 생략')
    args = ap.parse_args()

    expect = None
    if args.expect.lower() != 'off':
        try:
            a, b = args.expect.lower().split('x')
            expect = (float(a), float(b))
        except ValueError:
            sys.exit(f'--expect 형식이 이상하다: {args.expect} (예: 12x7)')

    maps_dir: Path = args.maps_dir
    maps_dir.mkdir(parents=True, exist_ok=True)

    n = next_version(maps_dir)
    stem = maps_dir / f'{PREFIX}{n}'
    pbstream = stem.with_suffix('.pbstream')
    if pbstream.exists():
        sys.exit(f'{pbstream} 이 이미 있다. 버전 계산이 틀렸다.')

    print(f'[1/5] 저장 버전: {PREFIX}{n}  ->  {pbstream}')

    rclpy.init()
    node = MapSaver()
    try:
        if not node.wait_services():
            sys.exit('cartographer_node 가 안 보인다. slam.launch.py 가 떠 있는지 확인.')

        states = node.trajectory_states()
        if not states:
            sys.exit('trajectory 가 하나도 없다. 매핑이 시작된 적이 없다.')
        print('[2/5] trajectory 상태: '
              + ', '.join(f'{i}={STATE_NAME.get(s, s)}' for i, s in states))

        active = [i for i, s in states if s == 0]
        for tid in active:
            res = node.call(node.finish, FinishTrajectory.Request(trajectory_id=int(tid)))
            if res.status.code != 0:
                print(f'      ! trajectory {tid} finish 실패: {res.status.message}')
            else:
                print(f'      trajectory {tid} finish -> 최적화 시작')

        if not active:
            print('      이미 전부 finish 상태. 최적화는 건너뛴다.')
        else:
            # finish 는 pose graph 최적화를 큐에 넣을 뿐이라 끝날 때까지 기다린다.
            print(f'[3/5] 최적화 대기 (최대 {args.settle:.0f}s + 상태 확인)...')
            deadline = time.time() + max(args.settle, 60.0)
            while time.time() < deadline:
                if not [i for i, s in node.trajectory_states() if s == 0]:
                    break
                time.sleep(0.5)
            time.sleep(args.settle)

        print(f'[4/5] pbstream 저장 중...')
        res = node.call(node.write, WriteState.Request(
            filename=str(pbstream), include_unfinished_submaps=True), timeout=300.0)
        if res.status.code != 0:
            sys.exit(f'write_state 실패: {res.status.message}')
    finally:
        node.destroy_node()
        rclpy.shutdown()

    if not pbstream.exists():
        sys.exit(f'{pbstream} 이 안 생겼다. cartographer 쪽 로그를 확인할 것.')
    print(f'      OK  {pbstream}  ({pbstream.stat().st_size / 1e6:.1f} MB)')

    if args.no_convert:
        print('[5/5] pgm 변환 생략 (--no-convert)')
        return

    print('[5/5] Nav2 정적 지도로 변환 중...')
    cmd = ['ros2', 'run', 'cartographer_ros', 'cartographer_pbstream_to_ros_map',
           '-pbstream_filename', str(pbstream),
           '-map_filestem', str(stem),
           '-resolution', str(args.resolution)]
    r = subprocess.run(cmd, capture_output=True, text=True)
    pgm, yaml = stem.with_suffix('.pgm'), stem.with_suffix('.yaml')
    if not pgm.exists():
        print(r.stdout, r.stderr, sep='\n')
        sys.exit('pgm 변환 실패')
    print(f'      OK  {pgm}')
    print(f'      OK  {yaml}')

    print()
    print('=' * 62)
    print(f' 지도 검사  {pgm.name}')
    print('=' * 62)
    ok, lines = inspect(pgm, args.resolution, expect)
    print('\n'.join(lines))
    if not args.no_sketch:
        print()
        print('\n'.join(sketch(pgm)))
    print('=' * 62)
    if ok:
        print(' 판정: 통과. lane_graph 작업에 써도 된다.')
    else:
        print(' 판정: 실패. 위 [실패] 항목을 보고 재매핑을 권한다.')
        print('       (이 버전은 지워지지 않으니 다음 매핑은 자동으로 다음 번호로 간다)')
    print('=' * 62)
    print()
    print(' 이 지도로 측위를 띄우려면:')
    print(f'   ros2 launch kau_localization localization.launch.py \\')
    print(f'       pbstream:={pbstream} rviz:=true')
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
