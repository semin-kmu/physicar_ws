#!/usr/bin/env python3
"""
매핑 중 pose graph 의 루프 클로저 constraint 길이 분포를 본다.

빈 직사각형 방은 어느 벽이든 똑같이 생겨서, 방 이쪽 끝 스캔이 반대쪽 submap 에
붙는 가짜 constraint 가 대량으로 생긴다. 그게 pose graph 를 잡아당겨 지도를
평행사변형으로 찌그러뜨린다. constraint 길이를 보면 바로 드러난다.

  정상 : inter constraint 가 대부분 짧다 (max_constraint_distance 안쪽)
  이상 : 방 크기에 육박하는 긴 constraint 가 수천 개

physicar_2d.lua 의 `max_constraint_distance` 를 바꾼 뒤 효과를 확인하는 용도다.
slam.launch.py 가 떠 있는 상태에서 실행한다.

    ./check_constraints.py
    ./check_constraints.py --limit 4.0      # 임계값을 직접 지정
"""

import argparse
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from visualization_msgs.msg import MarkerArray

# /constraint_list 는 volatile 로 발행된다. transient_local 로 구독하면
# QoS 불일치로 한 개도 못 받는다.
VOLATILE = QoSProfile(depth=5,
                      durability=QoSDurabilityPolicy.VOLATILE,
                      reliability=QoSReliabilityPolicy.RELIABLE,
                      history=QoSHistoryPolicy.KEEP_LAST)


class Sub(Node):
    def __init__(self):
        super().__init__('check_constraints')
        self.msg = None
        self.create_subscription(MarkerArray, '/constraint_list',
                                 lambda m: setattr(self, 'msg', m), VOLATILE)


def main():
    ap = argparse.ArgumentParser(description='루프 클로저 constraint 길이 분포')
    ap.add_argument('--limit', type=float, default=None,
                    help='합격 임계값 [m]. 기본은 lua 의 max_constraint_distance 인 4.0')
    ap.add_argument('--timeout', type=float, default=25.0)
    args = ap.parse_args()
    limit = args.limit if args.limit is not None else 4.0

    rclpy.init()
    n = Sub()
    t0 = time.time()
    while n.msg is None and time.time() - t0 < args.timeout:
        rclpy.spin_once(n, timeout_sec=0.5)
    if n.msg is None:
        print('/constraint_list 를 못 받았다. slam.launch.py 가 떠 있는지 확인.')
        rclpy.shutdown()
        return 1

    bad = False
    for mk in n.msg.markers:
        pts = mk.points
        if not pts or 'residual' in mk.ns.lower():
            continue
        # 마커는 constraint 하나당 선분 하나(점 2개)로 그려진다.
        lens = sorted(math.dist((pts[i].x, pts[i].y), (pts[i + 1].x, pts[i + 1].y))
                      for i in range(0, len(pts) - 1, 2))
        if not lens:
            continue
        cnt = len(lens)
        print(f'[{mk.ns}]  {cnt}개')
        print(f'   중앙값 {lens[cnt // 2]:.2f} m   평균 {sum(lens) / cnt:.2f}   '
              f'최대 {lens[-1]:.2f}')
        for thr in (2, 4, 6, 8, 10):
            c = sum(1 for x in lens if x > thr)
            print(f'   {thr:2d} m 초과: {c:6d}개 ({c / cnt * 100:5.1f}%)')
        if 'inter' in mk.ns.lower():
            over = sum(1 for x in lens if x > limit + 2.0)
            if over > cnt * 0.02:
                bad = True
                print(f'   -> [실패] {limit + 2.0:.1f} m 초과가 {over}개 '
                      f'({over / cnt * 100:.1f}%). 반대편 오매칭이 살아 있다.')
            else:
                print(f'   -> [통과] {limit + 2.0:.1f} m 초과 {over}개 '
                      f'({over / cnt * 100:.1f}%)')
        print()

    rclpy.shutdown()
    print('판정: ' + ('실패. max_constraint_distance 를 더 줄일 것.'
                     if bad else '통과. 루프 클로저가 근처끼리만 붙고 있다.'))
    return 1 if bad else 0


if __name__ == '__main__':
    raise SystemExit(main())
