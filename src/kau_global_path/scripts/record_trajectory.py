#!/usr/bin/env python3
"""
teleop 주행 궤적을 **바퀴마다 한 파일씩** CSV 로 기록한다.
lane_graph 수기 편집의 좌표 기준이 된다.

localization.launch.py 가 떠 있고 2D Pose Estimate 로 초기화가 끝난 뒤에 실행한다.
map -> base_footprint TF 를 폴링해서 일정 거리 이상 움직였을 때만 한 점을 남긴다.
시간 기준으로 남기면 정지 구간에 점이 수천 개 쌓여서 편집기에서 못 쓴다.

    ros2 run kau_global_path record_trajectory.py --lane inner

**한 바퀴 돌고 Ctrl+C** 하면 된다. 실행할 때마다 비어 있는 가장 작은 번호로
들어간다.

    1 회차 -> data/inner_1.csv
    2 회차 -> data/inner_2.csv
    3 회차 -> data/inner_3.csv

    rm data/inner_2.csv          <- 2 번째 바퀴가 마음에 안 들면 지우고
    4 회차 -> data/inner_2.csv   <- 다시 돌리면 그 자리를 다시 채운다

멈추지 않고 계속 돌아도 된다. 시작점을 지날 때마다 그 바퀴가 파일로 닫히고
다음 번호가 이어진다. 편집기는 있는 파일을 전부 읽어 평균낸다.

한 바퀴를 못 채우고 끝내면 `<lane>_partial_<k>.csv` 로 나가고 평균에서 빠진다.
완주했다고 판단되면 `<lane>_<n>.csv` 로 이름만 바꾸면 그대로 쓰인다.
`<lane>_recording.csv` 는 진행 중인 바퀴를 흘려 쓰는 파일이라 정상 종료하면
사라진다. 남아 있으면 지난 실행이 비정상 종료한 것이고, 다음 실행이 덮어쓰기
전에 `_partial_` 로 살려 둔다.

lane 은 반드시 따로 기록한다 — 안쪽 몇 바퀴, 그 다음 `--lane outer` 로 바깥쪽.
한 파일에 섞으면 어느 구간이 어느 차로인지 라벨이 없어진다 (README 3.2).

단위: 미터 / 라디안. 프레임: map. (KauPath 의 cm 규약은 발행 단계에서 적용한다)
"""

import argparse
import csv
import math
import re
import signal
import sys
from pathlib import Path

import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time

import tf2_ros


def path_length(pts):
    return sum(math.dist(pts[i], pts[i - 1]) for i in range(1, len(pts)))


def resample_closed_from(lap, anchor, n=400):
    """한 바퀴를 `anchor` 최근접점에서 출발하도록 돌린 뒤 호길이 등간격 n 점.

    바퀴마다 잘린 위상이 다르면(다른 세션, 파일을 지웠을 때) 같은 k 번째
    표본이 트랙의 다른 자리를 가리킨다. 그 위상차가 그대로 "벌어짐" 으로
    잡혀서 측위 오차를 실제보다 크게 보이게 한다. anchor 로 맞춰서 없앤다.

    시간이 아니라 호길이로 재므로 바퀴마다 속도가 달라도 상관없다.
    """
    if len(lap) < 2:
        return []

    m = min(range(len(lap)), key=lambda i: math.dist(lap[i], anchor))
    rot = list(lap[m:]) + list(lap[:m])
    rot.append(rot[0])                      # 폐곡선으로 닫는다

    cum = [0.0]
    for i in range(1, len(rot)):
        cum.append(cum[-1] + math.dist(rot[i], rot[i - 1]))

    total = cum[-1]
    if total < 1e-9:
        return []

    out, j = [], 0
    for k in range(n):
        target = total * k / n
        while j < len(cum) - 2 and cum[j + 1] < target:
            j += 1
        span = cum[j + 1] - cum[j]
        u = 0.0 if span < 1e-12 else (target - cum[j]) / span
        out.append((rot[j][0] + u * (rot[j + 1][0] - rot[j][0]),
                    rot[j][1] + u * (rot[j + 1][1] - rot[j][1])))
    return out


def _closest_on_segment(p, a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    l2 = dx * dx + dy * dy
    if l2 < 1e-18:
        return a, math.dist(p, a)
    t = max(0.0, min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / l2))
    q = (a[0] + t * dx, a[1] + t * dy)
    return q, math.dist(p, q)


def _nearest_windowed(p, samples, k, w):
    """`samples` 폐곡선 위에서 index k 주변 +-w 구간 중 p 에 가장 가까운 점.

    전역 최근접을 쓰면 헤어핀처럼 트랙이 자기 자신과 가까워지는 곳에서
    반대편 구간에 붙는다. 창을 씌우면 그럴 일이 없다. 표본이 아니라 선분에
    투영하므로 표본 간격만큼의 오차도 안 생긴다.
    """
    n = len(samples)
    best, best_d = None, float('inf')
    for off in range(-w, w + 1):
        i = (k + off) % n
        q, d = _closest_on_segment(p, samples[i], samples[(i + 1) % n])
        if d < best_d:
            best, best_d = q, d
    return best, best_d


def align_laps(laps, n=400):
    """바퀴들을 겹쳐서 (평균 곡선, 벌어짐[m]) 을 낸다.

    대응은 **호길이 비율이 아니라 최근접점**으로 잡는다. 바퀴마다 주행거리가
    조금씩 다르면 호길이 비율은 같은 k 번째 표본을 트랙의 다른 자리에 놓고,
    그 세로 어긋남이 가로 오차로 둔갑해서 측위를 실제보다 나쁘게 보이게 한다
    (무노이즈 합성 궤적에서 1.8 cm 를 쟀다. 정답은 0 이다).

    벌어짐이 측위 반복정밀도이고 global path 정확도의 하한이다 (README 6.1).
    `lane_editor.js` 의 alignLaps 와 같은 알고리즘이다.
    """
    laps = [l for l in laps if len(l) >= 2]
    if not laps:
        return [], 0.0
    if len(laps) == 1:
        return list(laps[0]), 0.0

    anchor = laps[0][0]
    sampled = [s for s in (resample_closed_from(l, anchor, n) for l in laps) if s]
    if len(sampled) < 2:
        return (sampled[0] if sampled else []), 0.0

    ref = sampled[0]
    w = max(4, n // 10)
    mean, spread = [], 0.0

    for k in range(n):
        group = [ref[k]]
        for other in sampled[1:]:
            group.append(_nearest_windowed(ref[k], other, k, w)[0])

        mx = sum(q[0] for q in group) / len(group)
        my = sum(q[1] for q in group) / len(group)
        mean.append((mx, my))
        spread = max(spread, max(math.dist(q, (mx, my)) for q in group))

    return mean, spread


def lap_spread(laps, n=400):
    return align_laps(laps, n)[1]


def default_out_dir():
    """편집기가 fetch 로 읽는 `src/kau_global_path/data/` 를 찾는다.

    `ros2 run` 으로 띄우면 __file__ 이 install/ 아래라서 거기서 거슬러 올라가면
    엉뚱한 곳에 쓴다. 소스 트리 옆(직접 실행) 과 현재 위치에서 위로 올라가며
    찾는 것(워크스페이스 루트에서 ros2 run) 둘 다 본다. 못 찾으면 현재 위치다.
    """
    beside = Path(__file__).resolve().parent.parent / 'data'
    if beside.is_dir():
        return beside

    for base in [Path.cwd(), *Path.cwd().parents]:
        cand = base / 'src' / 'kau_global_path' / 'data'
        if cand.is_dir():
            return cand

    return Path.cwd()


def next_free(out_dir, pattern, fmt):
    """`pattern` 에 안 잡힌 가장 작은 번호. **빈 자리를 채운다.**

    `inner_2.csv` 를 지우고 다시 주행하면 다시 `inner_2.csv` 로 들어간다.
    최대값+1 로 하면 지운 자리가 영영 비어서 번호가 실제 바퀴 수와 어긋난다.
    """
    used = set()
    for f in out_dir.glob(fmt.format(n='*')):
        m = re.fullmatch(pattern, f.stem)
        if m:
            used.add(int(m.group(1)))

    n = 1
    while n in used:
        n += 1
    return n


def next_lap_number(out_dir, lane):
    return next_free(out_dir, rf'{re.escape(lane)}_(\d+)', f'{lane}_{{n}}.csv')


def next_partial_number(out_dir, lane):
    # 바퀴 번호와 다른 대역을 쓴다. 미완주가 완주 번호 자리를 막으면 안 된다.
    return next_free(out_dir, rf'{re.escape(lane)}_partial_(\d+)',
                     f'{lane}_partial_{{n}}.csv')


def yaw_from_quat(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


class TrajectoryRecorder(Node):

    def __init__(self, args):
        super().__init__('record_trajectory')

        self.target = args.target_frame
        self.source = args.source_frame
        self.min_dist = args.min_dist
        self.max_gap = args.max_gap
        self.radius = args.lap_radius
        self.min_lap = args.min_lap_length

        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer, self)

        self.out_dir = Path(args.out_dir).expanduser()
        self.out_dir.mkdir(parents=True, exist_ok=True)
        self.lane = args.lane
        self.rescue_previous()

        # 진행 중인 바퀴는 여기로 흘려 쓴다. 다 돌면 최종 이름으로 옮긴다.
        # 크래시로 죽어도 여기까지는 디스크에 남는다.
        self.rec_path = self.out_dir / f'{self.lane}_recording.csv'
        self.open_recording()

        self.anchor = None       # 세션 첫 점. 모든 바퀴를 여기서 자른다
        self.lap = []            # 이번 바퀴의 (t, x, y, yaw)
        self.lap_acc = 0.0       # 이번 바퀴 누적 호길이
        self.best = None         # 복귀 반경 안에 있는 동안의 (index, 거리)
        self.saved = []          # 닫은 바퀴들의 (파일명, 점열)
        self.last = None
        self.total = 0.0
        self.t0 = None
        self.warned = False
        self.tf_ok = 0           # TF 를 실제로 받은 횟수. 안 움직인 것과 구분하려고

        self.create_timer(1.0 / args.rate, self.tick)

        self.get_logger().info(
            f'{self.target} -> {self.source} 기록 시작 '
            f'(min_dist={self.min_dist:.3f} m, 복귀 반경={self.radius:.2f} m)\n'
            f'  -> {self.out_dir.resolve()}/{self.lane}_{next_lap_number(self.out_dir, self.lane)}.csv')

    # ---------------------------------------------------------------- 기록

    def tick(self):
        try:
            tf = self.buffer.lookup_transform(
                self.target, self.source, Time(),
                timeout=Duration(seconds=0.1))
        except tf2_ros.TransformException as exc:
            if not self.warned:
                self.get_logger().warn(f'TF 대기 중: {exc}')
                self.warned = True
            return

        self.warned = False
        self.tf_ok += 1

        t = tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9
        x = tf.transform.translation.x
        y = tf.transform.translation.y
        yaw = yaw_from_quat(tf.transform.rotation)

        if self.t0 is None:
            self.t0 = t

        if self.last is not None:
            d = math.dist((x, y), self.last)
            if d < self.min_dist:
                return
            if d > self.max_gap:
                # 측위가 튀었다. 점을 남기되 눈에 띄게 경고한다.
                self.get_logger().warn(
                    f'{d:.2f} m 점프 ({self.max_gap:.2f} m 초과) — 측위가 재수렴한 '
                    '자리일 수 있다. 이 바퀴 파일은 지우는 편이 낫다')
            self.total += d
            self.lap_acc += d

        row = (round(t - self.t0, 3), round(x, 4), round(y, 4), round(yaw, 4))
        self.rec.writerow(row)
        self.lap.append(row)
        self.last = (x, y)

        if self.anchor is None:
            self.anchor = (x, y)

        self.check_closure()

        if len(self.lap) % 50 == 0:
            self.rec_fp.flush()
            back = math.dist((x, y), self.anchor)
            self.get_logger().info(
                f'{len(self.lap)} 점  {self.lap_acc:.2f} m  '
                f'현재 ({x:+.2f}, {y:+.2f})  시작점까지 {back:.2f} m')

    def check_closure(self):
        """시작점으로 돌아왔으면 이번 바퀴를 닫는다.

        복귀 "구간" 전체가 반경 안에 들어오므로, 반경 안에 머무는 동안 가장
        가까웠던 점을 기억해 뒀다가 반경을 벗어나는 순간 거기서 자른다.
        들어오자마자 자르면 매번 조금씩 다른 자리에서 잘려 바퀴들이 어긋난다.

        기준은 **직전 바퀴의 시작점이 아니라 세션 첫 점(anchor)** 이다. 직전
        바퀴를 기준으로 삼으면 자를 때마다 표본 간격만큼 밀려서 그 오차가
        바퀴 수만큼 쌓인다 (5 바퀴에 11 cm 를 쟀다). anchor 를 고정하면
        모든 바퀴가 같은 자리에서 잘려 길이도 위상도 맞는다.
        """
        if self.lap_acc <= self.min_lap or self.anchor is None:
            return

        here = (self.lap[-1][1], self.lap[-1][2])
        d = math.dist(here, self.anchor)

        if d < self.radius:
            if self.best is None or d < self.best[1]:
                self.best = (len(self.lap) - 1, d)
        elif self.best is not None:
            self.close_lap(self.best[0], self.best[1])
            self.best = None

    def close_lap(self, cut, gap):
        head, tail = self.lap[:cut + 1], self.lap[cut + 1:]

        # 번호는 지금 정한다. 시작 시점에 정해두면 그 사이에 파일을 지웠을 때
        # 빈 자리를 못 채운다.
        name = f'{self.lane}_{next_lap_number(self.out_dir, self.lane)}.csv'
        self.write(name, head)

        pts = [(r[1], r[2]) for r in head]
        self.saved.append((name, pts))
        self.get_logger().info(
            f'*** {name} 저장 — {len(head)} 점 · {path_length(pts):.2f} m · '
            f'시작점과 {gap * 100:.0f} cm')

        self.lap = tail
        self.lap_acc = path_length([(r[1], r[2]) for r in tail])
        self.open_recording(tail)

    def write(self, name, rows):
        with (self.out_dir / name).open('w', newline='') as fp:
            w = csv.writer(fp)
            w.writerow(['t', 'x', 'y', 'yaw'])
            w.writerows(rows)

    def open_recording(self, rows=()):
        self.rec_fp = self.rec_path.open('w', newline='')
        self.rec = csv.writer(self.rec_fp)
        self.rec.writerow(['t', 'x', 'y', 'yaw'])
        self.rec.writerows(rows)
        self.rec_fp.flush()

    def rescue_previous(self):
        """지난 실행이 비정상 종료해 남긴 recording 파일을 살려 둔다.

        그냥 두면 이번 실행이 덮어써서 지난 주행이 통째로 사라진다.
        한 번에 한 바퀴씩 도는 사용법에서는 그게 곧 한 바퀴 전체다.
        """
        stale = self.out_dir / f'{self.lane}_recording.csv'
        if not stale.is_file() or stale.stat().st_size < 200:
            return

        name = f'{self.lane}_partial_{next_partial_number(self.out_dir, self.lane)}.csv'
        stale.rename(self.out_dir / name)
        self.get_logger().warn(
            f'지난 실행이 비정상 종료했다 -> {name} 로 살려 뒀다. '
            '완주한 바퀴였다면 이름을 바꿔서 쓸 것')

    # ---------------------------------------------------------------- 종료

    def finish(self):
        try:
            self.rec_fp.flush()
            self.rec_fp.close()
        except Exception:
            pass

        # SIGTERM 으로 죽으면 rclpy context 가 먼저 닫혀서 get_logger 가 조용히
        # 실패한다. 요약은 결과물이므로 stdout 으로도 반드시 남긴다.
        def say(msg):
            print(msg, flush=True)
            try:
                self.get_logger().info(msg)
            except Exception:
                pass

        if not self.saved and len(self.lap) < 2:
            self.rec_path.unlink(missing_ok=True)

            if self.tf_ok == 0:
                say(f'[실패] {self.target} -> {self.source} TF 를 한 번도 못 받았다.\n'
                    '  localization 이 떠 있고 2D Pose Estimate 로 초기화되었는지 확인할 것')
            else:
                # TF 는 멀쩡했다. 여기서 측위를 의심하게 만들면 안 된다.
                say(f'[실패] TF 는 {self.tf_ok} 번 받았는데 차가 거의 안 움직였다 '
                    f'({self.total * 100:.0f} cm).\n'
                    f'  주행을 시작한 뒤에 기록할 것. '
                    f'(정지 판정 기준: 한 점당 {self.min_dist * 100:.0f} cm 이상 이동)')
            return

        partial, gap, dropped = None, 0.0, 0.0

        if self.lap:
            anchor = self.anchor or (self.lap[0][1], self.lap[0][2])
            gap = math.dist((self.lap[-1][1], self.lap[-1][2]), anchor)

            # 이미 닫은 바퀴가 있으면, 그 뒤에 조금 더 굴러간 꼬리는 버린다.
            # 매 실행마다 몇십 cm 짜리 _partial 이 쌓이는 게 더 성가시다.
            trivial = self.min_lap
            if self.saved:
                trivial = max(trivial, 0.15 * path_length(self.saved[-1][1]))

            if self.lap_acc > self.min_lap and gap < self.radius:
                self.close_lap(len(self.lap) - 1, gap)
            elif self.lap_acc < trivial:
                dropped = self.lap_acc
            else:
                partial = (f'{self.lane}_partial_'
                           f'{next_partial_number(self.out_dir, self.lane)}.csv')
                self.write(partial, self.lap)

        self.rec_path.unlink(missing_ok=True)

        report = [f'기록 종료 — {self.out_dir.resolve()}',
                  f'  전체 주행 : {self.total:.2f} m']

        if self.saved:
            report.append(f'  저장된 바퀴 : {len(self.saved)} 개')
            for name, pts in self.saved:
                report.append(
                    f'    {name:24s} {len(pts):5d} 점  {path_length(pts):6.2f} m')
        else:
            report.append('  저장된 바퀴 : 0 개')

        if partial:
            pts = [(r[1], r[2]) for r in self.lap]
            report.append(
                f'    {partial:24s} {len(pts):5d} 점  {path_length(pts):6.2f} m'
                '  <- 미완주')
            report.append(
                f'      시작점에서 {gap:.2f} m 떨어진 곳에서 끝났다 '
                f'(완주 판정 기준 {self.radius:.2f} m)')
            report.append(
                f'      한 바퀴를 다 돌았다고 보면 '
                f'{self.lane}_{next_lap_number(self.out_dir, self.lane)}.csv '
                '로 이름을 바꾸면 그대로 쓰인다')
        elif dropped:
            report.append(f'  (마지막 {dropped:.2f} m 는 짧아서 버렸다)')

        # 이 폴더에 있는 것 전부를 기준으로 알려준다. 지난 실행 결과가 같이 보여야
        # "몇 바퀴 더 돌아야 하는지" 를 판단할 수 있다.
        all_laps = self.load_all_laps()
        if len(all_laps) >= 2:
            spread = lap_spread([pts for _, pts in all_laps])
            verdict = ('좋다' if spread < 0.10 else
                       '쓸 만하다' if spread < 0.15 else '크다 — 측위를 먼저 볼 것')
            # 이 값보다 정밀하게 lane 을 그려봤자 의미가 없다 (README 6.1).
            report.append(f'  {self.lane} 전체 {len(all_laps)} 바퀴 '
                          f'({", ".join(n for n, _ in all_laps)})')
            report.append(f'  바퀴 간 벌어짐 : {spread * 100:.1f} cm ({verdict})')

            lens = sorted(path_length(pts) for _, pts in all_laps)
            mid = lens[len(lens) // 2]
            odd = [n for n, pts in all_laps if abs(path_length(pts) - mid) > 0.10 * mid]
            if odd:
                report.append(f'  길이가 튀는 바퀴 : {", ".join(odd)} — 지우고 다시 도는 것을 검토')
        elif len(all_laps) == 1:
            report.append('  이 lane 은 아직 1 바퀴다. 더 돌면 평균으로 지터를 줄일 수 있다')

        report.append('  잘못 돈 바퀴는 그 csv 만 지우면 된다. '
                      '다시 돌리면 그 번호를 다시 채운다')
        say('\n'.join(report))

    def load_all_laps(self):
        """폴더에 있는 이 lane 의 완주 바퀴 전부. 지난 실행 것까지 포함한다."""
        out = []
        for n in sorted(
                int(re.fullmatch(rf'{re.escape(self.lane)}_(\d+)', f.stem).group(1))
                for f in self.out_dir.glob(f'{self.lane}_*.csv')
                if re.fullmatch(rf'{re.escape(self.lane)}_(\d+)', f.stem)):
            path = self.out_dir / f'{self.lane}_{n}.csv'
            with path.open() as fp:
                pts = [(float(r['x']), float(r['y'])) for r in csv.DictReader(fp)]
            if len(pts) >= 10:
                out.append((path.name, pts))
        return out


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--lane', choices=['inner', 'outer'], default='inner',
                   help='파일 이름 앞부분. <lane>_1.csv, <lane>_2.csv ...')
    p.add_argument('--out-dir', default=None, help='출력 디렉터리 (기본: 패키지 data/)')
    p.add_argument('--min-dist', type=float, default=0.02,
                   help='m. 이만큼 움직여야 한 점을 남긴다')
    p.add_argument('--max-gap', type=float, default=0.30,
                   help='m. 이보다 크게 튀면 경고한다 (측위 재수렴 의심)')
    p.add_argument('--lap-radius', type=float, default=0.40,
                   help='m. 시작점에 이만큼 다가오면 한 바퀴로 친다')
    p.add_argument('--min-lap-length', type=float, default=2.0,
                   help='m. 이보다 짧으면 바퀴로 안 친다 (출발 직후 되돌기 방지)')
    p.add_argument('--rate', type=float, default=20.0, help='Hz, TF 폴링 주기')
    p.add_argument('--target-frame', default='map')
    p.add_argument('--source-frame', default='base_footprint')
    args, ros_args = p.parse_known_args()

    if args.out_dir is None:
        args.out_dir = str(default_out_dir())

    # Ctrl+C 는 KeyboardInterrupt 로 잡히지만 SIGTERM(kill, launch 종료, timeout)
    # 은 그냥 죽는다. 같은 경로로 돌려서 미완주 구간과 요약을 잃지 않는다.
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt))

    rclpy.init(args=ros_args)
    node = TrajectoryRecorder(args)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.finish()
        except Exception as exc:                      # 요약이 실패해도 파일은 남는다
            print(f'[경고] 요약 중 오류: {exc}', flush=True)

        # context 가 이미 내려간 뒤면 destroy/shutdown 이 던진다. 정리 실패로
        # 종료 코드를 더럽히지 않는다 — 바퀴 파일은 이미 디스크에 있다.
        for step in (node.destroy_node, rclpy.shutdown):
            try:
                step()
            except Exception:
                pass


if __name__ == '__main__':
    sys.exit(main())
