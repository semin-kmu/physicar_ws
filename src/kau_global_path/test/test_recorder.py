#!/usr/bin/env python3
"""
record_trajectory.py 의 바퀴 파일 관리 검증. ROS 없이 돈다.

  python3 src/kau_global_path/test/test_recorder.py

Node 를 띄우지 않고 실제 메서드(check_closure / close_lap / finish)를 그대로
호출한다. 로직을 테스트용으로 베껴 쓰면 진짜 코드가 바뀔 때 같이 안 바뀐다.

가장 중요한 것은 [3] 이다 — 한 번 실행에 한 바퀴, 마음에 안 들면 그 파일만
지우고 다시 돌리면 **그 번호를 다시 채운다.**
"""

import importlib.util
import math
import shutil
import sys
import tempfile
import types
from pathlib import Path

HERE = Path(__file__).resolve().parent
SCRIPT = HERE.parent / 'scripts' / 'record_trajectory.py'


def load_module():
    """rclpy / tf2_ros 없이 모듈을 읽는다. 쓰는 건 순수 함수와 메서드뿐이다."""
    for name in ('rclpy', 'rclpy.duration', 'rclpy.executors', 'rclpy.node',
                 'rclpy.time', 'tf2_ros'):
        sys.modules.setdefault(name, types.ModuleType(name))

    sys.modules['rclpy.node'].Node = object
    sys.modules['rclpy.duration'].Duration = object
    sys.modules['rclpy.time'].Time = object
    sys.modules['rclpy.executors'].ExternalShutdownException = Exception
    sys.modules['tf2_ros'].TransformException = Exception
    sys.modules['tf2_ros'].Buffer = object
    sys.modules['tf2_ros'].TransformListener = object

    spec = importlib.util.spec_from_file_location('record_trajectory', SCRIPT)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


RT = load_module()
FAILED = 0
QUIET = '-v' not in sys.argv


def check(name, ok, detail=''):
    global FAILED
    print(f'{"  통과" if ok else "  실패"}  {name}   {detail}')
    if not ok:
        FAILED += 1


class Logger:
    def info(self, *_a, **_k):
        pass

    warn = error = info


def make_recorder(out_dir, lane='inner', radius=0.40, min_lap=2.0):
    """Node.__init__ 을 건너뛰고 필요한 상태만 채운다."""
    r = object.__new__(RT.TrajectoryRecorder)
    r.out_dir = Path(out_dir)
    r.lane = lane
    r.radius = radius
    r.min_lap = min_lap
    r.anchor = None
    r.lap = []
    r.lap_acc = 0.0
    r.best = None
    r.saved = []
    r.total = 0.0
    r.get_logger = lambda: Logger()
    r.rescue_previous()
    r.rec_path = r.out_dir / f'{lane}_recording.csv'
    r.open_recording()
    return r


def drive(out_dir, pts, lane='inner'):
    """한 번 실행 = 한 바퀴. tick() 의 기록 부분만 흉내낸다."""
    rec = make_recorder(out_dir, lane)
    for i, (x, y) in enumerate(pts):
        if rec.lap:
            d = math.dist((x, y), (rec.lap[-1][1], rec.lap[-1][2]))
            rec.total += d
            rec.lap_acc += d
        row = (round(i * 0.05, 3), round(x, 4), round(y, 4), 0.0)
        rec.rec.writerow(row)
        rec.lap.append(row)
        if rec.anchor is None:
            rec.anchor = (x, y)
        rec.check_closure()

    import io
    import contextlib
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf if QUIET else sys.stdout):
        rec.finish()
    return rec


A, B, CX, CY = 2.0, 5.0, 0.1, 4.6
PER_LAP = 1150                     # 2 cm 간격 * 23 m


def oval(n=PER_LAP, laps=1, noise=0.0, k=0, stop_short=0):
    out = []
    for lap in range(laps):
        total = n if not stop_short or lap < laps - 1 else n - stop_short
        for i in range(total):
            t = 2 * math.pi * i / n
            r = noise * math.sin(3 * t + (k + lap) * 2.1)
            out.append((CX + (A + r) * math.cos(t), CY + (B + r) * math.sin(t)))
    return out


def laps_in(d, lane='inner'):
    import re
    return sorted(
        (f.name for f in d.glob(f'{lane}_*.csv')
         if re.fullmatch(rf'{lane}_\d+', f.stem)),
        key=lambda n: int(n.split('_')[1].split('.')[0]))


tmp = Path(tempfile.mkdtemp())

print('\n[1] 한 번 실행 = 한 바퀴 = 파일 하나')
d = tmp / 'one'
d.mkdir()
for i in range(3):
    drive(d, oval(noise=0.02, k=i))
check('3 번 실행 -> inner_1 2 3', laps_in(d) == ['inner_1.csv', 'inner_2.csv', 'inner_3.csv'],
      f'{laps_in(d)}')
check('recording 파일이 안 남는다', not (d / 'inner_recording.csv').exists())
check('_partial 이 안 생긴다', not list(d.glob('*partial*')),
      f'{[f.name for f in d.glob("*partial*")]}')

print('\n[2] 멈추지 않고 여러 바퀴 돌아도 나뉜다')
d = tmp / 'multi'
d.mkdir()
drive(d, oval(laps=3))
check('한 번 실행에 3 바퀴 -> 파일 3 개',
      laps_in(d) == ['inner_1.csv', 'inner_2.csv', 'inner_3.csv'], f'{laps_in(d)}')

print('\n[3] 지운 번호를 다시 채운다  <- 핵심')
d = tmp / 'refill'
d.mkdir()
for i in range(4):
    drive(d, oval(noise=0.02, k=i))
check('4 번 실행', laps_in(d) == [f'inner_{i}.csv' for i in (1, 2, 3, 4)], f'{laps_in(d)}')

(d / 'inner_2.csv').unlink()
check('inner_2 삭제됨', laps_in(d) == ['inner_1.csv', 'inner_3.csv', 'inner_4.csv'], f'{laps_in(d)}')
check('다음 번호가 2', RT.next_lap_number(d, 'inner') == 2)

drive(d, oval(noise=0.02, k=9))
check('다시 돌리면 inner_2 로 들어간다',
      laps_in(d) == [f'inner_{i}.csv' for i in (1, 2, 3, 4)], f'{laps_in(d)}')

(d / 'inner_1.csv').unlink()
(d / 'inner_3.csv').unlink()
drive(d, oval(noise=0.02, k=8))
drive(d, oval(noise=0.02, k=7))
check('구멍 두 개도 작은 번호부터 채운다',
      laps_in(d) == [f'inner_{i}.csv' for i in (1, 2, 3, 4)], f'{laps_in(d)}')

print('\n[4] lane 은 서로 번호를 안 건드린다')
d = tmp / 'lanes'
d.mkdir()
drive(d, oval(), 'inner')
drive(d, oval(), 'outer')
drive(d, oval(), 'outer')
check('inner 1 개 / outer 2 개',
      laps_in(d, 'inner') == ['inner_1.csv'] and
      laps_in(d, 'outer') == ['outer_1.csv', 'outer_2.csv'],
      f'{laps_in(d, "inner")} {laps_in(d, "outer")}')

print('\n[5] 미완주는 _partial_ 로 빠지고 바퀴 번호를 안 먹는다')
d = tmp / 'partial'
d.mkdir()
drive(d, oval())
drive(d, oval(stop_short=PER_LAP // 3))          # 한 바퀴를 못 채우고 종료
names = sorted(f.name for f in d.glob('inner_*.csv'))
check('완주 1 + 미완주 1',
      laps_in(d) == ['inner_1.csv'] and 'inner_partial_1.csv' in names, f'{names}')
check('미완주가 inner_2 자리를 안 막는다', RT.next_lap_number(d, 'inner') == 2)
drive(d, oval())
check('다음 완주가 inner_2 로', laps_in(d) == ['inner_1.csv', 'inner_2.csv'], f'{laps_in(d)}')

print('\n[6] 비정상 종료가 남긴 recording 을 덮어쓰지 않는다')
d = tmp / 'crash'
d.mkdir()
rec = make_recorder(d)
for i, (x, y) in enumerate(oval(stop_short=PER_LAP // 2)):
    rec.rec.writerow((i * 0.05, round(x, 4), round(y, 4), 0.0))
rec.rec_fp.flush()                                # finish() 없이 죽은 상황
before = (d / 'inner_recording.csv').read_text()
drive(d, oval())                                  # 다음 실행
rescued = sorted(f.name for f in d.glob('inner_partial_*.csv'))
check('_partial_ 로 살려 둔다', rescued == ['inner_partial_1.csv'], f'{rescued}')
check('내용이 그대로다', (d / 'inner_partial_1.csv').read_text() == before)
check('이번 실행은 정상 저장', laps_in(d) == ['inner_1.csv'], f'{laps_in(d)}')

print('\n[7] 자른 자리와 벌어짐')
d = tmp / 'quality'
d.mkdir()
for i in range(5):
    drive(d, oval(noise=0.03, k=i, n=round(PER_LAP * (1 + 0.1 * i))))
rec = make_recorder(d)
rec.rec_fp.close()
(d / 'inner_recording.csv').unlink(missing_ok=True)
all_laps = rec.load_all_laps()
check('폴더의 바퀴를 전부 읽는다', len(all_laps) == 5, f'{[n for n, _ in all_laps]}')
lens = [RT.path_length(p) for _, p in all_laps]
check('바퀴 길이가 고르다', max(lens) - min(lens) < 0.10,
      f'{min(lens):.2f} ~ {max(lens):.2f} m')
sp = RT.lap_spread([p for _, p in all_laps])
check('벌어짐이 노이즈 진폭 수준', 0.015 < sp < 0.045, f'{sp * 100:.1f} cm')

print('\n[8] 출발 직후 되돌아도 바퀴로 안 친다')
d = tmp / 'wiggle'
d.mkdir()
drive(d, [(CX + A, CY + i * 0.02) for i in range(30)] +
         [(CX + A, CY + (30 - i) * 0.02) for i in range(30)])
check('0.6 m 왕복은 바퀴가 아니다', laps_in(d) == [], f'{laps_in(d)}')
check('짧은 꼬리는 _partial 도 안 만든다', not list(d.glob('*partial*')),
      f'{[f.name for f in d.glob("*partial*")]}')

shutil.rmtree(tmp)
print(f'\n실패 {FAILED} 건\n' if FAILED else '\n전부 통과\n')
sys.exit(1 if FAILED else 0)
