#!/usr/bin/env python3
"""기동 시퀀스 검증용 스텁. ROS 없이 argv 만 기록하고 산다.

  --print-argv    받은 argv 를 stdout 에 1줄씩 남긴다
  --ignore-sigterm  SIGTERM 무시 (SIGKILL 에스컬레이션 확인용)
  --child         자식 프로세스를 하나 더 띄운다 (killpg 확인용)
  --exit-now N    즉시 종료 코드 N 으로 죽는다 (G1 실패 주입용)
"""

import os
import signal
import subprocess
import sys
import time


def main() -> None:
    argv = sys.argv[1:]
    if '--print-argv' in argv:
        for arg in sys.argv:
            print(arg, flush=True)
    if '--exit-now' in argv:
        sys.exit(int(argv[argv.index('--exit-now') + 1]))
    if '--ignore-sigterm' in argv:
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
    if '--child' in argv:
        subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(600)'])
    print(f'pid={os.getpid()} pgid={os.getpgid(0)}', flush=True)
    while True:
        time.sleep(1)


if __name__ == '__main__':
    main()
