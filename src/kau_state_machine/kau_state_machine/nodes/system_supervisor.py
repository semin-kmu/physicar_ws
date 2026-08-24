"""system_supervisor 노드. 기동·감시·복구·System 상태. 설계는 docs/01~04.

현 구현 범위는 **순차 spawn + 역순 종료**뿐이다 (docs/01 section 4-1, 11-1).
관문 G1~G4 · 재시도 · 감시 · enable 발행은 미구현.
"""

import os
import signal
import threading
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

from kau_state_machine.log import err, log
from kau_state_machine.ros.params import load_manifest, make_run_id, resolve_log_dir
from kau_state_machine.ros.process import dead, kill, spawn, wait_gate

PACKAGE = 'kau_state_machine'

# 관문 프로세스를 이만큼 기다려도 안 끝나면 죽이고 기동을 중단한다. 관문은
# 자기 타임아웃을 이미 갖고 있다 (clock_gate 는 bringup.yaml 에서 180 초).
# 여기는 그게 고장 났을 때의 마지막 덮개라 그보다 넉넉해야 한다.
GATE_TIMEOUT_SEC = 240.0


class SystemSupervisor(Node):
    """자신을 제외한 전 노드를 자식 프로세스로 소유한다."""

    def __init__(self) -> None:
        super().__init__('system_supervisor')

        default_yaml = os.path.join(
            get_package_share_directory(PACKAGE), 'config', 'bringup.yaml')
        self.declare_parameter('bringup_yaml', default_yaml)
        self.declare_parameter('run_id', '')

        manifest_path = self.get_parameter('bringup_yaml').value
        self._manifest = load_manifest(manifest_path)

        # use_sim_time 은 rclpy 가 자동 선언한다. 기본값은 launch 가 준다.
        self._use_sim_time = bool(self.get_parameter('use_sim_time').value)

        run_id = self.get_parameter('run_id').value or make_run_id()
        self._log_dir = resolve_log_dir(self._manifest.log_dir, run_id)

        self._children = []
        self._reported = set()
        self._lock = threading.Lock()

        log('supervisor', f'manifest={manifest_path}')
        log('supervisor', f'log_dir={self._log_dir}')
        log('supervisor', f'use_sim_time={self._use_sim_time}')

        # spawn 은 블로킹이므로 spin 과 분리한다 (01 section 7-2).
        self._thread = threading.Thread(target=self._bringup, daemon=True)
        self._thread.start()

    def _bringup(self) -> None:
        """단계 오름차순. 단계 안은 동시 spawn, 사이는 delay_sec 대기.

        delay_sec 은 다음 단계까지의 대기이자 생존 확인 전 정착 시간이다.
        """
        total = len(self._manifest.stages)
        skipped = 0
        for index, stage in enumerate(self._manifest.stages, 1):
            log('bringup', f'stage {index}/{total} (id={stage.id})')
            spawned = []
            for spec in stage.nodes:
                if not self._wanted(spec):
                    skipped += 1
                    log('skip', f'when={spec.when} · 이 기동에는 없다', node=spec.name)
                    continue
                try:
                    child = spawn(spec, self._log_dir, self._use_sim_time)
                except Exception as exc:  # noqa: BLE001
                    err('bringup', f'중단 · {spec.name} 실행 불가: {exc}')
                    return
                log('spawn', f'pid={child.pid}', node=spec.name)

                # 관문은 끝나는 게 정상이라 자식 목록에 넣지 않는다. 넣으면
                # 매 단계 사망 보고에 걸린다.
                if spec.wait:
                    if not self._gate(spec, child):
                        return
                    continue

                with self._lock:
                    self._children.append(child)
                spawned.append(child)

            if stage.delay_sec:
                time.sleep(stage.delay_sec)
            self._report_dead(spawned, f'stage {index}/{total}')

        with self._lock:
            everything = list(self._children)
        self._report_dead(everything, '전체')
        alive = len(everything) - len(dead(everything))
        tail = f' · 스킵 {skipped}' if skipped else ''
        log('bringup', f'spawn 완료 · 생존 {alive}/{len(everything)}{tail}')

    def _wanted(self, spec) -> bool:
        """when 이 이번 기동의 시계 소스와 맞는가. 맞지 않으면 띄우지 않는다."""
        if spec.when == 'sim':
            return self._use_sim_time
        if spec.when == 'real':
            return not self._use_sim_time
        return True

    def _gate(self, spec, child) -> bool:
        """관문 1 개를 통과했는가. 실패하면 기동을 중단한다 (docs/01 section 3)."""
        code = wait_gate(child, GATE_TIMEOUT_SEC)
        if code == 0:
            log('gate', '통과', node=spec.name)
            return True
        reason = (f'{GATE_TIMEOUT_SEC:.0f} 초 안에 안 끝나 죽였다'
                  if code is None else f'rc={code}')
        err('gate', f'중단 · {reason} · {self._log_dir}/{spec.name}.out',
            node=spec.name)
        return False

    def _report_dead(self, children, label: str) -> None:
        """죽은 자식만 알린다. 정상은 침묵 (관문 G1, docs/01 section 3)."""
        fresh = [c for c in dead(children) if c.name not in self._reported]
        if not fresh:
            return
        for child in fresh:
            self._reported.add(child.name)
            err('dead', f'rc={child.proc.returncode} · '
                        f'{self._log_dir}/{child.name}.out', node=child.name)
        alive = len(children) - len(dead(children))
        err('bringup', f'{label} 생존 {alive}/{len(children)}')

    def shutdown(self) -> None:
        """기동의 역순으로 정리. 실패해도 강행한다 (01 section 11-2)."""
        with self._lock:
            children, self._children = list(self._children), []
        if not children:
            return

        log('shutdown', '역순 정리 시작')
        for child in reversed(children):
            code = kill(child)
            log('kill', f'pid={child.pid} rc={code}', node=child.name)
        log('shutdown', '완료')


def main(args=None) -> None:
    """SIGINT/SIGTERM 을 직접 받아 곱게 내려온다.

    rclpy 기본 신호 처리(SignalHandlerOptions.ALL)는 컨텍스트를 **비동기로**
    내린다. rclpy.spin() 은 `while context.ok(): spin_once()` 인데, ok() 를
    통과한 직후 컨텍스트가 내려가면 wait set 생성이 RCLError 로 터진다.
    ExternalShutdownException 이 아니라서 안 잡히고 traceback 으로 끝나며,
    종료 코드도 0 이 아니게 되어 supervisor 의 사망 판정을 오염시킨다
    (실측 2026-08-24: state_machine.out 의 "failed to initialize wait set").

    신호를 직접 받아 루프를 빠져나오면 그 창 자체가 없다.
    platform_ekf_pause.py 와 같은 방식이다.
    """
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = SystemSupervisor()

    alive = [True]

    def stop(_signum, _frame):
        alive[0] = False

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    try:
        while alive[0] and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        # 자식 정리가 먼저다. 여기서 예외가 나면 자식이 고아로 남는다.
        node.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
