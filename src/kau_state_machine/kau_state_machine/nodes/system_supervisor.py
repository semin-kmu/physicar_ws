"""system_supervisor 노드. 기동·감시·복구·System 상태. 설계는 docs/01~04.

현 구현 범위는 **순차 spawn + 기동 후 상시 감시 + 역순 종료**다
(docs/01 section 4-1, 11-1). 관문 G2~G4 · enable 발행은 미구현.

감시는 launch 가 하던 일을 대신한다. 담당자들이 쓰는 launch 파일에는
Node(respawn=True) 와 on_exit=Shutdown 이 걸려 있는데, 여기서 노드를 직접
실행하면 그게 사라진다. bringup.yaml 의 respawn·critical 이 그 자리를 메운다.
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

# 기동이 끝난 뒤 자식 생사를 이 주기로 본다. 죽은 것을 알아채는 지연이자
# respawn 대기의 해상도다. 1 초면 ekf.launch.py 의 respawn_delay 2 초를
# 눈에 띄게 늦추지 않는다.
#
# **벽시계(monotonic)로 잰다.** ROS 타이머로 만들면 안 된다 -- rclpy 의
# create_timer 는 clock 을 안 주면 노드 시계를 쓰는데(node.py: `if clock is
# None: clock = self._clock`), 이 노드는 use_sim_time 이라 그게 sim 시계다.
# 그러면 sim 시계가 멈추거나 거꾸로 뛸 때 감시도 같이 멈춘다 -- 감시가 잡으려는
# 고장이 바로 그것이다.
#
# 실측 2026-08-24: 맵 리로드로 sim 시계가 뒤로 뛰자 감시가 서서, 죽은 EKF 를
# 되살리지 못했다. platform_ekf_pause 가 "우리 EKF 0 개" 를 세 번 보고 물러났고
# 기동 전체가 내려갔다. platform_ekf_pause.py 가 monotonic 을 쓰는 이유와 같다.
WATCH_PERIOD_SEC = 1.0


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

        # manifest 가 쓰는 스위치마다 `option.<이름>` 을 연다. 기본은 켬이라
        # 아무것도 안 주면 지금까지와 똑같이 전부 뜬다.
        self._options = {
            name: bool(self.declare_parameter(f'option.{name}', True).value)
            for name in self._manifest.options}

        run_id = self.get_parameter('run_id').value or make_run_id()
        self._log_dir = resolve_log_dir(self._manifest.log_dir, run_id)

        self._children = []
        self._reported = set()
        self._lock = threading.Lock()

        # 감시가 죽은 자식의 spec 을 다시 봐야 한다 (respawn·critical).
        self._specs = {spec.name: spec
                       for stage in self._manifest.stages for spec in stage.nodes}
        self._pending = {}      # 이름 -> 재시작 예정 monotonic 시각
        self._ready = False     # 기동이 끝나야 감시를 시작한다
        self._stopping = False  # 종료 중. spawn 과 겹치면 고아가 생긴다

        log('supervisor', f'manifest={manifest_path}')
        log('supervisor', f'log_dir={self._log_dir}')
        log('supervisor', f'use_sim_time={self._use_sim_time}')

        # spawn 은 블로킹이므로 spin 과 분리한다 (01 section 7-2).
        self._thread = threading.Thread(target=self._bringup, daemon=True)
        self._thread.start()

        # 감시도 별도 스레드다. ROS 타이머가 아닌 이유는 WATCH_PERIOD_SEC 주석 참고.
        self._watch_thread = threading.Thread(target=self._watch_loop, daemon=True)
        self._watch_thread.start()

    @property
    def stopping(self) -> bool:
        return self._stopping

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
                if self._stopping:
                    log('bringup', '종료 요청 · 기동 중단')
                    return
                if not self._wanted(spec):
                    skipped += 1
                    log('skip', f'{self._skip_reason(spec)} · 이 기동에는 없다',
                        node=spec.name)
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

                # 종료와 겹치면 이 자식은 목록에 못 들어가 고아가 된다.
                # 자물쇠 안에서 확인하고, 이미 늦었으면 여기서 회수한다.
                with self._lock:
                    if self._stopping:
                        kill(child)
                        log('bringup', '종료 요청 · 방금 띄운 것을 회수했다',
                            node=spec.name)
                        return
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
        self._ready = True
        log('watch', f'상시 감시 시작 · {WATCH_PERIOD_SEC:.0f} 초 주기')

    def _wanted(self, spec) -> bool:
        """이번 기동에 띄울 노드인가. when(시계 소스) · option(스위치) 둘 다 본다."""
        if spec.option and not self._options.get(spec.option, True):
            return False
        if spec.when == 'sim':
            return self._use_sim_time
        if spec.when == 'real':
            return not self._use_sim_time
        return True

    def _skip_reason(self, spec) -> str:
        if spec.option and not self._options.get(spec.option, True):
            return f'option.{spec.option}=false'
        return f'when={spec.when}'

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

    def _watch_loop(self) -> None:
        """monotonic 벽시계로 도는 감시 루프. sim 시계와 무관하다."""
        while not self._stopping:
            time.sleep(WATCH_PERIOD_SEC)
            try:
                self._watch()
            except Exception as exc:  # noqa: BLE001
                # 감시가 예외로 죽으면 그 뒤로는 아무도 자식을 안 본다.
                err('watch', f'감시 주기에서 예외 · 계속한다: {exc}')

    def _watch(self) -> None:
        """기동 후 상시 감시. launch 의 respawn / on_exit=Shutdown 을 대신한다.

        기동 중에는 돌지 않는다 -- 그때는 _bringup 이 단계마다 _report_dead 로
        같은 일을 하고, 아직 안 뜬 노드를 죽었다고 볼 수 없기 때문이다.
        """
        if not self._ready or self._stopping:
            return

        now = time.monotonic()
        with self._lock:
            gone = [c for c in self._children if c.proc.poll() is not None]

        for child in gone:
            spec = self._specs.get(child.name)
            if spec is None:            # 있을 수 없지만 감시가 죽으면 안 된다
                continue

            if spec.critical:
                err('watch', f'rc={child.proc.returncode} · 필수 프로세스가 죽었다 · '
                             f'{self._log_dir}/{child.name}.out', node=child.name)
                err('watch', '전체를 내린다 (ekf.launch.py 의 on_exit=Shutdown 과 같다)')
                self._stopping = True
                return

            if not spec.respawn:
                self._report_dead([child], '감시')
                continue

            due = self._pending.get(child.name)
            if due is None:
                self._pending[child.name] = now + spec.respawn_delay
                log('watch', f'rc={child.proc.returncode} · '
                             f'{spec.respawn_delay:.1f} 초 뒤 재시작', node=child.name)
                continue
            if now < due:
                continue
            self._pending.pop(child.name, None)
            self._respawn(child, spec)

    def _respawn(self, child, spec) -> None:
        """죽은 자식을 같은 자리에 새로 띄운다. 종료 순서를 흐트러뜨리지 않는다."""
        child.log.close()
        try:
            fresh = spawn(spec, self._log_dir, self._use_sim_time)
        except Exception as exc:  # noqa: BLE001
            err('watch', f'재시작 실패: {exc}', node=spec.name)
            return

        with self._lock:
            if self._stopping:
                kill(fresh)
                return
            # 자리를 그대로 지켜야 shutdown 의 역순 정리가 의미를 갖는다.
            self._children = [fresh if c is child else c for c in self._children]

        self._reported.discard(spec.name)
        log('respawn', f'pid={fresh.pid}', node=spec.name)

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
            self._stopping = True
            children, self._children = list(self._children), []
        if not children:
            return

        log('shutdown', '역순 정리 시작')
        # 한 자식에서 터져도 나머지를 반드시 정리한다. 예전에는 예외가 루프를
        # 통째로 끊어서 뒤쪽 자식이 전부 고아로 남았다 (실측 2026-08-24:
        # critical 발동 teardown 이 9 번째에서 멈춰 5 개가 ppid=1 로 남았다).
        failed = 0
        for child in reversed(children):
            try:
                code = kill(child)
            except Exception as exc:  # noqa: BLE001
                failed += 1
                err('kill', f'pid={child.pid} 정리 실패 · 계속한다: {exc}',
                    node=child.name)
                continue
            log('kill', f'pid={child.pid} rc={code}', node=child.name)
        tail = f' · 실패 {failed}' if failed else ''
        log('shutdown', f'완료{tail}')


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
        # node.stopping 은 감시가 critical 사망을 봤을 때 선다.
        while alive[0] and rclpy.ok() and not node.stopping:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        # 자식 정리가 먼저다. 여기서 예외가 나면 자식이 고아로 남는다.
        node.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
