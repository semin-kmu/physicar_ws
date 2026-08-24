"""노드 spawn · kill. docs/01 section 9"""

import os
import signal
import subprocess
from dataclasses import dataclass

from ament_index_python.packages import (
    get_package_prefix,
    get_package_share_directory,
)

from kau_state_machine.core.schema import NodeSpec

# params·args 값에 붙이면 `pkg:경로` 를 절대경로로 바꾼다.
REF_PREFIX = '@'


@dataclass
class Child:
    """살아있는 자식 프로세스 1개."""

    name: str
    proc: subprocess.Popen
    log: object

    @property
    def pid(self) -> int:
        return self.proc.pid


def resolve_ref(ref: str) -> str:
    """`pkg:상대경로` → share 절대경로."""
    package, sep, rel = ref.partition(':')
    if not sep or not rel:
        raise ValueError(f'참조 형식은 "pkg:경로": {ref!r}')
    path = os.path.join(get_package_share_directory(package), rel)
    if not os.path.exists(path):
        raise FileNotFoundError(f'{ref} -> {path} 없음')
    return path


def resolve_exe(package: str, executable: str) -> str:
    """`lib/<pkg>/<exe>` 직접 실행. ros2 run/launch 를 쓰지 않는다 (01 section 9-1)."""
    path = os.path.join(get_package_prefix(package), 'lib', package, executable)
    if not os.access(path, os.X_OK):
        raise FileNotFoundError(f'{package}/{executable} -> {path} 실행 불가')
    return path


def _value(value) -> str:
    """CLI 문자열로 변환. `@pkg:경로` 는 절대경로로 치환."""
    if isinstance(value, bool):
        return 'true' if value else 'false'
    text = str(value)
    if text.startswith(REF_PREFIX):
        return resolve_ref(text[len(REF_PREFIX):])
    return text


def build_argv(spec: NodeSpec) -> list:
    """ROS 인자는 `--ros-args` 뒤에만 온다. ros=false 면 붙이지 않는다."""
    argv = [resolve_exe(spec.package, spec.executable)]
    argv += [_value(arg) for arg in spec.args]
    if not spec.ros:
        return argv

    argv += ['--ros-args', '-r', f'__node:={spec.name}']
    for ref in spec.param_files:
        argv += ['--params-file', resolve_ref(ref)]
    for key, value in spec.params.items():
        argv += ['-p', f'{key}:={_value(value)}']
    for src, dst in spec.remap.items():
        argv += ['-r', f'{src}:={dst}']
    return argv


def spawn(spec: NodeSpec, log_dir: str) -> Child:
    """별도 세션으로 띄운다. 종료 시 killpg 로 자손까지 정리하기 위함.

    PR_SET_PDEATHSIG(01 section 9-1)는 preexec_fn 이 필요한데 스레드에서
    fork 하면 안전하지 않다. 미구현 — supervisor 가 kill -9 로 죽으면 고아가
    남는다 (01 시나리오 5). 복구 단계에서 exec 래퍼로 처리한다.
    """
    argv = build_argv(spec)
    log = open(os.path.join(log_dir, f'{spec.name}.out'), 'ab')
    try:
        proc = subprocess.Popen(
            argv,
            start_new_session=True,
            stdout=log,
            stderr=subprocess.STDOUT,
            env=os.environ.copy(),
        )
    except BaseException:
        log.close()
        raise

    with open(os.path.join(log_dir, f'{spec.name}.pid'), 'w') as handle:
        handle.write(str(proc.pid))
    return Child(name=spec.name, proc=proc, log=log)


def kill(child: Child, grace: float = 3.0):
    """SIGTERM → grace 초 → SIGKILL. 행 걸린 노드가 종료를 막지 못하게 한다."""
    proc = child.proc
    if proc.poll() is None:
        try:
            pgid = os.getpgid(proc.pid)
            os.killpg(pgid, signal.SIGTERM)
            try:
                proc.wait(timeout=grace)
            except subprocess.TimeoutExpired:
                os.killpg(pgid, signal.SIGKILL)
                proc.wait()
        except ProcessLookupError:
            pass
    child.log.close()
    return proc.returncode
