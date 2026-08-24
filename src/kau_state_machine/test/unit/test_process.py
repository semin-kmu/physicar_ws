"""ros/process.py argv 조립 · spawn/kill."""

import os
import subprocess
import sys
import time

import pytest

from kau_state_machine.core.schema import NodeSpec
from kau_state_machine.ros import process

STUB = os.path.join(os.path.dirname(__file__), '..', 'stub', 'stub_node.py')

def _alive(pgid):
    """프로세스 그룹에 살아있는 프로세스. 좀비(Z)는 이미 죽은 것이라 뺀다.

    부모가 먼저 죽으면 손자는 PID 1 에 재부모화되는데, 컨테이너 PID 1 이
    수거하지 않으면 좀비로 남는다. docker run --init 로 피한다.
    """
    out = subprocess.run(['ps', '-o', 'pid=,stat=', '-g', str(pgid)],
                         capture_output=True, text=True).stdout
    return [line for line in out.split('\n')
            if line.strip() and not line.split()[1].startswith('Z')]



@pytest.fixture
def fake_resolve(monkeypatch):
    monkeypatch.setattr(process, 'resolve_exe', lambda p, e: os.path.abspath(STUB))
    monkeypatch.setattr(process, 'resolve_ref', lambda ref: f'/share/{ref}')


def test_argv_ros_node(fake_resolve):
    spec = NodeSpec(name='n', package='p', executable='e',
                    param_files=('p:config/a.yaml',),
                    params={'use_sim_time': False, 'x': 1},
                    remap={'a': '/b'})
    argv = process.build_argv(spec)
    assert argv[1:] == [
        '--ros-args', '-r', '__node:=n',
        '--params-file', '/share/p:config/a.yaml',
        '-p', 'use_sim_time:=false', '-p', 'x:=1',
        '-r', 'a:=/b',
    ]


def test_argv_plain_process(fake_resolve):
    spec = NodeSpec(name='n', package='p', executable='e', ros=False,
                    args=('pat', '@p:config/a.yaml'))
    assert process.build_argv(spec)[1:] == ['pat', '/share/p:config/a.yaml']


def test_argv_ref_in_params(fake_resolve):
    spec = NodeSpec(name='n', package='p', executable='e',
                    params={'f': '@p:maps/m.yaml'})
    assert process.build_argv(spec)[1:] == [
        '--ros-args', '-r', '__node:=n', '-p', 'f:=/share/p:maps/m.yaml']


def test_spawn_writes_log_and_pid(fake_resolve, tmp_path):
    spec = NodeSpec(name='stub', package='p', executable='e', ros=False,
                    args=('--print-argv',))
    child = process.spawn(spec, str(tmp_path))
    try:
        time.sleep(0.5)
        assert child.proc.poll() is None
        assert (tmp_path / 'stub.pid').read_text() == str(child.pid)
        assert '--print-argv' in (tmp_path / 'stub.out').read_text()
    finally:
        process.kill(child)


def test_kill_escalates_to_sigkill(fake_resolve, tmp_path):
    spec = NodeSpec(name='stub', package='p', executable='e', ros=False,
                    args=('--ignore-sigterm',))
    child = process.spawn(spec, str(tmp_path))
    time.sleep(0.5)
    started = time.monotonic()
    code = process.kill(child, grace=1.0)
    assert time.monotonic() - started >= 1.0
    assert code == -9


def test_kill_reaps_grandchild(fake_resolve, tmp_path):
    spec = NodeSpec(name='stub', package='p', executable='e', ros=False,
                    args=('--child',))
    child = process.spawn(spec, str(tmp_path))
    time.sleep(0.5)
    pgid = os.getpgid(child.pid)
    assert pgid != os.getpgid(0)          # 별도 세션
    process.kill(child)
    time.sleep(0.3)
    assert _alive(pgid) == []


def test_dead_detects_exited_child(fake_resolve, tmp_path):
    live = NodeSpec(name='live', package='p', executable='e', ros=False)
    gone = NodeSpec(name='gone', package='p', executable='e', ros=False,
                    args=('--exit-now', '3'))
    children = [process.spawn(live, str(tmp_path)), process.spawn(gone, str(tmp_path))]
    try:
        time.sleep(0.8)
        found = process.dead(children)
        assert [c.name for c in found] == ['gone']
        assert found[0].proc.returncode == 3
    finally:
        for child in children:
            process.kill(child)


def test_use_sim_time_injected(fake_resolve):
    spec = NodeSpec(name='n', package='p', executable='e')
    assert process.build_argv(spec, True)[1:] == [
        '--ros-args', '-r', '__node:=n', '-p', 'use_sim_time:=true']
    assert process.build_argv(spec, False)[1:] == [
        '--ros-args', '-r', '__node:=n', '-p', 'use_sim_time:=false']
    assert process.build_argv(spec)[1:] == ['--ros-args', '-r', '__node:=n']


def test_node_params_override_use_sim_time(fake_resolve):
    """노드가 직접 적으면 그쪽이 이긴다. -p 는 1개만 나가야 한다."""
    spec = NodeSpec(name='n', package='p', executable='e',
                    params={'use_sim_time': False})
    argv = process.build_argv(spec, True)
    assert argv.count('use_sim_time:=false') == 1
    assert 'use_sim_time:=true' not in argv


def test_use_sim_time_not_injected_for_plain_process(fake_resolve):
    spec = NodeSpec(name='n', package='p', executable='e', ros=False, args=('x',))
    assert process.build_argv(spec, True)[1:] == ['x']


def _gate_child(tmp_path, code, sleep_sec=0.0):
    """즉시 끝나는 관문 프로세스 하나. 종료 코드를 지정한다."""
    script = f'import time,sys; time.sleep({sleep_sec}); sys.exit({code})'
    log = open(tmp_path / 'gate.out', 'ab')
    proc = subprocess.Popen([sys.executable, '-c', script],
                            start_new_session=True, stdout=log,
                            stderr=subprocess.STDOUT)
    return process.Child(name='gate', proc=proc, log=log)


def test_wait_gate_passes(tmp_path):
    assert process.wait_gate(_gate_child(tmp_path, 0), 10.0) == 0


def test_wait_gate_reports_failure(tmp_path):
    assert process.wait_gate(_gate_child(tmp_path, 1), 10.0) == 1


def test_wait_gate_kills_on_timeout(tmp_path):
    child = _gate_child(tmp_path, 0, sleep_sec=30.0)
    pgid = os.getpgid(child.pid)
    assert process.wait_gate(child, 0.5) is None
    time.sleep(0.3)
    assert not _alive(pgid)
