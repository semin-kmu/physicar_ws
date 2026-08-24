"""system_supervisor 의 감시·종료. 노드를 띄우지 않고 메서드만 부른다.

SystemSupervisor.__init__ 은 rclpy 컨텍스트를 요구하지만, 여기서 검사하는
_watch / shutdown 은 Node 의 것을 하나도 안 쓴다. 그래서 필요한 속성만 채운
대역 객체에 **언바운드 메서드**를 걸어 부른다. 노드 기동 없이 논리만 본다.
"""

import inspect
import threading
import types

import pytest

from kau_state_machine.core.schema import NodeSpec
from kau_state_machine.nodes import system_supervisor as sup


class FakeProc:
    def __init__(self, returncode=0, alive=False):
        self.pid = 1234
        self.returncode = returncode
        self._alive = alive

    def poll(self):
        return None if self._alive else self.returncode


class FakeChild:
    def __init__(self, name, alive=False, returncode=0):
        self.name = name
        self.proc = FakeProc(returncode, alive)
        self.log = types.SimpleNamespace(close=lambda: None)

    @property
    def pid(self):
        return self.proc.pid


def _fake(children=(), specs=(), ready=True):
    """감시·종료가 실제로 쓰는 속성만 채운 대역.

    _report_dead 는 진짜를 그대로 묶어 준다. 그것도 Node 의 것을 안 쓰고
    _reported / _log_dir 만 보므로 대역에서 그대로 돈다.
    """
    node = types.SimpleNamespace(
        _children=list(children),
        _specs={s.name: s for s in specs},
        _pending={},
        _reported=set(),
        _ready=ready,
        _stopping=False,
        _lock=threading.Lock(),
        _log_dir='/tmp/kau-test',
        _use_sim_time=True,
    )
    for name in ('_report_dead', '_respawn'):
        setattr(node, name, _bind(node, name))
    return node


def _bind(node, name):
    """언바운드 메서드를 대역에 묶는다. 셋 다 Node 의 것을 안 쓴다."""
    func = getattr(sup.SystemSupervisor, name)
    return lambda *a, **k: func(node, *a, **k)


# ---------------------------------------------------------------- 종료

def test_shutdown_continues_after_failure(monkeypatch):
    """한 자식이 터져도 나머지를 반드시 정리한다.

    2026-08-24 회귀: 예외가 루프를 끊어 뒤쪽 자식 5 개가 ppid=1 로 남았다.
    """
    killed = []

    def kill(child, *_a, **_k):
        if child.name == 'bad':
            raise OSError('killpg 실패')
        killed.append(child.name)
        return 0

    monkeypatch.setattr(sup, 'kill', kill)
    node = _fake([FakeChild('a'), FakeChild('bad'), FakeChild('c')])

    sup.SystemSupervisor.shutdown(node)

    # 역순이라 c 가 먼저다. bad 에서 터져도 a 까지 간다.
    assert killed == ['c', 'a']
    assert node._children == []


def test_shutdown_is_reverse_order(monkeypatch):
    killed = []
    monkeypatch.setattr(sup, 'kill', lambda c, *a, **k: (killed.append(c.name), 0)[1])
    node = _fake([FakeChild(n) for n in ('first', 'second', 'third')])

    sup.SystemSupervisor.shutdown(node)

    assert killed == ['third', 'second', 'first']


def test_shutdown_marks_stopping(monkeypatch):
    """종료 중 플래그가 서야 감시·기동 스레드가 새 자식을 안 띄운다."""
    monkeypatch.setattr(sup, 'kill', lambda c, *a, **k: 0)
    node = _fake([FakeChild('a')])

    sup.SystemSupervisor.shutdown(node)

    assert node._stopping is True


# ---------------------------------------------------------------- 감시

def test_watch_is_not_a_ros_timer():
    """감시를 ROS 타이머로 만들면 sim 시계에 묶인다.

    rclpy 의 create_timer 는 clock 을 안 주면 노드 시계를 쓴다. 이 노드는
    use_sim_time 이라 그게 sim 시계이고, sim 시계가 멈추는 것이 바로 감시가
    잡으려는 고장이다 (2026-08-24 실측: 맵 리로드로 감시가 같이 멈췄다).
    """
    src = inspect.getsource(sup.SystemSupervisor.__init__)
    assert 'create_timer' not in src
    assert '_watch_loop' in src
    assert 'time.monotonic' in inspect.getsource(sup.SystemSupervisor._watch)


def test_watch_respawns_after_delay(monkeypatch):
    """죽으면 바로가 아니라 respawn_delay 만큼 기다렸다 되살린다."""
    spec = NodeSpec(name='ekf', package='p', executable='e',
                    respawn=True, respawn_delay=2.0)
    dead_child = FakeChild('ekf', alive=False)
    fresh = FakeChild('ekf', alive=True)
    node = _fake([dead_child], [spec])

    monkeypatch.setattr(sup, 'spawn', lambda *a, **k: fresh)
    clock = [100.0]
    monkeypatch.setattr(sup.time, 'monotonic', lambda: clock[0])

    sup.SystemSupervisor._watch(node)                  # 사망 감지 · 예약만
    assert node._children == [dead_child]
    assert node._pending['ekf'] == 102.0

    clock[0] = 101.0
    sup.SystemSupervisor._watch(node)                  # 아직 이르다
    assert node._children == [dead_child]

    clock[0] = 102.5
    sup.SystemSupervisor._watch(node)                  # 이제 되살린다
    assert node._children == [fresh]
    assert 'ekf' not in node._pending


def test_watch_keeps_position_on_respawn(monkeypatch):
    """되살린 자식은 원래 자리에 들어가야 역순 종료가 뜻을 갖는다."""
    spec = NodeSpec(name='b', package='p', executable='e',
                    respawn=True, respawn_delay=0.0)
    a, b, c = FakeChild('a', alive=True), FakeChild('b'), FakeChild('c', alive=True)
    node = _fake([a, b, c], [spec])
    fresh = FakeChild('b', alive=True)

    monkeypatch.setattr(sup, 'spawn', lambda *a_, **k: fresh)
    monkeypatch.setattr(sup.time, 'monotonic', lambda: 0.0)

    sup.SystemSupervisor._watch(node)                  # 예약 (delay 0)
    sup.SystemSupervisor._watch(node)                  # 재시작

    assert [ch.name for ch in node._children] == ['a', 'b', 'c']
    assert node._children[1] is fresh


def test_watch_critical_stops_everything(monkeypatch):
    """critical 이 죽으면 기동 전체를 내린다 (launch 의 on_exit=Shutdown)."""
    spec = NodeSpec(name='guard', package='p', executable='e', critical=True)
    node = _fake([FakeChild('guard', returncode=1)], [spec])
    monkeypatch.setattr(sup.time, 'monotonic', lambda: 0.0)

    sup.SystemSupervisor._watch(node)

    assert node._stopping is True


def test_watch_silent_while_bringing_up(monkeypatch):
    """기동 중에는 돌지 않는다. 아직 안 뜬 노드를 죽었다고 볼 수 없다."""
    spec = NodeSpec(name='x', package='p', executable='e', respawn=True)
    node = _fake([FakeChild('x')], [spec], ready=False)
    monkeypatch.setattr(sup, 'spawn', lambda *a, **k: pytest.fail('띄우면 안 된다'))

    sup.SystemSupervisor._watch(node)

    assert node._pending == {}


def test_watch_plain_child_reported_once(monkeypatch):
    """respawn·critical 이 아니면 한 번만 알리고 조용히 둔다."""
    spec = NodeSpec(name='x', package='p', executable='e')
    node = _fake([FakeChild('x', returncode=2)], [spec])
    monkeypatch.setattr(sup.time, 'monotonic', lambda: 0.0)

    sup.SystemSupervisor._watch(node)
    sup.SystemSupervisor._watch(node)

    assert node._reported == {'x'}
    assert node._stopping is False
