"""core/schema.py 파싱·검증."""

import pytest

from kau_state_machine.core.schema import parse_manifest

MINIMAL = {
    'run': {'log_dir': '/tmp/kau/{run_id}'},
    'stages': [{'id': 1, 'nodes': [
        {'name': 'a', 'package': 'p', 'executable': 'e'}]}],
}


def test_minimal():
    manifest = parse_manifest(MINIMAL)
    assert manifest.node_count == 1
    assert manifest.stages[0].nodes[0].ros is True
    assert manifest.stages[0].delay_sec == 0.0


def test_full_node():
    data = {'run': {'log_dir': '/tmp'}, 'stages': [{'id': 1, 'delay_sec': 2.5, 'nodes': [{
        'name': 'a', 'package': 'p', 'executable': 'e', 'ros': False,
        'param_files': ['p:config/x.yaml'], 'params': {'k': 1},
        'remap': {'x': 'y'}, 'args': ['--flag']}]}]}
    spec = parse_manifest(data).stages[0].nodes[0]
    assert spec.ros is False
    assert spec.param_files == ('p:config/x.yaml',)
    assert spec.args == ('--flag',)


@pytest.mark.parametrize('data,msg', [
    ({'run': {'log_dir': '/tmp'}, 'stages': []}, 'stages 가 비었다'),
    ({'stages': [{'id': 1, 'nodes': [{'name': 'a', 'package': 'p', 'executable': 'e'}]}]},
     'run.log_dir 필수'),
    ({'run': {'log_dir': '/tmp'}, 'stages': [{'id': 1, 'nodes': []}]}, 'nodes 가 비었다'),
])
def test_reject(data, msg):
    with pytest.raises(ValueError, match=msg):
        parse_manifest(data)


def test_reject_unknown_key():
    data = {'run': {'log_dir': '/tmp'}, 'stages': [{'id': 1, 'nodes': [
        {'name': 'a', 'package': 'p', 'executable': 'e', 'lifecycle': True}]}]}
    with pytest.raises(ValueError, match='모르는 키'):
        parse_manifest(data)


def test_reject_duplicate_name():
    node = {'name': 'a', 'package': 'p', 'executable': 'e'}
    data = {'run': {'log_dir': '/tmp'},
            'stages': [{'id': 1, 'nodes': [node]}, {'id': 2, 'nodes': [node]}]}
    with pytest.raises(ValueError, match='노드명 중복'):
        parse_manifest(data)


def test_reject_missing_field():
    data = {'run': {'log_dir': '/tmp'},
            'stages': [{'id': 1, 'nodes': [{'name': 'a', 'package': 'p'}]}]}
    with pytest.raises(ValueError, match='executable 필수'):
        parse_manifest(data)


def test_gate_fields():
    data = {'run': {'log_dir': '/tmp'}, 'stages': [{'id': 0, 'nodes': [{
        'name': 'g', 'package': 'p', 'executable': 'g.py',
        'ros': False, 'wait': True, 'when': 'sim', 'args': [180.0]}]}]}
    spec = parse_manifest(data).stages[0].nodes[0]
    assert spec.wait is True
    assert spec.when == 'sim'
    assert spec.args == (180.0,)


def test_gate_defaults():
    spec = parse_manifest(MINIMAL).stages[0].nodes[0]
    assert spec.wait is False
    assert spec.when == 'always'


def test_reject_unknown_when():
    data = {'run': {'log_dir': '/tmp'}, 'stages': [{'id': 1, 'nodes': [
        {'name': 'a', 'package': 'p', 'executable': 'e', 'when': 'gazebo'}]}]}
    with pytest.raises(ValueError, match='when 은'):
        parse_manifest(data)



def _spec(**extra):
    """MINIMAL 노드에 키를 얹어 NodeSpec 하나를 얻는다."""
    node = {'name': 'a', 'package': 'p', 'executable': 'e', **extra}
    data = {'run': {'log_dir': '/tmp'}, 'stages': [{'id': 1, 'nodes': [node]}]}
    return parse_manifest(data).stages[0].nodes[0]


def test_supervision_fields():
    """respawn·critical 은 launch 의 respawn / on_exit=Shutdown 을 옮긴 것이다."""
    spec = _spec(respawn=True, respawn_delay=5.0, critical=True)
    assert spec.respawn is True
    assert spec.respawn_delay == 5.0
    assert spec.critical is True


def test_supervision_defaults():
    """안 적으면 감독하지 않는다. 기본 대기는 ekf.launch.py 와 같은 2 초."""
    spec = parse_manifest(MINIMAL).stages[0].nodes[0]
    assert spec.respawn is False
    assert spec.critical is False
    assert spec.respawn_delay == 2.0


def test_reject_gate_with_respawn():
    """관문은 끝나는 게 정상이라 되살리면 무한 반복이 된다."""
    with pytest.raises(ValueError, match='관문'):
        _spec(wait=True, respawn=True)


def test_reject_gate_with_critical():
    """관문의 정상 종료를 사망으로 보면 매번 전체가 내려간다."""
    with pytest.raises(ValueError, match='관문'):
        _spec(wait=True, critical=True)


def test_reject_negative_respawn_delay():
    with pytest.raises(ValueError, match='respawn_delay'):
        _spec(respawn=True, respawn_delay=-1.0)
