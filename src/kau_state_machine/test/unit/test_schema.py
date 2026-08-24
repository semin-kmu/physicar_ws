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
