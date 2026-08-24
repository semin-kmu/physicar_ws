"""YAML 스키마 정의 · 기동 전 검증 14종. docs/08 section 7"""

from dataclasses import dataclass, field

# 현 구현 범위는 순차 spawn + 관문 프로세스(wait)뿐이다. 재시도·evidence 키는
# 두지 않는다.
_ROOT_KEYS = {'run', 'stages'}
_RUN_KEYS = {'log_dir'}
_STAGE_KEYS = {'id', 'delay_sec', 'nodes'}
_NODE_KEYS = {'name', 'package', 'executable', 'ros', 'wait', 'when',
              'param_files', 'params', 'remap', 'args'}

# when: 이 노드를 언제 띄우는가. 시계 소스로만 가른다 (docs/01 section 10).
_WHEN = ('always', 'sim', 'real')


@dataclass(frozen=True)
class NodeSpec:
    """노드 1개. 파라미터 **값**이 아니라 위치(`pkg:경로`)만 갖는다."""

    name: str
    package: str
    executable: str
    ros: bool = True
    # 관문. 이 프로세스가 끝날 때까지 다음 노드를 띄우지 않는다. 종료 코드가
    # 0 이 아니면 기동을 중단한다 (clock_gate.py 처럼 조건을 재는 프로세스용).
    wait: bool = False
    when: str = 'always'
    param_files: tuple = ()
    params: dict = field(default_factory=dict)
    remap: dict = field(default_factory=dict)
    args: tuple = ()


@dataclass(frozen=True)
class Stage:
    """단계 안은 동시 spawn, 단계 사이는 `delay_sec` 대기. docs/01 section 4-1"""

    id: int
    nodes: tuple
    delay_sec: float = 0.0


@dataclass(frozen=True)
class Manifest:
    log_dir: str
    stages: tuple

    @property
    def node_count(self) -> int:
        return sum(len(stage.nodes) for stage in self.stages)


def _mapping(value, where):
    if not isinstance(value, dict):
        raise ValueError(f'{where}: 매핑이어야 한다')
    return value


def _reject_unknown(data, allowed, where):
    unknown = sorted(set(data) - allowed)
    if unknown:
        raise ValueError(f'{where}: 모르는 키 {unknown}')


def _node(data, where):
    _mapping(data, where)
    _reject_unknown(data, _NODE_KEYS, where)
    for key in ('name', 'package', 'executable'):
        if not data.get(key):
            raise ValueError(f'{where}: {key} 필수')

    when = str(data.get('when', 'always'))
    if when not in _WHEN:
        raise ValueError(f'{where}: when 은 {list(_WHEN)} 중 하나 (받은 값: {when!r})')

    return NodeSpec(
        name=str(data['name']),
        package=str(data['package']),
        executable=str(data['executable']),
        ros=bool(data.get('ros', True)),
        wait=bool(data.get('wait', False)),
        when=when,
        param_files=tuple(str(ref) for ref in data.get('param_files') or ()),
        params=dict(_mapping(data.get('params') or {}, f'{where}.params')),
        remap=dict(_mapping(data.get('remap') or {}, f'{where}.remap')),
        args=tuple(data.get('args') or ()),
    )


def _stage(data, where, seen):
    _mapping(data, where)
    _reject_unknown(data, _STAGE_KEYS, where)
    if 'id' not in data:
        raise ValueError(f'{where}: id 필수')

    nodes = []
    for index, raw in enumerate(data.get('nodes') or ()):
        spec = _node(raw, f'{where}.nodes[{index}]')
        if spec.name in seen:
            raise ValueError(f'{where}.nodes[{index}]: 노드명 중복 {spec.name}')
        seen.add(spec.name)
        nodes.append(spec)
    if not nodes:
        raise ValueError(f'{where}: nodes 가 비었다')

    return Stage(id=int(data['id']), nodes=tuple(nodes),
                 delay_sec=float(data.get('delay_sec', 0.0)))


def parse_manifest(data) -> Manifest:
    """dict → Manifest. 모르는 키는 오탈자로 보고 거부한다."""
    _mapping(data, 'manifest')
    _reject_unknown(data, _ROOT_KEYS, 'manifest')

    run = _mapping(data.get('run') or {}, 'run')
    _reject_unknown(run, _RUN_KEYS, 'run')
    if not run.get('log_dir'):
        raise ValueError('run.log_dir 필수')

    seen = set()
    stages = tuple(_stage(raw, f'stages[{index}]', seen)
                   for index, raw in enumerate(data.get('stages') or ()))
    if not stages:
        raise ValueError('stages 가 비었다')

    return Manifest(log_dir=str(run['log_dir']), stages=stages)
