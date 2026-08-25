"""bringup.yaml -> 감시 대상 노드 이름 목록.

    source run.sh
     -> ros2 launch kau_state_machine state_machine.launch.py
     -> system_supervisor
     -> bringup.yaml 의 노드를 전부 spawn

그래서 "run.sh 가 띄우는 노드" 의 정의는 bringup.yaml 하나뿐이다. 목록을
GUI 쪽에 따로 적어 두면 manifest 가 바뀔 때마다 둘이 어긋난다.

supervisor 는 ros=true 노드를 `-r __node:=<name>` 으로 띄우므로
manifest 의 name 이 곧 ROS 그래프 이름이다 (process.py build_argv).

제외 대상
    wait: true   관문. 조건이 서면 스스로 끝나는 게 정상이다 (clock_gate).
                 상시 감시 대상이 아니고, 끝났다고 빨간불을 켜면 오보다.
    ros: false   --ros-args 없이 뜬다. 그래프에 이름이 없어 판정 자체가 불가
                 (platform_ekf_pause). 회색으로 두면 "미기동" 과 구분이 안 된다.

필터
    when    always | sim | real. use_sim_time 으로 가른다.
    option  supervisor 의 `option.<이름>` 파라미터. 관측 전용인 GUI 는 그
            값을 물어볼 수 없으므로 (service client 금지, docs/09 section 2)
            gui.yaml 의 status.options 에 run.sh 설정과 맞춰 적어 둔다.

스키마를 kau_state_machine 에서 import 하지 않고 여기서 직접 읽는다.
쓰는 키가 다섯 개뿐이라 결합을 늘릴 값어치가 없다.
"""

from __future__ import annotations

import pathlib

import yaml
from ament_index_python.packages import (PackageNotFoundError,
                                         get_package_share_directory)

PACKAGE = "kau_state_machine"


def default_path() -> str:
    """kau_state_machine 이 share 에 설치하는 bringup.yaml. 못 찾으면 빈 문자열."""
    try:
        share = get_package_share_directory(PACKAGE)
    except PackageNotFoundError:
        return ""
    path = pathlib.Path(share) / "config" / "bringup.yaml"
    return str(path) if path.is_file() else ""


def load(path: str) -> dict:
    """bringup.yaml 을 읽는다. 실패하면 예외를 던진다."""
    return yaml.safe_load(pathlib.Path(path).read_text()) or {}


def _specs(data: dict):
    for stage in data.get("stages") or ():
        for spec in stage.get("nodes") or ():
            yield spec


def option_names(data: dict) -> list:
    """manifest 가 쓰는 option 스위치 이름들. GUI 가 이만큼 파라미터를 연다."""
    return sorted({str(s["option"]) for s in _specs(data) if s.get("option")})


def watch_names(data: dict, use_sim_time: bool, options: dict) -> list:
    """감시할 노드 이름을 기동 순서 그대로."""
    out = []
    for spec in _specs(data):
        if spec.get("wait", False) or not spec.get("ros", True):
            continue

        when = spec.get("when", "always")
        if when == "sim" and not use_sim_time:
            continue
        if when == "real" and use_sim_time:
            continue

        option = spec.get("option", "")
        if option and not options.get(option, True):
            continue

        out.append(str(spec["name"]))
    return out
