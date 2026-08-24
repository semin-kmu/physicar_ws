"""venv 의 site-packages 를 sys.path 에 얹는다.

venv 를 activate 하지 않고 경로만 추가하는 이유:
activate 하면 python 실행 파일 자체가 바뀌어 ros2 run 의 실행 경로 · 환경변수
순서와 얽힌다. pyqtgraph 는 순수 파이썬이고 의존(numpy · PyQt5)은 전부 시스템에
있으므로, 경로 하나만 더 보이면 그대로 import 된다.

찾는 순서
    1. 이미 import 가능하면 아무것도 하지 않는다 (시스템에 깔린 경우)
    2. 환경변수 KAU_GUI_VENV
    3. 이 파일에서 위로 올라가며 .venv 탐색 (colcon --symlink-install 기준
       설치본이 소스를 가리키므로 대개 여기서 찾힌다)
"""

from __future__ import annotations

import importlib.util
import os
import sys
from pathlib import Path


def _site_packages(venv: Path):
    hits = sorted(venv.glob("lib/python*/site-packages"))
    return hits[0] if hits else None


def ensure(package: str = "pyqtgraph") -> str | None:
    """`package` 가 import 가능해지도록 sys.path 를 손본다.

    반환값은 추가한 경로. 손댈 필요가 없었으면 None.
    실패 시 RuntimeError 를 던진다. 조용히 넘어가면 나중에 ImportError 가
    엉뚱한 곳에서 터져 원인을 못 찾는다.
    """
    if importlib.util.find_spec(package) is not None:
        return None

    candidates = []

    env = os.environ.get("KAU_GUI_VENV")
    if env:
        candidates.append(Path(env))

    here = Path(__file__).resolve()
    for parent in here.parents:
        candidates.append(parent / ".venv")

    for venv in candidates:
        sp = _site_packages(venv) if venv.is_dir() else None
        if sp is None:
            continue
        sys.path.insert(0, str(sp))
        if importlib.util.find_spec(package) is not None:
            return str(sp)
        sys.path.pop(0)

    raise RuntimeError(
        f"{package} 를 찾을 수 없다.\n"
        f"  src/kau_gui/scripts/setup_venv.sh 를 실행하거나\n"
        f"  KAU_GUI_VENV 에 venv 경로를 지정할 것")
