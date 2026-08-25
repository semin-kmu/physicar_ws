# Copyright 2026 KAU AMET Team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""지도 인자 해석. launch 파일이 아니라 launch 파일들이 같이 쓰는 헬퍼다.

지도는 항상 같은 폴더에 있으므로 전체 경로 대신 이름만 받는다.

    pbstream:=kau_v3.pbstream     이름
    pbstream:=kau_v3              확장자 생략
    pbstream:=latest              가장 번호가 큰 kau_vN
    pbstream:=/어디/다른.pbstream   경로를 주면 그대로 쓴다

기본 폴더는 설치된 share/kau_localization/maps 인데, 이 폴더 **자체가**
src/kau_localization/maps 를 가리키는 심링크다 (CMakeLists.txt 참고).
그래서 save_map.py 로 새로 저장한 지도가 저장 즉시 이름으로 불린다 --
colcon build 를 다시 할 필요가 없다.

다른 폴더를 뒤지고 싶으면 maps_dir:= 로 지정한다.
"""

import re
from pathlib import Path


VERSION_PAT = re.compile(r'^kau_v(\d+)$')


def latest_map(maps_dir: Path, suffix: str):
    """maps_dir 안에서 번호가 가장 큰 kau_vN<suffix>. 없으면 None."""
    best = None
    for path in maps_dir.glob(f'kau_v*{suffix}'):
        m = VERSION_PAT.match(path.stem)
        if m and (best is None or int(m.group(1)) > best[0]):
            best = (int(m.group(1)), path)
    return best[1] if best else None


def resolve_map(value: str, maps_dir: Path, suffix: str, arg_name: str) -> str:
    """지도 인자 -> 실제 파일 경로. 못 찾으면 RuntimeError.

    value    사용자가 준 값 (이름 / 확장자 없는 이름 / 'latest' / 경로)
    maps_dir 이름만 준 경우에 뒤질 폴더
    suffix   '.pbstream' 또는 '.yaml'
    arg_name 에러 메시지에 쓸 launch 인자 이름
    """
    value = value.strip()
    if not value:
        raise RuntimeError(f'{arg_name} 를 줘야 한다. {_hint(maps_dir, suffix, arg_name)}')

    if value == 'latest':
        found = latest_map(maps_dir, suffix)
        if found is None:
            raise RuntimeError(f'{maps_dir} 에 kau_vN{suffix} 이 없다.')
        return str(found.resolve())

    # 경로처럼 생겼으면 (구분자가 있거나 실제로 있으면) 그대로 쓴다.
    candidate = Path(value).expanduser()
    if '/' in value or candidate.is_file():
        if not candidate.is_file():
            raise RuntimeError(f'{arg_name} 파일이 없다: {candidate}')
        return str(candidate.resolve())

    # 이름만 준 경우. 확장자는 없으면 붙여준다.
    name = value if value.endswith(suffix) else value + suffix
    found = maps_dir / name
    if not found.is_file():
        raise RuntimeError(
            f'{maps_dir} 에 {name} 이 없다. {_hint(maps_dir, suffix, arg_name)}')
    return str(found.resolve())


def _hint(maps_dir: Path, suffix: str, arg_name: str) -> str:
    names = sorted(p.stem for p in maps_dir.glob(f'kau_v*{suffix}'))
    if names:
        return f'{arg_name}:= 로 고를 수 있는 것: {", ".join(names)}, latest'
    return (f'{maps_dir} 가 비어 있다. save_map.py 로 지도를 만들면 '
            '저장 즉시 여기 보인다 (이 폴더는 소스 폴더를 가리키는 심링크다). '
            '안 보이면 maps_dir:= 로 소스 폴더를 직접 가리킬 것.')
