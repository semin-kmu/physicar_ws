"""config/*.yaml 로드 → core 자료구조 변환. docs/08"""

import os
import time

import yaml

from kau_state_machine.core.schema import Manifest, parse_manifest


def load_manifest(path: str) -> Manifest:
    with open(path, encoding='utf-8') as handle:
        return parse_manifest(yaml.safe_load(handle))


def make_run_id() -> str:
    return time.strftime('%Y%m%d_%H%M%S')


def resolve_log_dir(template: str, run_id: str) -> str:
    """`{run_id}` 치환 후 생성. docs/09 section 7"""
    path = os.path.expanduser(template.replace('{run_id}', run_id))
    os.makedirs(path, exist_ok=True)
    return path
