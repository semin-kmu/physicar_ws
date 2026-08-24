"""터미널 로그. 접두어 `[단계][노드]` 고정. 이모지·장식 금지."""

import sys


def log(stage: str, message: str, node: str = '') -> None:
    """`[stage][node] message`. node 를 생략하면 대괄호 1개."""
    prefix = f'[{stage}]' + (f'[{node}]' if node else '')
    print(f'{prefix} {message}', flush=True)


def err(stage: str, message: str, node: str = '') -> None:
    """실패 전용. 형식은 동일하고 stderr 로 나간다."""
    prefix = f'[{stage}]' + (f'[{node}]' if node else '')
    print(f'{prefix} {message}', file=sys.stderr, flush=True)
