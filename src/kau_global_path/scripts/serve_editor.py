#!/usr/bin/env python3
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

"""lane_editor.html 를 띄우는 정적 서버. ROS 가 필요 없다.

    python3 src/kau_global_path/scripts/serve_editor.py

띄운 뒤 브라우저에서 열 주소:

    http://localhost:8001/src/kau_global_path/web/lane_editor.html

편집기는 지도(../../kau_localization/maps/)와 궤적(../data/)을 상대경로로
fetch 하므로 **워크스페이스 루트**가 문서 루트여야 한다. 이 스크립트는 자기
위치에서 루트를 역산하므로 어느 디렉터리에서 실행해도 된다.

왜 `python3 -m http.server` 를 안 쓰는가
    그쪽은 캐시를 허용한다. lane_editor.html 은 `<script src="lane_editor.js">`
    로 버전 없이 부르기 때문에, 편집기를 고치는 중에 HTML 만 새로 받고 JS 는
    캐시에서 꺼내 쓰는 일이 생긴다. 사이드바에는 새 버튼이 보이는데 동작은
    옛것인 상태가 되어 원인을 찾기 어렵다. 그래서 no-store 를 붙여 내보낸다.

기본 포트가 8001 인 이유
    8000 은 physicar_webserver 노드가 쓴다.

기본 bind 가 0.0.0.0 인 이유
    이 워크스페이스는 도커 컨테이너 안에서 돈다. 컨테이너에는 브라우저가 없어서
    편집기는 호스트 브라우저로 연다. 127.0.0.1 에 묶으면 컨테이너 루프백에만
    붙어 호스트에서 접속이 안 된다. 컨테이너 밖으로 내보내지 않으려면
    `--bind 127.0.0.1` 을 명시할 것.
"""

import argparse
import functools
import http.server
import socket
from pathlib import Path

# scripts/ -> kau_global_path/ -> src/ -> 워크스페이스 루트
WS_ROOT = Path(__file__).resolve().parents[3]
EDITOR = 'src/kau_global_path/web/lane_editor.html'


def outside_ip():
    """컨테이너/LAN 바깥에서 붙을 때 쓸 주소. 실제로 패킷을 보내지는 않는다."""
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        try:
            s.connect(('10.255.255.255', 1))
            return s.getsockname()[0]
        except OSError:
            return None


class NoCacheHandler(http.server.SimpleHTTPRequestHandler):
    def end_headers(self):
        self.send_header('Cache-Control', 'no-store, must-revalidate')
        self.send_header('Pragma', 'no-cache')
        self.send_header('Expires', '0')
        super().end_headers()

    def log_message(self, fmt, *args):
        # 404 만 남긴다. 편집기가 바퀴 파일을 번호 순서로 훑기 때문에
        # (inner_1, inner_2, ...) 200 까지 찍으면 로그가 도배된다.
        if args and str(args[1]) != '200':
            super().log_message(fmt, *args)


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('-p', '--port', type=int, default=8001)
    p.add_argument('--bind', default='0.0.0.0',
                   help='이 기기 안에서만 열려면 127.0.0.1')
    p.add_argument('--root', default=str(WS_ROOT), help='문서 루트')
    args = p.parse_args()

    root = Path(args.root).expanduser().resolve()
    if not (root / EDITOR).is_file():
        p.error(f'{root} 아래에 {EDITOR} 가 없다. --root 로 워크스페이스 루트를 지정할 것')

    handler = functools.partial(NoCacheHandler, directory=str(root))
    httpd = http.server.ThreadingHTTPServer((args.bind, args.port), handler)

    host = 'localhost' if args.bind in ('127.0.0.1', '0.0.0.0') else args.bind
    print(f'문서 루트 {root}')
    print(f'편집기   http://{host}:{args.port}/{EDITOR}')
    if args.bind == '0.0.0.0':
        ip = outside_ip()
        if ip:
            print(f'         http://{ip}:{args.port}/{EDITOR}  (도커 호스트 / 다른 PC)')
    print('Ctrl+C 로 종료\n')

    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print('\n종료')


if __name__ == '__main__':
    main()
