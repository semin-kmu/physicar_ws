#!/usr/bin/env bash
# kau_gui 전용 venv 를 패키지 안에 만든다.
#
# 시스템 python 에는 아무것도 설치하지 않는다. venv 는 디렉터리 하나이고
# 통째로 지우면 흔적이 남지 않는다.
#
#   --system-site-packages 는 필수다. 이게 없으면 venv 안에서 rclpy 가
#   안 보여 ROS 구독 자체가 불가능하다. numpy / PyQt5 도 시스템 것을 쓴다.
#   venv 에 새로 들어가는 것은 pyqtgraph 하나뿐이다.
#
# 대회장에는 인터넷이 없다. 미리 받아 둔 wheel 로 설치하려면
#   ./setup_venv.sh /path/to/wheels
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG="$(dirname "$HERE")"
VENV="$PKG/.venv"
WHEELS="${1-}"

if [ -d "$VENV" ]; then
  echo "[kau_gui] venv 가 이미 있다: $VENV"
else
  echo "[kau_gui] venv 생성: $VENV"
  python3 -m venv --system-site-packages "$VENV"
fi

if [ -n "$WHEELS" ]; then
  echo "[kau_gui] 오프라인 설치: $WHEELS"
  "$VENV/bin/pip" install --no-index --find-links "$WHEELS" pyqtgraph
else
  "$VENV/bin/pip" install --upgrade pyqtgraph
fi

echo
"$VENV/bin/python" - <<'PY'
import pyqtgraph, numpy, sys
print(f"[kau_gui] pyqtgraph {pyqtgraph.__version__} / numpy {numpy.__version__}")
try:
    import rclpy                      # noqa: F401
    print("[kau_gui] rclpy 보임 (--system-site-packages 정상)")
except ImportError:
    print("[kau_gui] rclpy 안 보임. --system-site-packages 없이 만든 venv 다.")
    sys.exit(1)
PY

echo "[kau_gui] 완료. 그대로 ros2 run kau_gui kau_gui 로 실행하면 된다"
