#!/usr/bin/env bash
#
# 가상 디스플레이(:1) 화면을 터미널에 그대로 출력한다.
# 이 컨테이너는 모니터가 없고 Xvfb :1 위에서만 GUI 가 돌기 때문에,
# 브라우저로 noVNC(:6080) 에 붙지 않으면 RViz 를 볼 방법이 없다.
# 이 스크립트는 그 화면을 캡처해 ANSI 블록 문자로 터미널에 뿌린다.
#
#   ./screen.sh              현재 화면을 터미널에 출력
#   ./screen.sh out.png      PNG 파일로만 저장
#
set -eu
export DISPLAY="${DISPLAY:-:1}"

PNG="${1:-$(mktemp /tmp/screen_XXXX.png)}"
scrot -o "$PNG"

if [ $# -gt 0 ]; then
  echo "saved: $PNG"
  exit 0
fi

python3 - "$PNG" <<'PY'
import sys, shutil
from PIL import Image

img = Image.open(sys.argv[1]).convert("RGB")
cols = max(20, shutil.get_terminal_size((100, 30)).columns - 1)
# 문자 한 칸에 세로 2픽셀(▀ 상단=전경, 하단=배경)
w = min(cols, img.width)
h = round(img.height * w / img.width / 2) * 2
img = img.resize((w, h), Image.LANCZOS)
px = img.load()

out = []
for y in range(0, h - 1, 2):
    line = []
    for x in range(w):
        r1, g1, b1 = px[x, y]
        r2, g2, b2 = px[x, y + 1]
        line.append(f"\033[38;2;{r1};{g1};{b1}m\033[48;2;{r2};{g2};{b2}m▀")
    out.append("".join(line) + "\033[0m")
print("\n".join(out))
PY
