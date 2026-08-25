#!/usr/bin/env bash
#
# 시뮬레이터 라이다 최대거리 조절.
#
#   ./sim_lidar_range.sh show          현재 값
#   ./sim_lidar_range.sh set 10        10 m 로
#   ./sim_lidar_range.sh restore       공장 초기값(16 m)으로
#
# ── 왜 이 스크립트가 있나 ────────────────────────────────────────────
# sim 라이다가 **16 m** 로 돼 있는데 실차 RPlidar C1 스펙 상한은 12 m 다.
# 즉 시뮬이 실차보다 좋은 센서를 쓰고 있었다. 대회장 실측으로는 그보다도
# 나빠서(먼 공간 인지 약함) 10 m 로 낮춰 재현한다.
#
# 이 값은 laser_odom(ICP) 과 AMCL **양쪽**에 다 걸린다. /scan 자체가
# 짧아지기 때문이다. AMCL 만 제한하고 싶으면 이게 아니라
# config/amcl.yaml 의 laser_max_range 를 고칠 것.
#
# ── 주의: /opt/physicar 를 고친다 ────────────────────────────────────
# 이 프로젝트는 원칙적으로 플랫폼 소스를 안 건드린다 (ekf.launch.py 주석
# 참고 -- updater.sh 가 덮고, 팀원 환경과 갈라진다). 여기는 예외다.
# gz 센서 스펙은 런타임 서비스로 못 바꾸고 model.sdf 에만 있기 때문이다.
#
# 그래서 원본을 저장소 안에 백업해 둔다:
#   src/kau_localization/sim_overrides/model.sdf.orig
#
# updater.sh 를 돌린 뒤에는 이 스크립트를 다시 실행해야 한다.
# `show` 로 언제든 현재 상태를 확인할 것.
#
# ── 언제 반영되나 ────────────────────────────────────────────────────
# gz 는 모델을 **스폰할 때** SDF 를 읽는다. 이미 떠 있는 차에는 즉시
# 반영되지 않는다. 맵을 다시 로드하거나 시뮬레이터를 재시작할 것.
# `show` 가 파일 값과 실제 /scan 의 range_max 를 같이 찍어 준다 --
# 둘이 다르면 아직 재시작이 안 된 것이다.
set -u

SDF=/opt/physicar/src/physicar-sim/share/models/physicar/model.sdf
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BACKUP="$HERE/../sim_overrides/model.sdf.orig"

file_value() {
  grep -oP '(?<=<max>)[0-9.]+(?=</max>)' "$SDF" 2>/dev/null | head -1
}

live_value() {
  timeout 8 ros2 topic echo --once --field range_max /scan 2>/dev/null \
    | head -1 | tr -d ' -'
}

case "${1:-}" in
  show)
    echo "model.sdf : $(file_value) m"
    v=$(live_value)
    if [ -n "$v" ]; then
      echo "/scan     : $v m"
      echo "(둘이 다르면 시뮬레이터 재시작 / 맵 재로드가 필요하다)"
    else
      echo "/scan     : 발행 없음 (시뮬레이터가 안 떠 있음)"
    fi ;;

  set)
    new="${2:-}"
    [ -z "$new" ] && { echo "usage: $(basename "$0") set <미터>" >&2; exit 1; }
    [ -f "$BACKUP" ] || cp "$SDF" "$BACKUP"
    # <max> 태그는 이 파일에 라이다 range 하나뿐이다 (확인 완료).
    sed -i "s#<max>[0-9.]*</max>#<max>${new}</max>#" "$SDF"
    echo "model.sdf 라이다 최대거리 -> $(file_value) m"
    echo "★ 시뮬레이터를 재시작해야 반영된다." ;;

  restore)
    if [ ! -f "$BACKUP" ]; then
      echo "백업이 없다: $BACKUP" >&2; exit 1
    fi
    cp "$BACKUP" "$SDF"
    echo "공장 초기값 복원 -> $(file_value) m"
    echo "★ 시뮬레이터를 재시작해야 반영된다." ;;

  *)
    echo "usage: $(basename "$0") {show|set <미터>|restore}" >&2
    exit 1 ;;
esac
