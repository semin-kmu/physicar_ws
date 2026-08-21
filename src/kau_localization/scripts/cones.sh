#!/usr/bin/env bash
#
# Gazebo 라바콘 제거 / 복원 (런타임 전용, world 파일은 건드리지 않는다).
#
#   ./cones.sh remove    라바콘 6개 제거
#   ./cones.sh restore   원래 pose 그대로 복원
#   ./cones.sh list      현재 월드에 남은 라바콘 확인
#
# 시뮬레이터를 재시작하면 라바콘은 어차피 다시 살아난다.
# cones/*.sdf 는 world 파일에서 떼어낸 <model> 블록이라 pose 가 원본과 같다.
#
# gz-sim 의 reset 은 쓰지 않는다. sim time 을 0 으로 되감아서
# use_sim_time 으로 도는 ekf_filter_node / laser_odom 의 tf2 버퍼가 깨진다.
set -u

WORLD=custom_71e69ee938032295503bfed557fde18c
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/cones"
CONES=(cone1 cone2 cone3 cone4 cone5 cone6)

call() {  # call <service> <req>
  gz service -s "/world/$WORLD/$1" --reqtype "$2" --reptype gz.msgs.Boolean \
    --timeout 5000 --req "$3" 2>/dev/null | tr -d '\n'
}

case "${1:-}" in
  remove)
    for c in "${CONES[@]}"; do
      printf '%-8s ' "$c"
      call remove gz.msgs.Entity "name: \"$c\", type: MODEL"
      echo
    done ;;
  restore)
    for c in "${CONES[@]}"; do
      printf '%-8s ' "$c"
      call create gz.msgs.EntityFactory "sdf_filename: \"$DIR/$c.sdf\", name: \"$c\""
      echo
    done ;;
  list)
    gz model --list 2>/dev/null | grep -c cone | sed 's/^/라바콘 /;s/$/개/' ;;
  *)
    echo "usage: $(basename "$0") {remove|restore|list}" >&2; exit 1 ;;
esac
