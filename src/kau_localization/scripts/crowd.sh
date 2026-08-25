#!/usr/bin/env bash
#
# 사람 노이즈 뿌리기 / 걷어내기 (런타임 전용, world 파일은 건드리지 않는다).
#
#   ./crowd.sh spawn              기본 배치 43명 (관중 30 + 인필드 8 + 도로 5)
#   ./crowd.sh spawn --outside 70 --stand-off 0.18    더 붐비게
#   ./crowd.sh clear              전부 제거
#   ./crowd.sh list               현재 떠 있는지 확인
#   ./crowd.sh where              좌표표 (sim / map 양쪽)
#
# 밀도별 /scan 실측표는 crowd.py 상단 주석에 있다. 사람 수보다
# --stand-off (트랙에서 떨어진 거리) 가 결과를 더 크게 바꾼다.
#
# 무엇을 재현하려는 것인가 (대회장 실측 3 가지):
#   1. 사람이 많아 포인트가 분산되어 찍힌다     -> 도로위 + 인필드 인원
#   2. 맵 가장자리 펜스가 잘 안 찍힌다          -> 관중이 벽을 가린다
#   3. 가까운 2/3 만 찍히고 먼 공간은 약하다     -> 관중이 원거리를 막는다
#
# 라이다에만 보이고 차는 통과한다 (gpu_lidar 는 visual 을 레이캐스팅하고
# collision 은 물리 엔진 쪽이라 별개다). 실제로 막고 싶으면 --collide.
#
# gz-sim 의 reset 은 쓰지 않는다. sim time 을 0 으로 되감아서
# use_sim_time 으로 도는 ekf_filter_node / laser_odom 의 tf2 버퍼가 깨진다.
# (cones.sh 와 같은 이유)
set -u

WORLD=custom_71e69ee938032295503bfed557fde18c
MODEL=crowd_noise
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SDF=/tmp/${MODEL}.sdf

call() {  # call <service> <reqtype> <req>
  gz service -s "/world/$WORLD/$1" --reqtype "$2" --reptype gz.msgs.Boolean \
    --timeout 5000 --req "$3" 2>/dev/null | tr -d '\n'
}

cmd="${1:-}"
shift || true

case "$cmd" in
  spawn)
    # 이미 떠 있으면 먼저 걷어낸다. 같은 이름이 두 번 뜨면 gz 가
    # crowd_noise_0 처럼 이름을 바꿔 붙여서 clear 로 안 지워진다.
    call remove gz.msgs.Entity "name: \"$MODEL\", type: MODEL" >/dev/null
    sleep 0.5

    "$HERE/crowd.py" --model-name "$MODEL" "$@" > "$SDF" || exit 1

    printf '%-14s ' "$MODEL"
    call create gz.msgs.EntityFactory "sdf_filename: \"$SDF\", name: \"$MODEL\""
    echo
    echo "좌표: $(basename "$0") where $*" ;;

  clear)
    printf '%-14s ' "$MODEL"
    call remove gz.msgs.Entity "name: \"$MODEL\", type: MODEL"
    echo ;;

  list)
    if gz model --list 2>/dev/null | grep -q "$MODEL"; then
      echo "$MODEL 떠 있음"
    else
      echo "$MODEL 없음"
    fi ;;

  where)
    "$HERE/crowd.py" --list-only "$@" ;;

  *)
    echo "usage: $(basename "$0") {spawn|clear|list|where} [crowd.py 인자]" >&2
    exit 1 ;;
esac
