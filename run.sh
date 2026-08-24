# 전체 스택 기동. system_supervisor 하나만 띄우고 나머지는 그가 spawn 한다.
#
#     source ~/physicar_ws/run.sh
#
# 아래 "설정" 값만 고쳐서 쓴다. 인자는 받지 않는다.
# Ctrl-C 로 내리면 supervisor 가 기동 역순으로 전부 정리한다.
#
# source 로 부르는 것을 전제로 짰다 -- exec / set -e / exit 를 쓰지 않는다.
# 셋 중 무엇이든 있으면 Ctrl-C 나 오류에 터미널이 같이 닫힌다.


# ====================================================================
# 설정 — 여기만 고친다
# ====================================================================

# 차선 인지 BEV 웹 뷰어 (http://localhost:5000).
KAU_LANE_VIEWER=true

# Gazebo 는 true, 실제 차량은 false.
KAU_USE_SIM_TIME=true

# 로그 디렉터리 이름 (~/.ros/kau/<이름>). 비우면 기동 시각으로 자동
KAU_RUN_ID=""

# 기동 manifest. 비우면 kau_state_machine 의 share/config/bringup.yaml
KAU_BRINGUP_YAML=""

# 워크스페이스 환경(install/setup.bash) 자동 소싱 여부
KAU_AUTO_SOURCE=true

# ====================================================================
# 이하 수정 불필요
# ====================================================================

KAU_WS="/home/physicar/physicar_ws"

if [ "${KAU_AUTO_SOURCE}" = "true" ] && \
   [ -f "${KAU_WS}/install/setup.bash" ] && \
   ! printf '%s' "${AMENT_PREFIX_PATH:-}" | grep -q "${KAU_WS}/install"
then
    echo "[run.sh] ${KAU_WS}/install/setup.bash 를 잡는다"
    . "${KAU_WS}/install/setup.bash"
fi

# 빈 값은 넘기지 않는다. launch 쪽 기본값이 살아야 한다.
kau_args=(
    "lane_viewer:=${KAU_LANE_VIEWER}"
    "use_sim_time:=${KAU_USE_SIM_TIME}"
)
[ -n "${KAU_RUN_ID}" ]       && kau_args+=("run_id:=${KAU_RUN_ID}")
[ -n "${KAU_BRINGUP_YAML}" ] && kau_args+=("bringup_yaml:=${KAU_BRINGUP_YAML}")

echo "[run.sh] ros2 launch kau_state_machine state_machine.launch.py ${kau_args[*]}"
ros2 launch kau_state_machine state_machine.launch.py "${kau_args[@]}"

# source 로 불렸을 때 셸에 흔적을 남기지 않는다.
unset kau_args KAU_WS
