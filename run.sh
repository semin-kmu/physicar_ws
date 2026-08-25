# 전체 스택 기동. system_supervisor 하나만 띄우고 나머지는 그가 spawn 한다.
#
#     source ~/physicar_ws/physicar_ws/run.sh
#
# 아래 "설정" 값만 고쳐서 쓴다. 인자는 받지 않는다.
# Ctrl-C 로 내리면 supervisor 가 기동 역순으로 전부 정리한다.
#
# source 로 부르는 것을 전제로 짰다 -- exec / set -e / exit 를 쓰지 않는다.
# 셋 중 무엇이든 있으면 Ctrl-C 나 오류에 터미널이 같이 닫힌다.


# ====================================================================
# 노드 스위치 — 어느 노드를 띄울지
# ====================================================================
#
# 왼쪽이 노드 이름(kau_state_machine/config/bringup.yaml 의 name),
# 오른쪽이 true/false. false 면 이번 기동에서 뺀다.
#
# 순서는 실제 기동 순서(단계 0 -> 9)와 같다. 여기서 순서를 바꿔도
# 기동 순서는 안 바뀐다 -- 순서의 주인은 bringup.yaml 이다.
#
# 표에 없는 노드는 켠 것으로 본다. 없는 이름을 적으면 기동이 선다.
#
# ★ option 스위치보다 이 표가 세다. 주행 필수 노드도 뺄 수 있으니
#   끄기 전에 그게 없어도 되는지 확인할 것. 뺀 노드는 기동 로그에
#   경고로 남는다.
#
KAU_NODES="
    clock_gate                    true
    platform_ekf_pause            true
    kau_ekf                       true
    map_server                    true
    amcl                          true
    map_amcl_lifecycle_manager    true
    laser_scan_clusterer          true
    start_signal_detector         true
    camera_info_bridge            true
    kau_lane_detection_node       false
    kau_lane_detection_viewer     false
    global_path_publisher         true
    local_planner_node            true
    state_machine                 true
    steer_controller              true
    speed_controller              true
"

# ====================================================================
# 설정 — 여기만 고친다
# ====================================================================

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

# 이 스크립트가 있는 위치를 워크스페이스 루트로 사용한다.
# (source 로 부르면 $0 가 아니라 ${BASH_SOURCE[0]} 를 봐야 한다)
if [ -n "${BASH_SOURCE[0]:-}" ]; then
    KAU_WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
else
    KAU_WS="$(cd "$(dirname "$0")" && pwd)"
fi

if [ "${KAU_AUTO_SOURCE}" = "true" ] && \
   [ -f "${KAU_WS}/install/setup.bash" ] && \
   ! printf '%s' "${AMENT_PREFIX_PATH:-}" | grep -qF "${KAU_WS}/install"
then
    echo "[run.sh] ${KAU_WS}/install/setup.bash 를 잡는다"
    . "${KAU_WS}/install/setup.bash"
fi

# 표를 읽어 false 인 이름만 모은다. 파이프를 쓰면 서브셸이라 결과가
# 안 남으므로 here-string 으로 먹인다.
kau_skip=""
kau_bad=""
while read -r kau_name kau_on _; do
    case "${kau_name}" in
        ''|'#'*) continue ;;
    esac
    case "${kau_on}" in
        true)  ;;
        false) kau_skip="${kau_skip:+${kau_skip},}${kau_name}" ;;
        *)     kau_bad="${kau_bad} ${kau_name}=${kau_on:-(빈값)}" ;;
    esac
done <<< "${KAU_NODES}"

if [ -n "${kau_bad}" ]; then
    # 오타를 그냥 켠 것으로 넘기면 끈 줄 알았던 노드가 조용히 뜬다.
    echo "[run.sh] KAU_NODES 는 true 나 false 만 받는다:${kau_bad}" >&2
else
    # 빈 값은 넘기지 않는다. launch 쪽 기본값이 살아야 한다.
    kau_args=("use_sim_time:=${KAU_USE_SIM_TIME}")
    [ -n "${kau_skip}" ]         && kau_args+=("skip:=${kau_skip}")
    [ -n "${KAU_RUN_ID}" ]       && kau_args+=("run_id:=${KAU_RUN_ID}")
    [ -n "${KAU_BRINGUP_YAML}" ] && kau_args+=("bringup_yaml:=${KAU_BRINGUP_YAML}")

    echo "[run.sh] ros2 launch kau_state_machine state_machine.launch.py ${kau_args[*]}"
    ros2 launch kau_state_machine state_machine.launch.py "${kau_args[@]}"
fi

# source 로 불렸을 때 셸에 흔적을 남기지 않는다.
unset kau_args kau_skip kau_bad kau_name kau_on KAU_WS
