# 백업 스택 기동.
#
#     source run_backup.sh
#
# 아래 "설정" 값만 고쳐서 쓴다. 인자는 받지 않는다.
# Ctrl-C 로 내리면 ros2 launch 가 기동 역순으로 정리한다.
#
# source 로 부르는 것을 전제로 짰다 -- exec / set -e / exit 를 쓰지 않는다.
# 셋 중 무엇이든 있으면 Ctrl-C 나 오류에 터미널이 같이 닫힌다.
#
# ★ /speed 와 /steering 을 발행한다. 기존 kau_* 스택(run.sh)과
#   절대 동시에 띄우지 않는다. 두 publisher 의 값이 섞여 차량에 간다.
#   백업본을 상시로 쓸 때는 기존 run.sh 를 run_kau.sh 로 옮기고
#   이 파일을 run.sh 로 이름을 바꾼다.


# ====================================================================
# 노드 스위치 — 어느 노드를 띄울지
# ====================================================================
#
# 왼쪽이 노드 이름(backup_bringup/launch/backup.launch.py 의 STACK),
# 오른쪽이 true/false. false 면 이번 기동에서 뺀다.
#
# 순서는 실제 기동 순서와 같다. 여기서 순서를 바꿔도 기동 순서는
# 안 바뀐다 -- 순서의 주인은 backup.launch.py 다.
#
# ★ 없는 이름을 적으면 기동이 선다 (오타를 켠 것으로 넘기지 않는다).
#
BACKUP_NODES="
    backup_start_signal_detector  true
    backup_obstacle_detector      true
    backup_lane_detector          true
    backup_path_planner           true
    backup_speed_controller       true
    backup_steer_controller       true
    backup_gui                    false
"

# ====================================================================
# 설정 — 여기만 고친다
# ====================================================================

# Gazebo 는 true, 실제 차량은 false.
# 카메라 주점(cy)과 ROI 행이 여기에 따라 갈린다
# (backup_lane_detection/config/platform/{sim,real}.yaml).
BACKUP_USE_SIM_TIME=true

# 로그 레벨: debug | info | warn | error
BACKUP_LOG_LEVEL=info

# 워크스페이스 환경(install/setup.bash) 자동 소싱 여부
BACKUP_AUTO_SOURCE=true

# ====================================================================
# 이하 수정 불필요
# ====================================================================

# 이 스크립트가 있는 위치를 워크스페이스 루트로 쓴다. 절대경로를 박지
# 않는다 -- 다른 컴퓨터에서 그대로 돌아야 한다.
# (source 로 부르면 $0 가 아니라 ${BASH_SOURCE[0]} 를 봐야 한다)
if [ -n "${BASH_SOURCE[0]:-}" ]; then
    BACKUP_WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
else
    BACKUP_WS="$(cd "$(dirname "$0")" && pwd)"
fi

if [ "${BACKUP_AUTO_SOURCE}" = "true" ] && \
   [ -f "${BACKUP_WS}/install/setup.bash" ] && \
   ! printf '%s' "${AMENT_PREFIX_PATH:-}" | grep -qF "${BACKUP_WS}/install"
then
    echo "[run_backup] ${BACKUP_WS}/install/setup.bash 를 잡는다"
    . "${BACKUP_WS}/install/setup.bash"
fi

# 표를 읽어 false 인 이름만 모은다. 파이프를 쓰면 서브셸이라 결과가
# 안 남으므로 here-string 으로 먹인다.
backup_skip=""
backup_bad=""
while read -r backup_name backup_on _; do
    case "${backup_name}" in
        ''|'#'*) continue ;;
    esac
    case "${backup_on}" in
        true)  ;;
        false) backup_skip="${backup_skip:+${backup_skip},}${backup_name}" ;;
        *)     backup_bad="${backup_bad} ${backup_name}=${backup_on:-(빈값)}" ;;
    esac
done <<< "${BACKUP_NODES}"

if [ -n "${backup_bad}" ]; then
    # 오타를 그냥 켠 것으로 넘기면 끈 줄 알았던 노드가 조용히 뜬다.
    echo "[run_backup] BACKUP_NODES 는 true 나 false 만 받는다:${backup_bad}" >&2
else
    backup_args=("use_sim_time:=${BACKUP_USE_SIM_TIME}"
                 "log_level:=${BACKUP_LOG_LEVEL}")
    [ -n "${backup_skip}" ] && backup_args+=("skip:=${backup_skip}")

    echo "[run_backup] ros2 launch backup_bringup backup.launch.py ${backup_args[*]}"
    ros2 launch backup_bringup backup.launch.py "${backup_args[@]}"
fi

# source 로 불렸을 때 셸에 흔적을 남기지 않는다.
unset backup_args backup_skip backup_bad backup_name backup_on BACKUP_WS
