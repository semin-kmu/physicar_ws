# quintic Bezier segment 열. 단위 m/rad, 좌측 +
# P0 는 항상 후륜축 원점, P1 은 진행방향 접선 -- 모드가 바뀌어도
# 자차 근처 경로가 움직이지 않게 하는 구조적 제약이다.

std_msgs/Header header      # frame_id: base_link, stamp: 원본 이미지 시각

uint8 SRC_LANE_CENTER=0
uint8 SRC_RACE_LINE=1
uint8 SRC_AVOID=2
uint8 SRC_DEGRADED=3        # 연속 코너 퇴화 모드
uint8 source

uint8     degree            # 5 고정
float64   total_length      # [m]
float64[] ctrl_x            # [m] 길이 = nseg*(degree+1)
float64[] ctrl_y            # [m]
float64[] seg_length        # [m] Gauss-Legendre 10 점
float32[] seg_kappa_max     # [1/m]
float32   confidence        # 0.0 ~ 1.0
float64   valid_length      # [m] 근거가 실제로 닿는 길이. Ld 를 이 안에 둔다

# --- 제어 부가정보 ---
float64 corner_entry_s      # [m] 코너 진입까지 호길이. 음수면 코너 없음
float32 corner_kappa_max    # [1/m]
bool    kappa_saturated     # 1/R_min 초과 요구 = 조향으로 통과 불가
bool    avoiding
bool    stop_request
uint8   lost_state          # LaneGeometry 의 LOST_* 와 동일
uint8   lost_first
