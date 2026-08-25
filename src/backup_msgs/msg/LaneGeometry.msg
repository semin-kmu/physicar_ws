# 차선 기하. 경로가 아니다 -- 경로 생성은 backup_path_planner 소유.
# 거리 원점은 후륜축. 단위 m/rad, 좌측 +

std_msgs/Header header      # frame_id: base_link, stamp: 원본 이미지 시각

LaneLine left
LaneLine right
LaneLine center
bool left_valid
bool right_valid
bool center_valid

# --- 변곡점 ---
# 판정 히스테리시스: 진입 20 deg / 이탈 15 deg
bool    inflection_valid
float64 inflection_x        # [m] 후륜축 기준 종방향
float64 inflection_y        # [m]
float64 delta_psi           # [rad] 진입/탈출 직선 교각. 좌회전 +
float64 arc_length          # [m] 전환 구간 호길이
bool    second_inflection   # 시야 내 변곡점 2개 이상 = 연속 코너

# --- 차선 손실 ---
uint8 LOST_NONE=0
uint8 LOST_LEFT=1
uint8 LOST_RIGHT=2
uint8 LOST_BOTH=3
uint8 lost_state
uint8 lost_first            # LOST_BOTH 일 때 먼저 끊긴 쪽

float64 valid_length        # [m] 실제로 검출된 최원거리. 그 너머는 외삽
