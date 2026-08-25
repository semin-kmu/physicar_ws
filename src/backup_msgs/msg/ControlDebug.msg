# 제어기 내부 상태. 튜닝/로그 전용이며 제어 판단에 쓰지 않는다.

std_msgs/Header header

uint8   path_source         # BackupPath 의 SRC_*
bool    tracking_ok

float64 cmd_speed           # [m/s] 실제 발행값
float64 v_target            # [m/s] 레이트 리밋 전 목표
float64 kappa_ahead         # [1/m] 전방 preview 구간 max|kappa|

float64 cmd_steer_deg       # [deg] clamp 후. /steering 과 같은 값
float64 lookahead_m         # [m] valid_length 절단까지 반영한 최종 Ld
float64 heading_error_rad   # [rad] 좌측 +
float64 cross_track_m       # [m] 좌측 +

bool    delta_saturated     # |delta| 가 delta_max 에 붙었다
bool    kappa_saturated     # 경로가 통과 불가 곡률을 요구했다
float64 dead_reckon_dt      # [s] 경로 stamp -> 이번 tick 보상량
