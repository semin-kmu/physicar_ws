# steer_controller 내부 상태. 디버깅 / 튜닝 전용.
#
# 제어 판단에 쓰지 않는다. 구독자 유무가 조향 결과에 영향을 주지 않는다.
# 값은 전부 steer_controller 가 이미 계산해 쓰던 것이고, 이 message 는
# 그것을 밖으로 내보내기만 한다 (새로 계산하는 양은 없다).
#
# 단위: 길이 cm / 각도 deg 또는 rad (필드별 명시)

std_msgs/Header header       # stamp: 제어 tick 시각
                             # frame_id: 지금 추종 중인 경로의 프레임

# false 면 아래 수치는 전부 무의미하고 조향 0 이 나가는 중이다.
# "노드가 죽음" 과 "노드는 살아 있으나 추종 불가" 를 구분하기 위해
# 실패 tick 에도 이 message 는 계속 발행한다.
bool    tracking_ok

uint8   path_source          # kau_msgs/KauPath 의 SRC_* 와 동일

# 최종 출력만 낸다. clamp 전 원출력(raw_steer_deg)은 2026-08-24 에 뺐다 --
# 실제로 나가는 값과 다른 수치가 같이 그려지면 어느 쪽이 차량에 갔는지
# 헷갈린다. 조향 포화는 이 값이 +-max_steer_deg 에 붙는 것으로 알 수 있다.
float64 cmd_steer_deg        # [deg] clamp 후. /steering 과 같은 값 (rad 변환 전)
float64 lookahead_cm         # [cm] Ld. valid_length 절단까지 반영한 최종값
float64 heading_error_rad    # [rad] 차량 yaw - 경로 접선. 좌측 +
float64 cross_track_cm       # [cm] 경로 최근접점까지 횡오차. 좌측 +
float64 s_cm                 # [cm] 경로 시작점부터의 호길이
