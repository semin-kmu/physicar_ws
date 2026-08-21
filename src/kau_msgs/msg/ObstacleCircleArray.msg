# Object Detection 이 한 관측 주기마다 발행하는 Object List.
#
# 단위: 길이 전부 [m]. KauPath 가 쓰는 [cm] 와 다르므로 혼용 금지.
#
# header.stamp 는 정상 프레임에서 원본 LaserScan 의 측정 시각이다.
# 발행 시각이 아니다. scan 자체가 끊겨 watchdog 이 상태만 내보내는
# 경우에만 발행 node 의 clock now 를 쓴다.
#
# header.frame_id 는 상태와 무관하게 항상 동일하다. 실패 상태에서도
# 수신측이 frame 을 분기 처리할 필요가 없도록 통일한다.

std_msgs/Header header

# --- status ---
# STATUS_OK                    정상 관측. obstacles 가 이번 주기의 전체 결과다.
# STATUS_LIDAR_UNAVAILABLE     LaserScan 이 없거나 구조 검증에 실패했다.
# STATUS_POSE_UNAVAILABLE      차량 pose 가 없거나 오래되어 절대 좌표를 못 만든다.
# STATUS_TRANSFORM_UNAVAILABLE 목표 frame 으로의 변환을 얻지 못했다.
# STATUS_INTERNAL_ERROR        설정 오류 등 발행 node 내부 문제다.
uint8 STATUS_OK=0
uint8 STATUS_LIDAR_UNAVAILABLE=1
uint8 STATUS_POSE_UNAVAILABLE=2
uint8 STATUS_TRANSFORM_UNAVAILABLE=3
uint8 STATUS_INTERNAL_ERROR=4

uint8 status

# STATUS_OK + 빈 배열   = "정상 관측, 장애물 없음". 유효한 결과이며 그대로 쓴다.
# non-OK  + 빈 배열   = "현재 인식 결과 사용 불가". 장애물이 없다는 뜻이 아니다.
#
# 실패 상태에서 발행 node 는 이전 주기의 장애물을 절대 재사용하지 않는다.
# 따라서 non-OK 프레임의 obstacles 는 항상 비어 있다.
ObstacleCircle[] obstacles
