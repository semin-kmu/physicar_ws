# 한 scan 주기의 장애물 전체. 단위 m
# 실패 상태에서 이전 주기를 재사용하지 않는다 -- non-OK 는 항상 빈 배열이다.

std_msgs/Header header      # frame_id: base_link, stamp: 원본 LaserScan 시각

uint8 STATUS_OK=0
uint8 STATUS_LIDAR_UNAVAILABLE=1
uint8 STATUS_INTERNAL_ERROR=2
uint8 status

ObstacleCircle[] obstacles
