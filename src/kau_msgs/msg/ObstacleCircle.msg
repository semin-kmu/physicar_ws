# 원형으로 근사한 물리 장애물 하나.
#
# 단위: 길이 전부 [m]. KauPath 가 쓰는 [cm] 와 다르므로 혼용 금지.
#
# center 는 이 원이 담긴 ObstacleCircleArray.header.frame_id 기준 좌표다.
# 자기 자신은 frame 을 들고 있지 않으므로 항상 배열 header 와 함께 해석한다.
#
# radius 는 관측된 물리 장애물의 반지름만 담는다.
# 차량 크기, 안전 margin, inflation 은 전부 Local Planner 가 적용한다.

float32 center_x            # [m] 배열 header.frame_id 기준
float32 center_y            # [m] 배열 header.frame_id 기준
float32 radius              # [m] 물리 장애물 반지름. safety margin 미포함
float32 confidence          # 0.0 ~ 1.0. v1 승인 후보는 전부 1.0
