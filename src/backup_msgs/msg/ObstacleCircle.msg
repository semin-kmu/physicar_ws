# 원으로 근사한 장애물 하나. 단위 m
# 반지름은 검출값이 아니라 설정값(obstacle_radius_m)으로 통일한다.
float32 center_x            # [m] 배열 header.frame_id 기준
float32 center_y            # [m]
float32 radius              # [m] 안전 margin 미포함
float32 confidence          # 0.0 ~ 1.0
