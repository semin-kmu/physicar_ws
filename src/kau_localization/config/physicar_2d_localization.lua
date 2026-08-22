-- Copyright 2026 KAU AMET Team
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--     http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.
--
-- PhysiCar 2D pure localization 설정.
--
-- physicar_2d.lua 를 그대로 상속하므로 프레임/토픽/센서 튜닝은 동일하고,
-- 지도를 새로 키우는 대신 -load_state_filename 으로 읽은 frozen submap 에
-- 붙어서 위치만 추정한다. 두 파일은 같은 디렉터리에 있어야 include 가 풀린다.

include "physicar_2d.lua"

-- 최근 submap 3개만 유지. 이게 없으면 pure localization 이라도
-- 주행 시간에 비례해 submap 이 계속 쌓여 메모리와 CPU 를 먹는다.
TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = 3,
}

-- mapping 보다 자주 최적화해야 재측위 후 수렴이 빠르다.
POSE_GRAPH.optimize_every_n_nodes = 20

-- 전역 재측위 샘플링. 초기 pose 를 안 주고 띄웠을 때 스스로 찾아오는 속도를
-- 좌우한다. 올리면 빨리 찾지만 CPU 를 더 쓴다.
--
-- 2026-08-22: cartographer 기본값(0.003 / 0.3)에서 낮췄다. tuning 문서가
-- pure localization 에서 권하는 것이다 — frozen 궤적(지도)과 현재 궤적 사이에
-- inter constraint 가 대량으로 생기므로 그걸 상쇄하라는 취지다.
--   "we strongly decrease global_sampling_ratio and
--    constraint_builder.sampling_ratio to compensate for the large number
--    of constraints"  (cartographer_ros docs/source/tuning.rst)
--
-- 대가: /initialpose 를 안 주고 띄우면 전역 재탐색이 그만큼 느려진다.
-- 리셋 대응은 scripts/reset_watcher.py 가 스폰 좌표를 쏘므로 영향이 없다.
-- 되돌리려면 0.003 / 0.3 으로 되돌리면 된다 (= cartographer 기본값).
POSE_GRAPH.global_sampling_ratio = 0.001             -- 기본 0.003
POSE_GRAPH.constraint_builder.sampling_ratio = 0.1   -- 기본 0.3

-- 시도했다가 되돌린 것 (2026-08-22)
--   ceres_scan_matcher.rotation_weight 40 -> 10
--
-- "회전이 늦게 따라온다" 는 증상에 손댔던 값이다. 이 가중치는 ceres 비용에서
-- prior(= pose extrapolator 가 /odom 으로 외삽한 자세) 의 각도에 붙잡아 두는
-- 용수철 강성이다. 낮추면 스캔이 회전을 지배한다.
--
-- 결과는 더 나빴다. 주행면이 12 x 7 m 빈 직사각형이라 어느 벽을 봐도 스캔이
-- 비슷하게 생겨서, 회전을 풀어주면 틀린 각도도 스캔상으로는 그럴듯해진다.
-- 매처가 엉뚱하게 돌고 그 자세로 live submap 이 삽입돼 지도가 어긋나 겹쳤다.
--
-- 게다가 여기서 prior 의 회전은 믿을 만하다 — EKF 가 회전을 100% IMU 로만
-- 받고 (ekf_params.yaml 의 odom0_config 는 yaw·vyaw 가 둘 다 false),
-- sim 자이로는 angular_velocity_covariance 가 4e-8 로 거의 완벽하다.
-- 좋은 prior 를 버리고 애매한 스캔에 맡긴 셈이었다.
--
-- 그래서 40 (= cartographer 기본값) 으로 돌아갔다.
--
-- 2026-08-22 (2차): 40 -> 60 도 시험했으나 이득이 없어 되돌렸다.
--
-- 실측으로 방향이 뒤집혔다. 회전은 늦는 게 아니라 빠를 때 흔들린다:
--   gyro / EKF yaw : 지연 0.00 s, 상관 0.98   (prior 는 단기적으로 정확하다)
--   cartographer   : 지연 0.00 s, 상관 0.80   (흔들린다)
--   각속도별 yaw 오차 평균 — <30 deg/s 1.3도 · 30~80 5.2도 · >80 6.6도
-- 라이다가 10 Hz 라 80 deg/s 면 스캔 사이에 8도, 165 deg/s 면 16.5도를 건너뛴다.
-- 그 구간은 prior 외삽에만 의존하고 다음 스캔에서 몰아서 보정하니 튄다.
-- prior 가 정확하니 조금 더 믿게 해서 그 지터를 누른다.
--
-- 60 실측 결과: 고속 구간은 5.23 -> 4.64도로 조금 나아졌으나 저속 구간이
-- 1.31 -> 2.34도, 최대 오차가 17 -> 23도로 나빠졌다. 게다가 같은 두 주행에서
-- EKF 드리프트가 5.9 -> 7.4 deg/min 로 변했는데 EKF 는 상류라 이 값의 영향을
-- 받을 수 없다 = 그 25% 가 주행 차이에서 오는 노이즈 바닥이다. 10% 안팎의
-- 개선은 그 안에 묻힌다.
--
-- 내려도(10) 나빴고 올려도(60) 이득이 없으므로 40 (cartographer 기본값) 을
-- 그대로 쓴다. 고속 회전의 5~7도 오차는 가중치로 못 없앤다 — 10 Hz 라이다로
-- 80 deg/s 를 돌면 스캔 사이 8도가 비고, 그 구간은 prior 외삽뿐이다.

-- 2026-08-22 (3차): 스캔 사이 회전을 IMU 로 메운다.
--
-- 실측: 고속 회전에서 yaw 오차가 커진다 (<30 deg/s 1.3도, >80 deg/s 6.6도).
-- 라이다가 10 Hz 라 80 deg/s 면 스캔 사이 8도가 비는데, 그 100 ms 를 지금은
-- /odom(30 Hz) 외삽으로만 메운다. IMU 를 cartographer 에 직접 주면
-- PoseExtrapolator 가 그 구간의 회전을 47 Hz 로 채운다.
--
-- use_odometry 는 끄지 않는다. 둘은 배타적이 아니라 상보적이다 —
-- 회전은 IMU, 병진은 odometry 로 간다. EKF 도 그대로 둔다. EKF 는
-- odom -> base_footprint TF 의 소유자이고 (provide_odom_frame = false 라
-- cartographer 는 그걸 절대 발행하지 않는다) PoseExtrapolator 는 TF 를
-- 발행하지 않는 내부 예측기라 서로 대체 관계가 아니다.
--
-- tracking_frame 을 같이 바꿔야 한다. cartographer_ros 는 IMU 프레임이
-- tracking_frame 과 같은 자리에 있어야 한다고 CHECK 하고, 아니면 죽는다:
--   "The IMU frame must be colocated with the tracking frame."
-- base_footprint -> imu_link 는 z 로 6.5 cm 떨어져 있어 (URDF: 0.0375 +
-- 0.0275) 그대로 켜면 노드가 즉사한다.
--
-- 저장된 지도는 그대로 쓸 수 있다. 오프셋이 순수 z 뿐이고
-- publish_frame_projected_to_2d = true 라 2D 평면에서 xy 변화가 0 이다.
--
-- 부작용: 같은 자이로가 EKF 와 cartographer 양쪽에 들어가 이중 계산이 된다.
-- physicar_2d.lua 가 IMU 를 EKF 경유로만 쓴 이유가 그것이다. 실측으로 판단한다.
-- 실측 결과 (주행 조건이 잘 맞은 비교 — 각속도 RMS 56 vs 55, 최대 166 vs 165):
--   <30 deg/s   1.31 -> 1.00 도   개선
--   30~80       5.23 -> 4.47 도   개선
--   >80         6.58 -> 8.03 도   **악화** (노렸던 구간이 오히려 나빠졌다)
--   각속도 상관 0.803 -> 0.738, 절대 오차 표준편차 4.65 -> 5.18
--
-- 짚이는 원인은 중력 정렬이다. use_imu_data 를 켜면 ImuTracker 가 자이로를
-- 적분하면서 가속도계로 중력 방향도 추정하고, 2D 투영이 그 축을 기준으로 된다.
-- 급선회 중에는 원심 가속도가 중력과 섞여 축이 흔들리고 스캔 투영이 왜곡된다.
-- 47 Hz 회전 외삽의 이득과 상쇄돼 버린다.
--
-- 게다가 EKF 드리프트가 같은 두 주행에서 5.9 -> 3.6 deg/min 로 39% 변했는데
-- EKF 는 이 설정의 영향을 못 받는다 = 주행 차이만으로 그만큼 흔들린다는 뜻이라,
-- 위 15~24% 는 노이즈 안이다. 그래서 되돌렸다.

return options
