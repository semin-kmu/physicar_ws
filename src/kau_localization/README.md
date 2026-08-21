# kau_localization

PhysiCar 용 Cartographer 2D SLAM / pure localization 설정 패키지.

실행 노드는 `cartographer_ros` 가 제공하므로 이 패키지는 lua 설정, launch,
그리고 재측위 릴레이 노드 하나만 담는다.

## TF 소유권

```
map ──────────────► odom              ← cartographer_node   (이 패키지)
odom ─────────────► base_footprint    ← ekf_filter_node     (physicar_bringup)
base_footprint ───► base_link ─► lidar_link / imu_link / camera_*
                                      ← robot_state_publisher (URDF)
```

`odom -> base_footprint` 는 이미 EKF 가 소유하므로 lua 의
`provide_odom_frame = false`, `published_frame = "odom"` 은 바꾸면 안 된다.
바꾸면 부모가 둘인 프레임이 생겨 tf2 가 깨진다.

같은 이유로 `map -> odom` 을 발행하는 노드는 동시에 하나만 떠야 한다.
`slam_toolbox`, `nav2` 의 `amcl`(`tf_broadcast: true`) 과 함께 띄우면 충돌한다.

## 입출력

| | |
|---|---|
| 입력 | `/scan` (LaserScan), `/odom` (Odometry, EKF 출력), TF `base_footprint -> lidar_link` |
| 출력 | TF `map -> odom`, `/map`, `/submap_list`, `/constraint_list`, `/trajectory_node_list`, `/scan_matched_points2` |

`/imu` 는 직접 쓰지 않는다 (`use_imu_data = false`). 38 Hz 는 Cartographer
pose extrapolator 에 느리고, 어차피 EKF 를 통해 `/odom` 에 반영된다.

## 사용법

physicar_bringup 의 `sim.launch.py` / `real.launch.py` 가 떠 있는 상태에서 얹는다.

### 1. 지도 작성

```bash
ros2 launch kau_localization slam.launch.py rviz:=true
# 실기라면
ros2 launch kau_localization slam.launch.py use_sim_time:=false rviz:=true
```

로봇을 천천히 주행시켜 지도를 채운 뒤 저장한다.

```bash
./scripts/save_map.py
```

저장된 지도는 나중에 언제든 다시 볼 수 있다 (ROS 안 떠 있어도 된다):

```bash
./scripts/view_map.py --all          # 버전별 한 줄 비교
./scripts/view_map.py --walls        # 벽이 몇 개이고 어디 있는지 (이중 벽 찾기)
./scripts/view_map.py --png          # PNG 로 뽑아서 이미지 뷰어로 보기
```

`finish_trajectory` -> 최적화 대기 -> `write_state` -> pgm 변환 -> 품질 검사를
한 번에 한다. 파일명은 `maps/kau_vN.*` 로 **자동 증가**하므로 기존 지도를
덮어쓰지 않는다. 매핑이 마음에 안 들면 그냥 다시 매핑하고 다시 실행하면 된다.

수동으로 하려면:

```bash
ros2 service call /finish_trajectory \
    cartographer_ros_msgs/srv/FinishTrajectory "{trajectory_id: 0}"
ros2 service call /write_state cartographer_ros_msgs/srv/WriteState \
    "{filename: '/home/physicar/physicar_ws/src/kau_localization/maps/kau_v1.pbstream', include_unfinished_submaps: true}"
```

> **주의**: `save_state:=<경로>` 를 주지 않고 띄웠다면 노드를 그냥 종료해도
> **저장되지 않는다.** 반드시 위 중 하나로 저장한 뒤에 끌 것.

### 2. 저장된 지도 위에서 위치 추정

```bash
ros2 launch kau_localization localization.launch.py \
    pbstream:=/home/physicar/physicar_ws/src/kau_localization/maps/kau.pbstream rviz:=true
```

재부팅하면 `/odom` 이 0 에서 다시 시작하므로 Cartographer 는 로봇이 지도
어디에 있는지 모른다. RViz 의 **2D Pose Estimate** 로 대략 위치를 찍으면
`initial_pose_relay` 가 `/finish_trajectory` + `/start_trajectory` 로
그 자리에서 재측위한다. 찍지 않아도 전역 재탐색으로 언젠가 수렴하지만 느리다.

### 3. Nav2 용 정적 지도로 변환

```bash
ros2 run cartographer_ros cartographer_pbstream_to_ros_map \
    -pbstream_filename /home/physicar/physicar_ws/src/kau_localization/maps/kau.pbstream \
    -map_filestem /home/physicar/physicar_ws/src/kau_localization/maps/kau -resolution 0.05
```

이때 `map_server` 는 TF 를 발행하지 않게 두고, `map -> odom` 은 계속
`localization.launch.py` 가 담당하게 한다. `amcl` 은 끈다.

## 파일

| 경로 | 설명 |
|---|---|
| `config/physicar_2d.lua` | SLAM 설정. 프레임/센서 튜닝의 기준 |
| `config/physicar_2d_localization.lua` | 위 파일을 include + pure localization trimmer |
| `launch/slam.launch.py` | cartographer_node + occupancy_grid_node |
| `launch/localization.launch.py` | 위 + initial_pose_relay |
| `src/initial_pose_relay.cpp` | `/initialpose` -> `/start_trajectory` 릴레이 (C++) |
| `rviz/cartographer.rviz` | map / scan / constraint / TF 뷰 |
| `scripts/save_map.py` | 매핑 종료 후 최적화 + `kau_vN` 자동 버전 저장 + pgm 변환 + 품질 검사 |
| `scripts/check_constraints.py` | 매핑 중 루프 클로저 constraint 길이 분포 확인 (아래 주의 참고) |
| `scripts/view_map.py` | 저장된 지도 확인. ROS 불필요. 검사 + ASCII + 벽 정밀측정 + PNG |
| `maps/kau_vN.pbstream` | 저장된 KAU 지도. localization.launch.py 의 `pbstream:=` 대상 |
| `maps/kau_vN.pgm`, `maps/kau_vN.yaml` | 위 pbstream 을 변환한 Nav2 용 정적 지도 |

## 주의

- `cartographer_node` 의 gflags 인자(`-configuration_directory` 등)는 반드시
  `--ros-args` **앞**에 와야 한다. 뒤에 두면 즉시 죽는다. launch 는 이미 맞춰뒀다.
- `scan_topic` 기본값은 raw `/scan` 이다. `scan_filter` 는 무효값을 `0.0` 으로
  바꾸는데 `0.0` 은 `range_min`(0.1) 미만이라 Cartographer 가 그냥 버린다.
- **루프 클로저 파라미터는 이 방 크기에 맞춰 조인 값이다.** Cartographer 기본값
  (`max_constraint_distance = 15 m`, `fast_correlative linear_search_window = 7 m`)
  은 넓은 실외 기준이라, 12 x 7 m 빈 직사각형에서는 방 안 어디든 후보가 되어
  **반대편 submap 에 붙는 가짜 constraint 가 수천 개** 생긴다. 그게 pose graph 를
  잡아당겨 지도가 평행사변형으로 찌그러진다 (2026-08-19 실측: inter constraint
  30,021 개 중 8 m 초과 5,046 개, 지도가 9.9 x 13.8 m 로 부풀었다).
  그래서 `max_constraint_distance = 4.0` / `linear_search_window = 3.0` 으로 조였다.
  단 **너무 조이면 반대로 진짜 루프 클로저가 안 걸린다.** 처음에 `min_score = 0.70` /
  `sampling_ratio = 0.15` 로 잡았더니 찌그러짐은 사라졌지만 같은 벽이 0.3~0.45 m
  떨어진 두 위치에 그려졌다(두 바퀴 주행했는데도 루프가 안 닫힘). 거리 제한은 두고
  점수만 풀어 `min_score = 0.65` / `sampling_ratio = 0.3` 으로 되돌렸다.
  **주행 공간이 바뀌면 이 값들을 다시 잡아야 한다.**
  매핑 **중에** `scripts/check_constraints.py` 로 확인한다 — constraint 가 너무 적으면
  `min_score` 를 더 풀고, 6 m 초과가 있으면 거리 제한을 더 조인다.
- **매핑 중에는 RViz 를 끄는 편이 낫다.** 4 코어 환경에서 rviz2 가 CPU 136% 를
  쓴다 (cartographer 는 35%). `/constraint_list` 점이 11 만 개까지 늘어나기
  때문이다. RViz 자신도 메시지를 버리고 있어 화면이 실시간이 아니다.
- lua 의 ceres `num_threads` 를 명시해 뒀다. 기본값은 4코어 환경에서
  "exceeds maximum available" 경고를 낸다.
