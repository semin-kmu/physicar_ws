# kau_localization

PhysiCar 용 2D 측위 패키지. Cartographer SLAM / pure localization 과,
그 대안인 AMCL 설정을 같이 담는다.

실행 노드는 `cartographer_ros` 와 `nav2` 가 제공하므로 이 패키지는 설정,
launch, 재측위 릴레이 노드 하나, 그리고 측정 스크립트만 담는다.

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
이 패키지의 `amcl.launch.py` 도 마찬가지다 — Cartographer 와 **택일**이다.

### 실측 발행 주기 (2026-08-23, sim)

| 링크 | 발행자 | 실측 | 설정 |
|---|---|---|---|
| `map -> odom` | `cartographer_node` | **175 Hz** | `pose_publish_period_sec = 5e-3` (= 200 Hz. 4코어라 못 채운다) |
| `map -> odom` | `amcl` | **9.9 Hz** | 스캔 주기. 값은 갱신 조건을 만족할 때만 바뀐다 |
| `odom -> base_footprint` | `ekf_filter_node` | **30.0 Hz** | `ekf_params.yaml` 의 `frequency: 30.0` |
| `base_link -> 바퀴/카메라` | `robot_state_publisher` | 20.0 Hz | `/joint_states` 주기 |

측정은 `scripts/tf_rate.py` 로 한다. **`ros2 topic hz /tf` 와
`ros2 run tf2_ros tf2_monitor` 는 여기서 쓰면 안 된다** — 둘 다 링크별로
가르지 못해서 EKF(30) + robot_state_publisher(20) 의 합인 49 Hz 를
`odom -> base_footprint` 의 주기라고 답한다. tf2_monitor 는 브로드캐스터를
`<no authority available>` 로 잡아 토픽 전체를 한 링크에 귀속시키고,
delay 값도 sim 시계라 `1.78e9` 로 무의미하다.

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
ros2 launch kau_localization cartographer_localization.launch.py \
    pbstream:=kau_v3 rviz:=true
```

지도는 항상 같은 폴더에 있으므로 **이름만** 준다. `kau_v3` · `kau_v3.pbstream` ·
`latest` 가 모두 되고, 다른 곳의 지도는 경로를 그대로 주면 된다.
이름으로 부를 때 뒤지는 곳은 설치된 `share/kau_localization/maps` 다. 워크스페이스가
`--symlink-install` 이라 거기 있는 지도는 소스 트리를 가리키는 심링크지만,
`save_map.py` 로 **새로** 저장한 지도는 심링크가 아직 없다. `colcon build
--packages-select kau_localization` 을 한 번 하거나 (1초쯤)
`maps_dir:=src/kau_localization/maps` 로 소스 폴더를 직접 가리키면 된다.

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
`cartographer_localization.launch.py` 가 담당하게 한다. `amcl` 은 끈다.
(이 패키지의 `amcl.launch.py` 를 쓸 때는 반대로 Cartographer 를 끈다.)

### 4. 리셋 / 재시작 버튼에 자동으로 대응 (장기 검증용)

```bash
source install/setup.bash
./src/kau_localization/scripts/reset_watcher.py
```

PhysiCar 앱의 **리셋** / **재시작** 버튼을 감지해서 localization 을 알아서
이어붙인다. 오래 돌리면서 위치 추정이 계속 살아있는지 볼 때 쓴다. 버튼을
누를 때마다 손으로 launch 를 다시 칠 필요가 없다.

띄우면 `cartographer_localization.launch.py` (RViz 포함) 를 먼저 기동하고, 이미 떠 있으면
그걸 물려받는다. `Ctrl-C` 로 끄면 자기가 띄운 것도 같이 정리한다.

**두 버튼은 성격이 달라서 대응이 다르다.**

| 버튼 | sim 시계 | 대응 | RViz |
|---|---|---|---|
| 리셋 | 계속 흐름 | `/initialpose` 만 스폰 좌표로 주입 | 안 끊김 |
| 재시작 | **0 으로 되감김** | 내렸다가 재런치 + `/initialpose` | 새로 뜸 |

리셋은 차량 pose 만 순간이동하는 것이라 재런치하면 trajectory 와 드리프트
이력이 날아가 장기 검증 자체가 끊긴다. 그래서 `initial_pose_relay` 로
trajectory 만 갈아끼운다. 재시작은 `/clock` 이 되감기므로 그대로 두면 TF 버퍼에
미래 stamp 가 남아 스스로 복구하지 못한다 — 반드시 재런치해야 한다.

**감지 방법.** 버튼은 원격 앱(`sim.physicar.ai`)에 있고 눌리면 `sim_api` 로
HTTP 요청만 간다. ROS 로는 토픽도 이벤트도 나오지 않으므로 (`/events` SSE 에도
없다) `sim_api` 로그(`/tmp/sim_api.log`)를 tail 해서 잡는다.

| 로그 줄 | 의미 |
|---|---|
| `light reset done` | 리셋 |
| `starting sim:` | 재시작 시작 — 여기서 즉시 내린다 |
| `physicar spawned, starting gz-launch` | 재시작 준비 완료 — 여기서 띄운다 |

`sim_api.py` 는 updater 가 덮어쓰는 벤더 파일이라 거기엔 훅을 심지 않았다.

**시계가 어긋나도 버틴다.** 감시 스크립트 자체는 벽시계로 돈다
(`use_sim_time=false`) — sim 시계가 멈추거나 되감겨도 감시 로직은 얼지 않는다.
TF 는 stamp 를 지정하지 않고 항상 latest 로만 조회하고, 되감김을 감지하면 TF
버퍼를 비운다. 재시작 뒤에는 `/status` 의 `running` · 시계가 실제로 전진하는지 ·
`/scan` 과 `/odom` 퍼블리셔 복귀를 모두 확인한 뒤에야 띄운다.
`/initialpose` 의 header stamp 는 `initial_pose_relay` 가 보지 않으므로 무관하다.

**좌표 보정.** 스폰 좌표는 gz 월드 좌표계로 나오고 `/initialpose` 는 map
프레임이라 둘 사이의 고정 변환이 필요하다. 처음 한 번, 수렴된 상태에서 Enter 를
누르면 역산해서 `~/.ros/kau_reset_watcher_offset.json` 에 저장하고 다음
실행부터 재사용한다. 저장된 오프셋은 지도 이름을 같이 기록하므로 `pbstream` 이
바뀌면 자동으로 무효가 되고 다시 묻는다.

**드리프트 기록.** gz 실측 대비 추정 오차를 1 Hz 로
`~/physicar_logs/localization_drift_*.csv` 에 남기고 10 초마다 콘솔에 찍는다.
리셋 / 재시작 이벤트도 같은 CSV 에 이벤트 행으로 들어가서 나중에 상관 확인이
된다. `--no-monitor` 로 끈다.

| 인자 | 설명 |
|---|---|
| `--pbstream` | 지도 지정. 생략하면 `maps/` 의 최신 `kau_vN.pbstream` |
| `--no-rviz` | RViz 없이 |
| `--relaunch-on-reset` | 리셋에도 `/initialpose` 대신 통째로 재런치 |
| `--spawn-pose x,y,yaw` | map 프레임 스폰 pose 를 직접 지정 (보정 생략) |
| `--offset x,y,yaw` | map <- gz world 오프셋을 직접 지정 |
| `--recalibrate` | 저장된 오프셋을 무시하고 다시 잡는다 |
| `--no-monitor`, `--csv` | 드리프트 기록 끄기 / 경로 지정 |

## AMCL (Cartographer 대안)

```bash
ros2 launch kau_localization amcl.launch.py \
    map:=kau_v3 start_pose:="0.012,8.979,-2.935" rviz:=true
```

**Cartographer 와 동시에 띄우면 안 된다.** 둘 다 `map -> odom` 을 발행한다.
`cartographer_localization.launch.py` 를 먼저 완전히 내리고 실행할 것.

`.pbstream` 이 아니라 `.yaml` + `.pgm` (사용법 3 의 변환 결과) 을 쓴다.

### 왜 필요한가

대회장에서 관계자가 lane 이탈이나 라바콘 접촉 시 차를 들어 이전 지점으로
되돌린다. Cartographer 는 그 순간 오도메트리와 스캔이 불연속으로 어긋나
pose graph 가 깨지고, 되돌리려면 trajectory 를 갈아끼워야 한다.
AMCL 은 `/initialpose` 를 한 번 쏘면 그 자리에서 파티클을 다시 뿌려 끝난다.
추정 정확도는 Cartographer 가 낫지만, **옮겼을 때 안 깨지는 쪽**이 AMCL 이다.

### 초기 위치

`start_pose:="x,y,yaw"` 를 주면 그 자리에서 시작한다. 생략하면 RViz 의
**2D Pose Estimate**(`/initialpose`) 를 기다린다 — AMCL 이 직접 구독하므로
Cartographer 때와 달리 `initial_pose_relay` 가 필요 없다.

지도 origin 이 `[-3.79, -1.76]` 이고 크기가 7.8 x 12.75 m 라 `(0,0,0)` 은
방 한가운데가 아니다. 아무 것도 안 주면 파티클이 초기 분산 그대로 떠 있고
수렴하지 않는다.

### RViz 에서 볼 것

`rviz/localization.rviz` 는 Cartographer 와 AMCL 이 같이 쓴다. 두 방식을 같은
뷰에서 번갈아 봐야 비교가 되기 때문이다. 안 쓰는 쪽 디스플레이는 기본으로
꺼져 있고, 켜져 있어도 "No messages received" 만 뜬다.

**ParticleCloud (`/particle_cloud`) 가 AMCL 진단의 거의 전부다.**

| 파티클 모양 | 뜻 |
|---|---|
| 좁게 모여 있다 | 정상 |
| 넓게 퍼져 있다 | 아직 수렴 안 됨. `/initialpose` 를 다시 찍는다 |
| **두 덩어리로 갈린다** | **180도 대칭 혼동.** 이 방이 빈 직사각형이라 생기는 것 |
| 회전할 때 확 퍼졌다 모인다 | `alpha1` / `alpha2` 가 너무 크다 |

`/particle_cloud` 는 타입이 `nav2_msgs/ParticleCloud` 라 RViz 기본 플러그인으로는
못 그린다. `nav2_rviz_plugins` 가 있어야 한다 (package.xml 에 넣어뒀다).

`AmclPose` (`/amcl_pose`) 는 공분산 타원을 그려서 불확실성을 정량으로 보여준다.
이 토픽은 필터가 갱신될 때만 나오므로 **정지 중에는 안 온다** — 정상이다.

### map -> odom 이 훨씬 안정적이다

차를 **세워둔 채로** 20 초, `scripts/map_odom_jitter.py` 실측 (2026-08-23, sim):

| | Cartographer | AMCL |
|---|---|---|
| `map -> odom` 발행 | 175 Hz | 9.9 Hz |
| 전체 변동폭 | 4.88 cm / 1.39도 | **1.12 cm / 0.094도** |
| 변화율 평균 | 7.54 cm/s / 0.63도/s | **0.06 cm/s / 0.005도/s** |
| 변화율 최대 | 167 cm/s / 67.8도/s | **11.2 cm/s / 0.94도/s** |
| (같은 구간 차량 실제 움직임) | 1.68 cm / 1.33도 | 1.48 cm / 3.53도 |

`map -> odom` 은 누적 드리프트 보정량이라 천천히만 변해야 한다. Cartographer 는
정지 중에도 차량 자체 노이즈의 3 배로 흔들리는데, RViz 에서 fixed frame 을 map
으로 두면 이게 "지도가 차를 따라 움직이는" 것으로 보인다.

원인은 발행 구조다. Cartographer 는 175 Hz 마다
`map -> odom = map -> tracking(외삽) x (odom -> base)^-1` 을 **다시 계산**하는데,
175 Hz 외삽 결과와 30 Hz EKF TF 가 같은 순간에 일치하지 않아 그 차이가 전부
`map -> odom` 에 실린다. 스캔 보정은 10 Hz 뿐이고 나머지는 외삽 노이즈다.
AMCL 은 갱신 조건(`update_min_d` / `update_min_a`) 을 만족할 때만 보정량을
바꾸고 그 사이엔 같은 값을 재발행하므로 구조적으로 이 문제가 없다.

> 위 수치는 **정지 상태** 비교다. 주행 / 회전 중 정확도는 별개 문제이고
> 아직 측정하지 않았다.

### EKF yaw 드리프트 — 정확도 문제의 근본 원인

**이 절이 이 패키지에서 가장 중요하다.** AMCL 튜닝을 하기 전에 먼저 읽어야 한다.

차를 완전히 세워 둔 채 45 초를 관측하면 이렇다 (2026-08-24, sim):

| 소스 | yaw 드리프트 |
|---|---|
| raw `/imu` gyro z 평균 | **+0.01 deg/min** (사실상 0) |
| raw `/imu` orientation (절대 방위) | **+0.00 deg/min** — 35.676도 에 고정 |
| `/odom/laser` (레이저 오도메트리) | **−0.10 deg/min** (사실상 0) |
| **`/odom` (EKF 출력)** | **+7.90 deg/min** |

입력 세 개가 전부 깨끗한데 EKF 출력만 흐른다. 센서 문제가 아니라 설정 문제다.
`physicar_bringup/config/ekf_params.yaml` 을 보면 원인이 바로 보인다.

```yaml
odom0: /odom/laser
odom0_config: [true,  true,  false,     # x, y
               false, false, false,     # roll, pitch, yaw  <- yaw 를 안 쓴다
               true,  false, false,     # vx
               false, false, false,     # vyaw 도 안 쓴다
               false, false, false]
imu0: /imu
imu0_config: [false, false, false,
              false, false, true,       # yaw
              false, false, false,
              false, false, true,       # vyaw
              false, false, false]
imu0_differential: true                 # <- 절대 yaw 를 차분해서 속도처럼 쓴다
```

`imu0_differential: true` 는 IMU 의 **절대 yaw 를 쓰지 않고** 연속 측정값의 차분을
각속도처럼 넣는다. 그리고 `/odom/laser` 의 yaw 는 `false` 라 아예 버려진다.
결과적으로 **yaw 에 절대 기준이 하나도 없고** 전부 각속도 적분(dead reckoning)이라
흐를 수밖에 없다. 버려지고 있는 IMU 절대 yaw(35.676도) 는 스캔 정합으로 구한
정답(35.5도) 과 0.18 도 안에서 일치한다. 멀쩡한 기준을 안 쓰고 있는 것이다.

고치려면 둘 중 하나다. **다만 `/opt/physicar/src/physicar-ros/` 는 플랫폼 제공
코드라 아직 손대지 않았다.**

- `imu0_differential: false` — 한 줄. sim 에서는 즉시 해결되지만, 실기 IMU 의 절대
  yaw 는 지자기(`/imu/mag`) 에서 나오므로 실내 철골·모터 자기장에 취약하다.
- `odom0_config` 의 yaw / vyaw 를 켜고 `imu0` 은 vyaw 만 쓰기 — 레이저 오도메트리는
  스캔 정합이라 자기장과 무관하고 sim / 실기가 동일하다. robot_localization 표준
  구성(절대 자세는 오도메트리, 고주파 각속도는 IMU)이고 이쪽이 더 안전하다.

이 드리프트가 아래 AMCL 파라미터 선택을 전부 좌우한다.

### 정지 중 강제 갱신(`/request_nomotion_update`) 은 답이 아니다

AMCL 은 `update_min_d` / `update_min_a` 를 넘게 움직여야 필터를 갱신한다. 그래서
"정지 중에는 갱신이 없으니 강제로 돌리면 되겠다" 는 발상으로 `nomotion_updater`
노드를 만들었는데, **실측 결과 처방이 틀렸다.** 정지 3 분 비교:

| 설정 | 스캔 10 cm 이내 | yaw 오차 | 파티클 xy 표준편차 |
|---|---|---|---|
| `update_min_a` 0.02 + 강제 갱신 4 회 | 71.6 % | +2 ~ +4.5 도 | — |
| `update_min_a` 0.02 + 2 Hz 무제한 강제 갱신 | **35.8 %** (100 초에 발산) | +4 도 발산 | 0.7 cm (과신) |
| **`update_min_a` 0.004 + 강제 갱신 없음** | **92 ~ 99 %** | **1.7 도 이내** | 1.1 ~ 1.5 cm |

**자연 갱신과 강제 갱신은 효과가 정반대다.**

- 자연 갱신(문턱 초과로 발생): odom 델타가 0 이 아니므로 모션 모델이
  `alpha1 * |dtheta|` 만큼 노이즈를 주입한다. 파티클 다양성이 유지된다.
- 강제 갱신: 델타가 0 이라 노이즈 주입 없이 리샘플링만 반복한다. 다양성이 단조
  소멸하고, 표준편차가 1 cm 밑으로 떨어지면 필터가 과신 상태가 되어 EKF 드리프트를
  더는 못 따라가고 발산한다.

그리고 위의 EKF 드리프트 덕에 **이 플랫폼은 정지 중에도 odom 이 가만히 있지 않는다.**
그래서 `update_min_a` 만 충분히 낮추면 드리프트 자체가 자연 갱신을 계속 트리거해서
강제 갱신이 아예 필요 없다. `nomotion_updater` 는 기본으로 꺼져 있다
(`nomotion_update:=true` 로 켤 수는 있다). EKF 가 나중에 고쳐져 정지 중 odom 이
진짜로 멈추면 그때 다시 필요해질 수 있어서 노드 자체는 남겨 뒀다.

### 설정에서 손댄 것

`physicar_bringup/config/nav2_params.yaml` 의 amcl 섹션을 기준선으로 두고
이 차량에 맞지 않는 것만 바꿨다. 이유는 `config/amcl.yaml` 에 전부 적어뒀다.

| 항목 | 기준선 | 이 패키지 | 이유 |
|---|---|---|---|
| `laser_min_range` | 0.05 | 0.1 | 라이다 실제 `range_min` |
| `max_beams` | 60 | 120 | 719 beam 중. 빈 직사각형이라 beam 이 적으면 벽을 따라 미끄러진다 |
| `z_hit` / `z_rand` / `sigma_hit` | 0.5 / 0.5 / 0.2 | 0.9 / 0.1 / 0.08 | 기준선은 확률질량의 절반이 균등분포고 `sigma_hit` 이 지도 해상도의 4 배라 우도장이 평평하다. 옳은 자세를 찍어줘도 필터가 거기서 밀려났다 (정지 정확도 89.6 → 96.2 %) |
| `alpha1` / `alpha2` | 0.2 | 0.1 | 회전 오도메트리가 정확한 편이라 낮추되, 0.05 는 EKF 드리프트를 흡수하기에 부족했다 |
| `update_min_d` | 0.15 | 0.01 | 보정을 자주 |
| `update_min_a` | 0.2 rad | 0.004 rad | 0.23 도. **EKF yaw 드리프트 때문이다.** 1.1 도 문턱은 드리프트의 랜덤워크가 잘 못 넘어서 정지 60 초에 갱신이 2 회밖에 안 일어났고 그 사이 +2 ~ +4.5 도 틀어진 채 방치됐다. 위 두 절 참고 |
| `min_particles` | 500 | 300 | 방이 작아 전역 탐색 부담이 적다. KLD 가 필요할 때 늘린다 |
| `set_initial_pose` | true (0,0,0) | launch 인자 | 지도 origin 이 (0,0) 이 아니다 |

**`recovery_alpha_slow/fast` 는 0 으로 끈 채로 둔다.** 켜면 스캔이 안 맞을 때
무작위 파티클을 뿌려 전역 재탐색을 하는데, 12 x 7 m 빈 직사각형은 180도 회전
대칭이라 정반대 자세에 붙어도 스캔상으로는 똑같이 그럴듯하다. 한 번 뒤집히면
스스로 못 돌아온다. 차를 옮겼을 때의 복구는 `/initialpose` 재주입으로 한다.

### 아직 안 된 것

- `scripts/reset_watcher.py` 는 Cartographer 전용이다 (`initial_pose_relay`
  존재를 확인하고 `cartographer_localization.launch.py` 를 띄운다). AMCL 로 장기 검증을
  하려면 launch 대상과 노드 확인을 갈라야 한다.
- 주행 / 회전 중 정확도 비교. 정지 상태 지터만 측정했다.

## 파일

| 경로 | 설명 |
|---|---|
| `config/physicar_2d.lua` | SLAM 설정. 프레임/센서 튜닝의 기준 |
| `config/physicar_2d_localization.lua` | 위 파일을 include + pure localization trimmer |
| `launch/slam.launch.py` | cartographer_node + occupancy_grid_node |
| `launch/cartographer_localization.launch.py` | 위 + initial_pose_relay |
| `launch/amcl.launch.py` | map_server + amcl + lifecycle_manager (Cartographer 와 택일) |
| `config/amcl.yaml` | AMCL 파라미터. 기준선 대비 바꾼 값의 이유를 전부 주석에 남겼다 |
| `src/initial_pose_relay.cpp` | `/initialpose` -> `/start_trajectory` 릴레이 (C++) |
| `rviz/localization.rviz` | Cartographer · AMCL 공용 뷰. map / scan / TF / 파티클 / 공분산 |
| `scripts/save_map.py` | 매핑 종료 후 최적화 + `kau_vN` 자동 버전 저장 + pgm 변환 + 품질 검사 |
| `scripts/check_constraints.py` | 매핑 중 루프 클로저 constraint 길이 분포 확인 (아래 주의 참고) |
| `scripts/view_map.py` | 저장된 지도 확인. ROS 불필요. 검사 + ASCII + 벽 정밀측정 + PNG |
| `scripts/tf_rate.py` | TF 링크별 발행 주기 측정 (벽시계 Hz + sim stamp Hz) |
| `scripts/map_odom_jitter.py` | `map -> odom` 보정량이 얼마나 흔들리는지 측정 |
| `scripts/reset_watcher.py` | 앱의 리셋/재시작 버튼 감지 -> localization 자동 유지 + 드리프트 기록 (사용법 4) |
| `launch/map_arg.py` | launch 파일이 아니다. 지도 이름 -> 경로 해석 공용 헬퍼 |
| `maps/kau_vN.pbstream` | 저장된 KAU 지도. `pbstream:=kau_vN` 의 대상 |
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
