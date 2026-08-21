# kau_control

KAU AMET PhysiCar 용 **경로 추종 제어** 패키지. `KauPath` 를 받아 `/speed`, `/steering` 을 발행한다.

> 상태: **포팅 완료, 실주행 검증 미완.** 아래 "검증 현황" 참고.
>
> 최종 갱신: 2026-08-20

## 1. 무엇인가

```text
 /path/local  (kau_msgs/KauPath, 2~10 Hz)
 /path/global (없으면 fallback)
        │
        ▼
  ┌─────────────────────────────────────────┐
  │ 8.3 전역 최근접점 (첫 tick / 복구)       │
  │ 8.4 국소 window 추적 (매 tick)          │
  │ 8.5 LookAhead (호길이 기준)              │
  │ 8.6 구간 max|kappa| -> 목표속도          │
  │     Pure Pursuit -> 조향각               │
  └─────────────────────────────────────────┘
        │                    ▲
        ▼                    │ TF map -> base_footprint
 /speed [m/s]  /steering [rad]
```

이 패키지는 **경로를 만들지 않는다.** 경로 생성은 `kau_global_path` (global) 와
local path 담당이 소유한다. 여기는 주어진 경로를 따라가기만 한다.

## 2. 출처 — 백지 설계가 아니다

`KAU_AMET_Test` 저장소의 **시뮬 검증본을 ROS 노드로 포팅**한 것이다.
알고리즘과 튜닝값은 거기서 이미 검증됐다.

| 원본 | 이 패키지 | 비고 |
|---|---|---|
| `sim_common/curve.py` 의 `Curve` / `TrackState` | `include/kau_control/curve.hpp` | 8.3 / 8.4 / 8.5 / 8.6 |
| `kau_controller/test/controller.py` | `include/kau_control/pure_pursuit.hpp` | 순수 함수 그대로 |
| `sim_common/config.py` | `include/kau_control/params.hpp` + `config/path_follower.yaml` | 튜닝값은 YAML |
| `kau_controller/test/runner.py` 의 `while` 루프 | `src/path_follower_node.cpp` 의 timer callback | 제어 loop |
| `sim_common/vehicle.py` (Kinematic Bicycle + 액추에이터) | **옮기지 않음** | 실차/Gazebo 가 그 역할 |
| `viewer*.py` / `plots.py` / `scenarios.py` / `noise` | **옮기지 않음** | 시뮬 평가 전용 |

segment **1개** 연산(8.1 최근접점 / 8.5 호길이 / 8.6 곡률 / 8.7 퇴화)은
`kau_lane_detection/include/kau_lane_detection/bezier.hpp` 에 이미 C++ 로 있어
`include/kau_control/bezier.hpp` 로 **복사**했다. 제어기가 lane detection 에
의존하는 건 방향이 거꾸로이므로 의도적 중복이다. 원본 헤더 주석대로
전역 탐색(8.3)과 window 추적(8.4)은 **소비자 기능**이라 `curve.hpp` 가 새로 담았다.

> 나중에 `kau_bezier` 공용 패키지로 빼는 게 맞다. lane detection 담당자와 합의 필요.

## 3. 사용법

```bash
colcon build --symlink-install --packages-select kau_control
source install/setup.bash

ros2 launch kau_control path_follower.launch.py
ros2 launch kau_control path_follower.launch.py use_sim_time:=false   # 실기
ros2 launch kau_control path_follower.launch.py log_level:=debug      # s/cte/Ld/steer 2 Hz 출력
```

`physicar_bringup` 의 `sim.launch.py` / `real.launch.py` 와
`kau_localization` 의 `localization.launch.py` 가 떠 있어야 한다.
**`map -> odom` TF 가 없으면 이 노드는 계속 정지 상태로 남는다** (의도된 동작).

### Lane Detection 으로 제어기만 검증하기 (권장 · 측위 불필요)

`kau_lane_detection` 이 `/lane/center` 를 **`base_link` 프레임**으로,
`s_offset = 0` 으로 발행한다 (localization 비의존). 즉 Cartographer 없이
제어기만 따로 검증할 수 있고, 측위 오차가 결과에 섞이지 않는다.

`pose_source:=identity` 로 두면 TF 를 보지 않고 차량을 경로 프레임의 원점으로
잡는다 (후륜축만 `rear_axle_offset` 만큼 뒤).

```bash
# 터미널 1
ros2 launch kau_lane_detection lane_detection.launch.py viewer:=false

# 터미널 2 — 속도 0, 조향만 관찰
ros2 run kau_control path_follower_node --ros-args \
    -p use_sim_time:=true \
    -p pose_source:=identity -p base_frame:=base_link \
    -p path_topic:=/lane/center \
    -p speed.enabled:=false -p speed.constant:=0.0 \
    --log-level path_follower:=debug
```

**교차 검증**: `lane_detection` 은 자체 슬라이딩 윈도우로 구한 `cte` / `yaw` 를
로그에 찍고, `path_follower` 는 같은 값을 Bezier 제어점에서 독립적으로 구한다.
**둘이 일치하면 제어기 계산이 맞는 것이다.** 2026-08-21 실측:

```text
path_follower  : s=0.0 cte=1.4 cm head_err=-0.3 deg Ld=30 steer=-1.1 deg
lane_detection : path 79cm        cte +1.5cm       yaw   -0.4deg
```

주행까지 보려면 `-p speed.enabled:=false -p speed.constant:=0.4` 로 올린다.
`/lane/center` 는 `valid_length` 가 80 cm 안팎이라 `ld` 가 거기에 맞춰 잘리고,
관측 구간이 `ld_min` 보다 짧아지면 자동 정지한다.

### 경로 발행자가 아직 없을 때

`kau_global_path` / local path 가 구현 전이므로 테스트용 발행자를 같이 넣었다.
**경로 발행자가 생기면 이 스크립트는 버린다.**

```bash
ros2 run kau_control fake_path.py --shape straight --length 500     # 시나리오 1
ros2 run kau_control fake_path.py --shape curve --radius 150        # 시나리오 2
ros2 run kau_control fake_path.py --shape circle --radius 200       # 폐곡선 (window wrap)

# 차량 현재 위치에 맞춰 배치 (cm, deg)
ros2 run kau_control fake_path.py --shape straight --origin "250,360" --yaw 82
```

## 4. 인터페이스

| | topic | 타입 | 비고 |
|---|---|---|---|
| 입력 | `/path/local` | `kau_msgs/KauPath` | RELIABLE, depth 1 |
| 입력 | `/path/global` | `kau_msgs/KauPath` | 주 경로 끊기면 fallback |
| 입력 | TF `map -> base_footprint` | | 이 노드는 TF 를 **발행하지 않는다** |
| 출력 | `/speed` | `std_msgs/Float64` | m/s |
| 출력 | `/steering` | `std_msgs/Float64` | **rad** |
| 출력 | `/viz/path/tracked` | `nav_msgs/Path` | RViz 표시용 |
| 출력 | `/viz/path/lookahead` | `geometry_msgs/PointStamped` | LookAhead 점 |

## 5. 포팅하면서 걸린 것 — 다음 사람이 다시 밟지 않도록

### 5.1 단위가 세 군데서 다르다

| | 단위 |
|---|---|
| 레퍼런스 코드 · `KauPath` · 이 패키지 내부 | **cm / deg / (1/cm)** |
| ROS (TF, `/odom`, `/speed`) | **m** |
| `/steering` | **rad** |

변환은 **pose 수신 직후와 명령 발행 직전, 두 곳에서만** 한다. 중간에 섞으면 반드시 틀어진다.

### 5.2 Pure Pursuit 기준점 ≠ `base_footprint`

Pure Pursuit 은 **후륜축 중심** 기준인데 PhysiCar URDF 는 `base_link`/`base_footprint` 를
**휠베이스 중앙**에 둔다 (뒷바퀴 `x=-0.09`, 앞바퀴 `x=+0.09`).
보정 없이 쓰면 9 cm 앞을 후륜축으로 착각해 **코너에서 계속 안쪽으로 파고든다.**

`vehicle.rear_axle_offset_cm: -9.0` 이 그 보정이며, 단위 테스트가
`-wheelbase/2` 인지 검사한다. 차량이 바뀌면 URDF 를 다시 보고 고칠 것.

### 5.3 `/speed` 워치독 — 조용히 return 하면 안 된다

driver 의 `cmd_timeout` 이 1 초라, 갱신을 멈추면 **1 초간 직전 속도로 계속 굴러간다.**
그래서 어떤 경로로 빠져나가든(`경로 없음` / `TF 끊김` / `발산` / `도착`)
반드시 `stop()` 으로 0 을 발행한다. 100 Hz timer 가 그 갱신도 겸한다.

### 5.4 `tf2::getYaw` 링크 에러

`tf2/utils.h` 만 include 하면 `fromMsg(Quaternion)` 이 **선언만** 보여서 링크 단계에서 터진다.
`tf2_geometry_msgs/tf2_geometry_msgs.hpp` 를 같이 include 하고
`tf2_geometry_msgs::tf2_geometry_msgs` 를 명시적으로 link 해야 한다
(`ament_target_dependencies` 만으로는 안 된다).

### 5.5 `dt` 를 고정값으로 쓰면 안 된다

시뮬은 `dt = 1/100` 고정이지만 ROS timer 는 밀린다. 속도 EMA 처럼 dt 에 의존하는
계산은 **실제 경과 시간**을 써야 한다.

### 5.6 폐곡선은 segment 3개 이상

`nseg = 2` 면 segment 1개가 전장의 절반이라 최소표현 `delta_s` 가 뒤집혀
window 추적이 무너진다 (레퍼런스 실측: 전역 탐색 대비 불일치 29%).
`windowSafe()` 가 검사해 경고한다.

## 6. 검증 현황

### 검증된 것

`colcon test --packages-select kau_control` — **16/16 통과**.

레퍼런스 `sim_common/curve.py` 를 반경 150 cm 사분원호(3 segment)에 대해 실행한
결과와 **1e-6 이내로 일치**함을 확인했다. 기준값 재생성 방법은
`test/test_curve.cpp` 상단 주석에 있다.

| 항목 | 결과 |
|---|---|
| 호길이 / segment 길이 / max\|kappa\| | 일치 |
| 8.3 전역 최근접점 (seg, u, s, dist) | 일치 |
| 좌표 / heading / kappa | 일치 |
| cross track error, heading error | 일치 |
| 8.5 LookAhead (개곡선 clamp, 폐곡선 wrap) | 일치 |
| 8.6 구간 max\|kappa\| | 일치 |
| 8.4 window 추적 = 전역 탐색 | 경로 전 구간에서 일치 |
| GATE 이탈 3회 -> 전역 재탐색 복구 | 동작 |
| G2 연속성 (이음매 theta/kappa) | 오차 0 |

`kau_lane_detection` 과 붙여 **독립 구현 교차 검증** (2026-08-21):
자체 슬라이딩 윈도우가 구한 `cte` / `yaw` 와 이 패키지가 Bezier 제어점에서
구한 값이 소수점까지 일치했다 (위 §3 참고). 서로 다른 구현이 같은 답을 냈다.

시뮬 기동 확인:

- 정확히 **100 Hz** 로 `/speed`, `/steering` 발행 (실측 99.998 Hz)
- `KauPath` 수신 -> 최근접점 -> cte/heading error -> Pure Pursuit -> 곡률 기반 속도 전 단계 동작
- 안전 정지 4종(`경로 없음` / `TF 지연` / `cte 발산` / `종점 도달`) 모두 실제로 발화
  (특히 `use_sim_time` 불일치로 TF 가 늦었을 때 **주행을 거부**하는 것을 확인)
- 30 cm 오프셋 -> 약 5 cm 로 수렴, 진동 없음

### 검증되지 **않은** 것

**실제 추종 정확도는 아직 판정할 수 없다.** 시뮬에서 차량의 TF 헤딩과 실제
진행 방향이 약 50 deg 어긋나 있어(Gazebo 참값에서도 동일하게 관측) 정상상태
오차를 신뢰할 수 없다. `odom` 기준으로 돌렸기 때문이며, **Cartographer 를 띄워
`map` 프레임에서 다시 측정해야 한다.**

이건 이 패키지의 문제가 아니라 측위/시뮬 환경 조건이지만, **원인을 규명하기
전에는 튜닝값을 건드리지 말 것.** 잘못된 기준으로 튜닝하면 실기에서 깨진다.

## 7. 해야 할 일

- [ ] `localization.launch.py` 를 띄우고 `map` 프레임에서 추종 정확도 재측정
- [ ] 시뮬 헤딩/진행방향 50 deg 불일치 원인 규명 (측위인지 차량 모델인지)
- [ ] 가이드의 시나리오 1(오프셋 복귀) / 2(단일 커브) / 3(S-curve) 실측,
      `k_v` / `ld_min` / `ld_max` 재튜닝. 시뮬 탐색 범위는
      `k_v` 0.3~1.0 s / `ld_min` 30~60 cm / `ld_max` 150 cm
- [ ] `kau_global_path` 또는 local path 가 나오면 `fake_path.py` 제거
- [ ] `confidence` 낮은 경로(lane 기반)에서 감속하는 처리
- [ ] `bezier.hpp` 를 `kau_bezier` 공용 패키지로 분리 (lane detection 담당자와 합의)
- [ ] 실기 조향 중립 캘리브레이션 확인 (`driver_params.yaml` 의 `steering_center`)

## 8. 파일

| 경로 | 설명 |
|---|---|
| `include/kau_control/bezier.hpp` | segment 1개 Bezier 연산. lane detection 에서 복사 |
| `include/kau_control/curve.hpp` | segment 열 = 경로. 8.3 / 8.4 / 8.5 / 8.6 |
| `include/kau_control/pure_pursuit.hpp` | Pure Pursuit 순수 함수 |
| `include/kau_control/params.hpp` | 차량 제원 / 제어기 / 속도 제어 parameter |
| `include/kau_control/kau_path.hpp` | `KauPath` <-> `Curve` 변환 + 무결성 검사 |
| `src/path_follower_node.cpp` | ROS 노드. 제어 loop |
| `config/path_follower.yaml` | 튜닝값 |
| `launch/path_follower.launch.py` | 노드 기동 |
| `scripts/fake_path.py` | 테스트용 경로 발행자 (**임시**) |
| `test/test_curve.cpp` | 레퍼런스 대조 단위 테스트 |
