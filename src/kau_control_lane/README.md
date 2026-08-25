# kau_control_lane

KAU AMET PhysiCar 용 **차선 추종 전용 주행 제어** 패키지.
`/lane/center` **하나만** 보고 `/speed`, `/steering` 을 발행한다.

`kau_control` 의 **대안**이다. 알고리즘은 같고 경로 소스만 다르다.
기존 방식으로 언제든 돌아갈 수 있게 `kau_control` 은 그대로 두었고,
이 패키지는 `bringup.yaml` / `run.sh` 를 건드리지 않는다 — 자기 launch
파일 하나로만 뜬다.

| 노드 | 출력 | 알고리즘 |
|---|---|---|
| `lane_speed_controller` | `/speed` [m/s] | 전방 곡률 -> 목표속도, PID |
| `lane_steer_controller` | `/steering` [rad] | Pure Pursuit |

> 상태: **구현 · 기동 확인 완료, 실주행 튜닝 미완.** §6 참고.
>
> 최종 갱신: 2026-08-25

## 1. 무엇이 다른가

```text
 kau_control                          kau_control_lane
 -----------                          ----------------
 /path/local   (1순위)                /lane/center   (유일)
 /path/global  (2순위)                       │
 /lane/center  (3순위)                       │
        │                                    │
        ▼                                    ▼
  path_tracker.hpp                     lane_tracker.hpp
  (mode 프리셋으로 순서 선택)           (프리셋 없음. 소스 하나)
        │                                    │
        ▼                                    ▼
  speed_controller                     lane_speed_controller
  steer_controller                     lane_steer_controller
        │                                    │
        ▼                                    ▼
  /speed, /steering                    /speed, /steering
```

**제어 알고리즘은 같다.** Pure Pursuit, 곡률 기반 목표속도 + PID,
8.3 전역 최근접점 / 8.4 국소 window / 8.5 LookAhead, 출발 게이트,
안전 정지 규칙까지 `kau_control` 에서 그대로 가져왔다 (§2).

다른 것은 셋뿐이다.

1. **경로 소스가 차선 하나다.** local / global fallback 이 없다.
   차선을 놓치면 다른 경로로 내려가지 않고 **선다.**
2. **측위를 안 봐도 된다.** `lane.pose_source: identity` 면 경로가 이미
   차량 프레임이라 TF 를 보지 않는다 (기본값). Cartographer / AMCL 없이
   카메라만으로 돈다.
3. **종점 판정이 없다.** 차선 경로는 차량과 같이 움직이는 rolling horizon
   이라 종점이 "앞으로 본 데까지" 일 뿐이다. 도착으로 처리하면 매번 멈춘다.

## 2. 출처 — `kau_control` 의 사본이다

알고리즘 헤더는 `kau_control/include/kau_control/` 을 **그대로 복사**했다.
namespace 만 `kau::control_lane` 으로 갈랐다 (두 패키지를 함께 링크할 일이
없어 사본이 부딪히지 않는다).

| kau_control | kau_control_lane | 관계 |
|---|---|---|
| `bezier.hpp` `curve.hpp` `kau_path.hpp` `params.hpp` `pid.hpp` `pure_pursuit.hpp` | 같은 이름 | **그대로 복사** |
| `path_tracker.hpp` | `lane_tracker.hpp` | 소스 하나로 축약 (§1) |
| `src/speed_controller_node.cpp` | `src/lane_speed_controller_node.cpp` | 경로 소스·노드 이름만 다름 |
| `src/steer_controller_node.cpp` | `src/lane_steer_controller_node.cpp` | 위와 같음 |
| `test/test_curve.cpp` | 같은 이름 | **그대로 복사** (16/16 통과) |

> 알고리즘을 고칠 일이 생기면 **두 패키지를 같이 고쳐야 한다.** 이 중복은
> "기존 방식을 언제든 되돌릴 수 있게" 라는 요구를 위해 의도적으로 진 빚이다.
> 어느 한쪽으로 정리되면 공통부를 `kau_bezier` / `kau_control_core` 로 빼는
> 것이 맞다 (`kau_control/README.md` §7 의 같은 항목과 함께).

## 3. 사용법

```bash
colcon build --symlink-install --packages-select kau_control_lane
source install/setup.bash
```

★ **`kau_control` 의 두 제어기와 같은 topic(`/speed`, `/steering`) 을 낸다.**
동시에 띄우면 두 발행자가 싸운다. 이 패키지를 쓰는 동안 `run.sh` 의
`KAU_NODES` 표에서 `steer_controller` / `speed_controller` 를 `false` 로
내릴 것. (`bringup.yaml` 에는 이 패키지를 넣지 않았다 — 기존 스택을
건드리지 않는 것이 이 패키지의 전제다)

```bash
# 조향만 관찰 (차는 안 움직인다). 처음엔 반드시 이걸로 시작할 것
ros2 launch kau_control_lane control_lane.launch.py speed:=0.0

# 인지까지 같이 띄우고 상수 속도로 주행
ros2 launch kau_control_lane control_lane.launch.py lane_detection:=true speed:=0.4

# 신호등 없이 곧바로 출발 (주행부만 시험할 때. 대회에서는 쓰지 말 것)
ros2 launch kau_control_lane control_lane.launch.py start_gate:=false

# s / cte / Ld / steer 를 2 Hz 로 본다
ros2 launch kau_control_lane control_lane.launch.py log_level:=debug
```

| 인자 | 기본 | 뜻 |
|---|---|---|
| `speed` | (yaml) | 상수 속도 [m/s]. `v_min = v_max` 로 덮는다. `0.0` 이면 조향만 |
| `pose_source` | (yaml) | `identity` \| `tf`. §4 |
| `start_gate` | (yaml) | `false` 면 신호등 허가 없이 출발 |
| `lane_detection` | `false` | 차선 인지도 같이 띄울지 |
| `viewer` | `true` | 웹 뷰어(포트 5000). `lane_detection:=true` 일 때만 |
| `use_sim_time` | `true` | 실기는 `false` |

한쪽만 띄우고 싶으면:

```bash
ros2 run kau_control_lane lane_steer_controller_node --ros-args \
    --params-file src/kau_control_lane/config/lane_source.yaml \
    --params-file src/kau_control_lane/config/lane_steer_controller.yaml
```

### 기존 방식으로 되돌리기

이 패키지를 끄면 끝이다. `kau_control` 은 아무것도 안 바뀌었다.

```bash
# 이 launch 를 Ctrl-C 로 내리고
source run.sh          # 기존 스택 (steer_controller / speed_controller)
```

## 4. 좌표계 — `lane.pose_source` 와 `path_frame_id` 는 짝이다

`config/lane_source.yaml` 의 **한 줄**이 정한다.

| `lane.pose_source` | `kau_lane_detection` 의 `path_frame_id` | 측위 |
|---|---|---|
| `identity` (**기본**) | `"base_link"` | **불필요** |
| `tf` | `"map"` (그쪽 기본값) | 필요 (`map -> base_frame`) |

`identity` 는 차량을 경로 프레임의 원점으로 잡는다 (후륜축만
`rear_axle_offset_cm` 만큼 뒤). 측위 오차가 결과에 섞이지 않는 것이
이 조합의 장점이고, "차선만 보고 제어" 의 원래 취지이기도 하다.

> **주의**: `kau_lane_detection/config/lane_detection.yaml` 의
> `path_frame_id` 는 지금 `"map"` 이다. 기본값인 `identity` 로 쓰려면
> 그 한 줄을 `"base_link"` 로 바꿔야 한다. 반대로 그 파일을 건드리지
> 않으려면 `pose_source:=tf` 로 띄운다.
>
> 짝이 안 맞으면 경로가 전부 버려지고 차가 안 움직인다. 그때 두 노드가
> 2 초마다 ERROR 로 **어디를 고치라고** 찍는다 — 조용히 서 있지 않는다.
>
> `path_frame_id` 를 `base_link` 로 바꾸면 `kau_control` 의 lane fallback
> (3순위) 이 map 을 기대하므로 그쪽이 못 쓰게 된다. 되돌릴 때 같이 되돌릴 것.

## 5. 인터페이스

| | topic | 타입 | 비고 |
|---|---|---|---|
| 입력 | `/lane/center` | `kau_msgs/KauPath` | **유일한 경로 입력.** RELIABLE, depth 1 |
| 입력 | TF `map -> base_frame` | | `pose_source: tf` 일 때만 |
| 입력 | `/odometry/filtered` | `nav_msgs/Odometry` | speed 만. PID 피드백. **아직 발행자 없음** |
| 입력 | `/perception/start_permission` | `std_msgs/Bool` | 출발 게이트. speed 만 |
| 입력 | `/speed` | `std_msgs/Float64` | steer 가 `Ld` 계산에 쓴다 |
| 출력 | `/speed` | `std_msgs/Float64` | `lane_speed_controller`, m/s |
| 출력 | `/steering` | `std_msgs/Float64` | `lane_steer_controller`, **rad** |
| 출력 | `/viz/path/tracked` | `nav_msgs/Path` | RViz 표시용 |
| 출력 | `/viz/path/lookahead` | `geometry_msgs/PointStamped` | LookAhead 점 |
| 출력 | `/debug/steer` | `kau_msgs/SteerDebug` | kau_gui 용. BEST_EFFORT |

topic 이름은 `kau_control` 과 **같다.** GUI · driver · state machine 이 그대로
붙는 것이 목적이고, 그래서 두 스택을 동시에 띄우면 안 된다 (§3).

### 안전 규칙 (`kau_control` 과 동일)

어떤 경로로 빠져나가든 **0 을 계속 발행한다.** driver 의 `cmd_timeout` 이
1 초라 조용히 멈추면 그동안 직전 명령으로 계속 굴러가기 때문이다.

정지 사유는 넷이다.

| 사유 | 조건 |
|---|---|
| 차선 경로 없음 | 한 번도 못 받았다 |
| 차선 경로가 N s 갱신되지 않았다 | `lane.timeout` (기본 0.5 s = 최대 7 프레임 연속 실패) |
| cross track error 발산 | \|cte\| > `cte_abort_cm` (기본 40 cm = 도로 반폭 31.25 + 여유) |
| 관측 구간이 짧아 LookAhead 를 둘 곳이 없다 | `valid_length - s - 5 cm < ld_min` (steer 만) |

## 6. 검증 현황

### 검증된 것 (2026-08-25)

`colcon test --packages-select kau_control_lane` — 알고리즘 단위 테스트
**16/16 통과** (`kau_control` 과 같은 기준값 대조). 남는 실패는 launch
docstring 스타일(`D213`) 2 건뿐이며 `kau_control` 도 같은 상태다.

기동 · 배선 확인:

- 두 노드 기동, parameter 반영 (`v=0.40~0.40`, `Ld=25~80`, `identity/base_link`)
- launch 인자 `speed:=` · `pose_source:=` · `start_gate:=` 가 실제로 먹는 것 확인
- 경로 없음 -> 두 노드 모두 0 발행 + 2 초마다 사유 WARN
- `fake_path.py` 로 `/lane/center` 를 물려 추종 확인
  - 직선 경로를 왼쪽 20 cm 로 옮김 -> `cte=-20.0 cm`, `steer=+20.0 deg`
    (좌회전 포화). 부호와 포화 모두 기대대로
- 프레임 짝이 어긋난 경우 -> 경로를 버리고 **고칠 곳을 지목하는** ERROR
- `lane.pose_source` 오타 -> FATAL 후 종료 코드 1 (supervisor 사망 판정 오염 방지)

### 검증되지 **않은** 것

- **실주행 추종 정확도.** 실차/시뮬 주행으로 `k_v` / `ld_min` / `ld_max` 재튜닝 필요
- **곡률 기반 가감속.** 차선 경로는 앞 80 cm 짜리 토막이라 코너를 미리 못 본다.
  기본값을 상수 속도(`v_min == v_max == 0.4`)로 둔 이유다. 먼저 상수 속도로
  조향을 확인하고, 그다음 `v_min` 을 내려(예: 0.3 / 0.5) 켠다
- **속도 PID.** 오도메트리 발행자가 아직 없어 게인이 실제로 쓰이지 않는다

## 7. 해야 할 일

- [ ] `speed:=0.0` 으로 조향만 관찰 -> 상수 속도 -> 곡률 가감속 순으로 실주행 시험
- [ ] `k_v` / `ld_min` / `ld_max` 재튜닝 (탐색 범위 `k_v` 0.3~1.0 s / `ld_min` 25~40 cm)
- [ ] `identity` 로 갈 거면 `kau_lane_detection` 의 `path_frame_id` 를 `base_link` 로
      (되돌릴 때 같이 되돌릴 것 — §4)
- [ ] 오도메트리 노드가 나오면 `speed.feedback_topic` 연결 후 `require_feedback: true`
- [ ] 어느 방식으로 갈지 정해지면 `kau_control` 과의 헤더 중복 정리 (§2)

## 8. 파일

| 경로 | 설명 |
|---|---|
| `include/kau_control_lane/lane_tracker.hpp` | **두 노드 공통부.** 차선 경로 수신 · pose · 최근접점 추적 |
| `include/kau_control_lane/curve.hpp` 외 5 | `kau_control` 사본. 알고리즘 (§2) |
| `src/lane_speed_controller_node.cpp` | 속도 노드. 곡률 -> 목표속도 -> PID -> `/speed` |
| `src/lane_steer_controller_node.cpp` | 조향 노드. Pure Pursuit -> `/steering` |
| `config/lane_source.yaml` | **경로 입력 (두 노드 공용).** `lane.pose_source` 가 여기 |
| `config/lane_speed_controller.yaml` | 속도 튜닝값 |
| `config/lane_steer_controller.yaml` | 조향 튜닝값 |
| `launch/control_lane.launch.py` | 두 노드 기동 (+ 선택적으로 차선 인지) |
| `test/test_curve.cpp` | 레퍼런스 대조 단위 테스트 (`kau_control` 사본) |

### yaml 섹션이 `/**` 인 이유

세 config 모두 노드 이름이 아니라 `/**` 절을 쓴다. ROS 2 는 **노드 이름
섹션이 `/**` 섹션을 항상 이기기** 때문이다 (파일 순서와 무관). 노드 이름으로
적으면 `speed:=0.4` 같은 launch 인자나 `-p speed.v_max:=...` CLI 오버라이드가
**조용히 씹힌다** — 둘 다 `/**` 로 들어간다. 각 파일은 해당 노드에만
넘기므로 `/**` 라도 남의 노드에 새지 않는다.
