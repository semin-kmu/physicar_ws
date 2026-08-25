# 백업 스택 구현 규약

모든 `backup_*` 패키지 구현자가 **반드시** 지킨다. 설계 근거는 `../Backup_Full_Stack_Design.md`.

---

## 0. 환경

| 항목 | 값 |
|---|---|
| OS | Ubuntu 24.04 |
| ROS | **ROS 2 Jazzy** |
| C++ | 17 |
| OpenCV | 시스템 (`libopencv-dev`) |

**빌드 검증은 사람이 따로 한다.** 컴파일해 볼 수 없으므로 헤더 이름·시그니처를 추측하지 말고 확실한 것만 쓴다.

---

## 1. Jazzy 함정 — 여기서 틀리면 시뮬이 깨진다

### 타이머는 반드시 clock-aware

```cpp
// 금지 -- use_sim_time 을 무시하고 벽시계로 돈다
timer_ = create_wall_timer(...);

// 필수
timer_ = rclcpp::create_timer(
  this, this->get_clock(),
  rclcpp::Duration::from_seconds(1.0 / control_hz_),
  std::bind(&Node::onTimer, this));
```

`use_sim_time:=true` 에서 Gazebo 실시간 배율이 1.0 이 아니면 `create_wall_timer` 는 제어 tick 만 벽시계로 돌린다. 센서 stamp 는 `/clock` 을 따르므로 데드레커닝 `dt` 가 통째로 어긋난다.

### 시각은 `this->now()`

`std::chrono::steady_clock` / `system_clock` 을 쓰지 않는다. `dt` 는 항상 `(this->now() - prev).seconds()`.

`use_sim_time` 이 켜진 직후에는 `/clock` 이 오기 전까지 `now()` 가 0 이다. **첫 tick 의 `dt` 는 버린다** (0 또는 음수 가드).

### 헤더

```cpp
#include <cv_bridge/cv_bridge.hpp>          // Jazzy 는 .hpp (.h 아님)
#include "backup_msgs/msg/lane_geometry.hpp"  // snake_case 파일명
#include "backup_msgs/msg/backup_path.hpp"
#include "backup_msgs/msg/obstacle_circle_array.hpp"
#include "backup_msgs/msg/control_debug.hpp"
```

---

## 2. 좌표·단위 — 예외 없음

| 항목 | 규약 |
|---|---|
| 길이 | **m** (cm 금지) |
| 각 | **rad**. yaml 입력만 deg 를 허용하고 코드는 즉시 rad 로 바꾼다 |
| 종방향 원점 | **후륜축**. 카메라 접지점보다 0.195 m 뒤 |
| 횡방향 | **좌측 +** |
| frame_id | 전부 `base_link` |

**TF 조회 금지.** `tf2_ros` 를 쓰지 않는다. 백업 스택은 측위 비의존이 전제다. `lidar_link -> base_link` 같은 변환은 yaml 상수로 처리한다.

---

## 3. 파라미터

- **모든 튜닝값은 `config/*.yaml` 에 이미 있다.** 이름을 정확히 그대로 `declare_parameter` 한다.
- yaml 에 없는 값을 코드에 리터럴로 박지 않는다. 필요하면 yaml 에 추가하고 주석 한 줄로 근거를 남긴다.
- `declare_parameter` 기본값은 yaml 값과 일치시킨다.
- yaml 최상단이 `/**:` 와일드카드이므로 노드 이름에 의존하지 않는다.

---

## 4. 공용 헤더 — `backup_common`

이미 작성·검증돼 있다. 다시 만들지 말고 그대로 쓴다.

```cpp
#include <backup_common/vehicle.hpp>   // VehicleParams, kDeg2Rad, wrapAngle, clampAbs
#include <backup_common/motion.hpp>    // Pose2, integrateBicycle, transformPoint, deadReckonPoint
#include <backup_common/bezier.hpp>    // Point2, Segment, Path, purePursuitSteer
```

검증된 값 — `VehicleParams{}.rMin() = 0.4945`, `kappaMax() = 2.0221`, `sweptWidth(0.296) = 0.267`.
`Segment` 의 `P0,P1,P2` 공선 + `P3,P4,P5` 공선이면 끝 곡률이 정확히 0 이다.

---

## 5. 로그

```cpp
RCLCPP_INFO(get_logger(), "[기동][lane] ROI 행 %d~%d, d %.2f~%.2f m", ...);
RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "[검출][lane] 양쪽 차선 미검출");
```

| 규칙 |
|---|
| **`[단계][노드]` 대괄호 접두어**로 시작 |
| **이모지 금지** |
| 주기 로그는 반드시 `_THROTTLE` (yaml 의 `log_period_s`) |
| 기동 시 1회: 확정 파라미터 요약 (ROI, 밴드, 주기) |

---

## 6. 주석

프로젝트 `CLAUDE.md` 규약이다.

| 규칙 |
|---|
| **줄글 금지** |
| 간략·명확하게 |
| 필요한 것만 최소한으로 |
| "무엇을" 이 아니라 **"왜 그 값/그 방식인지"** 만 남긴다 |

---

## 7. 노드 주기

| 노드 | 구동 |
|---|---|
| `backup_lane_detector` | **이미지 콜백** (타이머 없음) |
| `backup_obstacle_detector` | **scan 콜백** (타이머 없음) |
| `backup_start_signal_detector` | 10 Hz 타이머 + latch 후 구독 해제 |
| `backup_path_planner` | **lane 콜백** (`timer_hz: 0.0` 이면 타이머 없음) |
| `backup_speed_controller` | 50 Hz 타이머 |
| `backup_steer_controller` | 50 Hz 타이머 |

콜백 구동 노드는 입력이 끊기면 발행도 끊긴다. **이게 의도다** — 하류의 timeout 게이트가 고장을 잡는다. 낡은 값을 재발행하지 않는다.

---

## 8. QoS

```cpp
// 센서 입력 (카메라, LiDAR)
rclcpp::SensorDataQoS()

// 내부 토픽 · 차량 출력
rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile()
```

---

## 9. 파일 배치 — CMakeLists 가 이 경로를 기대한다

```
backup_lane_detection/    src/main.cpp  src/lane_detector_node.cpp
                          include/backup_lane_detection/*.hpp
backup_object_detection/  src/main_obstacle.cpp  src/obstacle_detector_node.cpp
                          src/main_start_signal.cpp  src/start_signal_node.cpp
                          include/backup_object_detection/*.hpp
backup_path_planner/      src/main.cpp  src/path_planner_node.cpp
                          include/backup_path_planner/*.hpp
backup_speed_controller/  src/main.cpp  src/speed_controller_node.cpp
                          include/backup_speed_controller/*.hpp
backup_steer_controller/  src/main.cpp  src/steer_controller_node.cpp
                          include/backup_steer_controller/*.hpp
```

`main.cpp` 는 `rclcpp::init` / `spin` / `shutdown` 만 한다. 노드 클래스는 별도 파일.

---

## 10. git 조작 금지

**파일 작성/수정만 한다.** 아래는 전부 금지다.

| 금지 |
|---|
| `git commit` / `git push` / `git rebase` / `git reset` / `git checkout` / 브랜치 생성·전환 |
| `gh` CLI 로 PR·이슈를 만드는 것 |
| `.gitignore`, `.git/config` 수정 |

커밋과 푸시는 **사람이 판단할 일**이다. 작업이 끝나면 무엇을 바꿨는지만 보고하고 손을 뗀다.
무관한 작업 트리 변경이 함께 묶이거나, 검증 전 코드가 원격에 올라가는 사고를 막기 위함이다.

커밋이 필요하다고 판단되면 **커밋하지 말고 그렇게 보고만 해라.**

> 사람이 커밋할 때도 `Co-Authored-By` / `Claude-Session` 등 **Claude 표기는 일절 넣지 않는다.**

---

## 11. 안전 불변식

| 규칙 |
|---|
| `/speed` 는 확신이 없으면 **0**. 입력이 끊기거나 permission 이 없으면 0 |
| `/steering` 은 항상 `±20°` 로 clamp |
| 종료 시 `/speed` 0 을 20 ms 간격 3회 발행 후 200 ms 대기 (1 회 발행 직후 종료하면 DDS 가 전송을 못 끝낸다) |
| 예외를 콜백 밖으로 던지지 않는다. 콜백은 catch 후 WARN 로그 |
| `/speed` 와 `/steering` 은 백업 스택만 발행한다 (기존 `kau_*` 와 동시 실행 금지) |
