# kau_control_drive

`/path/drive` 하나만 보는 주행 제어. `kau_path_arbiter` 가 장애물 유무에
따라 차선(`/lane/center`)과 회피 경로(`/path/local`) 중 골라 낸 토픽이다.

```
  /path/drive ──> drive_steer_controller ──> /steering   (Pure Pursuit + 슬루 제한)
              └─> drive_speed_controller ──> /speed      (곡률 기반 목표속도)
```

★ `kau_control` · `kau_control_lane` 과 **같은 topic** 을 낸다. 동시에 띄우면
발행자가 셋이 싸운다. 이걸 쓰는 동안 나머지 둘은 내릴 것.

## kau_control_lane 과 무엇이 다른가

알고리즘(Pure Pursuit, 곡률 기반 속도)은 **같다.** 다른 것 넷:

### 1. 곡선 수학을 복사하지 않는다

`kau_control_lane` 은 `bezier.hpp` / `curve.hpp` / `kau_path.hpp` 를
`kau_control` 에서 복사해 들고 namespace 만 갈라 두었다 (그 패키지 README §2).
합쳐 약 1,900 줄이다. 이 패키지는 **세 번째 사본을 만들지 않고**
`kau_control` 을 `find_package` 해서 그대로 쓴다 —
`kau_local_path_planner_lane` 이 이미 그렇게 하고 있어 검증된 방식이다.
`params.hpp` / `pure_pursuit.hpp` / `pid.hpp` 도 마찬가지다.

### 2. 조향 슬루(변화율) 제한을 실제로 건다

`VehicleParams::max_steer_rate` 는 `kau_control/params.hpp` 에 **차량
확정값 600 deg/s 로 선언되어 있는데 어느 조향 제어기도 쓰지 않는다** —
`kau_control/src/steer_controller_node.cpp` 와
`kau_control_lane/src/lane_steer_controller_node.cpp` 둘 다 grep 으로 확인했다.
서보가 못 따라가는 명령을 내면 실제 조향각이 명령과 달라지고, 그만큼
Pure Pursuit 의 전제가 깨진다.

여기에 더해 **소스 전환 직후에만 더 조인 상한**(`transition_rate_dps`,
기본 150 deg/s × 0.4s)을 건다. 차선 경로와 회피 경로는 서로 다른 곡선이라
전환 순간 lookahead 점이 튀고, 그대로 두면 조향이 계단처럼 꺾이기 때문이다.

실측 (전체 체인 라이브, 차선 R=+100 ↔ 회피 R=−100 으로 24.4° 뒤집기):

| | 조향 최대 변화율 |
|---|---|
| 슬루 제한 켬 | **169 deg/s** (램프가 여러 tick 에 퍼짐) |
| 슬루 제한 끔 | 첫 샘플에서 이미 −12.2° — 한 tick(0.02s) 안에 스냅 = **~1,220 deg/s** |

150 deg/s × 0.4s = 60deg 로 전 범위(−20~+20, 40deg)를 훑고도 남는다 —
정상 조향을 막지 않으면서 계단만 시간축으로 편다. (169 가 150 을 넘는 건
타이머 지터로 실제 dt 가 0.02s 보다 컸기 때문이다. 제한은 실제 dt 기준으로
걸린다.)

**근본 해법은 아니다.** 경로 자체를 블렌딩하는 것이 맞다. 전환 **횟수**는
`kau_path_arbiter` 의 히스테리시스가 줄이고, 여기서는 남은 전환의
**날카로움**만 줄인다.

### 3. 소스별 timeout

`/path/drive` 는 차선(14Hz)과 회피 경로(5Hz)를 오간다. 하나의 timeout 으로
둘을 덮으면, 14Hz 에 맞추면 회피 모드에서 멀쩡한 경로를 죽은 것으로 보고,
5Hz 에 맞추면 차선 모드 두절을 1 초 가까이 못 알아챈다. `KauPath.source` 를
보고 갈라 쓴다 (`drive.timeout_lane` / `drive.timeout_local`).

### 4. pose_source 분기가 없다

`/path/drive` 는 `kau_path_arbiter` 가 내는 것이고, 그 노드는 자차
프레임(`base_link`/`base_footprint`, 평면 동치)이 아닌 경로를 아예 중계하지
않는다. "경로가 map 으로 올 수도 있다" 는 경우가 계약상 존재하지 않으므로
TF 를 보지 않는다. 그래도 `drive.frame` 과 다르면 ERROR 를 찍고 버린다
(다른 발행자에 직접 물릴 수 있으므로).

## 인터페이스

| 방향 | 토픽 | 타입 | 노드 |
|---|---|---|---|
| 구독 | `/path/drive` | `kau_msgs/KauPath` | 둘 다 |
| 구독 | `/speed` | `std_msgs/Float64` | steer (Ld 계산용) |
| 발행 | `/steering` | `std_msgs/Float64` [rad] | steer |
| 발행 | `/speed` | `std_msgs/Float64` [m/s] | speed |
| 발행 | `/debug/steer` | `kau_msgs/SteerDebug` | steer |
| 발행 | `/viz/path/lookahead` | `geometry_msgs/PointStamped` | steer |

경로가 끊기면 다른 소스로 내려가지 않고 **조향 0 / 속도 0** 을 낸다
(`kau_control_lane` 의 전제 그대로).

## 빌드 / 실행 / 테스트

```bash
colcon build --packages-select kau_control_drive

# 조향만 관찰 (차는 안 움직인다). 처음엔 반드시 이걸로 시작할 것
ros2 launch kau_control_drive control_drive.launch.py speed:=0.0

# 중재 노드까지 같이
ros2 launch kau_control_drive control_drive.launch.py arbiter:=true speed:=0.4

colcon test --packages-select kau_control_drive
```

## 아직 안 한 것

- **실차 검증 전이다.** 위 실측은 합성 경로를 주입한 라이브 체인 테스트다.
- **속도 되먹임(PID)이 없다.** 목표속도를 그대로 내는 개루프다.
  `kau_control` 의 speed PID 는 `feedback_topic` 기본값이
  `/odometry/filtered` 인데 그 토픽은 이 시스템에 존재하지 않는다
  (`kau_gui/README.md` 142 — 실제 발행자는 `/odom`). 붙이려면 `/odom` 으로
  두고 `kau_control/pid.hpp` 를 재사용할 것.
- **파라미터가 런타임 변경이 안 된다.** 전부 생성자에서 한 번만 읽는다.
  `kau_control_lane` 은 튜닝용 파라미터 콜백이 있다. 스윕하려면 그걸
  옮겨오거나 노드를 다시 띄울 것.
- `/viz/path/tracked` publisher 는 만들어 두었지만 아직 채우지 않는다
  (lookahead 점만 발행한다).
