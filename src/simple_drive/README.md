# simple_drive

밴뱅 조향 + 타이머 회피만으로 트랙을 도는 최소 주행 스택.

정석 스택(`backup_path_planner` -> `backup_speed_controller` /
`backup_steer_controller`)의 경로계획 + Pure Pursuit + 곡률 속도프로파일을
전부 버리고, **완주 자체**를 목표로 다시 짠 것이다.

- **인지** — 새로 만들지 않았다. 기존 `backup_*` 인지 노드를 그대로 쓴다.
- **판단** (`simple_decision`) — 전방 횡오차로 조향 상태를, 라이다로 회피
  시퀀스를 정한다.
- **제어** (`simple_control`) — 단위변환 / 레이트 리밋 / 워치독 / 종료 0.
  주행 판단은 하지 않는다.

## 조향은 세 값밖에 없다

| 상황 | 조향 |
|---|---|
| 직선 (횡오차 작음) | `0 deg` |
| 코너 (횡오차 큼) | `+-20 deg` (하드웨어 전타) |
| 회피 DODGE / COUNTER | `+-20 deg` |
| 회피 HOLD | `0 deg` |

사이값은 만들지 않는다. 비례제어 튜닝을 포기하고 저속 + 전타 스위칭으로
가는 것이 이 스택의 전제다.

### 왜 전타를 줘도 안 나가는가

- 전타 선회반경 `R = wheelbase / tan(20 deg) = 0.18 / 0.36397 = 0.4945 m`
- 그때 차체가 쓸고 가는 폭 `0.248 m`. 도로 폭은 `0.634 m`
  (중앙선 <-> 흰선 중심 `0.317 m` x 2) 라 **편측 여유 `0.193 m`**
- `v = 0.45 m/s` 로 전타를 유지했을 때 `0.19 m` 밀려나기까지 **약 1.0 초**
  = 50 Hz 제어의 **50 tick**

즉 한 번 전타를 줘도 밴뱅이 되돌릴 시간이 50 사이클이나 남는다.
`R = 1.0 m` 코너에 필요한 정확한 조향은 `10.2 deg` 인데 `20 deg` 를 주므로,
밴뱅은 대략 50 % 듀티로 깜빡이며 평균 `10 deg` 를 만든다.

## 왜 곡률이 아니라 횡오차인가

곡률은 도로 모양만 알려주고 자차가 어디 있는지는 알려주지 않는다.
곡률로만 조향하면 오픈루프라 한 번 밀려난 뒤 **복귀를 못 한다**
(특히 회피 직후). 전방 `lookahead_m` 지점의 중앙선 횡오차 `e` 를 보면
코너에서 `e` 가 계속 생기므로 곡률이 자동으로 반영되고, 동시에 어디로
밀려도 돌아온다. 조향값이 두 개뿐인 것은 그대로다.

## 장애물 회피 — "5 도 1 초" 가 안 되는 이유

조향각 `delta` 로 거리 `d` 를 가는 동안의 횡변위는

```
y = R(1 - cos(d/R)) ~= d^2 / (2R),   R = wheelbase / tan(delta)
```

**속도가 아니라 주행거리가 정한다.** `5 deg` 면 `R = 2.06 m` 라
`v = 0.3 m/s` 로 1 초(`d = 0.30 m`) 꺾어도

```
0.30^2 / (2 x 2.06) = 0.022 m = 2.2 cm
```

밖에 안 움직인다. 콘 반경 `0.09` + 차 반폭 `0.1025` = **`0.19 m`** 를
비켜야 하는데 2 cm 는 아무것도 아니다. 전타라도 `0.19 m` 를 벌려면
`d = sqrt(2R x 0.19) = 0.43 m` 를 가야 하고, 이건 속도를 낮춰도 안 줄어든다.

그래서 회피는 **전타 S 자**다:

```
DODGE(전타 L) -> COUNTER(반대 전타 L) -> HOLD(0 도 직진) -> 차선추종 복귀

총 횡변위 = 2R(1 - cos theta),  총 종방향 = 2R sin theta,  최종 방위 = 0
목표 0.20 m -> theta = 0.640 rad -> L = 0.316 m -> v=0.30 에서 1.05 s
종방향 0.59 m
```

`HOLD` 가 필요한 이유: S 자가 끝난 순간 차는 중앙선에서 20 cm 옆에 있고,
바로 차선추종을 풀면 밴뱅이 **장애물이 있는 쪽으로** 돌아가려 한다.
0 도 직진으로 장애물을 지나친 뒤에 푼다. `HOLD` 는 재트리거 금지 구간이기도
하다.

> `avoid.dodge_s` / `avoid.counter_s` 는 `v_avoid` 에 묶여 있다.
> `dodge_s = 0.316 / v_avoid`. 속도를 바꾸면 반드시 같이 바꾼다.

## 배선

입력 (전부 기존 인지 노드가 이미 발행 중):

| 토픽 | 타입 | 내용 |
|---|---|---|
| `/backup/lane/path` | `backup_msgs/BackupPath` | BEV 노란 중앙선 (뷰어의 주황 선) |
| `/backup/perception/obstacles` | `backup_msgs/ObstacleCircleArray` | 라이다 원 근사 |
| `/perception/start_permission` | `std_msgs/Bool` | 초록불 |

내부:

| 토픽 | 타입 | |
|---|---|---|
| `/simple/cmd_steer_deg` | `std_msgs/Float64` | 판단 -> 제어. `[deg]` 좌측 + |
| `/simple/cmd_speed` | `std_msgs/Float64` | 판단 -> 제어. `[m/s]` |
| `/simple/state` | `std_msgs/String` | 상태 문자열 |
| `/simple/debug` | `backup_msgs/ControlDebug` | 튜닝/로그 전용 |

출력 (플랫폼 계약. 기존 컨트롤러와 같은 토픽/단위):

| 토픽 | 타입 | 단위 |
|---|---|---|
| `/steering` | `std_msgs/Float64` | `[rad]` 좌측 + |
| `/speed` | `std_msgs/Float64` | `[m/s]` |

판단 상태: `NO_PATH` / `PATH_TOO_SHORT` / `WAIT_PERMISSION` / `STRAIGHT` /
`TURN_LEFT` / `TURN_RIGHT` / `AVOID_DODGE` / `AVOID_COUNTER` / `AVOID_HOLD`

## 기동

```bash
colcon build --packages-select simple_drive
source install/setup.bash

# 1) 인지만 띄운다 -- run_backup.sh 의 BACKUP_NODES 에서
#    backup_path_planner / backup_speed_controller / backup_steer_controller
#    를 false 로 바꾼다
source run_backup.sh

# 2) 판단 + 제어
ros2 launch simple_drive simple_drive.launch.py use_sim_time:=false
```

> **★ 위 세 노드를 끄지 않으면 `/speed` 와 `/steering` 을 두 곳에서 쏜다.**
> 두 publisher 값이 섞여 차량에 간다. 반드시 확인하고 띄운다.

## 튜닝 순서

파라미터는 전부 `config/simple_drive.yaml` 에 있고, 근거는 주석에 있다.

1. **`bang.enter_m`** — 직선에서 조향이 떨면 올린다 (기본 `0.04`).
   코너 진입이 늦으면 내린다.
2. **`lookahead_m`** — 코너에서 안쪽으로 파고들면 늘리고, 반응이 굼뜨면
   줄인다 (기본 `0.50`).
3. **`v_corner`** — 코너에서 밀리면 내린다. 여기부터 손댄다.
4. **`bang.min_hold_s`** — 서보에서 소리가 나면 올린다.
5. **회피** — `v_avoid` 를 바꾸면 `dodge_s` / `counter_s` 를 같이 바꾼다.

## 테스트

```bash
colcon test --packages-select simple_drive
colcon test-result --test-result-base build/simple_drive --verbose
```

`test/test_simple_drive.cpp` 는 노드를 띄우지 않는다. 밴뱅 / 회피 FSM /
경로 샘플링이 전부 상태 없는 헤더라 단독으로 돈다.
