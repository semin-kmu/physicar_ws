# kau_path_arbiter

주행 경로 **소스 중재**. 장애물이 없으면 차선(`/lane/center`)을 그대로
따라가고, 있을 때만 회피 경로(`/path/local`)로 넘어간다. 둘을 골라
`/path/drive` 하나로 내보내므로 제어기는 토픽 하나만 보면 된다.

```
  /lane/center  (KauPath, base_link,      14Hz)  ─┐
  /path/local   (KauPath, base_footprint,  5Hz)  ─┼─> [arbiter] ─> /path/drive
  /perception/obstacles (ObstacleCircleArray)    ─┘                /debug/path_arbiter
```

## 왜 만들었나

`kau_local_path_planner_lane` 은 장애물 회피를 위해 존재하는데, 장애물이
없는 구간에서도 계획 기하가 주행 루프에 끼어 있었다. 실제 트랙 폴리곤
(`kau_object_detection/config/amet_2026_track.yaml`) 위에서 기구학 시뮬을
돌려 비교한 결과다 — 1 바퀴 28.3m, v=0.5m/s, 조향 서보 300deg/s,
자전거 모델, 차선 인지는 완벽하다고 가정:

| 방식 | 최대 \|cte\| | 최소 노면여유 | 이탈 시간 |
|---|---|---|---|
| **`/lane/center` 직접 추종 (Pure Pursuit)** | **10.1cm** | **+10.8cm** | **0%** |
| 곡률 피드포워드 + Pure Pursuit | 20.1cm | +4.7cm | 0% |
| 곡률 검출시 조향 max (bang-bang) | >300cm | −9.5cm | 14.0% |

장애물이 없으면 계획을 거치지 않는 쪽이 더 정확하다. 그래서 평상시엔
차선을 그대로 쓰고, 플래너는 그것이 실제로 필요한 순간에만 쓴다.

### bang-bang 이 왜 안 되는가

"곡률이 검출되면 그쪽으로 조향 최대" 는 직관적이지만 이 트랙에서 못 쓴다.
같은 시뮬에서 검출거리를 100 / 50 / 32cm 로 바꿔도 전부 이탈했다
(이탈 시간 14.0 / 8.2 / 13.6%).

트랙 실측:

- 중앙선 곡률반경 **중앙값 251cm**, p25 = 102cm. **R < 50cm 구간은 2.8%뿐.**
- 노면 반폭 중앙값 35cm, 최소 21.6cm.

조향 한계 20° 는 R = 49.5cm 다. 트랙의 97% 가 그보다 훨씬 완만하므로
full lock 은 대부분 구간에서 과조향이고, 이탈까지 걸리는 거리는
`s = sqrt(2*R*w) = sqrt(2*49.5*35) ~= 59cm`, 0.5m/s 에서 **1.2초**다.
곡률이 "있다/없다" 이진 판정인데 실제 곡률은 1/251 ~ 1/21 로 연속
분포하므로, 임계를 어디에 두든 한쪽이 틀린다.

## 설계에서 신경 쓴 것

### 히스테리시스는 비대칭이다

진입은 즉시(`engage_sec: 0.0`), 해제는 느리게(`release_sec: 2.0`). 장애물이
인지 경계에서 깜빡일 때 5Hz/14Hz 두 소스를 오가면 조향이 튄다. gtest
`DoesNotChatterOnFlickeringDetection` 이 10Hz 로 한 프레임 걸러 깜빡이는
3 초 동안 전환이 **한 번**뿐임을 확인한다.

### `status != OK` 는 "장애물 없음" 이 아니다

`ObstacleCircleArray` 계약상 non-OK 프레임은 항상 빈 배열이지만, 그건
"인식 결과 사용 불가" 이지 노면이 비었다는 뜻이 아니다. 이걸 clear 로
읽으면 라이다가 죽는 순간 장애물 앞에서 차선 모드로 내려간다. 그래서
모르는 동안에는 해제 타이머를 진행시키지 않고 현재 모드를 유지한다
(gtest `InvalidObservationDoesNotReleaseAvoid`, 10 초 두절 검증).

해제 시간을 벽시계가 아니라 **마지막 유효 관측 시각** 기준으로 재는 것도
같은 이유다 — 인지가 죽어 있는 동안 시간만 흘렀다고 회피를 그만두면 안 된다.

### 끊기면 다른 데로 안 내려가고 선다

`kau_control_lane` 의 전제를 그대로 따른다. 회피 중에 `/path/local` 이
끊겨도 차선으로 **내려가지 않는다** — 장애물이 있는데 장애물을 모르는
경로로 가는 건 장애물을 향해 그대로 가는 것과 같다. 발행을 멈추면 제어기가
timeout 으로 선다. 굳이 내려가고 싶으면
`fallback_to_lane_on_local_loss: true`.

### 재발행하지 않는다

발행은 **이벤트 구동** 이다. 고른 소스가 발행할 때만 중계한다. 타이머로
같은 경로를 새 stamp 로 다시 내면 죽은 소스가 살아있는 것처럼 보여 제어기의
timeout 안전망이 무력해진다. (모드 판정만 20Hz 타이머로 따로 돈다 — 회피
중 `/path/local` 이 죽으면 발행 이벤트가 없어 히스테리시스 해제가 멈추기
때문이다.)

### frame_id 만 바꿔 다는 것이 왜 안전한가, 그리고 언제 안전하지 않은가

`base_footprint -> base_link` 는 URDF 상 **순수 Z 오프셋**이다
(`physicar.urdf.xacro` 의 `base_joint`: `origin xyz="0 0 wheel_radius"
rpy="0 0 0"`). 평면 좌표가 동일하므로 제어점을 건드리지 않고 이름만 바꿔
중계할 수 있다.

그 전제가 깨지는 frame 이 오면 중계를 **거부하고 ERROR 를 찍는다.**
`kau_lane_detection` 의 `path_frame_id` 기본값이 `"map"` 이라 실수로 절대
좌표가 들어올 수 있는데, 그걸 `base_footprint` 라고 이름만 바꿔 내보내면
제어기가 절대좌표를 자차 상대좌표로 읽어 차가 즉시 튄다.

## 인터페이스

| 방향 | 토픽 | 타입 | QoS |
|---|---|---|---|
| 구독 | `/lane/center` | `kau_msgs/KauPath` | RELIABLE, depth 1 |
| 구독 | `/path/local` | `kau_msgs/KauPath` | RELIABLE, depth 1 |
| 구독 | `/perception/obstacles` | `kau_msgs/ObstacleCircleArray` | BEST_EFFORT, depth 1, volatile |
| 발행 | `/path/drive` | `kau_msgs/KauPath` | RELIABLE, depth 1 |
| 발행 | `/debug/path_arbiter` | `std_msgs/String` | BEST_EFFORT, depth 1 |

`/path/drive` 는 원본 메시지를 그대로 중계한다 — `source` 필드(`SRC_LANE` /
`SRC_LOCAL`), `confidence`, `valid_length`, `stamp` 가 전부 보존되므로
하류가 지금 무엇을 따라가는지 알 수 있다. 바뀌는 것은 `header.frame_id`
하나뿐이다.

## 제어기 연결 (아직 안 되어 있음)

이 노드만 띄워서는 아무 일도 안 일어난다.
`kau_control_lane/config/lane_source.yaml` 을 `/path/drive` 로 돌려야 한다:

```yaml
lane:
  topic: "/path/drive"
  frame: "base_footprint"     # topic 과 frame 은 짝이다
  timeout: 0.5                # 차선(14Hz)이 기본 소스이므로 center 쪽 값
```

`timeout` 은 두 소스 중 **느린 쪽**(local 5Hz)이 아니라 각각의 소스가
자기 주기로 오는 것을 전제해야 한다. 회피 모드에서는 5Hz 로 떨어지므로
`timeout: 0.5` 이면 5Hz(0.2s) 기준 2.5 주기다 — 여유가 빠듯하면 1.0 으로
둘 것. 이 값은 아직 실차 검증 전이다.

## 빌드 / 실행 / 테스트

```bash
colcon build --packages-select kau_path_arbiter
ros2 launch kau_path_arbiter path_arbiter.launch.py

# 지금 무엇을 따라가는지
ros2 topic echo /debug/path_arbiter

colcon test --packages-select kau_path_arbiter
```

## 아직 안 한 것

- **실차/시뮬 검증 전이다.** 위 표는 전부 기구학 시뮬이고, 차선 인지가
  완벽하다고 가정했다. 실제 `/lane/center` 는 노이즈가 있고 끊긴다.
  (bang-bang 의 실패는 노이즈가 아니라 순수 기하 문제라 순위 자체는
  바뀌지 않겠지만, PP 의 10.1cm 는 낙관적인 값이다.)
- **전환 순간의 경로 불연속**을 다루지 않는다. LANE -> AVOID 로 넘어갈 때
  두 경로의 기하가 다르면 Pure Pursuit 의 목표점이 튄다. 지금은 모드
  히스테리시스로 전환 **횟수**만 줄여 두었다. 실측에서 조향 튐이 보이면
  전환 구간 블렌딩이 필요하다.
- `release_sec` 은 v=0.5m/s 기준이다. `v_max` 를 바꾸면 다시 잡을 것.
