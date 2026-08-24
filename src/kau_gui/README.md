# kau_gui

주행 디버깅 GUI. **map + 추종오차 플롯 + 노드 상태를 한 화면에.**

화면 규약은 `KAU_AMET_Test/src/full_simulation.py` 의 `show_realtime` 과 같다.
시뮬과 실차를 나란히 놓고 보는 일이 많으므로 **같은 색이 같은 것을 가리킨다.**

> 상태: **작성 완료, 실행 미검증.** 실차(Ubuntu 24.04 / ROS 2 Jazzy)에서 확인 필요.
>
> 최종 갱신: 2026-08-25

## 1. 화면

```
┌──────────────────────────────────────────────────┐
│ 노드 14 개 상태등 + 동작 Hz  (가로 전체)           │
├─────────────────────┬────────────────────────────┤
│ target speed        │  map                       │
│ target steer        │   scan · TF · obstacle     │
│ lookahead distance  │   global / local / lane    │
│ cross track error   │              [HUD] [범례]  │
│ heading error       │                            │
└─────────────────────┴────────────────────────────┘
```

열 비율 95:125 (`setColumnStretchFactor`), 흰 바탕 / matplotlib tab10.

### map 표시물

| 표시 | 색 | 소스 |
| --- | --- | --- |
| LiDAR 점군 | 회색 `#8c8c8c` | `/scan_filtered` |
| global path | 파랑 `#1f77b4` (`COL_GLOBAL`) | `/viz/path/global` |
| lane center | 초록 `#2ca02c` (`COL_LANE`) | `/viz/path/lane` |
| local path | 빨강 `#d62728` (`COL_LOCAL`) | `/viz/path/local` |
| obstacle | 주황 `#ff7f0e`, **반지름 동적** | `/perception/obstacle_markers` |
| TF | **ROS 표준 축** x 빨강 / y 초록 | `map` → `odom` → `base_link` |
| 차체 | 빨강 외곽선 | TF `base_link` |
| 배경 | 회색조 | `/map` |

TF 는 세 프레임의 축 마커 + 부모·자식 연결선(점선)으로 그린다. 2D 탑뷰라
z 축(파랑)은 생략한다.

### HUD (맵 좌상단)

맵에 그려지는 것들의 수신 상태만 담는다. 제어 수치는 좌측 플롯 소관이다.

```
O global path   412 pt   0.0s
O local path     98 pt   0.1s
X lane center         미수신
O tf map>odom          0.0s
X tf odom>base         2.4s
O obstacles       3 EA   0.1s
```

`map→odom` 은 AMCL, `odom→base_link` 는 EKF 소관이라 나눠서 본다. 어느 링크가
끊겼는지가 곧 어느 노드 문제인지다 (`kau_localization/README.md` TF 소유권).

## 2. 설치 — venv

pyqtgraph 는 **패키지 안 `.venv`** 에만 들어간다. 시스템 python 은 건드리지 않는다.

```bash
src/kau_gui/scripts/setup_venv.sh
```

- `--system-site-packages` 로 만들어 `rclpy` · `numpy` · `PyQt5` 는 시스템 것을 쓴다.
  이 옵션이 없으면 venv 안에서 rclpy 가 안 보여 ROS 구독 자체가 불가능하다
- venv 에 새로 들어가는 것은 **pyqtgraph 하나뿐**이다. 지우려면 `.venv/` 삭제로 끝
- 대회장은 인터넷이 없다. 미리 wheel 을 받아 두고 오프라인 설치:
  ```bash
  # 노트북에서
  python3 -m pip download pyqtgraph --no-deps -d wheels/
  # 차량에서
  src/kau_gui/scripts/setup_venv.sh /path/to/wheels
  ```

`kau_gui/_venv.py` 가 실행 시 venv 의 site-packages 를 `sys.path` 에 얹는다.
venv 를 activate 하지 않는 이유는 python 실행 파일이 바뀌면 `ros2 run` 의 경로와
얽히기 때문이다. `KAU_GUI_VENV` 로 경로를 덮어쓸 수 있다.

## 3. 빌드 · 실행

```bash
colcon build --packages-select kau_msgs kau_control kau_gui --symlink-install
source install/setup.bash

ros2 run kau_gui kau_gui --ros-args \
  --params-file src/kau_gui/config/gui.yaml
```

`bringup.yaml` 에 **넣지 않는다.** Supervisor 가 spawn 하는 대상이 아니다
(`kau_state_machine/docs/09` section 8).

### 조작

| 키 | 동작 |
| --- | --- |
| `space` | 화면 갱신 일시정지. 수신은 계속되므로 재개하면 이력이 그대로 남는다 |
| `r` | 뷰 리셋 |
| `f` | 차량 추종 on/off |
| 휠 / 드래그 | 줌 / 팬 (pyqtgraph 기본) |

## 4. 구독 토픽

**관측 전용.** publisher · service client 가 하나도 없다 (`09` 계약 104·128).

| 용도 | 토픽 | 타입 |
| --- | --- | --- |
| 맵 배경 | `/map` | `nav_msgs/OccupancyGrid` |
| 스캔 | `/scan_filtered` | `sensor_msgs/LaserScan` |
| 장애물 | `/perception/obstacle_markers` | `visualization_msgs/MarkerArray` |
| 경로 3 종 | `/viz/path/{global,local,lane}` | `nav_msgs/Path` |
| target speed | `/speed` | `std_msgs/Float64` |
| real speed | `/odom` | `nav_msgs/Odometry` |
| cmd steer | `/steering` | `std_msgs/Float64` [rad] → deg |
| raw steer · Ld · heading err · cte | `/debug/steer` | `kau_msgs/SteerDebug` |
| 노드 감시 | `watch.topics` | raw 구독 (역직렬화 안 함) |

`/odometry/filtered` 는 **존재하지 않는다.** `bringup.yaml` 이 `/odom` 으로 remap 한다.

## 5. 노드 상태 판정

`/diag/heartbeat` 이 아직 없으므로 ROS graph + 토픽 주기 실측으로 대신한다.

| 색 | 조건 |
| --- | --- |
| 초록 | graph 에 있고 대표 토픽이 제 주기로 온다 |
| 빨강 | graph 에는 있으나 토픽이 끊겼다 |
| 회색 | graph 에 없다 (미기동) |

- 감시 목록은 `config/gui.yaml` 의 `watch.*` 에 **고정**한다. 자동 탐색으로는
  떠 있지 않은 노드를 보여줄 수 없어 정작 필요한 빨간불이 안 켜진다
- `rate: 0` 인 4 개(`map_server` `map_amcl_lifecycle_manager`
  `laser_scan_validator` `state_machine`)는 주기 판정이 불가능하다. 존재만으로
  정상 처리하고 Hz 칸은 `-`
- stale 임계에는 하한(`min_stale_s`, 기본 0.3 s)이 있다. 50 Hz 토픽은
  `stale_ratio` 3 배가 60 ms 라 무선 지터만으로 깜빡인다

### 알려진 오탐

| 증상 | 원인 |
| --- | --- |
| `global_path_publisher` 가 빨강 | `global_path.launch.py` 를 `rate:=0` 으로 띄웠다. `rate>0` 필요 |
| `kau_lane_detection_node` 가 빨강인데 노드는 정상 | `path_frame_id: map` 이라 측위가 끊기면 `/lane/center` 발행이 멈춘다. 실제로 알아야 할 정보라 그대로 둔다 |
| `state_machine` 이 초록인데 아무 일도 안 함 | 현재 스텁이라 발행 토픽이 없다. graph 존재만 본다 |

## 6. 설계 규약

| 규약 | 이유 |
| --- | --- |
| 맵 계열은 최신값 latch, **플롯 계열은 링버퍼에 전량 append** | 렌더가 10 Hz 라고 50 Hz 신호를 10 Hz 로 샘플링하면 조향 떨림·cte 스파이크가 사라진다 |
| 시간축은 GUI 노드 clock 기준 수신 시각 | 발행 노드마다 sim/system clock 이 섞이면 축이 어긋난다 |
| 감시 구독은 raw (역직렬화 안 함) | 큰 메시지를 감시해도 부하가 수신 시각 기록뿐 |
| 스캔은 TF 실패 시 갱신하지 않고 늙게 둔다 | 낡은 점군을 최신인 척 그리는 것이 가장 위험하다 |
| 색은 `viz.py` 한 곳 | 흩어 두면 범례와 실제 선 색이 어긋난다 |

## 7. 미확정

| 항목 | 현재 | 필요 작업 |
| --- | --- | --- |
| 실행 검증 | **미검증** | 실차에서 확인 |
| `watch.rates` | 대부분 설계값 | 실측 후 조정 (특히 `amcl` 5 Hz, `kau_ekf` 30 Hz) |
| 실행 위치 | 미정 | `09` 문서는 노트북 전용. 차량에서 띄우면 렌더 부하가 주행에 얹힌다 |
| `/diag/heartbeat` | 미구현 | 생기면 `rate: 0` 4 개를 그쪽 판정으로 옮긴다 |
| 맵 파일 폴백 | 미구현 | 현재는 `/map` 토픽만. `map_server` 없이 띄우면 배경이 없다 |
