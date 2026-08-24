# kau_gui

주행 디버깅 GUI. **map + 추종오차 플롯 + 노드 상태를 한 화면에.**

> 상태: **작성 완료, 빌드·실행 미검증.** 실차(Ubuntu 24.04 / ROS 2 Jazzy)에서
> 확인 필요.
>
> 최종 갱신: 2026-08-24

## 1. 화면

```
┌──────────────────────────────────────────────────┐
│ 노드 14 개 상태등 + 동작 Hz  (가로 전체, 고정)     │
├─────────────────────┬────────────────────────────┤
│ speed  target/real  │  맵 배경 (pgm 또는 /map)    │
│ steer  raw/cmd      │  /scan_filtered 점군        │
│ lookahead distance  │  장애물 원                  │
│ heading error       │  global / local / lane 경로 │
│ cross track error   │  차량 + lookahead  [범례]   │
└─────────────────────┴────────────────────────────┘
```

## 2. 왜 rviz2 가 아닌가

| | rviz2 | kau_gui |
| --- | --- | --- |
| 렌더 | Ogre 3D 씬그래프 · 셰이더 | QPainter 2D |
| RPi5 | `/dev/dri` 없음 -> llvmpipe 소프트웨어 렌더 | 3D 파이프라인 자체가 없음 |
| 추종오차 5 종 | 불가 (`rqt_plot` 별도 창) | 좌측 패널 |
| 노드별 Hz | 불가 (`ros2 topic hz` 터미널마다) | 상단 패널 |

**주된 이유는 경량화가 아니라 한 화면에 모으는 것이다.** rviz2 실측은
RSS 197 MB · CPU 5.8 % (생애 평균) 로, 절대값 자체는 크지 않다.

신규 패키지 설치는 **없다.** Qt5 는 `ros-jazzy-rviz2` / `ros-jazzy-rqt-gui-cpp`
가 이미 `qtbase5-dev` 를 물고 있다.

## 3. 빌드

```bash
colcon build --packages-select kau_msgs kau_control kau_gui
source install/setup.bash
```

`kau_msgs` -> `kau_control` -> `kau_gui` 순서로 의존한다.

## 4. 실행

```bash
ros2 run kau_gui kau_gui_node --ros-args \
  --params-file src/kau_gui/config/gui.yaml
```

맵 배경을 파일에서 읽으려면:

```bash
ros2 run kau_gui kau_gui_node --ros-args \
  --params-file src/kau_gui/config/gui.yaml \
  -p map.yaml_path:=$PWD/src/kau_localization/maps/kau_v3.yaml
```

`bringup.yaml` 에 **넣지 않는다.** Supervisor 가 spawn 하는 대상이 아니다
(`kau_state_machine/docs/09` section 8).

### 조작

| 키 | 동작 |
| --- | --- |
| `space` | 화면 갱신 일시정지. 수신은 계속되므로 재개하면 이력이 그대로 남는다 |
| `r` | 뷰 리셋 (맵 전체 보기) |
| `f` | 차량 추종 on/off |
| 휠 / 드래그 | 줌 / 팬 |

## 5. 구독 토픽

**관측 전용.** publisher · service client 가 하나도 없다 (`09` 계약 104·128).

### 표시용

| 표시 | 토픽 | 타입 |
| --- | --- | --- |
| 맵 배경 | `/map` (또는 pgm 파일) | `nav_msgs/OccupancyGrid` |
| 스캔 | `/scan_filtered` | `sensor_msgs/LaserScan` |
| 장애물 | `/perception/obstacle_markers` | `visualization_msgs/MarkerArray` |
| global path | `/viz/path/global` | `nav_msgs/Path` |
| local path | `/viz/path/local` | `nav_msgs/Path` |
| 중앙선 | `/viz/path/lane` | `nav_msgs/Path` |
| lookahead | `/viz/path/lookahead` | `geometry_msgs/PointStamped` |
| 차량 자세 | TF `map -> base_link` | |
| target speed | `/speed` | `std_msgs/Float64` |
| real speed | `/odom` | `nav_msgs/Odometry` |
| cmd steer | `/steering` | `std_msgs/Float64` [rad] -> deg 변환 |
| raw steer · Ld · heading err · cte | `/debug/steer` | `kau_msgs/SteerDebug` |

`/odometry/filtered` 는 **존재하지 않는다.** `bringup.yaml` 이 `/odom` 으로
remap 한다.

### 감시용

`watch.rates > 0` 인 항목마다 generic subscription 을 만든다. 역직렬화하지
않고 수신 시각만 찍으므로, 큰 메시지를 감시해도 부하가 없다.

감시 토픽은 `/viz/*` 대리물이 아니라 **제어단이 실제로 쓰는 원본**이다.

## 6. 노드 상태 판정

`/diag/heartbeat` 이 아직 없으므로 ROS graph + 토픽 주기 실측으로 대신한다.

| 색 | 조건 |
| --- | --- |
| 초록 | graph 에 있고 대표 토픽이 제 주기로 온다 |
| 빨강 | graph 에는 있으나 토픽이 끊겼다 |
| 회색 | graph 에 없다 (미기동) |

- 감시 목록은 `config/gui.yaml` 의 `watch.*` 에 **고정**한다. 자동 탐색으로는
  떠 있지 않은 노드를 보여줄 수 없어 정작 필요한 빨간불이 안 켜진다
- `rate: 0` 인 4 개(`map_server` `map_amcl_lifecycle_manager`
  `laser_scan_validator` `state_machine`)는 대표 발행 토픽이 없거나 latched 라
  주기 판정이 불가능하다. 존재만으로 정상 처리하고 Hz 칸은 `-`
- stale 임계에는 하한(`min_stale_s`, 기본 0.3 s)이 있다. 50 Hz 토픽은
  `stale_ratio` 3 배가 60 ms 라 무선 지터만으로 깜빡인다

### 알려진 오탐

| 증상 | 원인 |
| --- | --- |
| `global_path_publisher` 가 빨강 | `global_path.launch.py` 를 `rate:=0` (1 회 발행) 으로 띄웠다. `rate>0` 필요 |
| `kau_lane_detection_node` 가 빨강인데 노드는 정상 | `path_frame_id: map` 이라 측위가 끊기면 `/lane/center` 발행이 멈춘다. 실제로 알아야 할 정보라 그대로 둔다 |
| `state_machine` 이 초록인데 아무 일도 안 함 | 현재 스텁이라 발행 토픽이 없다. graph 존재만 본다 |

## 7. 설계 규약

| 규약 | 이유 |
| --- | --- |
| 맵 계열은 최신값 latch, **플롯 계열은 링버퍼에 전량 append** | 렌더가 10 Hz 라고 50 Hz 신호를 10 Hz 로 샘플링하면 조향 떨림·cte 스파이크가 화면에서 사라진다 |
| 플롯은 픽셀 열당 min/max 데시메이션 | 단순 솎아내기는 한두 표본짜리 사건을 통째로 지운다 |
| 낡은 표시물은 흐리게, 끊긴 구간은 선을 끊는다 | 낡은 데이터를 최신인 척 그리는 것이 디버깅 도구에서 가장 위험하다 |
| 시간축은 GUI 노드 clock 기준 수신 시각 | 발행 노드마다 sim/system clock 이 섞이면 축이 어긋난다 |
| 색은 `theme.hpp` 한 곳 | 흩어 두면 범례와 실제 선 색이 어긋난다 |

## 8. 미확정

| 항목 | 현재 | 필요 작업 |
| --- | --- | --- |
| 빌드·실행 | **미검증** | 실차에서 `colcon build` 후 확인 |
| `watch.rates` | 대부분 설계값 | 실측 후 조정 (특히 `amcl` 5 Hz, `kau_ekf` 30 Hz) |
| 실행 위치 | 미정 | `09` 문서는 노트북 전용. 차량에서 띄우면 렌더 부하가 주행에 얹힌다 |
| `/diag/heartbeat` | 미구현 | 생기면 `rate: 0` 4 개를 그쪽 판정으로 옮긴다 |
| 차량 외형 치수 | 0.30 x 0.20 m 가정 | 실측 후 `vehicle.*` 갱신 |
