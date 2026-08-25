# backup_gui

백업 스택 디버깅 GUI. **노드 상태 + map + 추종오차 플롯을 한 화면에.**

관측 전용이다. publisher 도 service client 도 없다.
좌표는 전부 `base_link` 상대 **m** — 백업 스택은 측위가 없어 절대좌표 자체가 없다.

> 상태: **작성 완료, 실행 미검증.** 실차(Ubuntu 24.04 / ROS 2 Jazzy)에서 확인 필요.
>
> 최종 갱신: 2026-08-26

## 1. 화면

```
┌──────────────────────────────────────────────────┐
│ 노드 상태 띠 (가로 전체)                          │
├─────────────────┬────────────────────────────────┤
│ speed  [m/s]    │  map · base_link 고정 탑뷰      │
│ steer  [deg]    │   scan · obstacle              │
│ CTE    [m]      │   lane geometry (근거)          │
│ θ_err  [deg]    │   BackupPath (판단)             │
│ κ_max  [1/m]    │   좌상단: 경로 상태             │
│                 │   우상단: 차선 상태             │
└─────────────────┴────────────────────────────────┘
     95          :               125
```

### ★ 통과 가능 판정

이 GUI 의 존재 이유다. **경로가 요구하는 곡률이 `1/R_min` 을 넘는지**만 보면 된다.

| 표시 | 뜻 |
| --- | --- |
| `κ_max` 패널의 빨간 점선 | `kappa_max_ref` (2.0221 = `VehicleParams::kappaMax()`) |
| 파란 선이 그 위로 올라감 | 그 순간 경로가 통과 불가 곡률을 요구했다 |
| map 경로가 **빨강 굵은 선** | `kappa_saturated` 가 켜져 있다 |
| 좌상단 `KAPPA SAT ● x3` | 지금 켜짐 + 누적 3 회 |
| 좌상단 `KAPPA SAT (1.2s 전) x3` | 방금 꺼졌다 |

렌더는 3 Hz 인데 경로는 12 Hz 다. **1 프레임짜리 플래그는 렌더가 못 본다.**
그래서 `bridge` 가 발생 시각·누적 횟수를 따로 기록하고 3 초간 화면에 붙든다.
구독은 전량 받는다 — 여기서 주기를 줄이면 봐야 할 신호가 사라진다.

### map 표시물

| 표시 | 색 | 소스 |
| --- | --- | --- |
| LiDAR 점군 | 거리색 (rviz rainbow) | `/scan_filtered` |
| 장애물 | 주황 `#ff7f0e` | `/backup/perception/obstacles` |
| 차선 좌/우 | 회청 `#7d94ab` 실선 | `/backup/lane/geometry` |
| 차선 중앙 | 회청 `#4f6577` 점선 | 〃 |
| 변곡점 | 검정 `x` + `Δψ` | 〃 (`inflection_valid`) |
| 경로 LANE_CENTER | 초록 `#2ca02c` | `/backup/path` |
| 경로 RACE_LINE | 파랑 `#1f77b4` | 〃 |
| 경로 AVOID | 보라 `#9467bd` | 〃 |
| 경로 DEGRADED | 황토 `#b8860b` | 〃 |
| **경로 `kappa_saturated`** | **빨강 `#d62728` 굵게** | 〃 |
| 차체 | 빨강 외곽선, 원점 고정 | — |

**실선 = `valid_length` 안, 점선 = 그 너머 외삽.** 근거가 닿는 구간과
추정 구간이 눈으로 갈려야 한다. 조각 경계는 작은 원으로 찍는다.

빨강은 `kappa_saturated` / `stop_request` 전용이다. 다른 것에 쓰지 않는다.

### 우상단 차선 상태

| 표시 | 뜻 |
| --- | --- |
| `lost NONE` | 정상 |
| `lost LEFT` / `RIGHT` | 한쪽 손실. 정상 주행이다 |
| `lost BOTH · first LEFT → 좌 전타` | fallback 이 `lost_first` 쪽으로 최대 조향 중 |
| `연속 코너` | `second_inflection`. 플래너 퇴화 모드 트리거 |
| `obstacle status N` | `ObstacleCircleArray.status` 가 OK(0) 가 아니다 |

### 노드 상태 띠

셀 하나 = 노드 하나. 좌측부터 **이름 · 상태등 · Hz**.

| 색 | 뜻 |
| --- | --- |
| 초록 | 그래프에 있고 대표 토픽이 기대 주기대로 온다 |
| 빨강 | 노드는 살아 있는데 토픽이 끊겼다 |
| 회색 | 그래프에 없다 (안 떴거나 죽었다) |

감시 목록은 `gui.yaml` 의 `status.watch` 하나로 정한다 —
`"<노드> | <topic> | <타입> | <기대 Hz>"`.
백업 스택에는 supervisor 도 bringup manifest 도 없다.

## 2. 설치 — venv

pyqtgraph 는 **패키지 안 `.venv`** 에만 들어간다. 시스템 python 은 건드리지 않는다.

```bash
src/backup_gui/scripts/setup_venv.sh
```

- `--system-site-packages` 로 만들어 `rclpy` · `numpy` · `PyQt5` 는 시스템 것을 쓴다.
  이 옵션이 없으면 venv 안에서 rclpy 가 안 보여 ROS 구독 자체가 불가능하다
- venv 에 새로 들어가는 것은 **pyqtgraph 하나뿐**이다. 지우려면 `.venv/` 삭제로 끝
- 대회장은 인터넷이 없다. 미리 wheel 을 받아 두고 오프라인 설치:
  ```bash
  python3 -m pip download pyqtgraph --no-deps -d wheels/   # 노트북
  src/backup_gui/scripts/setup_venv.sh /path/to/wheels     # 차량
  ```

`backup_gui/_venv.py` 가 실행 시 venv 의 site-packages 를 `sys.path` 에 얹는다.
venv 를 activate 하지 않는 이유는 python 실행 파일이 바뀌면 `ros2 run` 의 경로와
얽히기 때문이다. `BACKUP_GUI_VENV` 로 경로를 덮어쓸 수 있다.

## 3. 빌드 · 실행

```bash
colcon build --packages-select backup_msgs backup_gui --symlink-install
source install/setup.bash

ros2 launch backup_gui gui.launch.py                    # 인자 없이 뜬다
ros2 launch backup_gui gui.launch.py use_sim_time:=false
```

`backup_bringup/launch/backup.launch.py` 가 이 launch 를 포함한다.
GUI 만 빼려면 `skip:=backup_gui`.

| 키 | 동작 |
| --- | --- |
| `space` | 화면 갱신 일시정지. 수신은 계속되므로 재개하면 이력이 그대로 남는다 |
| `r` | 뷰 리셋 |
| 휠 / 드래그 | 줌 / 팬 |

## 4. 구독 토픽

| 용도 | 토픽 | 타입 |
| --- | --- | --- |
| 스캔 | `/scan_filtered` | `sensor_msgs/LaserScan` |
| 장애물 | `/backup/perception/obstacles` | `backup_msgs/ObstacleCircleArray` |
| 경로 | `/backup/path` | `backup_msgs/BackupPath` |
| 차선 기하 | `/backup/lane/geometry` | `backup_msgs/LaneGeometry` |
| target speed | `/speed` | `std_msgs/Float64` |
| real speed | `/odom` | `nav_msgs/Odometry` (없으면 빈 플롯) |
| cmd steer | `/steering` | `std_msgs/Float64` |
| CTE · heading err | `/backup/debug/steer` | `backup_msgs/ControlDebug` |

`topics.*` 를 빈 문자열로 두면 그 구독을 만들지 않는다.

상태 띠는 위 표에 이미 있는 토픽은 **표시용 구독을 그대로 쓴다.** 나머지는
`raw=True` 구독을 하나 더 만들어 역직렬화 없이 **수신 시각만 찍는다.**

## 5. 설계 규약

| 규약 | 이유 |
| --- | --- |
| 맵 계열은 최신값 latch, **플롯 계열은 링버퍼에 전량 append** | 렌더가 3 Hz 라고 50 Hz 신호를 3 Hz 로 샘플링하면 조향 떨림·CTE 스파이크가 사라진다 |
| 짧은 플래그는 **발생 시각·횟수를 따로 기록** | latch 만으로는 3 Hz 렌더가 12 Hz 의 1 프레임 이벤트를 못 본다 |
| 시간축은 GUI 노드 clock 기준 수신 시각 | 발행 노드마다 sim/system clock 이 섞이면 축이 어긋난다 |
| **TF 조회 없음** (`frames` 가 전부 `base_link`) | 백업 스택은 측위 비의존이 전제다. `lidar_link → 후륜축` 은 yaml 상수 |
| 맵·트랙이 없어도 기동. 경고는 기동 시 1 회 | 매 프레임 경고는 로그를 못 쓰게 만든다 |
| 색은 `viz.py` 한 곳 | 흩어 두면 범례와 실제 선 색이 어긋난다 |
| antialias 끔 · 렌더 3 Hz · 창 1280x720 | 렌더 예산 |

`antialias=False` 가 렌더 비용의 거의 전부다 (측정 1840 → 60 ms/frame).
pyqtgraph 는 aa 를 켜면 곡선을 `drawLines` 대신 `drawPath` 로 그린다.
같은 조건에 선 굵기 > 1.0 도 있으므로 **선 굵기를 1.0 이하로 내리지 말 것.**

## 6. 미확정

| 항목 | 현재 | 필요 작업 |
| --- | --- | --- |
| 실행 검증 | **미검증** | 실차에서 확인 (헤드리스 불가) |
| `scan.*` 정적 변환 | `obstacle_detector.yaml` 과 같은 값 | 한쪽만 바뀌면 점군이 장애물 원에서 어긋난다. **두 파일을 같이 고칠 것** |
| 참값 트랙 · 맵 배경 | `track.enabled: false`, `map.package: ""` | 코드는 남아 있다. 켜려면 yaml 만 채우면 된다 |

`/steering` 은 rad 다 (`kau_control/src/steer_controller_node.cpp:66`).
`ControlDebug.cmd_steer_deg` 만 deg 이며, 플롯은 deg 로 통일한다 (`steering_rad: true`).
