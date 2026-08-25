# kau_gui

주행 디버깅 GUI. **노드 상태 + map + 추종오차 플롯을 한 화면에.**

화면 규약은 `KAU_AMET_Test/src/full_simulation.py` 의 `show_realtime` 과 같다.
시뮬과 실차를 나란히 놓고 보는 일이 많으므로 **같은 색이 같은 것을 가리킨다.**

> 상태: **작성 완료, 실행 미검증.** 실차(Ubuntu 24.04 / ROS 2 Jazzy)에서 확인 필요.
>
> 최종 갱신: 2026-08-25

## 1. 화면

`full_simulation.show_realtime` 과 같은 배치다.

```
┌──────────────────────────────────────────────────┐
│ 노드 상태 띠 (가로 전체)                          │
├─────────────────┬────────────────────────────────┤
│ speed  [m/s]    │  map (rowspan 4)               │
│ steer  [deg]    │   scan · TF · obstacle         │
│ CTE    [cm]     │   global / local / lane        │
│ θ_err  [deg]    │                                │
└─────────────────┴────────────────────────────────┘
     95          :               125
```

시뮬과 다를 수밖에 없는 것은 하나뿐이다. 시뮬은 배치 재생이라 전체 로그를
옅은 회색으로 미리 깔고 커서를 옮기지만 실시간에는 "미래" 가 없다. 그래서
회색 밑그림과 커서 InfiniteLine 이 빠진다.

### 노드 상태 띠

셀 하나 = 노드 하나. 좌측부터 **이름 · 상태등 · Hz**.

| 색 | 뜻 |
| --- | --- |
| 초록 | 그래프에 있고 대표 토픽이 기대 주기대로 온다 |
| 빨강 | 노드는 살아 있는데 토픽이 끊겼다 |
| 회색 | 그래프에 없다 (안 떴거나 죽었다) |

감시 대상은 `kau_state_machine` 의 `bringup.yaml` 을 읽어 정한다 — `run.sh` 가
띄우는 노드의 정의가 그 파일 하나뿐이라 목록을 GUI 에 복사해 두지 않는다.
`when`(sim/real) 은 `use_sim_time` 으로 거른다.

제외 대상

- `wait: true` — 관문. 스스로 끝나는 게 정상이다 (`clock_gate`)
- `ros: false` — 그래프에 이름이 없어 판정 불가 (`platform_ekf_pause`)

Hz 는 `status.watch` 에 적은 대표 토픽에서 잰다. 없는 노드는 그래프 존재만으로
판정하고 Hz 칸을 비운다 (`/map` 처럼 latched 이거나 발행이 없는 노드).

`option.<이름>` 스위치 값은 supervisor 파라미터라 관측 전용인 GUI 가 물어볼 수
없다. `status.options` 에 `run.sh` 설정과 맞춰 적는다.

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
| 배경 | 회색조 | `kau_localization` 의 `maps/<name>.yaml` + pgm |

TF 는 세 프레임의 축 마커 + 부모·자식 연결선(점선)으로 그린다. 2D 탑뷰라
z 축(파랑)은 생략한다.

맵 배경은 `kau_localization` 이 share 에 설치하는 `maps/` 에서 `map.name`
(기본 `kau_v3`) 으로 찾는다. `bringup.yaml` 이 `map_server` 에 주는 지도와 같다.
`map.yaml_path` 를 주면 그쪽이 우선한다.

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

ros2 launch kau_gui gui.launch.py
```

launch 가 `config/gui.yaml` 을 물린다. `ros2 run` 으로 직접 띄우면 파일을
손으로 줘야 하고, 빠뜨리면 코드 기본값으로 조용히 돈다:

```bash
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
| 휠 / 드래그 | 줌 / 팬 (pyqtgraph 기본) |

## 4. 구독 토픽

**관측 전용.** publisher · service client 가 하나도 없다 (`09` 계약 104·128).

| 용도 | 토픽 | 타입 |
| --- | --- | --- |
| 스캔 | `/scan_filtered` | `sensor_msgs/LaserScan` |
| 장애물 | `/perception/obstacle_markers` | `visualization_msgs/MarkerArray` |
| 경로 3 종 | `/viz/path/{global,local,lane}` | `nav_msgs/Path` |
| target speed | `/speed` | `std_msgs/Float64` |
| real speed | `/odom` | `nav_msgs/Odometry` |
| cmd steer | `/steering` | `std_msgs/Float64` [rad] → deg |
| raw steer · heading err · cte | `/debug/steer` | `kau_msgs/SteerDebug` |

`/odometry/filtered` 는 **존재하지 않는다.** `bringup.yaml` 이 `/odom` 으로 remap 한다.

상태 띠는 위와 별개로 `status.watch` 의 대표 토픽을 하나씩 더 구독한다.
`raw=True` 라 역직렬화하지 않고 **수신 시각만 찍는다** — 주기 계산에 메시지
내용이 필요 없으므로 무거운 토픽이라도 비용이 붙지 않는다.

## 5. 설계 규약

| 규약 | 이유 |
| --- | --- |
| 맵 계열은 최신값 latch, **플롯 계열은 링버퍼에 전량 append** | 렌더가 10 Hz 라고 50 Hz 신호를 10 Hz 로 샘플링하면 조향 떨림·cte 스파이크가 사라진다 |
| 시간축은 GUI 노드 clock 기준 수신 시각 | 발행 노드마다 sim/system clock 이 섞이면 축이 어긋난다 |
| 스캔은 TF 실패 시 갱신하지 않고 늙게 둔다 | 낡은 점군을 최신인 척 그리는 것이 가장 위험하다 |
| 색은 `viz.py` 한 곳 | 흩어 두면 범례와 실제 선 색이 어긋난다 |

## 6. 미확정

| 항목 | 현재 | 필요 작업 |
| --- | --- | --- |
| 실행 검증 | **미검증** | 실차에서 확인 |
| 실행 위치 | 미정 | `09` 문서는 노트북 전용. 차량에서 띄우면 렌더 부하가 주행에 얹힌다 |
| 맵 배경 | `kau_localization` 의 pgm | `/map` 토픽은 구독하지 않는다 |
