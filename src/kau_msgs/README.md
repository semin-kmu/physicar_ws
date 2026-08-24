# kau_msgs

팀 공통 ROS2 message package.

**단위: 길이 cm / 각도 deg / 속도 m/s**

## 구성

| message | 용도 |
| --- | --- |
| `msg/KauPath.msg` | 함수형 경로 — quintic Bezier segment 열. Global / Local / Lane 공용 |
| `msg/ObstacleCircle.msg` | 원형 근사 장애물 1개. 길이 단위 **m** (KauPath 와 다름) |
| `msg/ObstacleCircleArray.msg` | 한 관측 주기의 Object List + status |
| `msg/SteerDebug.msg` | `steer_controller` 내부 상태. 디버깅 전용, 제어 미사용 |

표준 `nav_msgs/Path` 는 `PoseStamped[]` 이산 좌표 집합이므로 대전제 위반 → custom message

## 빌드

```bash
# 컨테이너 안, /ws
colcon build --packages-select kau_msgs
source install/setup.bash
ros2 interface show kau_msgs/msg/KauPath
```

## 사용

`package.xml` 에 `<depend>kau_msgs</depend>`, C++ 는 추가로 `ament_target_dependencies(... kau_msgs)`

```python
from kau_msgs.msg import KauPath
```
```cpp
#include "kau_msgs/msg/kau_path.hpp"
```

## 규약

| 항목 | 값 |
| --- | --- |
| 발행 차수 | `degree = 5` (저차는 degree elevation 후 발행, 무손실) |
| 배열 길이 | `len(ctrl_x) == len(ctrl_y) == len(seg_length) * (degree+1)`, `len(seg_kappa_max) == len(seg_length)` |
| `num_segments` | 필드 없음. `len(seg_length)` 에서 유도 |
| `source` | `SRC_GLOBAL` / `SRC_LOCAL` / `SRC_LANE` 상수 |
| `valid_length` | Lane Detection 전용, 그 외 0 |

발행 절차 §9.5 · 수신 복원 §9.6 · 무결성 검사 §9.7~9.8 · QoS 와 topic §9.9 는 문서 참조

## topic

| topic | QoS | 주기 |
| --- | --- | --- |
| `/path/global` | TRANSIENT_LOCAL, RELIABLE, depth 1 | latched |
| `/path/local` | RELIABLE, depth 1 | 2~10 Hz |
| `/lane/center` | RELIABLE, depth 1 | 10 Hz |

RViz 는 custom message 미표시 → 시각화는 `/viz/path/*` (`nav_msgs/Path`, BEST_EFFORT) 로 분리 발행
