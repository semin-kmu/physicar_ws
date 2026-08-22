# kau_local_path_planner

Local Path Planner. reference(global path + lane detection) 융합 → 후보
quintic Bezier 생성 → 장애물/도로/곡률 제약 필터링 → 비용 최소 선택 →
`/path/local` 발행.

원본은 `KAU_AMET_Test` 리포의 Python 시뮬레이션
(`src/kau_local_path_planner/test/planner.py`, `config.py`)이며, 이 패키지는
그 알고리즘을 ROS2 Jazzy + C++ 로 그대로 이식한 것이다. 곡선 수학
(quintic Bezier 평가, 최근접점, 곡률, 호길이) 자체는 새로 짜지 않고
`kau_control` 을 그대로 재사용한다.

## 아키텍처

```
include/kau_local_path_planner/
├── types.hpp               PlannerParams, Candidate, PlanResult
├── bezier_ext.hpp           hermite_to_bezier / kappa_bound / Frame 평가
│                            (kau_control 에 없는, 생산자 전용 연산)
├── reference_fusion.hpp     투영 + lane 종점 상태 융합
├── boundary_checker.hpp     도로 inner/outer 경계 hard constraint
├── collision_checker.hpp    장애물 충돌/여유 계산
├── candidate_generator.hpp  후보 quintic 생성 (corridor 7개 + fallback)
├── path_evaluator.hpp       비용함수 + degraded 최후수단
└── local_planner.hpp        전체 오케스트레이션 (plan())

src/local_planner_node.cpp   ROS2 wiring (구독/발행/TF/파라미터) 만 담당
```

`local_planner.hpp`/`LocalPlanner` 는 ROS 의존성이 전혀 없어 노드 없이도
단위/통합 테스트가 가능하다.

## Python → C++ 대응

| Python (`planner.py`) | C++ |
|---|---|
| `LocalPlanner._project` / `_ref_frame` | `ReferenceFusion` |
| `_candidate` / `_corridor_offsets` / `_obstacle_primitives` / `_direct_family` | `CandidateGenerator` |
| `_cost` / `_continuity_cost` / `_path_preview_cost` / `_least_violation` | `path_evaluator.hpp` |
| `_clearance` / `_near` / `_preview_clear` | `collision_checker.hpp` |
| `_road_ok` / `_road_clearance` / `_road_coordinates` | `boundary_checker.hpp` (알고리즘 재설계, 아래 참조) |
| `plan()` | `LocalPlanner::plan()` |

### 원본과 의도적으로 다른 점

- **도로 경계**: Python 은 `track.py` 의 divider-relative 사전계산
  corridor 를 썼다. ROS2 쪽엔 그 divider 정렬 데이터가 없어서,
  `kau_object_detection` 이 이미 쓰는 point-in-polygon + 거리 방식으로
  재설계했다 (`kau_object_detection/config/amet_2026_track.yaml` 그대로
  재사용, 복제하지 않음). corridor 폭은 기준점에서 법선 방향 이분탐색
  (`marginAlongNormal`)으로 구한다.
- **장애물이 매 사이클 바뀐다**: Python 시뮬은 장애물이 ground-truth로
  고정이었지만, 실제 `/perception/obstacles` 는 매 사이클 바뀌는 라이브
  토픽이다. `LocalPlanner::updateObstacles()` 로 매 사이클 갱신한다.
- `_direct_target_indices` 의 "장애물이 어느 쪽에 있는가" 판정은 원래
  divider 기준 절대좌표를 다시 구했는데, 이미 station 계산 시점에 구해둔
  global-path 기준 lateral (`ObstacleStation.lateral`, 원본 `_station()`과
  동일 정의)을 재사용한다 (부호만 필요하므로 실질적으로 동등).

## 인터페이스

| 방향 | 토픽 | 타입 | QoS |
|---|---|---|---|
| 구독 | `/path/global` | `kau_msgs/KauPath` | TRANSIENT_LOCAL, RELIABLE, depth 1 |
| 구독 | `/lane/center` | `kau_msgs/KauPath` | RELIABLE, depth 1 |
| 구독 | `/perception/obstacles` | `kau_msgs/ObstacleCircleArray` | BEST_EFFORT, depth 1, volatile |
| 구독 | `/steering` | `std_msgs/Float64` [rad] | 기본 |
| 구독 | TF `map -> base_footprint` | tf2 | — |
| 발행 | `/path/local` | `kau_msgs/KauPath` (`source=SRC_LOCAL`) | RELIABLE, depth 1 |

파라미터는 `config/local_planner.yaml` 하나로 관리한다. 비용 가중치
(`w_*`)는 이 파일만 바꾸면 재빌드 없이 바로 반영된다.

## 알려진 제약 (후속 확인 필요)

- `/path/global`, `/lane/center` 가 항상 `map` frame 으로 온다고 가정한다.
  Lane Detection 이 localization 상실로 `base_link` 로 강등하는 경우의
  frame 변환(§7.1, 제어점에 강체변환)은 아직 구현하지 않았다.
- `/steering` 의 부호 규약(좌회전 +)이 `kau_control`/`pure_pursuit` 과
  일치하는지 실측 확인 필요.
- `amet_2026_track.yaml` 좌표는 Gazebo world 프레임(m)이라 `map` 으로
  변환한다 (`sim_to_map_*` 파라미터, `kau_global_path` 확인값 재사용).
  회전(`rot_deg != 0`)은 아직 미구현.

## KAU_AMET_Test 세션에서 확정한 튜닝값 (config/local_planner.yaml 참고)

- `l_plan=300`: 400 이상으로 늘리면 2-segment 고정 구조와 충돌해 오히려
  악화됨 (실측 확인, 늘리지 말 것 — segment 수를 늘리는 게 맞는 방향).
- `preview=450`: `l_plan+preview=750cm` 총 예고거리. 스윕 결과 이 근방이
  최적, 600 이상은 오히려 진짜 물리적 min_clear 가 나빠짐.
- `w_lane=1.0`: heading jitter 원인으로 의심했으나 0~1.0 스윕해도 결과
  불변 확인 (원인 아님).
- `w_continuity=10.0`: switching 감소와 안전이 동시에 최적인 지점. 20
  이상부터 회피 반응이 지연되어 안전이 악화된다 (실측).
- `_least_violation`(degraded 최후수단)은 장애물/도로/곡률 위반 사이에
  우선순위를 두지 않고 정규화 합산한다 — 세 조건을 동시에 만족하는
  후보를 못 찾는 게 자주 발생한다면, 이 채점 방식이 아니라 candidate
  생성/탐색 범위(l_plan, corridor, preview, segment 수)를 먼저 의심할 것.

## 빌드 / 실행 / 테스트

```bash
colcon build --packages-select kau_local_path_planner
ros2 launch kau_local_path_planner local_planner.launch.py

# 단독 검증 (팀 다른 노드 없이)
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map base_footprint
ros2 run kau_local_path_planner fake_upstream.py --shape circle --radius 300
ros2 run kau_local_path_planner rejection_logger.py

colcon test --packages-select kau_local_path_planner
```
