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
| `_candidate` / `_corridor_offsets` / `_obstacle_offset_candidates`(Smart K1) / `_obstacle_primitives` / `_direct_family` | `CandidateGenerator` |
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

### 2026-08-24 KAU_AMET_Test 알고리즘 반영 (이번 포팅 세션)

- **corridor 0.625L reference knot**: corridor 가 2-frame(0.25L, 1.0L)에서
  3-frame(0.25L, 0.625L, 1.0L)으로 확장됐다 — 긴 2-segment chord 가 도로
  굴곡을 한 번에 가로질러 `road_boundary` 를 자주 위반하던 문제 완화
  (KAU_AMET_Test 1-lap 실측: road-valid 전멸 100→45). `corridorOffsets`/
  `candidate()` 를 N-frame(가변 길이 `std::vector<Frame>`/
  `std::vector<double>`) 지원하도록 일반화했다. obstacle_offsets/
  primitives/direct_family/K2 는 기존 2-frame `frames` 그대로.
- **Smart K1** (`_obstacle_offset_candidates` → `CandidateGenerator::
  obstacleOffsetCandidates`): 장애물 회피 시 middle knot(K1) 의
  longitudinal 위치를 장애물 station 기준 lead/ratio 4-combo 격자로
  탐색해 hard-check 통과하는 cost 최소 후보를 고른다. corridor 7개 +
  이 2개 = 9개로 여전히 "새 candidate family" 가 아니라 기존 후보군에
  합류하는 것 (6-combo → 4-combo 로 축소된 최신 버전 — KAU_AMET_Test
  세션 실측: 두 조합(leads[1]×ratio=1.0, leads[2]×ratio=0.82)이 selected/
  loo_change 전부 0 이라 제거해도 결과 완전 동일, 계산량 -33%).
- **`_least_violation` 버그 수정**: 이전 포팅은 장애물/도로/곡률 위반을
  동일 가중치로 정규화 합산했는데, 이는 KAU_AMET_Test 세션에서 이미
  "`obs_margin` 을 키울수록 오히려 collision 이 잦아지는" 안전 버그로
  판명되어 폐기된 방식이었다 (정규화 분모가 커지며 상대적 obstacle
  penalty 가 작아지는 부작용). `leastViolation()` 을 obstacle_violation
  최소 후보군으로 먼저 좁힌 뒤 그 안에서 road+kappa 위반 최소를 고르는
  현재 Python 로직으로 맞췄다 — 장애물 충돌은 항상 최우선으로 회피.
- **Committed/Prediction Horizon** (`_committed_segment_count`/`plan()`
  stage-2 → `committedSegmentCount()`/`CandidateGenerator::candidate()`
  + `LocalPlanner::plan()` 최종 선택 루프): 400cm(`l_plan`) 전체 경로에
  road/curvature hard constraint 를 적용하면, 실제로는 다음 재계획
  전까지 쓰지도 않을 먼 미래 구간의 위반 때문에 정상 실행 가능한
  candidate 까지 대량 false-rejection 되는 문제가 있었다
  (KAU_AMET_Test 1-lap 실측: degraded 148개 중 84개=56.8%가 "실행 구간은
  완전 valid, 먼 미래 구간만 invalid"). `kCommittedHorizonCm=52.422cm`
  (=2×실측 최대 재계획 주기당 이동거리, 5Hz) 이내는 그대로 hard
  reject 하고, 그 이후(prediction 구간)에서만의 kappa/road 위반은
  `kPredictionViolationPenalty=200.0` cost 페널티만 부여해 후보 pool 에
  남긴다(다른 완전-valid 후보가 있으면 그쪽이 낮은 cost 로 우선
  선택됨). **obstacle clearance 는 이 구분과 무관하게 항상 엄격
  유지**(안전 최우선, 완화하지 않음). `LocalPlanner::plan()` 최종
  선택 루프(stage-2, 14차 근 exact 검사)도 동일하게 committed 기준으로
  재검증한다. 1-lap dynamic+EMA 실측(baseline→적용 후): ok/degraded
  62/148→102/107, kappa_bound 실패 1015→621(-39%), 실제 vehicle
  boundary violation 503→403 rows(-20%), collision 없음 유지,
  min_clearance +7.989→+7.138cm, steer-rate 오히려 개선(mean/p95/max
  18.51/57.57/222.38→17.01/52.36/222.38 deg/s).
- **회전 사각형 차체 (3분할 원 근사 대체, 2026-08-25)**: road/obstacle
  hard-gate 두 곳(`roadClearance`/`roadOk` → `roadClearanceRect`/
  `roadOkRect`, `clearance` → `clearanceRect`, `leastViolation` →
  `leastViolationRect`)에서, 상수 반경(`body_radius_cm`, 3분할 원 근사 -
  candidate 생성의 offset 목표값 계산에는 그대로 남아있음)을 실제 차체
  치수(28x20cm, 후륜축 기준, `VehicleFootprint{body_front_cm=23,
  rear_overhang_cm=5, half_width_cm=10}`, 새 파라미터 `body_front_cm`/
  `rear_overhang_cm`/`half_width_cm`)를 그대로 반영한 회전 사각형으로
  교체했다. 곡선 구간에서 원 근사가 놓치던 실제 차체 전방부 초과분을
  없앤다.
  KAU_AMET_Test(Python)에서 먼저 검증: 회전 사각형 4 꼭짓점을 독립적으로
  divider 기준 nearest-point 재탐색에 넣었더니 곡률이 큰 구간에서 Frenet
  근사가 왜곡돼(꼭짓점이 centerline 밖 오프셋 점이라 재탐색 결과가 진짜
  위치와 다른 arc-length 로 튐) vehicle boundary violation 실측이
  오히려 악화(403→466 rows)됐고, "centerline 점만 1번 탐색 + 사각형은
  그 결과 위에 평행이동으로 얹는" 우회로 421 rows 까지 개선했지만
  baseline(403)에는 못 미쳤다(steer-rate max 도 222→400 로 악화). 다만
  C++ 쪽은 이 문제가 원천적으로 없다 — `pointClearance`가 임의의 (x,y)
  점을 실제 폴리곤과 직접 비교하는 절대 기하 계산이라 Frenet 근사/
  nearest-point 재탐색을 전혀 거치지 않으므로, 사각형 4 꼭짓점을
  그대로(정확한 heading 회전으로 계산) 각 `pointClearance` 에 넣으면
  된다 — Python 이 겪은 근사 왜곡 없이 더 정확하고 더 단순하게 구현
  가능. gtest 2개 추가(`test_boundary_checker.cpp`/
  `test_collision_checker.cpp`, 원 근사로는 놓치는 위반을 사각형이
  잡아내는지 확인). 61→63 tests, 0 errors, 0 failures.
- **토픽 예외처리**: KAU_AMET_Test 의 "Lane/Global/Object 두절" 3가지
  대응(`plan()` docstring)이 이미 이 포트에 구조적으로 반영돼 있음을
  이번에 확인했다 (추가 수정 불필요) — Lane 은 `lane_curve_` 가
  `std::optional` 이라 두절 시 자연히 `nullptr` 로 전파(fuse 로직이 이미
  global-only 로 폴백); Global Path 는 `TRANSIENT_LOCAL` QoS 로 latched
  라 ROS 구조상 "이번 사이클만 두절"이라는 개념 자체가 성립하지 않음
  (`planner_` 가 최초 1회 생성된 뒤로는 항상 유효); Object 는
  `latest_obstacles_` 캐시가 새 메시지 없으면 마지막 관측을 그대로
  유지해 `updateObstacles()` 가 매 사이클 그 캐시를 재사용한다.

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
- `_least_violation`(degraded 최후수단)은 obstacle_violation 최소인
  후보군으로 먼저 좁힌 뒤 그 안에서 road+kappa 위반 최소를 고른다 (장애물
  충돌 항상 최우선, 2026-08-24 세션에 이전 포팅의 정규화-합산 버그를
  수정 — 위 "2026-08-24 KAU_AMET_Test 알고리즘 반영" 참고). 세 조건을
  동시에 만족하는 후보를 못 찾는 게 자주 발생한다면, 이 채점 방식이
  아니라 candidate 생성/탐색 범위(l_plan, corridor, preview, segment 수)
  를 먼저 의심할 것.

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
