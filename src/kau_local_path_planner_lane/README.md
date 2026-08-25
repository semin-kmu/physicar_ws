# kau_local_path_planner_lane

Local Path Planner (lane-only). `/lane/center` 로 backbone 생성 → 후보
quintic Bezier 생성 → 장애물/도로/곡률 제약 필터링 → 비용 최소 선택 →
`/path/local` 발행.

**map / global path / localization 을 쓰지 않는다** (2026-08-25 재설계).
경로 형상의 유일한 출처는 `/lane/center` 이고, `/lane/left`,`/lane/right`
는 도로 경계 판정에만 쓴다. 따라서 이 패키지의 어떤 튜닝도 lane detection
품질보다 나은 결과를 못 만든다.

원본은 `KAU_AMET_Test` 리포의 Python 시뮬레이션
(`src/kau_local_path_planner_lane/test/planner.py`, `config.py`)이며, 이 패키지는
그 알고리즘을 ROS2 Jazzy + C++ 로 그대로 이식한 것이다. 곡선 수학
(quintic Bezier 평가, 최근접점, 곡률, 호길이) 자체는 새로 짜지 않고
`kau_control` 을 그대로 재사용한다.

## 아키텍처

```
include/kau_local_path_planner_lane/
├── types.hpp               PlannerParams, Candidate, PlanResult
├── bezier_ext.hpp           hermite_to_bezier / kappa_bound / Frame 평가
│                            (kau_control 에 없는, 생산자 전용 연산)
├── lane_backbone.hpp        /lane/center 로 backbone 생성 + 등곡률 외삽
│                            (옛 reference_fusion.hpp 를 대체)
├── boundary_checker.hpp     /lane/left,/lane/right 로 도로 경계 hard constraint
├── collision_checker.hpp    장애물 충돌/여유 계산
├── candidate_generator.hpp  후보 quintic 생성 (corridor 7개 + fallback)
├── path_evaluator.hpp       비용함수 + degraded 최후수단
└── local_planner.hpp        전체 오케스트레이션 (plan())

src/local_planner_node.cpp   ROS2 wiring (구독/발행/파라미터) 만 담당. TF 안 씀
```

`local_planner.hpp`/`LocalPlanner` 는 ROS 의존성이 전혀 없어 노드 없이도
단위/통합 테스트가 가능하다.

## Python → C++ 대응

| Python (`planner.py`) | C++ |
|---|---|
| `LocalPlanner._build_backbone` / `_extend_curve` | `lane_backbone.hpp` |
| `_candidate` / `_corridor_offsets` / `_obstacle_offset_candidates`(Smart K1) / `_obstacle_primitives` / `_direct_family` | `CandidateGenerator` |
| `_cost` / `_continuity_cost` / `_path_preview_cost` / `_least_violation` | `path_evaluator.hpp` |
| `_clearance` / `_near` / `_preview_clear` | `collision_checker.hpp` |
| `_road_ok` / `_road_clearance` / `_road_coordinates` | `boundary_checker.hpp` (알고리즘 재설계, 아래 참조) |
| `plan()` | `LocalPlanner::plan()` |

### 원본과 의도적으로 다른 점

- **도로 경계**: Python 은 `track.py` 의 divider-relative 사전계산
  corridor 를 썼다. 이 패키지는 map 폴리곤도 divider 정렬 데이터도 쓰지
  않고, 검출된 `/lane/left`,`/lane/right` **곡선 자체**로 판정한다
  (`boundary_checker.hpp`). corridor 폭은 기준점에서 법선 방향으로 edge
  곡선에 최근접 투영해 구한다 (`marginAlongNormal`, 반복 없이 정확).
  한쪽 edge 만 관측되면 반대쪽은 `kLaneWidthCm`(70cm) 로 근사한다.
- **장애물이 매 사이클 바뀐다**: Python 시뮬은 장애물이 ground-truth로
  고정이었지만, 실제 `/perception/obstacles` 는 매 사이클 바뀌는 라이브
  토픽이다. `LocalPlanner::updateObstacles()` 로 매 사이클 갱신한다.
- `_direct_target_indices` 의 "장애물이 어느 쪽에 있는가" 판정은 원래
  divider 기준 절대좌표를 다시 구했는데, 이미 station 계산 시점에 구해둔
  backbone 기준 lateral (`ObstacleStation.lateral`, 원본 `_station()`과
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
  hard-gate 두 곳(도로 판정 → `roadReportWheels`, 장애물 판정 →
  `clearanceRect`, 최후수단 → `leastViolationRect`)에서, 상수 반경(`body_radius_cm`, 3분할 원 근사 -
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
  가능.
  (당시 추가한 gtest 2개는 옛 폴리곤 API 를 쓰던 파일에 들어 있어
  2026-08-26 에 파일째 삭제됐다 — 아래 "테스트 현황" 참고.)
### 2026-08-25 곡선 구간 미추종 수정

합성 원호 폐루프(제어기가 발행 경로를 100% 따라간다고 가정, 40틱 x 10cm)로
재현했더니 직선은 횡오차 0.03cm 인데 **곡선은 어느 반지름이든 2m 까지 단조
발산**했다. 원인 세 가지가 겹쳐 있었다.

1. **`odom_topic` 기본값이 존재하지 않는 토픽**(`/odometry/filtered`)이었다.
   실제 발행자는 physicar_bringup 의 ekf 이고 토픽은 `/odom` 이다
   (`kau_gui/README.md` 142: "`/odometry/filtered` 는 존재하지 않는다").
   odom 이 없으면 `plan()` 의 `odom_delta` 가 항상 `nullopt` 라
   `previous_path_` 가 한 번도 재정렬되지 않고, 그러면 (a) `computeAnchor`
   의 최근접점이 매 틱 s≈0 에 머물러 **kappa0 가 자기가 직전에 쓴 값을 다시
   읽는다 — 0 에서 영원히 안 움직인다**(코너 한복판에서도 "지금 직진 중"으로
   계획한다), (b) continuity 항(`w_continuity=10.0`, 최대 가중치)이 옛 ego
   frame 기준으로 채점돼 매 틱 코너 바깥으로 끌어당긴다. 실측: R=100cm 에서
   ok 7/40 · 횡오차 206cm → odom 연결만으로 40/40 · 19cm.
   노드가 odom 미수신 시 5 초마다 WARN 을 찍도록 했다.
2. **`l_plan` 300 → 180** (위 튜닝값 절 참고).
3. **`extendCurve` 의 등곡률 외삽이 틀렸다.** tail 점을 시작점에서 *호길이*
   `remain` 만큼 떨어진 곳에 놓았는데, 등곡률 원호의 현 길이는
   `2*sin(dth/2)/kappa` 다. 비는 `1/sinc(dth/2)` 라 17° 에서 +0.4% 지만
   126° (R=60cm, remain=132cm) 에서 **+23%** 다. 게다가 회전각이 얼마든
   quintic 세그먼트 **1 개**로 붙였다. 그 결과 backbone 이 최대 22.6cm(R=60)
   벗어나고 곡률이 0.01839 → 0.02255 로 부풀어 차량 한계(0.020221)를
   넘겼다 — 그 순간 전 후보가 `kappa_bound` 로 탈락하고 `leastViolation` 이
   R=24cm 짜리 경로를 발행한다(최소회전반경 49.5cm, 물리적으로 못 따라감).
   정확한 chord + 45°/세그먼트 분할로 고쳤다: 기하오차 22.6cm → 0.1cm.
   회귀 방지 gtest 는 `test/test_lane_backbone.cpp`.

세 개를 다 적용한 뒤 폐루프 실측 (ok틱수 / 최대 횡오차, 40틱):

| 코너 R | 수정 전 | 수정 후 |
|---|---|---|
| 직선 | 40 / 0.03cm | 40 / 0.00cm |
| 150cm | — | 40 / 3.6cm |
| 100cm | 7 / **206cm** | 40 / 2.6cm |
| 80cm | 2 / **215cm** | 40 / 2.6cm |
| 70cm | — | 40 / 3.0cm |
| 60cm | 0 / **214cm** | 17 / 37cm (미해결, 아래 참고) |

**남은 한계**: R=60cm 코너는 여전히 못 넘는다. l_plan 180cm 로 R=60 을 돌면
172° 회전인데 corridor knot 3 개로는 그 형상을 표현하지 못한다. knot 수를
늘리거나 l_plan 을 곡률에 따라 가변으로 두는 게 맞는 방향이고, 이번 수정
범위 밖이다. 트랙 경계 폴리곤 실측 코너 반경이 0.5~0.9m 라 R=60 은 실제로
나오는 값이다.

- **토픽 두절 대응** (lane-only 기준으로 갱신, 2026-08-26):
  `/lane/center` 가 끊기면 노드가 마지막 메시지를 계속 들고 있다가
  (`lane_center_` 캐시) 그것도 무효면 `buildBackbone` 이 `nullopt` 를
  내고, `plan()` 은 직전 committed path 를 그대로 재발행하며 `kDegraded`
  로 표시한다. `/perception/obstacles` 는 `latest_obstacles_` 캐시가 마지막
  관측을 유지한다. `/odom` 이 끊기면 `odom_delta` 가 `nullopt` 라 무보정
  으로 돌지만 **곡선에서 횡오차가 발산하므로** 노드가 5초마다 WARN 을
  찍는다. Global Path 두절 항목은 그 입력 자체가 없어져 삭제했다.

## 인터페이스

| 방향 | 토픽 | 타입 | QoS | 쓰임 |
|---|---|---|---|---|
| 구독 | `/lane/center` | `kau_msgs/KauPath` (base_link) | RELIABLE, depth 1 | **경로 형상의 유일한 출처.** 없으면 계획 불가 |
| 구독 | `/lane/left` | `kau_msgs/KauPath` (base_link) | RELIABLE, depth 1 | 도로 좌 경계 (형상엔 기여 안 함) |
| 구독 | `/lane/right` | `kau_msgs/KauPath` (base_link) | RELIABLE, depth 1 | 도로 우 경계 |
| 구독 | `/perception/obstacles` | `kau_msgs/ObstacleCircleArray` | BEST_EFFORT, depth 1, volatile | 회피 대상 (m -> cm 변환은 노드가) |
| 구독 | `/odom` | `nav_msgs/Odometry` | BEST_EFFORT, depth 1 | **절대 위치가 아니라** 두 틱 사이 상대 변위만 |
| 발행 | `/path/local` | `kau_msgs/KauPath` (`source=SRC_LOCAL`) | RELIABLE, depth 1 | base_footprint, plan_hz |
| 발행 | `/viz/path/local` | `nav_msgs/Path` | RELIABLE, depth 1 | RViz 용 폴리라인 (cm -> m) |

TF 를 전혀 안 본다. map / global path / localization 없이 돈다.

파라미터는 `config/local_planner.yaml` 하나로 관리한다. 비용 가중치
(`w_*`)는 이 파일만 바꾸면 재빌드 없이 바로 반영된다.

2026-08-25: `/steering` 구독을 없앴다. P0 곡률을 제어기 출력에서 만들면
플래너 입력이 자기 출력의 함수가 되고(폐루프), Pure Pursuit 의 delta 는
경로 곡률이 아니며, ±20deg clamp 가 곡률 상한을 직접 건드린다 --
`kappa(0) == kappa0` 가 정확히 성립하므로 조향이 19.07deg 를 넘는 순간
모든 후보가 `kappa_bound` 로 탈락했다. 이제 이전 계획 경로에서 직접
구한다 (`config/local_planner.yaml` 의 P0 앵커 항목).

## 알려진 제약 (후속 확인 필요)

- **`/lane/*` 가 `base_link` 로 온다고 가정한다** (frame 검사를 안 한다).
  `kau_lane_detection` 의 `path_frame_id` 기본값이 `"map"` 이므로 그쪽이
  잘못 설정되면 절대좌표를 자차 상대좌표로 읽는다. 하류의
  `kau_path_arbiter` 는 이 경우를 거부하지만 이 노드는 아직 안 막는다.
- **`/steering` 의 부호 규약**(좌회전 +)이 `kau_control`/`pure_pursuit` 과
  일치하는지 실측 확인 필요. 실차 전 필수 확인 항목.
- **R=60cm 코너를 못 넘는다** (위 표 참고). corridor knot 3 개로는 172°
  회전 형상을 표현하지 못한다. 트랙 경계 실측 코너가 0.5~0.9m 라 실제로
  나오는 값이다.
- **`kCommittedHorizonCm`(52.422, stage-1) 과 `validated_horizon_cm`
  (yaml, stage-2) 이 같은 개념을 다른 값으로 쓴다.** 통합하면 동작이
  바뀌므로 계측 후에 손댈 것.

## KAU_AMET_Test 세션에서 확정한 튜닝값 (config/local_planner.yaml 참고)

- `l_plan=180` (2026-08-25, 300 에서 변경): **Planning Horizon 은 Observation
  Horizon 을 넘으면 안 된다.** 300 은 global path 를 쓰던 옛 패키지 값인데,
  이 패키지의 backbone 은 `/lane/center` 하나뿐이고 그 관측 길이는 168cm 다.
  corridor knot 이 `0.6/0.8/1.0 * l_plan` 이므로 300 이면 knot 3 개가
  180/240/300cm — 전부 관측 밖이라, 경로 모양을 정하는 점 전체가 외삽 위에
  놓였다. 합성 원호 폐루프 실측에서 180 이 전 R 구간 유일한 40/40 이다.
  190 이상은 R=70cm 코너에서 40/40 → 14/40 으로 절벽처럼 무너진다.
  자세한 표는 `config/local_planner.yaml` 의 `l_plan` 주석 참고.
- `preview` (450): **2026-08-26 삭제.** backbone 이 정확히 `l_plan` 에서
  끝나므로 "종점 너머 preview 구간" 에 드는 장애물이 구조적으로 존재할 수
  없어, `previewClear` 가 항상 상한값만 돌려주고 있었다 (열린 곡선의
  `deltaS(s0+l_plan, station_s) <= 0`). global path(임의 길이)를 참조로
  쓰던 시절의 값이다.
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
colcon build --packages-select kau_local_path_planner_lane
ros2 launch kau_local_path_planner_lane local_planner.launch.py use_sim_time:=false
```

### 단독 검증 (팀 다른 노드 없이)

`fake_upstream.py` 가 노드가 실제로 구독하는 5 개 토픽을 전부 낸다 --
`/lane/center`,`/lane/left`,`/lane/right`,`/perception/obstacles`,`/odom`.
TF 를 안 쓰므로 `static_transform_publisher` 는 필요 없다.

```bash
# R=100cm 코너를 0.5m/s 로 도는 상황 + 앞 120cm 에 콘 하나
ros2 run kau_local_path_planner_lane fake_upstream.py \
    --shape circle --radius 100 --obstacle 1.2,0.0

ros2 launch kau_local_path_planner_lane local_planner.launch.py use_sim_time:=false
ros2 run kau_local_path_planner_lane rejection_logger.py
```

주요 인자: `--shape straight`, `--radius`(+ 가 좌회전), `--speed`,
`--observed`(관측 길이, 기본 168), `--half-width`(노면 반폭, 기본 35),
`--no-edges`(한쪽 edge 폴백 경로 확인).

로그 한 줄에서 봐야 할 것:

```
plan st=0 d=+0.5 k0=+0.01000 kmax=0.01221 obs=+999.0 wheels_on=4
     road_full=+22.7 road_cmt=+23.1 e=0.1 a=0.00 vl=55 alive=6 13.2ms
     │       │      │            │                              │
     │       │      │            └ 채택 경로 최대 곡률           └ 계산 시간
     │       │      └ 현재 곡률. R=100 코너면 0.01 이어야 한다.
     │       │        코너인데 0 이면 odom 이 안 붙은 것이다.
     │       └ 선택된 횡오프셋 [cm]
     └ 0=ok  1=degraded  2=no_feasible  3=degenerate
```

`odom(...) 수신 없음` WARN 이 보이면 다른 건 볼 것도 없다 -- 그 상태로는
코너에서 횡오차가 발산한다.

### 테스트 현황

```bash
colcon test --packages-select kau_local_path_planner_lane
```

활성 gtest 는 2 개뿐이다 (`test_boundary_checker_lane`, `test_lane_backbone`).
둘 다 실제로 겪은 회귀를 잡아 두려고 만든 것이다 -- 도로 이탈 판정이 통째로
무력했던 건, 그리고 등곡률 외삽이 호길이를 현 길이로 쓰던 건.

2026-08-26 에 옛 패키지에서 복사해 온 gtest 5 개(42 TEST)를 삭제했다.
lane-only 재설계 후 **컴파일조차 안 되는 상태로 꺼져 있었고**, 그러면서
"테스트가 있다" 는 착시만 만들었다. 무커버리지 구간은 `CMakeLists.txt` 의
`BUILD_TESTING` 블록 주석에 적어 뒀다.
