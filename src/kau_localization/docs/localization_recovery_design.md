# Localization 방어·자동 복구 로직 설계 초안

> 상태: 구현 전 설계 메모  
> 대상: `kau_localization`의 Cartographer 2D pure localization  
> 목적: localization 이상을 감지하고, 마지막 정상 위치를 이용해 사람의
> `/initialpose` 입력 없이 안전하게 재측위한다.

## 1. 배경

Gazebo에서 차량을 드래그해 순간이동시키면 다음 상태가 동시에 발생한다.

- Gazebo의 실제 차량 pose는 새 위치로 즉시 바뀐다.
- EKF의 `/odom`은 이전 위치에서 계속되거나 불연속적으로 변한다.
- Cartographer는 `/odom`을 scan matching prior로 사용한다.
- `/scan`은 순간이동한 새 위치의 관측을 즉시 제공한다.

따라서 odometry prior와 LiDAR 관측이 서로 모순되어 localization이 잘못된
위치로 정합되거나 회복하지 못할 수 있다. 실차가 정상 운용 중 순간이동하지는
않지만, 충돌, 미끄러짐, 바퀴 헛돎, LiDAR 가림, 반복 구조 오정합, EKF 또는
laser odometry 재시작에서도 유사한 문제가 발생할 수 있다.

이 문서에서 Gazebo 드래그는 정상 운용 시나리오가 아니라 강제
`kidnapped robot`/재측위 시험으로 취급한다.

## 2. 핵심 원칙

복구 로직은 다음 세 문제를 분리한다.

1. **이상 감지**: 현재 localization을 신뢰할 수 있는가?
2. **복구 pose 생성**: 어느 위치와 방향을 initial pose 후보로 사용할 것인가?
3. **복구 검증**: trajectory 시작이 아니라 실제 정합 수렴을 어떻게 확인할 것인가?

추가 원칙은 다음과 같다.

- lane 이탈이나 scan 이상 하나만으로 trajectory를 재시작하지 않는다.
- 고정된 "N프레임 전"이 아니라 **마지막 검증 완료 pose**를 사용한다.
- 재측위 전에 차량을 정지시킨다.
- `/start_trajectory` 서비스 성공과 localization 수렴 성공을 구분한다.
- 로컬 후보가 실패하면 global path 기반 후보와 전역 탐색으로 범위를 넓힌다.
- 복구 후 검증 구간에서는 저속으로 주행한다.

## 3. 현재 시스템 전제

현재 TF 소유권은 다음과 같다.

```text
map ──────────────► odom              Cartographer
odom ─────────────► base_footprint    robot_localization EKF
base_footprint ───► base_link ─────── robot_state_publisher
```

Cartographer 입력:

- `/scan` (`sensor_msgs/msg/LaserScan`)
- `/odom` (`nav_msgs/msg/Odometry`, EKF 출력)
- TF `base_footprint -> lidar_link`

EKF는 laser odometry와 `/imu`를 융합한다. Cartographer는 `/imu`를 직접
사용하지 않으며 `use_imu_data = false`이다.

현재 `initial_pose_relay`는 다음 수동 입력 경로만 제공한다.

```text
/initialpose
  -> /get_trajectory_states
  -> ACTIVE trajectory에 /finish_trajectory
  -> /start_trajectory(use_initial_pose=true,
                       relative_to_trajectory_id=0)
```

현재 relay의 "재측위 완료" 로그는 새 trajectory가 시작되었다는 뜻일 뿐,
scan matching이 올바른 위치에 수렴했다는 보장은 아니다.

## 4. 제안 컴포넌트

기존 relay에 모든 기능을 넣지 않고 책임을 분리한다.

```text
localization_monitor
  ├─ scan/odom/imu/lane/TF 일관성 감시
  ├─ localization health 계산
  └─ 마지막 정상 pose ring buffer 유지

localization_recovery_manager
  ├─ 정지 요청 및 정차 확인
  ├─ initial pose 후보 생성
  ├─ trajectory 재시작 요청
  ├─ 후보별 수렴 검증
  └─ 정상/저속/고장 상태 전환

initial_pose_relay
  └─ Cartographer trajectory 서비스 호출 어댑터
```

`localization_monitor`와 `localization_recovery_manager`를 하나의 노드로 먼저
구현할 수도 있지만, 센서 판정과 시스템 복구 상태는 논리적으로 분리해 둔다.

## 5. 이상 감지 신호

### 5.1 명령과 odometry의 불일치

정지 또는 매우 낮은 속도 명령인데 pose가 크게 바뀌는지 검사한다.

```text
|commanded_speed| < v_stop
AND
(delta_odom_position > d_jump OR delta_odom_yaw > yaw_jump)
```

반대로 주행 명령과 odometry 이동은 있는데 map 기준 pose가 멈추거나 반대
방향으로 점프하는 경우도 검사한다.

임계값은 차량 최대 속도, 제어 주기, rosbag 실측값으로 정한다. 문서의 수치는
확정값으로 간주하지 않는다.

### 5.2 IMU와 odometry의 회전 불일치

짧은 시간 구간에서 다음 값을 비교한다.

```text
delta_yaw_imu  = integral(imu.angular_velocity.z * dt)
delta_yaw_odom = angle_diff(yaw_odom_now, yaw_odom_start)
residual       = abs(delta_yaw_imu - delta_yaw_odom)
```

residual이 임계값 이상으로 일정 시간 지속되면 이상 점수를 높인다. IMU 하나만으로
차량이 들렸다고 확정하지 않는다.

### 5.3 물리적으로 불가능한 pose 변화

TF `map -> base_footprint`의 연속 샘플에서 암시 속도를 계산한다.

```text
v_implied        = distance(pose_now, pose_prev) / dt
yaw_rate_implied = angle_diff(yaw_now, yaw_prev) / dt
```

차량 물리 한계를 여유 계수까지 포함해 넘으면 localization invalid 후보로 본다.
이 신호는 차량 이동과 Cartographer 오정합을 구분하지는 못하므로 다른 신호와
조합한다.

### 5.4 LiDAR 메시지 기본 유효성

각 `/scan`에서 다음을 검사한다.

- `header.frame_id`가 기대 프레임인지
- timestamp가 증가하며 현재 시각에서 지나치게 오래되지 않았는지
- `ranges.size()`가 기대 범위인지
- angle/range 메타데이터가 유한하고 일관적인지
- NaN/Inf 및 range 밖 측정 비율
- 유효 거리점 개수와 전체 대비 비율
- 전후좌우 sector별 유효점 분포

토픽이 존재하거나 메시지가 도착한다는 사실만으로 scan을 정상으로 판정하지 않는다.

### 5.5 LiDAR 정합 품질

가능하면 다음 정보를 사용한다.

- scan matcher score 또는 residual
- inlier 비율
- 최적화 전후 pose correction 크기
- 최근 scan 대비 ICP residual

Cartographer가 필요한 품질 지표를 외부로 제공하지 않으면 다음 중 하나를 검토한다.

1. Cartographer metrics/diagnostics에서 취득
2. Cartographer wrapper 수정
3. 감시 전용 lightweight scan-to-scan 또는 scan-to-map 검증기 구현

`/scan_matched_points2` 발행 여부만으로는 올바른 정합을 보장할 수 없다.

### 5.6 Lane 이탈

Lane 이탈은 localization 이상 신호 중 하나이며 단독 재시작 조건이 아니다.
원인은 localization, 조향 제어, path, lane detection, 장애물 회피, 미끄러짐 등
여러 가지일 수 있다.

권장 조건 형태:

```text
lane_departure_persistent
AND
(
  map_pose_jump
  OR scan_matching_bad
  OR odom_imu_inconsistent
  OR global_path_pose_inconsistent
)
```

Lane 이탈만 있고 localization 일관성이 정상이면 감속 및 차선 복귀 제어를 먼저
수행한다.

## 6. Health 판정과 히스테리시스

각 신호를 즉시 hard fault로 바꾸지 않고 다음과 같이 분류한다.

- `GOOD`: pose를 복구 기준으로 저장할 수 있음
- `SUSPECT`: 일시적 이상. 감속하며 추가 관찰
- `INVALID`: localization 사용 금지 및 복구 필요

판정에는 지속시간과 진입/해제 임계값을 다르게 둔다.

```text
GOOD -> SUSPECT       하나 이상의 이상 신호 지속
SUSPECT -> GOOD       정상 신호가 recovery_clear_time 동안 지속
SUSPECT -> INVALID    복수 신호 또는 hard jump가 confirm_time 동안 지속
```

초기 튜닝값 예시는 다음과 같으며 반드시 rosbag으로 검증한다.

- soft 이상 확정 지속시간: 0.3~0.5초
- 정상 복귀 확인시간: 1~2초
- hard physical jump: 즉시 감속/정지하되 복구 판단은 추가 검증

## 7. 마지막 정상 pose 저장

최근 10초 정도의 ring buffer를 시간 기준으로 유지한다.

```cpp
struct PoseSnapshot {
  rclcpp::Time stamp;
  geometry_msgs::msg::Pose map_pose;
  geometry_msgs::msg::Pose odom_pose;
  double lane_error;
  double scan_quality;
  double localization_quality;
  double speed;
  bool lane_valid;
  bool scan_valid;
  bool odom_imu_consistent;
};
```

저장 시각에는 동일하거나 보간 가능한 timestamp의 TF와 센서 값을 사용한다.
현재 시각의 서로 다른 timestamp 메시지를 단순 조합하지 않는다.

복구 기준 pose는 다음 조건으로 선택한다.

```text
timestamp < suspect_start - safety_margin
AND scan_valid
AND localization_quality >= threshold
AND no_pose_jump
AND odom_imu_consistent
AND (가능하면 lane_valid)
```

고정 프레임 수 대신 시간과 품질 조건을 사용한다. 초기 검토값:

- ring buffer: 10초
- 저장 주기: 10~20 Hz
- `safety_margin`: 이상 시작 전 약 0.5초

## 8. Initial pose 후보 생성

### 8.1 1순위: 마지막 정상 pose에서 odometry 전파

마지막 정상 시각 이후 odometry가 유효하다면 다음 후보를 만든다.

```text
T_map_base(candidate_now)
  = T_map_base(last_good)
  * inverse(T_odom_base(last_good))
  * T_odom_base(now)
```

이는 이상 감지와 실제 정차 사이에 이동한 거리를 반영한다.

odom reset, 순간 점프, IMU 불일치 등으로 odometry 자체를 신뢰할 수 없으면 이
후보를 사용하지 않는다.

### 8.2 2순위: 마지막 정상 map pose

odometry가 무효이면 마지막 정상 `map -> base_footprint` pose를 그대로 사용한다.
차량이 실제로 먼 거리 이동한 경우에는 실패할 수 있다.

### 8.3 3순위: global path 기반 후보

마지막 정상 pose에 대응하는 경로 진행도 `s_last`를 함께 저장한다. 예상 이동거리와
트랙 진행 방향을 이용해 다음 후보들을 생성한다.

- `s_last + expected_distance`의 경로 pose
- 그 전후 일정 거리의 경로 pose
- lane tangent를 기준으로 작은 yaw 편차를 둔 pose
- 필요한 경우 진행 방향 반대 후보

후보의 위치, yaw 및 시도 횟수는 트랙 중복 구조에서 오정합하지 않도록 제한한다.

### 8.4 최종 fallback

로컬 및 path 후보가 모두 실패하면 다음 정책 중 시스템 수준에서 선택한다.

- Cartographer global localization 탐색
- `lane_only` 저속 모드
- 안전 정지 후 운영자 개입
- 시스템 `FAULT`

## 9. 복구 상태 머신

```text
TRACKING
  정상 주행, good pose 저장
    |
    +-- 이상 신호 지속 --> SUSPECT

SUSPECT
  감속 및 센서 일관성 재확인
    |-- 회복 ----------------------> TRACKING
    +-- 이상 확정 -----------------> STOPPING

STOPPING
  정지 명령, 실제 정차 확인
    +------------------------------> RELOCALIZING_LOCAL

RELOCALIZING_LOCAL
  odom 전파 후보 -> 마지막 정상 pose
    |-- 후보 수렴 -----------------> VERIFYING
    +-- 모두 실패 -----------------> RELOCALIZING_PATH

RELOCALIZING_PATH
  global path 주변 후보 순차 시험
    |-- 후보 수렴 -----------------> VERIFYING
    +-- 모두 실패 -----------------> GLOBAL_SEARCH/LANE_ONLY/FAULT

VERIFYING
  정지 상태 검증 후 저속 주행 검증
    |-- 성공 ----------------------> TRACKING
    +-- 실패 ----------------------> 다음 후보 또는 FAULT
```

복구 중에는 외부 planner/controller가 오래된 global pose를 계속 사용하지 않도록
localization validity 상태를 명시적으로 전달해야 한다.

## 10. 후보 수렴 검증

`/start_trajectory` 응답 성공은 다음 trajectory ID가 만들어졌다는 뜻이다.
각 후보에 대해 1~3초 정도 관찰하며 다음을 검증한다.

- TF `map -> odom -> base_footprint`가 연속적으로 존재
- `map -> odom` 변화가 검증 구간 동안 안정
- `/scan` 기본 및 공간 품질 정상
- scan matching score/residual 정상
- pose가 트랙 주행 가능 영역 내부
- pose heading이 global path/lane tangent와 허용 범위 내에서 일치
- lane lateral error가 허용 범위
- 암시 속도와 yaw rate가 물리 한계 이내

정지 상태 검증 성공 후 제한 속도로 짧게 이동하며 동적 검증을 수행한다. 그 후에만
정상 속도를 허용한다.

## 11. `initial_pose_relay` 확장 방향

현재 relay는 RViz 수동 입력용이므로 자동 복구 구현 시 다음 인터페이스를 검토한다.

```text
/relocalize (custom service)
request:
  geometry_msgs/Pose initial_pose
response:
  bool trajectory_started
  int32 trajectory_id
  string message
```

relay의 책임은 trajectory 시작 성공까지로 제한한다. 실제 수렴 여부는 recovery
manager가 판단한다.

필요한 보완 사항:

- 서비스 요청과 요청 ID/trajectory ID 연결
- 처리 중 요청을 단순 폐기하지 않는 명시적 busy 응답
- timeout과 Cartographer status code 외부 전달
- frozen map trajectory ID를 0으로 고정하는 전제 검증
- shutdown 중 worker thread와 pending request 안전 처리

`PoseWithCovarianceStamped`의 covariance는 현재 relay가 무시한다. 후보 탐색 범위를
나타내는 데 활용하려면 Cartographer 서비스 동작과 별도의 탐색 정책을 검토한다.

## 12. 안전 요구사항

- 재측위 시작 전에 명령 속도를 0으로 만들고 실제 정차를 확인한다.
- localization `INVALID` 동안 정상 global path 추종을 금지한다.
- recovery loop의 최대 후보 수와 최대 시간을 제한한다.
- 반복 실패 시 무한 재시작하지 않고 `lane_only` 또는 `FAULT`로 전환한다.
- 한 시점에 `map -> odom` 발행자는 하나만 존재해야 한다.
- Cartographer만 재시작하고 EKF odom 원점은 유지되는지 실측한다.
- EKF/laser odometry가 재시작되어 odom 원점이 바뀌면 저장한 변환의 유효성을
  폐기하고 별도 복구 경로를 사용한다.

## 13. 구현 전 수집할 데이터

정상 주행과 다음 고장 주입 시나리오를 rosbag으로 기록한다.

1. 정상 직선, 코너, 정지 및 재출발
2. Gazebo 차량 위치/방향 순간이동
3. `/scan` 일시 중단, 지연, NaN/Inf 증가 및 LiDAR 가림
4. `/odom` 점프 또는 laser odometry 재시작
5. `/imu` 지연 및 yaw-rate 불일치
6. lane detection 일시 상실
7. 실제 lane 이탈과 localization만 잘못된 가상 lane 이탈
8. 반복 벽 구간의 Cartographer 오정합

최소 기록 대상:

```text
/scan
/odom
/odom/laser (실제 토픽명 확인 필요)
/imu
/tf
/tf_static
/clock
차량 속도/조향 명령
lane center/error 및 validity
global/local path
Cartographer diagnostics/metrics
시스템 상태와 recovery event
```

rosbag에서 정상 분포와 고장 분포를 비교한 뒤 임계값을 정한다. 임계값을 코드에
하드코딩하지 않고 ROS parameter로 노출한다.

## 14. 단계별 구현 계획

### 단계 1: 관측만 하는 monitor

- 어떤 제어도 하지 않고 health 신호와 판단 근거를 발행한다.
- ring buffer와 마지막 정상 pose를 기록한다.
- rosbag 재생으로 false positive/negative를 측정한다.

### 단계 2: 수동 트리거 복구

- 차량이 정지한 상태에서 서비스/명령으로 자동 후보 생성을 실행한다.
- 1순위와 2순위 후보, 수렴 검증을 구현한다.
- RViz 2D Pose Estimate와 결과를 비교한다.

### 단계 3: global path 후보

- pose를 path 진행도 `s`에 투영한다.
- 주변 후보 생성, 순차 시험, 최대 시간 제한을 구현한다.

### 단계 4: 자동 상태 전환

- supervisor/controller와 정지 및 validity 계약을 연결한다.
- `SUSPECT -> STOPPING -> RELOCALIZING -> VERIFYING`을 자동화한다.
- 반복 실패 fallback을 구현한다.

### 단계 5: 실차 검증

- 저속, 넓은 안전 공간에서 시작한다.
- 센서 가림과 작은 미끄러짐부터 검증한다.
- 실제 차량을 들어 옮기는 시험은 안전 절차와 운영자 개입 하에서만 수행한다.

## 15. 구현 시작 시 우선 확인할 미정 사항

- 실제 laser odometry 노드명과 출력 토픽명
- EKF 입력 설정, 출력 주기, odom 원점 재시작 동작
- 차량 명령 및 실제 속도 토픽
- lane error/validity 토픽과 frame 의미
- global path에서 pose와 진행도 `s`를 변환하는 기존 API
- Cartographer scan matching 품질을 외부에서 읽을 수 있는 방법
- supervisor에 localization validity와 정지 요청을 연결할 인터페이스
- 트랙에서 허용 가능한 위치 및 heading 오차
- recovery 최대 시간과 대회 규정상 허용 동작

## 16. 완료 기준

다음 조건을 만족하면 1차 구현 완료로 본다.

- 정상 주행 rosbag에서 불필요한 recovery가 발생하지 않는다.
- 순간적인 lane/scan 이상은 `SUSPECT` 후 정상 복귀한다.
- 명백한 pose jump에서 차량이 먼저 정지한다.
- 마지막 정상 pose 기반 복구가 자동으로 수행된다.
- trajectory 시작과 실제 수렴 성공이 별도로 기록된다.
- 잘못된 후보를 성공으로 판정하지 않는다.
- 제한 횟수 실패 후 안전한 fallback으로 전환한다.
- 모든 판정 원인, 후보 pose, 결과와 소요 시간이 로그/diagnostic에 남는다.

