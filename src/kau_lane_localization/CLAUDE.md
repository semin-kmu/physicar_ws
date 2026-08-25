# kau_lane_localization

차선 검출 기반 2D 측위. **라이다를 쓰지 않는다.**

이 문서는 설계 근거와 실측값이다. 코드에는 "무엇을 하는지"만 짧게 남긴다.

---

## 1. 왜 만드는가

2D 라이다 높이가 트랙 펜스 높이와 겹쳐 오검출이 잦다. 라이다 측위가
성립하지 않을 경우의 대체 경로다. `kau_localization`(AMCL / Cartographer)은
그대로 두고 **택일**한다.

---

## 2. TF 소유권

```
map ──────────────► odom            lane_localization_node   (이 패키지)
odom ─────────────► base_footprint  dead_reckoning_node      (이 패키지)
base_footprint ───► base_link ─►…   robot_state_publisher
```

`map -> odom` 발행자는 동시에 하나만 떠야 한다. `kau_localization` 의
`amcl.launch.py` / `cartographer_localization.launch.py` 와 **동시 실행 금지**다.

플랫폼 `ekf_filter_node` 는 정지시킨다. `robot_localization` 을 쓰지 않는
이유는 §4 에 있다.

---

## 3. 관측 가능성 — 차선은 3-DOF 중 2개만 준다

| 성분 | 출처 | 성질 |
| ---- | ---- | ---- |
| `d` 횡오프셋 | 차선 검출 | 매 프레임, 강함 |
| `psi` 헤딩 | 차선 검출 | 매 프레임, 강함 |
| `s` 종방향 | 추측항법 + 코너 리셋 | **직선 구간에서 관측 불가** |

`s` 는 코너에서만 관측된다. 그래서 코너 이벤트로 리셋하고, 그 사이는
`/speed` 적분으로 버틴다. 누적 오차 상한은 **최장 직선 하나**로 묶인다.

| `/speed` 오차 | 9.49 m 직선 끝 `s` 오차 |
| ------------- | ---------------------- |
| 1 % | 9.5 cm |
| 2 % | **18.9 cm** |
| 5 % | 47.4 cm |

pan 피드포워드 요구치 ±20 cm 안이다.

**차선 하나만 보여도 `d`·`psi` 는 관측된다.** 그 선의 맵상 오프셋을 알기
때문이다 (`road_map.hpp` 의 `LINES`). 측위 유효 조건을 검출 유효 조건보다
느슨하게 잡는다.

---

## 4. robot_localization 을 쓰지 않는다

각 DOF 마다 소스가 하나뿐이라 융합할 것이 없다.

```
종방향  /speed        하나
각속도  /imu gyro-z   하나
```

`ekf_node` 는 15-state 3D 필터다. 평평한 트랙에 z / roll / pitch / 가속도
상태 13개가 낭비고, 별도 프로세스 + DDS 왕복이 붙는다. `dead_reckoning_node`
는 3줄 적분 + 3x3 공분산 전파가 전부다.

**IMU 는 gyro-z 만 쓴다.** 대회 측 답변: 실물 `/imu` 는 각속도·가속도·지자기만
주고 자세각은 없다 (`orientation_covariance[0] = -1`). `kau_localization/config/ekf.yaml`
실측대로, IMU yaw 를 절대 관측으로 켜면 "항상 yaw=0" 인 가짜 측정을 먹어
추정 방위가 0 으로 끌려간다 (오차 14 %). 절대 방위는 **차선 `psi` 하나로
통일**한다.

지자기는 모터 근처 + 실내라 신뢰하지 않는다.

---

## 5. `/speed` — 지금은 순수 피드포워드다

`kau_control/config/speed_controller.yaml`:

```yaml
feedback_topic: "/odometry/filtered"   # 실재하지 않는 토픽 (EKF 는 /odom 으로 remap)
require_feedback: false                # 피드백 없으면 PID 보정 없이 v_ref 만 발행
```

따라서 `/speed` 에 라이다가 섞여 있지 않다. 명령속도와 추정속도가
가감속·전압강하에서 일치했다는 관측은 **차량이 실제로 명령을 잘 추종한다는
증거**이지 PID 가 맞춰준 결과가 아니다.

### 주의 — feedback_topic 을 이 패키지의 `/odom` 에 연결하면 안 된다

`dead_reckoning_node` 가 `/speed` 를 적분해 `/odom` 을 만든다. 거기에
PID 피드백을 걸면 `v_meas == v_cmd` 가 항등적으로 성립해 **PID 가 조용히
무력화**되고, 실제 추종 오차는 보정도 관측도 안 된다. 겉으로는 완벽해 보이는
것이 최악이다.

독립 측정이 필요하면 dash 오도미터(§6)를 쓴다.

---

## 6. dash 오도미터 — 유일한 독립 종방향 측정

라이다가 빠지면 `/speed` 를 검증할 수단이 없어진다. 슬립이 생겨도 아무도
모른다. 황색 점선이 그 역할을 한다.

`amet2026_track.json` 의 `layers/center_line` 실측 (dash 폴리곤 306개):

| 항목 | 값 |
| ---- | -- |
| dash 길이 | 5.0 cm |
| dash 폭 | 2.5 cm |
| **dash pitch** | **9.96 cm** |
| 검증 | 306 x 0.0996 = 30.5 m = 루프 길이 |

BEV 종방향 축척 기준 dash 20 px / gap 20 px, 시야 90 cm 에 약 9개가 동시에
보인다. 프레임 간 dash 열의 1차원 상관으로 변위를 잰다.

**노면 텍스처가 없으므로 일반 BEV 시각 오도메트리는 성립하지 않는다.**
흰 실선은 종방향으로 균일해 구멍 문제(aperture problem)에 걸리고, 노면은
매끈하다. 종방향 신호를 주는 특징은 dash 경계뿐이다.

---

## 7. 트랙 테이블

`scripts/gen_track_table.py` 가 `kau_global_path/config/amet2026_track.json`
에서 `config/track_amet2026.yaml` 을 만든다. **생성물은 커밋한다.** 런타임에
`/sim/api` 를 호출하지 않으므로 시뮬과 실차가 같은 파일을 읽는다.

### 좌표계 — 두 번 뒤집힌다

**sim -> map**: json 의 `note` 대로 `map_x = 3.68 - sim_y`, `map_y = sim_x - 1.39`.
야코비안 `[[0,-1],[1,0]]` 은 det +1 이라 회전각 부호는 보존된다.

**json 점 순서 -> 주행 방향**: **반대다.** `lane_graph.yaml`(주행 경로, map
프레임)과 signed area 부호를 비교해 판정한다.

```
signed_area  center = -35.506    lane_graph = +35.295    -> direction = -1
```

뒤집으면 `turn_deg` 부호가 뒤집히고 `run_in` 은 이웃의 `run_out` 이 된다.
결과적으로 주행 방향 기준 `turn_deg` 합이 **+360.0** 으로 닫힌다.

### 곡률은 primitives 가 아니라 방향각 차분으로 구한다

`primitives` 의 `kappa` 는 값이 정확하지만 **적분이 안 맞는다.** 원본이
10 cm 간격인데 코너 반경은 6 cm 라, 호의 kappa 를 현(chord) 길이에 곱하면
실제 호보다 긴 구간에 얹힌다.

| 방식 | `∫kappa ds` |
| ---- | ----------- |
| primitives | **+483 deg** |
| 방향각 차분 + 이동평균 | **+360.000 deg** |

폐곡선에서 `sum(d_theta)` 는 정확히 `2*pi` 로 망원합이 되고, 이동평균은
적분을 보존한다. 코너 첨두는 뭉개지지만(`|kappa|max` 13.0 -> 7.2) 원본
해상도가 애초에 6 cm 반경을 담지 못한다. Frenet 보정도 pan 피드포워드도
첨두보다 적분이 중요하다.

### 검증 (2026-08-26)

| 항목 | 결과 |
| ---- | ---- |
| `∫kappa ds` | **+360.000 deg** |
| 코너별 `∫kappa ds` vs `turn_deg` | 17개 전부 **±2 deg 이내** |
| `lane_graph` -> centerline 거리 | med 7.6 mm |
| 코너 `s` 차분 vs `run_in` | 불일치 없음 (5 cm 기준) |
| bbox | centerline / lane_graph 5 cm 이내 일치 |

---

## 8. 코너 시그니처 — 전역 재측위

차선만으로는 전역 재측위가 불가능하다. 닫힌 고리이고 직선 구간이 전부
똑같이 생겼다. **코너 시퀀스가 유일한 수단이다.**

`(run_in_m, turn_deg)` 순환 수열의 유일성:

| 허용오차 | 코너 1개 | **코너 2개** |
| -------- | -------- | ------------ |
| ±0.15 m / ±10 deg | 모호 1쌍 | **유일** |
| ±0.30 m / ±15 deg | 모호 4쌍 | **유일** |
| ±0.50 m / ±20 deg | 모호 12쌍 | 모호 1쌍 |

**코너 2개면 트랙 어디서든 확정된다.** 재측위 비용은 최악 13.78 m
(9.49 m 직선 시작점), 평균 3.59 m.

종료 조건은 거리가 아니라 **정보량**으로 잡는다 — 상관 피크 대 차순위 피크
비(PSR)가 임계를 넘을 때까지. 직선에서는 상관이 평평해 계속 대기하고,
코너에 들어가면 즉시 확정된다.

### 미해결 — 코너 정의 일원화

위 유일성은 `skeleton` 기준 코너 정의에서 성립한다. **런타임 검출이 다른
정의를 쓰면 무의미해진다.** 맵 생성기와 측위 노드가 같은 파라미터를 쓰고,
재생 평가로 어긋남을 잡아야 한다.

---

## 9. Frenet 보정 — 코너에서 지배적이다

`s` 는 황색 중앙선 기준이고 차량은 그 옆을 달린다.

```
ds_center = v * dt / (1 - kappa * d)
```

`lane_graph`(global path)는 중앙선에서 med 7.6 mm 라 사실상 중앙선 위를
달린다. 그래도 보정을 뺄 수 없다 — `|kappa|max` 7.2 에서 `d` 가 5 cm 만
벗어나도 계수가 1.56 이 된다.

**`1 - kappa*d` 가 0 에 가까워지면 발산한다. 반드시 클램프한다.**

---

## 10. 알려진 미해결

- 코너 정의 일원화 (§8) — 최대 리스크
- `LaneObservation` 메시지가 아직 없다. `kau_lane_detection` 이 `d`·`psi`·
  `kappa` 를 발행하지 않는다 (내부에서 계산만 한다, `d_cm` 은 L4043)
- EMA 이전 원 측정값 경로가 없다. EMA 를 거친 값을 다시 KF 에 넣으면
  지연이 두 번 쌓이고 공분산이 거짓말을 한다
- camera pan 이 BEV 외부파라미터를 바꾸는데 서보 인코더가 없어
  `/joint_states` 가 명령의 에코다. pan 과도구간에는 측정을 기각해야 한다
- 들어올림 감지 미구현. 평평한 트랙 주행 중 `|a| ~ g` 가 깨지는 것으로 잡는다
