# 06. Behavior State

Behavior 계층(초 단위 기동) 정의. Mission 이 지시하는 기저/이벤트 기동, Planner 프로파일과의 직교 조합, 전파 값(Gain·가중치·차선 이탈 허용) 규약.

> 전제: Behavior 는 `state_machine` 이 소유. Mission 상태·이벤트 판정은 `05`, System 전이는 `04` 참조. 본 문서는 인용만 하고 재정의하지 않음.

<br>

## 1. 범위

| 포함 | 미포함 (별도 문서) |
| --- | --- |
| Behavior 상태 4개 정의 | Mission 상태·이벤트 판정 로직 → `05` |
| 기저 기동 ↔ 이벤트 기동 매핑 | System 상태 전이 → `04` |
| Behavior × Planner 프로파일 직교 조합 | 프로파일 결정(노드 고장 기준) → `02` section 7-4 |
| 전파 값(Gain / 가중치 / 이탈 허용) 규약과 소비자 | 회피 후보 생성·비용함수 자체 → `kau_local_path_planner` |
| 채터링 방지 | 정지 채널 자체의 정의 → `01` section 4-3 |
| Behavior 전이표 | 메시지 정밀 스키마 최종 확정 → `07` |

- 회피 **방향**(좌/우 offset 부호) 선택은 `local_path_planner` 비용함수 소관(`경로_형식.md` section 6.4). 본 문서는 **허용 여부와 한계**만 규정

<br>

---

## 2. Behavior 상태 4개

| 상태 | 의미 |
| --- | --- |
| `HOLD` | 정지 유지 (출발 전 / 완주 후) |
| `LANE_KEEPING` | 차선 추종 주행 |
| `AVOID_OBSTACLE` | 라바콘 회피 기동 |
| `STOP_AND_WAIT` | 회피 경로 부재로 정지 |

**명명 규칙**. 기동은 **동사형**. Mission 이름(`READY`/`DRIVING`/`FINISHED`, `LANE_SECTION`, `OBSTACLE`)과 절대 중복 금지

**삭제됨 (규정상 부재, `Notes/State_Machine_확정.md` 대회 규칙)**

| 삭제 상태 | 사유 |
| --- | --- |
| `FREE_SPACE_FOLLOW` | 슬라롬 구간(`CONE_SECTION`) 없음: 라바콘은 구간이 아니라 회피 대상 (`05` 소관 이벤트) |
| `TURN` | 교차로 없음 |
| `PARK_MANEUVER` | 주차 없음 |

<br>

---

## 3. 기저 기동 vs 이벤트 기동

### 3-1. 기저 기동 · Mission 상태가 결정

| Mission 상태 | 기저 Behavior |
| --- | --- |
| `READY` | `HOLD` |
| `DRIVING` (`LANE_SECTION`) | `LANE_KEEPING` |
| `FINISHED` | `HOLD` |

### 3-2. 이벤트 기동 · 기저를 일시 대체

| Mission 이벤트 | 진입 조건 | Behavior |
| --- | --- | --- |
| `OBSTACLE` | 회피 경로 존재 (section 6) | `AVOID_OBSTACLE` |
| `OBSTACLE` | 회피 경로 부재(전폭 차단) | `STOP_AND_WAIT` |

- `OBSTACLE` 이벤트 자체의 확정/해제(N프레임, 신뢰도, 경로 간섭 판정)는 **`05` 소관**. 06 은 이미 확정된 이벤트만 입력으로 받음
- **이벤트 기동 종료 시 진입 시점의 기저 기동으로 자동 복귀**. 이벤트가 `OBSTACLE` 단독이므로 복귀 주소는 항상 `LANE_KEEPING`. 이벤트별 복귀 규칙 불필요
- `AVOID_OBSTACLE` ↔ `STOP_AND_WAIT` 는 **같은 이벤트 안에서의 내부 전이**. Mission 이벤트는 유지된 채 Behavior 만 회피 가능 여부에 따라 오감 (section 9)

<br>

---

## 4. Behavior × Planner 프로파일 · 직교 축 ⚠

**Behavior(무엇을 하는가, Mission 이 판단)와 프로파일(어떤 입력으로 하는가, 노드 고장이 결정, `02` section 7-4)은 서로 다른 축.**
Behavior 전이 로직은 현재 프로파일을 몰라도 동작해야 하고, 프로파일 전환은 Behavior 를 몰라도 동작해야 함.

| Behavior | `lane_centering` (평시) | `global_only` (`lane_detection` 상실) | `lane_only` (`localization` 상실) |
| --- | --- | --- | --- |
| `LANE_KEEPING` | O | O | O |
| `AVOID_OBSTACLE` | O | O (section 4-2 위험) | O (section 4-3 성립조건) |

- `HOLD` / `STOP_AND_WAIT` 는 **정지 상태**. 프로파일이 능동 제어에 영향을 주지 않음. 프로파일 값 자체는 유지되며 재개 시 그대로 사용

### 4-1. 이탈 판단 기준 · 통일 규칙

**`lane_detection` 이 살아있으면 차선(직접 관측) 기준, 죽었으면(`global_only`) Global Path(간접) 기준.**
기준이 프로파일 이름이 아니라 **관측 가능한 데이터**를 따르므로, `lane_centering` 과 `lane_only` 는 판단 기준이 같다.

| 프로파일 | 이탈 판단 기준 | 추종오차 예산 소모 | 잔여 회피 offset 여유 |
| --- | --- | --- | --- |
| `lane_centering` | 차선 (직접) | max\|cte\| **2.06 cm** (시뮬 실측, `경로_형식.md` section 6.7, `w_lane=1.0`) | 공식 산출 |
| `lane_only` | 차선 (직접, 상대좌표) | **미실측**: 관측 경로 동일하므로 `lane_centering` 과 유사 추정 | **확정 필요** |
| `global_only` | Global Path (간접) | max\|cte\| **16.22 cm** (시뮬 실측, `w_lane=0`, `02` section 7-4) | 공식 산출 |

```
허용 상한 = 주행가능 반폭 − 차체 반폭 10 cm − 현재 profile 추종오차 예산
```

> **`주행가능 반폭` 은 미확정.** 시뮬의 차선 반폭 35 cm 는 예시값이며, section 6-2 로 이탈이 반대 차선까지 열렸으므로
> 기준 자체가 차선 경계가 아니라 **트랙 주행가능 영역 경계**로 바뀜. 잔여 여유의 절대값은 실측 후에만 산출 가능
> (`08` `constants.drivable_half_width_cm`, `08` section 11 항목 11)

### 4-2. `global_only` × `AVOID_OBSTACLE` · 위험

**차선 이탈 없이 회피가 가능한가. 판정식**

```
회피 시 차량 중심 ↔ 라바콘 중심 최소거리   d = r_cone + collision_radius
차체 외측 도달점                          = d + 차체 반폭 + 경로 오차
이탈 없이 회피 가능 ⟺  차체 외측 도달점 ≤ 차선 반폭
```

| 기호 | 값 | 상태 |
| --- | --- | --- |
| `collision_radius` | 11.04 cm | **확정**: 대회측 차량 제원(280×200 mm, L 18 cm)에서 유도 |
| 차체 반폭 | 10 cm | **확정**: 차폭 200 mm |
| `r_cone` | ? | **미확정**: 시뮬의 7.5 cm 는 **예시값** |
| 차선 반폭 | ? | **미확정**: 시뮬의 35 cm(차선폭 70)는 **예시값** |
| 경로 오차 | `lane_centering` 2.06 / `global_only` 16.22 cm | **참고치**: 시뮬 트랙·오차 가정 하의 실측 (`src/README.md`) |

- **시뮬의 차선폭·장애물 크기로 충돌 판정을 내리지 않음.** 위 식만 확정이며, 결론은 실측값 대입 후
- 다만 `global_only` 의 경로 오차가 `lane_centering` 의 **약 8배**(16.22 / 2.06)라는 관계는 값과 무관하게 성립
  → 어떤 실측값이 오든 **`global_only` 가 먼저 이탈 필요 조건에 걸림**
- `global_only` 는 차선을 관측하지 못해 **어느 쪽으로 피해야 차선 안에 남는지 알 수 없음** → 최악을 가정할 수밖에 없음. 이것이 이 조합의 본질적 한계이며 **실측값과 무관**
- **대응 확정: 이탈 허용 범위를 반대 차선까지 연다** (section 6-2) → 위 판정에서 `차선 반폭` 대신 **트랙 주행가능 반폭**을 쓰게 되어 여유가 크게 늘어남
- `collision_radius` 는 3분할 원 근사값, 차체 반폭은 실제 폭 기준. **서로 다른 판정에 쓰이는 값**이므로 혼용 금지
- corridor margin 은 `경로_형식.md` section 6.8 미확정

<br>

### 4-3. `lane_only` × `AVOID_OBSTACLE` · 성립 조건

- **절대좌표(`map`) 없음**. Global Path 미사용, `02` section 7-4 그대로
- 라바콘 좌표는 `base_link` 상대 발행(`02` 계약 35) → 회피 자체는 `map` 없이 성립
- 이탈 판단은 section 4-1 규칙대로 **차선 기준(직접 관측)**. `lane_only` 진입 조건 자체가 이미 "차선 + 장애물 입력 생존"을 전제하므로 (`02` section 7-4 필수 입력 집합), `lane_only` 가 성립한 시점에 이 조합도 **자동으로 성립**. 별도 조건 불필요
- 단, `localization` **과** `lane_detection` 이 동시 상실이면 `FAULT`(`02` section 7-4 조합 규칙) → `enable` 차단 정지로 직행. 이 조합 자체가 발생하지 않음

<br>

---

## 5. 전파 값

| 값 | 소비 노드 | 토픽 (잠정, `07` 확정) |
| --- | --- | --- |
| 제어 Gain set (`smooth` / `aggressive`) | `steer_controller` | `/state_machine/behavior` |
| Local Path Planner 가중치 set (`normal` / `avoidance`) | `local_path_planner` | `/state_machine/behavior` |
| 차선 이탈 허용 여부 + 한계값(cm) | `local_path_planner` | `/state_machine/behavior` |

- **`/state_machine/speed_limit` 은 여기 포함되지 않음**. 정지 수단은 `01` section 4-3 확정대로 `/state_machine/enable` · `/state_machine/speed_limit` 2계층뿐. `STOP_AND_WAIT` 도 기존 `/state_machine/speed_limit` 채널을 재사용 (section 7). **새 채널 아님**
- **가중치 set 은 `02` section 7-4 프로파일과 이름이 겹치지 않게 명명** (`normal`/`avoidance` vs `lane_centering`/`global_only`/`lane_only`). 두 축을 코드에서도 혼동하지 않기 위함

### 5-1. `/state_machine/behavior` 메시지 (provisional, `07` 에서 확정)

```
std_msgs/Header header

uint8 HOLD=0 / LANE_KEEPING=1 / AVOID_OBSTACLE=2 / STOP_AND_WAIT=3
uint8 behavior

uint8 GAIN_SMOOTH=0 / GAIN_AGGRESSIVE=1
uint8 gain_set

uint8 WEIGHT_NORMAL=0 / WEIGHT_AVOIDANCE=1
uint8 weight_set

bool    lane_departure_allowed
float32 lane_departure_limit_cm   # allowed=true 일 때만 유효, section 4-1 공식으로 매 tick 산출
```

발행 주기 20 Hz. `state_machine` 평가 tick 안에서 (`01` section 14-1 / `02` 계약 19와 동일 원칙, 별도 타이머 금지)

### 5-2. 신규 결합 · `state_machine` → `local_path_planner` 구독

- `AVOID_OBSTACLE` ↔ `STOP_AND_WAIT` 판정을 위해 `state_machine` 이 **`/path/local` 를 구독**해야 함. `01`~`03` 에는 없던 결합 (`02` section 7-2 "최근접점을 남에게 발행하지 않음"과는 무관, 여기서 쓰는 것은 `s` 값이 아니라 section 9.8 무효 판정)
- 판정에 쓰는 것은 **경로 유효성**(`경로_형식.md` section 9.8: `len(seg_length)==0` → 무효) **단독**. 제어점 전체를 소비하지 않음
- **`02` 의 등급 판정(FAULT/DEGRADED)과 절대 혼동 금지**. 회피 후보가 기하학적으로 없는 것은 **정상 동작 중** 발생하는 상황이지 노드 고장이 아님. heartbeat `status` 로 보고하지 않음 (`02` section 7 원칙 위반 방지)

<br>

---

## 6. `AVOID_OBSTACLE` 상세

### 6-1. 차선 이탈 허용이 필요한 이유

- 라바콘이 차선 안에 놓여 있으면, 차선 내부에서만 회피할 여유가 항상 있는 것은 아님
- `차체 반폭 + r_cone + collision_radius` 가 차선 반폭을 넘는 배치가 존재할 수 있음 → **이탈 허용이 회피의 전제조건**
- 어떤 배치에서 실제로 넘는지는 `r_cone` · 차선 반폭 실측 후에만 판단 (section 12)

### 6-2. 이탈 허용 범위 · 확정

| 항목 | 확정 |
| --- | --- |
| 허용 범위 | **반대 차선까지 허용** |
| 근거 | 순회 트랙이라 **대향차가 존재하지 않음**. 회피 실패(정지)가 이탈보다 손해 |
| 한계 | 트랙 주행가능 영역(경계 밖)은 넘지 않는다 |
| 규정 확인 | 감점 규정 유무: 미확인 (section 12) |

- 이 확정으로 section 4-2 판정에서 기준이 `차선 반폭` → **트랙 주행가능 반폭**으로 바뀜
- `global_only` 에서도 회피 가능성이 크게 올라감. 단 차선 미관측 한계(section 4-2)는 그대로
- 이탈 허용은 **`AVOID_OBSTACLE` 중에만**. `LANE_KEEPING` 에서는 차선 유지가 목표

<br>

### 6-3. 제어 Gain 상향 · 근거와 위험

**근거**. 회피 offset 이 클수록 필요 곡률이 커짐 (`κ ∝ apex / L_plan²`). `경로_형식.md` section 6.5 기준 → 평시보다 빠른 응답 필요

> **`L_plan` 은 미확정.** `경로_형식.md` section 10 에 "Local Path 길이. 별도 논의" 로 열려 있음.
> 400 cm 는 section 6.5 의 **비교용 케이스**이지 확정 제원이 아니며, 같은 절의 채택값은 200 cm, 시뮬 통일값은 150 cm.
> 현재 검토 범위는 **1.5~5 m** (사용자 확정). 확정 시 이 절의 곡률 여유를 재계산해야 함

**위험. 진동**

| 항목 | 값 | 비고 |
| --- | --- | --- |
| 조향 각속도 한계 | 600 °/s | 여유 큼: 지배 요인 아님 (`경로_형식.md` 부록 B) |
| 조향 응답 시상수 τ | **0.1 s (잠정, 미회신)** | `경로_형식.md` 부록 B: "시상수가 지배적 제약으로 전환" |

- `aggressive` gain 튜닝은 **τ 확정 전까지 잠정**. τ 실측 전에 공격적으로 올리면 지연-이득 조합에서 오버슈트/진동 위험 (`경로_형식.md` section 10 τ 확정 우선순위 상향과 동일 근거)
- gain 전환은 계단이 아니라 **램프** (section 9, 계약 73). 진동 유발 요인을 전이 시점에서 한 번 더 차단

<br>

---

## 7. `STOP_AND_WAIT`

| 항목 | 규정 |
| --- | --- |
| 진입 조건 | `OBSTACLE` 이벤트 확정 **AND** `local_path_planner` 무효 경로 발행: **이탈 허용이 반대 차선까지 열린 뒤에도 후보가 없을 때**(section 6-2) = 사실상 **트랙 전폭 물리 차단** |
| 탈출 조건 A | 회피 후보 재확보 (`/path/local` 유효, 연속 확인 section 8) |
| 탈출 조건 B | `OBSTACLE` 이벤트 자체 해제 (라바콘이 경로에서 벗어남, `05` 판정) |
| 정지 수단 | **`/state_machine/speed_limit = 0`**. 다른 정지와 동일 채널 재사용 (`01` section 4-3), 새 채널 금지 |
| timeout | **없음: 무한 대기** (확정) |

**진입 빈도가 매우 낮아짐. 이탈 허용 확정의 결과**

- section 6-2 로 이탈이 **반대 차선까지** 열렸으므로, 회피 후보가 전부 사라지려면 **트랙 전폭이 물리적으로 막혀야** 함
- 대회 시나리오상 라바콘 6개는 개별 배치 → 전폭 차단은 **배치상 성립하기 어려움**
- 따라서 `STOP_AND_WAIT` 는 **거의 도달하지 않는 상태**. 그러나 삭제하지 않음. 도달했다는 것은 회피가 불가능하다는 뜻이고, 그때 필요한 동작은 정지뿐

**timeout 없음. 확정**

- 라바콘은 정적이므로 대기해도 상황이 바뀌지 않음 → timeout 을 둔들 할 수 있는 일이 없음
- 이탈 허용 확대는 **이미 진입 전에 시도**되었으므로 남은 수단이 없음
- `lane_only` 무한 주행 정책(`02` section 7-4)과 같은 방향: **스스로 포기 판정을 하지 않고 운영자 개입에 맡김**

<br>

---

## 8. 채터링 방지

- `OBSTACLE` 이벤트 자체의 debounce(N프레임 + 신뢰도)는 **`05` 소관**. 06 은 중복 구현하지 않음
- `AVOID_OBSTACLE` ↔ `STOP_AND_WAIT` 전이는 **06 자체 판단**(`/path/local` 유효성)이므로 자체 min dwell 필요

| 조건 | 확정 tick | 근거 |
| --- | --- | --- |
| 회피 경로 무효 확정 (`AVOID_OBSTACLE` → `STOP_AND_WAIT`) | 연속 **3 tick** | 안전 방향(정지)은 빠르게: `02` section 5-2 패턴 |
| 회피 경로 유효 복귀 (`STOP_AND_WAIT` → `AVOID_OBSTACLE`) | 연속 **10 tick** | 위험 방향(재개)은 느리게: 동일 패턴, 히스테리시스 |

- 20 Hz tick 기준 3 tick = 150 ms, 10 tick = 500 ms. `02` section 5-2 확정 값과 **정합**
- `LANE_KEEPING` → `AVOID_OBSTACLE` 진입, `AVOID_OBSTACLE`/`STOP_AND_WAIT` → `LANE_KEEPING` 복귀는 **`05` 의 이벤트 확정/해제 판정을 그대로 신뢰**. 06 에서 추가 dwell 두지 않음(이중 debounce 로 인한 반응 지연 방지)

<br>

---

## 9. 전이표

| from | to | guard | entry action | exit action | min dwell |
| --- | --- | --- | --- | --- | --- |
| `HOLD` | `LANE_KEEPING` | Mission `READY`→`DRIVING` | gain=smooth, weight=normal, departure=false | - | - (Mission 확정 전이 그대로) |
| `LANE_KEEPING` | `HOLD` | Mission `DRIVING`→`FINISHED` | gain=smooth | - | - |
| `LANE_KEEPING` | `AVOID_OBSTACLE` | `OBSTACLE` 확정(`05`) **AND** `/path/local` 유효 | gain=aggressive(램프), weight=avoidance, departure=true, limit=section 4-1 산출값 | - | - (`05` 확정을 신뢰) |
| `AVOID_OBSTACLE` | `LANE_KEEPING` | `OBSTACLE` 해제(`05`) | gain=smooth(램프), weight=normal, departure=false | - | - |
| `AVOID_OBSTACLE` | `STOP_AND_WAIT` | `/path/local` 무효 | `speed_limit = 0` **즉시** | - | 3 tick |
| `STOP_AND_WAIT` | `AVOID_OBSTACLE` | `/path/local` 유효 **AND** `OBSTACLE` 유지 | speed_limit 하향값 → 램프 복귀(`03` section 10-3 패턴), gain=aggressive | - | 10 tick |
| `STOP_AND_WAIT` | `LANE_KEEPING` | `OBSTACLE` 해제(`05`) | gain=smooth, speed_limit 복귀 | - | - |

- **System 계층 정지(Emergency/`speed_limit = 0` 정지, `03`)는 Behavior 전이와 독립**. 정지는 항상 `/state_machine/enable`·`/state_machine/speed_limit` 값으로 성립하며, Behavior 라벨이 정지를 유발하지 않음(`STOP_AND_WAIT` 제외, 이 상태 자체가 `/state_machine/speed_limit=0` 을 발행하는 주체)
- 노드 장애로 인한 정지 중에도 Behavior 라벨은 **마지막 값을 유지**. 복구 후 Mission 상태에 따라 자연 갱신 (강제 전이 규칙 불필요, `03` section 12 "Behavior 상태: 리셋 → 기저 기동"과 정합, `state_machine` 자체 재시작 시에만 리셋)
- 우선순위: `System > Mission > Behavior`. 상위 정지 지시가 있으면 Behavior 전이 자체가 의미를 잃음 (표에 별도 행 불필요)

<br>

---

## 10. 각 파트 담당자 계약

`03` section 13 에 이어짐.

| # | 요구 | 이유 |
| --- | --- | --- |
| 69 | `local_path_planner`: 가중치 set 2종(`normal`/`avoidance`) 구현. `avoidance` 시 이탈 허용 상한을 파라미터/토픽으로 수신 | `AVOID_OBSTACLE` corridor 확장 (section 4) |
| 70 | `local_path_planner`: 회피 후보 전부 실현 불가 시 `경로_형식.md` section 9.8 규약대로 **빈 경로(무효)를 정상 주기로 발행** | `STOP_AND_WAIT` 진입 판정의 유일한 근거 (section 7) |
| 71 | `local_path_planner`: 이탈 허용 상한 초과 후보는 비용함수 이전에 사전 제외 | 한계를 넘는 경로가 실수로 채택되는 것 방지 |
| 72 | `steer_controller`: Gain set 2종(`smooth`/`aggressive`)을 파라미터·토픽 양쪽으로 수신 | 재시작 시 파라미터, 주행 중 전환은 토픽 (`03` 계약 43과 동일 패턴) |
| 73 | `steer_controller`: Gain 전환은 **계단이 아니라 램프** | 이득 급변은 진동 유발 (section 6-3) |
| 74 | `state_machine`: `/state_machine/behavior` 를 평가 tick(20 Hz) 안에서 발행, 별도 타이머 금지 | `01` section 14-1 / `02` 계약 19와 동일 원칙 |
| 75 | `state_machine`: `/state_machine/behavior` 에 behavior·gain_set·weight_set·departure 여부·한계값 전부 포함 | section 5 전파값 단일 발행원 |
| 76 | `state_machine`: 이탈 허용 한계값을 section 4-1 공식으로 매 tick 산출(프로파일 전환 즉시 반영) | 프로파일 전환과 한계값 갱신 사이 지연 차단 |
| 77 | `state_machine`: `/path/local` 를 section 9.8 무효 판정 **용도로만** 구독. 판정 결과를 `02` 등급 체계(heartbeat status)에 반영하지 않음 | section 5-2: 정상 동작과 노드 고장 혼동 방지 |
| 78 | `local_path_planner`/`steer_controller`: Behavior 진입 액션(gain/weight 전환)이 반영된 시점을 로그로 남김 | section 9 전이표 반영 지연 검증 근거 |

<br>

---

## 11. 검증 계획

`03` section 14 스텁을 확장.

| # | 시나리오 | 기대 |
| --- | --- | --- |
| 90 | 평시 `LANE_KEEPING` 유지, 장애물 없음 1랩 | Behavior 전이 없음, gain=smooth 고정 |
| 91 | 라바콘 검출 + 경로 간섭, 회피 후보 존재 | `LANE_KEEPING`→`AVOID_OBSTACLE`, gain aggressive 램프 적용 |
| 92 | 회피 완료(라바콘이 경로에서 벗어남) | `AVOID_OBSTACLE`→`LANE_KEEPING`, gain smooth 복귀 |
| 93 | 회피 후보 전부 corridor 침범(전폭 차단 배치) | `/path/local` 무효 → `AVOID_OBSTACLE`→`STOP_AND_WAIT` (3 tick), `speed_limit=0` |
| 94 | `STOP_AND_WAIT` 중 `OBSTACLE` 이벤트 해제 | `STOP_AND_WAIT`→`LANE_KEEPING`, `speed_limit` 복귀 |
| 95 | `STOP_AND_WAIT` 중 회피 후보 재확보 | `STOP_AND_WAIT`→`AVOID_OBSTACLE` (10 tick), `speed_limit` 하향 후 램프 |
| 96 | `global_only` 중 회피 발생, 필요 offset 이 **section 4-1 공식의 잔여 여유** 초과 | `/path/local` 무효 판정 → `STOP_AND_WAIT` (section 4-2 위험이 실제로 정지로 귀결되는지 확인) |
| 97 | `lane_only` 중 회피 발생 | `AVOID_OBSTACLE` 정상 성립, 이탈 판단이 **차선 기준**으로 이루어지는지 확인 (Global Path 미사용, section 4-3) |
| 98 | 회피 유효/무효 신호를 1 tick 씩 반복 주입 (경계값 흔들림) | `AVOID_OBSTACLE`↔`STOP_AND_WAIT` 진동 없음 (section 8 min dwell 검증) |
| 99 | Behavior 전이 도중 임의 노드 크래시로 `enable` 차단 정지 발생 | `/state_machine/enable` 즉시 중단, Behavior 라벨은 마지막 값 유지, 강제 전이 없음 (section 9 System 독립성 검증) |

- 96, 97 은 **section 4 직교 조합의 실동작 검증**. 정기 회귀 고정 (`02` section 11 시나리오 31/34 와 함께 fallback 회귀 세트로 관리)
- 98 은 **오탐 방지 검증**. 실주행 최대 위험

<br>

---

## 12. 확정 필요

**⚠ 시뮬 값은 예시. 충돌 판정 근거로 쓰지 말 것**

| 값 | 상태 |
| --- | --- |
| 차선폭 70 cm (반폭 35) | **예시**: 실측 필요 |
| 라바콘 반경 `r_cone` 7.5 cm | **예시**: 실측 필요 |
| 트랙 전장 2722.5 cm · 형상 | **예시** |
| 라바콘 6개 | 규정 확정 (개수만 유효) |

대회측 확정 차량 제원은 유효: 휠베이스 18 cm, 차폭 200 mm(반폭 10 cm), δ_max 20°, δ̇_max 600 °/s,
`collision_radius` 11.04 cm, `κ_max` 0.020221 /cm. 단 **조향 응답 시상수 τ 0.1 s 는 미회신 잠정**.


| 항목 | 현재 가정 | 필요 작업 |
| --- | --- | --- |
| 이탈 허용 범위 | **반대 차선까지 확정** (section 6-2). 트랙 주행가능 영역은 넘지 않음 | **대회 규정 확인**: 감점 규정 유무만 남음 |
| `drivable_half_width_cm` (허용 상한 계산의 기준값) | **미정** (`08` `constants`) | 트랙 실측. 이 값 없이는 section 4-1 공식이 수치를 못 냄 |
| `STOP_AND_WAIT` timeout | 무한 대기 (잠정) | 대회 규정 확인: 초과 시 실격 처리 여부 (section 7) |
| `lane_only` 추종오차 예산 | `lane_centering` 과 유사 추정 | 실측: section 4-1 표 |
| `aggressive` gain 실값 | 미정 | 조향 응답 시상수 τ 확정(`경로_형식.md` 부록 B) 후 결정 |
| `local_path_planner` 가중치 set 명칭/파라미터 | `normal`/`avoidance` 잠정 명명, 값 미정 | `local_path_planner` 담당 확정 |
| `global_only` 회피 여유 | **수치 없음**: section 4-1 공식만 확정 | `drivable_half_width_cm` + `경로_형식.md` section 6.8 corridor margin 확정 후 산출 |
| min dwell 3 tick / 10 tick | `02` section 5-2 값 재사용 | 실측 후 조정 (정지거리 예산 반영) |
| `/state_machine/behavior` 정밀 스키마 | section 5-1 provisional | `07` 인터페이스 문서에서 확정 |
