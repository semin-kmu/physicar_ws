# 04. System State

`system_supervisor` 가 소유하는 **System 계층 상태 정의와 전이 규칙**.

> 검출 규칙은 `02`, 대응·복구 절차는 `03`, 기동·종료 내부 동작은 `01`. 이 문서는 **상태와 전이만** 정의.

<br>

## 1. 범위

| 포함 | 미포함 (별도 문서) |
| --- | --- |
| System 상태 **7종** 정의, 허용 하위 상태 집합 | 기동 내부 절차 → `01` section 7 |
| 전이표 (guard / entry / exit / timeout) | 이상 검출 규칙 → `02` |
| **출발 게이트 판정 조건** (`01` section 4-3 구조의 판정부) | MRM·복구 절차 상세 → `03` |
| **Deadman 규약** (`/state_machine/enable` 주기·timeout·안전값) | Mission 상태 정의, 도착 판정 → `05` |
| tick 구조, 계층 간 전이 우선순위 | Behavior 상태 정의 → `06` |
| 채터링 방지 (min dwell, 히스테리시스) | 메시지 타입 확정 → `07` |
| 안전 상태 도달성, 전이 로깅 | |

- **이 문서의 모든 수치는 잠정**. 근거를 함께 적었고, 실측 확정 항목은 section 13
- `01`~`03` 과 중복되는 절차는 **참조로만** 처리. 재서술하지 않음

<br>

---

## 2. 계층 구조와 소유권

| 계층 | 소유 노드 | 상태 집합 | 전이 빈도 | 판단 근거 |
| --- | --- | --- | --- | --- |
| **System** | `system_supervisor` | 본 문서 section 3 | 매우 드묾 | 노드 헬스 등급, 출발 요청, 정차 |
| **Mission** | `state_machine` | `READY` `DRIVING` `FINISHED` (`05`) | 랩 단위 | 랩 진행도 `s`, 도착 판정 |
| **Behavior** | `state_machine` | `HOLD` `LANE_KEEPING` `AVOID_OBSTACLE` `STOP_AND_WAIT` (`06`) | 초 단위 | 인지 결과 |

**계층 간 이름 중복 금지. 유지**

| 계층 | 완료 상태 | 이유 |
| --- | --- | --- |
| System | `SHUTDOWN` | Mission `FINISHED` 도달이 System `SHUTDOWN` 전이를 **유발**하는 인과 관계 |
| Mission | `FINISHED` | 이름이 같으면 이 인과가 로그에서 구분되지 않음 |

- 상위 계층은 하위의 **허용 상태 집합을 제한**할 뿐, 권한을 이전하지 않음. 상위는 계속 살아서 전이를 평가
- **`state_machine` 은 System 상태를 스스로 판단하지 않음.** `/state_machine/system` 수신값만 사용 (계약 58)

<br>

---

## 3. 상태 정의

### 3-1. 상태 7종

| 상태 | 의미 | 이 상태에서만 하는 일 | 차량 |
| --- | --- | --- | --- |
| `INIT` | 순차 기동 진행 중 | 프로세스 spawn, Lifecycle 전이, G1~G4 판정 (`01` section 7) | 정지 |
| `STANDBY` | 기동 완료, 출발 대기 | `/state_machine/enable` 발행 시작 | 정지 |
| `ENGAGED` | 정상 주행 | 구간 상한 100 % 허가 | **주행** |
| `DEGRADED` | 축소 주행 | 주행 중 복구 (`03` section 8) | **주행** |
| `STOPPED` | 정지 + 복구 | **`/state_machine/enable` 중단**, 억제, Soft/Hard 복구, 복구 검증 (`03` section 5~section 9) | 정지 |
| `SHUTDOWN` | 완주 또는 SIGTERM → 정차·종료 | `speed_limit = 0` **즉시** → 정차 확인 → 역순 정리 → **자기 종료** (`01` section 11) | 정지 |
| `TERMINATED` | 영구 정지 | `/state_machine/enable` **영구 차단**, 로그 보존 | 정지 |

**병합 근거. 출력이 같으면 상태를 나누지 않음**

| 이전 | 현재 | 근거 |
| --- | --- | --- |
| `EMERGENCY` + `RECOVERY` | **`STOPPED`** | 둘의 출력 계약이 **완전히 동일**했다 (enable 중단, speed_limit 0, Mission 동결, Behavior `HOLD`). 차이는 "내부에서 복구 중인가"뿐이며 그것은 **상태가 아니라 로그 이벤트** |
| `COMPLETED` + `SHUTDOWN` | **`SHUTDOWN`** | `COMPLETED` 는 정차 대기 구간이고 `01` section 11-1 단계 1이 이미 정차 확인. 자동 전이(`stopped` OR 5 s)로 이어지므로 사실상 한 절차 |
| `DEGRADED` | **유지** | 병합 후보였으나 존치: 축소 주행을 상태로 관측하고 싶다는 판단 |

- **`TERMINATED` 는 프로세스를 종료하지 않음.** 종료하는 상태는 `SHUTDOWN` 하나뿐 (`01` section 11-3)
- 상태 구분의 기준은 **"차가 움직이는가"** 와 **"`/state_machine/enable` 이 나가는가"**. 둘이 1:1 대응

### 3-2. 상태별 출력 계약

| 상태 | `/state_machine/enable` | `/state_machine/speed_limit` | `/state_machine/system` |
| --- | --- | --- | --- |
| `INIT` | **미발행** | 0 (`state_machine` ACTIVE 이후) | `INIT` |
| `STANDBY` | 발행 (20 Hz) | **0**: 신호등 미확정 | `STANDBY` |
| `ENGAGED` | 발행 | 구간 상한 × 100 % | `ENGAGED` |
| `DEGRADED` | 발행 | 프로파일 비율 (`02` section 7-4), 또는 fallback 불성립 시 **`0` 즉시** | `DEGRADED` |
| `STOPPED` | **중단** | 0 | `STOPPED` |
| `SHUTDOWN` | 정차 확인까지 발행 → 이후 중단 | **`0` 즉시** | `SHUTDOWN` |
| `TERMINATED` | **영구 차단** | 0 | `TERMINATED` |

- 속도는 **구간 상한 대비 비율로만** 규정 (`02` section 7-4)
- `/state_machine/system` 은 감시 tick 안에서 20 Hz 발행 (section 7)

**⚠ `DEGRADED` 는 `/state_machine/profile` 과 같은 사실을 표현**

- 축소 주행 여부는 `/state_machine/profile` 이 **유일 원천**이고 `DEGRADED` 는 그 파생 표시
- 둘이 어긋나면 **`/state_machine/profile` 을 진실로 봄.** Supervisor 는 불일치를 경고 로깅만 하고 상태를 바꾸지 않음
- 이 규칙이 없으면 desync 시 어느 쪽이 맞는지 현장에서 판단할 수 없음

### 3-3. 허용 하위 상태 집합

| System | 허용 Mission | Behavior | Mission 전이 평가 |
| --- | --- | --- | --- |
| `INIT` | `READY` | `HOLD` | 중단 |
| `STANDBY` | `READY`, **`DRIVING`** | `HOLD`, `LANE_KEEPING` | **수행** |
| `ENGAGED` | `DRIVING`, `FINISHED` | 전체 | **수행** |
| `DEGRADED` | `DRIVING`, `FINISHED` | 전체 | **수행** |
| `STOPPED` | 진입 시점 값 **동결** | `HOLD` | 중단 |
| `SHUTDOWN` | 동결 | `HOLD` | 중단 |
| `TERMINATED` | 동결 | `HOLD` | 중단 |

- **`STANDBY` 가 `DRIVING` 을 허용하는 것이 1-hop 게이트의 핵심** (section 5). `state_machine` 이 먼저 `DRIVING` 으로 가고 Supervisor 가 1 tick 뒤 `ENGAGED` 로 따라옴. 그 사이 차량은 `/state_machine/enable` 이 있을 때만 움직임
- **동결(freeze)은 리셋이 아님**. `s`·이벤트·latch 보존 (`03` section 12). Behavior 만 기저 `HOLD` 로
- 허용 집합은 **설정 데이터**로 보유하고 `state_machine` 이 자체 차단 (계약 58)

<br>

---

## 4. 전이

### 4-1. Guard 불리언 정의

**guard 는 전부 명시적 불리언.** "정상이면" 같은 서술을 쓰지 않음.

| 이름 | 정의 | 산출 |
| --- | --- | --- |
| `all_ready` | 전 단계 노드가 G4 통과 (`01` section 3) | Supervisor |
| `init_failed` | 노드 재시도 한도 초과 **OR** 총 기동 예산 120 s 초과 (`01` section 6) | Supervisor |
| `grade` | `OK` / `DEGRADED` / `FAULT`: **연속 3 tick** stale 로 확정 (`02` section 5-2) | Supervisor |
| `grade_ok_10` | `grade == OK` 가 **연속 10 tick** (`02` section 5-2) | Supervisor |
| `fallback_ok` | 현재 이상 조합에 대해 필수 입력을 채우는 프로파일이 존재 (`02` section 7-4) | Supervisor |
| `give_up` | 해당 노드의 복구를 포기 확정 (`03` section 11-2) | Supervisor |
| `drive_permitted` | `grade == OK` **OR** (`grade == DEGRADED` **AND** `fallback_ok` **AND** `give_up`) | Supervisor |
| `mission_driving` | `/state_machine/mission == DRIVING`: `state_machine` 이 `start_permission == true` 를 받아 스스로 전이 (section 5) | `state_machine` |
| `engaged_latch` | `ENGAGED` 도달 이력 (section 5-4) | Supervisor |
| `stopped` | \|v\| < 0.05 m/s 가 1.0 s 유지 (`01` section 11-1). **속도원 부재 시 section 4-5 로 대체** | Supervisor |
| `recovery_ok` | 복구 검증 통과 **AND** 추가 안정 2 s **AND** 억제 해제 후 전 노드 재평가 정상 (`03` section 9·section 10-2) | Supervisor |
| `limit_exceeded` | 재engage 3회 초과 **OR** 누적 복구 60 s 초과 **OR** 포기 노드가 `TERMINATED` 대상 (`03` section 11) | Supervisor |
| `mission_finished` | `/state_machine/mission == FINISHED` (`05`) | `state_machine` |
| `sigterm` | Supervisor 가 SIGTERM 수신 (`01` section 14-1 (4)) | Supervisor |

**`drive_permitted` 의 세 번째 항이 핵심**

| 상황 | `drive_permitted` | 근거 |
| --- | --- | --- |
| `grade == DEGRADED`, 복구 여지 있음 | **false** | 아직 고칠 수 있으면 고친다. 정지 비용이 낮은 쪽을 택함 |
| `grade == DEGRADED`, 복구 포기 확정 + fallback 성립 | **true** | 더 고칠 수 없음 → 축소 주행으로 완주 (`03` section 11-2) |

### 4-2. 전이표

| # | from → to | guard | entry action | timeout |
| --- | --- | --- | --- | --- |
| T1 | (부팅) → `INIT` | 프로세스 시작 | `run_id` 발급, 로그 디렉터리, 사전 점검 (`01` section 10) | **120 s** → T3 |
| T2 | `INIT → STANDBY` | `all_ready` | `/state_machine/enable` 발행 시작, `/state_machine/system` 발행 시작, G4 구독을 감시로 승계 | 없음 |
| T3 | `INIT → TERMINATED` | `init_failed` | 자식 역순 정리 (`01` section 11), 실패 관문·stderr flush | - |
| T4 | `STANDBY → ENGAGED` | `mission_driving` **AND** `drive_permitted` | `engaged_latch = true`, 출발 시각 기록 | 없음 (section 5-4) |
| T5 | `STANDBY → STOPPED` | **NOT** `drive_permitted` 확정 | STOPPED entry (section 4-3) | - |
| T6 | `ENGAGED → DEGRADED` | `grade == DEGRADED` 확정 | 프로파일 전환 지시, 속도 하향 지시 (`02` section 7-4) | - |
| **T7** | `ENGAGED → STOPPED` | `grade == FAULT` 확정 | STOPPED entry | - |
| T8 | `ENGAGED → SHUTDOWN` | `mission_finished` | SHUTDOWN entry (section 4-3) | 전체 **30 s** → SIGKILL 일괄 |
| T9 | `DEGRADED → ENGAGED` | `grade_ok_10` | 프로파일·속도 상한 **램프** 복귀 (`03` section 10-3) | - |
| **T10** | `DEGRADED → STOPPED` | `grade == FAULT` 확정 **OR** (**NOT** `fallback_ok` **AND** `stopped`) | STOPPED entry | 정차 소요 **10 s** 초과 시 즉시 |
| T11 | `DEGRADED → SHUTDOWN` | `mission_finished` | T8 과 동일 | 동일 |
| T12 | `STOPPED → STANDBY` | `recovery_ok` | 억제 해제, `/state_machine/enable` 재개, 속도 상한 **하향값** 지시, 재engage 횟수 +1 | 누적 복구 **60 s** → T13 |
| **T13** | `STOPPED → TERMINATED` | `limit_exceeded` **OR** (`NOT stopped` 가 5 s 지속) | `/state_machine/enable` 영구 차단, 링버퍼 전체 flush (`02` section 9) | - |
| **T14** | `*` → `SHUTDOWN` | `sigterm` | SHUTDOWN entry: **① `speed_limit = 0` 부터** 수행 | 동일 |

**전이 18개 → 14개.** 굵은 번호는 **안전 전이** (section 7-2)

### 4-3. 상태별 entry 절차

**`STOPPED` entry**. 이전 `EMERGENCY → RECOVERY` 2단계를 한 상태의 순차 절차로 통합

```
① /state_machine/enable 발행 중단          ← 즉시. 이것이 정지 수단
② Mission 동결, Behavior HOLD
③ stopped 확인 대기 (최대 5 s, 속도원 부재 시 section 4-5)
④ 억제 진입 (03 section 7) → 복구 절차 (03 section 6)
⑤ 복구 검증 → recovery_ok
```

- ①~② 는 **같은 tick**. ③ 이후는 절차이며 **상태를 바꾸지 않음**
- 각 단계 진입을 **로그 이벤트로 기록**. 이전 설계에서 `EMERGENCY`/`RECOVERY` 상태가 하던 구분을 로그가 대신
- ③ 이 5 s 를 넘으면 T13

**`SHUTDOWN` entry**. 이전 `COMPLETED → SHUTDOWN` 2단계 통합

```
① speed_limit → 0 (즉시)                 ← 이전 COMPLETED
② stopped 확인 (최대 5 s)
③ /state_machine/enable 중단
④ 역순 deactivate → cleanup → SIGTERM → SIGKILL (01 section 11-1)
⑤ 자기 프로세스 종료
```

### 4-4. 전이도

```
                    sigterm (T14, 전 상태 공통)
       ┌─────────────────────────────────────────┐
       ↓                                         ↓
  INIT ──T2──→ STANDBY ──T4──→ ENGAGED ──T8──→ SHUTDOWN
   │             ↑  │           │  ↑              (프로세스 종료)
   │ T3      T12 │  │ T5     T6 │  │ T9
   │             │  ↓           ↓  │
   │             │ STOPPED ←── DEGRADED ──T11──→ SHUTDOWN
   │             └──┘ ↑  │        │
   │                T7│  │T13     │ T10
   │                  └──┼────────┘
   ↓                     ↓
  TERMINATED ←───────────┘
  (프로세스 유지)
```

- **`ENGAGED → STOPPED` 가 곧 정지 + 복구**. 별도 상태 전이 없이 entry 절차로 이어짐
- **`*` → `SHUTDOWN` (T14) 이 `TERMINATED` 에서도 성립**. `lane_only` 무한 주행(`02` section 7-4)과 `TERMINATED` 정지 유지를 **운영자가 끝내는 유일한 경로**

### 4-5. `stopped` · 속도원이 없을 때

**문제**: `ekf` 고장으로 `STOPPED` 에 들어가면 차속을 알 수 없어 `stopped` 가 영원히 false → section 4-3 ③ 교착.

| 항목 | 규정 |
| --- | --- |
| 1차 판정 | `/odometry/filtered` 의 종방향 속도 (계약 57) |
| 속도원 stale 시 | **`/state_machine/enable` 차단 후 `T_coast` 경과를 정차로 간주** |
| `T_coast` | **3 s (잠정)**: 근거: 구간 상한 2.0 m/s 가정, 마찰 감속 하한 1.0 m/s² 가정 시 2 s + 마진 1 s |
| 로깅 | "정차 미확인, coast 타임아웃으로 진행" 기록 필수 |

- 이 대체 경로가 없으면 **TF 체인이 죽은 순간 상태 기계가 멈춤**. 가장 잡기 어려운 교착
- `T_coast` 는 실측 확정 대상 (section 13)

<br>

---

## 5. 출발 게이트

`01` section 4-3 이 **구조**(값으로 연다, 정지 2계층, latch)를 확정. 이 절은 **판정 조건과 책임 분리**를 확정.

### 5-1. 판정은 `object_detection`, 실행은 `state_machine`

**신호등 판정을 State Machine 이 하지 않음.** `object_detection` 이 색 판정·아웃라이어 필터링까지 끝내고 **주행 가능 여부를 bool 로** 발행.

```
object_detection ──/perception/start_permission (std_msgs/Bool, 10Hz)──→ state_machine ──/state_machine/speed_limit > 0──→ speed_controller
   색 판정 + 필터링 + 확정                              latch + DRIVING 전이

system_supervisor ──/state_machine/enable──────────────────────────────────────────────→ speed_controller
                     거부권 (헬스 불량 시 미발행)
```

| 주체 | 하는 일 | 하지 않는 일 |
| --- | --- | --- |
| `object_detection` | 색 판정, 아웃라이어 필터링, **출발 가능 확정** | System 상태를 모름 |
| `state_machine` | bool 소비 → latch → `DRIVING` 전이 → `speed_limit` 상승 | **색·신뢰도·ROI 를 보지 않음** |
| `system_supervisor` | 헬스 불량 시 `/state_machine/enable` 미발행 | 인지 소비 안 함 |

**State Machine 이 재판정하지 않는 이유**

| 항목 | 내용 |
| --- | --- |
| 이중 판정 금지 | 같은 사실을 두 노드가 판정하면 어긋났을 때 어느 쪽이 맞는지 알 수 없음 |
| 정보 부족 | 프레임 신뢰도·ROI 는 `object_detection` 만 가진 정보. bool 만 받는 쪽이 재판정할 근거가 없음 |
| 관측성 | 오검출 원인 추적이 `object_detection` 한 곳으로 모임 |

**AND 조건은 구조적으로 성립**

| 조건 | 성립 방식 |
| --- | --- |
| 출발 허가 | bool 이 `true` 여야 `speed_limit > 0` |
| 헬스 정상 | `/state_machine/enable` 이 있어야 컨트롤러가 출력 |
| **두 조건 AND** | 둘 중 하나라도 없으면 **차량이 움직이지 않음** |

**⚠ `state_machine` 은 `ENGAGED` 관측을 기다리지 않는다 (1-hop 의 정의)**

| 잘못된 설계 | 왜 안 되는가 |
| --- | --- |
| "`/state_machine/system == ENGAGED` 를 봐야 `speed_limit > 0`" | Supervisor 의 `STANDBY → ENGAGED` guard 가 `mission_driving`(= `state_machine` 이 이미 `DRIVING`)이므로 **서로를 기다리는 교착** |

- 따라서 계약 49(스스로 전이·발행)와 계약 50(Supervisor stale 시 0)은 **역할이 다름**: 49 는 출발 조건, 50 은 정지 조건
- `state_machine` 이 헬스를 모르는 대가는 `/state_machine/enable` 이 지불. 출발 권한은 나누지 않음

**반드시 로깅할 상황: `DRIVING` 인데 `drive_permitted` 가 false**

- `state_machine` 이 헬스를 모르므로 고장 상태에서도 `DRIVING` 으로 갈 수 있음
- 차량은 `/state_machine/enable` 이 없어 안 움직이나, **"출발하려 했으나 막혔다"가 로그에 남아야 함**
- Supervisor 가 이 불일치를 **1 Hz 경고 로깅**

### 5-2. bool 소비 규약

| 항목 | 규정 (`object_detection` 파트 확정) |
| --- | --- |
| 토픽 | **`/perception/start_permission`** |
| 타입 | **`std_msgs/Bool`** (`data` 필드 1개) |
| QoS | RELIABLE · KEEP_LAST(1) · VOLATILE |
| 발행 주기 | **10 Hz** |
| `true` 의미 | `GREEN` **5프레임 연속** 검출 확정. 카메라 ≈ 15 Hz 기준 약 **0.33 s** |
| `false` · 미수신 | **전부 동일하게 불허** (화이트리스트 판정) |
| `state_machine` 처리 | `false → true` 관측 시 `DRIVING` 전이. **재확인·재판정 없음** |
| 초기값 | `false` |

**stale 판정은 `gap` 단독**

`std_msgs/Bool` 에는 `header` 가 없어 `age` 를 산출할 수 없음 (`07` section 3-2).

| 지표 | 가용 | 임계 |
| --- | --- | --- |
| `gap` (수신 중단) | **가능** | `08` section 4-0 실측 표에서 산출 (발행 10 Hz 확정이므로 **300~500 ms 범위**) |
| `age` (데이터 지연) | **불가** | - |

- 잃는 것은 "낡은 판정을 계속 재발행하는 고장"의 검출. `true` 는 latch 라 원래 안 바뀌고 `false` 는 안전 방향이므로 **실질 위험 없음**

**⚠ latch 가 두 곳에 있다. 둘 다 필요**

| 위치 | 목적 | 없으면 |
| --- | --- | --- |
| `object_detection` 내부 (확정) | `true` 이후 신호등이 사라져도 `true` 유지 | 신호등이 시야를 벗어나는 순간 `false` |
| **`state_machine` (계약 51)** | **`object_detection` 재시작 시 방어** | Hard Restart 로 그쪽 latch 가 초기화되어 `false` 재발행 → **주행 중 정지** |

- `object_detection` 이 latch 한다고 해서 `state_machine` latch 를 없애면 안 됨. **복구가 곧 오정지가 됨** (`03` section 3-1: `object_detection` 은 Soft → Hard 복구 대상)

### 5-3. latch · 출발 후 신호등 영구 무시

| 항목 | 규정 |
| --- | --- |
| latch 원천 | **`state_machine` 자신의 `DRIVING` 도달 이력**: 단일 원천 |
| Supervisor 측 | `engaged_latch` 는 **재engage 판정 로그용**. 게이트 판정에 쓰지 않음 |
| latch 이후 | `/perception/start_permission` 을 **판정에 일절 사용하지 않음**. 구독·로깅은 유지 |
| 복구 통과 시 | **유지**: 복구 후 신호등을 다시 보지 않음 (`03` section 10-1·section 12) |
| 재시작 시 | 파라미터로 주입 복원 (`03` 계약 41) |
| 근거 | 주행 중 신호등은 존재하지 않음 → 이후 검출은 **정의상 오검출** (`01` section 4-3) |

**`STANDBY` 재진입 시 게이트 동작**

| `engaged_latch` | `STANDBY → ENGAGED` 조건 | 상황 |
| --- | --- | --- |
| `false` | `/perception/start_permission == true` 재관측 필요 | 출발 전 고장 → 복구 → 신호등 재평가 (`03` section 10-1) |
| `true` | 재관측 **불요**: latch 를 보고 즉시 `DRIVING` 유지 | 주행 중 고장 → 복구 → **자동 재engage** |

- 자동 재engage 의 안전 상쇄는 `STOPPED` 출구의 `recovery_ok` 와 재engage 횟수 한도 (`03` section 10-2). 이 문서는 그 결과만 사용

### 5-4. `STANDBY` 에는 timeout 이 없다

| 항목 | 내용 |
| --- | --- |
| 규정 | 신호 주기 미상 → **타임아웃 자동 출발 금지** |
| 검출 실패 시 | `STANDBY` **무한 유지** |
| 관측성 대책 | 미충족 guard 를 **1 Hz 로 요약 로깅** (section 10): 전이가 없으면 로그도 없어 현장에서 원인을 못 밝힘 |

<br>

---

## 6. Deadman

### 6-1. `/state_machine/enable` 규약

| 항목 | 값 (잠정) | 근거 |
| --- | --- | --- |
| 토픽 | `/state_machine/enable` (타입은 `07`) | - |
| 발행 주체 | `system_supervisor` **단독** | 판정은 Supervisor 단일 지점 (`02` section 2) |
| 발행 시점 | **감시 tick 의 마지막 단계** (`02` section 8-1 6번) | 별도 타이머 금지: 감시가 멈추면 발행도 멈춰야 함 (`01` section 14-1 대응 A) |
| 발행 주기 | **20 Hz** (= 감시 tick 주기) | `02` section 8-1 |
| QoS | `best_effort`, `volatile`, depth **1** | 최신값만 의미. 재전송은 지연만 늘림 |
| 컨트롤러 timeout | **250 ms** | 발행 주기 50 ms 의 **5배** (`01` section 14-2 규칙). rclpy 지터 흡수 |
| 구독자 | `speed_controller`, `steer_controller`, (대행 시) `state_machine` (`03` 계약 47) | - |

**⚠ `01` section 14-2 / `03` section 4-1 과의 수치 불일치**

| 문서 | 전제 | Deadman timeout |
| --- | --- | --- |
| `01` section 14-2 | enable **50 Hz** 발행 | 150~200 ms |
| `02` section 8-1 | enable 을 **20 Hz 감시 tick** 안에서 발행 (확정) | - |
| **본 문서 (채택)** | 20 Hz 발행 | **250 ms** |

- `02` 가 발행 주기를 20 Hz 로 확정했으므로 "5배 이상" 규칙을 그대로 적용하면 250 ms
- 대안 검토: tick 을 50 Hz 로 올리면 RPi5 rclpy 부하 위험, timeout 을 150 ms 로 낮추면 지터에 의한 **오탐 정지** 증가
- **채택 근거**: 정지 수단이 실제로 동작하는 것이 지연보다 우선. 대가는 정지거리 예산 증가이며 section 13 에서 속도 상한에 반영
- `02` section 5-2 의 **30 cm 는 검출 확정분(150 ms)만**이고, 위 400 ms/80 cm 는 **`enable` 차단 정지 전체 지연**임. 대체 관계가 아니라 포함 관계

**정지거리 예산 갱신 (잠정)**

| 항목 | 값 | 출처 |
| --- | --- | --- |
| 이상 확정 | 150 ms | `02` section 5-2 (연속 3 tick) |
| Deadman 만료 | 250 ms | 본 절 |
| **합계** | **400 ms** | 2.0 m/s 기준 **80 cm** |

### 6-2. 안전값 정의

| 컨트롤러 | Deadman 만료 시 출력 | 근거 |
| --- | --- | --- |
| `speed_controller` | `target_speed = 0` **즉시**. 램프 금지 | 구동을 즉시 끊는 것이 정지 |
| `steer_controller` | **직전 조향각 유지 → `steer_zero_tau` 로 0 수렴** | 즉시 0 은 곡선 구간 차선 이탈 (`03` section 4-2) |
| `state_machine` (대행 중) | 상수 속도 발행 중단 | `03` 계약 47 |

- `steer_zero_tau` 는 **정지 소요 시간과 같은 크기**. 값은 `03` section 15 실측 대상

### 6-3. 두 컨트롤러 동일성 강제

| 항목 | 규정 | 검사 시점 |
| --- | --- | --- |
| 구현 | **공용 Deadman 구현 1벌** (`kau_controller` 내 공용 모듈) | 코드 리뷰 |
| 파라미터명 | `deadman_timeout_ms` (양 노드 동일) | - |
| 값 | 양 노드 **동일값 강제** | **기동 단계 8·9 에서 `get_parameters` 로 조회 후 비교** |
| 불일치 시 | **`INIT_FAILED`** | `01` section 5 의 "동일성 확인" 방법을 여기서 확정 |

- 갈라지면 한쪽만 먼저 멈춰 **반쪽 명령**. 조향 없는 구동 또는 구동 없는 조향

### 6-4. `/state_machine/speed_limit` stale 과의 역할 구분 · 정지 2계층

| | `/state_machine/enable` | `/state_machine/speed_limit` |
| --- | --- | --- |
| 발행 | `system_supervisor`, 20 Hz | `state_machine`, 20 Hz |
| 정지 조건 | **미수신 250 ms** | **값 = 0** 또는 **미수신 250 ms** |
| 커버하는 고장 | Supervisor tick 정지·사망, `FAULT` 판정, `TERMINATED` | `state_machine` tick 정지·사망, 신호등 대기, `speed_limit = 0` 정지, `STOP_AND_WAIT`, `FINISHED` |
| 조향에 대한 영향 | **있음**: 안전값 수렴 | **없음**: 조향은 계속 추종 |
| 해제 | 재발행 즉시 | 값 복원 즉시 |
| 성격 | 명령 경로 **전체 차단** | **속도만** 0 |

- **두 계층은 서로 다른 노드의 고장을 커버하므로 합칠 수 없음**. Supervisor 가 죽으면 `/state_machine/speed_limit` 은 정상 발행될 수 있고, `state_machine` 이 죽으면 `/state_machine/enable` 은 정상 발행될 수 있음
- **stale timeout 은 양쪽 동일값(250 ms)**. 다르면 어느 쪽이 먼저 걸리는지가 상황마다 달라져 사후 분석이 불가능해짐 (계약 54)
- 컨트롤러 게이트 입력은 이 **2개뿐**. 새 채널을 만들지 않음 (`01` section 4-3)

<br>

---

## 7. tick 구조와 전이 우선순위

### 7-1. tick 구조 · 두 노드 모두 고정 주기 20 Hz

```
system_supervisor tick (20 Hz):
  1~5. 감시 (02 section 8-1: 생존 / heartbeat / stale / 교차 대조 / 등급 산출)
  6.   System 전이 평가        ← 안전 전이 우선 (section 7-2)
  7.   /state_machine/enable 발행            ← 전이 결과가 주행 허가일 때만
  8.   /state_machine/system 발행 + 전이 로깅

state_machine tick (20 Hz):
  1.   /state_machine/system 수신값 확인 → 허용 Mission 집합 제한 (section 3-3)
  2.   Mission 전이 평가        ← 허용 집합 안에서만
  3.   Behavior 전이 평가
  4.   /state_machine/speed_limit + /state_machine/mission 발행   ← tick 안에서 (01 계약 18)
```

| 규칙 | 이유 |
| --- | --- |
| **전이 평가가 발행보다 앞** | 전이 결과가 그 tick 의 발행값에 즉시 반영 |
| **발행이 tick 마지막** | 앞 단계가 멈추면 발행도 멈춤 → Deadman·stale 이 잡음 (`01` section 14-1 A) |
| **별도 타이머 금지** | 두 노드 모두. 타이머 분리 시 "일은 안 하는데 신호는 나가는" 상태 발생 |
| 콜백은 **수신·기록만** | 판정이 콜백에 흩어지면 tick 정지를 검출할 수 없음 (`02` section 8-1) |

### 7-2. 전이 우선순위

| 순위 | 대상 | 규칙 |
| --- | --- | --- |
| **0** | **안전 전이**: T7 T10 T13 T14 (→ `STOPPED` / `TERMINATED` / `SHUTDOWN`) | 매 tick **무조건 먼저** 평가. 다른 전이를 **선점** |
| 1 | System 나머지 전이 | 안전 전이 미성립 시 |
| 2 | Mission 전이 | System 이 허용하는 집합 안에서만 (section 3-3) |
| 3 | Behavior 전이 | Mission 이 허용하는 집합 안에서만 |

| 규칙 | 내용 |
| --- | --- |
| **한 tick 에 계층당 최대 1회 전이** | 다중 전이는 entry/exit action 순서가 모호해지고 로그에서 인과가 사라짐 |
| 예외 | **안전 전이는 dwell·다중 전이 제한을 무시**하고 즉시 적용 |
| 상위 전이 후 | **같은 tick 에서** 하위 계층을 재평가. 허용 집합을 벗어났으면 동결 또는 강제 기저 복귀 |
| 계층 간 지연 | System 전이 → `state_machine` 반영까지 **최대 1 tick (50 ms)**. 안전 전이는 `/state_machine/enable` 이 같은 tick 에 끊기므로 이 지연에 의존하지 않음 |

<br>

---

## 8. 채터링 방지

### 8-1. 원칙

**dwell 은 안전 → 주행 방향에만 건다. 주행 → 안전 방향에는 걸지 않음.**

| 방향 | 지연 | 근거 |
| --- | --- | --- |
| 위험 검출 → 정지 | **없음** (즉시) | 안전 전이 |
| 정지 → 주행 재개 | dwell + 히스테리시스 | 오판 재개 차단 |

- `02` section 5-2 의 "확정 3 tick / 복귀 10 tick" 과 동일 패턴. **위험 방향으로는 빠르게, 안전 방향으로는 느리게**

### 8-2. 히스테리시스 (`02` section 5-2 확정값 그대로 사용)

| 항목 | 값 | 출처 |
| --- | --- | --- |
| 등급 상승 확정 | **연속 3 tick** (150 ms) | `02` section 5-2 |
| 등급 복귀 확정 | **연속 10 tick** (500 ms) | `02` section 5-2 |
| 속도 상한 복귀 | 계단 금지, **램프** | `03` section 10-3 |
| 프로파일 복귀 | 가중치 **램프 복원** | `02` 계약 27 |

### 8-3. min dwell (잠정)

| 상태 | min dwell | 근거 |
| --- | --- | --- |
| `INIT` | 없음 | 1회성 |
| `STANDBY` | **0.5 s** (10 tick) | `STOPPED → STANDBY → ENGAGED` 를 1 tick 에 통과하면 재engage 검증이 로그에 남지 않음 |
| `ENGAGED` | **1.0 s** | 진입 직후 즉시 `DEGRADED` 로 되돌아가는 진동 차단 |
| `DEGRADED` | **2.0 s** | 프로파일 전환 램프가 끝나기 전에 복귀하면 경로가 두 번 급변 (`02` section 7-4) |
| `STOPPED` | **없음** | 안전 전이. 출구가 `recovery_ok` 검증으로 통제 |
| `SHUTDOWN` `TERMINATED` | 없음 | 종료 경로 |

- **min dwell 은 진입 후 경과 시간만 봄.** dwell 미충족이라도 안전 전이는 통과 (section 7-2)
- 전 값 잠정. 실주행 진동 관측 후 확정 (section 13)

<br>

---

## 9. 안전 상태 도달성

**모든 상태는 안전 상태로 가는 경로를 가짐.** 빠져나갈 길 없는 상태 0개.

### 9-1. 상태별 안전 출구

| 상태 | 안전 출구 | 실제 정지 수단 | 최악 지연 (잠정) |
| --- | --- | --- | --- |
| `INIT` | T3 → `TERMINATED` | `/state_machine/enable` **미발행** (애초에 정지) | 0 |
| `STANDBY` | T5 → `STOPPED` | `/state_machine/enable` 중단 + `speed_limit` 0 유지 | 250 ms |
| `ENGAGED` | T7 → `STOPPED` | `/state_machine/enable` 중단 | 150 + 250 = **400 ms** |
| `DEGRADED` | T10 → `STOPPED` | `/state_machine/enable` 중단 또는 `speed_limit → 0` | 400 ms / 50 ms |
| `STOPPED` | T13 → `TERMINATED` | 이미 `/state_machine/enable` 차단 상태 | 0 |
| `SHUTDOWN` | (terminal, 자기 종료) | `speed_limit = 0` 즉시 → 정차 → 프로세스 종료 = 명령 소멸 | 5 s (강행) |
| `TERMINATED` | (terminal) | `/state_machine/enable` 영구 차단 | - |

### 9-2. 상태 기계 밖의 안전망

| 실패 | 커버 | 상태 기계 의존 |
| --- | --- | --- |
| Supervisor 프로세스 사망 | `/state_machine/enable` 끊김 → Deadman | **무관** |
| Supervisor tick 전체 정지 | 발행이 tick 안에 있음 → 함께 중단 | **무관** |
| Supervisor tick 지연 누적 | `tick_duration > 100 ms`(주기 50 ms 의 2배) 연속 3회 → enable 스킵 (`02` section 8-2) | **무관** |
| `state_machine` 사망 | `/state_machine/speed_limit` stale → `limit = 0` | **무관** |
| 상태 기계 버그로 전이가 멈춤 | **커버 안 됨** → 위 3개가 tick 정지를 잡을 때만 성립 | 의존 |

- 마지막 행이 **유일한 구멍**. tick 은 도는데 전이 평가만 잘못된 경우. 검증(section 12)으로만 막음

<br>

---

## 10. 전이 로깅

`02` section 9 의 로깅 규약을 따름. **매 tick 상태값 기록 금지.**

### 10-1. 전이 레코드

| 필드 | 내용 |
| --- | --- |
| `t_mono`, `t_wall` | 단조 시각 + 벽시계 시각 |
| `layer` | `System` / `Mission` / `Behavior` |
| `from`, `to` | 상태명 |
| `transition_id` | `T4` 등 section 4-2 표의 번호 |
| `trigger` | 전이를 유발한 guard 이름 |
| **`guard_snapshot`** | 해당 전이의 **전 guard 불리언 값** |
| `grade`, `fallback_ok`, `give_up` | 등급 문맥 |
| `dwell_ms` | 직전 상태 체류 시간 |
| `run_id` | 실행 식별자 (`01` section 8) |

- **`guard_snapshot` 이 핵심**. 없으면 "왜 출발하지 않았는가"를 사후에 밝힐 수 없음
- System 전이는 드물므로 **전수 기록**. Mission/Behavior 도 전수 기록 (`02` section 9 의 "등급 전이 시점만" 과 동일 밀도)

### 10-2. 전이가 없을 때의 로깅

| 상황 | 규칙 | 이유 |
| --- | --- | --- |
| `STANDBY` 대기 중 | **1 Hz** 로 미충족 guard 요약 1줄 | 전이가 없으면 로그도 없음 → 현장에서 출발 실패 원인 파악 불가 |
| 그 외 상태 | 기록 없음 | SD 수명 (`01` section 15) |
| `FAULT` 확정 시 | 30 s 링버퍼 전체 flush (`02` section 9) | - |
| `TERMINATED` 진입 시 | 동일 flush | `03` section 11-3 |

<br>

---

## 11. 각 파트 담당자 계약

`03` section 13 (계약 48) 에 이어짐.

| # | 요구 | 이유 |
| --- | --- | --- |
| 49 | `state_machine`: `/perception/start_permission == true` 관측 시 **스스로 `DRIVING` 전이 + `speed_limit` 상승**. Supervisor 승인을 기다리지 않음 | 1-hop 게이트 (section 5-1). 헬스 거부권은 `/state_machine/enable` 이 담당 |
| 50 | `state_machine`: `/state_machine/system` 이 **stale(250 ms) 이거나 허용 집합 밖**이면 `speed_limit = 0`. **`ENGAGED` 관측을 출발 조건으로 요구하지 않음** (1-hop, 계약 49) | Supervisor 사망 시 정지. 단 출발 자체를 막지는 않음 (section 3-3) |
| 51 | `state_machine`: `DRIVING` 도달 시 **자체 latch**. 이후 `/perception/start_permission` 을 판정에 미사용(로깅만). latch 는 파라미터 주입으로 복원 | **`object_detection` 재시작 시 그쪽 latch 가 풀려 `false` 가 재발행됨** → 주행 중 오정지 방지 (section 5-2) |
| 52 | `object_detection`: **`/perception/start_permission`** (`std_msgs/Bool`, **10 Hz**, RELIABLE/KEEP_LAST(1)/VOLATILE) 발행 — **파트 확정**<br>ⓐ `true` = `GREEN` **5프레임 연속** 검출. 이후 **노드 내부에서 latch**, `RED`/`YELLOW`/소실에도 `false` 로 되돌리지 않음<br>ⓑ `false` = `RED`·`YELLOW`·`UNKNOWN`, 카메라 미수신·판별 실패, `GREEN` 5프레임 미만<br>ⓒ 판별 실패·미검출 시에도 **`false` 를 계속 발행**. 발행 중단 금지 | **State Machine 이 재판정하지 않으므로 여기가 유일한 방어선.** 발행이 끊기면 미발행과 불허를 구분할 수 없음 |
| 53 | 컨트롤러 2종: **`deadman_timeout_ms` 파라미터명·값 동일**, Deadman **공용 구현 1벌**, `get_parameters` 로 조회 가능 | 기동 시 동일성 자동 검사 (section 6-3) |
| 54 | 컨트롤러 2종: `/state_machine/enable` 과 `/state_machine/speed_limit` 의 **stale timeout 을 동일값**으로 적용 | 어느 쪽이 먼저 걸리는지가 상황마다 달라지면 사후 분석 불가 (section 6-4) |
| 55 | `speed_controller`: Deadman 만료 또는 `speed_limit = 0` 시 `target_speed = 0` **즉시**. 램프 금지 | 정지는 지연 없이 (section 6-2) |
| 56 | `steer_controller`: Deadman 만료 시 **직전 조향각 유지 → `steer_zero_tau` 로 0 수렴**. 즉시 0 금지 | 곡선 구간 차선 이탈 방지 (`03` section 4-2) |
| 57 | `ekf`(또는 대체 속도원): 종방향 차속을 **`/odometry/filtered` 로 상시 제공** | `stopped` 판정의 유일한 입력 (section 4-4) |
| 58 | `state_machine`: System 상태별 **허용 Mission/Behavior 집합을 설정 데이터로 보유**, 허용 밖 전이는 자체 차단 + 로깅 | 상위 제약을 코드에 흩뿌리지 않음 (section 3-3) |

**추가 제출물**

| 항목 | 용도 | 담당 |
| --- | --- | --- |
| ~~출발 허가 토픽·타입·주기~~ | **확정 완료**: `/perception/start_permission` · `std_msgs/Bool` · 10 Hz | `object_detection` |
| 컨트롤러 Deadman 실측 반응 시간 | section 6-1 timeout 확정 | `kau_controller` |
| 마찰 감속도 실측 | section 4-5 `T_coast` 확정 | `speed_controller` |

<br>

---

## 12. 검증

`01` section 13-1 / `02` section 11 의 스텁 노드를 그대로 사용. `/perception/start_permission` 의 `data` 주입만 추가.

> **`GREEN` 5프레임 확정 로직 자체는 본 문서의 검증 대상이 아님.** `object_detection` 파트가 계약 52 ⓐ~ⓒ 에 대해 자체 검증

| # | 시나리오 | 기대 |
| --- | --- | --- |
| 60 | 전 노드 정상 기동 | `STANDBY` 도달, `/state_machine/enable` 20 Hz 발행 시작, `speed_limit == 0`, `mission_state == READY` |
| 61 | `data = false` 를 10분 발행 | **전이 없음.** `STANDBY` 무한 유지, 1 Hz 미충족 guard 로그 존재 |
| 62 | `data` 를 `false → true` 로 전환 | `DRIVING` 전이 → 다음 tick `ENGAGED`, `speed_limit > 0` 까지 **≤ 100 ms** (2 tick) |
| 63 | 발행을 완전히 중단 (`gap` 초과) | **전이 없음** (미수신 = 불허, section 5-2) |
| 64 | `data = true` 와 동시에 핵심 노드 `FAULT` 주입 | Mission `DRIVING` 이 되어도 **`/state_machine/enable` 미발행 → 차량 미이동**. 불일치 경고 로그 발생 (section 5-1) |
| 65 | `ENGAGED` 후 `data` 를 1분간 `true`/`false` 교대 주입 | 무시, 주행 유지 (latch 검증) |
| **74-b** | 주행 중 `object_detection` 을 Hard Restart → `data` 가 `false` 로 재발행 | **주행 유지**. `state_machine` 자체 latch 가 막는지 확인 (계약 51) |
| 66 | 주행 중 `system_supervisor` 를 `kill -9` | `/state_machine/system` stale 250 ms → `state_machine` 이 **스스로 `speed_limit = 0`** (계약 50). `/state_machine/enable` 도 함께 끊겨 **이중 정지** |
| 67 | `ENGAGED` 중 핵심 노드 `FAULT` | `STOPPED` 진입, `/state_machine/enable` 중단 ~ 컨트롤러 안전값 출력 차 **≤ 250 ms** |
| 68 | `STOPPED` 중 속도원(`ekf`) 사망 | `T_coast` 경과로 복구 절차 진입: **교착 없음** (section 4-5) |
| 69 | `lane_detection` 사망 → `fallback_ok` | `DEGRADED`, **정지 없이** 주행 유지, 속도 하향 |
| 70 | `lane_detection` + `localization` 동시 이상 | `speed_limit = 0` 정지 → 정차 → `STOPPED` 복구 절차 (T10) |
| 71 | 주행 중 복구 성공 (latch == true) | `STOPPED → STANDBY → ENGAGED` 자동 진행, **신호등 재평가 0회** |
| 72 | 출발 전 고장 → 복구 성공 (`engaged_latch == false`) | `STANDBY` 유지, **신호등 게이트 재평가 수행** |
| 73 | Mission `FINISHED` 도달 | `SHUTDOWN` 진입 → `speed_limit = 0` 즉시 → 정차 → **잔존 프로세스 0개** |
| 74 | `grade` 를 `OK ↔ DEGRADED` 로 3 tick 주기 왕복 | **System 전이 0회** (min dwell + 히스테리시스) |

- **62, 63, 64, 65 는 출발 게이트 회귀 고정**. 오출발은 즉시 실격
- **64 는 1-hop 의 안전 상쇄를 잡는 유일한 시나리오**. `state_machine` 이 헬스를 모른 채 `DRIVING` 으로 가도 차량이 서 있는지 확인
- **66 은 계약 50 을 잡는 유일한 시나리오**. Supervisor 사망이 `speed_limit` 까지 0 으로 만드는지 확인
- **68 은 상태 기계 교착을 잡는 유일한 시나리오**
- 두 컨트롤러 `deadman_timeout_ms` 불일치 → `INIT_FAILED` 검사는 **`01` 시나리오 6 에 항목 추가**로 처리 (새 번호 부여 안 함)

<br>

---

## 13. 확정 필요

### 13-1. 잠정 수치

| 항목 | 현재값 | 근거 | 필요 작업 |
| --- | --- | --- | --- |
| Deadman timeout | **250 ms** | 20 Hz 발행 × 5배 (`01` section 14-2 규칙) | rclpy 지터 실측: `01` section 14-2 의 150~200 ms 와 불일치 해소 |
| `/state_machine/speed_limit` stale timeout | 250 ms | `01` section 4-3, `02` section 5-3 과 일치 | 동일 |
| 정지거리 예산 | **400 ms = 80 cm @ 2.0 m/s** | 확정 150 ms + Deadman 250 ms | **구간 상한 결정에 직접 반영** (`02` section 12) |
| 출발 지연 | **100 ms (2 tick)** | `permit` 수신 → `DRIVING` 1 tick → `ENGAGED` 1 tick | `object_detection` 측 확정 프레임 수는 **별도 가산**. 계약 52 확정 후 합산 |
| 확정 조건 파라미터 | **본 문서에 없음** | `object_detection` 소관 (계약 52) | 해당 파트에서 실측·확정 |
| `/perception/start_permission` `gap` 임계 | 300~500 ms | 발행 10 Hz 확정 (`07` section 3-2) | `08` section 4-0 에서 확정 |
| `T_coast` | 3 s | 2.0 m/s ÷ 1.0 m/s² + 마진 | 마찰 감속도 실측 |
| 정차 소요 예산 | 10 s | 감속도 `a` 미실측 | 실측 후 확정 (`05` section 8-2) |
| `SHUTDOWN` 전체 예산 | 30 s | 노드 9개 × (deactivate 2 s + SIGTERM 3 s) 의 부분 중첩 가정 | 실측 |
| min dwell (section 8-3) | 0.5 / 1.0 / 2.0 s | 램프 시간 가정 | 실주행 진동 관측 |

### 13-2. 설계 확인

| 항목 | 현재 결정 | 확인 필요 사항 |
| --- | --- | --- |
| `TERMINATED` 진입 시 자식 처리 | **경로별로 다름**: `INIT` 경로(T3)는 자식 정리 후 진입(`01` section 6-3), 주행 중 경로(T13)는 자식 유지(`03` section 11-3) | 두 문서의 서술이 상충: 경로별 분기로 해석한 것이 맞는지 확인 |
| 1-hop 게이트 | `state_machine` 단독 판정·실행. 계약 50 은 **정지 조건 전용** (section 5-1) | `07` 에서 `start_request` 필드 **삭제** 반영 완료 |
| 상태 7종 병합 | `EMERGENCY`+`RECOVERY`→`STOPPED`, `COMPLETED`→`SHUTDOWN` | 로그에서 복구 단계 구분이 충분한지 실주행 확인 |
| `STOPPED` 중 새 고장 | 상태 전이 없음, 복구 실패로 계산 | 한도 소진 속도가 과도하지 않은지 |
| `* → SHUTDOWN` (SIGTERM) | 전 상태에서 허용 | `run.sh` 종료 방식·대회 진행자 개입 수단 확인 (`01` section 15-2) |
| 출발 후 `data` 가 `false` 로 복귀 | 이중 latch 로 주행 유지 | 신호등이 꺼진 뒤에도 주행 계속이 규정상 문제없는지 |
| **`RED` 관측 이력 조건 없음** | `object_detection` 확정 스펙은 **`GREEN` 5프레임 연속만** 요구 | ⚠ **기동 완료 시점에 이미 `GREEN` 이면 즉시 출발함.** 이전 주기의 `GREEN` 이나 대기 중 점등에 반응할 수 있는지 `object_detection` 파트·대회 진행 방식 확인 |
| `stopped` 속도원 | `/odometry/filtered` | `ekf` 출력 필드·주기 확인 (계약 57) |
| 상태 기계 전이 버그 | 검증으로만 커버 (section 9-2) | 상태 기계 자체의 워치독이 필요한지 판단 |
