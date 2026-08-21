# 09. 로깅 / GUI

주행 중 발생한 사실을 사후 분석 가능하게 남기는 **로컬 로깅**과, 그 사실을 실시간으로 훑어보는 **read-only 원격 GUI** 를 정의.
로깅은 GUI 연결 여부와 무관하게 완결되어야 하며, GUI 는 주행 판단에 어떤 영향도 주지 않음.

> 전제: 로깅 대상·정책은 `02` section 9 확정(전이 시점만 기록, 30 s 링버퍼, `FAULT` 시 flush)을 그대로 따름. 이 문서는 그 확정을 구체화.

<br>

## 1. 범위

| 포함 | 미포함 |
| --- | --- |
| 로그 저장 위치·포맷·회전 | 이상 검출·등급 산출 로직 → `02` |
| 로그 레코드 스키마 | 상태 전이 조건 자체 → `04`/`05`/`06` |
| 링버퍼·`FAULT` 덤프 구체화 | 인터페이스 정식 정의 (토픽·메시지) → `07` |
| `run_id` 디렉터리 구조, stdout/stderr 관계 | |
| GUI 화면 구성·통신·성능 | GUI 코드 구현 |
| GUI 구독 토픽·대역폭·QoS | |

<br>

---

## 2. 원칙

| 원칙 | 내용 |
| --- | --- |
| **로깅과 GUI 는 독립** | GUI 미접속 상태에서도 사후 분석이 완결되어야 한다. 로그는 Supervisor 가 로컬 파일로 직접 기록: GUI 구독과 무관 |
| **GUI 는 read-only** | 차량으로 명령을 보내지 않는다. 원격 정지는 GUI 경로가 아닌 독립 E-stop |
| **무선 단절 = GUI 문제, 주행 문제 아님** | GUI 가 끊겨도 차량은 영향받지 않는다. 판정은 여전히 Supervisor 단일 지점 (`02` section 2) |
| **GUI 는 대회 실주행 경로 밖** | `run.sh` 단독 실행 규정(`State_Machine_확정`) → GUI 는 bringup manifest(`01` section 8)에 없다. 개발·연습 전용, 차량 측은 발행만 |
| **판정 로직을 GUI 에서 중복 구현하지 않는다** | stale·등급 판정은 Supervisor 결과를 그대로 받아 표시. GUI 가 독자적으로 재판정하면 표시와 실제 판정이 어긋날 수 있음 |

<br>

---

## 3. 로깅 · 저장 위치·포맷·회전

### 3-1. tmpfs 미채택 · 직접 영구 저장으로 확정

`01` section 15 의 미확정 항목("SD 수명 고려, tmpfs + 종료 시 flush 검토")에 대한 결정.

| 검토 | 판단 |
| --- | --- |
| 기록 대상이 이미 저빈도(`02` section 9: 전이 시점만) | 이벤트 로그·tick 통계 쓰기 자체가 SD 마모 요인이 되기 어려움 |
| 1 랩 기준 예상 기록량 (**잠정**, 근거 section 3-2) | 이벤트 수십~수백 건 + tick 통계 1 Hz + `FAULT` 덤프 수 회 ≈ 수 MB/run |
| tmpfs 도입 시 위험 | 전원 급차단 시 flush 전 로그 **전체 소실**: 사고 직후가 가장 필요한 순간에 데이터가 없어짐 |

→ **tmpfs 사용하지 않음.** `/var/log/kau/{run_id}/` 에 **직접 기록**한다 (`01` section 8 `bringup.yaml` 의 `log_dir` 규약과 동일 경로 체계).

- 근거 요약: 기록량 자체가 작아 SD 마모 이득보다 전원차단 시 유실 위험이 더 크다 (`FAULT` 직후가 바로 사후 분석이 가장 필요한 시점)
- 실제 SD 모델·용량·대회 실행 횟수는 미확인 → section 16 확정 필요

### 3-2. 기록량 추정 (잠정, RPi5 실측 전)

| 항목 | 산출 근거 | 추정 |
| --- | --- | --- |
| 이벤트 레코드 | 전이·기동·복구 이벤트 수백 건, 레코드당 ≈ 200 B (JSON) | 수십 KB/run |
| tick 통계 | 1 Hz × run 길이(≈ 수 분), 레코드당 ≈ 100 B | 수십 KB/run |
| `FAULT` 덤프 | 30 s × 20 Hz(Supervisor tick) = 600줄, 줄당 ≈ 500 B~1 KB, `03` section 11 한도(Soft 2·Hard 2·재engage 3)로 회수 제한 | 300 KB~1 MB / 회, run 당 최대 수 회 |
| **run 당 합계 (잠정)** | | **수 MB 이내** |

### 3-3. 포맷 · JSON Lines

- 한 줄 = 한 JSON 객체. append-only, 파싱 단순, 사람이 눈으로도 확인 가능
- 별도 DB·바이너리 포맷 도입하지 않음. 사후 분석 도구가 아직 없는 상태에서 과설계 지양

### 3-4. 회전 정책

| 항목 | 정책 |
| --- | --- |
| 단위 | `run_id` 디렉터리 |
| 자동 삭제 | **없음** (1차 구현): section 3-2 추정이 맞다면 대회 당일 실행 횟수로는 용량 문제 없음 |
| 용량 부족 감지 | `01` section 10 사전 점검 "로그 디렉터리 쓰기 가능·여유 공간" 에서 이미 커버 |
| 정리 | 대회 후 운영자 수동 정리 |

<br>

---

## 4. 기록 대상

`02` section 9 확정 그대로: **전이 시점만.** 매 tick 기록 금지. 대상을 아래로 구체화.

| 종류 | 내용 | 근거 |
| --- | --- | --- |
| 노드 등급 전이 | `OK`/`DEGRADED`/`FAULT` 전이 | `02` section 7 |
| System/Mission/Behavior 상태 전이 | 전이 시점의 이전값·새값 | `04`/`05`/`06` 확정 시 필드 확정 (section 16) |
| 기동 이벤트 | 관문 통과/실패 `(node, gate, elapsed, result)` | `01` section 7-2 |
| 복구 이벤트 | Soft/Hard 시도, 성공/실패, 재engage, 복구 포기 | `03` |
| tick 통계 | 1 s 마다 `tick_duration` 평균·최대만 | `02` section 9 |

**매 tick 기록 금지가 절대 원칙**. 등급·상태가 그대로여도 매번 쓰면 section 3-1 의 저빈도 전제가 깨짐.

<br>

---

## 5. 링버퍼 → `FAULT` 덤프

`02` section 9 확정을 그대로 구체화.

| 항목 | 규정 |
| --- | --- |
| 보관 위치 | Supervisor 메모리 (파일 아님) |
| 보관 길이 | 최근 **30 s** |
| 샘플 주기 | Supervisor 감시 tick 과 동일 (**20 Hz**, `02` section 8) → 최대 600줄 |
| 레코드 내용 | 해당 tick 의 전 노드 등급, heartbeat age/gap, 입력 stale 여부, `/state_machine/enable`·`/state_machine/speed_limit` 값, 현재 프로파일 |
| 덤프 트리거 | `FAULT` 등급 확정 시점 (`02` section 5-2, 연속 3 tick) |
| 덤프 대상 파일 | `faults/{ts}_{node_id}.jsonl` |
| 실패 처리 | 파일 쓰기 실패해도 tick(section 8-1, `02`)은 계속 진행: 로깅이 감시를 막지 않는다 |

- 이상 발생 **직전** 데이터가 원인 분석의 전부 (`02` section 9) → 매 tick 기록을 안 하는 대신, 딱 그 30 s 는 통째로 남김
- `03` section 11 한도 초과로 `TERMINATED` 진입 시에도 동일 덤프가 이미 남아있음 (직전 `FAULT` 경유)

<br>

---

## 6. 로그 레코드 스키마

### 6-1. 이벤트 레코드 (`events.jsonl`)

| 필드 | 타입 | 설명 |
| --- | --- | --- |
| `ts` | float64 (unix s) | 이벤트 시각 |
| `kind` | enum | 아래 section 6-2 |
| `node_id` | string \| null | 대상 노드. System 전이는 `null` |
| `from` | string | 이전 값 |
| `to` | string | 새 값 |
| `reason` | string | age/gap 실측값, 실패 사유 등 (`02` section 9 "근거"와 동일 취지) |
| `seq` | uint32 \| null | 관련 heartbeat `seq` (해당 시) |

### 6-2. `kind` 값

| kind | 의미 |
| --- | --- |
| `GATE` | 기동 관문 통과/실패 (`01` section 3) |
| `NODE_GRADE` | 노드 등급 전이 (`02` section 7) |
| `SYSTEM_STATE` / `MISSION_STATE` / `BEHAVIOR_STATE` | 계층별 상태 전이 (`04`/`05`/`06`) |
| `FALLBACK_PROFILE` | `lane_centering`/`global_only`/`lane_only` 전환 (`02` section 7-4) |
| `RECOVERY_SOFT` / `RECOVERY_HARD` | 복구 시도 (`03` section 5). **`STOPPED` 내부 단계는 상태가 아니라 이 로그로 구분** (`04` section 4-3) |
| `REENGAGE` | 재engage 시도 (`03` section 10) |
| `GIVE_UP` | 노드 복구 포기 (`03` section 11-2) |
| `TERMINATED` / `SHUTDOWN` | 종료 (`01` section 11-3) |

### 6-3. tick 통계 레코드 (`tick_stats.jsonl`)

| 필드 | 타입 | 설명 |
| --- | --- | --- |
| `ts` | float64 | 1 s 창의 종료 시각 |
| `tick_duration_mean_ms` | float32 | (`02` section 8-2) |
| `tick_duration_max_ms` | float32 | |

### 6-4. 링버퍼 레코드 (`faults/*.jsonl`, section 5 의 600줄 각각)

| 필드 | 타입 | 설명 |
| --- | --- | --- |
| `ts` | float64 | 해당 tick 시각 |
| `grades` | object | `{node_id: OK\|DEGRADED\|FAULT}` |
| `heartbeats` | object | `{node_id: {age_ms, gap_ms, seq}}` |
| `input_stale` | object | `{node_id: [stale 입력 토픽명]}` (`02` section 6) |
| `enable` | bool | |
| `speed_limit` | float32 | |
| `profile` | string | 현재 fallback 프로파일 |

<br>

---

## 7. `run_id` 디렉터리 구조

```
/var/log/kau/{run_id}/
├── manifest.yaml           # 실행 당시 bringup.yaml 스냅샷 (재현성)
├── events.jsonl             # section 6-1
├── tick_stats.jsonl         # section 6-3
├── faults/
│   └── {ts}_{node_id}.jsonl # section 6-4
└── nodes/
    ├── system_supervisor.out
    ├── state_machine.out
    └── ...                  # 노드별 stdout/stderr (01 section 9-1 그대로)
```

- `run_id`. 타임스탬프 기반 (예: `20260818_143200`). `01` section 8 `bringup.yaml` 의 `log_dir: /var/log/kau/{run_id}` 와 동일 규약
- `nodes/*.out` 은 **로깅과 별개 채널**. `01` section 9-1 의 stdout/stderr 리다이렉트가 그대로 이 디렉터리 하위에 위치. crash 원인은 여기(stderr)에만 남는다는 원칙(`01` section 6-3) 유지
- `manifest.yaml` 스냅샷은 노드 구성이 실행마다 달랐을 가능성(수동 테스트 등)에 대비. 사후 분석 시 "무엇을 기동했는지" 자체가 불확실해지는 것을 방지

<br>

---

## 8. GUI · 개요

| 항목 | 확정 |
| --- | --- |
| 성격 | **관측 전용 (read-only)**. 차량 방향 publisher/service client **없음** |
| 원격 정지 | GUI 경로 아님. 독립 E-stop 하드웨어 |
| 대회 실주행 편입 | **금지**: `run.sh` 단독 실행 규정, bringup manifest 에 GUI 없음 |
| 용도 | 개발·연습 전용. 사용해도 무방하나 접속 여부가 주행에 어떤 영향도 주면 안 됨 |
| 실행 위치 | 노트북 (차량 RPi5 아님) |
| 구현 | `rclpy` + PyQtGraph: Supervisor/`state_machine` 과 동일 클라이언트 라이브러리 (검증된 패턴) |

- 차량 측은 GUI 를 위해 **아무것도 추가하지 않음**. 이미 있는 토픽을 발행만 함
- GUI 노드는 Lifecycle 이 아님. Supervisor 가 관리하는 대상이 아니므로 configure/activate 관문이 없음

<br>

---

## 9. GUI · 통신

| 항목 | 내용 |
| --- | --- |
| 네트워크 | 노트북·차량 동일 네트워크(대회 규정). **인터넷 없음** |
| 연결 방식 | 별도 브릿지 없이 **DDS 직접 참여**: `ROS_DOMAIN_ID` 를 차량과 동일하게 설정 |
| RMW | `rmw_fastrtps_cpp` (전체 시스템 확정, `01` section 15). `rmw_zenoh` **사용 불가** |
| Discovery | FastDDS 기본은 UDP 멀티캐스트. 대회장 Wi-Fi AP 가 멀티캐스트를 차단/유니캐스트 변환하면 discovery 실패 가능 |
| 대응 | AP 설정 사전 확인. 멀티캐스트 불가 시 `ROS_STATIC_PEERS` 로 유니캐스트 peer 강제 지정 검토 |
| GUI reader QoS | 가능하면 `BEST_EFFORT` 요청: `RELIABLE` writer 와도 호환되며(DDS QoS 하위호환), 재전송 트래픽을 GUI 쪽에 발생시키지 않음. **FastDDS 실측 필요** (section 16) |
| History depth | GUI 구독은 `KEEP_LAST` + 얕은 depth(1~2): 단절 후 재접속 시 밀린 메시지가 쌓이지 않도록 |

**차량 내부 노드 간 통신은 GUI 무선 연결과 무관**. 전부 같은 머신(RPi5) 위의 로컬 DDS. 다만 발행 노드가 GUI 리더 하나를 추가로 서빙하는 비용(직렬화·전송)은 발생 → section 10 의 옵션화가 필요한 이유

<br>

---

## 10. GUI · 구독 토픽 · 대역폭 · QoS

### 10-1. 기본 구독 (30 Hz 갱신은 GUI 렌더 주기이며, 아래 발행 주기와 독립, section 12)

| 토픽 | 타입 | 발행 주기 | 메시지 크기 (잠정) | 대역폭 (잠정) |
| --- | --- | --- | --- | --- |
| `/path/global` | `KauPath` | latched (1회) | ≈ 3.2 KB (`경로_형식.md` section 9.10, 30 m 트랙 기준 근사) | ≈ 0 |
| `/path/local` | `KauPath` | 미정 | 272 B (5 m 기준, section 9.10) | 최대 2.7 KB/s (10 Hz 가정) |
| `/lane/center` | `KauPath` | 미정 | 164 B (section 9.10) | 1.6 KB/s (10 Hz 가정) |
| `/diag/heartbeat` | `Heartbeat` | 10 Hz × A분류 7 노드 = 70/s | ≈ 150 B (section 4-1 필드 구성 기준 추정) | ≈ 10.5 KB/s |
| `/perception/objects` | `kau_msgs/DetectedObjectArray` | 미정 | ≈ 200 B (라바콘 최대 6개) | ≈ 2 KB/s (10 Hz 가정) |
| `/state_machine/mission` | `kau_msgs/MissionState` | 20 Hz | ≈ 100 B | ≈ 2 KB/s |
| `/state_machine/speed_limit` | float32 + enum | 20 Hz | ≈ 50 B | ≈ 1 KB/s |
| `/state_machine/enable` | bool + stamp | 20 Hz | ≈ 30 B | ≈ 0.6 KB/s |
| `/state_machine/profile` | `kau_msgs/PlannerProfile` | 전환 시 + 저주기 | 소량 | ≈ 0 |
| `/state_machine/health` | `kau_msgs/SupervisorHealth` | 5 Hz | ≈ 300 B (노드 10개 등급 요약) | ≈ 1.5 KB/s |
| **합계 (잠정)** | | | | **≈ 28 KB/s** (`07` section 9-2 갱신값) |

토픽명·타입은 `07` section 3 확정본. `/state_machine/health` 는 본 문서가 제안해 **신설 확정**된 토픽 (`07` section 3-5).

### 10-2. 기본 제외 · 옵션 토글

| 토픽 | 사유 |
| --- | --- |
| 카메라 원본 | 수백 KB/s~수 MB/s. 무선 혼잡 유발 |
| `/scan` (LiDAR) | 동일 |

- 대용량 토픽은 GUI 설정에서 **명시적으로 켜야만** 구독. 기본은 꺼짐
- `01` section 10 "Supervisor 는 이미지·LiDAR 원본 토픽을 구독 금지" 규칙은 Supervisor 전용이지만, GUI 가 이를 구독하면 **차량 측 발행 부하**(추가 리더 서빙 비용)가 그대로 생김 → 옵션화로 대응

### 10-3. 총량 판단

- 기본 구독 세트(≈ 28 KB/s)는 일반 Wi-Fi 대역폭 대비 무시 가능한 수준 → **정상적인 무선 환경에서는 대역폭 자체가 위험 요인이 아님**
- 실질 위험은 (1) 대용량 옵션 토픽 활성화, (2) discovery 폭주(리더 매칭 오버헤드). 둘 다 section 9·section 10-2 로 대응

<br>

---

## 11. GUI · 화면 구성

**시뮬(`viz.py`/각 `viewer.py`)과 동일 레이아웃 원칙**: 오른쪽 = map, 왼쪽 = 지표 패널.
단, 시뮬은 배치 재생(탭 구조)이고 GUI 는 **실시간 단일 뷰**. `sim_realtime.py` 계열에 더 가까움.

### 11-1. map 패널 (오른쪽)

| 표시 항목 | 소스 | 비고 |
| --- | --- | --- |
| Global Path | `/path/global` (`KauPath`) | 제어점 그대로 수신, **표시 시점에만 로컬 샘플링** (`경로_형식.md` section 9.6~9.7 복원·평가 routine 재사용, GUI 가 별도 곡선 재구성 로직을 만들지 않는다) |
| Local Path | `/path/local` | 동일 |
| 차선 (Lane Detection) | `/lane/center` | 동일 |
| 차량 자세 | TF (`map→base_link`, 없으면 `odom→base_link`) | `viz.py` 의 `car_shape()` 와 동일 solid 표시 |
| 장애물 (index 표시) | `/perception/objects` | 원판 + `index` 라벨 (계약 37, 프레임 간 id 유지 전제) |
| 랩 진행도 | `/state_machine/mission` 의 `s` | 진행 바 또는 텍스트 오버레이. 전장 값은 실제 대회 트랙 확정 전까지 참고치 |

**`KauPath.header.frame_id` 에 따라 렌더 기준이 갈림**. `map` 이면 절대좌표로, `base_link` 이면 차량 상대로 그림 (`lane_only` 모드, `02` section 7-4). `lane_only` 중에는 map 프레임 자체가 없으므로 절대좌표 표시가 **정지되거나 고정**됨 → section 11-3 stale 표시로 사용자에게 알림

### 11-2. 지표 패널 (왼쪽)

| 표시 항목 | 소스 | 형식 |
| --- | --- | --- |
| System / Mission / Behavior 상태 | 상태 토픽 (`04`/`05`/`06`) | 텍스트/배지 |
| 현재 프로파일 | `/state_machine/profile` | 텍스트 (`lane_centering`/`global_only`/`lane_only`) |
| `/state_machine/speed_limit` | `/state_machine/speed_limit` | 숫자 + 시계열 그래프 (`viz.stack_plots` 패턴) |
| enable 상태 | `/state_machine/enable` | on/off 인디케이터 |
| 노드별 등급·heartbeat | `/state_machine/health` | 표 (`node_id` \| 등급 \| age \| seq): `_summary_widget` 패턴 |
| tick 통계 | `/state_machine/health` | 숫자 또는 시계열 |
| 복구 이력 | `/state_machine/health` | 최근 N건 리스트 (node, 시각, soft/hard, 성공/실패) |

- 등급 색상: `OK` = 녹색 / `DEGRADED` = 주황 / `FAULT` = 빨강 (신호등과 무관한 별도 배색, 혼동 방지)
- 그래프 이력은 GUI 로컬 롤링 버퍼(예: 최근 30 s)로만 유지. **로깅과 무관**, GUI 재시작 시 소실되어도 무방 (section 2 로깅·GUI 분리)

### 11-3. 조작키 · 시뮬 규약을 실시간 관측용으로 재해석

| 키 | 시뮬 (배치 재생) | GUI (실시간) |
| --- | --- | --- |
| `space` | 재생 정지 | **화면 갱신 일시정지** (수신은 계속, 렌더만 멈춤, 순간 관찰용) |
| `up`/`down` | 배속 | **미사용**: 라이브 데이터라 배속 개념 없음 |
| `r` | 처음부터 | **뷰 리셋** (팬/줌 초기화) |

- `space`/`r` 은 **표시 상태만** 바꿈. 차량에는 어떤 영향도 없음 (section 2 read-only 원칙)

<br>

---

## 12. GUI · 성능

| 항목 | 규정 |
| --- | --- |
| 갱신 주기 | **30 Hz**: Qt `QTimer(33 ms)` 로 화면만 재그림. ROS 콜백 도착 주기와 **독립** |
| 스레드 구조 | `rclpy` spin 은 별도 스레드, 콜백은 최신값만 공유 dict 에 저장(래치). Qt 메인스레드 타이머가 그 dict 를 읽어 렌더: Qt/rclpy 스레드 안전성 확보 |
| 렌더 부하 근거 | PyQtGraph 30 Hz 실시간 렌더는 각 파트 `sim_realtime.py` 에서 이미 검증된 패턴 (`src/README.md`) |
| 수신 누락 시 표시 | 토픽별 마지막 수신 시각 추적. `now - last_recv > 임계(발행주기 3배, `02` section 5-2 와 동일 비율)` 시 해당 패널 항목을 **회색/STALE 배지**로 전환 |
| 전체 단절 표시 | `/diag/heartbeat`(또는 `/state_machine/health`) 전체 stale 시 상단 배너로 "연결 끊김" 표시 |

- **GUI 측 stale 판정은 표시 전용**. 어떤 형태로도 차량에 피드백되지 않음 (section 2 재확인)

<br>

---

## 13. GUI · 구현·실행 환경

| 항목 | 확정 |
| --- | --- |
| 언어 | Python, PyQtGraph |
| 의존성 | `numpy`, `PyQt5`, `pyqtgraph`, `rclpy` (ROS 설치에 포함): `src/README.md` 관례와 동일 |
| 실행 위치 | 노트북. `venv` 권장 |
| ROS 패키지 여부 | 별도 패키지(예: `kau_gui`)로 두되 **bringup manifest 에는 포함하지 않음**: Supervisor 가 spawn 하지 않는다 |
| Lifecycle | 아님. 단순 `rclpy.Node` |

<br>

---

## 14. 각 파트 담당자 계약

`03` section 13 에 이어짐.

| # | 요구 | 이유 |
| --- | --- | --- |
| 96 | Supervisor: 이벤트 로그를 감시 tick(20 Hz) 흐름 안에서 기록: 등급/상태 전이 검출 시점에 append | 로깅 로직이 별도 타이머로 빠지면 `01` section 14-1 과 같은 은폐 위험 재현 |
| 97 | Supervisor: `FAULT` 확정 시 30 s 링버퍼를 즉시 flush. 쓰기 실패해도 tick 진행에 영향 없게 예외 처리 | 로깅이 감시를 막아서는 안 됨 (section 5) |
| 98 | Supervisor: `run_id` 디렉터리 생성 + `bringup.yaml` 스냅샷 저장 | 사후 재현성 (section 7) |
| 99 | 각 노드: stdout/stderr 는 `01` section 9-1 리다이렉트만 하면 됨. 추가 로깅 API 구현 불필요 | 로깅은 Supervisor 단일 책임 |
| 100 | `object_detection`: `/perception/objects` 의 `index` 를 GUI 표시용으로도 안정 유지 (계약 37 재확인) | GUI 가 별도 매칭 로직을 만들지 않도록 |
| 101 | `state_machine`/`local_path_planner`: 프로파일 상태를 토픽으로 노출 (계약 43 활용) | GUI "현재 프로파일" 표시 (section 11-2) |
| 102 | Supervisor: 노드별 등급·heartbeat·tick 통계·복구 이력을 집계해 **`/state_machine/health` 로 5 Hz 발행** (신설 확정) | GUI 가 stale·등급 판정을 중복 구현하지 않도록 (section 2, section 11-2) |
| 103 | 전 노드: `/diag/heartbeat` QoS(`best_effort`)를 GUI 구독을 이유로 변경하지 않음 | 제어 신뢰성이 관측 편의보다 우선 |
| 104 | Supervisor/`state_machine`: GUI 로부터의 명령을 수신하는 구독자·서비스를 만들지 않는다 | read-only 원칙 (section 2, section 8) |
| 105 | `kau_msgs`: `/state_machine/health`(`SupervisorHealth`)·`/state_machine/profile`(`PlannerProfile`) 를 `07` section 5 정의대로 구현 | 인터페이스 일원화 |

<br>

---

## 15. 검증

`03` section 14 에 이어짐.

| # | 시나리오 | 기대 |
| --- | --- | --- |
| 120 | 정상 1랩 주행 후 로그 확인 | `events.jsonl` 에 상태/등급 전이·기동 이벤트만 기록, 매 tick 기록 없음 |
| 121 | `FAULT` 발생 | `faults/` 에 30 s 분량 jsonl 생성, 등급 확정 직전 데이터 포함 |
| 122 | GUI 미연결 상태로 전체 시나리오 실행 | 로그(이벤트 + 링버퍼)만으로 사후 원인 재구성 가능 |
| 123 | `run.sh` 도중 전원 급차단 | 마지막 flush 이전까지만 보존. 재기동 시 새 `run_id` 생성 |
| 124 | GUI 접속 중 무선 단절 | 차량 주행 지속(속도·조향 변화 없음), GUI 는 각 패널 STALE 표시로 전환 |
| 125 | GUI 미접속 상태에서 정상 기동·주행 | 기동 절차 지연·실패 없음 (GUI 가 전제조건이 아님을 확인) |
| 126 | GUI 재접속(단절 후 재연결) | latched 토픽(`/path/global`)은 즉시 재수신, 나머지는 다음 발행부터 갱신 |
| 127 | GUI 구독 상태에서 무선 대역폭 실측 | section 10-1 추정치(≈ 28 KB/s) 대비 실측 근접 확인 (대용량 옵션 비활성 기준) |
| 128 | GUI → 차량 방향 송신 여부 점검 | 코드 검사 + 네트워크 캡처로 publisher/service client 부재 및 송신 0건 확인 |
| 129 | `run.sh` 단독 실행 시 GUI 프로세스 흔적 | bringup manifest(`01` section 8)에 GUI 부재, Supervisor 가 GUI 를 spawn 하지 않음 확인 |

- 124, 125, 128 은 **read-only·독립성 검증**. 대회 규정 직결이므로 회귀 테스트에 고정

<br>

---

## 16. 확정 필요

| 항목 | 현재 가정 | 필요 작업 |
| --- | --- | --- |
| 로그 저장소(tmpfs 미사용) | section 3-2 추정 기록량 기반 결정 | 실제 SD 모델·용량, 대회 당일 실행 횟수 확인 후 재검토 |
| `/diag/heartbeat` 메시지 크기 | ≈ 150 B (필드 구성 추정) | `kau_msgs/Heartbeat.msg` 확정(`07`) 후 실측 |
| `SupervisorHealth` 필드 구성 | `07` section 5 정의 | 복구 이력 보관 건수 등 세부는 실사용 후 조정 |
| GUI reader `BEST_EFFORT` 오버라이드 호환성 | DDS 스펙상 호환 가정 | FastDDS 실측 필요 |
| Wi-Fi 멀티캐스트 discovery | 대회장 AP 환경 불명 | 사전 확인, 필요 시 `ROS_STATIC_PEERS` 검토 |
| GUI 대역폭 총합 (≈ 28 KB/s) | `07` section 9-2 필드 크기 기반 | section 15 시나리오 127 실측 |
| 랩 진행도 표시 기준 전장 | 시뮬 참값 2722.5 cm | 실제 대회 트랙 확정 후 교체 |
| GUI 대역폭 산출의 발행 주기 | 10 Hz 가정 | `08` section 4-0 실측 후 재계산 |
| `04`/`05`/`06` 상태 토픽 필드명 | 미확정 (문서 예정) | 해당 문서 확정 후 section 6-1 `kind`, section 11-2 표 갱신 |

<br>
