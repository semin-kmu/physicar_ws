# kau_state_machine

차량 전체의 기동 · 감시 · 상태 판단 · 종료를 담당하는 패키지.

**단위: 길이 cm / 각도 deg / 속도 m/s** (팀 통일, `docs/경로_형식.md` 와 동일)

## 노드

| 노드 | 역할 | 분리 이유 |
| --- | --- | --- |
| `system_supervisor` | 프로세스 기동·감시·복구, Lifecycle 관리, enable 발행, 출발 게이트, 종료 | 검증 부담 최대 → 미션 로직 혼입 금지 |
| `state_machine` | Mission State + Behavior State | Supervisor 사망 시에도 분리 유지 |

- Supervisor 는 **자신을 제외한 전 노드**를 자식 프로세스로 소유
- `run.sh` → launch → `system_supervisor` **1개만** 기동, 나머지는 Supervisor 가 spawn

## 문서

`docs/` 하위. 각 문서는 **그대로 구현 착수 가능한 수준**을 목표로 함.

| 문서 | 내용 | 상태 |
| --- | --- | --- |
| **`00_치트시트.md`** | **현장 1페이지**. 상태·토픽·임계값·증상별 확인 순서 | 작성 완료 |
| `01_기동_종료_시퀀스.md` | 노드 순차 기동 절차, 준비 판정, 타임아웃/재시도, 종료 역순 | 작성 완료 |
| `02_감시_하트비트.md` | heartbeat 규약, stale 판정, 센서/노드 원인 구분, 이상 등급 | 작성 완료 |
| `03_고장대응_복구.md` | MRM, Soft/Hard 복구, 연쇄 오탐 억제, 재engage, 한도 | 작성 완료 |
| `04_system_state.md` | System 계층 전이, 출발 게이트(신호등), Deadman | 작성 완료 |
| `05_mission_state.md` | 구간/이벤트, 랩 진행도 `s`, 도착 판정, 정지거리 예산 | 작성 완료 |
| `06_behavior_state.md` | Behavior 4상태, 프로파일과의 직교 축, 전파 값 | 작성 완료 |
| `07_인터페이스.md` | 토픽·메시지·서비스·TF 정식 정의 (`kau_msgs`) | 작성 완료 |
| `08_설정_스키마.md` | bringup·mission·감시·복구 설정 통합, 파라미터 인벤토리 | 작성 완료 |
| `09_로깅_GUI.md` | 로그 스키마·링버퍼, PyQtGraph 관측 GUI 30 Hz | 작성 완료 |
| `10_테스트_검증.md` | 시나리오 전체 색인, 스텁 명세, 테스트 레벨, 커버리지 공백 | 작성 완료 |

설계 배경과 논의 경위는 `Notes/State_Machine_Study.md` / `Notes/State_Machine_확정.md` 참조.

### 문서 작성 규약

- **계약 번호 · 검증 시나리오 번호는 전 문서 통합 연번.** 병렬 작성을 위해 문서별 블록을 선점하므로 **번호 사이에 빈 구간이 생길 수 있음** (의도된 것)
- 전체 색인은 `10_테스트_검증.md`

| 문서 | 계약 | 시나리오 |
| --- | --- | --- |
| 01 | 1 ~ 18 | 1 ~ 17 |
| 02 | 19 ~ 38 | 18 ~ 40 |
| 03 | 39 ~ 48 | 41 ~ 59 |
| 00 | 없음 (요약 전용) | 없음 |
| 04 | 49 ~ 58 | 60 ~ 74 |
| 05 | 59 ~ 68 | 75 ~ 89 |
| 06 | 69 ~ 78 | 90 ~ 99 |
| 07 | 79 ~ 88 | 100 ~ 109 |
| 08 | 89 ~ 95 | 110 ~ 119 |
| 09 | 96 ~ 105 | 120 ~ 129 |
| 10 | 신설 없음 | 전체 색인 + **보완 130 ~ 137** |



### 미확정 · 타 파트 대기

| 대상 | 상태 |
| --- | --- |
| 전 노드의 **노드명 · 패키지명 · 실행파일명** | **미정**. `01` section 2-2 는 가정값 |
| 타 파트 **토픽명 · 메시지 타입** | **미정**. `07` section 2-3 에 확정 상태 표 |
| 전 노드 **동작 주기** | **미정**. G4·stale 임계가 전부 여기 종속 |
| `/perception/start_permission` | **확정** (`std_msgs/Bool`, 10 Hz) |
| `/speed` · `/steer` | 토픽명 확정 / **타입 미정** |

**전 파트 패키지가 한 폴더에 모인 뒤 일괄 확인.** 절차와 반영 위치는 `08` section 4-0.
이름·주기가 미정이어도 설계는 성립함 — 전부 `bringup.yaml` 데이터이고 코드 상수가 아니기 때문 (`08` 계약 90).

### 구현 범위

**1차 = 전체.** 단계 축소 없이 전 기능을 대회 목표로 삼는다 (사용자 확정).
Soft Reset · `lane_only` · `speed_controller` 대행 · 관측 GUI 전부 포함.

### 전 문서 공통 확정

| 항목 | 확정 |
| --- | --- |
| 정지 수단 | `/state_machine/enable`(Supervisor, 끊김=정지) · `/state_machine/speed_limit`(state_machine, 0=정지) **2계층뿐**. 모든 정지는 **즉시 0** |
| 속도 | **구간 상한 대비 비율로만** 규정. 절대값은 구간 상한 하나 |
| 발행 | 감시·평가 tick **안에서** 발행. 별도 타이머 금지 |
| 경로 | `kau_msgs/KauPath`. quintic Bezier 제어점, 이산 좌표 금지 |
| 단위 | 길이 cm / 각도 deg / 속도 m/s |
| 구현 언어 | `system_supervisor` · `state_machine` 둘 다 `rclpy` |
| System 상태 | **7종**. `INIT` `STANDBY` `ENGAGED` `DEGRADED` `STOPPED` `SHUTDOWN` `TERMINATED` |
| 출발 게이트 | **1-hop**. `/perception/start_permission`(`std_msgs/Bool`, 10 Hz) 을 받아 `state_machine` 단독 실행. 헬스 거부권은 `/state_machine/enable` |
| RMW | `rmw_fastrtps_cpp` (`rmw_zenoh` 사용 불가) |

## 확정

| 항목 | 내용 |
| --- | --- |
| 구현 언어 | 두 노드 모두 **`rclpy`**. 위험과 대응은 `01` section 14 |
| 센서 드라이버 | 대회 차량 제공, 토픽 이미 발행 중 — 관리 대상 아님 |
| RMW | `rmw_fastrtps_cpp` (`rmw_zenoh` 사용 불가) |

## 미확정

| 항목 | 비고 |
| --- | --- |
| `localization` 알고리즘 | Cartographer 우선, AMCL 병행 검토. 복구 전략에 직결 (`01` section 5) |
| 빌드 파일 (`package.xml`, `setup.py`) | 구현 착수 시 추가 |
