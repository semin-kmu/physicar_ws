# 백업 풀스택 설계 확정본

`Backup_Full_Stack_Structure.md` 초안의 확정본. 초안과 다른 결정은 근거를 함께 남긴다.

---

## 0. 원칙

| 항목 | 결정 |
|---|---|
| 기존 `kau_*` 스택과의 관계 | **배타적 실행.** `run.sh` 이름 하나로 보장 (§14) |
| 좌표계 | `base_link` 상대좌표 전용. `map`/`odom` TF 불필요 |
| 단위 | **전부 m·rad.** `kau_msgs`의 cm/m 혼용을 승계하지 않는다 |
| 기하 소유 | **경로 생성은 Path Planner 단독.** 검출기와 제어기는 경로를 만들지 않는다 |
| 지연 처리 | **상대좌표 메시지를 쓰는 모든 소비자가 stamp → now 데드레커닝을 한다.** 지터 최대 원인 (§7-A) |
| 파라미터 | 노드별 **`config/*.yaml` 에 전부 노출.** 코드에 상수 리터럴을 두지 않는다 |
| 코드 재사용 | `bezier.hpp`, intrinsic은 기존 패키지에서 **복사**. 빌드 의존은 두지 않는다 |

패키지 — 코드 7개 + launch 전용 1개

```
backup_msgs              메시지 정의            ament_cmake
backup_lane_detection    차선 기하 검출          ament_cmake (C++)
backup_object_detection  장애물 + 출발신호       ament_cmake (C++)
backup_path_planner      경로 생성              ament_cmake (C++)
backup_speed_controller  /speed                ament_cmake (C++)
backup_steer_controller  /steering             ament_cmake (C++)
backup_gui               kau_gui 복제           ament_python
backup_bringup           launch + config       ament_cmake (코드 없음)
```

---

## 1. 확정 파라미터

| 기호 | 값 | 출처 |
|---|---|---|
| `L` 휠베이스 | 0.180 m | 확정 |
| 차량 폭 × 길이 | **0.205 × 0.280 m** | 확정 |
| 후륜축 → 앞범퍼 | 0.230 m | `L` + 전방 오버행 0.05 |
| `δ_max` | **20°** | 확정 (`steer_controller.yaml:32` "하드웨어 한계") |
| `R_min` = `L/tan(δ_max)` | **0.4945 m** | 파생 |
| `V_max` | 1.5 m/s | §3 |
| `v_min` | 0.5 m/s | 초안 |
| `a_accel` | 1.5 m/s² | 초안 |
| `a_decel` | 0.5 m/s² | 초안 |
| `a_lat_max` | 3.0 m/s² **(잠정)** | 실측 필요 §16 |
| `d_sight` | 1.5 m **(목표)** | §4 카메라 틸트로 결정 |
| `t_lat` | **0.10 s** | 모션 보상 적용 시 (§7-A 1위). 미적용이면 0.25 s |
| `k_v` / `Ld_min` / `Ld_max` | **0.85 / 0.8 / 1.2 m** | §11 위상여유 유도 |
| `τ` 조향 응답 시정수 | 0.2 s **(추정)** | 실측 필요 §16 |
| 트랙 폭 | **0.700 m** | `world.json` 실측 |
| 장애물 반경 | **0.142 m** | 모든 장애물을 이 원으로 처리 (확정) |
| 회피 여유 | **0.10 m** | 장애물 표면 ↔ 차체 (확정) |
| 차선 여유 | 0.05 m (편측) | 차선 이탈은 실격 |

조향 액추에이터(600 deg/s)는 제약이 아니다. 0→20° 전타가 **33 ms**다.

### 선회 시 차체 소요 폭

직사각 차체는 곡선에서 정지 폭보다 넓게 쓸린다. `R_out = hypot(R + 폭/2, 0.230)`, `R_in = R − 폭/2`.

| R [m] | 소요 폭 | 정지폭 대비 | 트랙 70 cm 중 편측 여유 |
|---|---|---|---|
| 0.296 | 26.7 cm | 1.30× | 21.7 cm |
| 0.4945 | 24.8 cm | 1.21× | 22.6 cm |
| 1.390 | 22.3 cm | 1.09× | 23.9 cm |

**코리도 폭은 이 트랙에서 제약이 아니다.** 최악에도 편측 21.7 cm가 남는다.

---

## 2. 트랙 실측 — `sim/api/world`

> **런타임에 쓰지 않는다.** 백업 스택은 상대좌표 전용이고 카메라가 본 차선만 본다.
> 아래는 설계 파라미터 산출용 오프라인 근거이며 패키지에 포함하지 않는다.

| 항목 | 값 |
|---|---|
| 월드 | `custom_71e69ee938032295503bfed557fde18c` (AMET 2026) |
| 트랙 | 폐루프, 길이 **30.5 m**, 폭 **70.0 cm** (코너 최대 85) |
| 코너 | **16개**, 직선 비율 70% |
| 중앙선 `R_min` | **0.151 m** (40 cm 창) |
| 장애물 | 콘 6개, 18×18 cm, 중앙선에서 **±18 cm** 좌우 번갈아, **전부 직선 구간** |
| 신호등 | `light1`, s=0.60 m, 중앙선 우측 39.4 cm |

코너 분포 — s=1.9 / 11.4 는 Δψ≈99°의 고립된 직각 코너(전후 직선 3.8 / 9.0 m).
**s=12.4~28.1 구간에 14개가 직선 0.3~1.7 m를 사이에 두고 연속**한다.

### ⚠ 조향 여유가 빠듯하다 — 통과 가능 여부 미확정

중앙선 `R_min` 0.151 m 는 차량 한계 `R_min` 0.4945 m (δ 20°) 를 크게 밑돈다.
**중앙선 추종은 애초에 불가능하고, 레이스 라인이 유일한 수단이다.**

레이스 라인으로 충분한지는 **해석으로 판정되지 않았다** — 방법에 따라 결론이 갈린다.
근거와 채택 방침은 §15. 설계는 어느 쪽이든 성립하는 형태로 되어 있다.

이건 설계가 아니라 차량-트랙 조합의 기하 문제이며, 기존 `kau_*` 스택도 동일한 제약을 받는다.

### out-in-out 이 여전히 필수인 이유

```
R_width = W / (1 − cos(θ/2))
R_sight = (d_sight − v·t_lat) / tan(θ/2)      진입/탈출 직선이 R·tan(θ/2) 만큼 필요
R       = min(R_width, R_sight)
W       = 0.700 − sweptWidth(R) − 2×0.05      ★ 정지폭이 아니라 선회 소요폭
```

**`W` 가 `R` 에 의존하므로 반복해서 푼다.** `sweptWidth` 는 R 이 작을수록 커지므로
정지폭(0.395)을 그대로 쓰면 R 을 최대 0.08 m 낙관하게 된다.

`d_sight` 1.6 m, `v` 1.4 m/s, `t_lat` 0.10 s 기준 (구현과 손계산이 소수점 4자리 일치):

| θ | R | W | δ | 필요 진입/탈출 직선 |
|---|---|---|---|---|
| 45° | 3.525 m | 0.388 | 2.92° | 1.46 m |
| 60° | 2.529 m | 0.385 | 4.07° | 1.46 m |
| **90°** | **1.284 m** | 0.376 | **7.98°** | 1.28 m |
| 99° | 1.063 m | 0.373 | 9.61° | 1.25 m |
| 120° | 0.727 m | 0.364 | 13.90° | 1.26 m |
| 135° | 0.579 m | 0.357 | 17.28° | 1.40 m |
| **142°** | **0.503 m** | 0.353 | **19.70°** | 1.46 m |

**θ 142° 에서 R 0.503 이 `R_min` 0.4945 를 겨우 넘는다** — §2 서두의 "통과 한계 θ≈142°" 와
독립적으로 일치한다.

고립된 직각 코너(θ≈99°, 전후 직선 3.8/9.0 m)는 `R 1.06 m`, `δ 9.6°`, **무감속 통과**다.
중앙선을 그대로 따르면 `R 0.28 m`로 통과 자체가 불가능하다. 레이스 라인은 선택이 아니다.

---

## 3. V_max 1.5 의 근거

```
d_req(V, v_c) = (V² − v_c²) / (2·a_decel) + V·t_lat  ≤  d_sight
```

밀집 코너의 계획용 보수값 `R = 0.296` → `v_c = sqrt(3.0·0.296) = 0.94 m/s` 기준
(§15의 오프셋 파라미터화 해에서 나온 값. 실제 달성 R은 이보다 클 수 있어 안전한 쪽이다):

| `d_sight` | `t_lat` 0.15 | **`t_lat` 0.25 (모션 보상 없음)** |
|---|---|---|
| 1.00 m | 1.30 m/s | 1.25 m/s |
| **1.50 m** | 1.47 m/s | **1.43 m/s** |
| 1.68 m | 1.50 (상한) | 1.50 (상한) |

`a_lat` 실측이 1.64 m/s² 아래면 V_max를 더 낮춘다.

---

## 4. 카메라 틸트

`camera_info_real.yaml`: 480×360, `fx=fy=261.559`, `cx=231.820`, `cy=169.603` (왜곡보정 완료 영상).

**"100° 화각"은 대각이다.** 대각 97.8° / 수평 85.1° / **수직 69.0°**. 틸트 계산에는 수직을 쓴다.

```
α = atan(h/d) + atan(cy/fy),      h = 0.1465 m
명령 틸트 = α − 0.284°             (lane_detection.yaml 실측편차)
```

| 목표 `d_sight` | 명령 틸트 (실차) | (시뮬 cy=180) | 지면 밴드 | 지평선 |
|---|---|---|---|---|
| 1.0 m | 41.01° | 42.59° | 3.3 ~ 100 cm | 화면 밖 |
| **1.5 m** | **38.25°** | **39.83°** | 4.0 ~ 150 cm | 화면 밖 |
| 현재 5.0° | 5.00° | 5.00° | 16.7 cm ~ ∞ | y=145 |

**1.5 m 를 채택한다** — 1.0 m는 V_max를 1.30까지 깎는다 (§3).

부수 효과 둘:

- **지평선이 화면 밖으로 나간다.** 전 화면이 지면이 되어 밝은 배경발 흰선 오검출이 사라진다. 대신 원거리 참조가 없어진다.
- **근거리 끝이 4 cm로 붙는다.** 수직 FOV 69°가 고정이라 원거리를 150 cm로 자르면 근거리는 자동 결정된다 — 독립적으로 못 고른다. 근거리 20 cm를 유지하며 150 cm를 보려면 수직 FOV 30.6°가 필요하다.

camera pan은 **비활성** (초안 §58 유지).

---

## 5. 노드·토픽

| 노드 | 패키지 | 주기 | 입력 | 출력 |
|---|---|---|---|---|
| `backup_lane_detector` | `backup_lane_detection` | **이미지 콜백** (~12 Hz) | `/camera/image_raw/compressed` | `/backup/lane/geometry` |
| `backup_obstacle_detector` | `backup_object_detection` | **scan 콜백** (~9.8 Hz) | `/scan_filtered` | `/backup/perception/obstacles` |
| `backup_start_signal_detector` | `backup_object_detection` | 10 Hz | `/camera/image_raw/compressed` | `/perception/start_permission` |
| `backup_path_planner` | `backup_path_planner` | **lane 콜백** (~12 Hz) | `/backup/lane/geometry`, `/backup/perception/obstacles` | `/backup/path` |
| `backup_speed_controller` | `backup_speed_controller` | **50 Hz** | `/backup/path`, `/perception/start_permission` | `/speed` |
| `backup_steer_controller` | `backup_steer_controller` | **50 Hz** | `/backup/path`, `/speed` | `/steering` |
| `backup_gui` | `backup_gui` | **3 Hz** | 전 토픽 | 화면 |

### 주기 결정 원칙

| 유형 | 규칙 |
|---|---|
| 센서를 소비하는 노드 | **콜백.** 센서 주기가 곧 정보율이다 |
| 정보를 만들지 않는 노드 | **상류 정보율에 맞춘다.** 더 빨리 돌려도 같은 입력에서 같은 출력이 나온다 |
| 차량 인터페이스로 나가는 노드 | **플랫폼 계약 ≥ 50 Hz** (`07_인터페이스.md` §159-160, 기존 `control_hz: 50.0`) |

### Path Planner 를 타이머로 두지 않는 이유

| # | 근거 |
|---|---|
| 1 | 정보율이 12 Hz다. 더 빨리 돌려도 같은 차선 기하에서 같은 경로가 나온다. 모션 보상은 조향이 50 Hz로 한다 |
| 2 | **10 Hz 타이머는 12 Hz 입력과 비팅한다.** 맥놀이 주기 `1/(12−10) = 0.5 s` — 0.5초마다 프레임을 하나 버리고, 데이터 나이가 0~83 ms를 **2 Hz로 왕복**한다. §7-A가 잡으려는 바로 그 주기적 교란이다 |
| 3 | **안전.** 콜백이면 차선 검출이 멈출 때 경로도 멈춰 `speed_controller` 의 `timeout → /speed = 0` 게이트가 자동으로 작동한다. **타이머는 낡은 경로를 계속 재발행해 고장을 감춘다** |

비용은 제약이 아니다 — 플래너는 <0.1 ms다 (§12-A).

`start_signal_detector` 는 계약대로 10 Hz를 유지하되 **`permission` 이 `true` 로 latch되면 구독을 해제**한다. 레이스 중에는 아예 돌지 않는다.

### 입력 토픽 — compressed vs raw

compressed가 무조건 가볍지는 않다. 전송량은 확실히 준다(JPEG ~30 KB vs raw 518 KB)지만 `cv::imdecode` 비용이 붙는다(~1–3 ms). 같은 호스트 intra-process면 raw가 CPU상 유리할 수도 있다.

**compressed를 택한다** — 드라이버가 어차피 compressed를 내고 `start_signal_detector`가 이미 그걸 쓴다. 실측(§16 #5)에서 뒤집히면 토픽만 바꾸면 되도록 **`input_topic` 을 yaml 파라미터로 노출**한다.

### 카메라 제어

| 대상 | 결정 |
|---|---|
| `/camera/pan` | **완전 비활성.** 구독도 발행도 하지 않는다 |
| `/camera/tilt` | **기동 시 1회만 발행**하고 끝. 값은 `camera_tilt_deg` (기본 38.25, §4) |

주행 중 카메라를 움직이면 호모그래피 `H`가 매 프레임 달라진다. 고정해야 `H`를 기동 시 1회 계산하고 행별 top-hat 커널 폭도 사전계산할 수 있다.

QoS — 센서 입력 `SensorDataQoS`, 내부 토픽 `RELIABLE · VOLATILE · depth 1`.
출력은 기존 `kau_control`과 동일하게 `/speed`, `/steering` 둘 다 `std_msgs/Float64`.

---

## 6. backup_msgs

### 경로 표현 — quintic Bezier

원호 스플라인을 쓰지 않는다. 이음매마다 곡률이 계단으로 뛰어 조향각 계단이 생기고, 회피 S자에서 특히 나쁘다.

| | 원호 스플라인 | **quintic Bezier** |
|---|---|---|
| 곡률 연속성 | G1 (κ 계단) | **G2** (양 끝 κ=0 강제 가능) |
| 회피 S자 | 나쁨 | 제어점 횡이동으로 자연 표현 |
| 레이스 라인 | 좋음 | 진입/정점/탈출 + 접선 구속 = 제어점 6개에 대응 |
| 구현 | 신규 | **`bezier.hpp` 이식** — 호길이(Gauss-Legendre 10점)·곡률·최근접점 검증됨 |

레이스 라인과 회피가 **같은 표현·같은 코드**로 처리된다. `KauPath` 필드 배치를 따르되 단위만 cm→m.

`BackupPath.msg`
```
std_msgs/Header header      # frame_id: base_link. stamp: 원본 이미지 시각

uint8 SRC_LANE_CENTER=0
uint8 SRC_RACE_LINE=1
uint8 SRC_AVOID=2
uint8 SRC_DEGRADED=3        # 연속 코너 퇴화 모드
uint8 source

uint8     degree            # 5 고정
float64   total_length      # [m]
float64[] ctrl_x            # [m] 길이 = nseg*(degree+1)
float64[] ctrl_y            # [m]
float64[] seg_length        # [m]
float32[] seg_kappa_max     # [1/m]
float32   confidence
float64   valid_length      # [m] 실제로 본 구간. 그 너머는 외삽

float64 corner_entry_s      # [m] 코너 진입까지 호길이. 음수면 코너 없음
float32 corner_kappa_max    # [1/m]
bool    kappa_saturated     # |kappa| > 1/R_min 요구 = 통과 불가
bool    avoiding
bool    stop_request
uint8   lost_state
uint8   lost_first
```

`LaneLine.msg`
```
# base_link 기준 지면 직선. 단위 m
float64 x0
float64 y0
float64 x1
float64 y1
float32 confidence
```

`LaneGeometry.msg`
```
std_msgs/Header header      # frame_id: base_link

LaneLine left
LaneLine right
LaneLine center
bool left_valid
bool right_valid
bool center_valid

bool    inflection_valid
float64 inflection_x        # [m] 종방향
float64 inflection_y        # [m] 좌측 +
float64 delta_psi           # [rad] 교각. 좌회전 +
float64 arc_length          # [m] 전환 구간 호길이
bool    second_inflection   # true = 연속 코너. 플래너 퇴화 모드 트리거

uint8 LOST_NONE=0
uint8 LOST_LEFT=1
uint8 LOST_RIGHT=2
uint8 LOST_BOTH=3
uint8 lost_state
uint8 lost_first

float64 valid_length        # [m]
```

`ObstacleCircle.msg` / `ObstacleCircleArray.msg` — `kau_msgs` 정의 복사 (이미 m 단위).
`ControlDebug.msg` — 튜닝 전용.

---

## 7. backup_lane_detection

**경로를 만들지 않는다.** 차선 기하만 낸다. 입력을 `/camera/image_raw` → **`/camera/image_raw/compressed`** 로 바꾼다.

### 조명 불변 흰선 추출 — 절대 HLS 임계를 버린다

현재 `white_hls_lo: [0, 205, 0]` 은 **절대 밝기 임계**다. 전역 조명이 변하면 흰선이 임계 위아래를 넘나든다. 근본적으로 조명 불변이 아니다.

```
BGR → HLS, L 채널만 사용 (백업은 흰선만 보므로 H 불필요)
  → ROI 하단
  → Top-hat:  T = L − open(L, kernel)          ← 핵심
       kernel 가로폭 = 그 행의 흰선 폭 × 3     (호모그래피로 행별 사전계산)
  → 고정 임계 (T > t0)                          ← top-hat 후엔 조명 불변
  → S < S_max 로 채색 배제 (노란선·잔디·주황 콘)
  → OPEN → CLOSE
```

**Top-hat이 맞는 도구인 이유**: 흰선은 폭이 알려진(7.5 cm) 밝은 리지다. Top-hat은 커널보다 넓은 배경 밝기(조명, 그림자, 노면 밝기 차)를 전부 빼고 좁은 밝은 구조만 남긴다. 가산성 조명 변화에 완전 불변이라, 뒤따르는 임계를 고정값으로 둘 수 있다.

곱셈성 변화까지 잡으려면 `T / (blur(L) + eps)` 로 정규화하거나 L에 CLAHE를 먼저 건다. 비용을 보고 결정한다.

행별 커널 폭이 필요한 이유 — 전체 BEV warp를 안 하므로 임계는 영상 공간에서 걸린다. 흰선 픽셀 폭이 행마다 다르다. 카메라가 고정이므로 **행별 커널 폭을 기동 시 1회 사전계산**한다.

우선순위 — **카메라 노출·화이트밸런스 고정이 가장 싸고 효과가 크다.** AE가 프레임마다 밝기를 바꾸면 어떤 임계도 흔들린다 (§16 #6).

### 파이프라인

```
CompressedImage
  → imdecode
  → ROI 절단 (행 0~133)                     ← 전체의 37%
  → max/min(B,G,R) 로 L·채도 동시 산출       ← BGR2HLS 호출 안 함
  → 스트립별 top-hat + 고정 임계 + 채도 배제
  → OPEN/CLOSE
  → 행별 런 검출 + 서브픽셀 센트로이드        ← 슬라이딩 윈도우 대체
  → 점에만 호모그래피 H 적용                 ← BEV 전체 warp 없음
  → base_link 지면 좌표 [m]
  → 좌/우 분류 → IRLS 직선 피팅 → 변곡점 분할
  → 예측-보정 필터 (§7-A 4위) → 발행
```

**undistort 를 하지 않는다.** `camera_info_real.yaml` 의 `D` 가 전부 0이고, 드라이버(`physicar_bringup` 의 `undistort.*`)가 640×480을 이미 `cv::remap` 으로 편 뒤 480×360을 낸다. 한 번 더 remap하면 **이중 보정으로 기하가 틀어지고 2~3 ms를 버린다.** yaml에 `undistort: false` 로 노출만 해둔다.

`H`는 지면 z=0 가정에서 `K`, `camera_height_cm 14.65`, 틸트(§4)로 기동 시 1회 유도한다.

> 초안 §29의 "BEV 지연이 크면 생략"을 대체한다. 이미지 평면 각도는 지면 각도가 아니라서 BEV를 통째로 없애면 `Δψ`가 성립하지 않는다.

### ROI 와 스트립

틸트 38.25°에서 전체 360행 중 **행 0~133 만 d = 1.5~0.25 m** 를 담는다. 나머지 227행(63%)은 d < 0.25 m — 이미 지나간 지면이라 버린다.

흰선 픽셀 폭이 행에 따라 13 → 68 px 로 5배 변하므로 커널 하나로는 안 된다. 스트립을 나눈다:

| 행 | d [m] | 흰선 폭 | top-hat 커널 |
|---|---|---|---|
| 0~20 | 0.95~1.50 | 13~20 px | 51 |
| 20~45 | 0.63~0.95 | 20~30 px | 77 |
| 45~80 | 0.41~0.63 | 30~45 px | 113 |
| 80~133 | 0.25~0.41 | 45~68 px | 171 |

van Herk/Gil-Werman 침식·팽창은 **커널 폭과 무관하게 O(1)/px** 이므로 스트립을 나눠도 총 비용은 ROI 1패스다. 카메라가 고정이라 경계와 커널 폭은 **기동 시 1회 사전계산**한다.

참고 — 수평 반각 41.6°라 **양쪽 흰선이 동시에 보이는 최소 거리는 d = 0.37 m** 다. 그보다 가까운 행은 한쪽만 보인다 (횡오프셋 추정에는 여전히 유용하다).

### 산출

| 항목 | 방법 |
|---|---|
| 좌/우 흰선 직선 | 최근접 유효점 → 신뢰도 조건 만족 최원점. **이전 프레임 해를 초기값으로 한 IRLS(Huber)** — RANSAC 아님 (§7-A 3위) |
| 중앙 직선 | 양쪽 유효 → 각이등분선. 한쪽만 → 유효한 쪽에서 트랙 반폭 오프셋 |
| 변곡점 | 피팅 잔차 임계 초과 + 교각 ≥ 20° → `[직선A][전환구간][직선B]` |
| `arc_length` | 전환구간 검출점들의 실제 호길이 |
| `second_inflection` | 시야 내 변곡점 2개 이상 → 연속 코너 신호 |

### 유효 판정 히스테리시스

| 전이 | 조건 |
|---|---|
| 무효 → 유효 | 2 프레임 연속 검출 |
| 유효 → 무효 | 3 프레임 연속 미검출 |

변곡점 판정도 히스테리시스를 건다 — **진입 20°, 이탈 15°**. 단일 임계면 20° 근처에서 `inflection_valid` 가 프레임마다 뒤집히고, 그게 §7-A 2위 원인으로 직결된다.

---

## 7-A. Jitter 최소화 — 원인별 우선순위

측정 잡음 자체는 문제가 아니다. **불연속과 지연이 문제다.** 아래는 영향 큰 순.

### 1위 — 모션 보상 부재 (단일 최대 원인)

경로·차선 기하는 전부 `base_link` **상대좌표**인데, 생산 주기와 소비 주기가 다르다.

| 구간 | 위상차 | v=1.43에서 전진 | Ld=1.2 기준 조향 오차 |
|---|---|---|---|
| 경로 ~12 Hz → 조향 50 Hz | 최악 83 ms | 11.9 cm | **1.70°** |
| ~~차선 12 Hz → 플래너 10 Hz 비팅~~ | **해소** | — | — (플래너 콜백 구동, §5) |

보상하지 않으면 **12 Hz 톱니파가 조향에 그대로 실린다.** 측정 잡음(0.5 cm → 0.07°)의 24배다.
플래너를 콜백으로 둔 덕에 비팅 성분은 아예 생기지 않고, 남은 것은 이 한 구간뿐이다.

**해결 — 상대좌표 메시지를 쓰는 모든 소비자가 stamp 이후 자차 이동을 적분해 현재 `base_link` 로 옮긴다.**

```
ω  = v·tan(δ)/L,   Δψ = ω·Δt,   R = L/tan(δ)
t  = ( R·sin(Δψ),  R·(1−cos(Δψ)) )
p_new = Rot(−Δψ) · (p_old − t)
```

`v`는 `/speed`, `δ`는 자기가 방금 낸 명령이다. 제어점 6~18개 강체변환 = 연산량 무시 가능.

부수 효과가 크다 — 유효 지연이 **0.25 → 0.10 s**로 줄어 위상여유가 14° → 31°(Ld 1.0)로 회복되고 V_max도 1.43 → 1.47로 돌아온다.

### 2위 — 모드 전환 불연속

`source` 가 `LANE_CENTER ↔ RACE_LINE ↔ AVOID` 로 뒤집히면 경로가 횡으로 **최대 ±20 cm** 점프한다.

**구조적 해결 — `P0`, `P1` 을 항상 자차에 고정한다.**

```
P0 = (0, 0)                       자차 원점
P1 = P0 + ε·(1, 0)                접선 = 현재 진행방향
```

경로가 자차에서 시작하고 접선이 진행방향이면 **모드가 바뀌어도 자차 근처 경로는 안 움직인다.** 변화는 전방으로 밀려나고 Pure Pursuit의 `Ld` 평활이 흡수한다. 레이스 라인의 "진입점"은 `P2`~`P3` 로 표현한다 — 자차에서 진입점까지 가는 것 자체가 경로의 일부다.

추가로 `source` 전이에 **최소 유지시간 0.3 s**.

### 3위 — RANSAC 비결정성

같은 입력에 다른 해가 나와 프레임 간 무상관 잡음이 된다.

**해결 — 이전 프레임 해를 초기값으로 하는 IRLS(Huber).** 결정적이고, 시간적으로 연속이고, RANSAC보다 싸다.

### 4위 — 필터 지연

EMA는 잡음을 줄이는 만큼 지연을 만든다 (12 Hz, T=83 ms):

| α | 잡음 σ 배율 | 그룹지연 |
|---|---|---|
| 0.3 | 0.420 | **194 ms** |
| 0.5 | 0.577 | 83 ms |
| 0.7 | 0.734 | 36 ms |

α 0.3이면 `t_lat`이 배가 된다 — 잡음을 잡고 위상여유를 잃는 맞바꿈이다.

**해결 — 1위의 모션 보상이 들어가면 EMA 대신 예측-보정 필터를 쓸 수 있다.** 예측이 차량 운동을 처리하므로 **지연 없이 긴 시상수**를 쓸 수 있다. `(y, ψ)` 2상태 칼만, 비용 무시 가능.

### 5위 — Ld 클램프

`Ld ≤ valid_length` 에서 `Ld_max = d_sight` 로 두면 클램프가 매 프레임 물려 루프 게인 `2L/Ld²` 가 2~4배 요동한다. **`Ld_max = 1.2` 로 0.3 m 여유를 둔다** (§11).

### 6위 — 서브픽셀

슬라이딩 윈도우 argmax는 1 px 양자화가 그대로 잡음이 된다.

**해결 — 행별 런 검출 + 센트로이드.** 정밀도가 `1/√(12N)` px로 올라가고, 윈도우 탐색이 없어져 더 싸다 (§7-B).

### 7위 — 원거리 측정점을 lookahead로 쓰는 것

| d | 종방향 해상도 | 흰선 폭 |
|---|---|---|
| 1.5 m | **4.17 cm/px** | 13 px |
| 1.2 m | 2.85 cm/px | 16 px |
| 1.0 m | 1.99 cm/px | 19 px |

`Ld_max` 1.2가 1.5보다 유리한 또 하나의 이유다.

---

## 8. backup_object_detection

- **장애물**: `kau_object_detection`의 Adaptive Adjacent Point Clustering 이식. `/scan_filtered` (720점, ~9.8 Hz) → 클러스터 → **반경 0.142 m 원으로 통일 근사** → `ObstacleCircleArray`. `lidar_link → base_link`는 정적 상수 (TF 조회 없음).
- **출발 신호**: `start_signal_detector_node` 이식. 이미 `/camera/image_raw/compressed` → `/perception/start_permission` 이라 그대로 쓴다.

---

## 9. backup_path_planner

경로 생성 단독 소유. 매 lane 콜백마다 `BackupPath` 하나를 새로 만든다.

| 상황 | `source` | 경로 |
|---|---|---|
| 직선 | `LANE_CENTER` | 중앙 직선을 그대로 quintic으로 |
| 단일 코너 | `RACE_LINE` | out-in-out |
| 연속 코너 | `DEGRADED` | 중앙선 + 실현 가능한 최대 곡률로 클램프 |
| 장애물 | `AVOID` | 위 경로에 횡 offset 중첩 |

### 레이스 라인 생성

```
0. 차선 기하를 stamp → now 로 데드레커닝                ← 필수 (§7-A 1위)
1. Δψ, arc_length 로 코너 판정.  second_inflection 이면 DEGRADED
2. R = min( W/(1−cos(Δψ/2)),  (d_sight − v·t_lat)/tan(Δψ/2) )
   W = 0.700 − 선회소요폭(R) − 2×0.05                  ← R 의존이므로 1회 반복
3. R < R_min  →  kappa_saturated = true
4. 제어점 6개:
     P0 = (0, 0)               ★ 항상 자차 원점
     P1 = P0 + ε·(1, 0)        ★ 접선 = 현재 진행방향
     P2, P3                    진입점 → 정점 (레이스 라인 형상)
     P4, P5                    탈출 접선, κ=0
5. seg_kappa_max, seg_length 계산 (bezier.hpp)
```

**`P0`·`P1` 을 자차에 고정하는 것이 지터 방어의 핵심이다** (§7-A 2위). 경로가 항상 자차에서 현재 진행방향으로 출발하므로, `source` 가 바뀌어도 자차 근처 경로는 움직이지 않는다. 레이스 라인의 "진입점"은 목표지 시작점이 아니며 `P2`~`P3` 로 표현한다 — 자차에서 진입점까지 가는 것 자체가 경로의 일부다.

**끝 곡률을 0으로 강제**해 다음 구간과 G2로 이어진다. 조향각이 계단으로 뛰지 않는다.

### 회피 — 중앙선 횡방향 offset

실측 기준 (장애물 반경 0.142, 중앙선에서 ±0.18, 회피 여유 0.10, 차량 폭 0.205):

| | 값 |
|---|---|
| 장애물 점유 | `y ∈ [+3.8, +32.2] cm` |
| 차량 중심 가능 범위 | **`y ∈ [−24.8, −16.4] cm`** (창 8.3 cm) |
| **채택 offset** | **`y = −16.4 cm`** — 장애물 여유 정확히 10 cm, 차선 여유 8.3 cm |

차선 이탈이 실격이므로 차선 여유를 최대로 잡는다. 부호는 장애물 반대편.

```
1. 전방 d_trigger 이내 & 경로 횡거리 |dy| < w_clear 인 장애물 선택
2. offset 부호 = 장애물 반대편
3. |offset| = |y_obs| − r_obs − 차량폭/2 − 회피여유
4. 기준 경로의 중간 제어점(P2, P3)만 offset 만큼 횡이동
   → 양 끝 접선·곡률이 보존되어 S자가 자동으로 G2
5. 차선 여유(0.05) 침범 시 클램프. 그래도 부족하면 stop_request
```

**콘 6개가 전부 직선 구간에 있다** — 회피와 코너 로직이 겹치지 않는다.

### 경로 안정화

~12 Hz로 매번 재생성하면 경로가 튄다. 3단으로 막는다.

| 장치 | 대상 |
|---|---|
| `P0`·`P1` 자차 고정 | 구조적으로 자차 근처 경로가 안 움직인다 (위 레이스 라인 생성 4번) |
| 생성 파라미터에 필터 | 제어점이 아니라 **진입점 횡오프셋 · 정점 오프셋 · `R`** 에 건다 |
| `source` 전이 | 최소 유지시간 0.3 s + offset 레이트 리밋 |

`R = min(R_width, R_sight)` 에서 `R_sight = (d_sight − v·t_lat)/tan(Δψ/2)` 는 **`Δψ` 가 작을 때 발산**한다. 변곡점 판정 하한 20°(이탈 15°)가 그 아래를 직선으로 걷어내고, `R` 자체에도 상한을 건다.

---

## 10. backup_speed_controller

50 Hz (플랫폼 계약 ≥50 Hz). 기하 판단 없음 — 경로의 곡률만 본다.

```
전방 s ∈ [0, min(valid_length, d_preview)] 의 max|κ|
v_curve  = sqrt(a_lat_max / max|κ|)
v_target = clamp( min(V_max, v_curve),  v_min,  V_max )

d_brake = (v² − v_corner²) / (2·a_decel) + v·t_lat
corner_entry_s ≤ d_brake  →  감속 개시
```

레이트 리밋 (50 Hz, tick 20 ms) — 가속 **+0.030 m/s** (1.5 m/s²), 감속 **−0.010 m/s** (0.5 m/s²).

### 게이트 (우선순위 순)

| 조건 | `/speed` |
|---|---|
| `start_permission == false` | 0 |
| `stop_request` | 0 으로 감속 |
| `kappa_saturated` | `v_min` (조향 포화 — 속도로 더 할 게 없다) |
| `lost_state == LOST_BOTH` | `v_min` 으로 감속, `T_lost` 초과 시 0 |
| `BackupPath` 수신 끊김 > `timeout` | 0 |
| 정상 | 위 프로파일 |

### 종료 안전

SIGINT 핸들러에서 `0.0`을 **20 ms 간격 3회 발행 → 200 ms 대기 → 종료**. 1회 발행 직후 종료하면 DDS가 전송을 끝내지 못한다.

> SIGKILL에는 무력하다. 차량 측 `/speed` 타임아웃 유무 확인 필요 (§16 #7).

---

## 11. backup_steer_controller

50 Hz. **Pure Pursuit 단일 모드.** 모드 전환도 δ PID도 없다 — 기하는 전부 플래너가 처리했다.

```
경로를 stamp → now 로 데드레커닝               ← 필수 (§7-A 1위)
Ld     = clamp(k_v · v, Ld_min, Ld_max),  Ld ≤ valid_length
target = 경로 위 호길이 Ld 지점                 ← 호길이 LUT 보간 (경로 수신 시 1회 구축)
δ      = atan( 2·L·sin(α) / Ld )
δ_cmd  = clamp(δ, ±20°)
```

`kau_control/pure_pursuit.hpp`의 순수 함수를 그대로 쓴다.

### Ld 확정 — `k_v 0.85 / Ld_min 0.8 / Ld_max 1.2`

선형화하면 이상 Pure Pursuit은 `s² + (2v/Ld)s + 2v²/Ld² = 0` 이라 **ζ = 1/√2 로 Ld와 무관하게 고정**이고 `ωn = √2·v/Ld` 다. Ld는 감쇠가 아니라 대역폭을 정한다.

액추에이터 1차 지연 `τ` 를 넣으면 3차가 된다.

```
s³ + (1/τ)s² + (2v/(Ld·τ))s + 2v²/(Ld²·τ) = 0
```

Routh-Hurwitz → **안정조건 `Ld > v·τ`**. `τ` 0.2 s, v 1.43 → `Ld > 0.286 m`. 안정성으로는 갈리지 않는다. **갈리는 것은 위상여유다** (센싱 지연 포함):

| v | Ld | ζ | PM (`t_lat` 0.25) | PM (`t_lat` 0.10) | 지터 게인 `2L/Ld²` |
|---|---|---|---|---|---|
| 1.43 | 0.8 | 0.34 | 4° ✗ | 24° ✗ | 0.322 °/cm |
| 1.43 | 1.0 | 0.434 | 14° ✗ | 31° △ | 0.206 °/cm |
| 1.43 | **1.2** | **0.524** | 22° ✗ | **37°** ✓ | **0.143 °/cm** |
| 1.43 | 1.5 | 0.623 | 31° △ | 42° ✓ | 0.092 °/cm |

**`Ld_max` 를 1.5(= `d_sight`)로 두지 않는 이유** — `Ld ≤ valid_length` 클램프가 매 프레임 물려 Ld가 변동하고 루프 게인이 2~4배 요동한다. 게다가 d=1.5는 종방향 해상도 4.17 cm/px 로 측정이 가장 나쁜 지점이다. **1.2가 여유 0.3 m 와 위상여유 37°를 동시에 준다.**

**모션 보상이 전제다.** `t_lat` 0.25 s로는 어떤 Ld도 위상여유가 안 나온다.

조향 각속도 600 deg/s는 여전히 제약이 아니다 — 코너 진입 요구 각속도가 ~35 deg/s, fallback 전타도 33 ms다. 채터 방지용 슬루 제한만 **300 deg/s (50 Hz에서 6°/tick)** 로 둔다.

| 예외 | 동작 |
|---|---|
| `δ` 포화 | `δ_saturated` 디버그 발행 + 속도 `v_min` 요청 |
| `lost_state == LOST_BOTH` | §12 |
| 경로 수신 끊김 | `δ` 유지, 속도 게이트가 0을 만든다 |

---

## 12. 차선 손실 fallback

```
BOTH_OK ──(한쪽 3프레임 미검출)──▶ ONE_LOST ──(나머지도 미검출)──▶ BOTH_LOST
   ▲                                   │                              │
   └───────────(2프레임 연속 복귀)──────┴──────────────────────────────┘
```

`ONE_LOST`는 정상 주행이다 — 남은 한쪽에서 트랙 반폭 오프셋으로 중앙선을 복원한다.

`BOTH_LOST` 진입 시:

| 항목 | 동작 |
|---|---|
| 조향 | `sign(lost_first) · δ_max` — **먼저 끊긴 쪽으로 최대 조향** |
| 속도 | 동시에 `v_min`(0.5)까지 감속 |
| 지속 상한 | `T_lost = 0.6 s` 초과 시 `/speed = 0`, 조향 유지 |
| 복귀 | 한쪽이라도 유효해지면 즉시 정상 모드. Pure Pursuit이 자연히 수렴 |
| 양쪽 동시 손실 | `lost_first` 미정 → 마지막 유효 `sign(Δψ)`, 그마저 없으면 `δ = 0` |

**안전성**: `R = 0.4945`, `v_min` 0.5 m/s → 요레이트 58 deg/s. `T_lost` 0.6 s 동안 주행 0.30 m, **횡변위 8.9 cm**. 트랙 반폭 35 cm 대비 안전하다.

---

## 12-A. 연산량 예산

### backup_lane_detector — 프레임당 예산 83 ms

| 단계 | 비용 | 근거 |
|---|---|---|
| `imdecode` 480×360 JPEG | 1~3 ms | |
| ~~undistort~~ | **0** | 드라이버가 이미 보정. 하면 이중 보정 + 2~3 ms 낭비 |
| ROI 절단 | 0 | 이후 전 단계가 63,840 px (전체의 37%) |
| L·채도 산출 | ~0.3 ms | `max/min(B,G,R)`. `BGR2HLS` 호출 안 함 |
| top-hat 4 스트립 | <1 ms | van Herk O(1)/px → 총 ROI 1패스 |
| 임계 + 채도 배제 + OPEN/CLOSE | ~1 ms | |
| 행별 런 + 센트로이드 | ~0.5 ms | 윈도우 탐색 없음 |
| 호모그래피 (점 수백 개) | <0.1 ms | |
| IRLS 직선 피팅 + 변곡점 | <0.5 ms | |
| **합계** | **5~8 ms** | **예산의 10% 이하** |

### 나머지 노드

| 노드 | 주기 | 비용 | 비고 |
|---|---|---|---|
| `backup_obstacle_detector` | 9.8 Hz | <1 ms | 720점 클러스터링 |
| `backup_start_signal_detector` | 10 Hz | 1~3 ms | **permission이 true로 latch되면 구독 해제** — 레이스 내내 도는 이미지 처리 하나가 사라진다 |
| `backup_path_planner` | lane 콜백 ~12 Hz | <0.1 ms | 제어점 6~18개 + Gauss-Legendre 10점 × 3세그먼트 |
| `backup_speed_controller` | 50 Hz | <0.05 ms | 곡률 샘플링만 |
| `backup_steer_controller` | 50 Hz | <0.05 ms | 호길이 LUT는 경로 수신 시 1회 구축, 50 Hz는 보간만 |
| `backup_gui` | **3 Hz** | 노드 중 가장 무겁다 | Python + Qt. 화면 갱신만 하므로 3 Hz면 충분하다. `run.sh` 기본 `false` |

전 노드 합쳐 1코어의 일부다. 연산량은 이 설계의 제약이 아니다.

### 비용을 만들지 않는 규칙

| 규칙 | 이유 |
|---|---|
| 카메라 고정 → `H`, ROI 경계, 스트립 커널 폭, 행별 선폭을 **기동 시 1회** 계산 | 매 프레임 재계산하면 전부 낭비 |
| 경로 호길이 LUT는 **경로 수신 시** 구축 (~12 Hz), 조향은 보간만 (50 Hz) | Newton 반복을 50 Hz로 돌리지 않는다 |
| 이미지 전체 warp 금지, 점 단위 `H` 만 | `warpPerspective` 14.8 ms → ~0 |
| RANSAC 대신 이전 해 시드 IRLS | 더 싸고 결정적 (§7-A 3위) |

---

## 13. 패키지 구조

```
src/
├── backup_msgs/                              ament_cmake (rosidl)
│   ├── CMakeLists.txt · package.xml
│   └── msg/
│       ├── LaneLine.msg
│       ├── LaneGeometry.msg
│       ├── BackupPath.msg
│       ├── ObstacleCircle.msg
│       ├── ObstacleCircleArray.msg
│       └── ControlDebug.msg
│
├── backup_lane_detection/                    ament_cmake (C++)
│   ├── include/backup_lane_detection/
│   │   ├── lane_detector_node.hpp
│   │   ├── white_mask.hpp                    top-hat 흰선 추출 (§7)
│   │   ├── homography.hpp                    점 단위 지면 투영
│   │   └── line_fit.hpp                      직선 피팅 · 변곡점 분할
│   ├── src/lane_detector_node.cpp · main.cpp
│   ├── config/
│   │   ├── lane_detector.yaml                ★ 전 파라미터
│   │   ├── camera_info.yaml                  intrinsic (kau_lane_detection 복사)
│   │   └── platform/{sim,real}.yaml          cy 180.0 vs 169.603 등
│   └── launch/lane_detection.launch.py
│
├── backup_object_detection/                  ament_cmake (C++)
│   ├── include/backup_object_detection/
│   │   ├── obstacle_detector_node.hpp
│   │   ├── clustering.hpp                    kau_object_detection 이식
│   │   └── start_signal_node.hpp             kau_object_detection 이식
│   ├── src/obstacle_detector_node.cpp · start_signal_node.cpp · main_*.cpp
│   ├── config/obstacle_detector.yaml · start_signal.yaml
│   └── launch/object_detection.launch.py     두 노드 동시
│
├── backup_path_planner/                      ament_cmake (C++)
│   ├── include/backup_path_planner/
│   │   ├── path_planner_node.hpp
│   │   ├── bezier.hpp                        kau_lane_detection 복사, 단위 cm→m
│   │   ├── race_line.hpp                     out-in-out 생성 (§9)
│   │   └── avoidance.hpp                     횡 offset (§9)
│   ├── src/path_planner_node.cpp · main.cpp
│   ├── config/path_planner.yaml
│   └── launch/path_planner.launch.py
│
├── backup_speed_controller/                  ament_cmake (C++)
│   ├── include/backup_speed_controller/speed_controller_node.hpp · profile.hpp
│   ├── src/speed_controller_node.cpp · main.cpp
│   ├── config/speed_controller.yaml
│   └── launch/speed_controller.launch.py
│
├── backup_steer_controller/                  ament_cmake (C++)
│   ├── include/backup_steer_controller/steer_controller_node.hpp
│   │                                         pure_pursuit.hpp  (kau_control 복사)
│   ├── src/steer_controller_node.cpp · main.cpp
│   ├── config/steer_controller.yaml
│   └── launch/steer_controller.launch.py
│
├── backup_gui/                               ament_python (kau_gui 복제)
│   ├── setup.py · setup.cfg · package.xml
│   ├── backup_gui/app.py · bridge.py · viz.py · status_bar.py
│   │              manifest.py · _venv.py · __init__.py
│   ├── config/gui.yaml
│   ├── resource/backup_gui
│   ├── scripts/setup_venv.sh
│   └── launch/gui.launch.py
│
└── backup_bringup/                           ament_cmake (코드 없음)
    ├── CMakeLists.txt · package.xml
    ├── launch/backup.launch.py               전체 스택 + skip 인자
    └── config/bringup.yaml                   공통 파라미터 (차량 제원 등)

run.sh                                        워크스페이스 루트
```

### `backup_gui` 복제 시 고쳐야 할 곳

| 대상 | 내용 |
|---|---|
| 모듈 디렉터리 | `kau_gui/` → `backup_gui/` |
| `package.xml` | `<name>`, 의존을 `kau_msgs` → `backup_msgs` |
| `setup.py` | `package_name`, `entry_points`, `data_files` 경로 |
| `resource/` | 마커 파일명 |
| `bridge.py` | 구독 토픽을 `/backup/*` 로, 메시지 타입을 `backup_msgs` 로 |
| `viz.py` | `KauPath` → `BackupPath` (필드 배치 동일, 단위 cm→m) |

### launch 규약 — 옵션 없이 떠야 한다

```bash
ros2 launch backup_lane_detection lane_detection.launch.py     # 이것만으로 동작
ros2 launch backup_bringup       backup.launch.py              # 전체 스택
```

| 규칙 | 이유 |
|---|---|
| **모든 `DeclareLaunchArgument` 에 기본값** | 인자 없이 실행돼야 한다 |
| launch가 자기 패키지 share의 `config/*.yaml` 을 기본으로 물고 있음 | 파라미터 파일을 손으로 넘길 일이 없다 |
| yaml 최상단은 `/**:` 와일드카드 | 노드 이름이 바뀌어도 파라미터가 안 끊긴다 |
| 패키지별 launch는 **그 패키지 노드만** 띄운다 | 단독 테스트용 |
| `backup_bringup/backup.launch.py` 는 각 패키지 launch를 `IncludeLaunchDescription` 으로 조립 | 파라미터 정의가 한 곳에만 있다 |

---

## 14. 기동 — run.sh

기존 `run.sh`는 `system_supervisor` 하나를 띄우고 나머지를 spawn시킨다. **백업 스택은 상태머신이 없으므로 launch 파일이 노드를 직접 나열한다.**

`backup_bringup/launch/backup.launch.py` — 6개 노드 + `skip` 인자.

```bash
source run.sh
```

| 규칙 | 이유 |
|---|---|
| `exec` / `set -e` / `exit` 금지 | `source`로 부르므로 Ctrl-C·오류에 터미널이 같이 닫힌다 |
| `${BASH_SOURCE[0]}` 로 WS 루트 판정 | `source`에서는 `$0`가 스크립트가 아니다 |
| **모든 경로는 WS 루트 기준 상대경로** | 다른 컴퓨터에서 실행한다. 절대경로 금지 |
| **`BACKUP_USE_SIM_TIME` true/false 전환** | Gazebo ↔ 실차. launch 인자로 전달 |
| **워크스페이스 자동 소싱** | `install/setup.bash` 가 있고 아직 안 잡혔으면 소싱 |
| `BACKUP_NODES` 표로 노드 on/off | `false` 는 `skip:=` 로 전달 |
| `true`/`false` 외 값은 기동 거부 | 오타를 "켠 것"으로 넘기면 끈 줄 안 노드가 조용히 뜬다 |
| 끝에 임시 변수 `unset` | 셸에 흔적을 남기지 않는다 |

```bash
BACKUP_NODES="
    backup_start_signal_detector  true
    backup_obstacle_detector      true
    backup_lane_detector          true
    backup_path_planner           true
    backup_speed_controller       true
    backup_steer_controller       true
    backup_gui                    false     # Qt. 주행 중엔 끈다
"
BACKUP_USE_SIM_TIME=true      # Gazebo true / 실차 false
BACKUP_AUTO_SOURCE=true
BACKUP_RUN_ID=""
```

**전환 방법** — 기존 `run.sh`를 `run_kau.sh`로 개명하고 백업본이 `run.sh` 자리를 차지한다. 배타적 실행이 파일 이름 하나로 보장된다.

### config 규약

노드별 파라미터는 **전부** 해당 패키지의 `config/<node>.yaml` 에 둔다.

| 규칙 |
|---|
| 코드에 튜닝 상수 리터럴 금지. `declare_parameter` 기본값도 yaml과 일치시킨다 |
| 런타임 변경 가능/불가를 주석 한 줄로 표시 (`kau_lane_detection`의 `min_pixels` 사고 재발 방지) |
| 시뮬/실차 차이는 `config/platform_overrides/` 로 분리 |
| **주석은 최소한으로.** 왜 그 값인지 한 줄. 줄글 금지 |

---

## 15. 조향 부족 — 손상 구간 허용 설계 (채택)

### 해석 결과는 결정적이지 않다

| 방법 | 결과 | 한계 |
|---|---|---|
| 오프셋 파라미터화 최소곡률 (IRLS) | R_min 0.296 m, **31.3° 필요** | 오프셋 곡선 곡률이 `κ_b/(1−n·κ_b)` 라 중앙선 곡률이 큰 곳에서 안 펴진다. 허용 폭을 15 cm 넓혀도 R_min이 안 움직인 게 증거 |
| 자전거 모델 도달성 (격자) | **통과 가능** (생존 74%, 이탈 0 cm) | 763스텝 셀 반올림이 없는 기동성을 만들 수 있다 |
| 격자 없는 연속 롤아웃 | **실패** | 탐욕적 정책(최소 \|δ\|)이라 실패가 곧 불가능은 아니다 |

**결론: 해석으로 판정 불가. 실차/시뮬 주행으로만 결정된다.**

### 채택 — 손상 구간 허용

어느 쪽이 맞든 안전한 상위집합으로 간다.

```
1. 플래너는 항상 실현 가능한 최소 최대곡률 경로를 만든다 (레이스 라인)
2. |κ| > 1/R_min 이 요구되면  kappa_saturated = true
3. 조향은 ±20° 포화 상태로 유지 (더 할 수 있는 게 없다)
4. 속도는 v_min 으로 내린다 — 이탈 깊이는 속도가 아니라 기하가 정하지만,
   저속이 이탈 지속시간과 복귀 거리를 줄인다
5. 이탈 중에도 차선 검출은 계속 돌린다. 한쪽이라도 복귀하면 즉시 정상 추종
```

**설계에 이미 들어 있는 것** — `BackupPath.kappa_saturated`, 속도 게이트의 `kappa_saturated → v_min`, `δ_saturated` 디버그. 추가 구현이 필요 없다.

**로그 필수** — `kappa_saturated` 가 선 구간의 s 위치·지속시간·최대 요구 δ 를 남긴다. 실주행 로그가 곧 §16 #2의 답이 된다.

---

## 16. 실측 필요

| # | 항목 | 방법 | 영향 |
|---|---|---|---|
| 1 | `a_lat_max` | 정상원 선회 반경 축소 | `v_corner`, V_max |
| 2 | **실제 통과 가능 여부** | 저속 1바퀴 주행 + `kappa_saturated` 로그 | §15 전체 |
| 3 | `d_sight` (틸트 38.25° 적용 후 최원 검출거리) | 정차 라이브 | V_max |
| 4 | `t_lat` (이미지 stamp → `/steering`) | end-to-end 타임스탬프 | V_max |
| 5 | `/camera/image_raw/compressed` 실제 Hz·해상도 | `ros2 topic hz` | `t_lat` |
| 6 | **카메라 노출·화이트밸런스 고정 가능 여부** | 드라이버 파라미터 확인 | §7 조명 강건성 |
| 7 | 차량 측 `/speed` 타임아웃 유무 | 인터페이스 문의 | 종료 안전 |
| 8 | 조향 액추에이터 응답 시정수 | 스텝 응답 | Pure Pursuit 게인 |

---

## 17. 미확정

- `kau_state_machine` / `/diag/heartbeat` 연동 여부 (현 설계는 완전 독립)
- 정지선·미션 처리 필요 여부
- `backup_gui` 가 표시할 항목 (기존 `kau_gui` 화면 중 무엇을 남길지)
