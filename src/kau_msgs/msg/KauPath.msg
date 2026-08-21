# 함수형 경로. quintic Bezier segment 열
# 이산 좌표 집합이 아니며, 수신 node 가 해석적으로 곡선을 복원
# 정의 근거: docs/경로_형식.md §9.2

std_msgs/Header header      # frame_id: map | base_link
                            # stamp: 경로 생성 시각

# --- 경로 종류 ---
uint8 SRC_GLOBAL = 0
uint8 SRC_LOCAL  = 1
uint8 SRC_LANE   = 2
uint8 source

# --- 기하 정의 ---
uint8   degree              # segment 공통 차수. 운용값 5
bool    is_closed           # 폐곡선 여부. true 이면 수신측이 index 를 mod n 순환
float64 s_offset            # [cm] 전역 호길이 기준점. Global Path 는 0
float64 total_length        # [cm] 전 segment 호길이 합

# --- 제어점 (flat) ---
# segment k 의 j 번째 제어점 = ctrl_x[k*(degree+1) + j]
# 인접 segment 는 단부 제어점을 공유하나 중복 저장 (G0 검증 및 indexing 단순화)
float64[] ctrl_x            # [cm] 길이 = nseg * (degree+1)
float64[] ctrl_y            # [cm]

# --- segment 별 사전계산값 ---
float64[] seg_length        # [cm] Gauss-Legendre 10 점. 길이 = nseg
float32[] seg_kappa_max     # [1/cm] |kappa| 상한. 강체변환 불변

# --- 품질 ---
float32 confidence          # 0.0 ~ 1.0
float64 valid_length        # [cm] 실관측 구간. Lane Detection 전용, 그 외 0
