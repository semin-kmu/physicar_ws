// ====================================================================
// types.hpp
//
// Local Path Planner 고유 타입. Curve/TrackState 는 kau_control 의 것을
// 그대로 쓰므로 여기서 재정의하지 않는다 (KAU_AMET_Test 세션 결정).
//
// 원본: KAU_AMET_Test / src/kau_local_path_planner/test/config.py 의
//       PlannerParams, src/kau_local_path_planner/test/planner.py 의
//       PlanResult / 후보 튜플 (offset, curve, cost, reason)
// ====================================================================

#ifndef KAU_LOCAL_PATH_PLANNER__TYPES_HPP_
#define KAU_LOCAL_PATH_PLANNER__TYPES_HPP_

#include <optional>
#include <string>

#include "kau_control/curve.hpp"

namespace kau
{
namespace local_path_planner
{

using kau::control::Curve;

// ====================================================================
// PlannerParams -- config/local_planner.yaml 의 ros__parameters 와 1:1.
//
// 값들은 KAU_AMET_Test 세션에서 실측/스윕으로 확정한 최종값이다. 임의로
// 바꾸지 말고 config/local_planner.yaml 의 주석을 먼저 볼 것.
// ====================================================================

struct PlannerParams
{
    double l_plan        = 300.0;   // cm, 계획 길이
    std::string ref_mode = "fuse";  // global | lane | fuse
    double w_lane         = 1.0;    // lane 신뢰 가중치 (실측: 결과에 영향 없음, 유지)

    // --- 제약 ---
    double kappa_margin = 0.95;     // kappa_max 대비 허용 상한
    int    bound_depth  = 2;        // kappa_bound 분할 단계
    double cusp_guard   = 0.6;      // |d * kappa_ref| 상한
    double obs_margin   = 4.0;      // cm, 차체 반폭에 더하는 corridor 여유
    double lane_gate    = 60.0;     // cm, 이보다 먼 lane 관측은 융합 제외
    double preview      = 450.0;    // cm, 종점 너머 장애물/곡률 예고 구간

    // --- 비용 가중치 ---
    double w_obstacle     = 4.0;
    double w_ref          = 1.0;
    double w_kappa        = 2.0;
    double w_end          = 0.6;
    double w_continuity   = 10.0;   // 스윕 결과 10 근방 최적, 20+ 부터 안전 악화
    double w_path_preview = 2.0;

    // 2026-08-24: 바퀴가 노면 밖인 구간의 비율에 붙는 비용.
    //
    // 규정상 hard reject 는 "네 바퀴 전부 밖" 이라 사실상 거의 걸리지 않는다.
    // 그것만 두면 도로 제약이 사라져 잔디 위로 세 바퀴 걸친 경로가 정상 후보가
    // 된다. 그래서 이탈을 벽이 아니라 **비용**으로 둔다 -- 평소엔 노면 안으로
    // 붙고, 그럴 만한 이유(장애물/곡률)가 있을 때만 물고 나간다.
    // 선을 밟기만 하는 것에는 페널티가 0 이다 (규정상 감점이 아니므로).
    //
    // 크기 근거 (스윕 전 초기값): 이 항은 "장애물 회피보다는 싸고, 횡오프셋
    // 선호보다는 확실히 비싸야" 한다.
    //   - 장애물 항 최대 = w_obstacle * 1.0 = 4.0  (충돌 회피가 항상 이겨야 함)
    //   - 오프셋 항 = w_ref*|d|/18 + w_end*0.8|d|/18, d=20cm 에서 약 1.6
    //   - "경로의 10% 를 두 바퀴 이탈" = ratio 0.05 -> 이게 1.5 쯤 되게
    // -> 0.05 * w_road = 1.5, w_road = 30.
    double w_road = 30.0;

    double clear_target = 15.0;     // cm, 이 이상 여유면 장애물 항 0

    double d_scale = 18.0;          // cm, 후보 offset 정규화 기준 (L_plan 무관 고정값)

    // --- P0 앵커 (2026-08-25) ---
    //
    // P0 를 실측 자세에 그대로 두면 추종오차 e 가 매 틱 계획에 실린다.
    // 첫 knot 이 0.25*l_plan(=75cm) 이므로 플래너는 e 를 75cm 안에 없애라고
    // 요구하는데(추가 곡률 ~ 4e/d^2), Pure Pursuit 의 횡오차 수렴은 Ld(=50cm
    // @1m/s)의 몇 배 거리가 든다. 즉 오차가 줄기 전에 요구가 커져 발산한다:
    //   e=7cm, 조향 70% 코너 -> kappa0 가 kappa_lim 을 넘어 전 후보 탈락.
    //
    // 그래서 P0 를 이전 틱 경로 위 최근접점으로 옮긴다. 이 투영은 자차 편차를
    // 종방향(s* 에 흡수 = 진행)과 횡방향(e = 추종오차)으로 정확히 가르고,
    // 횡방향만 버린다 -- 횡오차 수렴은 제어기 소관이다.
    //
    // alpha 는 두 극단을 잇는 혼합비다. e 가 크면(측위 점프, 물리적 이탈)
    // 실측으로 되돌아가 복귀 능력을 유지한다.
    //   e <= lo  -> alpha=0 : 이전 경로의 G2 순수 연장
    //   e >= hi  -> alpha=1 : 실측 자세 + 참조 곡률
    double anchor_blend_lo_cm = 5.0;    // 정상 주행 추종오차보다 커야 루프가 끊긴다
    double anchor_blend_hi_cm = 15.0;   // 이 밖은 이전 경로를 앵커로 쓰지 않는다
    double anchor_max_age_sec = 0.5;    // 이전 경로가 이보다 낡으면 alpha=1

    // --- 검증 구간 (2026-08-25) ---
    //
    // 경로는 l_plan(300cm) 을 그리지만 hard gate 를 이 길이까지만 건다. 그
    // 너머는 도착 전에 재계획되므로 비용으로만 다룬다.
    //
    // 크기는 **제어기가 보는 점**이 정한다. Pure Pursuit 의 목표점은 경로
    // 시작이 아니라 (경로 위 자차 위치 + Ld) 에 있고, 자차는 경로를 받은 뒤
    // 다음 경로가 올 때까지 이미 앞으로 나가 있다:
    //
    //   필요 길이 = Ld + (한 주기 이동) + LD_VALID_MARGIN_CM
    //             = k_v*v + v/plan_hz + 5
    //             = 0.5v*100 + v*100/5 + 5  =  70v + 5   [cm, v in m/s]
    //
    //   v 1.0 -> 75cm   v 1.5 -> 110cm   v 2.0 -> 145cm
    //
    // 예전 값(kCommittedHorizonCm 52.422)은 세그먼트 단위 절단이라 실효
    // 75cm 였고, 그건 정확히 v=1.0 짜리다. v_max=2.0 으로 달리면 제어기가
    // 검증 안 된 구간의 점을 보고 조향한다 (실측 로그에서 그 구간 곡률이
    // 반경 28~47cm 로 나왔다 -- 차량 최소회전반경 49.5cm 보다 작다).
    //
    // ★ plan_hz / k_v / v_max 중 하나라도 바꾸면 이 값을 다시 계산할 것.
    double validated_horizon_cm = 145.0;
};

// ====================================================================
// VehicleFootprint -- 회전 사각형 차체 (후륜축 기준), 2026-08-25.
//
// 원본: KAU_AMET_Test / src/sim_common/config.py 의 VehicleParams
// (body_length=28, body_width=20, rear_overhang=5 -> body_front=23,
// half_width=10). 3분할 원 근사(body_radius_cm, 여전히 candidate 생성
// -- offset 목표값 계산 -- 에는 그대로 쓰인다)를 hard-gate 두 곳
// (_clearance/_road_clearance 대응, roadClearanceRect/clearanceRect)
// 에서만 실제 사각형으로 교체한다.
// ====================================================================

struct VehicleFootprint
{
    double body_front_cm    = 23.0;   // 후륜축 -> 전단
    double rear_overhang_cm = 5.0;    // 후륜축 -> 후단
    double half_width_cm    = 10.0;

    // 2026-08-24: 위 치수는 **후륜축** 기준인데, 경로 점(및 TF 로 받는 자차
    // 위치)은 base_footprint 다. PhysiCar URDF 는 base_footprint 를 휠베이스
    // **중앙**에 둔다 (physicar.urdf.xacro: 바퀴 x = ±wheelbase/2). 따라서
    // 치수를 경로 점 위에 그대로 얹으면 사각형이 9cm 앞으로 밀린다.
    // kau_control VehicleParams::rear_axle_offset (-9.0) 과 같은 보정이며,
    // 그쪽 주석도 "빼먹으면 9cm 앞을 후륜축으로 착각한다" 고 경고한다.
    //
    // 기본값 0 은 "치수가 이미 경로 점 기준" 이라는 뜻이다 (기하 단위테스트가
    // 쓰는 중립값). 실차/실사용 값은 노드 파라미터 rear_axle_offset_cm 이 준다.
    double rear_axle_offset_cm = 0.0;

    // 경로 점 기준 사각형 종방향 범위 [뒤, 앞].
    double rearEdge() const { return rear_axle_offset_cm - rear_overhang_cm; }
    double frontEdge() const { return rear_axle_offset_cm + body_front_cm; }
};

// ====================================================================
// WheelFootprint -- 도로 이탈(감점) 판정 전용 바퀴 4점, 2026-08-24.
//
// 대회 규정: 흰 실선은 밟아도 되고 (선의 바깥 모서리가 곧 아스팔트 끝이라
// track 폴리곤과 일치한다), 네 바퀴가 전부 노면 밖으로 나가야 감점이다.
// 즉 이탈 판정의 대상은 차체가 아니라 바퀴다 -- 범퍼가 잔디 위로 넘어가는
// 것은 이탈이 아니다.
//
// 차체 사각형으로 판정하면 코너에서 실제보다 훨씬 바깥을 본다
// (base_footprint 기준 앞모서리 25.1cm vs 앞바퀴 외측 13.3cm, 약 12cm 차이).
// 그만큼 후보가 과도하게 탈락해 leastViolation 으로 떨어진다.
//
// 치수 출처: physicar.urdf.xacro (wheelbase 0.18, track_width 0.16 "measured",
// wheel_width 0.035).
// ====================================================================

struct WheelFootprint
{
    double rear_axle_offset_cm = 0.0;   // 경로 점 -> 후륜축 (VehicleFootprint 와 같은 규약)
    double wheelbase_cm   = 18.0;
    double track_width_cm = 16.0;       // 좌우 바퀴 **중심** 간 거리
    double wheel_width_cm = 3.5;

    // 노면 위에 남은 바퀴가 이보다 적어지면 hard reject. 규정이 "한 바퀴만
    // 안에 있으면 감점 없음" 이므로 1 이 규정 그대로다. 더 보수적으로 몰고
    // 싶으면 4 (네 바퀴 모두 안) 로 올린다.
    int min_wheels_on = 1;

    double rearAxleX() const { return rear_axle_offset_cm; }
    double frontAxleX() const { return rear_axle_offset_cm + wheelbase_cm; }
    double outerY() const { return 0.5 * (track_width_cm + wheel_width_cm); }
    double innerY() const
    {
        const double y = 0.5 * (track_width_cm - wheel_width_cm);
        return y > 0.0 ? y : 0.0;
    }
};

// ====================================================================
// Candidate -- 후보 offset 1 개의 평가 결과.
// Python: (d, curve|None, cost, reason) 튜플
// ====================================================================

struct Candidate
{
    double                d      = 0.0;    // 종점 offset [cm]
    std::optional<Curve>  curve;           // 없으면 구성 자체 실패
    double                cost   = 0.0;    // inf 면 hard constraint 위반
    std::string           reason;          // "" 면 유효 (cusp/degenerate/kappa_bound/
                                            // road_boundary/obstacle/kappa_exact 등)

    bool valid() const { return curve.has_value() && reason.empty(); }
};

// ====================================================================
// PlanResult -- plan() 최종 출력. local_planner_node 가 이걸 KauPath 로 변환해 발행.
// ====================================================================

enum class PlanStatus
{
    kOk,          // hard constraint 전부 만족하는 후보 중 최소비용 선택
    kDegraded,    // 만족하는 후보 없음 -- 위반 최소 후보를 그대로 발행
    kNoFeasible,  // curve 구성 자체가 전부 실패 (path 없음)
    kDegenerate,
};

struct PlanResult
{
    std::optional<Curve> path;
    PlanStatus            status = PlanStatus::kNoFeasible;
    double                chosen_offset = 0.0;
    double                kappa_max     = 0.0;
    double                clearance     = 0.0;   // cm, 음수면 충돌
    double                calc_ms       = 0.0;
    double                s0            = 0.0;   // reference 투영 호길이
    int                   alive         = 0;     // 제약 통과 후보 수 (진단용)
    bool                  lane_used     = false;

    // --- 도로 이탈 진단 (2026-08-24). 채택된 경로에 대해서만 채운다. ---
    //
    // 지금까지는 status 만 나와서 "정말 노면 밖으로 나갔는지, 어디서부터
    // 나갔는지" 를 알 수 없었다. 특히 committed horizon 완화 때문에 앞
    // 구간만 검증되고 뒤가 무검증인데, 그 사실이 status 에 드러나지 않는다.
    int    min_wheels_on        = 4;     // 전 구간에서 가장 적게 남은 "노면 위" 바퀴 수
    double road_clear_full      = 0.0;   // cm, 전 구간 바퀴 최소 여유 (음수 = 경계 물음)
    double road_clear_committed = 0.0;   // cm, committed 구간만 같은 값
    double road_viol_s          = -1.0;  // cm, 바퀴가 처음 완전히 나간 호길이 (없으면 음수)
    double road_off_len         = 0.0;   // cm, 이탈 적분값 (w_road 가 쓰는 값과 동일)

    // --- P0 앵커 진단 (2026-08-25) ---
    //
    // 증폭 루프가 끊겼는지는 anchor_e_cm 이 유계인지로 판정한다. 이 값이
    // 단조 증가하면서 kappa_max 도 같이 오르면 루프가 살아있는 것이다.
    double anchor_e_cm     = 0.0;   // cm, 이전 경로에서 자차까지 횡거리 (추종오차)
    double anchor_alpha    = 1.0;   // 0=이전 경로 앵커, 1=실측 자세
    double anchor_margin_cm = 0.0;  // cm, 이번 틱 road/obstacle 마진에 더한 값
    double kappa0          = 0.0;   // 1/cm, 이번 틱 P0 곡률 (clamp 후)

    // hard gate 를 실제로 건 호길이. KauPath.valid_length 로 발행되어
    // steer_controller 가 Ld 를 이 안으로 제한한다 (그 너머는 무검증이라
    // lookahead 점을 두면 안 된다). 0 이면 "제한 없음" 으로 해석되므로
    // 경로가 있을 때는 반드시 양수여야 한다.
    double valid_length_cm = 0.0;
};

}  // namespace local_path_planner
}  // namespace kau

#endif  // KAU_LOCAL_PATH_PLANNER__TYPES_HPP_
