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

    double clear_target = 15.0;     // cm, 이 이상 여유면 장애물 항 0

    double d_scale = 18.0;          // cm, 후보 offset 정규화 기준 (L_plan 무관 고정값)
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
};

}  // namespace local_path_planner
}  // namespace kau

#endif  // KAU_LOCAL_PATH_PLANNER__TYPES_HPP_
