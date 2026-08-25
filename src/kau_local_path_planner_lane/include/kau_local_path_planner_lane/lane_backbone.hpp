// ====================================================================
// lane_backbone.hpp (lane-only 재설계, 2026-08-25)
//
// reference_fusion.hpp(global+lane 융합, TF 투영)를 대체한다. backbone 을
// /lane/left, /lane/right, /lane/center(KauPath, base_link) 만으로 매
// 사이클 새로 만든다 -- map/global path/localization 배제.
//
// 원본: KAU_AMET_Test Python 세션(2026-08-25, map-free 재설계) 의
// LocalPlanner._build_backbone / _extend_curve / _tail_segments /
// _extrapolate_knot.
//
// Observation Horizon(lane detection 실제 관측 범위) 과 Planning
// Horizon(l_plan) 을 분리한다: center/edge 가 target_length 보다 짧으면
// 마지막 knot 의 heading/curvature 로 등곡률(원호) 외삽해 채운다 -- 그
// 구간은 실제 관측이 아니다(Curve 자체엔 이 구분을 남기지 않음, 필요하면
// 호출측이 원본 길이를 별도로 기억할 것).
// ====================================================================

#ifndef KAU_LOCAL_PATH_PLANNER_LANE__LANE_BACKBONE_HPP_
#define KAU_LOCAL_PATH_PLANNER_LANE__LANE_BACKBONE_HPP_

#include <optional>

#include "kau_control/curve.hpp"

namespace kau
{
namespace local_path_planner_lane
{

using kau::control::Curve;

struct BackboneResult
{
    std::optional<Curve> backbone;   // ego 원점(s=0)에서 시작, target_length 까지
    std::optional<Curve> left;       // 연장된 좌 edge (road boundary 용)
    std::optional<Curve> right;      // 연장된 우 edge
};

// left/right/center 는 /lane/left,/lane/right,/lane/center 를 이미 Curve 로
// 변환한 것(base_link 상대), 없으면 nullptr. kappa0 는 ego 원점 knot(현재
// 곡률). target_length 는 Planning Horizon(params.l_plan). center 가
// 없으면(양쪽 edge 모두 무효) backbone=nullopt -- 호출부가 최후수단으로
// 처리할 것.
BackboneResult buildBackbone(
    const Curve * left, const Curve * right, const Curve * center,
    double kappa0, double target_length, int bound_depth);

// 실측 곡선을 마지막 knot 의 heading/curvature 로 등곡률 외삽해 최소
// target_length 까지 늘린다. 이미 충분히 길면 그대로 반환.
Curve extendCurve(const Curve & cv, double target_length, int bound_depth);

}  // namespace local_path_planner_lane
}  // namespace kau

#endif  // KAU_LOCAL_PATH_PLANNER_LANE__LANE_BACKBONE_HPP_
