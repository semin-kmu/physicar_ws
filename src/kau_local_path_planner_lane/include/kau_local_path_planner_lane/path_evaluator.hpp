// ====================================================================
// path_evaluator.hpp
//
// 비용함수 + degraded 최후수단 선택.
//
// 원본: KAU_AMET_Test / src/kau_local_path_planner_lane/test/planner.py 의
//       LocalPlanner._cost / _continuity_cost / _path_preview_cost /
//       _least_violation
// ====================================================================

#ifndef KAU_LOCAL_PATH_PLANNER_LANE__PATH_EVALUATOR_HPP_
#define KAU_LOCAL_PATH_PLANNER_LANE__PATH_EVALUATOR_HPP_

#include <optional>
#include <utility>
#include <vector>

#include "kau_control/curve.hpp"
#include "kau_local_path_planner_lane/boundary_checker.hpp"
#include "kau_local_path_planner_lane/collision_checker.hpp"
#include "kau_local_path_planner_lane/types.hpp"

namespace kau
{
namespace local_path_planner_lane
{

using kau::control::Curve;

// Python: _continuity_cost(cv). previous_path 가 nullptr 이면 0.
double continuityCost(
    const Curve & candidate, const Curve * previous_path, double d_scale);

// Python: _path_preview_cost(d, s0). preview 구간 global path 자체 곡률
// 여유 부족 -- 부호(증폭/완화측) 구분 없이 최악을 가정하는 보수적 근사.
double pathPreviewCost(
    const Curve & global_path, double d, double end_ratio, double s0,
    double l_plan, double preview, double kappa_lim);

// Python: _cost(d, peak, bound, clear, cv).
// clear 는 호출측이 이미 min(obstacle_clear, preview_clear) 로 합친 값을
// 넘긴다 (Python 의 _candidate 호출부와 동일 관례).
//
// road_off_ratio: 바퀴가 노면 밖인 구간의 비율 [0,1]
// (RoadReport::off_integral_cm / l_plan). 규정상 hard reject 는 "네 바퀴
// 전부 밖" 뿐이라 거의 걸리지 않으므로, 그보다 약한 이탈은 여기서 w_road
// 로 흡수한다 -- 이탈을 벽이 아니라 트레이드오프로 두기 위한 항이다.
// 선을 밟기만 하는 것은 0 이다 (규정상 감점이 아니다).
double cost(
    const PlannerParams & params, double kappa_max_vehicle,
    const Curve & global_path, double kappa_lim, double s0,
    double d, double peak, double bound, double clear,
    const Curve & candidate, const Curve * previous_path,
    double road_off_ratio = 0.0);

// Python: _least_violation(cands). hard constraint 를 전부 만족하는 fresh
// 후보가 하나도 없을 때의 최후 수단. 장애물/도로/곡률 위반 사이에 우선순위를
// 두지 않고(KAU_AMET_Test 세션 결정) 각 위반을 해당 안전마진으로 정규화해
// 동일 가중치로 합산, 최소인 후보를 고른다. 이 함수가 자주 호출된다면
// 채점 방식이 아니라 candidate 생성/탐색(l_plan, corridor, preview) 을
// 먼저 의심할 것.
std::optional<std::pair<double, Curve>> leastViolation(
    const std::vector<Candidate> & cands,
    const RoadBoundary & boundary, double body_radius_cm,
    double road_safety_margin_cm, double road_sample_interval_cm,
    const std::vector<Obstacle> & obstacles, double obs_margin,
    double clear_target_cm, double kappa_max_vehicle);

// 2026-08-25: `leastViolation` 과 동일하되, 3분할 원 근사 대신 실제 차체
// (회전 사각형)로 obstacle 위반을 계산한다 (`clearanceRect`).
// 2026-08-24: 도로 위반은 `wheels`(바퀴 4점), 장애물 위반은 `body`(차체
// 사각형) 로 잰다. 예전에는 둘 다 차체로 재서, 실제로는 바퀴가 멀쩡히
// 노면에 있는 후보를 "도로 위반" 으로 깎고 엉뚱한 후보를 골랐다.
std::optional<std::pair<double, Curve>> leastViolationRect(
    const std::vector<Candidate> & cands,
    const RoadBoundary & boundary, const VehicleFootprint & body,
    const WheelFootprint & wheels,
    double road_safety_margin_cm, double road_sample_interval_cm,
    const std::vector<Obstacle> & obstacles, double obs_margin,
    double clear_target_cm, double kappa_max_vehicle);

}  // namespace local_path_planner_lane
}  // namespace kau

#endif  // KAU_LOCAL_PATH_PLANNER_LANE__PATH_EVALUATOR_HPP_
