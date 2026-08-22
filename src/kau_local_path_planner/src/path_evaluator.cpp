#include "kau_local_path_planner/path_evaluator.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "kau_local_path_planner/bezier_ext.hpp"

namespace kau
{
namespace local_path_planner
{

double continuityCost(
    const Curve & candidate, const Curve * previous_path, double d_scale)
{
    if (previous_path == nullptr)
    {
        return 0.0;
    }

    const Point2 p0 = referenceFrame(candidate, candidate.wrapS(0.0)).point;
    // Python: prev.nearest_global(p0) -- window 상태를 안 쓰고 매번 전역
    // 탐색 (previous_path 는 candidate 와 다른 곡선이라 window 재사용 불가).
    const TrackState st = previous_path->nearestGlobal(p0);

    const double common = std::min(
        candidate.length(), previous_path->length() - st.s);
    if (common <= 1e-6)
    {
        return 0.0;
    }

    constexpr int kSamples = 5;
    double sum_sq = 0.0;
    for (int i = 0; i < kSamples; ++i)
    {
        const double s = common * static_cast<double>(i) / (kSamples - 1);
        const Point2 a =
            referenceFrame(candidate, candidate.wrapS(s)).point;
        const Point2 b = referenceFrame(
            *previous_path, previous_path->wrapS(st.s + s)).point;
        const double dx = a.x - b.x;
        const double dy = a.y - b.y;
        sum_sq += dx * dx + dy * dy;
    }
    return (sum_sq / kSamples) / (d_scale * d_scale);
}

double pathPreviewCost(
    const Curve & global_path, double d, double end_ratio, double s0,
    double l_plan, double preview, double kappa_lim)
{
    if (preview <= 0.0)
    {
        return 0.0;
    }
    const double end = s0 + l_plan;
    const double kappa_ahead = global_path.kappaMaxOver(end, end + preview);
    if (kappa_ahead <= 1e-9)
    {
        return 0.0;
    }
    const double d_end = end_ratio * d;
    const double denom = 1.0 - std::abs(d_end) * kappa_ahead;
    if (denom <= 1e-3)
    {
        return 1.0;
    }
    const double kappa_via_d = kappa_ahead / denom;
    return std::max(0.0, kappa_via_d - kappa_lim) / kappa_lim;
}

double cost(
    const PlannerParams & params, double kappa_max_vehicle,
    const Curve & global_path, double kappa_lim, double s0,
    double d, double peak, double bound, double clear,
    const Curve & candidate, const Curve * previous_path)
{
    const double sc = params.d_scale;
    const double obs = std::max(
        0.0, (params.clear_target - clear) / params.clear_target);
    const double cont = continuityCost(candidate, previous_path, sc);
    const double path_ahead = pathPreviewCost(
        global_path, d, /* END_RATIO */ 0.80, s0, params.l_plan,
        params.preview, kappa_lim);

    return params.w_obstacle * obs
         + params.w_ref * peak / sc
         + params.w_kappa * bound / kappa_max_vehicle
         + params.w_end * std::abs(0.80 * d) / sc
         + params.w_continuity * cont
         + params.w_path_preview * path_ahead;
}

std::optional<std::pair<double, Curve>> leastViolation(
    const std::vector<Candidate> & cands,
    const RoadBoundary & boundary, double body_radius_cm,
    double road_safety_margin_cm, double road_sample_interval_cm,
    const std::vector<Obstacle> & obstacles, double obs_margin,
    double clear_target_cm, double kappa_max_vehicle)
{
    // Python: pool = [(d, cv) for d, cv, _, _ in cands
    //                 if cv is not None and cv.is_regular()]
    // reason 은 여기서 무시한다 -- kappa_bound/road_boundary/obstacle/
    // kappa_exact 로 걸러진 후보도 "곡선 구성 자체는 성공"했다면 pool 대상.
    std::optional<std::pair<double, Curve>> best;
    double best_score = std::numeric_limits<double>::infinity();

    for (const Candidate & c : cands)
    {
        if (!c.curve.has_value() || !c.curve->isRegular())
        {
            continue;
        }
        const Curve & cv = *c.curve;

        const double kappa_violation =
            std::max(0.0, cv.kappaMax() - kappa_max_vehicle);
        const double road_violation = std::max(
            0.0, -roadClearance(boundary, cv,
                               body_radius_cm + road_safety_margin_cm,
                               road_sample_interval_cm));
        const double obstacle_violation = std::max(
            0.0, obs_margin - clearance(cv, obstacles, body_radius_cm,
                                        clear_target_cm));

        const double score =
            obstacle_violation / obs_margin
            + road_violation / road_safety_margin_cm
            + kappa_violation / kappa_max_vehicle;

        if (score < best_score)
        {
            best_score = score;
            best = std::make_pair(c.d, cv);
        }
    }
    return best;
}

}  // namespace local_path_planner
}  // namespace kau
