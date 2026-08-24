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
    const kau::control::TrackState st = previous_path->nearestGlobal(p0);

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
    const Curve & candidate, const Curve * previous_path,
    double road_off_ratio)
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
         + params.w_path_preview * path_ahead
         + params.w_road * std::max(0.0, road_off_ratio);
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
    //
    // 2026-08-24 KAU_AMET_Test 세션 결정 반영 (이 함수는 원래 세 위반을
    // 동일 가중치로 합산했는데, 그 경우 obs_margin 을 키우면 정규화
    // 분모가 커져 상대적 obstacle penalty 가 작아지면서 오히려 collision
    // 을 더 자주 고르는 역효과가 실측 확인됐다: obs_margin 4->10 으로
    // 키웠더니 min_clearance 가 -0.44 -> -0.79cm 로 악화). 장애물 충돌을
    // 항상 최우선으로 회피하도록, 먼저 obstacle_violation 최소인 후보군
    // 으로 pool 을 좁히고, 그 안에서만 road+kappa 위반 합이 최소인 것을
    // 고른다 -- 곡률/도로 이탈보다 장애물 충돌이 항상 더 나쁘다고 본다.
    struct Scored { double d; Curve cv; double obstacle_violation; double road_violation; double kappa_violation; };
    std::vector<Scored> scored;

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

        scored.push_back({c.d, cv, obstacle_violation, road_violation, kappa_violation});
    }
    if (scored.empty())
    {
        return std::nullopt;
    }

    double min_obstacle = std::numeric_limits<double>::infinity();
    for (const Scored & s : scored)
    {
        min_obstacle = std::min(min_obstacle, s.obstacle_violation);
    }

    std::optional<std::pair<double, Curve>> best;
    double best_score = std::numeric_limits<double>::infinity();
    for (const Scored & s : scored)
    {
        if (s.obstacle_violation > min_obstacle + 1e-9)
        {
            continue;
        }
        const double score =
            s.road_violation / road_safety_margin_cm
            + s.kappa_violation / kappa_max_vehicle;
        if (score < best_score)
        {
            best_score = score;
            best = std::make_pair(s.d, s.cv);
        }
    }
    return best;
}

std::optional<std::pair<double, Curve>> leastViolationRect(
    const std::vector<Candidate> & cands,
    const RoadBoundary & boundary, const VehicleFootprint & body,
    const WheelFootprint & wheels,
    double road_safety_margin_cm, double road_sample_interval_cm,
    const std::vector<Obstacle> & obstacles, double obs_margin,
    double clear_target_cm, double kappa_max_vehicle)
{
    struct Scored { double d; Curve cv; double obstacle_violation; double road_violation; double kappa_violation; };
    std::vector<Scored> scored;

    for (const Candidate & c : cands)
    {
        if (!c.curve.has_value() || !c.curve->isRegular())
        {
            continue;
        }
        const Curve & cv = *c.curve;

        const double kappa_violation =
            std::max(0.0, cv.kappaMax() - kappa_max_vehicle);
        // 도로 위반의 깊이는 "가장 깊이 나간 바퀴" 로 잰다 (차체 외곽이 아님).
        const double road_violation = std::max(
            0.0, -roadReportWheels(boundary, cv, wheels, road_safety_margin_cm,
                                   road_sample_interval_cm).min_clear_cm);
        const double obstacle_violation = std::max(
            0.0, obs_margin - clearanceRect(cv, obstacles, body, clear_target_cm,
                                            road_sample_interval_cm));

        scored.push_back({c.d, cv, obstacle_violation, road_violation, kappa_violation});
    }
    if (scored.empty())
    {
        return std::nullopt;
    }

    double min_obstacle = std::numeric_limits<double>::infinity();
    for (const Scored & s : scored)
    {
        min_obstacle = std::min(min_obstacle, s.obstacle_violation);
    }

    std::optional<std::pair<double, Curve>> best;
    double best_score = std::numeric_limits<double>::infinity();
    for (const Scored & s : scored)
    {
        if (s.obstacle_violation > min_obstacle + 1e-9)
        {
            continue;
        }
        const double score =
            s.road_violation / road_safety_margin_cm
            + s.kappa_violation / kappa_max_vehicle;
        if (score < best_score)
        {
            best_score = score;
            best = std::make_pair(s.d, s.cv);
        }
    }
    return best;
}

}  // namespace local_path_planner
}  // namespace kau
