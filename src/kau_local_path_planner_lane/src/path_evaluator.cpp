#include "kau_local_path_planner_lane/path_evaluator.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "kau_local_path_planner_lane/bezier_ext.hpp"

namespace kau
{
namespace local_path_planner_lane
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

namespace
{

// ------------------------------------------------------------------
// 곡률 배리어 (2026-08-25).
//
// 문제: 곡선에서 코너 **바깥쪽** 오프셋이 곡률 수요를 크게 낮추는데
// (kappa/(1-d*kappa), R=60cm 좌커브에서 d=-18cm 면 0.01667 -> 0.01282,
// 실효반경 60 -> 78cm) 비용함수가 그쪽을 못 고르고 있었다. 오프셋
// 페널티가 곡률 이득보다 비쌌기 때문이다:
//     d=18.7cm 오프셋 페널티  w_ref*|d|/18 + w_end*0.8|d|/18 = 1.54
//     같은 오프셋의 곡률 이득  w_kappa*(dkappa)/kappa_max     = 0.40
// 실측(R=70cm 코너, 후보 덤프): 살아남는 후보가 바깥쪽 둘뿐인데
// (d=-12.6 kmax 0.01628 / d=-4.2 kmax 0.01716), 곡률이 더 나쁜 -4.2 를
// cost 2.187 < 2.875 로 골랐다.
//
// 고침: 선형항(bound/kappa_max_vehicle)은 그대로 두고, 한계 근처에서만
// 급상승하는 항을 **더한다**. 무릎(knee) 아래에서는 정확히 0 이라
// 직선/완만한 곡선의 기존 거동은 바뀌지 않는다 (R=150 에서 +0.07,
// 직선에서 0.00). 한계에 다가갈수록 발산해 "곡률 여유"를 오프셋
// 선호보다 확실히 비싸게 만든다.
//
//     u = bound / kappa_lim,  x = (u - knee) / (1 - knee)
//     barrier = x^2 / (1 - x)      (u <= knee 면 0)
//
// x^2 으로 시작해 무릎에서 값도 기울기도 연속이다(비용 지형에 계단이
// 생기면 후보 선택이 틱마다 튄다). 분모의 하한은 bound 가 kappa_lim 을
// 넘는 구간(prediction horizon 완화로 살아남은 후보)에서 inf/NaN 이
// 되지 않게 한다 -- 그 후보들은 이미 kPredictionViolationPenalty 를
// 따로 맞으므로 여기서는 유한하고 아주 큰 값이면 충분하다.
// ------------------------------------------------------------------
constexpr double kKappaBarrierFloor = 1.0e-3;   // 분모 하한 (0 나눗셈 방지)

double kappaBarrier(double bound, double kappa_lim, double knee, double cap)
{
    if (!(kappa_lim > 0.0) || !(cap > 0.0) || !std::isfinite(bound))
    {
        return 0.0;
    }
    const double k = std::clamp(knee, 0.0, 0.99);
    const double u = bound / kappa_lim;
    if (u <= k)
    {
        return 0.0;
    }
    const double x = (u - k) / (1.0 - k);
    return std::min(cap, x * x / std::max(kKappaBarrierFloor, 1.0 - x));
}

}  // namespace

double cost(
    const PlannerParams & params, double kappa_max_vehicle,
    const Curve & global_path, double kappa_lim, double s0,
    double d, double peak, double bound, double clear,
    const Curve & candidate, const Curve * previous_path,
    double road_off_ratio)
{
    // 2026-08-25 (lane-only 재설계): `pathPreviewCost`(global path 곡률
    // 프리뷰) 제거 -- global path 가 없어져 l_plan 너머 "미래 곡률"을 알
    // 방법이 없다 (KAU_AMET_Test Python 세션과 동일 판단). 인자로 받는
    // global_path/s0 은 이제 이 함수 안에서 안 쓴다(시그니처는 호출부
    // 변경을 최소화하려고 유지). kappa_lim 은 아래 배리어가 다시 쓴다.
    (void)global_path; (void)s0;
    const double sc = params.d_scale;
    const double obs = std::max(
        0.0, (params.clear_target - clear) / params.clear_target);
    const double cont = continuityCost(candidate, previous_path, sc);

    return params.w_obstacle * obs
         + params.w_ref * peak / sc
         + params.w_kappa * (bound / kappa_max_vehicle
                             + kappaBarrier(bound, kappa_lim,
                                            params.kappa_barrier_knee,
                                            params.kappa_barrier_cap))
         + params.w_end * std::abs(0.80 * d) / sc
         + params.w_continuity * cont
         + params.w_road * std::max(0.0, road_off_ratio);
}

// reason 은 여기서 무시한다 -- kappa_bound/road_boundary/obstacle/kappa_exact
// 로 걸러진 후보도 "곡선 구성 자체는 성공" 했다면 pool 대상이다.
//
// 2026-08-24 KAU_AMET_Test 세션 결정: 원래 세 위반을 동일 가중치로 합산했는데,
// 그러면 obs_margin 을 키울수록 정규화 분모가 커져 상대적 obstacle penalty 가
// 작아지면서 오히려 collision 을 더 자주 고르는 역효과가 실측됐다 (obs_margin
// 4->10 으로 키웠더니 min_clearance -0.44 -> -0.79cm 악화). 그래서 먼저
// obstacle_violation 최소인 후보군으로 pool 을 좁히고, 그 안에서만 road+kappa
// 위반 합이 최소인 것을 고른다.
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

}  // namespace local_path_planner_lane
}  // namespace kau
