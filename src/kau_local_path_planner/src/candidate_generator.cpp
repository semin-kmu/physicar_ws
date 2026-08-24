#include "kau_local_path_planner/candidate_generator.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "kau_local_path_planner/path_evaluator.hpp"

namespace kau
{
namespace local_path_planner
{

namespace
{
constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr std::array<double, 3> kSigmaRatios{0.85, 1.05, 1.30};
}  // namespace

std::pair<std::optional<Ctrl>, double> fitSegment(
    const Knot & a, const Knot & b, int bound_depth)
{
    const double chord = std::hypot(b.p.x - a.p.x, b.p.y - a.p.y);
    if (chord < 1e-6)
    {
        return {std::nullopt, kInf};
    }

    std::optional<Ctrl> best;
    double best_kb = kInf;
    for (double r : kSigmaRatios)
    {
        const double s = r * chord;
        Ctrl c = hermiteToBezier(
            a.p, a.theta, a.kappa, b.p, b.theta, b.kappa, s, s);
        const double kb = kappaBound(c, bound_depth);
        if (kb < best_kb)
        {
            best = std::move(c);
            best_kb = kb;
        }
    }
    if (!std::isfinite(best_kb))
    {
        return {std::nullopt, kInf};
    }
    return {best, best_kb};
}

int committedSegmentCount(const std::vector<Ctrl> & segs)
{
    double station = 0.0;
    for (std::size_t i = 0; i < segs.size(); ++i)
    {
        station += kau::bezier::segLength(segs[i]);
        if (station >= kCommittedHorizonCm)
        {
            return static_cast<int>(i) + 1;
        }
    }
    return static_cast<int>(segs.size());
}

ObstacleStation makeStation(const Curve & global_path, const Obstacle & obstacle)
{
    const kau::control::TrackState st = global_path.nearestGlobal(obstacle.center);
    const Frame f = referenceFrame(global_path, st.s);
    const double vx = obstacle.center.x - f.point.x;
    const double vy = obstacle.center.y - f.point.y;

    ObstacleStation out;
    out.station_s = st.s;
    out.lateral = -std::sin(f.heading) * vx + std::cos(f.heading) * vy;
    out.obstacle = obstacle;
    return out;
}

CandidateGenerator::CandidateGenerator(
    const Curve & global_path, const RoadBoundary & boundary,
    const std::vector<Obstacle> & obstacles,
    const std::vector<ObstacleStation> & stations,
    const PlannerParams & params, double kappa_lim,
    double kappa_max_vehicle, double body_radius_cm,
    VehicleFootprint body_footprint, WheelFootprint wheels)
: global_path_(global_path), boundary_(boundary), obstacles_(obstacles),
  stations_(stations), params_(params), kappa_lim_(kappa_lim),
  kappa_max_vehicle_(kappa_max_vehicle), body_radius_cm_(body_radius_cm),
  body_footprint_(body_footprint), wheels_(wheels)
{
}

std::array<std::array<double, 3>, 7> CandidateGenerator::corridorOffsets(
    const std::array<Frame, 3> & frames) const
{
    const double footprint = body_radius_cm_ + roadMargin();
    std::array<std::array<double, 7>, 3> offsets{};

    for (int f = 0; f < 3; ++f)
    {
        const Point2 normal{-std::sin(frames[f].heading), std::cos(frames[f].heading)};
        const double safe_lower = -marginAlongNormal(
            boundary_, frames[f].point, normal, -1.0, footprint);
        const double safe_upper = marginAlongNormal(
            boundary_, frames[f].point, normal, 1.0, footprint);
        for (std::size_t i = 0; i < kCorridorFractions.size(); ++i)
        {
            offsets[static_cast<std::size_t>(f)][i] =
                safe_lower + kCorridorFractions[i] * (safe_upper - safe_lower);
        }
    }

    std::array<std::array<double, 3>, 7> pairs{};
    for (std::size_t i = 0; i < 7; ++i)
    {
        pairs[i] = {offsets[0][i], offsets[1][i], offsets[2][i]};
    }
    return pairs;
}

Candidate CandidateGenerator::candidate(
    const Point2 & p, double yaw, double kappa0,
    const std::vector<Frame> & frames, const std::vector<double> & offsets,
    double s0, const Curve * previous_path) const
{
    std::vector<Knot> knots;
    knots.push_back(Knot{p, yaw, kappa0});
    double peak = 0.0;
    for (std::size_t i = 0; i < frames.size(); ++i)
    {
        const double d = offsets[i];
        const Frame & fr = frames[i];
        if (std::abs(d * fr.kappa) > params_.cusp_guard)
        {
            Candidate c; c.d = offsets.back(); c.reason = "cusp"; c.cost = kInf;
            return c;
        }
        const Point2 n{-std::sin(fr.heading), std::cos(fr.heading)};
        Knot k;
        k.p = Point2{fr.point.x + d * n.x, fr.point.y + d * n.y};
        k.theta = fr.heading;
        k.kappa = fr.kappa / (1.0 - d * fr.kappa);
        knots.push_back(k);
        peak = std::max(peak, std::abs(d));
    }

    std::vector<Ctrl> segs;
    std::vector<double> bounds;
    double bound = 0.0;
    for (std::size_t i = 0; i + 1 < knots.size(); ++i)
    {
        auto [seg_opt, kb] = fitSegment(knots[i], knots[i + 1], params_.bound_depth);
        if (!seg_opt)
        {
            Candidate c; c.d = offsets.back(); c.reason = "degenerate"; c.cost = kInf;
            return c;
        }
        segs.push_back(std::move(*seg_opt));
        bounds.push_back(kb);
        bound = std::max(bound, kb);
    }

    Curve cv(segs, false);
    // 2026-08-24 (committed/prediction horizon): stage-1(convex-hull 상한)
    // 도 committed horizon 안에서만 hard 하게 유지한다. 그 이후(먼 미래)
    // 에서만의 위반은 즉시 폐기 대신 cost 페널티로 남긴다.
    bool prediction_violation = false;
    if (bound > kappa_lim_)
    {
        const int n_committed = committedSegmentCount(segs);
        double committed_bound = 0.0;
        for (int k = 0; k < n_committed; ++k)
        {
            committed_bound = std::max(
                committed_bound, bounds[static_cast<std::size_t>(k)]);
        }
        if (committed_bound > kappa_lim_)
        {
            Candidate c; c.d = offsets.back(); c.curve = cv; c.reason = "kappa_bound";
            c.cost = kInf;
            return c;
        }
        prediction_violation = true;   // committed 는 OK, 나머지(먼 미래)만 위반
    }
    // 도로 이탈은 바퀴 기준 (규정: 흰선은 밟아도 되고 네 바퀴가 전부 나가야
    // 감점). hard reject 는 wheels_.min_wheels_on 미만일 때뿐이고, 그보다
    // 약한 이탈은 아래 w_road 비용으로 흡수한다.
    const RoadReport road = roadReportWheels(
        boundary_, cv, wheels_, roadMargin(), kRoadSampleIntervalCm);
    if (road.min_wheels_on < wheels_.min_wheels_on)
    {
        const int n_committed = committedSegmentCount(segs);
        bool committed_road_ok = false;
        if (n_committed < static_cast<int>(segs.size()))
        {
            Curve committed_cv(
                std::vector<Ctrl>(segs.begin(), segs.begin() + n_committed), false);
            committed_road_ok = roadOkWheels(
                boundary_, committed_cv, wheels_, roadMargin(),
                kRoadSampleIntervalCm);
        }
        if (!committed_road_ok)
        {
            Candidate c; c.d = offsets.back(); c.curve = cv; c.reason = "road_boundary";
            c.cost = kInf;
            return c;
        }
        prediction_violation = true;
    }
    // obstacle: committed/prediction 구분 없이 항상 엄격 (변경 없음)
    const double clear = clearanceRect(
        cv, obstacles_, body_footprint_, params_.clear_target, kRoadSampleIntervalCm);
    if (clear < obsMargin())
    {
        Candidate c; c.d = offsets.back(); c.curve = cv; c.reason = "obstacle";
        c.cost = kInf;
        return c;
    }

    const double d_final = offsets.back();
    const double preview = previewClear(
        global_path_, stations_, d_final, kEndRatio, s0, params_.l_plan,
        params_.preview, body_radius_cm_);
    const double combined_clear = std::min(clear, preview);
    double c_cost = cost(
        params_, kappa_max_vehicle_, global_path_, kappa_lim_, s0, d_final,
        peak, bound, combined_clear, cv, previous_path,
        road.off_integral_cm / params_.l_plan);
    if (prediction_violation)
    {
        c_cost += kPredictionViolationPenalty;
    }

    Candidate c; c.d = d_final; c.curve = cv; c.cost = c_cost; c.reason = "";
    return c;
}

Candidate CandidateGenerator::candidate(
    const Point2 & p, double yaw, double kappa0,
    const std::array<Frame, 2> & frames,
    const std::pair<double, double> & offsets,
    double s0, const Curve * previous_path) const
{
    return candidate(
        p, yaw, kappa0,
        std::vector<Frame>{frames[0], frames[1]},
        std::vector<double>{offsets.first, offsets.second},
        s0, previous_path);
}

std::vector<Candidate> CandidateGenerator::obstacleOffsetCandidates(
    const Point2 & p, double yaw, double kappa0,
    const std::array<Frame, 2> & frames, double s0,
    const Curve * previous_path) const
{
    struct Fwd { double advance; double station_s; const Obstacle * obstacle; };
    std::vector<Fwd> forward;
    for (const auto & st : stations_)
    {
        const double advance = global_path_.deltaS(s0, st.station_s);
        if (advance >= 20.0 && advance <= params_.l_plan + 80.0)
        {
            forward.push_back({advance, st.station_s, &st.obstacle});
        }
    }
    if (forward.empty())
    {
        return {};
    }
    std::sort(forward.begin(), forward.end(),
             [](const Fwd & a, const Fwd & b) { return a.advance < b.advance; });

    const double advance = forward.front().advance;
    const double station = forward.front().station_s;
    const Obstacle & obstacle = *forward.front().obstacle;
    const double required =
        obstacle.radius + body_radius_cm_ + obsMargin() + 0.5;
    const double footprint = body_radius_cm_ + roadMargin();
    const Frame & term_frame = frames[1];

    const auto build = [&](double lead, double ratio, double side) -> Candidate
    {
        Frame mid_frame;
        if (advance <= params_.l_plan - 20.0)
        {
            mid_frame = referenceFrame(global_path_, global_path_.wrapS(station - lead));
        }
        else
        {
            mid_frame = frames[0];
        }
        const std::array<Frame, 2> obstacle_frames{mid_frame, term_frame};
        std::array<double, 2> offsets{};
        for (int k = 0; k < 2; ++k)
        {
            const Frame & fr = obstacle_frames[static_cast<std::size_t>(k)];
            const Point2 normal{-std::sin(fr.heading), std::cos(fr.heading)};
            const double lateral =
                (obstacle.center.x - fr.point.x) * normal.x +
                (obstacle.center.y - fr.point.y) * normal.y;
            const double safe_lower =
                -marginAlongNormal(boundary_, fr.point, normal, -1.0, footprint);
            const double safe_upper =
                marginAlongNormal(boundary_, fr.point, normal, 1.0, footprint);
            const double scale = (k == 0) ? ratio : 1.0;
            const double target = lateral + side * required * scale;
            offsets[static_cast<std::size_t>(k)] =
                std::clamp(target, safe_lower, safe_upper);
        }
        return candidate(
            p, yaw, kappa0, obstacle_frames,
            std::make_pair(offsets[0], offsets[1]), s0, previous_path);
    };

    const std::array<double, 3> leads{
        std::min(60.0, 0.35 * advance),
        std::min(90.0, 0.55 * advance),
        std::min(35.0, 0.20 * advance)};
    // 2026-08-24 (grid 축소, KAU_AMET_Test 세션): 원래 6-combo(leads x
    // ratios) 중 (leads[1],1.0)/(leads[2],0.82) 는 selected/sole_rescuer/
    // loo_change 전부 0 -- 정적으로 제거해도 결과가 완전히 동일함을 1-lap
    // 실측으로 확인해 4-combo 로 축소했다 (계산량 -33%).
    const std::array<std::pair<double, double>, 4> combos{{
        {leads[0], 1.0}, {leads[0], 0.82}, {leads[1], 0.82}, {leads[2], 1.0}}};

    std::vector<Candidate> cands;
    cands.reserve(2);
    for (double side : {-1.0, 1.0})
    {
        std::vector<Candidate> pool;
        pool.reserve(combos.size());
        for (const auto & combo : combos)
        {
            pool.push_back(build(combo.first, combo.second, side));
        }
        std::vector<Candidate> feasible;
        for (auto & c : pool)
        {
            if (c.reason.empty())
            {
                feasible.push_back(c);
            }
        }
        if (!feasible.empty())
        {
            cands.push_back(*std::min_element(
                feasible.begin(), feasible.end(),
                [](const Candidate & a, const Candidate & b)
                { return a.cost < b.cost; }));
        }
        else
        {
            cands.push_back(build(leads[0], 1.0, side));
        }
    }
    return cands;
}

Candidate CandidateGenerator::primitiveCandidate(
    const std::vector<Knot> & knots, double terminal_offset, double peak,
    double s0, const Curve * previous_path) const
{
    std::vector<Ctrl> segs;
    double bound = 0.0;
    for (std::size_t i = 0; i + 1 < knots.size(); ++i)
    {
        auto [seg_opt, kb] = fitSegment(knots[i], knots[i + 1], params_.bound_depth);
        if (!seg_opt)
        {
            Candidate c; c.d = terminal_offset; c.reason = "degenerate"; c.cost = kInf;
            return c;
        }
        segs.push_back(std::move(*seg_opt));
        bound = std::max(bound, kb);
    }

    Curve cv(segs, false);
    if (bound > kappa_lim_)
    {
        Candidate c; c.d = terminal_offset; c.curve = cv; c.reason = "kappa_bound";
        c.cost = kInf;
        return c;
    }
    const RoadReport road = roadReportWheels(
        boundary_, cv, wheels_, roadMargin(), kRoadSampleIntervalCm);
    if (road.min_wheels_on < wheels_.min_wheels_on)
    {
        Candidate c; c.d = terminal_offset; c.curve = cv; c.reason = "road_boundary";
        c.cost = kInf;
        return c;
    }
    const double clear = clearanceRect(
        cv, obstacles_, body_footprint_, params_.clear_target, kRoadSampleIntervalCm);
    if (clear < obsMargin())
    {
        Candidate c; c.d = terminal_offset; c.curve = cv; c.reason = "obstacle";
        c.cost = kInf;
        return c;
    }

    const double c_cost = cost(
        params_, kappa_max_vehicle_, global_path_, kappa_lim_, s0,
        terminal_offset, peak, bound, clear, cv, previous_path,
        road.off_integral_cm / params_.l_plan);
    Candidate c; c.d = terminal_offset; c.curve = cv; c.cost = c_cost; c.reason = "";
    return c;
}

std::vector<Candidate> CandidateGenerator::obstaclePrimitives(
    const Point2 & start, double yaw, double kappa0, const Frame & terminal_frame,
    double s0, const Curve * previous_path) const
{
    struct Fwd { double advance; double station_s; const Obstacle * obstacle; };
    std::vector<Fwd> forward;
    for (const auto & st : stations_)
    {
        const double advance = global_path_.deltaS(s0, st.station_s);
        if (advance >= 20.0 && advance <= params_.l_plan + 80.0)
        {
            forward.push_back({advance, st.station_s, &st.obstacle});
        }
    }
    if (forward.empty())
    {
        return {};
    }
    std::sort(forward.begin(), forward.end(),
             [](const Fwd & a, const Fwd & b) { return a.advance < b.advance; });

    const double advance = forward.front().advance;
    const double station = forward.front().station_s;
    const Obstacle & obstacle = *forward.front().obstacle;
    if (advance > params_.l_plan - 20.0)
    {
        return {};
    }
    const Obstacle * next_obstacle =
        (forward.size() > 1) ? forward[1].obstacle : nullptr;

    const double lead = std::min(60.0, 0.35 * advance);
    const double apex_station = global_path_.wrapS(station - lead);
    const Frame ref = referenceFrame(global_path_, apex_station);
    const Point2 normal{-std::sin(ref.heading), std::cos(ref.heading)};
    const double cone_lateral =
        (obstacle.center.x - ref.point.x) * normal.x +
        (obstacle.center.y - ref.point.y) * normal.y;
    const double required =
        obstacle.radius + body_radius_cm_ + obsMargin() + 0.5;

    std::vector<Candidate> candidates;
    for (double side : {-1.0, 1.0})
    {
        const double apex_offset = cone_lateral + side * required;
        const Point2 apex_point{
            ref.point.x + apex_offset * normal.x,
            ref.point.y + apex_offset * normal.y};

        const Point2 terminal_normal{
            -std::sin(terminal_frame.heading), std::cos(terminal_frame.heading)};
        double terminal_offset;
        if (next_obstacle == nullptr)
        {
            terminal_offset = apex_offset;
        }
        else
        {
            const double next_lateral =
                (next_obstacle->center.x - terminal_frame.point.x) * terminal_normal.x +
                (next_obstacle->center.y - terminal_frame.point.y) * terminal_normal.y;
            const double direction = next_lateral >= 0.0 ? 1.0 : -1.0;
            const double terminal_required =
                next_obstacle->radius + body_radius_cm_ + obsMargin() + 0.5;
            terminal_offset = next_lateral - direction * terminal_required;
        }
        const Point2 end_point{
            terminal_frame.point.x + terminal_offset * terminal_normal.x,
            terminal_frame.point.y + terminal_offset * terminal_normal.y};

        const double apex_heading = ref.heading;
        const double apex_kappa =
            (std::abs(apex_offset * ref.kappa) < params_.cusp_guard)
                ? ref.kappa / (1.0 - apex_offset * ref.kappa) : 0.0;

        const Frame hold_ref =
            referenceFrame(global_path_, global_path_.wrapS(station + lead));
        const Point2 hold_normal{
            -std::sin(hold_ref.heading), std::cos(hold_ref.heading)};
        const Point2 hold_point{
            hold_ref.point.x + apex_offset * hold_normal.x,
            hold_ref.point.y + apex_offset * hold_normal.y};
        const double hold_kappa =
            (std::abs(apex_offset * hold_ref.kappa) < params_.cusp_guard)
                ? hold_ref.kappa / (1.0 - apex_offset * hold_ref.kappa) : 0.0;

        const double chord_heading = std::atan2(
            end_point.y - hold_point.y, end_point.x - hold_point.x);
        const double terminal_heading_used = terminal_frame.heading +
            std::clamp(kau::control::wrapPi(chord_heading - terminal_frame.heading),
                      -0.30, 0.30);
        constexpr double end_kappa = 0.0;

        std::vector<Knot> knots{
            Knot{start, yaw, kappa0},
            Knot{apex_point, apex_heading, apex_kappa},
            Knot{hold_point, hold_ref.heading, hold_kappa},
            Knot{end_point, terminal_heading_used, end_kappa},
        };
        candidates.push_back(primitiveCandidate(
            knots, terminal_offset,
            std::max(std::abs(apex_offset), std::abs(terminal_offset)),
            s0, previous_path));
    }
    return candidates;
}

std::vector<int> CandidateGenerator::directTargetIndices(
    double s0, const std::array<std::array<double, 3>, 7> & offset_pairs) const
{
    std::array<double, 7> absolute_targets;
    for (std::size_t i = 0; i < 7; ++i)
    {
        absolute_targets[i] = offset_pairs[i].back();
    }

    struct Fwd { double advance; double lateral; };
    std::vector<Fwd> forward;
    for (const auto & st : stations_)
    {
        const double advance = global_path_.deltaS(s0, st.station_s);
        if (advance >= 0.0 && advance <= params_.l_plan)
        {
            forward.push_back({advance, st.lateral});
        }
    }

    if (forward.empty())
    {
        int best_i = 0;
        double best_abs = kInf;
        for (std::size_t i = 0; i < 7; ++i)
        {
            const double a = std::abs(absolute_targets[i]);
            if (a < best_abs)
            {
                best_abs = a;
                best_i = static_cast<int>(i);
            }
        }
        return {best_i};
    }

    const auto nearest = std::min_element(
        forward.begin(), forward.end(),
        [](const Fwd & a, const Fwd & b) { return a.advance < b.advance; });
    const double obstacle_lateral = nearest->lateral;

    const std::array<double, 3> desired = (obstacle_lateral < 0.0)
        ? std::array<double, 3>{0.1625, 0.3875, 0.50}
        : std::array<double, 3>{0.8375, 0.6125, 0.50};

    std::vector<int> out;
    for (double fraction : desired)
    {
        int best_i = 0;
        double best_d = kInf;
        for (std::size_t i = 0; i < kCorridorFractions.size(); ++i)
        {
            const double d = std::abs(kCorridorFractions[i] - fraction);
            if (d < best_d)
            {
                best_d = d;
                best_i = static_cast<int>(i);
            }
        }
        out.push_back(best_i);
    }
    return out;
}

Candidate CandidateGenerator::directCandidate(
    const Point2 & start, double yaw, double kappa0, const Point2 & terminal,
    double reference_heading, double target, double s0,
    const Curve * previous_path)
{
    const double chord = std::hypot(terminal.x - start.x, terminal.y - start.y);
    if (chord < 1e-6)
    {
        Candidate c; c.d = target; c.reason = "degenerate"; c.cost = kInf;
        return c;
    }

    const double delta = kau::control::wrapPi(reference_heading - yaw);
    const double turn = delta >= 0.0 ? 1.0 : -1.0;
    const double chord_heading = std::atan2(terminal.y - start.y, terminal.x - start.x);
    const double geometry_offset = std::clamp(
        kau::control::wrapPi(chord_heading - reference_heading), -0.4, 0.4);

    using Combo = std::array<double, 4>;
    const std::vector<Combo> raw{
        {-0.20 * turn, 0.0, 0.6, 2.2},
        {geometry_offset, 0.0, 0.6, 2.2},
        {0.0, 0.0, 0.6, 2.2},
        {0.20 * turn, 0.0, 0.6, 2.2},
        {-0.20 * turn, 0.006 * turn, 0.6, 2.2},
        {-0.20 * turn, 0.0, 1.0, 2.2},
        {-0.20 * turn, 0.0, 0.6, 1.4},
        {0.0, 0.0, 1.0, 1.4},
    };
    std::vector<Combo> combinations;
    for (const auto & c : raw)
    {
        if (std::find(combinations.begin(), combinations.end(), c) ==
            combinations.end())
        {
            combinations.push_back(c);
        }
    }
    if (direct_seed_.has_value())
    {
        auto it = std::find(combinations.begin(), combinations.end(), *direct_seed_);
        if (it != combinations.end())
        {
            combinations.erase(it);
        }
        combinations.insert(combinations.begin(), *direct_seed_);
    }

    std::string best_reason = "kappa_exact";
    for (const auto & combo : combinations)
    {
        const double heading_offset = combo[0];
        const double terminal_kappa = combo[1];
        const double start_ratio = combo[2];
        const double end_ratio = combo[3];

        Ctrl ctrl = hermiteToBezier(
            start, yaw, kappa0, terminal, reference_heading + heading_offset,
            terminal_kappa, start_ratio * chord, end_ratio * chord);
        if (!kau::bezier::isRegular(ctrl))
        {
            continue;
        }

        double max_sampled = 0.0;
        for (int i = 0; i < 25; ++i)
        {
            const double u = static_cast<double>(i) / 24.0;
            max_sampled = std::max(
                max_sampled, std::abs(kau::bezier::curvature(ctrl, u)));
        }
        if (max_sampled > kappa_lim_)
        {
            continue;
        }

        Curve candidate_curve(std::vector<Ctrl>{ctrl}, false);
        const RoadReport road = roadReportWheels(
            boundary_, candidate_curve, wheels_, roadMargin(),
            kRoadSampleIntervalCm);
        if (road.min_wheels_on < wheels_.min_wheels_on)
        {
            best_reason = "road_boundary";
            continue;
        }
        const double clear = clearanceRect(
            candidate_curve, obstacles_, body_footprint_, params_.clear_target,
            kRoadSampleIntervalCm);
        if (clear < obsMargin())
        {
            best_reason = "obstacle";
            continue;
        }
        const double exact = candidate_curve.kappaMax();
        if (exact > kappa_lim_)
        {
            continue;
        }

        direct_seed_ = combo;
        const double c_cost = cost(
            params_, kappa_max_vehicle_, global_path_, kappa_lim_, s0, target,
            std::abs(target), exact, clear, candidate_curve, previous_path,
            road.off_integral_cm / params_.l_plan);
        Candidate c; c.d = target; c.curve = candidate_curve; c.cost = c_cost;
        c.reason = "";
        return c;
    }

    // Python 원본도 실패 시 curve 를 끝까지 None 으로 둔다 (best_curve 가
    // 루프 안에서 한 번도 대입되지 않음) -- 그대로 재현.
    Candidate c; c.d = target; c.reason = best_reason; c.cost = kInf;
    return c;
}

std::vector<Candidate> CandidateGenerator::directFamily(
    const Point2 & p, double yaw, double kappa0, const Frame & terminal_frame,
    const std::array<std::array<double, 3>, 7> & offset_pairs,
    std::vector<Candidate> base_candidates, double s0,
    const Curve * previous_path)
{
    const Point2 normal{
        -std::sin(terminal_frame.heading), std::cos(terminal_frame.heading)};
    std::vector<Candidate> family = std::move(base_candidates);

    for (int index : directTargetIndices(s0, offset_pairs))
    {
        const double target = offset_pairs[static_cast<std::size_t>(index)].back();
        const Point2 terminal{
            terminal_frame.point.x + target * normal.x,
            terminal_frame.point.y + target * normal.y};
        family[static_cast<std::size_t>(index)] = directCandidate(
            p, yaw, kappa0, terminal, terminal_frame.heading, target, s0,
            previous_path);
        if (family[static_cast<std::size_t>(index)].reason.empty())
        {
            break;
        }
    }
    return family;
}

}  // namespace local_path_planner
}  // namespace kau
