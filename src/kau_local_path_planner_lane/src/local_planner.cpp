#include "kau_local_path_planner_lane/local_planner.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>

#include "kau_local_path_planner_lane/path_evaluator.hpp"

namespace kau
{
namespace local_path_planner_lane
{

namespace
{
constexpr double kInf = std::numeric_limits<double>::infinity();

// 직전 ego frame 기준 곡선을 이번 ego frame 으로 재정렬 (odom_delta 적용).
// Python: curve.realign / curve.rebase(cv, frm=odom_delta, to=(0,0,0)).
Curve realignCurve(const Curve & cv, const OdomDelta & d)
{
    const double c = std::cos(-d.dyaw);
    const double s = std::sin(-d.dyaw);
    std::vector<kau::bezier::Ctrl> segs;
    segs.reserve(static_cast<std::size_t>(cv.nseg()));
    for (int i = 0; i < cv.nseg(); ++i)
    {
        kau::bezier::Ctrl seg = cv.seg(i);
        for (auto & pt : seg)
        {
            const double x = pt.x - d.dx;
            const double y = pt.y - d.dy;
            pt = Point2{x * c - y * s, x * s + y * c};
        }
        segs.push_back(seg);
    }
    return Curve(std::move(segs), cv.closed());
}

}  // namespace

LocalPlanner::LocalPlanner(
    PlannerParams params, double kappa_max_vehicle, double body_radius_cm,
    VehicleFootprint body_footprint, WheelFootprint wheels)
: params_(std::move(params)),
  kappa_lim_(kappa_max_vehicle * params_.kappa_margin),
  kappa_max_vehicle_(kappa_max_vehicle), body_radius_cm_(body_radius_cm),
  body_footprint_(body_footprint), wheels_(wheels),
  global_path_(std::vector<kau::bezier::Ctrl>{}, false),
  boundary_(), obstacles_(), stations_(),
  candidate_gen_(
      global_path_, boundary_, obstacles_, stations_, params_, kappa_lim_,
      kappa_max_vehicle_, body_radius_cm_, body_footprint_, wheels_)
{
}

void LocalPlanner::updateObstacles(std::vector<Obstacle> obstacles)
{
    obstacles_ = std::move(obstacles);
    // stations_ 는 backbone 이 매 사이클 바뀌므로 plan() 안에서 매번 다시 만든다.
}

LocalPlanner::Anchor LocalPlanner::computeAnchor(
    double kappa_ref, double now_sec) const
{
    Anchor a;
    a.p = Point2{0.0, 0.0};   // ego 는 항상 자기 frame 원점
    a.yaw = 0.0;
    a.kappa = std::clamp(kappa_ref, -kappa_lim_, kappa_lim_);

    const double age = now_sec - prev_stamp_sec_;
    const bool fresh = previous_path_.has_value() &&
        age >= 0.0 && age <= params_.anchor_max_age_sec;
    if (!fresh)
    {
        return a;
    }

    const kau::control::TrackState st = previous_path_->nearestGlobal(a.p);
    if (!st.valid)
    {
        return a;
    }
    a.e = st.dist;

    const double lo = params_.anchor_blend_lo_cm;
    const double hi = params_.anchor_blend_hi_cm;
    a.alpha = (hi > lo) ? std::clamp((a.e - lo) / (hi - lo), 0.0, 1.0)
                        : ((a.e <= lo) ? 0.0 : 1.0);

    const Frame pf = referenceFrame(*previous_path_, previous_path_->wrapS(st.s));

    a.p = Point2{
        pf.point.x + a.alpha * (0.0 - pf.point.x),
        pf.point.y + a.alpha * (0.0 - pf.point.y)};
    a.yaw = pf.heading + a.alpha * kau::control::wrapPi(0.0 - pf.heading);
    a.kappa = std::clamp(
        (1.0 - a.alpha) * pf.kappa + a.alpha * kappa_ref, -kappa_lim_, kappa_lim_);
    a.margin = (1.0 - a.alpha) * a.e;
    return a;
}

PlanResult LocalPlanner::plan(
    const Curve * left, const Curve * right, const Curve * center,
    const std::optional<OdomDelta> & odom_delta, double now_sec)
{
    const auto t0 = std::chrono::steady_clock::now();

    // 직전 경로/장애물을 이번 ego frame 으로 재정렬 (hold-last/continuity/
    // 앵커 전부 이 위에서 동작한다). odom_delta 없으면 무보정(재계획 주기가
    // 짧다는 근사).
    if (previous_path_.has_value() && odom_delta.has_value())
    {
        previous_path_ = realignCurve(*previous_path_, *odom_delta);
    }

    const BackboneResult bb = buildBackbone(
        left, right, center,
        // kappa0 (근접 사각지대 브릿지용) 는 이전 경로/anchor 값으로 근사.
        // 실측 자세를 못 쓰므로(폐루프 방지, 옛 패키지와 동일 원칙)
        // 이전 사이클 anchor kappa 를 그대로 재사용한다.
        previous_path_.has_value()
            ? referenceFrame(*previous_path_, 0.0).kappa : 0.0,
        params_.l_plan, params_.bound_depth);

    const double calc_ms0 = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - t0).count();

    PlanResult result;
    if (!bb.backbone.has_value())
    {
        // 양쪽 edge 모두 무효(hold 만료 등) -- 직전 committed path 를 그대로
        // 이어 발행한다(Python 최후수단과 동일).
        result.calc_ms = calc_ms0;
        if (previous_path_.has_value())
        {
            result.path = *previous_path_;
            result.status = PlanStatus::kDegraded;
            result.kappa_max = previous_path_->kappaMax();
            result.valid_length_cm = validatedHorizon(*previous_path_);
        }
        else
        {
            result.status = PlanStatus::kNoFeasible;
        }
        return result;
    }

    global_path_ = *bb.backbone;
    boundary_.left = bb.left;
    boundary_.right = bb.right;

    stations_.clear();
    stations_.reserve(obstacles_.size());
    for (const Obstacle & o : obstacles_)
    {
        stations_.push_back(makeStation(global_path_, o));
    }

    const double kappa_ref = referenceFrame(global_path_, 0.0).kappa;
    const Anchor anchor = computeAnchor(kappa_ref, now_sec);
    const Point2 p = anchor.p;
    const double yaw0 = anchor.yaw;
    const double kappa0 = anchor.kappa;
    const double s0 = 0.0;   // backbone 이 항상 ego 원점에서 시작 (구 _project 대체)

    anchor_margin_cm_ = anchor.margin;
    candidate_gen_.setAnchorOffset(anchor.margin);

    std::array<Frame, 2> frames{
        referenceFrame(global_path_, 0.6 * params_.l_plan),
        referenceFrame(global_path_, params_.l_plan),
    };
    const std::array<Frame, 3> corridor_frames{
        referenceFrame(global_path_, 0.6 * params_.l_plan),
        referenceFrame(global_path_, 0.8 * params_.l_plan),
        referenceFrame(global_path_, params_.l_plan),
    };

    const auto offset_pairs = candidate_gen_.corridorOffsets(corridor_frames);

    std::vector<Candidate> cands;
    cands.reserve(offset_pairs.size() + 2);
    for (const auto & pair : offset_pairs)
    {
        cands.push_back(candidate_gen_.candidate(
            p, yaw0, kappa0,
            std::vector<Frame>{corridor_frames[0], corridor_frames[1], corridor_frames[2]},
            std::vector<double>{pair[0], pair[1], pair[2]},
            s0, previous_path_ ? &*previous_path_ : nullptr));
    }
    {
        auto obstacle_cands = candidate_gen_.obstacleOffsetCandidates(
            p, yaw0, kappa0, frames, s0,
            previous_path_ ? &*previous_path_ : nullptr);
        cands.insert(cands.end(), obstacle_cands.begin(), obstacle_cands.end());
    }

    const auto aliveOf = [](const std::vector<Candidate> & v)
    {
        std::vector<std::size_t> out;
        for (std::size_t i = 0; i < v.size(); ++i)
        {
            if (v[i].reason.empty())
            {
                out.push_back(i);
            }
        }
        return out;
    };

    std::vector<std::size_t> alive = aliveOf(cands);
    if (alive.empty())
    {
        std::vector<Candidate> primitives = candidate_gen_.obstaclePrimitives(
            p, yaw0, kappa0, frames.back(), s0,
            previous_path_ ? &*previous_path_ : nullptr);
        cands.insert(cands.end(), primitives.begin(), primitives.end());
        alive = aliveOf(cands);
    }
    if (alive.empty())
    {
        cands = candidate_gen_.directFamily(
            p, yaw0, kappa0, frames.back(), offset_pairs, std::move(cands), s0,
            previous_path_ ? &*previous_path_ : nullptr);
        alive = aliveOf(cands);
    }

    std::sort(alive.begin(), alive.end(),
             [&cands](std::size_t a, std::size_t b)
             { return cands[a].cost < cands[b].cost; });

    std::optional<std::size_t> chosen;
    for (std::size_t i : alive)
    {
        Candidate & c = cands[i];
        const double horizon = validatedHorizon(*c.curve);
        if (c.curve->kappaMaxOver(0.0, horizon) > kappa_lim_)
        {
            c.cost = kInf; c.reason = "kappa_exact";
        }
        else if (!c.curve->isRegular())
        {
            c.cost = kInf; c.reason = "degenerate";
        }
        else if (!roadOkWheels(boundary_, *c.curve, wheels_,
                              roadMargin(), kRoadSampleIntervalCm, horizon))
        {
            c.cost = kInf; c.reason = "road_boundary";
        }
        else
        {
            chosen = i;
            break;
        }
    }

    const double calc_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - t0).count();

    result.s0 = s0;
    result.calc_ms = calc_ms;
    result.lane_used = true;
    result.anchor_e_cm = anchor.e;
    result.anchor_alpha = anchor.alpha;
    result.anchor_margin_cm = anchor.margin;
    result.kappa0 = kappa0;

    if (!chosen)
    {
        auto degraded = leastViolationRect(
            cands, boundary_, body_footprint_, wheels_, roadMargin(),
            kRoadSampleIntervalCm, obstacles_, obsMargin(),
            params_.clear_target, kappa_max_vehicle_);
        if (degraded)
        {
            const auto & [d, cv] = *degraded;
            previous_path_ = cv;
            prev_stamp_sec_ = now_sec;
            result.path = cv;
            result.valid_length_cm = validatedHorizon(cv);
            result.status = PlanStatus::kDegraded;
            result.chosen_offset = d;
            result.kappa_max = cv.kappaMax();
            result.clearance = clearanceRect(
                cv, obstacles_, body_footprint_, params_.clear_target,
                kRoadSampleIntervalCm);
            result.alive = static_cast<int>(alive.size());
            fillRoadDiag(result, cv);
            return result;
        }

        bool only_cusp_or_degenerate = true;
        for (const Candidate & c : cands)
        {
            if (c.reason != "cusp" && c.reason != "degenerate")
            {
                only_cusp_or_degenerate = false;
                break;
            }
        }
        result.status = only_cusp_or_degenerate
            ? PlanStatus::kDegenerate : PlanStatus::kNoFeasible;
        return result;
    }

    const Candidate & c = cands[*chosen];
    previous_path_ = *c.curve;
    prev_stamp_sec_ = now_sec;
    result.path = *c.curve;
    result.valid_length_cm = validatedHorizon(*c.curve);
    result.status = PlanStatus::kOk;
    result.chosen_offset = c.d;
    result.kappa_max = c.curve->kappaMax();
    result.clearance = clearanceRect(
        *c.curve, obstacles_, body_footprint_, params_.clear_target,
        kRoadSampleIntervalCm);
    result.alive = static_cast<int>(alive.size());
    fillRoadDiag(result, *c.curve);
    return result;
}

void LocalPlanner::fillRoadDiag(PlanResult & result, const Curve & cv) const
{
    // 전 구간 스캔 한 번으로 committed 구간 최솟값까지 같이 받는다. 예전엔
    // 같은 스캔을 horizon 으로 끊어 한 번 더 돌았는데, 그건 전 구간 스캔의
    // 접두부를 그대로 다시 계산하는 것이라 값이 같으면서 (스테이션당 곡선
    // 최근접 투영 16 회) 전 구간 판정 한 번을 통째로 더 쓰는 셈이었다.
    const RoadReport full = roadReportWheels(
        boundary_, cv, wheels_, kRoadSafetyMarginCm, kRoadSampleIntervalCm,
        kInf, validatedHorizon(cv));
    result.min_wheels_on = full.min_wheels_on;
    result.road_clear_full = full.min_clear_cm;
    result.road_viol_s = full.first_viol_s_cm;
    result.road_off_len = full.off_integral_cm;
    result.road_clear_committed = full.min_clear_upto_cm;
}

double LocalPlanner::validatedHorizon(const Curve & cv) const
{
    const double h = params_.validated_horizon_cm;
    const double len = cv.length();
    if (!(h > 0.0))
    {
        return len;
    }
    return std::min(h, len);
}

}  // namespace local_path_planner_lane
}  // namespace kau
