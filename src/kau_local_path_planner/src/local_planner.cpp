#include "kau_local_path_planner/local_planner.hpp"

#include <algorithm>
#include <chrono>
#include <limits>
#include <numeric>

#include "kau_local_path_planner/path_evaluator.hpp"

namespace kau
{
namespace local_path_planner
{

namespace
{
constexpr double kInf = std::numeric_limits<double>::infinity();
}

LocalPlanner::LocalPlanner(
    Curve global_path, RoadBoundary boundary,
    std::vector<Obstacle> obstacles, PlannerParams params,
    double kappa_max_vehicle, double body_radius_cm)
: global_path_(std::move(global_path)), boundary_(std::move(boundary)),
  obstacles_(std::move(obstacles)), stations_(), params_(std::move(params)),
  kappa_lim_(kappa_max_vehicle * params_.kappa_margin),
  kappa_max_vehicle_(kappa_max_vehicle), body_radius_cm_(body_radius_cm),
  ref_fusion_(global_path_, params_.ref_mode, params_.w_lane, params_.lane_gate),
  candidate_gen_(
      global_path_, boundary_, obstacles_, stations_, params_, kappa_lim_,
      kappa_max_vehicle_, body_radius_cm_)
{
    stations_.reserve(obstacles_.size());
    for (const Obstacle & o : obstacles_)
    {
        stations_.push_back(makeStation(global_path_, o));
    }
}

void LocalPlanner::updateObstacles(std::vector<Obstacle> obstacles)
{
    obstacles_ = std::move(obstacles);
    stations_.clear();
    stations_.reserve(obstacles_.size());
    for (const Obstacle & o : obstacles_)
    {
        stations_.push_back(makeStation(global_path_, o));
    }
    // candidate_gen_ 은 obstacles_/stations_ 를 참조로 들고 있으므로 (같은
    // 벡터 객체를 그대로 갱신) 별도 재구성 없이 다음 plan() 부터 반영된다.
}

PlanResult LocalPlanner::plan(
    double x, double y, double yaw, double kappa0,
    const Curve * lane_curve, float lane_confidence)
{
    const auto t0 = std::chrono::steady_clock::now();
    const Point2 p{x, y};

    ref_fusion_.project(p, lane_curve);
    const double s0_frames = ref_fusion_.projectedS();   // Python: s0 = state.s
    const double s0 = ref_fusion_.s0();                  // Python: self._s0

    std::array<Frame, 2> frames{
        ref_fusion_.evalFrame(
            s0_frames + 0.25 * params_.l_plan, lane_curve, lane_confidence),
        ref_fusion_.evalFrame(
            s0_frames + params_.l_plan, lane_curve, lane_confidence),
    };

    const auto offset_pairs = candidate_gen_.corridorOffsets(frames);

    std::vector<Candidate> cands;
    cands.reserve(offset_pairs.size());
    for (const auto & pair : offset_pairs)
    {
        cands.push_back(candidate_gen_.candidate(
            p, yaw, kappa0, frames, pair, s0, previous_path_ ? &*previous_path_ : nullptr));
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
            p, yaw, kappa0, frames.back(), s0,
            previous_path_ ? &*previous_path_ : nullptr);
        cands.insert(cands.end(), primitives.begin(), primitives.end());
        alive = aliveOf(cands);
    }
    if (alive.empty())
    {
        cands = candidate_gen_.directFamily(
            p, yaw, kappa0, frames.back(), offset_pairs, std::move(cands), s0,
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
        const double kmax = c.curve->kappaMax();   // stage 2, 14차 근
        if (kmax > kappa_lim_)
        {
            c.cost = kInf; c.reason = "kappa_exact";
        }
        else if (!c.curve->isRegular())
        {
            c.cost = kInf; c.reason = "degenerate";
        }
        else if (!roadOk(boundary_, *c.curve, body_radius_cm_ + kRoadSafetyMarginCm,
                         kRoadSampleIntervalCm))
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

    PlanResult result;
    result.s0 = s0_frames;
    result.calc_ms = calc_ms;
    result.lane_used = ref_fusion_.laneUsed();

    if (!chosen)
    {
        auto degraded = leastViolation(
            cands, boundary_, body_radius_cm_, kRoadSafetyMarginCm,
            kRoadSampleIntervalCm, obstacles_, params_.obs_margin,
            params_.clear_target, kappa_max_vehicle_);
        if (degraded)
        {
            const auto & [d, cv] = *degraded;
            previous_path_ = cv;
            result.path = cv;
            result.status = PlanStatus::kDegraded;
            result.chosen_offset = d;
            result.kappa_max = cv.kappaMax();
            result.clearance = clearance(
                cv, obstacles_, body_radius_cm_, params_.clear_target);
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
    result.path = *c.curve;
    result.status = PlanStatus::kOk;
    result.chosen_offset = c.d;
    result.kappa_max = c.curve->kappaMax();
    result.clearance = clearance(
        *c.curve, obstacles_, body_radius_cm_, params_.clear_target);
    result.alive = static_cast<int>(alive.size());
    return result;
}

}  // namespace local_path_planner
}  // namespace kau
