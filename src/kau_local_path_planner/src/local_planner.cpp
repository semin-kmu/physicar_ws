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
    double kappa_max_vehicle, double body_radius_cm,
    VehicleFootprint body_footprint, WheelFootprint wheels)
: global_path_(std::move(global_path)), boundary_(std::move(boundary)),
  obstacles_(std::move(obstacles)), stations_(), params_(std::move(params)),
  kappa_lim_(kappa_max_vehicle * params_.kappa_margin),
  kappa_max_vehicle_(kappa_max_vehicle), body_radius_cm_(body_radius_cm),
  body_footprint_(body_footprint), wheels_(wheels),
  ref_fusion_(global_path_, params_.ref_mode, params_.w_lane, params_.lane_gate),
  candidate_gen_(
      global_path_, boundary_, obstacles_, stations_, params_, kappa_lim_,
      kappa_max_vehicle_, body_radius_cm_, body_footprint_, wheels_)
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
    // corridor 전용 3-frame (0.25L/0.625L/1.0L, 2026-08-24 KAU_AMET_Test
    // 알고리즘 반영): 긴 2-segment chord 가 도로 굴곡을 한 번에 가로질러
    // road_boundary 를 자주 위반하던 문제를 완화하려고 중간 reference
    // knot 을 하나 더 둔다. obstacle_offsets/primitives/direct_family/K2
    // 는 기존 2-frame `frames` 를 그대로 쓴다 (corridor 만 변경).
    const std::array<Frame, 3> corridor_frames{
        ref_fusion_.evalFrame(
            s0_frames + 0.25 * params_.l_plan, lane_curve, lane_confidence),
        ref_fusion_.evalFrame(
            s0_frames + 0.625 * params_.l_plan, lane_curve, lane_confidence),
        ref_fusion_.evalFrame(
            s0_frames + params_.l_plan, lane_curve, lane_confidence),
    };

    const auto offset_pairs = candidate_gen_.corridorOffsets(corridor_frames);

    std::vector<Candidate> cands;
    cands.reserve(offset_pairs.size() + 2);
    for (const auto & pair : offset_pairs)
    {
        cands.push_back(candidate_gen_.candidate(
            p, yaw, kappa0,
            std::vector<Frame>{corridor_frames[0], corridor_frames[1], corridor_frames[2]},
            std::vector<double>{pair[0], pair[1], pair[2]},
            s0, previous_path_ ? &*previous_path_ : nullptr));
    }
    // Smart K1 (2026-08-24 KAU_AMET_Test 알고리즘 반영): 장애물 회피 시
    // middle knot longitudinal 위치를 장애물 station 기준으로 정한 후보
    // 2개. corridor 후보군에 합류할 뿐 새 candidate family 가 아니다.
    {
        auto obstacle_cands = candidate_gen_.obstacleOffsetCandidates(
            p, yaw, kappa0, frames, s0,
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

    // 2026-08-24 (committed/prediction horizon, KAU_AMET_Test 알고리즘
    // 반영): stage-1 과 동일하게, committed horizon(kCommittedHorizonCm)
    // 안에서는 exact kappa/road 를 그대로 hard 하게 유지하되, 그 이후(먼
    // 미래)에서만의 위반은 즉시 폐기하지 않는다 -- stage-1(candidate())
    // 에서 이미 그런 후보에 cost 페널티를 부여해뒀으므로, 여기서는
    // committed 기준으로만 재검증한다.
    std::optional<std::size_t> chosen;
    for (std::size_t i : alive)
    {
        Candidate & c = cands[i];
        const std::vector<Ctrl> & segs = c.curve->ctrl();
        const int n_committed = committedSegmentCount(segs);
        const bool has_prediction_zone = n_committed < c.curve->nseg();
        const Curve committed_cv = has_prediction_zone
            ? Curve(std::vector<Ctrl>(
                  segs.begin(), segs.begin() + n_committed), false)
            : *c.curve;
        const double kmax = c.curve->kappaMax();   // stage 2, 14차 근 (기록용, 전체)
        const double kmax_committed = has_prediction_zone
            ? committed_cv.kappaMax() : kmax;
        if (kmax_committed > kappa_lim_)
        {
            c.cost = kInf; c.reason = "kappa_exact";
        }
        else if (!c.curve->isRegular())
        {
            c.cost = kInf; c.reason = "degenerate";
        }
        else if (!roadOkWheels(boundary_, committed_cv, wheels_,
                              kRoadSafetyMarginCm, kRoadSampleIntervalCm))
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
        auto degraded = leastViolationRect(
            cands, boundary_, body_footprint_, wheels_, kRoadSafetyMarginCm,
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
    result.path = *c.curve;
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
    const RoadReport full = roadReportWheels(
        boundary_, cv, wheels_, kRoadSafetyMarginCm, kRoadSampleIntervalCm);
    result.min_wheels_on = full.min_wheels_on;
    result.road_clear_full = full.min_clear_cm;
    result.road_viol_s = full.first_viol_s_cm;
    result.road_off_len = full.off_integral_cm;

    // committed 구간만 따로 -- "실행 구간은 멀쩡한데 뒤쪽(무검증 구간) 때문에
    // 나쁜 것인가" 를 로그 한 줄로 가르기 위한 값이다.
    const std::vector<Ctrl> & segs = cv.ctrl();
    const int n_committed = committedSegmentCount(segs);
    if (n_committed < cv.nseg())
    {
        const Curve committed_cv(
            std::vector<Ctrl>(segs.begin(), segs.begin() + n_committed), false);
        result.road_clear_committed = roadReportWheels(
            boundary_, committed_cv, wheels_, kRoadSafetyMarginCm,
            kRoadSampleIntervalCm).min_clear_cm;
    }
    else
    {
        result.road_clear_committed = full.min_clear_cm;
    }
}

}  // namespace local_path_planner
}  // namespace kau
