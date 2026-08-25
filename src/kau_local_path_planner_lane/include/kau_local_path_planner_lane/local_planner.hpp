// ====================================================================
// local_planner.hpp (lane-only 재설계, 2026-08-25)
//
// map/global path/localization 배제. backbone 을 매 사이클 /lane/left,
// /lane/right, /lane/center 만으로 새로 만든다 (lane_backbone.hpp).
// candidate 생성/도로/장애물/비용 로직은 옛 패키지(kau_local_path_planner)
// 것을 최대한 그대로 재사용한다.
// ====================================================================

#ifndef KAU_LOCAL_PATH_PLANNER_LANE__LOCAL_PLANNER_HPP_
#define KAU_LOCAL_PATH_PLANNER_LANE__LOCAL_PLANNER_HPP_

#include <array>
#include <optional>
#include <vector>

#include "kau_control/curve.hpp"
#include "kau_local_path_planner_lane/boundary_checker.hpp"
#include "kau_local_path_planner_lane/candidate_generator.hpp"
#include "kau_local_path_planner_lane/collision_checker.hpp"
#include "kau_local_path_planner_lane/lane_backbone.hpp"
#include "kau_local_path_planner_lane/types.hpp"

namespace kau
{
namespace local_path_planner_lane
{

using kau::control::Curve;

// 직전 plan() 호출 이후 상대 오도메트리. (dx, dy, dyaw) -- 이번 ego pose 를
// 직전 ego frame 기준으로 표현한 값 (map 아님, 실제로는 /odometry 등에서
// 뽑는다). previous_path_ 재정렬(hold-last/continuity/앵커)에만 쓰인다.
struct OdomDelta
{
    double dx = 0.0;
    double dy = 0.0;
    double dyaw = 0.0;
};

class LocalPlanner
{
public:
    LocalPlanner(
        PlannerParams params, double kappa_max_vehicle, double body_radius_cm,
        VehicleFootprint body_footprint = VehicleFootprint{},
        WheelFootprint wheels = WheelFootprint{});

    LocalPlanner(const LocalPlanner &) = delete;
    LocalPlanner & operator=(const LocalPlanner &) = delete;
    LocalPlanner(LocalPlanner &&) = delete;
    LocalPlanner & operator=(LocalPlanner &&) = delete;

    // obstacles 는 base_link 상대 Cartesian, m->cm 변환은 호출측(node)이 한다.
    void updateObstacles(std::vector<Obstacle> obstacles);

    // left/right/center: 이번 사이클 /lane/left,/lane/right,/lane/center
    // (KauPath -> Curve, base_link 상대), 없으면 nullptr. kappa0 는 현재
    // 곡률(이전 경로/참조에서 유도 -- 옛 패키지와 동일하게 조향 출력을
    // 직접 참조하지 않아 폐루프를 피한다, computeAnchor 참고).
    // odom_delta 는 직전 plan() 호출 이후 상대 오도메트리 (없으면 nullopt
    // -> 무보정, 재계획 주기가 짧다는 근사).
    PlanResult plan(
        const Curve * left, const Curve * right, const Curve * center,
        const std::optional<OdomDelta> & odom_delta, double now_sec);

private:
    struct Anchor
    {
        Point2 p;
        double yaw    = 0.0;
        double kappa  = 0.0;
        double e      = 0.0;
        double alpha  = 1.0;
        double margin = 0.0;
    };

    Anchor computeAnchor(double kappa_ref, double now_sec) const;

    double roadMargin() const { return kRoadSafetyMarginCm + anchor_margin_cm_; }
    double obsMargin() const { return params_.obs_margin + anchor_margin_cm_; }

    void fillRoadDiag(PlanResult & result, const Curve & cv) const;
    double validatedHorizon(const Curve & cv) const;

    PlannerParams params_;
    double kappa_lim_;
    double kappa_max_vehicle_;
    double body_radius_cm_;
    VehicleFootprint body_footprint_;
    WheelFootprint   wheels_;

    // 이번 사이클 backbone/boundary. 매 plan() 호출마다 내용을 재대입한다
    // (Python `self.gp = backbone` 재대입과 동일 원리) -- 주소는 고정이라
    // candidate_gen_ 이 들고 있는 포인터는 항상 유효하다.
    Curve global_path_;
    RoadBoundary boundary_;
    std::vector<Obstacle> obstacles_;
    std::vector<ObstacleStation> stations_;

    CandidateGenerator candidate_gen_;

    std::optional<Curve> previous_path_;
    double prev_stamp_sec_ = 0.0;
    double anchor_margin_cm_ = 0.0;
};

}  // namespace local_path_planner_lane
}  // namespace kau

#endif  // KAU_LOCAL_PATH_PLANNER_LANE__LOCAL_PLANNER_HPP_
