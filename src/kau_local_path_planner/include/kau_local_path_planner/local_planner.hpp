// ====================================================================
// local_planner.hpp
//
// 최종 오케스트레이션. 원본: KAU_AMET_Test /
// src/kau_local_path_planner/test/planner.py 의 LocalPlanner.plan()
//
// ROS 의존성(구독/발행/TF/파라미터)은 local_planner_node.cpp 가 맡고,
// 이 클래스는 순수 알고리즘만 담아 노드 없이도 테스트 가능하게 한다.
// ====================================================================

#ifndef KAU_LOCAL_PATH_PLANNER__LOCAL_PLANNER_HPP_
#define KAU_LOCAL_PATH_PLANNER__LOCAL_PLANNER_HPP_

#include <optional>
#include <vector>

#include "kau_control/curve.hpp"
#include "kau_local_path_planner/boundary_checker.hpp"
#include "kau_local_path_planner/candidate_generator.hpp"
#include "kau_local_path_planner/collision_checker.hpp"
#include "kau_local_path_planner/reference_fusion.hpp"
#include "kau_local_path_planner/types.hpp"

namespace kau
{
namespace local_path_planner
{

using kau::control::Curve;

class LocalPlanner
{
public:
    // kappa_max_vehicle: config.VEHICLE.kappa_max (물리 상한, margin 미적용).
    // kappa_lim = kappa_max_vehicle * params.kappa_margin 은 생성자가 계산.
    LocalPlanner(
        Curve global_path, RoadBoundary boundary,
        std::vector<Obstacle> obstacles, PlannerParams params,
        double kappa_max_vehicle, double body_radius_cm,
        VehicleFootprint body_footprint = VehicleFootprint{},
        WheelFootprint wheels = WheelFootprint{});

    // ref_fusion_/candidate_gen_ 이 이 객체 자신의 멤버(global_path_ 등)를
    // 참조로 들고 있으므로, 복사/이동하면 그 참조가 원본을 계속 가리켜
    // 댕글링된다. std::unique_ptr<LocalPlanner> 로만 다룰 것.
    LocalPlanner(const LocalPlanner &) = delete;
    LocalPlanner & operator=(const LocalPlanner &) = delete;
    LocalPlanner(LocalPlanner &&) = delete;
    LocalPlanner & operator=(LocalPlanner &&) = delete;

    // 실제 ROS2 환경에서는 /perception/obstacles 가 매 주기 바뀌는 라이브
    // 토픽이다 (Python 시뮬의 ground-truth 고정 장애물과 다른 지점 --
    // KAU_AMET_ROS 세션에서 추가). plan() 호출 전에 이번 사이클 관측으로
    // 갱신한다. stations 도 함께 재계산.
    void updateObstacles(std::vector<Obstacle> obstacles);

    // lane_curve/lane_confidence 는 이번 사이클 lane detection 관측
    // (없으면 lane_curve=nullptr). Python: LocalPlanner.plan(x,y,yaw,kappa0,lane_result).
    PlanResult plan(
        double x, double y, double yaw, double kappa0,
        const Curve * lane_curve, float lane_confidence);

private:
    // 채택된 경로의 도로 이탈 진단을 PlanResult 에 채운다 (전 구간 / committed
    // 구간 / 처음 나간 호길이). status 만으로는 "어디서부터 나갔는지" 를 알 수
    // 없어서 튜닝 근거가 없었다.
    void fillRoadDiag(PlanResult & result, const Curve & cv) const;

    Curve global_path_;
    RoadBoundary boundary_;
    std::vector<Obstacle> obstacles_;
    std::vector<ObstacleStation> stations_;
    PlannerParams params_;
    double kappa_lim_;
    double kappa_max_vehicle_;
    double body_radius_cm_;
    VehicleFootprint body_footprint_;   // 장애물 충돌 판정 (범퍼 포함)
    WheelFootprint   wheels_;           // 도로 이탈 판정 (바퀴 4개)

    ReferenceFusion ref_fusion_;
    CandidateGenerator candidate_gen_;

    std::optional<Curve> previous_path_;   // Python: self._previous_path
};

}  // namespace local_path_planner
}  // namespace kau

#endif  // KAU_LOCAL_PATH_PLANNER__LOCAL_PLANNER_HPP_
