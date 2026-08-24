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

    // x/y/yaw 는 TF 실측 자세 (cm, rad, map frame).
    // lane_curve/lane_confidence 는 이번 사이클 lane detection 관측
    // (없으면 lane_curve=nullptr).
    // now_sec 은 이전 경로 나이 판정에만 쓴다 (이 클래스를 ROS-free 로 두려고
    // 시계를 주입받는다). 단조 증가하기만 하면 어떤 기준시각이어도 된다.
    //
    // 2026-08-25: kappa0 인자가 사라졌다. 예전에는 노드가 /steering(제어기
    // 출력)에서 tan(delta)/L 로 만들어 넘겼는데, 그러면 (a) 플래너 입력이
    // 자기 출력의 함수가 되어 폐루프가 되고, (b) Pure Pursuit 의 delta 는
    // 자차->lookahead 현의 곡률이라 경로 곡률이 아니며, (c) delta 가 +-20deg
    // 로 clamp 되므로 kappa0 가 곡률 상한 자체를 건드린다. 특히 (c) 는
    // kappa(0) == kappa0 가 정확히 성립하는 탓에 치명적이다 -- 조향이
    // 19.07deg(= kappa_lim 에 해당) 를 넘는 순간 **모든 후보**가
    // kappa_bound/kappa_exact 로 탈락한다. 이제 plan() 이 이전 경로에서
    // 직접 구한다 (computeAnchor).
    PlanResult plan(
        double x, double y, double yaw,
        const Curve * lane_curve, float lane_confidence, double now_sec);

private:
    // P0 로 쓸 상태. 실측 자세와 이전 경로 위 최근접점을 alpha 로 섞은 것.
    struct Anchor
    {
        Point2 p;                 // 위치
        double yaw    = 0.0;      // 방위
        double kappa  = 0.0;      // 곡률 (clamp 후)
        double e      = 0.0;      // cm, 이전 경로에서 자차까지 횡거리
        double alpha  = 1.0;      // 0=이전 경로, 1=실측
        double margin = 0.0;      // cm, (1-alpha)*e -- 검사 마진에 더할 값
    };

    // kappa_ref 는 이전 경로를 못 쓸 때의 곡률 폴백 (참조 곡선의 s0 곡률).
    // ref_fusion_.project() 이후에 호출할 것.
    Anchor computeAnchor(
        const Point2 & p, double yaw, double kappa_ref, double now_sec) const;

    // 앵커 오프셋을 반영한 실효 마진 (CandidateGenerator 쪽과 같은 규약).
    double roadMargin() const { return kRoadSafetyMarginCm + anchor_margin_cm_; }
    double obsMargin() const { return params_.obs_margin + anchor_margin_cm_; }

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
    double prev_stamp_sec_ = 0.0;          // previous_path_ 를 채운 시각 (나이 판정)
    double anchor_margin_cm_ = 0.0;        // 이번 틱 마진 보정 (roadMargin/obsMargin)
};

}  // namespace local_path_planner
}  // namespace kau

#endif  // KAU_LOCAL_PATH_PLANNER__LOCAL_PLANNER_HPP_
