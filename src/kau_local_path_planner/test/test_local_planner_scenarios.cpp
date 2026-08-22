// 통합 시나리오 테스트. KAU_AMET_Test 세션의 full_simulation.py 검증
// 방식(0.05/0.30/1.0 lap 시나리오 돌려 plan_status 분포/min_clear 확인)을
// 축소해, "열린 도로", "정면 장애물" 두 시나리오에서 plan() 이 크래시
// 없이 합리적인 결과를 내는지만 확인한다. 실제 트랙 데이터 기반 회귀는
// 대회 시뮬레이터 통합 이후 별도로 추가할 것.

#include <gtest/gtest.h>

#include "kau_local_path_planner/bezier_ext.hpp"
#include "kau_local_path_planner/local_planner.hpp"

using kau::control::Curve;
using kau::local_path_planner::hermiteToBezier;
using kau::local_path_planner::LocalPlanner;
using kau::local_path_planner::Obstacle;
using kau::local_path_planner::PlannerParams;
using kau::local_path_planner::PlanStatus;
using kau::local_path_planner::Point2;
using kau::local_path_planner::RoadBoundary;

namespace
{

RoadBoundary wideOpenBoundary()
{
    RoadBoundary rb;
    rb.outer = {{-100.0, -1000.0}, {2100.0, -1000.0}, {2100.0, 1000.0},
               {-100.0, 1000.0}};
    rb.inner = {{-10000.0, -10000.0}, {-9999.0, -10000.0}, {-9999.0, -9999.0}};
    return rb;
}

Curve straightGlobalPath(double length)
{
    const auto ctrl = hermiteToBezier(
        Point2{0.0, 0.0}, 0.0, 0.0, Point2{length, 0.0}, 0.0, 0.0,
        length, length);
    return Curve(std::vector<decltype(ctrl)>{ctrl}, false);
}

PlannerParams tunedParams()
{
    PlannerParams p;
    p.l_plan = 300.0;
    p.preview = 450.0;
    p.w_continuity = 10.0;
    return p;
}

}  // namespace

TEST(LocalPlannerScenarios, OpenStraightRoadReachesOkStatus)
{
    LocalPlanner planner(
        straightGlobalPath(2000.0), wideOpenBoundary(), {}, tunedParams(),
        /*kappa_max_vehicle=*/0.020221, /*body_radius_cm=*/11.0);

    const auto result = planner.plan(
        0.0, 0.0, 0.0, 0.0, /*lane_curve=*/nullptr, /*lane_confidence=*/0.0f);

    EXPECT_EQ(result.status, PlanStatus::kOk);
    ASSERT_TRUE(result.path.has_value());
    EXPECT_GT(result.clearance, 0.0);
}

TEST(LocalPlannerScenarios, HeadOnObstacleProducesNonCrashingResult)
{
    // 차량 정면 150cm, 도로 중앙에 반경 10cm 장애물 -- 반드시 옆으로
    // 피해야 한다. 최소한 크래시 없이 path 를 내거나(회피 성공), 정직하게
    // degraded/no_feasible 로 보고해야 한다 (조용히 장애물을 무시하면 안 됨).
    LocalPlanner planner(
        straightGlobalPath(2000.0), wideOpenBoundary(),
        {Obstacle{Point2{150.0, 0.0}, 10.0}}, tunedParams(),
        /*kappa_max_vehicle=*/0.020221, /*body_radius_cm=*/11.0);

    const auto result = planner.plan(
        0.0, 0.0, 0.0, 0.0, /*lane_curve=*/nullptr, /*lane_confidence=*/0.0f);

    if (result.status == PlanStatus::kOk)
    {
        // 회피에 성공했다면 실제로 옆으로 비켜났어야 한다.
        EXPECT_GT(std::abs(result.chosen_offset), 5.0);
        EXPECT_GE(result.clearance, 0.0);
    }
    else
    {
        EXPECT_TRUE(
            result.status == PlanStatus::kDegraded ||
            result.status == PlanStatus::kNoFeasible);
    }
}
