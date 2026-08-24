#include <gtest/gtest.h>

#include "kau_local_path_planner/bezier_ext.hpp"
#include "kau_local_path_planner/path_evaluator.hpp"

using kau::control::Curve;
using kau::local_path_planner::Candidate;
using kau::local_path_planner::continuityCost;
using kau::local_path_planner::cost;
using kau::local_path_planner::hermiteToBezier;
using kau::local_path_planner::leastViolation;
using kau::local_path_planner::Obstacle;
using kau::local_path_planner::PlannerParams;
using kau::local_path_planner::Point2;
using kau::local_path_planner::pathPreviewCost;
using kau::local_path_planner::RoadBoundary;

namespace
{

Curve straightLine(double x0, double x1)
{
    auto ctrl = hermiteToBezier(
        Point2{x0, 0.0}, 0.0, 0.0, Point2{x1, 0.0}, 0.0, 0.0,
        x1 - x0, x1 - x0);
    return Curve(std::vector<decltype(ctrl)>{ctrl}, false);
}

Curve straightLineAtY(double x0, double x1, double y)
{
    auto ctrl = hermiteToBezier(
        Point2{x0, y}, 0.0, 0.0, Point2{x1, y}, 0.0, 0.0, x1 - x0, x1 - x0);
    return Curve(std::vector<decltype(ctrl)>{ctrl}, false);
}

}  // namespace

TEST(PathEvaluator, ContinuityCostZeroWithoutPreviousPath)
{
    const Curve candidate = straightLine(0.0, 100.0);
    EXPECT_DOUBLE_EQ(continuityCost(candidate, nullptr, 18.0), 0.0);
}

TEST(PathEvaluator, ContinuityCostZeroWhenIdenticalToPrevious)
{
    const Curve candidate = straightLine(0.0, 100.0);
    const Curve previous = straightLine(0.0, 100.0);
    EXPECT_NEAR(continuityCost(candidate, &previous, 18.0), 0.0, 1e-9);
}

TEST(PathEvaluator, ContinuityCostPositiveWhenOffsetFromPrevious)
{
    // candidate 는 previous 보다 y=+20cm 만큼 평행이동된 직선 -- 형상은
    // 같지만 위치가 달라야 하므로 continuity cost 는 0보다 커야 한다.
    auto ctrl_prev = hermiteToBezier(
        Point2{0.0, 0.0}, 0.0, 0.0, Point2{100.0, 0.0}, 0.0, 0.0, 100.0, 100.0);
    auto ctrl_cand = hermiteToBezier(
        Point2{0.0, 20.0}, 0.0, 0.0, Point2{100.0, 20.0}, 0.0, 0.0, 100.0, 100.0);
    const Curve previous(std::vector<decltype(ctrl_prev)>{ctrl_prev}, false);
    const Curve candidate(std::vector<decltype(ctrl_cand)>{ctrl_cand}, false);

    EXPECT_GT(continuityCost(candidate, &previous, 18.0), 0.0);
}

TEST(PathEvaluator, PathPreviewCostZeroWhenPreviewDisabled)
{
    const Curve global_path = straightLine(0.0, 1000.0);
    EXPECT_DOUBLE_EQ(
        pathPreviewCost(global_path, 20.0, 0.80, 0.0, 300.0, 0.0, 0.02), 0.0);
}

TEST(PathEvaluator, PathPreviewCostZeroOnStraightRoad)
{
    // 곡률 0 인 직선 도로에서는 preview 구간 곡률도 0 -> cost 0.
    const Curve global_path = straightLine(0.0, 1000.0);
    EXPECT_DOUBLE_EQ(
        pathPreviewCost(global_path, 20.0, 0.80, 0.0, 300.0, 450.0, 0.02), 0.0);
}

TEST(PathEvaluator, CostIsFiniteForReasonableCandidate)
{
    const Curve global_path = straightLine(0.0, 1000.0);
    const Curve candidate = straightLine(0.0, 300.0);
    const PlannerParams p;
    const double c = cost(
        p, /*kappa_max_vehicle=*/0.020221, global_path,
        /*kappa_lim=*/0.020221 * 0.95, /*s0=*/0.0, /*d=*/0.0, /*peak=*/0.0,
        /*bound=*/0.0, /*clear=*/999.0, candidate, nullptr);
    EXPECT_TRUE(std::isfinite(c));
    EXPECT_GE(c, 0.0);
}

// 2026-08-24 KAU_AMET_Test 세션 결정 반영: leastViolation 은 세 위반을
// 동일 가중치로 합산하지 않고, obstacle_violation 최소인 후보군으로 먼저
// 좁힌 뒤 그 안에서만 road+kappa 위반 최소를 고른다 -- 장애물 충돌을
// 항상 최우선으로 회피 (실측: 그렇지 않으면 obs_margin 을 키울수록
// 오히려 collision 이 잦아지는 역효과가 있었다).
TEST(PathEvaluator, LeastViolationPrioritizesObstacleOverRoad)
{
    // X: 장애물에서 멀어 obstacle_violation=0, 대신 road boundary 밖 ->
    //    road_violation 큼.
    // Y: road boundary 안(road_violation=0), 대신 장애물과 겹쳐
    //    obstacle_violation 큼.
    // 장애물 우선 규칙이면 road_violation 이 훨씬 큰데도 X 가 선택돼야 한다.
    const Curve x_curve = straightLineAtY(0.0, 300.0, 40.0);
    const Curve y_curve = straightLineAtY(0.0, 300.0, 0.0);

    std::vector<Candidate> cands(2);
    cands[0].d = 1.0; cands[0].curve = x_curve; cands[0].reason = "road_boundary";
    cands[1].d = 2.0; cands[1].curve = y_curve; cands[1].reason = "obstacle";

    RoadBoundary boundary;
    boundary.outer = {{-100.0, -10.0}, {1100.0, -10.0}, {1100.0, 10.0}, {-100.0, 10.0}};
    boundary.inner = {{-10000.0, -10000.0}, {-9999.0, -10000.0}, {-9999.0, -9999.0}};

    const std::vector<Obstacle> obstacles{Obstacle{Point2{150.0, 0.0}, 5.0}};

    const auto result = leastViolation(
        cands, boundary, /*body_radius_cm=*/11.0,
        /*road_safety_margin_cm=*/2.0, /*road_sample_interval_cm=*/4.0,
        obstacles, /*obs_margin=*/10.0, /*clear_target_cm=*/15.0,
        /*kappa_max_vehicle=*/0.020221);

    ASSERT_TRUE(result.has_value());
    EXPECT_DOUBLE_EQ(result->first, 1.0) << "obstacle_violation=0 인 X 가 선택돼야 함";
}
