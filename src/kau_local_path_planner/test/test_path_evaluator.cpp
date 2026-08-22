#include <gtest/gtest.h>

#include "kau_local_path_planner/bezier_ext.hpp"
#include "kau_local_path_planner/path_evaluator.hpp"

using kau::control::Curve;
using kau::local_path_planner::continuityCost;
using kau::local_path_planner::cost;
using kau::local_path_planner::hermiteToBezier;
using kau::local_path_planner::PlannerParams;
using kau::local_path_planner::Point2;
using kau::local_path_planner::pathPreviewCost;

namespace
{

Curve straightLine(double x0, double x1)
{
    const auto ctrl = hermiteToBezier(
        Point2{x0, 0.0}, 0.0, 0.0, Point2{x1, 0.0}, 0.0, 0.0,
        x1 - x0, x1 - x0);
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
    const auto ctrl_prev = hermiteToBezier(
        Point2{0.0, 0.0}, 0.0, 0.0, Point2{100.0, 0.0}, 0.0, 0.0, 100.0, 100.0);
    const auto ctrl_cand = hermiteToBezier(
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
