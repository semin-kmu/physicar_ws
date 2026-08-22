#include <gtest/gtest.h>

#include "kau_local_path_planner/bezier_ext.hpp"
#include "kau_local_path_planner/collision_checker.hpp"

using kau::control::Curve;
using kau::local_path_planner::clearance;
using kau::local_path_planner::hermiteToBezier;
using kau::local_path_planner::minDistToPoint;
using kau::local_path_planner::Obstacle;
using kau::local_path_planner::obstaclesNear;
using kau::local_path_planner::Point2;

namespace
{

// (0,0) -> (100,0) 직선, 곡률 0.
Curve straightLine()
{
    auto ctrl = hermiteToBezier(
        Point2{0.0, 0.0}, 0.0, 0.0, Point2{100.0, 0.0}, 0.0, 0.0, 100.0, 100.0);
    return Curve(std::vector<decltype(ctrl)>{ctrl}, false);
}

}  // namespace

TEST(CollisionChecker, MinDistToPointOnStraightLine)
{
    const Curve cv = straightLine();
    EXPECT_NEAR(minDistToPoint(cv, Point2{50.0, 10.0}), 10.0, 1e-3);
    EXPECT_NEAR(minDistToPoint(cv, Point2{50.0, 0.0}), 0.0, 1e-6);
    // 종점 너머 (150,0) -> 가장 가까운 점은 종점(100,0), 거리 50.
    EXPECT_NEAR(minDistToPoint(cv, Point2{150.0, 0.0}), 50.0, 1e-3);
}

TEST(CollisionChecker, ClearanceNoObstacleReturnsCap)
{
    const Curve cv = straightLine();
    const double clear = clearance(cv, {}, /*body=*/9.0, /*clear_target=*/15.0);
    EXPECT_DOUBLE_EQ(clear, 999.0);
}

TEST(CollisionChecker, ClearanceWithNearbyObstacle)
{
    const Curve cv = straightLine();
    std::vector<Obstacle> obstacles{Obstacle{Point2{50.0, 10.0}, 2.0}};
    const double body = 1.0;
    const double clear = clearance(cv, obstacles, body, /*clear_target=*/15.0);
    // dist(curve, center)=10, - radius(2) - body(1) = 7
    EXPECT_NEAR(clear, 7.0, 1e-3);
}

TEST(CollisionChecker, ObstaclesNearFiltersFarAway)
{
    const Curve cv = straightLine();
    std::vector<Obstacle> obstacles{
        Obstacle{Point2{50.0, 5.0}, 1.0},        // AABB 안
        Obstacle{Point2{5000.0, 5000.0}, 1.0},   // 멀리 -> 제외
    };
    const auto near = obstaclesNear(cv, obstacles, /*reach=*/20.0);
    ASSERT_EQ(near.size(), 1U);
    EXPECT_DOUBLE_EQ(near.front().center.x, 50.0);
}
