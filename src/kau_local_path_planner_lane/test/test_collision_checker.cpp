#include <gtest/gtest.h>

#include "kau_local_path_planner_lane/bezier_ext.hpp"
#include "kau_local_path_planner_lane/collision_checker.hpp"

using kau::control::Curve;
using kau::local_path_planner_lane::clearance;
using kau::local_path_planner_lane::clearanceRect;
using kau::local_path_planner_lane::hermiteToBezier;
using kau::local_path_planner_lane::minDistToPoint;
using kau::local_path_planner_lane::Obstacle;
using kau::local_path_planner_lane::obstaclesNear;
using kau::local_path_planner_lane::Point2;
using kau::local_path_planner_lane::VehicleFootprint;

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

// 2026-08-25: 회전 사각형 차체(clearanceRect)가 좁은 원 근사(clearance,
// body=1.0)로는 놓치는 충돌을 잡아내는지 확인. 장애물이 실제 차체 오른쪽
// 모서리(half_width=10cm) 에 거의 닿아 있는 상황.
TEST(CollisionChecker, ClearanceRectCatchesWideBodyCollisionMissedByNarrowCircle)
{
    const Curve cv = straightLine();
    std::vector<Obstacle> obstacles{Obstacle{Point2{50.0, 10.0}, 1.0}};

    // 좁은 원 근사: dist(curve,center)=10, - radius(1) - body(1) = 8 -> 안전.
    const double clear_circle =
        clearance(cv, obstacles, /*body=*/1.0, /*clear_target=*/15.0);
    EXPECT_NEAR(clear_circle, 8.0, 1e-3);

    // 실제 차체(half_width=10cm) 기준: 오른쪽 모서리(y=10)가 장애물 중심과
    // 겹침 -> dist(rect,center)=0, - radius(1) = -1 (충돌).
    const VehicleFootprint body{/*body_front_cm=*/5.0, /*rear_overhang_cm=*/5.0,
                               /*half_width_cm=*/10.0};
    const double clear_rect = clearanceRect(
        cv, obstacles, body, /*clear_target=*/15.0, /*sample_interval_cm=*/2.0);
    EXPECT_NEAR(clear_rect, -1.0, 1e-3);
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
