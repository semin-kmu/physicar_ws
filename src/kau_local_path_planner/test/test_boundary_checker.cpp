#include <gtest/gtest.h>

#include "kau_local_path_planner/bezier_ext.hpp"
#include "kau_local_path_planner/boundary_checker.hpp"

using kau::control::Curve;
using kau::local_path_planner::distanceToPolygonBoundary;
using kau::local_path_planner::hermiteToBezier;
using kau::local_path_planner::marginAlongNormal;
using kau::local_path_planner::Point2;
using kau::local_path_planner::pointClearance;
using kau::local_path_planner::pointInPolygon;
using kau::local_path_planner::roadClearance;
using kau::local_path_planner::roadClearanceRect;
using kau::local_path_planner::RoadBoundary;
using kau::local_path_planner::VehicleFootprint;

namespace
{

std::vector<Point2> square(double x0, double y0, double x1, double y1)
{
    return {{x0, y0}, {x1, y0}, {x1, y1}, {x0, y1}};
}

Curve straightLineAtY(double x0, double x1, double y)
{
    auto ctrl = hermiteToBezier(
        Point2{x0, y}, 0.0, 0.0, Point2{x1, y}, 0.0, 0.0, x1 - x0, x1 - x0);
    return Curve(std::vector<decltype(ctrl)>{ctrl}, false);
}

}  // namespace

TEST(BoundaryChecker, PointInPolygonBasic)
{
    const auto poly = square(0.0, 0.0, 10.0, 10.0);
    EXPECT_TRUE(pointInPolygon(poly, Point2{5.0, 5.0}));
    EXPECT_FALSE(pointInPolygon(poly, Point2{15.0, 5.0}));
    EXPECT_FALSE(pointInPolygon(poly, Point2{-1.0, 5.0}));
}

TEST(BoundaryChecker, DistanceToPolygonBoundary)
{
    const auto poly = square(0.0, 0.0, 10.0, 10.0);
    // 중앙점 (5,5) 는 4변 모두에서 5.0 거리.
    EXPECT_NEAR(distanceToPolygonBoundary(poly, Point2{5.0, 5.0}), 5.0, 1e-9);
    // 경계 위의 점은 거리 0.
    EXPECT_NEAR(distanceToPolygonBoundary(poly, Point2{0.0, 5.0}), 0.0, 1e-9);
}

TEST(BoundaryChecker, PointClearanceDrivableRing)
{
    // outer: 20x20 정사각형, inner: 중앙 4x4 정사각형(주행 금지 구역).
    RoadBoundary rb;
    rb.outer = square(0.0, 0.0, 20.0, 20.0);
    rb.inner = square(8.0, 8.0, 12.0, 12.0);

    // (2, 10): outer 로부터 2, inner 밖 -> outer 가 더 가까움 -> margin ~= 2 - footprint.
    const double footprint = 1.0;
    const double clear_near_outer = pointClearance(rb, Point2{2.0, 10.0}, footprint);
    EXPECT_NEAR(clear_near_outer, 2.0 - footprint, 1e-6);

    // inner 내부(10,10) -> 위반, 음수.
    const double clear_inside_inner = pointClearance(rb, Point2{10.0, 10.0}, footprint);
    EXPECT_LT(clear_inside_inner, 0.0);

    // outer 밖(30,10) -> 위반, 음수.
    const double clear_outside_outer = pointClearance(rb, Point2{30.0, 10.0}, footprint);
    EXPECT_LT(clear_outside_outer, 0.0);
}

TEST(BoundaryChecker, MarginAlongNormalFindsBoundary)
{
    RoadBoundary rb;
    rb.outer = square(0.0, 0.0, 20.0, 20.0);
    rb.inner = square(8.0, 8.0, 12.0, 12.0);

    // origin=(2,10), normal=(1,0) (즉 +x 방향), footprint=0.
    // +x 로 가면 inner 정사각형(8,8)-(12,12) 의 왼쪽 변(x=8)에서 위반 시작
    // -> margin ~= 8 - 2 = 6.
    const double m = marginAlongNormal(
        rb, Point2{2.0, 10.0}, Point2{1.0, 0.0}, 1.0, 0.0, 50.0);
    EXPECT_NEAR(m, 6.0, 0.05);

    // 반대 방향(-x, sign=-1)으로는 outer 왼쪽 변(x=0)까지 2.
    const double m_neg = marginAlongNormal(
        rb, Point2{2.0, 10.0}, Point2{1.0, 0.0}, -1.0, 0.0, 50.0);
    EXPECT_NEAR(m_neg, 2.0, 0.05);
}

// 2026-08-25: 회전 사각형 차체(roadClearanceRect)가, centerline 점 하나만
// 보는 기존 원 근사(roadClearance)로는 놓치는 위반을 잡아내는지 확인.
// 경로는 y=0 을 따라 직진(heading=0). inner 에 y=8~20, x=40~60 구간에
// "벽"을 두면 -- centerline(y=0)은 벽에서 8cm 떨어져 안전하지만, 차폭
// half_width=10cm 인 차체의 오른쪽 모서리(y=+10)는 그 벽 안으로 들어간다.
TEST(BoundaryChecker, RoadClearanceRectCatchesCornerMissedByCenterlineOnly)
{
    RoadBoundary rb;
    // outer 를 경로(x: 0~100) 보다 넉넉히 크게 둬서 경로 양 끝점이 outer
    // 경계에 걸쳐 거리 0이 되는 것을 피한다.
    rb.outer = square(-50.0, -20.0, 150.0, 20.0);
    rb.inner = square(40.0, 8.0, 60.0, 20.0);   // 벽: x in [40,60], y in [8,20]

    const Curve path = straightLineAtY(0.0, 100.0, 0.0);

    // centerline 점만 보는 기존 방식(footprint=0): 벽까지 8cm 여유 -> 안전.
    EXPECT_GT(roadClearance(rb, path, /*footprint_cm=*/0.0,
                           /*sample_interval_cm=*/2.0),
             0.0);

    // 실제 차체 폭(half_width=10cm)을 반영하면 오른쪽 모서리가 벽 안으로
    // 들어가 위반.
    const VehicleFootprint body{/*body_front_cm=*/1.0, /*rear_overhang_cm=*/1.0,
                               /*half_width_cm=*/10.0};
    EXPECT_LT(roadClearanceRect(rb, path, body, /*road_safety_margin_cm=*/0.0,
                               /*sample_interval_cm=*/2.0),
             0.0);
}
