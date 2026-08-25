#include <gtest/gtest.h>

#include "kau_local_path_planner_lane/bezier_ext.hpp"
#include "kau_local_path_planner_lane/boundary_checker.hpp"

using kau::control::Curve;
using kau::local_path_planner_lane::distanceToPolygonBoundary;
using kau::local_path_planner_lane::hermiteToBezier;
using kau::local_path_planner_lane::marginAlongNormal;
using kau::local_path_planner_lane::Point2;
using kau::local_path_planner_lane::pointClearance;
using kau::local_path_planner_lane::pointInPolygon;
using kau::local_path_planner_lane::roadClearance;
using kau::local_path_planner_lane::roadClearanceRect;
using kau::local_path_planner_lane::RoadBoundary;
using kau::local_path_planner_lane::roadOkRect;
using kau::local_path_planner_lane::roadOkWheels;
using kau::local_path_planner_lane::roadReportWheels;
using kau::local_path_planner_lane::VehicleFootprint;
using kau::local_path_planner_lane::WheelFootprint;

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

// ====================================================================
// 2026-08-24: 도로 이탈 판정을 바퀴 기준으로 바꾼 것에 대한 테스트.
//
// 규정: 흰 실선은 밟아도 되고, 네 바퀴가 전부 노면 밖으로 나가야 감점.
// 그래서 판정 대상은 차체 사각형이 아니라 바퀴 4개다.
// ====================================================================

namespace
{

// 실차 제원 (physicar.urdf.xacro). 경로 점은 base_footprint = 휠베이스 중앙
// 이므로 rear_axle_offset = -wheelbase/2.
WheelFootprint physicarWheels()
{
    WheelFootprint w;
    w.rear_axle_offset_cm = -9.0;
    w.wheelbase_cm = 18.0;
    w.track_width_cm = 16.0;
    w.wheel_width_cm = 3.5;
    w.min_wheels_on = 1;
    return w;
}

VehicleFootprint physicarBody()
{
    VehicleFootprint b;
    b.body_front_cm = 23.0;
    b.rear_overhang_cm = 5.0;
    b.half_width_cm = 10.0;
    b.rear_axle_offset_cm = -9.0;
    return b;
}

}  // namespace

// 바퀴 기하가 URDF 대로 유도되는지. 바깥 모서리 (16+3.5)/2 = 9.75,
// 안쪽 모서리 (16-3.5)/2 = 6.25, 축은 -9 / +9.
TEST(BoundaryChecker, WheelFootprintGeometryFromUrdf)
{
    const WheelFootprint w = physicarWheels();
    EXPECT_NEAR(w.rearAxleX(), -9.0, 1e-9);
    EXPECT_NEAR(w.frontAxleX(), 9.0, 1e-9);
    EXPECT_NEAR(w.outerY(), 9.75, 1e-9);
    EXPECT_NEAR(w.innerY(), 6.25, 1e-9);
}

// rear_axle_offset 이 차체 사각형을 실제로 뒤로 옮기는지. 보정 전에는
// 후륜축 치수(-5..+23)를 경로 점에 그대로 얹어 9cm 앞으로 밀려 있었다.
TEST(BoundaryChecker, RearAxleOffsetShiftsBodyBox)
{
    VehicleFootprint uncorrected;   // rear_axle_offset_cm = 0 (기본값)
    uncorrected.body_front_cm = 23.0;
    uncorrected.rear_overhang_cm = 5.0;
    EXPECT_NEAR(uncorrected.frontEdge(), 23.0, 1e-9);
    EXPECT_NEAR(uncorrected.rearEdge(), -5.0, 1e-9);

    const VehicleFootprint corrected = physicarBody();
    // base_footprint 기준으로는 앞뒤 대칭 ±14 여야 한다 (차체 길이 28).
    EXPECT_NEAR(corrected.frontEdge(), 14.0, 1e-9);
    EXPECT_NEAR(corrected.rearEdge(), -14.0, 1e-9);
}

// 규정 재현: 노면 폭이 딱 바퀴 바깥 모서리에 닿는 경우.
// 경로가 y=0 직진, 노면이 y in [-9.0, 9.0] 이면 바깥 모서리(±9.75)는 밖으로
// 나가지만 안쪽 모서리(±6.25)는 안에 있다 = "흰선을 밟았을 뿐 이탈 아님".
// -> 네 바퀴 모두 노면 위(wheels_on=4), 다만 여유(min_clear)는 음수.
TEST(BoundaryChecker, WheelsOnRoadWhenOnlyOuterEdgeCrossesLine)
{
    RoadBoundary rb;
    rb.outer = square(-50.0, -9.0, 150.0, 9.0);
    rb.inner = square(1000.0, 1000.0, 1001.0, 1001.0);   // 사실상 없음

    const Curve path = straightLineAtY(0.0, 100.0, 0.0);
    const auto rep = roadReportWheels(
        rb, path, physicarWheels(), /*road_safety_margin_cm=*/0.0,
        /*sample_interval_cm=*/4.0);

    EXPECT_EQ(rep.min_wheels_on, 4);
    EXPECT_LT(rep.min_clear_cm, 0.0);        // 바깥 모서리는 선을 물었다
    EXPECT_LT(rep.first_viol_s_cm, 0.0);     // 이탈로 기록되지는 않는다
    EXPECT_NEAR(rep.off_integral_cm, 0.0, 1e-9);
    EXPECT_TRUE(roadOkWheels(rb, path, physicarWheels(), 0.0, 4.0));
}

// 노면을 더 좁혀 안쪽 모서리(6.25)까지 밖으로 내보내면 그 쪽 두 바퀴가
// 완전히 이탈한다 -- wheels_on 이 2 로 떨어지고 이탈 구간이 적분된다.
// min_wheels_on=1 규정에서는 여전히 거부되지 않는다 (비용으로만 반영).
TEST(BoundaryChecker, WheelsOffCountedWhenInnerEdgeCrosses)
{
    RoadBoundary rb;
    rb.outer = square(-50.0, -50.0, 150.0, 5.0);   // y > 5 는 노면 밖
    rb.inner = square(1000.0, 1000.0, 1001.0, 1001.0);

    const Curve path = straightLineAtY(0.0, 100.0, 0.0);
    const auto rep = roadReportWheels(
        rb, path, physicarWheels(), /*road_safety_margin_cm=*/0.0,
        /*sample_interval_cm=*/4.0);

    EXPECT_EQ(rep.min_wheels_on, 2);          // 왼쪽 두 바퀴(y=+)가 완전히 밖
    EXPECT_GE(rep.first_viol_s_cm, 0.0);      // 이탈 시작 호길이가 기록된다
    EXPECT_GT(rep.off_integral_cm, 0.0);      // w_road 가 쓸 적분값
    // 규정 기준(한 바퀴만 남아도 OK)으로는 통과.
    EXPECT_TRUE(roadOkWheels(rb, path, physicarWheels(), 0.0, 4.0));

    // 보수적으로 몰고 싶으면 min_wheels_on=4 로 올려 거부시킬 수 있다.
    WheelFootprint strict = physicarWheels();
    strict.min_wheels_on = 4;
    EXPECT_FALSE(roadOkWheels(rb, path, strict, 0.0, 4.0));
}

// 이번 변경의 핵심: 차체 사각형 게이트는 실제 바퀴보다 훨씬 바깥을 본다.
// 경로 앞쪽(x>=30)에 벽을 두면, 보정된 차체 앞모서리(+14, ±10)는 벽에
// 닿지만 앞바퀴(+9, ±9.75)는 아직 닿지 않는 구간이 존재한다.
TEST(BoundaryChecker, WheelGateIsLooserThanBodyBoxGate)
{
    RoadBoundary rb;
    rb.outer = square(-50.0, -20.0, 150.0, 20.0);
    // 경로 끝(x=100) 바로 앞에 가로 벽. 차체 앞모서리는 x=114 까지 뻗지만
    // 앞바퀴는 x=109 까지만 간다.
    rb.inner = square(112.0, -20.0, 130.0, 20.0);

    const Curve path = straightLineAtY(0.0, 100.0, 0.0);

    EXPECT_FALSE(roadOkRect(rb, path, physicarBody(),
                           /*road_safety_margin_cm=*/0.0,
                           /*sample_interval_cm=*/4.0));
    EXPECT_TRUE(roadOkWheels(rb, path, physicarWheels(),
                            /*road_safety_margin_cm=*/0.0,
                            /*sample_interval_cm=*/4.0));
}
