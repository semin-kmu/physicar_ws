// ====================================================================
// test_boundary_checker_lane.cpp (2026-08-25)
//
// lane-only 재설계 이후의 boundary_checker (좌/우 edge Curve 기반) 회귀
// 테스트. 폴리곤(outer/inner) 시절 API 를 검증하던 test_boundary_checker.cpp
// 는 2026-08-26 에 삭제했고 (컴파일 불가 상태로 꺼져 있었다), 이 파일이 그
// 규격 중 "도로 이탈 판정" 부분을 새 API 로 옮겨 담은 것이다.
//
// 이 테스트가 존재하는 이유: 2026-08-25 에 `marginAlongNormal` 의
// `clamp(d, 0.0, ...)` 때문에 여유가 절대 음수가 될 수 없어 **도로 이탈
// 판정이 통째로 무력**했던 회귀가 있었다 (경로를 도로 밖 50m 로 보내도
// min_wheels_on=4 / off_integral=0 / roadOkWheels=통과). 같은 일이 다시
// 생기면 여기서 잡힌다.
// ====================================================================

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

#include "kau_control/curve.hpp"
#include "kau_local_path_planner_lane/boundary_checker.hpp"

using kau::bezier::Ctrl;
using kau::bezier::Point2;
using kau::control::Curve;
using kau::local_path_planner_lane::marginAlongNormal;
using kau::local_path_planner_lane::RoadBoundary;
using kau::local_path_planner_lane::roadOkWheels;
using kau::local_path_planner_lane::roadReportWheels;
using kau::local_path_planner_lane::WheelFootprint;

namespace
{

// y = const 인 직선을 quintic 제어점 열로. (x0 -> x0+len)
Curve straightAtY(double y, double x0, double len, int nseg = 3)
{
    std::vector<Ctrl> segs;
    const double dl = len / nseg;
    for (int i = 0; i < nseg; ++i)
    {
        Ctrl c(6);
        for (int k = 0; k < 6; ++k)
        {
            c[k] = Point2{x0 + i * dl + dl * k / 5.0, y};
        }
        segs.push_back(c);
    }
    return Curve(std::move(segs), false);
}

// PhysiCar 실제 바퀴 제원 (config/local_planner.yaml 과 같은 값).
// 바깥 모서리 ±9.85, 안쪽 모서리 ±6.15.
WheelFootprint physicarWheels()
{
    WheelFootprint w;
    w.rear_axle_offset_cm = 0.0;   // 테스트는 경로점 = 후륜축 으로 단순화
    w.wheelbase_cm   = 18.0;
    w.track_width_cm = 16.0;
    w.wheel_width_cm = 3.7;
    w.min_wheels_on  = 1;
    return w;
}

// 좌 edge y=+half, 우 edge y=-half 인 직선 도로. 경로보다 넉넉히 길게.
RoadBoundary straightRoad(double half_width)
{
    RoadBoundary rb;
    rb.left  = straightAtY( half_width, -50.0, 400.0);
    rb.right = straightAtY(-half_width, -50.0, 400.0);
    return rb;
}

constexpr double kNoMargin = 0.0;
constexpr double kStep = 4.0;

}  // namespace

// 도로 안을 여유 있게 달리면 네 바퀴 모두 노면 위, 여유는 양수.
TEST(BoundaryCheckerLane, FullyInsideHasPositiveClearance)
{
    const RoadBoundary rb = straightRoad(35.0);
    const Curve path = straightAtY(0.0, 0.0, 300.0);
    const auto rep = roadReportWheels(rb, path, physicarWheels(), kNoMargin, kStep);

    EXPECT_EQ(rep.min_wheels_on, 4);
    EXPECT_GT(rep.min_clear_cm, 0.0);
    EXPECT_LT(rep.first_viol_s_cm, 0.0);
    EXPECT_NEAR(rep.off_integral_cm, 0.0, 1e-9);
    EXPECT_TRUE(roadOkWheels(rb, path, physicarWheels(), kNoMargin, kStep));
}

// 규정 재현: 바깥 모서리만 선을 물고 안쪽 모서리는 노면 위.
// -> 네 바퀴 모두 "노면 위" 지만 여유(min_clear)는 음수여야 한다.
// 바깥 9.85 / 안쪽 6.15 사이인 8.0 을 반폭으로 둔다.
TEST(BoundaryCheckerLane, OuterEdgeOnLineIsNotDeparture)
{
    const RoadBoundary rb = straightRoad(8.0);
    const Curve path = straightAtY(0.0, 0.0, 300.0);
    const auto rep = roadReportWheels(rb, path, physicarWheels(), kNoMargin, kStep);

    EXPECT_EQ(rep.min_wheels_on, 4);
    EXPECT_LT(rep.min_clear_cm, 0.0);          // 선을 물었다
    EXPECT_LT(rep.first_viol_s_cm, 0.0);       // 이탈로는 기록되지 않는다
    EXPECT_NEAR(rep.off_integral_cm, 0.0, 1e-9);
    EXPECT_TRUE(roadOkWheels(rb, path, physicarWheels(), kNoMargin, kStep));
}

// 한쪽으로 치우쳐 그쪽 두 바퀴의 안쪽 모서리까지 나가면 wheels_on 이 2 로
// 떨어지고 이탈 구간이 적분된다. 규정(min_wheels_on=1)상 거부는 아니다.
TEST(BoundaryCheckerLane, TwoWheelsOffAreCountedAndIntegrated)
{
    RoadBoundary rb;
    rb.left  = straightAtY(  5.0, -50.0, 400.0);   // 왼쪽 노면이 y=+5 에서 끝
    rb.right = straightAtY(-50.0, -50.0, 400.0);
    const Curve path = straightAtY(0.0, 0.0, 300.0);
    const auto rep = roadReportWheels(rb, path, physicarWheels(), kNoMargin, kStep);

    EXPECT_EQ(rep.min_wheels_on, 2);           // 왼쪽 두 바퀴가 완전히 밖
    EXPECT_LT(rep.min_clear_cm, 0.0);
    EXPECT_GE(rep.first_viol_s_cm, 0.0);       // 이탈 시작 호길이가 남는다
    EXPECT_GT(rep.off_integral_cm, 0.0);       // w_road 가 쓰는 적분값
    EXPECT_TRUE(roadOkWheels(rb, path, physicarWheels(), kNoMargin, kStep));

    // 보수적으로 몰고 싶으면 min_wheels_on 을 올려 거부시킬 수 있다.
    WheelFootprint strict = physicarWheels();
    strict.min_wheels_on = 4;
    EXPECT_FALSE(roadOkWheels(rb, path, strict, kNoMargin, kStep));
}

// 회귀의 핵심: 도로를 완전히 벗어난 경로는 반드시 거부돼야 한다.
// (버그 당시엔 도로 밖 50m 짜리 경로도 wheels_on=4 로 통과했다.)
TEST(BoundaryCheckerLane, FarOutsidePathIsRejected)
{
    const RoadBoundary rb = straightRoad(35.0);
    for (const double bias : {60.0, 200.0, 5000.0})
    {
        const Curve path = straightAtY(bias, 0.0, 300.0);
        const auto rep = roadReportWheels(rb, path, physicarWheels(), kNoMargin, kStep);

        EXPECT_EQ(rep.min_wheels_on, 0) << "bias=" << bias;
        EXPECT_LT(rep.min_clear_cm, 0.0) << "bias=" << bias;
        EXPECT_GT(rep.off_integral_cm, 0.0) << "bias=" << bias;
        EXPECT_FALSE(roadOkWheels(rb, path, physicarWheels(), kNoMargin, kStep))
            << "bias=" << bias;
    }
}

// edge 를 하나도 관측 못 했으면 도로 제약이 없어야 한다 (hold 만료 등).
TEST(BoundaryCheckerLane, NoEdgeObservedMeansNoConstraint)
{
    const RoadBoundary rb;   // left/right 모두 nullopt
    const Curve path = straightAtY(500.0, 0.0, 300.0);
    const auto rep = roadReportWheels(rb, path, physicarWheels(), kNoMargin, kStep);

    EXPECT_EQ(rep.min_wheels_on, 4);
    EXPECT_GT(rep.min_clear_cm, 0.0);
    EXPECT_TRUE(roadOkWheels(rb, path, physicarWheels(), kNoMargin, kStep));
}

// 반대 방향 회귀 방지: 폭 질의용 marginAlongNormal 은 계속 **비음수** 여야
// 한다. 여기가 음수가 되면 corridorOffsets 의 safe_lower > safe_upper 로
// 뒤집혀 std::clamp(v, lo, hi) 가 정의되지 않은 동작이 된다.
TEST(BoundaryCheckerLane, WidthQueryStaysNonNegative)
{
    const RoadBoundary rb = straightRoad(35.0);
    const Point2 normal{0.0, 1.0};
    for (const double y : {0.0, 30.0, 60.0, 500.0})
    {
        const Point2 origin{100.0, y};
        EXPECT_GE(marginAlongNormal(rb, origin, normal,  1.0, 11.0), 0.0) << "y=" << y;
        EXPECT_GE(marginAlongNormal(rb, origin, normal, -1.0, 11.0), 0.0) << "y=" << y;
    }
}

// min_clear_upto_cm(한 번 스캔) 은 s_max_cm 으로 끊어 따로 돈 min_clear_cm
// 과 정확히 같아야 한다 (진단 스캔 2회 -> 1회 최적화의 계약).
TEST(BoundaryCheckerLane, PrefixMinMatchesTruncatedScan)
{
    RoadBoundary rb;
    rb.left  = straightAtY( 12.0, -50.0, 400.0);
    rb.right = straightAtY(-35.0, -50.0, 400.0);
    const Curve path = straightAtY(4.0, 0.0, 300.0);

    for (const double horizon : {20.0, 52.422, 60.0, 145.0, 290.0})
    {
        const auto one = roadReportWheels(
            rb, path, physicarWheels(), 2.0, kStep,
            std::numeric_limits<double>::infinity(), horizon);
        const auto two = roadReportWheels(
            rb, path, physicarWheels(), 2.0, kStep, horizon);
        EXPECT_DOUBLE_EQ(one.min_clear_upto_cm, two.min_clear_cm)
            << "horizon=" << horizon;
    }
}
