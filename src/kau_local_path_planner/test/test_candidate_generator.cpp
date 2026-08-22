#include <gtest/gtest.h>

#include "kau_local_path_planner/bezier_ext.hpp"
#include "kau_local_path_planner/candidate_generator.hpp"

using kau::control::Curve;
using kau::local_path_planner::CandidateGenerator;
using kau::local_path_planner::fitSegment;
using kau::local_path_planner::Frame;
using kau::local_path_planner::hermiteToBezier;
using kau::local_path_planner::Knot;
using kau::local_path_planner::makeStation;
using kau::local_path_planner::Obstacle;
using kau::local_path_planner::ObstacleStation;
using kau::local_path_planner::PlannerParams;
using kau::local_path_planner::Point2;
using kau::local_path_planner::referenceFrame;
using kau::local_path_planner::RoadBoundary;

namespace
{

// (-100,-1000) ~ (1100,1000) 넓은 사각형 -- 사실상 열린 벌판.
RoadBoundary wideOpenBoundary()
{
    RoadBoundary rb;
    rb.outer = {{-100.0, -1000.0}, {1100.0, -1000.0}, {1100.0, 1000.0},
               {-100.0, 1000.0}};
    // inner 는 도로 밖 아주 먼 곳에 작게 둬 사실상 영향 없게 한다.
    rb.inner = {{-10000.0, -10000.0}, {-9999.0, -10000.0}, {-9999.0, -9999.0}};
    return rb;
}

Curve straightGlobalPath(double length)
{
    auto ctrl = hermiteToBezier(
        Point2{0.0, 0.0}, 0.0, 0.0, Point2{length, 0.0}, 0.0, 0.0,
        length, length);
    return Curve(std::vector<decltype(ctrl)>{ctrl}, /*closed=*/false);
}

}  // namespace

TEST(CandidateGenerator, FitSegmentProducesRegularCurveForSimpleKnots)
{
    Knot a{Point2{0.0, 0.0}, 0.0, 0.0};
    Knot b{Point2{100.0, 20.0}, 0.0, 0.0};
    auto [ctrl, bound] = fitSegment(a, b, 2);
    ASSERT_TRUE(ctrl.has_value());
    EXPECT_TRUE(std::isfinite(bound));
    EXPECT_TRUE(kau::bezier::isRegular(*ctrl));
}

TEST(CandidateGenerator, MakeStationLateralSignMatchesSide)
{
    const Curve gp = straightGlobalPath(1000.0);
    // 장애물이 경로 왼쪽(+y)에 있으면 lateral > 0 이어야 한다
    // (좌법선 convention: n = (-sin th, cos th), th=0 이므로 n=(0,1)).
    const Obstacle left{Point2{500.0, 30.0}, 5.0};
    const ObstacleStation st = makeStation(gp, left);
    EXPECT_NEAR(st.station_s, 500.0, 1.0);
    EXPECT_GT(st.lateral, 0.0);
}

TEST(CandidateGenerator, BaseCorridorCandidateSucceedsOnOpenStraightRoad)
{
    const Curve gp = straightGlobalPath(1000.0);
    const RoadBoundary boundary = wideOpenBoundary();
    const std::vector<Obstacle> obstacles;
    const std::vector<ObstacleStation> stations;
    PlannerParams params;   // 기본값 (l_plan=150 dataclass 기본, 테스트엔 충분)
    params.l_plan = 300.0;

    const double kappa_max_vehicle = 0.020221;
    const double kappa_lim = kappa_max_vehicle * params.kappa_margin;

    CandidateGenerator gen(
        gp, boundary, obstacles, stations, params, kappa_lim,
        kappa_max_vehicle, /*body_radius_cm=*/11.0);

    const std::array<Frame, 2> frames{
        referenceFrame(gp, 0.25 * params.l_plan),
        referenceFrame(gp, params.l_plan),
    };
    const auto offset_pairs = gen.corridorOffsets(frames);

    // 중앙(fraction=0.5, index 3) 후보는 열린 직선 도로에서 반드시 성공해야 한다.
    const Point2 start{0.0, 0.0};
    const auto center = gen.candidate(
        start, /*yaw=*/0.0, /*kappa0=*/0.0, frames, offset_pairs[3],
        /*s0=*/0.0, /*previous_path=*/nullptr);

    EXPECT_TRUE(center.reason.empty()) << "reason=" << center.reason;
    ASSERT_TRUE(center.curve.has_value());
    EXPECT_TRUE(std::isfinite(center.cost));
}
