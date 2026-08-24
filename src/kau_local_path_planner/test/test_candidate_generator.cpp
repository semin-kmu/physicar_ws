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

    // corridor 는 3-frame (0.25L/0.625L/1.0L, 2026-08-24 KAU_AMET_Test
    // 알고리즘 반영) -- N-knot 일반형 candidate() 로 검증한다.
    const std::array<Frame, 3> frames{
        referenceFrame(gp, 0.25 * params.l_plan),
        referenceFrame(gp, 0.625 * params.l_plan),
        referenceFrame(gp, params.l_plan),
    };
    const auto offset_pairs = gen.corridorOffsets(frames);

    // 중앙(fraction=0.5, index 3) 후보는 열린 직선 도로에서 반드시 성공해야 한다.
    const Point2 start{0.0, 0.0};
    const auto center = gen.candidate(
        start, /*yaw=*/0.0, /*kappa0=*/0.0,
        std::vector<Frame>{frames[0], frames[1], frames[2]},
        std::vector<double>{
            offset_pairs[3][0], offset_pairs[3][1], offset_pairs[3][2]},
        /*s0=*/0.0, /*previous_path=*/nullptr);

    EXPECT_TRUE(center.reason.empty()) << "reason=" << center.reason;
    ASSERT_TRUE(center.curve.has_value());
    EXPECT_TRUE(std::isfinite(center.cost));
}

TEST(CandidateGenerator, CorridorOffsetsMiddleFrameLiesBetweenEndpoints)
{
    // 0.625L knot 의 offset 이 0.25L/1.0L 사이(같은 fraction 기준)와
    // 물리적으로 부합하는지 -- 완전 직선 도로에서는 세 frame 의 안전폭이
    // 전부 동일해야 하므로 offset 값도 전부 같아야 한다.
    const Curve gp = straightGlobalPath(1000.0);
    const RoadBoundary boundary = wideOpenBoundary();
    const std::vector<Obstacle> obstacles;
    const std::vector<ObstacleStation> stations;
    PlannerParams params;
    params.l_plan = 300.0;
    const double kappa_max_vehicle = 0.020221;
    const double kappa_lim = kappa_max_vehicle * params.kappa_margin;

    CandidateGenerator gen(
        gp, boundary, obstacles, stations, params, kappa_lim,
        kappa_max_vehicle, /*body_radius_cm=*/11.0);

    const std::array<Frame, 3> frames{
        referenceFrame(gp, 0.25 * params.l_plan),
        referenceFrame(gp, 0.625 * params.l_plan),
        referenceFrame(gp, params.l_plan),
    };
    const auto offset_pairs = gen.corridorOffsets(frames);
    for (const auto & pair : offset_pairs)
    {
        EXPECT_NEAR(pair[0], pair[1], 1e-6);
        EXPECT_NEAR(pair[1], pair[2], 1e-6);
    }
}

TEST(CandidateGenerator, ObstacleOffsetCandidatesEmptyWhenNoObstacleInRange)
{
    const Curve gp = straightGlobalPath(1000.0);
    const RoadBoundary boundary = wideOpenBoundary();
    const std::vector<Obstacle> obstacles;
    const std::vector<ObstacleStation> stations;
    PlannerParams params;
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
    const auto cands = gen.obstacleOffsetCandidates(
        Point2{0.0, 0.0}, 0.0, 0.0, frames, 0.0, nullptr);
    EXPECT_TRUE(cands.empty());
}

TEST(CandidateGenerator, ObstacleOffsetCandidatesProduceTwoFeasibleSidesOnOpenRoad)
{
    // 열린 직선 도로 + 전방 장애물 하나 -- 좌/우 side 모두 "Smart K1"
    // 후보가 완성돼야 한다 (corridor 7개가 전멸하는 상황을 흉내내는
    // 것이 아니라, 이 함수 자체가 독립적으로 feasible 후보를 만드는지
    // 검증).
    const Curve gp = straightGlobalPath(1000.0);
    const RoadBoundary boundary = wideOpenBoundary();
    PlannerParams params;
    params.l_plan = 300.0;
    const double kappa_max_vehicle = 0.020221;
    const double kappa_lim = kappa_max_vehicle * params.kappa_margin;

    const std::vector<Obstacle> obstacles{Obstacle{Point2{150.0, 0.0}, 5.0}};
    std::vector<ObstacleStation> stations{makeStation(gp, obstacles[0])};

    CandidateGenerator gen(
        gp, boundary, obstacles, stations, params, kappa_lim,
        kappa_max_vehicle, /*body_radius_cm=*/11.0);

    const std::array<Frame, 2> frames{
        referenceFrame(gp, 0.25 * params.l_plan),
        referenceFrame(gp, params.l_plan),
    };
    const auto cands = gen.obstacleOffsetCandidates(
        Point2{0.0, 0.0}, 0.0, 0.0, frames, 0.0, nullptr);
    ASSERT_EQ(cands.size(), 2u);
    for (const auto & c : cands)
    {
        EXPECT_TRUE(c.reason.empty()) << "reason=" << c.reason;
        ASSERT_TRUE(c.curve.has_value());
    }
    // 근접측(side=-1)과 원접측(side=+1) offset 부호가 서로 반대여야 한다.
    EXPECT_LT(cands[0].d * cands[1].d, 0.0);
}
