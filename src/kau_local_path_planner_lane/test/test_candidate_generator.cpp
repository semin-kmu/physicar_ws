#include <gtest/gtest.h>

#include "kau_local_path_planner_lane/bezier_ext.hpp"
#include "kau_local_path_planner_lane/candidate_generator.hpp"

using kau::control::Curve;
using kau::local_path_planner_lane::CandidateGenerator;
using kau::local_path_planner_lane::committedSegmentCount;
using kau::local_path_planner_lane::fitSegment;
using kau::local_path_planner_lane::Frame;
using kau::local_path_planner_lane::hermiteToBezier;
using kau::local_path_planner_lane::kCommittedHorizonCm;
using kau::local_path_planner_lane::Knot;
using kau::local_path_planner_lane::kPredictionViolationPenalty;
using kau::local_path_planner_lane::makeStation;
using kau::local_path_planner_lane::Obstacle;
using kau::local_path_planner_lane::ObstacleStation;
using kau::local_path_planner_lane::PlannerParams;
using kau::local_path_planner_lane::Point2;
using kau::local_path_planner_lane::referenceFrame;
using kau::local_path_planner_lane::RoadBoundary;

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

// 2026-08-24 (committed/prediction horizon, KAU_AMET_Test 알고리즘 반영)

TEST(CandidateGenerator, CommittedSegmentCountSplitsAtHorizon)
{
    // 20cm 직선 segment 3개(누적 20/40/60cm) -- horizon(52.422cm)은
    // 2번째(누적 40)까지는 못 미치고 3번째(누적 60)에서 넘는다.
    auto straightSeg = [](double x0, double x1)
    {
        return hermiteToBezier(
            Point2{x0, 0.0}, 0.0, 0.0, Point2{x1, 0.0}, 0.0, 0.0,
            x1 - x0, x1 - x0);
    };
    const std::vector<decltype(straightSeg(0.0, 1.0))> segs{
        straightSeg(0.0, 20.0), straightSeg(20.0, 40.0), straightSeg(40.0, 60.0)};
    EXPECT_EQ(committedSegmentCount(segs), 3);

    // 총 길이가 horizon 에 못 미치면(40cm) 전부 committed.
    const std::vector<decltype(straightSeg(0.0, 1.0))> short_segs{
        straightSeg(0.0, 20.0), straightSeg(20.0, 40.0)};
    EXPECT_EQ(committedSegmentCount(short_segs), 2);
}

TEST(CandidateGenerator, PredictionZoneCurvatureViolationGetsPenaltyNotRejection)
{
    // frame0(committed, s=100cm > horizon)는 offset 0 으로 고정해 segment0
    // 을 사실상 직선으로 유지한다 -- committed 구간 자체는 curvature 위반이
    // 없다. frame1(s=300cm, segment1 = 순수 prediction 구간)에만 극단적
    // lateral offset 을 줘 그 segment 만 kappa_bound 를 위반하게 만든다.
    // committed 는 안전하므로 즉시 폐기(kappa_bound)되지 않고, cost
    // 페널티(kPredictionViolationPenalty)만 부여돼야 한다.
    const Curve gp = straightGlobalPath(1000.0);
    // 이 테스트만 road_boundary 가 아니라 순수 curvature(kappa_bound) 로
    // prediction 위반을 유발해야 하므로, 큰 lateral offset 도 도로 폭 안에
    // 남도록 y 범위를 넉넉히 잡은 전용 boundary 를 쓴다.
    RoadBoundary boundary;
    boundary.outer = {{-100.0, -5000.0}, {1100.0, -5000.0},
                      {1100.0, 5000.0}, {-100.0, 5000.0}};
    boundary.inner = {{-10000.0, -10000.0}, {-9999.0, -10000.0},
                      {-9999.0, -9999.0}};
    const std::vector<Obstacle> obstacles;
    const std::vector<ObstacleStation> stations;
    PlannerParams params;
    params.l_plan = 300.0;
    const double kappa_max_vehicle = 0.020221;
    const double kappa_lim = kappa_max_vehicle * params.kappa_margin;

    CandidateGenerator gen(
        gp, boundary, obstacles, stations, params, kappa_lim,
        kappa_max_vehicle, /*body_radius_cm=*/11.0);

    ASSERT_GT(100.0, kCommittedHorizonCm)
        << "frame0 station 이 committed horizon 보다 멀어야 segment0 만으로 "
           "committed 가 채워진다";
    const Frame frame0 = referenceFrame(gp, 100.0);
    const Frame frame1 = referenceFrame(gp, 300.0);

    const auto mild = gen.candidate(
        Point2{0.0, 0.0}, 0.0, 0.0,
        std::vector<Frame>{frame0, frame1}, std::vector<double>{0.0, 0.0},
        0.0, nullptr);
    ASSERT_TRUE(mild.reason.empty()) << "reason=" << mild.reason;

    // frame0->frame1 forward 거리는 200cm 뿐인데 frame1 에서만 2000cm
    // lateral offset 을 요구 -- segment1(=prediction) 혼자서 kappa_bound
    // 를 확실히 초과하게 만든다 (committed 인 segment0 은 offset=0 이라
    // 무관).
    const auto sharp = gen.candidate(
        Point2{0.0, 0.0}, 0.0, 0.0,
        std::vector<Frame>{frame0, frame1}, std::vector<double>{0.0, 2000.0},
        0.0, nullptr);

    EXPECT_TRUE(sharp.reason.empty()) << "reason=" << sharp.reason;
    ASSERT_TRUE(sharp.curve.has_value());
    // committed(segment0) 은 여전히 안전하므로 하드 폐기되지 않되, prediction
    // 구간 위반 페널티만큼은 cost 에 반영돼야 한다.
    EXPECT_GT(sharp.cost, mild.cost + 0.5 * kPredictionViolationPenalty);
}
