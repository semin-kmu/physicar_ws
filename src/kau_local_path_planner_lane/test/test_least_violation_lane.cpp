// ====================================================================
// test_least_violation_lane.cpp (2026-08-26)
//
// `leastViolationRect` (degraded 최후수단 선택) 회귀 방지.
//
// 이 함수는 hard constraint 를 전부 만족하는 후보가 하나도 없을 때만
// 불린다 -- 즉 **잘 돌 때는 한 번도 실행되지 않는다.** 그래서 여기가
// 깨져도 정상 주행 스모크로는 안 잡히고, 정작 코너에서 후보가 전멸한
// 순간(가장 위험한 순간)에만 티가 난다. 단위테스트가 유일한 방어선이다.
//
// 지키려는 계약 두 개 (둘 다 실측 사고에서 나왔다):
//
//   1. **장애물 우선** (2026-08-24). 원래는 obstacle/road/kappa 세 위반을
//      정규화해서 합산했는데, 그러면 obs_margin 을 키울수록 정규화 분모가
//      커져 상대적 obstacle penalty 가 오히려 작아진다. 실측에서
//      obs_margin 4->10 으로 키웠더니 min_clearance 가 -0.44 -> -0.79cm 로
//      **악화**됐다 -- 안전 여유를 늘리라고 준 파라미터가 충돌을 늘렸다.
//      그래서 obstacle_violation 최소인 후보군으로 pool 을 먼저 좁히고,
//      그 안에서만 road+kappa 를 본다.
//
//   2. **도로는 바퀴로, 장애물은 차체로** (2026-08-24). 예전엔 둘 다
//      차체 사각형으로 재서, 바퀴는 멀쩡히 노면에 있는 후보를 "도로 위반"
//      으로 깎고 엉뚱한 후보를 골랐다.
// ====================================================================

#include <gtest/gtest.h>

#include <cmath>
#include <optional>
#include <vector>

#include "kau_control/curve.hpp"
#include "kau_local_path_planner_lane/boundary_checker.hpp"
#include "kau_local_path_planner_lane/candidate_generator.hpp"
#include "kau_local_path_planner_lane/collision_checker.hpp"
#include "kau_local_path_planner_lane/path_evaluator.hpp"
#include "kau_local_path_planner_lane/types.hpp"

using kau::bezier::Ctrl;
using kau::bezier::Point2;
using kau::control::Curve;
using kau::local_path_planner_lane::Candidate;
using kau::local_path_planner_lane::fitSegment;
using kau::local_path_planner_lane::Knot;
using kau::local_path_planner_lane::leastViolationRect;
using kau::local_path_planner_lane::Obstacle;
using kau::local_path_planner_lane::RoadBoundary;
using kau::local_path_planner_lane::VehicleFootprint;
using kau::local_path_planner_lane::WheelFootprint;

namespace
{

// y = const 인 직선을 quintic 제어점 열로 (test_boundary_checker_lane 과 동일).
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

// 중심 (0, R+y0) 인 원 위의 점. s=0 에서 (0,y0), heading 0.
Point2 arcPoint(double radius, double y0, double s)
{
    const double th = s / radius;
    return Point2{radius * std::sin(th), y0 + radius - radius * std::cos(th)};
}

// 곡률이 정확히 1/radius 인 원호 (test_lane_backbone 과 동일한 적합 방식).
Curve arcCurve(double radius, double y0, double len, int nseg = 3)
{
    std::vector<Ctrl> segs;
    for (int i = 0; i < nseg; ++i)
    {
        const double sa = len * i / nseg;
        const double sb = len * (i + 1) / nseg;
        const Knot a{arcPoint(radius, y0, sa), sa / radius, 1.0 / radius};
        const Knot b{arcPoint(radius, y0, sb), sb / radius, 1.0 / radius};
        const auto [seg, kb] = fitSegment(a, b, 2);
        EXPECT_TRUE(seg.has_value());
        if (seg)
        {
            segs.push_back(*seg);
        }
    }
    return Curve(std::move(segs), false);
}

Candidate cand(double d, Curve cv)
{
    Candidate c;
    c.d = d;
    c.curve = std::move(cv);
    // reason 은 leastViolationRect 가 의도적으로 무시한다 -- hard gate 로
    // 걸러진 후보야말로 이 함수의 대상이다. 실제 호출부와 같게 채워 둔다.
    c.reason = "road_boundary";
    return c;
}

// 좌 edge y=+left, 우 edge y=-right 인 직선 도로.
RoadBoundary road(double left, double right)
{
    RoadBoundary rb;
    rb.left  = straightAtY( left,  -50.0, 500.0);
    rb.right = straightAtY(-right, -50.0, 500.0);
    return rb;
}

VehicleFootprint body()
{
    return VehicleFootprint{};      // front 23 / rear 5 / half_width 10
}

WheelFootprint wheels()
{
    WheelFootprint w;               // 바깥 모서리 ±9.75
    w.min_wheels_on = 1;
    return w;
}

constexpr double kStep        = 4.0;
constexpr double kClearTarget = 15.0;
constexpr double kKappaMax    = 1.0 / 60.0;   // R=60cm, 차량 한계

}  // namespace


// ====================================================================
// 1. 장애물 우선 -- obs_margin 을 키워도 선택이 뒤집히지 않는다.
//
// 이것이 이 파일의 핵심이다. 위 주석 (1) 의 실측 사고를 그대로 재현한다.
//
//   A  도로를 크게 물고 나가지만 장애물과는 무관        (obstacle 0)
//   B  도로는 완벽하지만 장애물 한가운데를 통과         (obstacle 큼)
//
// 정규화-합산으로 되돌아가면 obs_margin 이 커질수록 B 가 싸 보여서
// **충돌 경로인 B** 를 고르게 된다. 장애물 우선 규칙에서는 obs_margin 이
// 무엇이든 pool 이 A 하나로 좁혀지므로 항상 A 다.
// ====================================================================
TEST(LeastViolationLane, ObstacleFirstSurvivesObsMarginSweep)
{
    // 왼쪽 노면이 y=+12 에서 끝난다. 오른쪽은 멀리(-60) 열어 둔다.
    const RoadBoundary rb = road(12.0, 60.0);

    // A: y=+45 -- 바퀴 바깥이 +54.75 라 노면을 42cm 넘게 벗어난다.
    const Candidate a = cand(45.0, straightAtY(45.0, 0.0, 200.0));
    // B: y=-20 -- 노면 한가운데. 그 위에 장애물을 얹는다.
    const Candidate b = cand(-20.0, straightAtY(-20.0, 0.0, 200.0));

    const std::vector<Obstacle> obs{Obstacle{Point2{100.0, -20.0}, 10.0}};
    const std::vector<Candidate> cands{a, b};

    for (const double obs_margin : {4.0, 10.0, 20.0, 40.0})
    {
        const auto pick = leastViolationRect(
            cands, rb, body(), wheels(), /*road_safety_margin_cm=*/2.0, kStep,
            obs, obs_margin, kClearTarget, kKappaMax);

        ASSERT_TRUE(pick.has_value()) << "obs_margin=" << obs_margin;
        EXPECT_DOUBLE_EQ(pick->first, 45.0)
            << "obs_margin=" << obs_margin
            << " 에서 충돌 경로가 선택됐다 -- 정규화-합산 회귀";
    }
}

// 후보 순서를 뒤집어도 같은 답이 나와야 한다 (pool 축소가 순서에 의존하면 안 된다).
TEST(LeastViolationLane, ObstacleFirstIsOrderIndependent)
{
    const RoadBoundary rb = road(12.0, 60.0);
    const std::vector<Obstacle> obs{Obstacle{Point2{100.0, -20.0}, 10.0}};

    const Candidate a = cand(45.0, straightAtY(45.0, 0.0, 200.0));
    const Candidate b = cand(-20.0, straightAtY(-20.0, 0.0, 200.0));

    const auto fwd = leastViolationRect(
        {a, b}, rb, body(), wheels(), 2.0, kStep, obs, 10.0, kClearTarget, kKappaMax);
    const auto rev = leastViolationRect(
        {b, a}, rb, body(), wheels(), 2.0, kStep, obs, 10.0, kClearTarget, kKappaMax);

    ASSERT_TRUE(fwd.has_value());
    ASSERT_TRUE(rev.has_value());
    EXPECT_DOUBLE_EQ(fwd->first, 45.0);
    EXPECT_DOUBLE_EQ(rev->first, 45.0);
}

// ====================================================================
// 2. 장애물 위반이 같으면 그때서야 road + kappa 로 고른다.
// ====================================================================

// 장애물이 아예 없으면 (전부 위반 0) 도로를 덜 벗어난 쪽.
TEST(LeastViolationLane, TieOnObstacleFallsBackToRoad)
{
    const RoadBoundary rb = road(12.0, 60.0);
    const std::vector<Candidate> cands{
        cand(45.0, straightAtY(45.0, 0.0, 200.0)),   // 42cm 이탈
        cand( 0.0, straightAtY( 0.0, 0.0, 200.0)),   // 노면 안
    };

    const auto pick = leastViolationRect(
        cands, rb, body(), wheels(), 2.0, kStep, {}, 4.0, kClearTarget, kKappaMax);

    ASSERT_TRUE(pick.has_value());
    EXPECT_DOUBLE_EQ(pick->first, 0.0);
}

// 도로 위반도 같으면 곡률이 덜 넘친 쪽. (둘 다 노면 안 -> road 0 동률)
TEST(LeastViolationLane, TieOnObstacleAndRoadFallsBackToKappa)
{
    const RoadBoundary rb = road(60.0, 60.0);   // 넓은 도로, 둘 다 이탈 없음
    const std::vector<Candidate> cands{
        cand(0.0, arcCurve(30.0, 0.0, 40.0)),    // kappa 1/30 -- 한계의 2배
        cand(5.0, arcCurve(55.0, 0.0, 40.0)),    // kappa 1/55 -- 살짝 초과
    };

    const auto pick = leastViolationRect(
        cands, rb, body(), wheels(), 2.0, kStep, {}, 4.0, kClearTarget, kKappaMax);

    ASSERT_TRUE(pick.has_value());
    EXPECT_DOUBLE_EQ(pick->first, 5.0);
}

// 전부 충돌하더라도 **덜** 충돌하는 것을 반드시 하나 돌려준다.
// degraded 는 "발행할 게 없다" 로 끝나면 안 된다 (그러면 차가 선다).
TEST(LeastViolationLane, AllCollidingStillReturnsLeastColliding)
{
    const RoadBoundary rb = road(60.0, 60.0);
    // 장애물을 y=0 에 두고 두 후보를 서로 다른 깊이로 통과시킨다.
    const std::vector<Obstacle> obs{Obstacle{Point2{100.0, 0.0}, 12.0}};
    const std::vector<Candidate> cands{
        cand( 0.0, straightAtY( 0.0, 0.0, 200.0)),   // 정면 관통
        cand(18.0, straightAtY(18.0, 0.0, 200.0)),   // 스쳐 지나감
    };

    const auto pick = leastViolationRect(
        cands, rb, body(), wheels(), 2.0, kStep, obs, 4.0, kClearTarget, kKappaMax);

    ASSERT_TRUE(pick.has_value());
    EXPECT_DOUBLE_EQ(pick->first, 18.0);
}

// ====================================================================
// 3. 도로 위반은 차체가 아니라 바퀴로 잰다.
//
// 노면 반폭을 바퀴 바깥(9.75) 과 차체 반폭(10.0) **사이**에 둔다.
// 바퀴로 재면 A 는 위반 0 이고, 차체로 재면 위반이 생긴다.
// 그래서 차체로 되돌아가면 A 가 밀려 곡률을 넘긴 B 가 뽑힌다.
// ====================================================================
TEST(LeastViolationLane, RoadViolationMeasuredByWheelsNotBody)
{
    // 왼쪽 노면만 바퀴 바깥(9.75) 과 차체 반폭(10.0) 사이에 둔다. 오른쪽은
    // 멀리 열어 둬서 B 가 도로와 무관하게 놀 공간을 준다 -- 양쪽을 다 좁히면
    // B 도 같이 도로를 벗어나 버려서 두 측정 방식이 같은 답을 낸다.
    RoadBoundary rb;
    rb.left  = straightAtY( 9.85, -50.0, 500.0);
    rb.right = straightAtY(-60.0, -50.0, 500.0);

    const std::vector<Candidate> cands{
        // A: 왼쪽 선에 바짝 붙은 직선. 바퀴 +9.75 는 안(여유 0.05),
        //    차체 +10.0 은 0.20cm 밖. 곡률 위반은 없다.
        cand(0.0, straightAtY(0.0, 0.0, 200.0)),
        // B: 오른쪽 빈 공간에서 오른쪽으로 휘는 원호. 도로 위반은 어느
        //    기준으로 재도 0 이고, 곡률만 한계를 살짝 넘는다(R=55 < 60).
        cand(-20.0, arcCurve(-55.0, -20.0, 40.0)),
    };

    const auto pick = leastViolationRect(
        cands, rb, body(), wheels(), /*road_safety_margin_cm=*/0.05, kStep,
        {}, 4.0, kClearTarget, kKappaMax);

    ASSERT_TRUE(pick.has_value());
    EXPECT_DOUBLE_EQ(pick->first, 0.0)
        << "도로 위반을 차체로 재고 있다 -- 바퀴 기준으로 되돌릴 것";
}

// ====================================================================
// 4. 퇴화 입력
// ====================================================================

TEST(LeastViolationLane, EmptyCandidatesReturnNullopt)
{
    const auto pick = leastViolationRect(
        {}, road(60.0, 60.0), body(), wheels(), 2.0, kStep, {}, 4.0,
        kClearTarget, kKappaMax);
    EXPECT_FALSE(pick.has_value());
}

// curve 구성 자체가 실패한 후보만 있으면 고를 게 없다 (kNoFeasible).
TEST(LeastViolationLane, CurvelessCandidatesReturnNullopt)
{
    Candidate broken;
    broken.d = 10.0;
    broken.reason = "degenerate";      // curve 없음

    const auto pick = leastViolationRect(
        {broken}, road(60.0, 60.0), body(), wheels(), 2.0, kStep, {}, 4.0,
        kClearTarget, kKappaMax);
    EXPECT_FALSE(pick.has_value());
}

// curve 없는 후보는 건너뛰고 성한 것을 고른다.
TEST(LeastViolationLane, SkipsCurvelessAndPicksTheRest)
{
    Candidate broken;
    broken.d = 10.0;
    broken.reason = "degenerate";

    const Candidate good = cand(-5.0, straightAtY(-5.0, 0.0, 200.0));

    const auto pick = leastViolationRect(
        {broken, good}, road(60.0, 60.0), body(), wheels(), 2.0, kStep, {}, 4.0,
        kClearTarget, kKappaMax);

    ASSERT_TRUE(pick.has_value());
    EXPECT_DOUBLE_EQ(pick->first, -5.0);
}

// 돌려주는 curve 는 고른 후보의 것 그대로여야 한다 (d 만 맞고 곡선이
// 엉뚱하면 발행 경로가 후보와 달라진다).
TEST(LeastViolationLane, ReturnsChosenCandidatesOwnCurve)
{
    const Curve cv = straightAtY(-5.0, 0.0, 200.0);
    const auto pick = leastViolationRect(
        {cand(-5.0, cv)}, road(60.0, 60.0), body(), wheels(), 2.0, kStep, {},
        4.0, kClearTarget, kKappaMax);

    ASSERT_TRUE(pick.has_value());
    EXPECT_EQ(pick->second.nseg(), cv.nseg());
    EXPECT_NEAR(pick->second.length(), cv.length(), 1e-9);
    EXPECT_NEAR(pick->second.point(0.0).y, -5.0, 1e-9);
}
