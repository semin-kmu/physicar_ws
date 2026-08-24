// 통합 시나리오 테스트. KAU_AMET_Test 세션의 full_simulation.py 검증
// 방식(0.05/0.30/1.0 lap 시나리오 돌려 plan_status 분포/min_clear 확인)을
// 축소해, "열린 도로", "정면 장애물" 두 시나리오에서 plan() 이 크래시
// 없이 합리적인 결과를 내는지만 확인한다. 실제 트랙 데이터 기반 회귀는
// 대회 시뮬레이터 통합 이후 별도로 추가할 것.

#include <cmath>
#include <vector>

#include <gtest/gtest.h>

#include "kau_local_path_planner/bezier_ext.hpp"
#include "kau_local_path_planner/local_planner.hpp"

using kau::control::Curve;
using kau::local_path_planner::hermiteToBezier;
using kau::local_path_planner::LocalPlanner;
using kau::local_path_planner::Obstacle;
using kau::local_path_planner::PlannerParams;
using kau::local_path_planner::PlanStatus;
using kau::local_path_planner::Point2;
using kau::local_path_planner::RoadBoundary;

namespace
{

RoadBoundary wideOpenBoundary()
{
    RoadBoundary rb;
    rb.outer = {{-100.0, -1000.0}, {2100.0, -1000.0}, {2100.0, 1000.0},
               {-100.0, 1000.0}};
    rb.inner = {{-10000.0, -10000.0}, {-9999.0, -10000.0}, {-9999.0, -9999.0}};
    return rb;
}

Curve straightGlobalPath(double length)
{
    auto ctrl = hermiteToBezier(
        Point2{0.0, 0.0}, 0.0, 0.0, Point2{length, 0.0}, 0.0, 0.0,
        length, length);
    return Curve(std::vector<decltype(ctrl)>{ctrl}, false);
}

PlannerParams tunedParams()
{
    PlannerParams p;
    p.l_plan = 300.0;
    p.preview = 450.0;
    p.w_continuity = 10.0;
    return p;
}

}  // namespace

TEST(LocalPlannerScenarios, OpenStraightRoadReachesOkStatus)
{
    LocalPlanner planner(
        straightGlobalPath(2000.0), wideOpenBoundary(), {}, tunedParams(),
        /*kappa_max_vehicle=*/0.020221, /*body_radius_cm=*/11.0);

    const auto result = planner.plan(
        0.0, 0.0, 0.0, /*lane_curve=*/nullptr, /*lane_confidence=*/0.0f,
        /*now_sec=*/0.0);

    EXPECT_EQ(result.status, PlanStatus::kOk);
    ASSERT_TRUE(result.path.has_value());
    EXPECT_GT(result.clearance, 0.0);
}

TEST(LocalPlannerScenarios, HeadOnObstacleProducesNonCrashingResult)
{
    // 차량 정면 150cm, 도로 중앙에 반경 10cm 장애물 -- 반드시 옆으로
    // 피해야 한다. 최소한 크래시 없이 path 를 내거나(회피 성공), 정직하게
    // degraded/no_feasible 로 보고해야 한다 (조용히 장애물을 무시하면 안 됨).
    LocalPlanner planner(
        straightGlobalPath(2000.0), wideOpenBoundary(),
        {Obstacle{Point2{150.0, 0.0}, 10.0}}, tunedParams(),
        /*kappa_max_vehicle=*/0.020221, /*body_radius_cm=*/11.0);

    const auto result = planner.plan(
        0.0, 0.0, 0.0, /*lane_curve=*/nullptr, /*lane_confidence=*/0.0f,
        /*now_sec=*/0.0);

    if (result.status == PlanStatus::kOk)
    {
        // 회피에 성공했다면 실제로 옆으로 비켜났어야 한다.
        EXPECT_GT(std::abs(result.chosen_offset), 5.0);
        EXPECT_GE(result.clearance, 0.0);
    }
    else
    {
        EXPECT_TRUE(
            result.status == PlanStatus::kDegraded ||
            result.status == PlanStatus::kNoFeasible);
    }
}

// ====================================================================
// P0 앵커 (2026-08-25)
//
// 추종오차 e 를 P0 에 그대로 실으면 플래너가 e 를 첫 knot(0.25*l_plan
// =75cm) 안에 없애라고 요구한다 (추가 곡률 ~ 4e/d^2). 제어기의 횡오차
// 수렴은 그보다 느려서 오차가 줄기 전에 요구가 커지고, kappa(0)==kappa0
// 이므로 결국 kappa0 가 kappa_lim 을 넘어 **전 후보**가 탈락한다.
//
// 그래서 P0 를 이전 틱 경로 위 최근접점으로 옮긴다. 아래 테스트는 그
// 투영이 종방향(진행)은 남기고 횡방향(추종오차)만 버리는지 확인한다.
// ====================================================================

namespace
{

constexpr double kKappaMaxVehicle = 0.020221;
constexpr double kKappaMargin = 0.95;

double dist(const Point2 & a, const Point2 & b)
{
    return std::hypot(a.x - b.x, a.y - b.y);
}

// 반지름 radius 원호를 quintic 세그먼트 nseg 개로 근사 (좌회전).
Curve arcGlobalPath(double radius, double total_angle, int nseg)
{
    const double dphi = total_angle / nseg;
    auto first = hermiteToBezier(
        Point2{0.0, 0.0}, 0.0, 1.0 / radius, Point2{0.0, 0.0}, 0.0,
        1.0 / radius, 1.0, 1.0);
    std::vector<decltype(first)> segs;
    for (int i = 0; i < nseg; ++i)
    {
        const double a = i * dphi;
        const double b = (i + 1) * dphi;
        segs.push_back(hermiteToBezier(
            Point2{radius * std::sin(a), radius * (1.0 - std::cos(a))}, a,
            1.0 / radius,
            Point2{radius * std::sin(b), radius * (1.0 - std::cos(b))}, b,
            1.0 / radius, radius * dphi, radius * dphi));
    }
    return Curve(segs, false);
}

RoadBoundary wideBoxBoundary()
{
    RoadBoundary rb;
    rb.outer = {{-500.0, -500.0}, {1500.0, -500.0}, {1500.0, 1500.0},
               {-500.0, 1500.0}};
    rb.inner = {{-10000.0, -10000.0}, {-9999.0, -10000.0}, {-9999.0, -9999.0}};
    return rb;
}

}  // namespace

TEST(LocalPlannerAnchor, FirstTickUsesMeasuredPose)
{
    // 이전 경로가 없으면 앵커할 대상이 없다 -> 실측 그대로 (alpha=1).
    LocalPlanner planner(
        straightGlobalPath(2000.0), wideOpenBoundary(), {}, tunedParams(),
        kKappaMaxVehicle, /*body_radius_cm=*/11.0);

    const auto r = planner.plan(0.0, 0.0, 0.0, nullptr, 0.0f, 0.0);

    EXPECT_DOUBLE_EQ(r.anchor_alpha, 1.0);
    EXPECT_DOUBLE_EQ(r.anchor_e_cm, 0.0);
    EXPECT_DOUBLE_EQ(r.anchor_margin_cm, 0.0);
}

TEST(LocalPlannerAnchor, SmallLateralErrorIsDroppedFromP0)
{
    // 핵심 회귀: 추종오차가 작으면 발행 경로가 **자차가 아니라 이전 경로**
    // 에서 출발해야 한다. 이게 증폭 루프를 끊는 지점이다.
    LocalPlanner planner(
        straightGlobalPath(2000.0), wideOpenBoundary(), {}, tunedParams(),
        kKappaMaxVehicle, /*body_radius_cm=*/11.0);

    const auto first = planner.plan(0.0, 0.0, 0.0, nullptr, 0.0f, 0.0);
    ASSERT_TRUE(first.path.has_value());
    const Curve prev = *first.path;

    // 이전 경로 s=10cm 지점에서 법선으로 3cm (= lo 5cm 미만) 띄운 곳.
    const auto f = kau::local_path_planner::referenceFrame(prev, prev.wrapS(10.0));
    const Point2 n{-std::sin(f.heading), std::cos(f.heading)};
    constexpr double e = 3.0;
    const Point2 veh{f.point.x + e * n.x, f.point.y + e * n.y};

    const auto r = planner.plan(veh.x, veh.y, f.heading, nullptr, 0.0f, 0.1);
    ASSERT_TRUE(r.path.has_value());

    EXPECT_NEAR(r.anchor_e_cm, e, 0.2);
    EXPECT_DOUBLE_EQ(r.anchor_alpha, 0.0);
    EXPECT_NEAR(r.anchor_margin_cm, e, 0.2);   // 마진 보정 = (1-alpha)*e

    const Point2 start = r.path->point(0.0);
    EXPECT_NEAR(dist(start, f.point), 0.0, 0.3);   // 이전 경로 위
    EXPECT_NEAR(dist(start, veh), e, 0.3);         // 자차에서 e 만큼 떨어짐
}

TEST(LocalPlannerAnchor, LargeLateralErrorFallsBackToMeasuredPose)
{
    // 측위 점프/물리적 이탈. 앵커를 놓고 실측으로 복귀해야 한다.
    LocalPlanner planner(
        straightGlobalPath(2000.0), wideOpenBoundary(), {}, tunedParams(),
        kKappaMaxVehicle, /*body_radius_cm=*/11.0);

    const auto first = planner.plan(0.0, 0.0, 0.0, nullptr, 0.0f, 0.0);
    ASSERT_TRUE(first.path.has_value());
    const Curve prev = *first.path;

    const auto f = kau::local_path_planner::referenceFrame(prev, prev.wrapS(10.0));
    const Point2 n{-std::sin(f.heading), std::cos(f.heading)};
    constexpr double e = 30.0;   // hi(15cm) 초과
    const Point2 veh{f.point.x + e * n.x, f.point.y + e * n.y};

    const auto r = planner.plan(veh.x, veh.y, f.heading, nullptr, 0.0f, 0.1);
    ASSERT_TRUE(r.path.has_value());

    EXPECT_DOUBLE_EQ(r.anchor_alpha, 1.0);
    EXPECT_NEAR(r.anchor_margin_cm, 0.0, 1e-9);
    EXPECT_NEAR(dist(r.path->point(0.0), veh), 0.0, 1e-6);   // 자차에서 출발
}

TEST(LocalPlannerAnchor, StalePreviousPathFallsBackToMeasuredPose)
{
    // 나이 가드. previous_path_ 는 실패한 틱에 갱신되지 않으므로 이 가드가
    // 없으면 몇 초 전 경로가 계속 기준으로 남는다.
    LocalPlanner planner(
        straightGlobalPath(2000.0), wideOpenBoundary(), {}, tunedParams(),
        kKappaMaxVehicle, /*body_radius_cm=*/11.0);

    const auto first = planner.plan(0.0, 0.0, 0.0, nullptr, 0.0f, 0.0);
    ASSERT_TRUE(first.path.has_value());
    const Curve prev = *first.path;

    const auto f = kau::local_path_planner::referenceFrame(prev, prev.wrapS(10.0));
    const Point2 n{-std::sin(f.heading), std::cos(f.heading)};
    const Point2 veh{f.point.x + 3.0 * n.x, f.point.y + 3.0 * n.y};

    // anchor_max_age_sec = 0.5 를 넘긴 시각.
    const auto r = planner.plan(veh.x, veh.y, f.heading, nullptr, 0.0f, 10.0);
    ASSERT_TRUE(r.path.has_value());

    EXPECT_DOUBLE_EQ(r.anchor_alpha, 1.0);
    EXPECT_NEAR(dist(r.path->point(0.0), veh), 0.0, 1e-6);
}

TEST(LocalPlannerAnchor, Kappa0ComesFromPreviousPathCurvature)
{
    // kappa0 의 출처가 제어기 조향각이 아니라 이전 경로 곡률인지 확인한다.
    // 곡선 도로라야 0 과 구분된다.
    LocalPlanner planner(
        // 호 길이 750cm -- l_plan(300cm) + 20틱 이동(300cm) 보다 길어야
        // 자차가 경로 끝에 닿지 않는다. kappa=0.0033 (kappa_max 의 16%).
        arcGlobalPath(/*radius=*/300.0, /*total_angle=*/2.5, /*nseg=*/10),
        wideBoxBoundary(), {}, tunedParams(),
        kKappaMaxVehicle, /*body_radius_cm=*/11.0);

    const auto first = planner.plan(0.0, 0.0, 0.0, nullptr, 0.0f, 0.0);
    ASSERT_TRUE(first.path.has_value());
    const Curve prev = *first.path;

    // 자차를 이전 경로 위에 정확히 둔다 (e~0 -> alpha=0).
    const auto f = kau::local_path_planner::referenceFrame(prev, prev.wrapS(15.0));
    const auto r = planner.plan(f.point.x, f.point.y, f.heading, nullptr, 0.0f, 0.1);

    EXPECT_DOUBLE_EQ(r.anchor_alpha, 0.0);

    // 구현과 같은 방식으로 기대값을 만든다 (nearestGlobal 의 s* 를 그대로).
    const auto st = prev.nearestGlobal(f.point);
    ASSERT_TRUE(st.valid);
    EXPECT_NEAR(r.kappa0, prev.kappa(prev.wrapS(st.s)), 1e-9);

    // 곡선이므로 0 이 아니어야 한다 (상수 0 을 넣는 회귀 방지).
    EXPECT_GT(std::abs(r.kappa0), 1e-4);
}

TEST(LocalPlannerAnchor, Kappa0NeverExceedsKappaLimOverRepeatedTicks)
{
    // kappa(0) == kappa0 이므로 kappa0 > kappa_lim 이면 전 후보가
    // kappa_bound 로 탈락한다. 곡선 위에서 오차를 유지하며 반복해도
    // 그 선을 넘지 않아야 한다 (기존 /steering 방식의 실패 모드).
    LocalPlanner planner(
        // 호 길이 750cm -- l_plan(300cm) + 20틱 이동(300cm) 보다 길어야
        // 자차가 경로 끝에 닿지 않는다. kappa=0.0033 (kappa_max 의 16%).
        arcGlobalPath(/*radius=*/300.0, /*total_angle=*/2.5, /*nseg=*/10),
        wideBoxBoundary(), {}, tunedParams(),
        kKappaMaxVehicle, /*body_radius_cm=*/11.0);

    constexpr double kKappaLim = kKappaMaxVehicle * kKappaMargin;
    constexpr double e = 3.0;   // 매 틱 유지되는 추종오차 (lo 미만)

    auto r = planner.plan(0.0, 0.0, 0.0, nullptr, 0.0f, 0.0);
    ASSERT_TRUE(r.path.has_value());

    for (int tick = 1; tick <= 20; ++tick)
    {
        const Curve prev = *r.path;
        // 한 틱에 15cm 전진 + 법선으로 e 만큼 밀린 상태를 유지한다.
        const auto f =
            kau::local_path_planner::referenceFrame(prev, prev.wrapS(15.0));
        const Point2 n{-std::sin(f.heading), std::cos(f.heading)};
        const Point2 veh{f.point.x + e * n.x, f.point.y + e * n.y};

        r = planner.plan(
            veh.x, veh.y, f.heading, nullptr, 0.0f, 0.1 * tick);

        ASSERT_TRUE(r.path.has_value()) << "tick " << tick;
        EXPECT_LE(std::abs(r.kappa0), kKappaLim + 1e-12) << "tick " << tick;
        // 오차가 계획에 실리지 않으므로 요구 곡률이 발산하지 않는다.
        EXPECT_LE(r.kappa_max, kKappaLim + 1e-9) << "tick " << tick;
        EXPECT_EQ(r.status, PlanStatus::kOk) << "tick " << tick;
    }
}
