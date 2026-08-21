// ====================================================================
// test_curve.cpp
//
// 포팅 검증. **레퍼런스 구현과 같은 입력에 같은 값이 나오는지** 대조한다.
//
// 기준값 출처
//     KAU_AMET_Test / src/sim_common/curve.py 를 아래 경로에 대해 실행한 결과.
//     재생성:
//         cd /home/physicar/KAU_AMET_Test/src && python3 -c "
//         import sys, math; sys.path.insert(0,'.')
//         import numpy as np; from sim_common import curve
//         r,k = 150.0, 1.0/150.0
//         knots=[curve.Knot(np.array([r*math.sin((math.pi/2)*i/3),
//                                     r*(1-math.cos((math.pi/2)*i/3))]),
//                           (math.pi/2)*i/3, k) for i in range(4)]
//         cv=curve.Curve.from_knots(knots, closed=False, optimize=False)
//         ..."
//
// 경로: 반경 150 cm 사분원호를 knot 4개(= 3 segment)로 근사. G2 공유.
// ====================================================================

#include <cmath>
#include <vector>

#include <gtest/gtest.h>

#include "kau_control/curve.hpp"
#include "kau_control/params.hpp"
#include "kau_control/pure_pursuit.hpp"

using kau::bezier::Point2;
using kau::control::Curve;
using kau::control::TrackState;

namespace
{

// 레퍼런스가 뱉은 제어점 (cm)
Curve makeQuarterArc()
{
    const std::vector<std::vector<Point2>> raw = {
        {
            {0.000000000000, 0.000000000000},
            {15.529142706151, 0.000000000000},
            {31.058285412302, 2.009618943233},
            {47.097926363342, 6.307427782950},
            {61.551367917479, 12.331618079259},
            {75.000000000000, 20.096189432334},
        },
        {
            {75.000000000000, 20.096189432334},
            {88.448632082521, 27.860760785410},
            {100.892454693425, 37.365713195252},
            {112.634286804748, 49.107545306575},
            {122.139239214590, 61.551367917479},
            {129.903810567666, 75.000000000000},
        },
        {
            {129.903810567666, 75.000000000000},
            {137.668381920741, 88.448632082521},
            {143.692572217050, 102.902073636658},
            {147.990381056767, 118.941714587697},
            {150.000000000000, 134.470857293849},
            {150.000000000000, 150.000000000000},
        },
    };

    return Curve(raw, false);
}

// 레퍼런스 출력값. 주석의 python 스니펫이 그대로 뱉은 숫자다.
constexpr double REF_LENGTH     = 235.587729086728;
constexpr double REF_SEG_LEN    = 78.529243028909;
constexpr double REF_KAPPA_MAX  = 0.006805863496;

constexpr double QX             = 100.0;    // 질의점 (경로 바깥)
constexpr double QY             = 20.0;

constexpr int    REF_NG_SEG     = 1;
constexpr double REF_NG_U       = 0.253655139909;
constexpr double REF_NG_S       = 98.318494167483;
constexpr double REF_NG_DIST    = 14.031358478690;

constexpr double REF_PT_X       = 91.425515306078;
constexpr double REF_PT_Y       = 31.106630136598;
constexpr double REF_HEADING    = 0.657442453824;
constexpr double REF_KAPPA_AT_S = 0.006695035773;

constexpr double REF_YAW        = 0.3;
constexpr double REF_CTE        = -14.031358478690;
constexpr double REF_HEAD_ERR   = -0.357442453824;

constexpr double REF_LA_S       = 148.318494167483;
constexpr double REF_LA_X       = 125.313486780281;
constexpr double REF_LA_Y       = 67.563998479181;

constexpr double REF_KMO        = 0.006805863496;

// 호길이는 Gauss-Legendre 10점이라 100 cm 당 27 um. 그 이상은 포팅 오류다.
constexpr double TOL = 1e-6;

}  // namespace


TEST(Curve, GeometryMatchesReference)
{
    const Curve cv = makeQuarterArc();

    EXPECT_EQ(cv.nseg(), 3);

    EXPECT_FALSE(cv.closed());

    EXPECT_NEAR(cv.length(), REF_LENGTH, TOL);

    for (int i = 0; i < cv.nseg(); ++i)
    {
        EXPECT_NEAR(cv.segLen(i), REF_SEG_LEN, TOL) << "segment " << i;
    }

    EXPECT_NEAR(cv.kappaMax(), REF_KAPPA_MAX, TOL);
}


TEST(Curve, G2ContinuityHolds)
{
    // knot 이 (theta, kappa) 를 공유하면 이음매가 연립방정식 없이 이어진다.
    const Curve cv = makeQuarterArc();

    double dtheta = 0.0;

    double dkappa = 0.0;

    cv.g2Error(dtheta, dkappa);

    EXPECT_NEAR(dtheta, 0.0, 1e-9);

    EXPECT_NEAR(dkappa, 0.0, 1e-9);

    EXPECT_TRUE(cv.isRegular());
}


TEST(Curve, NearestGlobalMatchesReference)
{
    // 8.3 전역 탐색. 주행 시작 / 복구 시 1회.
    const Curve cv = makeQuarterArc();

    const TrackState st = cv.nearestGlobal(Point2{QX, QY});

    ASSERT_TRUE(st.valid);

    EXPECT_EQ(st.seg, REF_NG_SEG);

    EXPECT_NEAR(st.u, REF_NG_U, TOL);

    EXPECT_NEAR(st.s, REF_NG_S, TOL);

    EXPECT_NEAR(st.dist, REF_NG_DIST, TOL);
}


TEST(Curve, PointHeadingKappaMatchReference)
{
    const Curve cv = makeQuarterArc();

    const Point2 p = cv.point(REF_NG_S);

    EXPECT_NEAR(p.x, REF_PT_X, TOL);

    EXPECT_NEAR(p.y, REF_PT_Y, TOL);

    EXPECT_NEAR(cv.heading(REF_NG_S), REF_HEADING, TOL);

    EXPECT_NEAR(cv.kappa(REF_NG_S), REF_KAPPA_AT_S, TOL);
}


TEST(Curve, ErrorsMatchReference)
{
    const Curve cv = makeQuarterArc();

    const TrackState st = cv.nearestGlobal(Point2{QX, QY});

    double cte = 0.0;

    double head_err = 0.0;

    cv.errors(QX, QY, REF_YAW, st, cte, head_err);

    EXPECT_NEAR(cte, REF_CTE, TOL);

    EXPECT_NEAR(head_err, REF_HEAD_ERR, TOL);

    // 부호 규약: 좌측 +. 질의점이 곡선 안쪽(우측)이므로 음수여야 한다.
    EXPECT_LT(cte, 0.0);
}


TEST(Curve, LookaheadMatchesReference)
{
    // 8.5 LookAhead. 호길이 기준 전진.
    const Curve cv = makeQuarterArc();

    const TrackState st = cv.nearestGlobal(Point2{QX, QY});

    const double la_s = cv.lookahead(st.s, 50.0);

    EXPECT_NEAR(la_s, REF_LA_S, TOL);

    const Point2 la = cv.point(la_s);

    EXPECT_NEAR(la.x, REF_LA_X, TOL);

    EXPECT_NEAR(la.y, REF_LA_Y, TOL);
}


TEST(Curve, LookaheadClampsOnOpenCurve)
{
    // 개곡선은 단부 clamp. 종점 너머를 요구해도 종점을 준다.
    const Curve cv = makeQuarterArc();

    EXPECT_NEAR(cv.lookahead(200.0, 1000.0), cv.length(), TOL);

    EXPECT_NEAR(cv.lookahead(10.0, -1000.0), 0.0, TOL);
}


TEST(Curve, KappaMaxOverMatchesReference)
{
    // 8.6 구간 최대 곡률. Speed Controller 가 쓴다.
    const Curve cv = makeQuarterArc();

    const TrackState st = cv.nearestGlobal(Point2{QX, QY});

    EXPECT_NEAR(cv.kappaMaxOver(st.s + 20.0, st.s + 100.0), REF_KMO, TOL);

    // 구간 폭 0 이면 그 점의 곡률
    EXPECT_NEAR(
        cv.kappaMaxOver(REF_NG_S, REF_NG_S),
        std::abs(REF_KAPPA_AT_S), TOL);

    // 전 구간을 덮으면 전체 최대와 같다
    EXPECT_NEAR(cv.kappaMaxOver(0.0, cv.length()), cv.kappaMax(), TOL);
}


TEST(Curve, WindowTrackingAgreesWithGlobal)
{
    // 8.4 국소 window 가 전역 탐색과 같은 해를 준다 (경로를 따라 전진하는 동안).
    const Curve cv = makeQuarterArc();

    TrackState st;   // valid=false -> 첫 질의는 전역

    for (int i = 0; i <= 40; ++i)
    {
        const double s = cv.length() * i / 40.0;

        // 경로에서 좌측으로 5 cm 떨어진 점을 따라간다
        const Point2 c = cv.point(s);

        const double th = cv.heading(s);

        const Point2 q{c.x - 5.0 * std::sin(th), c.y + 5.0 * std::cos(th)};

        st = cv.nearest(q, st);

        const TrackState g = cv.nearestGlobal(q);

        ASSERT_TRUE(st.valid);

        EXPECT_NEAR(st.s, g.s, 1e-6) << "i=" << i;

        EXPECT_NEAR(st.dist, 5.0, 1e-3) << "i=" << i;

        EXPECT_EQ(st.fails, 0);
    }
}


TEST(Curve, WindowRecoversAfterGateViolation)
{
    // GATE(250 cm) 밖 질의가 FAIL_LIMIT(3) 회 연속되면 전역 재탐색으로 복구.
    const Curve cv = makeQuarterArc();

    TrackState st = cv.nearestGlobal(Point2{0.0, 0.0});

    ASSERT_TRUE(st.valid);

    const Point2 far{5000.0, 5000.0};

    st = cv.nearest(far, st);

    EXPECT_EQ(st.fails, 1);

    st = cv.nearest(far, st);

    EXPECT_EQ(st.fails, 2);

    st = cv.nearest(far, st);

    // 3회째 -> 전역 재탐색 수행 후 fails 리셋
    EXPECT_EQ(st.fails, 0);

    EXPECT_TRUE(st.valid);
}


TEST(Curve, ClosedCurveWrapsArcLength)
{
    // 폐곡선 wrap + 최소표현 부호거리 (8.4). nseg >= 3 이어야 성립.
    const Curve cv = makeQuarterArc();

    const Curve loop(cv.ctrl(), true);

    EXPECT_TRUE(loop.closed());

    EXPECT_TRUE(loop.windowSafe());

    const double len = loop.length();

    EXPECT_NEAR(loop.wrapS(len + 10.0), 10.0, TOL);

    EXPECT_NEAR(loop.wrapS(-10.0), len - 10.0, TOL);

    // 종점 너머 lookahead 가 시점으로 순환한다 (개곡선과 다른 점)
    EXPECT_NEAR(loop.lookahead(len - 5.0, 20.0), 15.0, TOL);

    // 최소표현: 뒤로 10 은 -10 이지 len-10 이 아니다
    EXPECT_NEAR(loop.deltaS(20.0, 10.0), -10.0, TOL);

    EXPECT_NEAR(loop.deltaS(len - 5.0, 5.0), 10.0, TOL);
}


TEST(Curve, WindowSafeRejectsTooFewSegments)
{
    // 폐곡선 segment 1개가 전장의 절반 이상이면 최소표현이 뒤집힌다.
    // 레퍼런스 실측: nseg=2 에서 전역 탐색 대비 불일치 29%.
    const Curve cv = makeQuarterArc();

    std::vector<std::vector<Point2>> two = {cv.ctrl()[0], cv.ctrl()[1]};

    EXPECT_FALSE(Curve(two, true).windowSafe());

    EXPECT_TRUE(Curve(two, false).windowSafe());   // 개곡선은 무관
}


// ====================================================================
// Pure Pursuit  (controller.py 대조)
// ====================================================================

TEST(PurePursuit, LookaheadDistanceClamps)
{
    kau::control::ControllerParams p;   // k_v=0.5, 30~175 cm

    EXPECT_NEAR(kau::control::pure_pursuit::lookaheadDistance(1.0, p),
                50.0, 1e-12);           // 0.5 * 1.0 * 100

    EXPECT_NEAR(kau::control::pure_pursuit::lookaheadDistance(0.1, p),
                p.ld_min, 1e-12);       // 5 cm -> 하한

    EXPECT_NEAR(kau::control::pure_pursuit::lookaheadDistance(5.0, p),
                p.ld_max, 1e-12);       // 250 cm -> 상한
}


TEST(PurePursuit, SteerCommandGeometry)
{
    kau::control::VehicleParams v;      // wheelbase 18 cm

    // 정면 목표 -> 조향 0
    EXPECT_NEAR(
        kau::control::pure_pursuit::steerCommand(
            0.0, 0.0, 0.0, 50.0, 0.0, 50.0, v),
        0.0, 1e-12);

    // 좌측 목표 -> 좌회전(+)
    EXPECT_GT(
        kau::control::pure_pursuit::steerCommand(
            0.0, 0.0, 0.0, 50.0, 10.0, 50.0, v),
        0.0);

    // 우측 목표 -> 우회전(-)
    EXPECT_LT(
        kau::control::pure_pursuit::steerCommand(
            0.0, 0.0, 0.0, 50.0, -10.0, 50.0, v),
        0.0);

    // delta = atan(2 L sin(alpha) / Ld). alpha = 30deg, Ld = 50
    const double expect =
        std::atan2(2.0 * 18.0 * std::sin(M_PI / 6.0), 50.0) * 180.0 / M_PI;

    EXPECT_NEAR(
        kau::control::pure_pursuit::steerCommand(
            0.0, 0.0, 0.0,
            50.0 * std::cos(M_PI / 6.0), 50.0 * std::sin(M_PI / 6.0),
            50.0, v),
        expect, 1e-9);
}


TEST(VehicleParams, GeometryConstants)
{
    kau::control::VehicleParams v;

    // R_min = L / tan(delta_max) = 18 / tan(20deg)
    EXPECT_NEAR(v.r_min(), 18.0 / std::tan(20.0 * M_PI / 180.0), 1e-9);

    EXPECT_NEAR(v.kappa_max(), 1.0 / v.r_min(), 1e-12);

    // steer_of_kappa 와 kappa_of_steer 는 서로 역함수
    EXPECT_NEAR(v.kappa_of_steer(v.steer_of_kappa(0.005)), 0.005, 1e-12);

    // 기준점 보정 기본값이 -wheelbase/2 여야 한다 (PhysiCar URDF 기준).
    // 이 값이 0 이면 코너에서 계속 안쪽으로 파고든다.
    EXPECT_NEAR(v.rear_axle_offset, -v.wheelbase / 2.0, 1e-12);
}


TEST(SpeedParams, CurvatureToTargetSpeed)
{
    kau::control::VehicleParams v;

    kau::control::SpeedParams s;

    // 직선 -> v_max
    EXPECT_NEAR(s.target(0.0, v), s.v_max, 1e-12);

    // kappa_ref -> v_min (두 law 모두 이 기준점을 지난다)
    EXPECT_NEAR(s.target(s.kappa_ref(v), v), s.v_min, 1e-9);

    s.use_sqrt = false;

    EXPECT_NEAR(s.target(s.kappa_ref(v), v), s.v_min, 1e-9);

    // 곡률이 커질수록 느려진다 (단조)
    s.use_sqrt = true;

    EXPECT_GT(s.target(0.001, v), s.target(0.005, v));

    // 범위 밖은 clamp
    EXPECT_NEAR(s.target(1.0, v), s.v_min, 1e-12);
}
