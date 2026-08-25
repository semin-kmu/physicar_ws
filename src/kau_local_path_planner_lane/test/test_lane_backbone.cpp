// ====================================================================
// test_lane_backbone.cpp (2026-08-25)
//
// extendCurve(등곡률 외삽) 회귀 방지. 원래 코드는 tail 점을 시작점에서
// **호길이** remain 만큼 떨어진 곳에 놓았는데, 등곡률 원호의 현 길이는
// 2*sin(dth/2)/kappa 라 회전각이 커질수록 어긋난다(126deg 에서 +23%).
// 그 결과 backbone 곡률이 차량 한계를 넘어 곡선 구간에서 전 후보가
// kappa_bound 로 탈락했다.
//
// 여기 테스트는 "외삽 구간이 진짜 원호 위에 있는가" 를 절대 기하로 잰다.
// 옛 구현은 case ExtrapolatesSharpArcAccurately 에서 22cm 넘게 벗어나
// 반드시 실패한다.
// ====================================================================

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "kau_local_path_planner_lane/lane_backbone.hpp"
#include "kau_local_path_planner_lane/candidate_generator.hpp"

using kau::bezier::Ctrl;
using kau::bezier::Point2;
using kau::control::Curve;
using kau::local_path_planner_lane::extendCurve;
using kau::local_path_planner_lane::fitSegment;
using kau::local_path_planner_lane::Knot;

namespace
{

// 중심 (0, R) 인 원 위의 점. s=0 에서 원점, heading 0.
Point2 arcPoint(double radius, double s)
{
    const double th = s / radius;
    return Point2{radius * std::sin(th), radius - radius * std::cos(th)};
}

// s_begin..s_end 를 nseg 개 quintic 으로 적합한 원호.
Curve arcCurve(double radius, double s_begin, double s_end, int nseg)
{
    std::vector<Ctrl> segs;
    for (int i = 0; i < nseg; ++i)
    {
        const double sa = s_begin + (s_end - s_begin) * i / nseg;
        const double sb = s_begin + (s_end - s_begin) * (i + 1) / nseg;
        const Knot a{arcPoint(radius, sa), sa / radius, 1.0 / radius};
        const Knot b{arcPoint(radius, sb), sb / radius, 1.0 / radius};
        const auto [seg, kb] = fitSegment(a, b, 2);
        EXPECT_TRUE(seg.has_value());
        if (seg)
        {
            segs.push_back(*seg);
        }
    }
    return Curve(std::move(segs), false);
}

// 곡선 위 점들이 중심 (0,R) 반지름 R 인 원에서 벗어난 최대 거리 [cm].
double maxRadialError(const Curve & cv, double radius)
{
    double worst = 0.0;
    const double len = cv.length();
    for (double s = 0.0; s <= len; s += 2.0)
    {
        const Point2 p = cv.point(s);
        worst = std::max(
            worst, std::abs(std::hypot(p.x, p.y - radius) - radius));
    }
    return worst;
}

}  // namespace

// 이미 target 보다 길면 손대지 않는다.
TEST(LaneBackboneExtend, LeavesLongEnoughCurveUntouched)
{
    const Curve cv = arcCurve(200.0, 0.0, 150.0, 4);
    const Curve out = extendCurve(cv, 100.0, 2);
    EXPECT_EQ(out.nseg(), cv.nseg());
    EXPECT_NEAR(out.length(), cv.length(), 1e-9);
}

// 직선은 외삽해도 직선이고 길이가 정확히 target 이다.
TEST(LaneBackboneExtend, ExtrapolatesStraightExactly)
{
    std::vector<Ctrl> segs;
    const Knot a{Point2{0.0, 0.0}, 0.0, 0.0};
    const Knot b{Point2{80.0, 0.0}, 0.0, 0.0};
    const auto [seg, kb] = fitSegment(a, b, 2);
    ASSERT_TRUE(seg.has_value());
    segs.push_back(*seg);
    const Curve cv(std::move(segs), false);

    const Curve out = extendCurve(cv, 300.0, 2);
    EXPECT_NEAR(out.length(), 300.0, 1.0);
    for (double s = 0.0; s <= out.length(); s += 5.0)
    {
        EXPECT_NEAR(out.point(s).y, 0.0, 1e-6) << "s=" << s;
    }
}

// ★ 회귀 본체. R=60cm 를 168 -> 300cm 로 늘리면 회전각이 126deg 다.
// 옛 구현(호길이를 현 길이로 씀)은 여기서 22cm 넘게 벗어나고 곡률이
// 0.01667 -> 0.02255 로 부풀어 차량 한계(0.020221)를 넘겼다.
TEST(LaneBackboneExtend, ExtrapolatesSharpArcAccurately)
{
    constexpr double kRadius = 60.0;
    constexpr double kKappaMaxVehicle = 0.020221;

    const Curve cv = arcCurve(kRadius, 32.0, 168.0, 6);
    const Curve out = extendCurve(cv, 300.0, 2);

    // 외삽 구간이 진짜 원호 위에 있어야 한다.
    EXPECT_LT(maxRadialError(out, kRadius), 2.0)
        << "외삽이 원호에서 벗어났다 -- chord 길이를 호길이로 쓰고 있지 않은지 확인";

    // 곡률이 참값(1/60=0.016667)에서 크게 부풀면 안 된다. kappaMax 는
    // 이산 샘플 기반이라 약간의 여유만 준다.
    EXPECT_LT(out.kappaMax(), kKappaMaxVehicle)
        << "외삽 곡률이 차량 한계를 넘었다 -- 이 상태면 전 후보가 kappa_bound 로 탈락한다";

    // 길이도 target 근처여야 한다 (옛 구현은 330cm 가 나왔다).
    EXPECT_NEAR(out.length(), 300.0, 10.0);
}

// 회전각이 크면 세그먼트 하나로 붙이지 않고 쪼갠다.
TEST(LaneBackboneExtend, SplitsLargeTurnIntoMultipleSegments)
{
    const Curve cv = arcCurve(60.0, 32.0, 168.0, 6);
    const Curve out = extendCurve(cv, 300.0, 2);
    // remain=132cm, kappa=1/60 -> 126deg. 45deg/세그먼트면 3개가 붙는다.
    EXPECT_GE(out.nseg(), cv.nseg() + 2);
}

// 완만한 곡선은 예전처럼 세그먼트 하나로 끝난다 (불필요한 분할 금지).
TEST(LaneBackboneExtend, KeepsGentleCurveSingleSegment)
{
    const Curve cv = arcCurve(600.0, 32.0, 168.0, 4);
    const Curve out = extendCurve(cv, 300.0, 2);
    // remain=132cm, kappa=1/600 -> 12.6deg < 45deg.
    EXPECT_EQ(out.nseg(), cv.nseg() + 1);
    EXPECT_LT(maxRadialError(out, 600.0), 1.0);
}
