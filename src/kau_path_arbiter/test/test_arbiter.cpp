#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "kau_path_arbiter/arbiter.hpp"

using kau::path_arbiter::Arbiter;
using kau::path_arbiter::ArbiterParams;
using kau::path_arbiter::Decision;
using kau::path_arbiter::Mode;
using kau::path_arbiter::obstacleRelevant;
using kau::path_arbiter::Source;

namespace
{
ArbiterParams defaults()
{
    ArbiterParams p;
    p.trigger_min_x_cm = 0.0;
    p.trigger_max_x_cm = 150.0;
    p.trigger_half_width_cm = 40.0;
    p.engage_sec = 0.0;
    p.release_sec = 2.0;
    return p;
}
}  // namespace

// ---------------- 관심 영역 ----------------

TEST(ObstacleRelevant, AcceptsObstacleAhead)
{
    EXPECT_TRUE(obstacleRelevant(80.0, 0.0, 10.0, defaults()));
}

TEST(ObstacleRelevant, RejectsBehind)
{
    EXPECT_FALSE(obstacleRelevant(-50.0, 0.0, 10.0, defaults()));
}

TEST(ObstacleRelevant, RejectsTooFar)
{
    EXPECT_FALSE(obstacleRelevant(200.0, 0.0, 10.0, defaults()));
}

TEST(ObstacleRelevant, RejectsFarToTheSide)
{
    EXPECT_FALSE(obstacleRelevant(80.0, 90.0, 10.0, defaults()));
}

// 중심만 보면 경계 밖이지만 반지름 때문에 걸치는 경우 -- 놓치면 안 된다.
TEST(ObstacleRelevant, AcceptsLargeObstacleStraddlingBoundary)
{
    EXPECT_TRUE(obstacleRelevant(80.0, 45.0, 10.0, defaults()));
    EXPECT_TRUE(obstacleRelevant(160.0, 0.0, 15.0, defaults()));
}

TEST(ObstacleRelevant, RejectsNonFinite)
{
    const double nan = std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(obstacleRelevant(nan, 0.0, 10.0, defaults()));
    EXPECT_FALSE(obstacleRelevant(80.0, nan, 10.0, defaults()));
}

// ---------------- 기본 중재 ----------------

TEST(Arbiter, DefaultsToLaneWhenNoObstacle)
{
    Arbiter a(defaults());
    a.updateObstacles(true, false, 1.0);
    const Decision d = a.decide(true, true, 1.0);
    EXPECT_EQ(d.mode, Mode::kLane);
    EXPECT_EQ(d.source, Source::kLane);
}

TEST(Arbiter, SwitchesToLocalImmediatelyOnObstacle)
{
    Arbiter a(defaults());
    a.updateObstacles(true, true, 1.0);
    const Decision d = a.decide(true, true, 1.0);
    EXPECT_EQ(d.mode, Mode::kAvoid);
    EXPECT_EQ(d.source, Source::kLocal);
}

// 해제는 release_sec 이 지나야 한다. 그 전에 돌아가면 안 된다.
TEST(Arbiter, HoldsAvoidForReleaseWindow)
{
    Arbiter a(defaults());
    a.updateObstacles(true, true, 1.0);
    ASSERT_EQ(a.decide(true, true, 1.0).mode, Mode::kAvoid);

    a.updateObstacles(true, false, 2.0);
    EXPECT_EQ(a.decide(true, true, 2.0).mode, Mode::kAvoid) << "1.0s 만에 풀렸다";

    a.updateObstacles(true, false, 2.9);
    EXPECT_EQ(a.decide(true, true, 2.9).mode, Mode::kAvoid) << "1.9s 만에 풀렸다";

    a.updateObstacles(true, false, 3.0);
    EXPECT_EQ(a.decide(true, true, 3.0).mode, Mode::kLane);
}

// 깜빡이는 장애물이 소스를 오가게 만들면 안 된다 (조향이 튄다).
TEST(Arbiter, DoesNotChatterOnFlickeringDetection)
{
    Arbiter a(defaults());
    int switches = 0;
    Mode prev = Mode::kLane;
    // 10Hz 로 한 프레임 걸러 검출/미검출이 반복되는 상황을 3 초.
    for (int i = 0; i < 30; ++i)
    {
        const double t = 1.0 + 0.1 * i;
        a.updateObstacles(true, (i % 2) == 0, t);
        const Mode m = a.decide(true, true, t).mode;
        if (m != prev)
        {
            ++switches;
            prev = m;
        }
    }
    EXPECT_EQ(switches, 1) << "LANE->AVOID 한 번이어야 한다 (실제 " << switches << ")";
}

// ---------------- 인지 두절 ----------------

// status != OK 는 "장애물 없음" 이 아니라 "모름" 이다. 회피 중이었다면
// 모드를 유지해야 한다 -- 이걸 clear 로 읽으면 장애물 앞에서 차선 모드로
// 내려간다.
TEST(Arbiter, InvalidObservationDoesNotReleaseAvoid)
{
    Arbiter a(defaults());
    a.updateObstacles(true, true, 1.0);
    ASSERT_EQ(a.decide(true, true, 1.0).mode, Mode::kAvoid);

    for (int i = 1; i <= 100; ++i)   // 10 초 동안 인지 두절
    {
        const double t = 1.0 + 0.1 * i;
        a.updateObstacles(false, false, t);
        EXPECT_EQ(a.decide(true, true, t).mode, Mode::kAvoid)
            << "인지 두절을 '장애물 없음' 으로 읽었다 (t=" << t << ")";
    }
}

TEST(Arbiter, InvalidObservationDoesNotEngageAvoid)
{
    Arbiter a(defaults());
    a.updateObstacles(false, false, 1.0);
    EXPECT_EQ(a.decide(true, true, 1.0).mode, Mode::kLane);
}

// ---------------- 소스 두절 ----------------

TEST(Arbiter, PublishesNothingWhenLaneStale)
{
    Arbiter a(defaults());
    a.updateObstacles(true, false, 1.0);
    const Decision d = a.decide(false, true, 1.0);
    EXPECT_EQ(d.mode, Mode::kLane);
    EXPECT_EQ(d.source, Source::kNone) << "차선이 끊겼는데 회피 경로로 내려갔다";
}

// 장애물이 있는데 /path/local 이 끊기면, 장애물을 모르는 차선 경로로
// 내려가지 않고 멈춘다 (제어기 timeout).
TEST(Arbiter, DoesNotFallBackToLaneWhileAvoidingByDefault)
{
    Arbiter a(defaults());
    a.updateObstacles(true, true, 1.0);
    const Decision d = a.decide(true, false, 1.0);
    EXPECT_EQ(d.mode, Mode::kAvoid);
    EXPECT_EQ(d.source, Source::kNone);
}

TEST(Arbiter, FallsBackToLaneWhenExplicitlyEnabled)
{
    ArbiterParams p = defaults();
    p.fallback_to_lane_on_local_loss = true;
    Arbiter a(p);
    a.updateObstacles(true, true, 1.0);
    const Decision d = a.decide(true, false, 1.0);
    EXPECT_EQ(d.source, Source::kLane);
}

// ---------------- 진입 디바운스 ----------------

TEST(Arbiter, EngageDebounceDelaysSwitch)
{
    ArbiterParams p = defaults();
    p.engage_sec = 0.3;
    Arbiter a(p);
    a.updateObstacles(true, true, 1.0);
    EXPECT_EQ(a.decide(true, true, 1.0).mode, Mode::kLane);
    a.updateObstacles(true, true, 1.2);
    EXPECT_EQ(a.decide(true, true, 1.2).mode, Mode::kLane);
    a.updateObstacles(true, true, 1.3);
    EXPECT_EQ(a.decide(true, true, 1.3).mode, Mode::kAvoid);
}

// 디바운스 중에 검출이 끊기면 카운터가 리셋되어야 한다.
TEST(Arbiter, EngageDebounceResetsOnGap)
{
    ArbiterParams p = defaults();
    p.engage_sec = 0.3;
    Arbiter a(p);
    a.updateObstacles(true, true, 1.0);
    a.updateObstacles(true, false, 1.1);
    a.updateObstacles(true, true, 1.2);
    EXPECT_EQ(a.decide(true, true, 1.2).mode, Mode::kLane);
    a.updateObstacles(true, true, 1.5);
    EXPECT_EQ(a.decide(true, true, 1.5).mode, Mode::kAvoid);
}
