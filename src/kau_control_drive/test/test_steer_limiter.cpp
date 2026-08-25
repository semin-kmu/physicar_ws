#include <gtest/gtest.h>

#include <cmath>

#include "kau_control_drive/steer_limiter.hpp"

using kau::control_drive::SteerLimiter;
using kau::control_drive::SteerLimiterParams;

namespace
{
SteerLimiterParams defaults()
{
    SteerLimiterParams p;
    p.max_steer_deg = 20.0;
    p.max_steer_rate_dps = 600.0;
    p.transition_rate_dps = 150.0;
    p.transition_sec = 0.4;
    return p;
}
constexpr double kDt = 0.02;   // 50Hz
}  // namespace

// 작은 변화는 그대로 통과해야 한다 (슬루가 정상 조향을 방해하면 안 된다).
TEST(SteerLimiter, PassesSmallChangeUnchanged)
{
    SteerLimiter L(defaults());
    // 600deg/s * 0.02s = 12deg 까지는 한 tick 에 통과.
    EXPECT_NEAR(L.apply(5.0, kDt, false, 0.0), 5.0, 1e-9);
}

TEST(SteerLimiter, ClampsToMaxSteer)
{
    SteerLimiter L(defaults());
    for (int i = 0; i < 100; ++i)
    {
        L.apply(90.0, kDt, false, i * kDt);
    }
    EXPECT_NEAR(L.current(), 20.0, 1e-9);
}

// 계단 명령이 max_steer_rate 를 넘지 않게 퍼져야 한다.
TEST(SteerLimiter, LimitsStepToMaxRate)
{
    SteerLimiter L(defaults());
    const double first = L.apply(20.0, kDt, false, 0.0);
    EXPECT_NEAR(first, 600.0 * kDt, 1e-9) << "한 tick 에 12deg 를 넘었다";
    const double second = L.apply(20.0, kDt, false, kDt);
    EXPECT_NEAR(second, 20.0, 1e-9) << "두 tick 이면 20deg 에 도달해야 한다";
}

// 소스 전환 직후에는 더 조인 상한이 걸린다.
TEST(SteerLimiter, TightensRateAfterSourceChange)
{
    SteerLimiter L(defaults());
    const double v = L.apply(20.0, kDt, /*source_changed=*/true, 0.0);
    EXPECT_NEAR(v, 150.0 * kDt, 1e-9) << "전환 직후인데 600deg/s 로 움직였다";
    EXPECT_TRUE(L.inTransition(0.1));
}

// 전환 창이 지나면 원래 상한으로 돌아온다.
TEST(SteerLimiter, RestoresRateAfterTransitionWindow)
{
    SteerLimiter L(defaults());
    L.apply(0.0, kDt, true, 0.0);
    EXPECT_TRUE(L.inTransition(0.39));
    EXPECT_FALSE(L.inTransition(0.41));

    const double before = L.current();
    const double v = L.apply(20.0, kDt, false, 0.5);
    EXPECT_NEAR(v - before, 600.0 * kDt, 1e-9);
}

// 전환 창 안에서 또 전환이 나면 창이 갱신된다.
TEST(SteerLimiter, RetriggerExtendsWindow)
{
    SteerLimiter L(defaults());
    L.apply(0.0, kDt, true, 0.0);
    L.apply(0.0, kDt, true, 0.3);
    EXPECT_TRUE(L.inTransition(0.6)) << "두 번째 전환이 창을 늘리지 않았다";
}

// 계단이 실제로 몇 tick 에 걸쳐 퍼지는지 -- 전환 창에서 40deg 를 훑는 데
// 필요한 시간이 transition_sec 안이어야 한다 (정상 조향을 막지 않음).
TEST(SteerLimiter, TransitionRateStillCoversFullRange)
{
    const SteerLimiterParams p = defaults();
    const double full_range = 2.0 * p.max_steer_deg;          // -20 -> +20
    const double time_needed = full_range / p.transition_rate_dps;
    EXPECT_LT(time_needed, p.transition_sec)
        << "전환 상한이 너무 조여서 창 안에 전 범위를 못 훑는다";
}

// 추종 실패 시 0 은 슬루를 거치지 않는다 -- 조향을 푸는 방향을 늦추면
// 위험한 쪽으로 남는다.
TEST(SteerLimiter, ReleaseToZeroIsImmediate)
{
    SteerLimiter L(defaults());
    for (int i = 0; i < 10; ++i)
    {
        L.apply(20.0, kDt, false, i * kDt);
    }
    ASSERT_GT(L.current(), 10.0);
    EXPECT_NEAR(L.releaseToZero(), 0.0, 1e-9);
    EXPECT_NEAR(L.current(), 0.0, 1e-9);
}

// releaseToZero 뒤에는 전환 창도 풀려야 한다 (경로가 다시 붙었을 때
// 이유 없이 느리게 반응하면 안 된다).
TEST(SteerLimiter, ReleaseClearsTransitionWindow)
{
    SteerLimiter L(defaults());
    L.apply(10.0, kDt, true, 0.0);
    ASSERT_TRUE(L.inTransition(0.1));
    L.releaseToZero();
    EXPECT_FALSE(L.inTransition(0.1));
}

// dt <= 0 (시각이 안 흐름) 이면 값을 유지한다. 0 으로 나누거나 튀면 안 된다.
TEST(SteerLimiter, HandlesZeroDt)
{
    SteerLimiter L(defaults());
    L.apply(10.0, kDt, false, 0.0);
    const double v = L.current();
    EXPECT_NEAR(L.apply(20.0, 0.0, false, kDt), v, 1e-9);
}
