// simple_drive 순수 로직 테스트. 노드는 띄우지 않는다 --
// 밴뱅 / 회피 FSM / 경로 샘플링은 전부 상태 없는 헤더라 단독으로 돈다.

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "simple_drive/avoid_fsm.hpp"
#include "simple_drive/bang_bang.hpp"
#include "simple_drive/lane_source.hpp"
#include "simple_drive/path_sample.hpp"

using simple_drive::AvoidFsm;
using simple_drive::AvoidState;
using simple_drive::BangBang;
using simple_drive::LaneOrigin;
using simple_drive::LaneResolver;

namespace
{

constexpr double kDt = 0.02;  // 50 Hz

// 등간격 제어점 quintic 은 직선이고 B(u) = P0 + 5*d*u 다.
// length 만큼 뻗은 heading 방향 직선을 만든다.
backup_msgs::msg::BackupPath straightPath(
  double length, double heading_rad,
  double valid_length = -1.0)
{
  backup_msgs::msg::BackupPath p;
  p.degree = 5;
  const double step = length / 5.0;
  for (int i = 0; i <= 5; ++i) {
    p.ctrl_x.push_back(i * step * std::cos(heading_rad));
    p.ctrl_y.push_back(i * step * std::sin(heading_rad));
  }
  p.total_length = length;
  p.valid_length = (valid_length < 0.0) ? length : valid_length;
  return p;
}

backup_msgs::msg::ObstacleCircleArray obstacleAt(double x, double y)
{
  backup_msgs::msg::ObstacleCircleArray a;
  a.status = backup_msgs::msg::ObstacleCircleArray::STATUS_OK;
  backup_msgs::msg::ObstacleCircle o;
  o.center_x = static_cast<float>(x);
  o.center_y = static_cast<float>(y);
  o.radius = 0.142f;
  o.confidence = 1.0f;
  a.obstacles.push_back(o);
  return a;
}

}  // namespace


// ====================================================================
// path_sample
// ====================================================================

TEST(PathSample, StraightAheadHasNoLateralError)
{
  const auto p = straightPath(1.0, 0.0);
  const auto la = simple_drive::lookaheadAt(p, 0.5, 0.05);

  ASSERT_TRUE(la.valid);
  EXPECT_NEAR(la.s, 0.5, 1e-3);
  EXPECT_NEAR(la.point.x, 0.5, 1e-3);
  EXPECT_NEAR(la.lateral, 0.0, 1e-6);
}

TEST(PathSample, LeftLeaningPathGivesPositiveLateral)
{
  // 좌측 + 규약. 30 도 왼쪽으로 뻗은 직선의 0.5 m 지점은 y = 0.25.
  const auto p = straightPath(1.0, 30.0 * M_PI / 180.0);
  const auto la = simple_drive::lookaheadAt(p, 0.5, 0.05);

  ASSERT_TRUE(la.valid);
  EXPECT_NEAR(la.lateral, 0.25, 2e-3);
  EXPECT_GT(la.lateral, 0.0);
  EXPECT_NEAR(la.heading, 30.0 * M_PI / 180.0, 2e-3);
}

TEST(PathSample, ClampsToValidLengthNotTotalLength)
{
  // 경로는 1.0 m 지만 근거는 0.30 m 까지다. margin 0.05 -> 0.25 를 본다.
  // 이걸 안 자르면 외삽 구간을 쫓는다.
  const auto p = straightPath(1.0, 0.0, 0.30);
  const auto la = simple_drive::lookaheadAt(p, 0.8, 0.05);

  ASSERT_TRUE(la.valid);
  EXPECT_NEAR(la.s, 0.25, 1e-3);
}

TEST(PathSample, RejectsMalformedControlPointCount)
{
  backup_msgs::msg::BackupPath p;
  p.degree = 5;
  p.ctrl_x = {0.0, 0.1, 0.2};  // 6 의 배수가 아니다
  p.ctrl_y = {0.0, 0.0, 0.0};
  p.valid_length = 1.0;

  EXPECT_FALSE(simple_drive::lookaheadAt(p, 0.5, 0.05).valid);
}

TEST(PathSample, TooShortPathIsInvalid)
{
  const auto p = straightPath(0.04, 0.0);
  EXPECT_FALSE(simple_drive::lookaheadAt(p, 0.5, 0.05).valid);
}


// ====================================================================
// BangBang
// ====================================================================

TEST(BangBang, StaysZeroInsideDeadband)
{
  BangBang b;
  for (int i = 0; i < 50; ++i) {
    EXPECT_EQ(b.update(0.01, kDt), 0);
  }
}

TEST(BangBang, TurnsTowardPositiveLateral)
{
  BangBang b;
  // 경로가 왼쪽(y>0)이면 좌조향(+1).
  EXPECT_EQ(b.update(0.10, kDt), 1);
  EXPECT_EQ(b.update(-0.10, kDt), 1);  // min_hold 안이라 아직 못 뒤집는다
}

TEST(BangBang, HysteresisPreventsChatterAtThreshold)
{
  BangBang::Params p;
  p.enter_m = 0.04;
  p.exit_m = 0.015;
  p.min_hold_s = 0.0;  // 히스테리시스만 본다
  BangBang b(p);

  ASSERT_EQ(b.update(0.05, kDt), 1);
  // 임계 바로 아래로 내려와도 exit 위면 유지한다. 임계 하나였다면
  // 여기서 매 tick 0 <-> 1 로 떨었다.
  for (int i = 0; i < 20; ++i) {
    EXPECT_EQ(b.update(0.030, kDt), 1);
  }
  EXPECT_EQ(b.update(0.010, kDt), 0);
}

TEST(BangBang, MinHoldLimitsSwitchRate)
{
  BangBang::Params p;
  p.enter_m = 0.04;
  p.exit_m = 0.015;
  p.min_hold_s = 0.10;
  BangBang b(p);

  ASSERT_EQ(b.update(0.10, kDt), 1);
  // 부호가 뒤집힌 입력이 와도 0.10 s 는 유지한다.
  for (int i = 0; i < 4; ++i) {
    EXPECT_EQ(b.update(-0.10, kDt), 1);
  }
  EXPECT_EQ(b.update(-0.10, kDt), -1);
}

TEST(BangBang, NanFallsBackToStraight)
{
  BangBang b;
  ASSERT_EQ(b.update(0.10, kDt), 1);
  // 근거 없음 = 직진. 전타를 유지하면 차선을 벗어난다.
  EXPECT_EQ(b.update(std::numeric_limits<double>::quiet_NaN(), kDt), 0);
}


// ====================================================================
// AvoidFsm
// ====================================================================

TEST(AvoidFsm, IdleWithoutObstacle)
{
  AvoidFsm f;
  EXPECT_EQ(f.update(nullptr, kDt), 0.0);
  EXPECT_FALSE(f.ownsSteer());
  EXPECT_EQ(f.state(), AvoidState::Idle);
}

TEST(AvoidFsm, IgnoresObstacleOutsideCorridor)
{
  AvoidFsm f;
  const auto far_side = obstacleAt(0.4, 0.5);   // 옆으로 비켜 있다
  const auto too_far = obstacleAt(2.0, 0.0);    // 너무 멀다
  const auto too_near = obstacleAt(0.05, 0.0);  // 이미 지나쳤다

  EXPECT_EQ(f.update(&far_side, kDt), 0.0);
  EXPECT_EQ(f.update(&too_far, kDt), 0.0);
  EXPECT_EQ(f.update(&too_near, kDt), 0.0);
  EXPECT_FALSE(f.ownsSteer());
}

TEST(AvoidFsm, IgnoresNonOkStatus)
{
  AvoidFsm f;
  auto a = obstacleAt(0.4, 0.05);
  a.status = backup_msgs::msg::ObstacleCircleArray::STATUS_LIDAR_UNAVAILABLE;
  EXPECT_EQ(f.update(&a, kDt), 0.0);
  EXPECT_FALSE(f.ownsSteer());
}

TEST(AvoidFsm, DodgesRightWhenObstacleIsOnTheLeft)
{
  AvoidFsm f;
  const auto left = obstacleAt(0.4, 0.10);
  const double steer = f.update(&left, kDt);

  EXPECT_EQ(f.state(), AvoidState::Dodge);
  EXPECT_LT(steer, 0.0);  // 왼쪽 장애물 -> 오른쪽(-) 회피
}

TEST(AvoidFsm, DodgesLeftWhenObstacleIsOnTheRight)
{
  AvoidFsm f;
  const auto right = obstacleAt(0.4, -0.10);
  EXPECT_GT(f.update(&right, kDt), 0.0);
}

TEST(AvoidFsm, RunsDodgeCounterHoldThenReleases)
{
  AvoidFsm::Params p;
  p.dodge_deg = 20.0;
  p.dodge_s = 0.10;
  p.counter_s = 0.10;
  p.hold_s = 0.10;
  AvoidFsm f(p);

  const auto obs = obstacleAt(0.4, 0.10);

  const double dodge = f.update(&obs, kDt);
  ASSERT_EQ(f.state(), AvoidState::Dodge);
  ASSERT_LT(dodge, 0.0);

  // DODGE 구간 (0.10 s = 5 tick)
  double last = dodge;
  for (int i = 0; i < 4; ++i) {
    last = f.update(&obs, kDt);
                                                          }
  EXPECT_EQ(f.state(), AvoidState::Dodge);
  EXPECT_NEAR(last, -20.0, 1e-9);

  // COUNTER -- 부호가 뒤집힌다. 이래야 방위가 0 으로 돌아온다.
  const double counter = f.update(&obs, kDt);
  EXPECT_EQ(f.state(), AvoidState::Counter);
  EXPECT_NEAR(counter, 20.0, 1e-9);

  for (int i = 0; i < 5; ++i) {
    f.update(&obs, kDt);
                                                   }
  EXPECT_EQ(f.state(), AvoidState::Hold);
  // HOLD 는 0 도 직진이지만 여전히 조향을 쥐고 있다 -- 차선추종에
  // 넘기면 방금 비킨 장애물 쪽으로 되돌아간다.
  EXPECT_EQ(f.update(&obs, kDt), 0.0);
  EXPECT_TRUE(f.ownsSteer());

  for (int i = 0; i < 4; ++i) {
    f.update(&obs, kDt);
                                                   }
  EXPECT_EQ(f.state(), AvoidState::Idle);
  EXPECT_FALSE(f.ownsSteer());
}

TEST(AvoidFsm, DoesNotRetriggerWhileManeuvering)
{
  AvoidFsm::Params p;
  p.dodge_s = 0.10;
  p.counter_s = 0.10;
  p.hold_s = 0.10;
  AvoidFsm f(p);

  const auto left = obstacleAt(0.4, 0.10);
  ASSERT_LT(f.update(&left, kDt), 0.0);

  // 같은 장애물이 계속 보여도 시퀀스가 다시 시작되면 안 된다.
  // (재트리거되면 DODGE 를 반복하며 옆으로 계속 밀린다)
  for (int i = 0; i < 14; ++i) {
    f.update(&left, kDt);
    EXPECT_NE(f.state(), AvoidState::Idle);
  }
}

TEST(AvoidFsm, PicksNearestObstacleInCorridor)
{
  AvoidFsm f;
  backup_msgs::msg::ObstacleCircleArray a;
  a.status = backup_msgs::msg::ObstacleCircleArray::STATUS_OK;
  // 먼 쪽이 왼쪽, 가까운 쪽이 오른쪽. 가까운 쪽을 피해야 한다.
  backup_msgs::msg::ObstacleCircle far_left;
  far_left.center_x = 0.80f;
  far_left.center_y = 0.10f;
  backup_msgs::msg::ObstacleCircle near_right;
  near_right.center_x = 0.30f;
  near_right.center_y = -0.10f;
  a.obstacles = {far_left, near_right};

  EXPECT_GT(f.update(&a, kDt), 0.0);  // 오른쪽 장애물 -> 좌(+) 회피
}


// ====================================================================
// LaneResolver -- 중앙선 -> 흰선 폴백
//
// 회피 전타 직후 노란 중앙선이 화각을 벗어나도 흰선은 남는다는 것이
// 실측이다. 그때 계속 달릴 수 있어야 한다.
// ====================================================================

TEST(LaneResolver, PrefersCenterWhenAvailable)
{
  LaneResolver r;
  // 중앙선은 정면, 흰선들은 엉뚱한 방향. 중앙선이 이겨야 한다.
  const auto center = straightPath(1.0, 0.0);
  const auto left = straightPath(1.0, 45.0 * M_PI / 180.0);
  const auto right = straightPath(1.0, -45.0 * M_PI / 180.0);

  const auto e = r.resolve(&center, &left, &right);
  ASSERT_TRUE(e.valid);
  EXPECT_EQ(e.origin, LaneOrigin::Center);
  EXPECT_NEAR(e.lateral, 0.0, 1e-6);
}

TEST(LaneResolver, RecoversCenterFromBothWhiteLines)
{
  LaneResolver r;
  // 자차가 차로 한가운데 있고 중앙선만 안 보이는 상황.
  // 좌 흰선은 +0.317, 우 흰선은 -0.317 에 평행하게 있다.
  auto left = straightPath(1.0, 0.0);
  auto right = straightPath(1.0, 0.0);
  for (auto & y : left.ctrl_y) {
    y += 0.317;
                                           }
  for (auto & y : right.ctrl_y) {
    y -= 0.317;
                                            }

  const auto e = r.resolve(nullptr, &left, &right);
  ASSERT_TRUE(e.valid);
  EXPECT_EQ(e.origin, LaneOrigin::WhiteBoth);
  // 복원된 중앙선은 자차 정면이어야 한다 -- 횡오차 0.
  EXPECT_NEAR(e.lateral, 0.0, 1e-6);
}

TEST(LaneResolver, RecoversCenterFromLeftWhiteLineOnly)
{
  LaneResolver r;
  auto left = straightPath(1.0, 0.0);
  for (auto & y : left.ctrl_y) {
    y += 0.317;
                                           }

  const auto e = r.resolve(nullptr, &left, nullptr);
  ASSERT_TRUE(e.valid);
  EXPECT_EQ(e.origin, LaneOrigin::WhiteLeft);
  EXPECT_NEAR(e.lateral, 0.0, 1e-6);
}

TEST(LaneResolver, RecoversCenterFromRightWhiteLineOnly)
{
  LaneResolver r;
  auto right = straightPath(1.0, 0.0);
  for (auto & y : right.ctrl_y) {
    y -= 0.317;
                                            }

  const auto e = r.resolve(nullptr, nullptr, &right);
  ASSERT_TRUE(e.valid);
  EXPECT_EQ(e.origin, LaneOrigin::WhiteRight);
  EXPECT_NEAR(e.lateral, 0.0, 1e-6);
}

TEST(LaneResolver, WhiteLineOffsetShowsAsLateralError)
{
  LaneResolver r;
  // 자차가 차로 중앙보다 0.10 m 오른쪽에 있다 -> 좌 흰선이 0.417 에
  // 보인다 -> 복원된 중앙선은 왼쪽 0.10 m.
  auto left = straightPath(1.0, 0.0);
  for (auto & y : left.ctrl_y) {
    y += 0.417;
                                           }

  const auto e = r.resolve(nullptr, &left, nullptr);
  ASSERT_TRUE(e.valid);
  EXPECT_NEAR(e.lateral, 0.10, 1e-6);
  EXPECT_GT(e.lateral, 0.0);  // 좌측 + -> 좌조향으로 복귀
}

TEST(LaneResolver, RejectsWhiteLineTooShortToTrust)
{
  LaneResolver::Params p;
  p.white_min_valid_length_m = 0.15;
  LaneResolver r(p);

  // 검출 길이가 0.10 m 뿐이면 방위가 잡음이라 안 쓴다.
  auto left = straightPath(1.0, 0.0);
  for (auto & y : left.ctrl_y) {
    y += 0.317;
                                           }
  left.valid_length = 0.10;

  EXPECT_FALSE(r.resolve(nullptr, &left, nullptr).valid);
}

TEST(LaneResolver, NothingValidWhenAllSourcesMissing)
{
  LaneResolver r;
  const auto e = r.resolve(nullptr, nullptr, nullptr);
  EXPECT_FALSE(e.valid);
  EXPECT_EQ(e.origin, LaneOrigin::None);
}
