// Copyright 2026 KAU AMET Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <limits>
#include <stdexcept>

#include "gtest/gtest.h"
#include "kau_object_detection/start_signal_logic.hpp"

namespace
{

using kau_object_detection::FrameOutcome;
using kau_object_detection::StartPermissionLatch;
using kau_object_detection::StartSignalOptions;

/// Camera cadence used throughout: about 15 Hz, matching the real stream.
constexpr double kFramePeriodS = 1.0 / 15.0;

StartSignalOptions default_options()
{
  StartSignalOptions options;
  options.green_confirmation_frames = 5;
  options.camera_timeout_s = 0.5;
  return options;
}

/// Feeds `count` frames of one outcome starting at `start_s`, returning the
/// timestamp one frame period after the last one.
double feed(
  StartPermissionLatch & latch,
  const FrameOutcome outcome,
  const int count,
  double start_s)
{
  for (int index = 0; index < count; ++index) {
    latch.observe_frame(outcome, start_s);
    start_s += kFramePeriodS;
  }
  return start_s;
}

// ---------------------------------------------------------------------------
// Confirmation run
// ---------------------------------------------------------------------------

TEST(StartSignalLogicTest, StartsForbidden)
{
  const StartPermissionLatch latch{default_options()};

  EXPECT_FALSE(latch.start_permitted());
  EXPECT_FALSE(latch.latched());
  EXPECT_FALSE(latch.camera_active());
  EXPECT_EQ(latch.consecutive_green_frames(), 0);
}

TEST(StartSignalLogicTest, StaysForbiddenBeforeTheFifthGreenFrame)
{
  StartPermissionLatch latch{default_options()};

  double now_s = 0.0;
  for (int frame = 1; frame <= 4; ++frame) {
    latch.observe_frame(FrameOutcome::kGreen, now_s);
    now_s += kFramePeriodS;

    EXPECT_FALSE(latch.start_permitted()) << "green frame " << frame;
    EXPECT_EQ(latch.consecutive_green_frames(), frame);
  }
}

TEST(StartSignalLogicTest, PermitsOnTheFifthConsecutiveGreenFrame)
{
  StartPermissionLatch latch{default_options()};

  feed(latch, FrameOutcome::kGreen, 4, 0.0);
  ASSERT_FALSE(latch.start_permitted());

  latch.observe_frame(FrameOutcome::kGreen, 4 * kFramePeriodS);

  EXPECT_TRUE(latch.start_permitted());
  EXPECT_TRUE(latch.latched());
}

TEST(StartSignalLogicTest, ConfirmationRunLengthIsConfigurable)
{
  StartSignalOptions options = default_options();
  options.green_confirmation_frames = 2;
  StartPermissionLatch latch{options};

  latch.observe_frame(FrameOutcome::kGreen, 0.0);
  EXPECT_FALSE(latch.start_permitted());

  latch.observe_frame(FrameOutcome::kGreen, kFramePeriodS);
  EXPECT_TRUE(latch.start_permitted());
}

// ---------------------------------------------------------------------------
// Run resets
// ---------------------------------------------------------------------------

TEST(StartSignalLogicTest, NonGreenFrameResetsTheRun)
{
  StartPermissionLatch latch{default_options()};

  double now_s = feed(latch, FrameOutcome::kGreen, 4, 0.0);
  ASSERT_EQ(latch.consecutive_green_frames(), 4);

  // Red, yellow, unknown and an absent signal all arrive as kNotGreen.
  latch.observe_frame(FrameOutcome::kNotGreen, now_s);
  now_s += kFramePeriodS;

  EXPECT_EQ(latch.consecutive_green_frames(), 0);
  EXPECT_FALSE(latch.start_permitted());

  // Four more greens are still one short, proving the counter really restarted.
  now_s = feed(latch, FrameOutcome::kGreen, 4, now_s);
  EXPECT_FALSE(latch.start_permitted());

  latch.observe_frame(FrameOutcome::kGreen, now_s);
  EXPECT_TRUE(latch.start_permitted());
}

TEST(StartSignalLogicTest, UnusableFrameResetsTheRun)
{
  StartPermissionLatch latch{default_options()};

  double now_s = feed(latch, FrameOutcome::kGreen, 4, 0.0);
  latch.observe_frame(FrameOutcome::kUnusable, now_s);
  now_s += kFramePeriodS;

  EXPECT_EQ(latch.consecutive_green_frames(), 0);
  EXPECT_FALSE(latch.start_permitted());
}

TEST(StartSignalLogicTest, CameraTimeoutBeforeLatchKeepsPermissionFalse)
{
  StartPermissionLatch latch{default_options()};

  double now_s = feed(latch, FrameOutcome::kGreen, 4, 0.0);
  ASSERT_EQ(latch.consecutive_green_frames(), 4);
  ASSERT_TRUE(latch.camera_active());

  // The stream stops. The timer keeps running.
  now_s += 0.6;
  latch.advance_time(now_s);

  EXPECT_FALSE(latch.start_permitted());
  EXPECT_FALSE(latch.camera_active());
  EXPECT_EQ(latch.consecutive_green_frames(), 0);
}

TEST(StartSignalLogicTest, TimeShorterThanTheTimeoutKeepsTheRun)
{
  StartPermissionLatch latch{default_options()};

  const double now_s = feed(latch, FrameOutcome::kGreen, 4, 0.0);
  latch.advance_time(now_s + 0.2);

  EXPECT_EQ(latch.consecutive_green_frames(), 4);
  EXPECT_TRUE(latch.camera_active());
}

TEST(StartSignalLogicTest, NoFrameAtAllLeavesPermissionFalse)
{
  StartPermissionLatch latch{default_options()};

  for (int tick = 0; tick < 50; ++tick) {
    latch.advance_time(0.1 * tick);
  }

  EXPECT_FALSE(latch.start_permitted());
  EXPECT_FALSE(latch.camera_active());
}

// ---------------------------------------------------------------------------
// green_streak, the value the node prints as [streak/required]
// ---------------------------------------------------------------------------

TEST(StartSignalLogicTest, GreenStreakStartsAtZero)
{
  const StartPermissionLatch latch{default_options()};

  EXPECT_EQ(latch.consecutive_green_frames(), 0);
  EXPECT_EQ(latch.options().green_confirmation_frames, 5);
}

TEST(StartSignalLogicTest, GreenStreakWalksOneToFiveAndLatchesOnTheLast)
{
  StartPermissionLatch latch{default_options()};

  double now_s = 0.0;
  const int expected[] = {1, 2, 3, 4, 5};
  for (const int streak : expected) {
    latch.observe_frame(FrameOutcome::kGreen, now_s);
    now_s += kFramePeriodS;

    EXPECT_EQ(latch.consecutive_green_frames(), streak);
    EXPECT_EQ(latch.start_permitted(), streak == 5) << "at green_streak=[" << streak << "/5]";
  }
}

TEST(StartSignalLogicTest, GreenStreakCountsConsecutiveFramesNotACumulativeTotal)
{
  StartPermissionLatch latch{default_options()};

  // Three greens, one red, two greens. Six green frames have now been seen in
  // total, but the run is only two long, so the start stays forbidden.
  double now_s = feed(latch, FrameOutcome::kGreen, 3, 0.0);
  latch.observe_frame(FrameOutcome::kNotGreen, now_s);
  now_s += kFramePeriodS;
  now_s = feed(latch, FrameOutcome::kGreen, 2, now_s);

  EXPECT_EQ(latch.consecutive_green_frames(), 2);
  EXPECT_FALSE(latch.start_permitted());

  // Three more complete the run of five.
  feed(latch, FrameOutcome::kGreen, 3, now_s);
  EXPECT_EQ(latch.consecutive_green_frames(), 5);
  EXPECT_TRUE(latch.start_permitted());
}

TEST(StartSignalLogicTest, GreenStreakReturnsToZeroOnANonGreenFrame)
{
  StartPermissionLatch latch{default_options()};

  const double now_s = feed(latch, FrameOutcome::kGreen, 4, 0.0);
  ASSERT_EQ(latch.consecutive_green_frames(), 4);

  latch.observe_frame(FrameOutcome::kNotGreen, now_s);

  EXPECT_EQ(latch.consecutive_green_frames(), 0);
  EXPECT_FALSE(latch.start_permitted());
}

TEST(StartSignalLogicTest, GreenStreakReturnsToZeroOnCameraTimeoutBeforeLatch)
{
  StartPermissionLatch latch{default_options()};

  const double now_s = feed(latch, FrameOutcome::kGreen, 3, 0.0);
  ASSERT_EQ(latch.consecutive_green_frames(), 3);

  latch.advance_time(now_s + 0.6);

  EXPECT_EQ(latch.consecutive_green_frames(), 0);
  EXPECT_FALSE(latch.start_permitted());
}

TEST(StartSignalLogicTest, GreenStreakHoldsAtTheConfirmationCountAfterLatch)
{
  StartPermissionLatch latch{default_options()};

  double now_s = feed(latch, FrameOutcome::kGreen, 5, 0.0);
  ASSERT_TRUE(latch.start_permitted());

  // Red, then a dead camera. The printed line must keep reading [5/5].
  now_s = feed(latch, FrameOutcome::kNotGreen, 10, now_s);
  latch.advance_time(now_s + 5.0);

  EXPECT_EQ(latch.consecutive_green_frames(), 5);
  EXPECT_TRUE(latch.start_permitted());
}

// ---------------------------------------------------------------------------
// Latch behaviour
// ---------------------------------------------------------------------------

TEST(StartSignalLogicTest, LatchSurvivesNonGreenFrames)
{
  StartPermissionLatch latch{default_options()};

  double now_s = feed(latch, FrameOutcome::kGreen, 5, 0.0);
  ASSERT_TRUE(latch.start_permitted());

  // The signal turns red, then disappears from the frame entirely.
  now_s = feed(latch, FrameOutcome::kNotGreen, 100, now_s);

  EXPECT_TRUE(latch.start_permitted());
  EXPECT_TRUE(latch.latched());
}

TEST(StartSignalLogicTest, LatchSurvivesCameraTimeout)
{
  StartPermissionLatch latch{default_options()};

  const double now_s = feed(latch, FrameOutcome::kGreen, 5, 0.0);
  ASSERT_TRUE(latch.start_permitted());

  // The camera dies for a minute.
  for (int tick = 0; tick < 600; ++tick) {
    latch.advance_time(now_s + 0.1 * tick);
  }

  EXPECT_TRUE(latch.start_permitted());
  EXPECT_TRUE(latch.latched());
}

TEST(StartSignalLogicTest, LatchSurvivesUnusableFrames)
{
  StartPermissionLatch latch{default_options()};

  double now_s = feed(latch, FrameOutcome::kGreen, 5, 0.0);
  ASSERT_TRUE(latch.start_permitted());

  now_s = feed(latch, FrameOutcome::kUnusable, 30, now_s);

  EXPECT_TRUE(latch.start_permitted());
}

TEST(StartSignalLogicTest, ConfirmationCountFreezesOnceLatched)
{
  StartPermissionLatch latch{default_options()};

  double now_s = feed(latch, FrameOutcome::kGreen, 5, 0.0);
  ASSERT_EQ(latch.consecutive_green_frames(), 5);

  now_s = feed(latch, FrameOutcome::kGreen, 20, now_s);

  // Nothing keeps counting after the decision is made.
  EXPECT_EQ(latch.consecutive_green_frames(), 5);
  EXPECT_TRUE(latch.start_permitted());
}

// ---------------------------------------------------------------------------
// Clock robustness
// ---------------------------------------------------------------------------

TEST(StartSignalLogicTest, BackwardClockDoesNotClearTheRun)
{
  StartPermissionLatch latch{default_options()};

  const double now_s = feed(latch, FrameOutcome::kGreen, 4, 100.0);
  ASSERT_EQ(latch.consecutive_green_frames(), 4);

  // A simulator reset moves the clock back to the start of the run.
  latch.advance_time(now_s - 50.0);

  EXPECT_EQ(latch.consecutive_green_frames(), 4);
  EXPECT_FALSE(latch.start_permitted());
}

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

TEST(StartSignalLogicTest, RejectsNonPositiveConfirmationFrames)
{
  StartSignalOptions options = default_options();

  options.green_confirmation_frames = 0;
  EXPECT_THROW(StartPermissionLatch{options}, std::invalid_argument);

  options.green_confirmation_frames = -1;
  EXPECT_THROW(StartPermissionLatch{options}, std::invalid_argument);
}

TEST(StartSignalLogicTest, RejectsNonPositiveCameraTimeout)
{
  StartSignalOptions options = default_options();

  options.camera_timeout_s = 0.0;
  EXPECT_THROW(StartPermissionLatch{options}, std::invalid_argument);

  options.camera_timeout_s = -0.5;
  EXPECT_THROW(StartPermissionLatch{options}, std::invalid_argument);

  options.camera_timeout_s = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(StartPermissionLatch{options}, std::invalid_argument);
}

TEST(StartSignalLogicTest, AcceptsASingleFrameConfirmation)
{
  StartSignalOptions options = default_options();
  options.green_confirmation_frames = 1;

  StartPermissionLatch latch{options};
  latch.observe_frame(FrameOutcome::kGreen, 0.0);

  EXPECT_TRUE(latch.start_permitted());
}

}  // namespace
