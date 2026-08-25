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

// ---------------------------------------------------------------------------
// Forked from package "kau_object_detection", its test
// test_start_signal_logic.cpp, on 2026-08-25.
//
// 2026-08-25: the consecutive-run cases were replaced by sliding-window ones.
// The policy under test is now "green in at least `green_required_frames` of
// the last `green_window_frames` usable frames", not "N greens in a row".
//
// Everything that was NOT about the run length is kept as it was: the latch is
// still permanent, the camera timeout still clears progress before the latch
// and never after it, and a backward clock is still re-anchored rather than
// treated as a gap.
// ---------------------------------------------------------------------------

#include <cmath>
#include <stdexcept>
#include <vector>

#include "gtest/gtest.h"
#include "kau_object_detection_lane/start_signal_logic.hpp"

namespace
{

using kau_object_detection_lane::FrameOutcome;
using kau_object_detection_lane::StartPermissionLatch;
using kau_object_detection_lane::StartSignalOptions;

/// Camera cadence used throughout: about 15 Hz, matching the real stream.
constexpr double kFramePeriodS = 1.0 / 15.0;

/// The shipped policy: green in 4 of the last 8 usable frames.
StartSignalOptions default_options()
{
  StartSignalOptions options;
  options.green_window_frames = 8;
  options.green_required_frames = 4;
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

/// Feeds an explicit green/not-green pattern, one frame per element.
double feed_pattern(
  StartPermissionLatch & latch,
  const std::vector<bool> & greens,
  double start_s)
{
  for (const bool green : greens) {
    latch.observe_frame(
      green ? FrameOutcome::kGreen : FrameOutcome::kNotGreen, start_s);
    start_s += kFramePeriodS;
  }
  return start_s;
}

}  // namespace

TEST(StartSignalLogicTest, StartsForbidden)
{
  const StartPermissionLatch latch{default_options()};

  EXPECT_FALSE(latch.start_permitted());
  EXPECT_FALSE(latch.latched());
  EXPECT_FALSE(latch.camera_active());
  EXPECT_EQ(latch.green_hits(), 0);
  EXPECT_EQ(latch.window_samples(), 0);
}

// --- The sliding window ----------------------------------------------------

/// Four greens as the very first four samples latch, even though the window is
/// only half full. The threshold is a count, not a ratio.
TEST(StartSignalLogicTest, FourImmediateGreensLatch)
{
  StartPermissionLatch latch{default_options()};

  feed(latch, FrameOutcome::kGreen, 3, 0.0);
  EXPECT_FALSE(latch.start_permitted()) << "three greens must not be enough";
  EXPECT_EQ(latch.green_hits(), 3);
  EXPECT_EQ(latch.window_samples(), 3);

  latch.observe_frame(FrameOutcome::kGreen, 3.0 * kFramePeriodS);
  EXPECT_TRUE(latch.start_permitted());
  EXPECT_EQ(latch.green_hits(), 4);
  EXPECT_EQ(latch.window_samples(), 4);
}

/// The regression this change exists for: alternating green and non-green never
/// reaches four in a row, but it does reach four within seven frames.
TEST(StartSignalLogicTest, FourOfSevenWithMissesLatch)
{
  StartPermissionLatch latch{default_options()};

  feed_pattern(latch, {true, false, true, false, true, false}, 0.0);
  EXPECT_FALSE(latch.start_permitted()) << "only three greens so far";
  EXPECT_EQ(latch.green_hits(), 3);

  latch.observe_frame(FrameOutcome::kGreen, 6.0 * kFramePeriodS);
  EXPECT_TRUE(latch.start_permitted()) << "T,F,T,F,T,F,T is 4 of the last 7";
  EXPECT_EQ(latch.green_hits(), 4);
  EXPECT_EQ(latch.window_samples(), 7);
}

TEST(StartSignalLogicTest, ThreeOfEightDoesNotLatch)
{
  StartPermissionLatch latch{default_options()};

  feed_pattern(latch, {true, false, true, false, true, false, false, false}, 0.0);

  EXPECT_FALSE(latch.start_permitted());
  EXPECT_EQ(latch.green_hits(), 3);
  EXPECT_EQ(latch.window_samples(), 8);
}

TEST(StartSignalLogicTest, ThreeLeadingGreensThenFiveMissesDoNotLatch)
{
  StartPermissionLatch latch{default_options()};

  feed_pattern(latch, {true, true, true, false, false, false, false, false}, 0.0);

  EXPECT_FALSE(latch.start_permitted());
  EXPECT_EQ(latch.green_hits(), 3);
  EXPECT_EQ(latch.window_samples(), 8);
}

/// The window is bounded: sample nine pushes sample one out, and a green that
/// leaves the window stops counting.
TEST(StartSignalLogicTest, OldGreenIsEvicted)
{
  StartPermissionLatch latch{default_options()};

  double now_s = feed(latch, FrameOutcome::kGreen, 3, 0.0);
  ASSERT_EQ(latch.green_hits(), 3);

  // Fill the window out to eight with non-green frames.
  now_s = feed(latch, FrameOutcome::kNotGreen, 5, now_s);
  ASSERT_EQ(latch.window_samples(), 8);
  ASSERT_EQ(latch.green_hits(), 3);

  // The ninth frame evicts the first green.
  latch.observe_frame(FrameOutcome::kNotGreen, now_s);
  EXPECT_EQ(latch.window_samples(), 8) << "the window must not grow past its size";
  EXPECT_EQ(latch.green_hits(), 2);
  EXPECT_FALSE(latch.start_permitted());
}

/// Exactly four greens spread through the last eight samples is the boundary
/// case, and it latches.
TEST(StartSignalLogicTest, FourNonConsecutiveWithinEightLatch)
{
  StartPermissionLatch latch{default_options()};

  feed_pattern(latch, {true, false, false, true, false, true, false}, 0.0);
  ASSERT_FALSE(latch.start_permitted());
  ASSERT_EQ(latch.green_hits(), 3);

  latch.observe_frame(FrameOutcome::kGreen, 7.0 * kFramePeriodS);
  EXPECT_TRUE(latch.start_permitted());
  EXPECT_EQ(latch.green_hits(), 4);
  EXPECT_EQ(latch.window_samples(), 8);
}

/// Four greens over the whole run are not four greens in the window. This is
/// what stops the policy from degenerating into "four greens ever".
TEST(StartSignalLogicTest, FourGreensSpreadAcrossMoreThanEightDoNotLatch)
{
  StartPermissionLatch latch{default_options()};

  // 11 samples, greens at 0, 1, 2 and 10. Four greens overall, but the run
  // peaked at three hits and by the last frame the window holds indices 3..10,
  // so only the final green is still inside it.
  feed_pattern(
    latch,
    {true, true, true, false, false, false,
      false, false, false, false, true},
    0.0);

  EXPECT_FALSE(latch.start_permitted());
  EXPECT_EQ(latch.window_samples(), 8);
  EXPECT_EQ(latch.green_hits(), 1) << "only the green at index 10 is still in the window";
}

// --- Frame classification --------------------------------------------------

/// Red, yellow, an absent signal and an unrecognised one all arrive as
/// kNotGreen and must never count towards the threshold.
TEST(StartSignalLogicTest, RedYellowUnknownAreFalseSamples)
{
  StartPermissionLatch latch{default_options()};

  feed(latch, FrameOutcome::kNotGreen, 20, 0.0);

  EXPECT_FALSE(latch.start_permitted());
  EXPECT_EQ(latch.green_hits(), 0);
  EXPECT_EQ(latch.window_samples(), 8);
}

/// A frame that failed to decode enters the window as a non-green sample. It is
/// the conservative choice: it can evict an older green and so only ever makes
/// latching harder.
TEST(StartSignalLogicTest, UnusableFrameIsANonGreenSample)
{
  StartPermissionLatch latch{default_options()};

  double now_s = feed(latch, FrameOutcome::kGreen, 3, 0.0);
  ASSERT_EQ(latch.green_hits(), 3);

  latch.observe_frame(FrameOutcome::kUnusable, now_s);
  EXPECT_EQ(latch.green_hits(), 3) << "an unusable frame is not a green";
  EXPECT_EQ(latch.window_samples(), 4);
  EXPECT_FALSE(latch.start_permitted());
}

/// An unusable frame does not prove the camera is delivering, so it must not
/// refresh the liveness clock. This is the pre-existing safety policy, kept.
TEST(StartSignalLogicTest, UnusableFrameDoesNotCountAsCameraActivity)
{
  StartPermissionLatch latch{default_options()};

  latch.observe_frame(FrameOutcome::kGreen, 0.0);
  ASSERT_TRUE(latch.camera_active());

  // Only unusable frames from here on. The liveness clock stays at 0.0.
  feed(latch, FrameOutcome::kUnusable, 5, kFramePeriodS);

  latch.advance_time(0.6);
  EXPECT_FALSE(latch.camera_active());
}

// --- Camera timeout --------------------------------------------------------

/// Greens seen before a long outage must not combine with greens seen after it.
TEST(StartSignalLogicTest, TimeoutClearsWindowBeforeLatch)
{
  StartPermissionLatch latch{default_options()};

  feed(latch, FrameOutcome::kGreen, 3, 0.0);
  ASSERT_EQ(latch.green_hits(), 3);
  ASSERT_FALSE(latch.start_permitted());

  latch.advance_time(5.0);
  EXPECT_FALSE(latch.camera_active());
  EXPECT_EQ(latch.green_hits(), 0) << "the pre-outage greens must be gone";
  EXPECT_EQ(latch.window_samples(), 0);

  // One green after recovery is nowhere near the threshold.
  latch.observe_frame(FrameOutcome::kGreen, 5.1);
  EXPECT_FALSE(latch.start_permitted());
  EXPECT_EQ(latch.green_hits(), 1);
}

TEST(StartSignalLogicTest, TimeoutDoesNotClearLatchedTrue)
{
  StartPermissionLatch latch{default_options()};

  feed(latch, FrameOutcome::kGreen, 4, 0.0);
  ASSERT_TRUE(latch.start_permitted());

  latch.advance_time(60.0);
  EXPECT_TRUE(latch.start_permitted());
  EXPECT_TRUE(latch.latched());
}

TEST(StartSignalLogicTest, TimeShorterThanTheTimeoutKeepsTheWindow)
{
  StartPermissionLatch latch{default_options()};

  feed(latch, FrameOutcome::kGreen, 3, 0.0);
  ASSERT_EQ(latch.green_hits(), 3);

  // Three frame periods is well inside the 0.5 s timeout.
  latch.advance_time(3.0 * kFramePeriodS);
  EXPECT_TRUE(latch.camera_active());
  EXPECT_EQ(latch.green_hits(), 3);
}

TEST(StartSignalLogicTest, NoFrameAtAllLeavesPermissionFalse)
{
  StartPermissionLatch latch{default_options()};

  latch.advance_time(10.0);

  EXPECT_FALSE(latch.start_permitted());
  EXPECT_FALSE(latch.camera_active());
  EXPECT_EQ(latch.window_samples(), 0);
}

/// A simulator reset can move the clock backwards. That is re-anchored, not
/// reported as an outage that never happened.
TEST(StartSignalLogicTest, BackwardClockDoesNotClearTheWindow)
{
  StartPermissionLatch latch{default_options()};

  feed(latch, FrameOutcome::kGreen, 3, 100.0);
  ASSERT_EQ(latch.green_hits(), 3);

  latch.advance_time(50.0);

  EXPECT_EQ(latch.green_hits(), 3);
  EXPECT_TRUE(latch.camera_active());
}

// --- The latch is permanent ------------------------------------------------

TEST(StartSignalLogicTest, LatchNeverReturnsFalse)
{
  StartPermissionLatch latch{default_options()};

  double now_s = feed(latch, FrameOutcome::kGreen, 4, 0.0);
  ASSERT_TRUE(latch.start_permitted());

  now_s = feed(latch, FrameOutcome::kNotGreen, 50, now_s);
  EXPECT_TRUE(latch.start_permitted());

  now_s = feed(latch, FrameOutcome::kUnusable, 50, now_s);
  EXPECT_TRUE(latch.start_permitted());

  latch.advance_time(now_s + 100.0);
  EXPECT_TRUE(latch.start_permitted());
}

/// The reported hit count freezes at the value that latched, so the log line
/// after the latch keeps naming the evidence the decision was made on.
TEST(StartSignalLogicTest, HitCountFreezesOnceLatched)
{
  StartPermissionLatch latch{default_options()};

  const double now_s = feed(latch, FrameOutcome::kGreen, 4, 0.0);
  ASSERT_TRUE(latch.latched());
  ASSERT_EQ(latch.green_hits(), 4);

  feed(latch, FrameOutcome::kNotGreen, 20, now_s);
  EXPECT_EQ(latch.green_hits(), 4);
  EXPECT_EQ(latch.window_samples(), 4);
}

/// Camera liveness is still tracked after the latch, for diagnostics only.
TEST(StartSignalLogicTest, CameraActivityStillTracksAfterLatch)
{
  StartPermissionLatch latch{default_options()};

  const double now_s = feed(latch, FrameOutcome::kGreen, 4, 0.0);
  ASSERT_TRUE(latch.latched());
  ASSERT_TRUE(latch.camera_active());

  latch.observe_frame(FrameOutcome::kNotGreen, now_s);
  EXPECT_TRUE(latch.camera_active());
  EXPECT_TRUE(latch.start_permitted());
}

// --- Configuration ---------------------------------------------------------

/// A misconfigured policy must stop the node at start-up, not silently forbid
/// or permit the start.
TEST(StartSignalLogicTest, InvalidWindowParametersRejected)
{
  StartSignalOptions options = default_options();

  options.green_required_frames = 0;
  EXPECT_THROW(StartPermissionLatch{options}, std::invalid_argument);

  options = default_options();
  options.green_required_frames = -1;
  EXPECT_THROW(StartPermissionLatch{options}, std::invalid_argument);

  options = default_options();
  options.green_window_frames = 0;
  EXPECT_THROW(StartPermissionLatch{options}, std::invalid_argument);

  options = default_options();
  options.green_window_frames = -1;
  EXPECT_THROW(StartPermissionLatch{options}, std::invalid_argument);

  // Unreachable threshold: it would forbid the start for the whole race with
  // no other symptom.
  options = default_options();
  options.green_window_frames = 4;
  options.green_required_frames = 5;
  EXPECT_THROW(StartPermissionLatch{options}, std::invalid_argument);
}

TEST(StartSignalLogicTest, RejectsNonPositiveCameraTimeout)
{
  StartSignalOptions options = default_options();

  options.camera_timeout_s = 0.0;
  EXPECT_THROW(StartPermissionLatch{options}, std::invalid_argument);

  options.camera_timeout_s = -1.0;
  EXPECT_THROW(StartPermissionLatch{options}, std::invalid_argument);

  options.camera_timeout_s = std::nan("");
  EXPECT_THROW(StartPermissionLatch{options}, std::invalid_argument);
}

TEST(StartSignalLogicTest, WindowSizeAndThresholdAreConfigurable)
{
  StartSignalOptions options = default_options();
  options.green_window_frames = 3;
  options.green_required_frames = 2;

  StartPermissionLatch latch{options};

  feed_pattern(latch, {true, false}, 0.0);
  EXPECT_FALSE(latch.start_permitted());
  EXPECT_EQ(latch.green_hits(), 1);

  latch.observe_frame(FrameOutcome::kGreen, 2.0 * kFramePeriodS);
  EXPECT_TRUE(latch.start_permitted()) << "2 of the last 3 is the threshold here";
}

/// The degenerate but legal setting: one green anywhere permits the start.
TEST(StartSignalLogicTest, AcceptsASingleFrameConfirmation)
{
  StartSignalOptions options = default_options();
  options.green_window_frames = 1;
  options.green_required_frames = 1;

  StartPermissionLatch latch{options};

  latch.observe_frame(FrameOutcome::kGreen, 0.0);
  EXPECT_TRUE(latch.start_permitted());
}

/// A window of 3 that has already evicted a green cannot latch on a stale one.
TEST(StartSignalLogicTest, SmallWindowEvictsCorrectly)
{
  StartSignalOptions options = default_options();
  options.green_window_frames = 3;
  options.green_required_frames = 2;

  StartPermissionLatch latch{options};

  feed_pattern(latch, {true, false, false}, 0.0);
  ASSERT_EQ(latch.green_hits(), 1);

  // The fourth sample evicts the green.
  latch.observe_frame(FrameOutcome::kNotGreen, 3.0 * kFramePeriodS);
  EXPECT_EQ(latch.green_hits(), 0);
  EXPECT_EQ(latch.window_samples(), 3);

  // One green now leaves only one hit in the window, still short of two.
  latch.observe_frame(FrameOutcome::kGreen, 4.0 * kFramePeriodS);
  EXPECT_FALSE(latch.start_permitted());
  EXPECT_EQ(latch.green_hits(), 1);
}

TEST(StartSignalLogicTest, OptionsAreReportedBack)
{
  const StartPermissionLatch latch{default_options()};

  EXPECT_EQ(latch.options().green_window_frames, 8);
  EXPECT_EQ(latch.options().green_required_frames, 4);
  EXPECT_DOUBLE_EQ(latch.options().camera_timeout_s, 0.5);
}
