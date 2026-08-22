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

#include <cstdint>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "gtest/gtest.h"
#include "kau_msgs/msg/obstacle_circle_array.hpp"
#include "kau_object_detection/object_list.hpp"
#include "kau_object_detection/object_list_diagnostic.hpp"
#include "kau_object_detection/obstacle_tracker.hpp"
#include "kau_object_detection/track_roi.hpp"

namespace
{

using kau_object_detection::ObjectListOptions;
using kau_object_detection::ObjectListStatus;
using kau_object_detection::ObstacleMeasurement;
using kau_object_detection::ObstacleTracker;
using kau_object_detection::ObstacleTrackerOptions;
using kau_object_detection::ObstacleTrackerUpdate;
using kau_object_detection::WorldCircleCandidate;
using kau_object_detection::build_object_list;
using kau_object_detection::build_raw_diagnostic_object_list;

using ObstacleCircleArray = kau_msgs::msg::ObstacleCircleArray;

builtin_interfaces::msg::Time scan_stamp()
{
  builtin_interfaces::msg::Time stamp;
  stamp.sec = 1234;
  stamp.nanosec = 567000000U;
  return stamp;
}

/// Candidate as it leaves the tf2 placement stage: target-frame centre already
/// resolved, nothing smoothed yet. This is what the raw diagnostic reports.
WorldCircleCandidate placed_candidate(
  const double world_x_m,
  const double world_y_m,
  const double radius_m)
{
  WorldCircleCandidate candidate;
  candidate.world_center_valid = true;
  candidate.world_x_m = world_x_m;
  candidate.world_y_m = world_y_m;
  candidate.inside_track_ring = true;
  candidate.sensor_x_m = 0.5;
  candidate.sensor_y_m = -0.25;
  candidate.radius_m = radius_m;
  candidate.source_closest_range_m = 1.5;
  candidate.point_count = 4U;
  return candidate;
}

/// Runs the tracker over `frames` and returns the smoothed candidates of the
/// final frame, exactly the way the node builds its published list.
///
/// `build_diagnostic` mirrors the node's raw diagnostic call: when true, the
/// diagnostic message is built from the same placed candidates before the
/// tracker sees them.
std::vector<WorldCircleCandidate> run_tracker_frames(
  const std::vector<std::vector<WorldCircleCandidate>> & frames,
  const bool build_diagnostic,
  ObstacleTrackerUpdate * last_update = nullptr)
{
  ObstacleTrackerOptions options;
  options.association_distance_m = 0.30;
  options.outlier_distance_m = 0.20;
  options.smoothing_alpha = 0.4;
  options.minimum_confirmation_frames = 2U;
  options.track_timeout_s = 0.7;

  ObstacleTracker tracker(options);
  std::vector<WorldCircleCandidate> published;

  double timestamp_s = 100.0;
  for (const auto & placed : frames) {
    if (build_diagnostic) {
      const auto diagnostic = build_raw_diagnostic_object_list(
        ObjectListStatus::kOk, placed, scan_stamp(), ObjectListOptions{});
      // Consumed so the call cannot be optimised away; the message itself is
      // irrelevant to what the tracker does next.
      EXPECT_EQ(diagnostic.obstacles.size(), placed.size());
    }

    std::vector<ObstacleMeasurement> measurements;
    measurements.reserve(placed.size());
    for (const auto & candidate : placed) {
      measurements.push_back(
        ObstacleMeasurement{candidate.world_x_m, candidate.world_y_m, candidate.radius_m});
    }

    const auto update = tracker.update(measurements, timestamp_s);
    timestamp_s += 0.1;

    published.clear();
    published.reserve(update.obstacles.size());
    for (const auto & obstacle : update.obstacles) {
      WorldCircleCandidate candidate = placed[obstacle.measurement_index];
      candidate.world_x_m = obstacle.x_m;
      candidate.world_y_m = obstacle.y_m;
      published.push_back(candidate);
    }

    if (last_update != nullptr) {
      *last_update = update;
    }
  }

  return published;
}

/// Three frames of one jittering obstacle, enough for the tracker to confirm a
/// track and for the EMA to visibly lag the newest measurement.
std::vector<std::vector<WorldCircleCandidate>> jittering_frames()
{
  return {
    {placed_candidate(1.000, 2.000, 0.09)},
    {placed_candidate(1.060, 2.020, 0.09)},
    {placed_candidate(1.120, 2.040, 0.09)},
  };
}

// ---------------------------------------------------------------------------
// The diagnostic frame never influences the published Object List
// ---------------------------------------------------------------------------

TEST(RawDiagnosticObjectListTest, BuildingTheDiagnosticLeavesTrackerOutputBitIdentical)
{
  const auto frames = jittering_frames();

  ObstacleTrackerUpdate without_diagnostic_update;
  const auto without_diagnostic =
    run_tracker_frames(frames, false, &without_diagnostic_update);

  ObstacleTrackerUpdate with_diagnostic_update;
  const auto with_diagnostic = run_tracker_frames(frames, true, &with_diagnostic_update);

  ASSERT_EQ(without_diagnostic.size(), with_diagnostic.size());
  ASSERT_FALSE(with_diagnostic.empty());
  for (std::size_t index = 0U; index < with_diagnostic.size(); ++index) {
    // Exact equality on purpose: the smoothing must run on the very same
    // arithmetic, not merely on a comparable value.
    EXPECT_EQ(without_diagnostic[index].world_x_m, with_diagnostic[index].world_x_m);
    EXPECT_EQ(without_diagnostic[index].world_y_m, with_diagnostic[index].world_y_m);
    EXPECT_EQ(without_diagnostic[index].radius_m, with_diagnostic[index].radius_m);
    EXPECT_EQ(
      without_diagnostic[index].world_center_valid,
      with_diagnostic[index].world_center_valid);
  }

  EXPECT_EQ(
    without_diagnostic_update.active_tracks, with_diagnostic_update.active_tracks);
  EXPECT_EQ(
    without_diagnostic_update.confirmed_observations,
    with_diagnostic_update.confirmed_observations);
  EXPECT_EQ(
    without_diagnostic_update.rejected_outliers,
    with_diagnostic_update.rejected_outliers);
  ASSERT_EQ(
    without_diagnostic_update.obstacles.size(), with_diagnostic_update.obstacles.size());
  for (std::size_t index = 0U; index < with_diagnostic_update.obstacles.size(); ++index) {
    EXPECT_EQ(
      without_diagnostic_update.obstacles[index].track_id,
      with_diagnostic_update.obstacles[index].track_id);
  }
}

TEST(RawDiagnosticObjectListTest, BuildingTheDiagnosticDoesNotMutateItsCandidates)
{
  const auto original = placed_candidate(1.25, -0.75, 0.09);
  std::vector<WorldCircleCandidate> candidates{original};

  const auto message = build_raw_diagnostic_object_list(
    ObjectListStatus::kOk, candidates, scan_stamp(), ObjectListOptions{});
  EXPECT_EQ(message.obstacles.size(), 1U);

  ASSERT_EQ(candidates.size(), 1U);
  EXPECT_EQ(candidates[0].world_center_valid, original.world_center_valid);
  EXPECT_EQ(candidates[0].world_x_m, original.world_x_m);
  EXPECT_EQ(candidates[0].world_y_m, original.world_y_m);
  EXPECT_EQ(candidates[0].radius_m, original.radius_m);
  EXPECT_EQ(candidates[0].sensor_x_m, original.sensor_x_m);
  EXPECT_EQ(candidates[0].sensor_y_m, original.sensor_y_m);
  EXPECT_EQ(candidates[0].point_count, original.point_count);
}

// ---------------------------------------------------------------------------
// Header and status agreement with the frame it is a diagnostic of
// ---------------------------------------------------------------------------

TEST(RawDiagnosticObjectListTest, RawAndFinalFrameShareStampFrameAndStatus)
{
  const auto frames = jittering_frames();
  const auto & placed = frames.back();
  const auto smoothed = run_tracker_frames(frames, false);

  ObjectListOptions options;
  options.frame_id = "map";
  options.confidence = 1.0F;

  const auto stamp = scan_stamp();
  const auto final_message =
    build_object_list(ObjectListStatus::kOk, smoothed, stamp, options);
  const auto raw_message =
    build_raw_diagnostic_object_list(ObjectListStatus::kOk, placed, stamp, options);

  EXPECT_EQ(raw_message.header.stamp.sec, final_message.header.stamp.sec);
  EXPECT_EQ(raw_message.header.stamp.nanosec, final_message.header.stamp.nanosec);
  EXPECT_EQ(raw_message.header.stamp.sec, stamp.sec);
  EXPECT_EQ(raw_message.header.stamp.nanosec, stamp.nanosec);
  EXPECT_EQ(raw_message.header.frame_id, final_message.header.frame_id);
  EXPECT_EQ(raw_message.status, final_message.status);
  EXPECT_EQ(
    raw_message.status, static_cast<std::uint8_t>(ObjectListStatus::kOk));
}

TEST(RawDiagnosticObjectListTest, RawFrameCarriesUnsmoothedCoordinates)
{
  const auto frames = jittering_frames();
  const auto & placed = frames.back();
  const auto smoothed = run_tracker_frames(frames, false);

  const auto stamp = scan_stamp();
  const auto final_message =
    build_object_list(ObjectListStatus::kOk, smoothed, stamp, ObjectListOptions{});
  const auto raw_message =
    build_raw_diagnostic_object_list(ObjectListStatus::kOk, placed, stamp, ObjectListOptions{});

  ASSERT_EQ(raw_message.obstacles.size(), 1U);
  ASSERT_EQ(final_message.obstacles.size(), 1U);

  EXPECT_FLOAT_EQ(raw_message.obstacles[0].center_x, static_cast<float>(placed[0].world_x_m));
  EXPECT_FLOAT_EQ(raw_message.obstacles[0].center_y, static_cast<float>(placed[0].world_y_m));
  // The EMA lags the newest measurement, so the two frames must not agree.
  EXPECT_NE(raw_message.obstacles[0].center_x, final_message.obstacles[0].center_x);
  // Radius and confidence are carried verbatim on both frames.
  EXPECT_FLOAT_EQ(raw_message.obstacles[0].radius, final_message.obstacles[0].radius);
  EXPECT_FLOAT_EQ(raw_message.obstacles[0].confidence, final_message.obstacles[0].confidence);
}

TEST(RawDiagnosticObjectListTest, TransformFailurePublishesTheSameEmptyNonOkFrame)
{
  const auto stamp = scan_stamp();
  const std::vector<WorldCircleCandidate> placed{placed_candidate(1.0, 2.0, 0.09)};

  // A failed transform empties the published list, and the diagnostic must not
  // report the candidates that the failing frame could not place.
  const auto final_message = build_object_list(
    ObjectListStatus::kTransformUnavailable, {}, stamp, ObjectListOptions{});
  const auto raw_message = build_raw_diagnostic_object_list(
    ObjectListStatus::kTransformUnavailable, placed, stamp, ObjectListOptions{});

  EXPECT_TRUE(raw_message.obstacles.empty());
  EXPECT_TRUE(final_message.obstacles.empty());
  EXPECT_EQ(raw_message.status, final_message.status);
  EXPECT_EQ(
    raw_message.status,
    static_cast<std::uint8_t>(ObjectListStatus::kTransformUnavailable));
  EXPECT_EQ(raw_message.header.stamp.sec, final_message.header.stamp.sec);
  EXPECT_EQ(raw_message.header.stamp.nanosec, final_message.header.stamp.nanosec);
  EXPECT_EQ(raw_message.header.frame_id, final_message.header.frame_id);
}

TEST(RawDiagnosticObjectListTest, LidarUnavailableFramePublishesEmptyOnBothTopics)
{
  const auto stamp = scan_stamp();

  const auto final_message = build_object_list(
    ObjectListStatus::kLidarUnavailable, {}, stamp, ObjectListOptions{});
  const auto raw_message = build_raw_diagnostic_object_list(
    ObjectListStatus::kLidarUnavailable, {}, stamp, ObjectListOptions{});

  EXPECT_TRUE(raw_message.obstacles.empty());
  EXPECT_EQ(raw_message.status, final_message.status);
  EXPECT_EQ(raw_message.header.stamp.sec, final_message.header.stamp.sec);
  EXPECT_EQ(raw_message.header.stamp.nanosec, final_message.header.stamp.nanosec);
}

TEST(RawDiagnosticObjectListTest, UnplacedCandidateIsDroppedFromTheRawFrame)
{
  WorldCircleCandidate unplaced;
  unplaced.world_center_valid = false;
  unplaced.sensor_x_m = 2.0;
  unplaced.sensor_y_m = 1.0;
  unplaced.radius_m = 0.09;

  const std::vector<WorldCircleCandidate> placed{
    placed_candidate(1.0, 2.0, 0.09), unplaced};

  const auto raw_message = build_raw_diagnostic_object_list(
    ObjectListStatus::kOk, placed, scan_stamp(), ObjectListOptions{});

  // Same rule as the published list: a candidate without a resolved centre
  // would be a misplaced obstacle, not an extra observation.
  ASSERT_EQ(raw_message.obstacles.size(), 1U);
  EXPECT_FLOAT_EQ(raw_message.obstacles[0].center_x, 1.0F);
}

}  // namespace
