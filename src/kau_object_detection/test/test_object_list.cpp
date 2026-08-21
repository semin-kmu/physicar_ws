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
#include "kau_object_detection/track_roi.hpp"

namespace
{

using kau_object_detection::ObjectListOptions;
using kau_object_detection::ObjectListStatus;
using kau_object_detection::WorldCircleCandidate;
using kau_object_detection::build_object_list;
using kau_object_detection::decide_object_list_status;

using ObstacleCircleArray = kau_msgs::msg::ObstacleCircleArray;

builtin_interfaces::msg::Time scan_stamp()
{
  builtin_interfaces::msg::Time stamp;
  stamp.sec = 1234;
  stamp.nanosec = 567000000U;
  return stamp;
}

/// Candidate whose centre the ROI stage resolved to the world frame.
WorldCircleCandidate resolved_candidate(
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

/// Candidate the ROI stage handed back unresolved, so only the sensor-frame
/// values are meaningful. This is what the fail-open path produces.
WorldCircleCandidate unresolved_candidate()
{
  WorldCircleCandidate candidate;
  candidate.world_center_valid = false;
  candidate.sensor_x_m = 2.0;
  candidate.sensor_y_m = 1.0;
  candidate.radius_m = 0.09;
  candidate.source_closest_range_m = 2.2;
  candidate.point_count = 3U;
  return candidate;
}

// ---------------------------------------------------------------------------
// Status decision
// ---------------------------------------------------------------------------

TEST(ObjectListStatusTest, AllPreconditionsHeldReportsOk)
{
  EXPECT_EQ(decide_object_list_status(true, true, true, true), ObjectListStatus::kOk);
}

TEST(ObjectListStatusTest, InvalidScanDominatesEveryOtherFailure)
{
  EXPECT_EQ(
    decide_object_list_status(false, false, false, false),
    ObjectListStatus::kLidarUnavailable);
}

TEST(ObjectListStatusTest, StaleWorldPoseReportsPoseUnavailable)
{
  EXPECT_EQ(
    decide_object_list_status(true, false, true, false),
    ObjectListStatus::kPoseUnavailable);
}

TEST(ObjectListStatusTest, InvalidTrackRingReportsInternalError)
{
  EXPECT_EQ(
    decide_object_list_status(true, true, false, false),
    ObjectListStatus::kInternalError);
}

TEST(ObjectListStatusTest, UnappliedRoiNeverReportsOk)
{
  // Every named precondition held, yet the ROI stage still failed open. The
  // frame must not claim a usable result.
  EXPECT_EQ(
    decide_object_list_status(true, true, true, false),
    ObjectListStatus::kInternalError);
}

TEST(ObjectListStatusTest, EnumMatchesMessageConstants)
{
  EXPECT_EQ(static_cast<std::uint8_t>(ObjectListStatus::kOk), ObstacleCircleArray::STATUS_OK);
  EXPECT_EQ(
    static_cast<std::uint8_t>(ObjectListStatus::kLidarUnavailable),
    ObstacleCircleArray::STATUS_LIDAR_UNAVAILABLE);
  EXPECT_EQ(
    static_cast<std::uint8_t>(ObjectListStatus::kPoseUnavailable),
    ObstacleCircleArray::STATUS_POSE_UNAVAILABLE);
  EXPECT_EQ(
    static_cast<std::uint8_t>(ObjectListStatus::kTransformUnavailable),
    ObstacleCircleArray::STATUS_TRANSFORM_UNAVAILABLE);
  EXPECT_EQ(
    static_cast<std::uint8_t>(ObjectListStatus::kInternalError),
    ObstacleCircleArray::STATUS_INTERNAL_ERROR);
}

// ---------------------------------------------------------------------------
// Conversion
// ---------------------------------------------------------------------------

TEST(ObjectListBuildTest, ConvertsWorldCentreRadiusAndConfidence)
{
  const std::vector<WorldCircleCandidate> candidates{
    resolved_candidate(1.25, -2.5, 0.09)};

  const auto message =
    build_object_list(ObjectListStatus::kOk, candidates, scan_stamp(), ObjectListOptions{});

  ASSERT_EQ(message.obstacles.size(), 1U);
  EXPECT_FLOAT_EQ(message.obstacles[0].center_x, 1.25F);
  EXPECT_FLOAT_EQ(message.obstacles[0].center_y, -2.5F);
  EXPECT_FLOAT_EQ(message.obstacles[0].radius, 0.09F);
  EXPECT_FLOAT_EQ(message.obstacles[0].confidence, 1.0F);
  EXPECT_EQ(message.status, ObstacleCircleArray::STATUS_OK);
}

TEST(ObjectListBuildTest, CarriesScanStampAndConfiguredFrame)
{
  const auto message = build_object_list(
    ObjectListStatus::kOk, {}, scan_stamp(), ObjectListOptions{});

  EXPECT_EQ(message.header.stamp.sec, 1234);
  EXPECT_EQ(message.header.stamp.nanosec, 567000000U);
  EXPECT_EQ(message.header.frame_id, "map");
}

TEST(ObjectListBuildTest, KeepsCandidateOrder)
{
  const std::vector<WorldCircleCandidate> candidates{
    resolved_candidate(1.0, 0.0, 0.09),
    resolved_candidate(2.0, 0.0, 0.09),
    resolved_candidate(3.0, 0.0, 0.09)};

  const auto message =
    build_object_list(ObjectListStatus::kOk, candidates, scan_stamp(), ObjectListOptions{});

  ASSERT_EQ(message.obstacles.size(), 3U);
  EXPECT_FLOAT_EQ(message.obstacles[0].center_x, 1.0F);
  EXPECT_FLOAT_EQ(message.obstacles[1].center_x, 2.0F);
  EXPECT_FLOAT_EQ(message.obstacles[2].center_x, 3.0F);
}

TEST(ObjectListBuildTest, DropsCandidatesWithoutResolvedWorldCentre)
{
  const std::vector<WorldCircleCandidate> candidates{
    unresolved_candidate(),
    resolved_candidate(4.0, 5.0, 0.09),
    unresolved_candidate()};

  const auto message =
    build_object_list(ObjectListStatus::kOk, candidates, scan_stamp(), ObjectListOptions{});

  ASSERT_EQ(message.obstacles.size(), 1U);
  EXPECT_FLOAT_EQ(message.obstacles[0].center_x, 4.0F);
  EXPECT_FLOAT_EQ(message.obstacles[0].center_y, 5.0F);
}

TEST(ObjectListBuildTest, NeverPublishesSensorFrameValues)
{
  const std::vector<WorldCircleCandidate> candidates{unresolved_candidate()};

  const auto message =
    build_object_list(ObjectListStatus::kOk, candidates, scan_stamp(), ObjectListOptions{});

  // The unresolved candidate carries sensor_x_m = 2.0. Nothing may leak it into
  // a message that claims the map frame.
  EXPECT_TRUE(message.obstacles.empty());
  EXPECT_EQ(message.status, ObstacleCircleArray::STATUS_OK);
}

TEST(ObjectListBuildTest, HonoursConfiguredConfidenceAndFrame)
{
  ObjectListOptions options;
  options.frame_id = "odom";
  options.confidence = 0.5F;

  const std::vector<WorldCircleCandidate> candidates{resolved_candidate(1.0, 1.0, 0.09)};
  const auto message =
    build_object_list(ObjectListStatus::kOk, candidates, scan_stamp(), options);

  ASSERT_EQ(message.obstacles.size(), 1U);
  EXPECT_FLOAT_EQ(message.obstacles[0].confidence, 0.5F);
  EXPECT_EQ(message.header.frame_id, "odom");
}

// ---------------------------------------------------------------------------
// Empty and failing frames
// ---------------------------------------------------------------------------

TEST(ObjectListBuildTest, NoObservedObstacleStaysOk)
{
  const auto message =
    build_object_list(ObjectListStatus::kOk, {}, scan_stamp(), ObjectListOptions{});

  EXPECT_EQ(message.status, ObstacleCircleArray::STATUS_OK);
  EXPECT_TRUE(message.obstacles.empty());
}

TEST(ObjectListBuildTest, EveryRoiRejectionStillReportsOk)
{
  // The ROI stage accepted nothing, but the observation itself was sound.
  const std::vector<WorldCircleCandidate> accepted;
  const auto message =
    build_object_list(ObjectListStatus::kOk, accepted, scan_stamp(), ObjectListOptions{});

  EXPECT_EQ(message.status, ObstacleCircleArray::STATUS_OK);
  EXPECT_TRUE(message.obstacles.empty());
}

TEST(ObjectListBuildTest, EveryFailureStatusPublishesEmptyList)
{
  const std::vector<WorldCircleCandidate> candidates{
    resolved_candidate(1.0, 2.0, 0.09),
    resolved_candidate(3.0, 4.0, 0.09)};

  const ObjectListStatus failures[] = {
    ObjectListStatus::kLidarUnavailable,
    ObjectListStatus::kPoseUnavailable,
    ObjectListStatus::kTransformUnavailable,
    ObjectListStatus::kInternalError};

  for (const auto status : failures) {
    const auto message =
      build_object_list(status, candidates, scan_stamp(), ObjectListOptions{});

    EXPECT_EQ(message.status, static_cast<std::uint8_t>(status));
    EXPECT_TRUE(message.obstacles.empty());
    EXPECT_EQ(message.header.frame_id, "map");
  }
}

TEST(ObjectListBuildTest, FailingFrameNeverReusesPreviousObstacles)
{
  const std::vector<WorldCircleCandidate> candidates{resolved_candidate(7.0, 8.0, 0.09)};

  const auto healthy =
    build_object_list(ObjectListStatus::kOk, candidates, scan_stamp(), ObjectListOptions{});
  ASSERT_EQ(healthy.obstacles.size(), 1U);

  // Same candidates, failing status: the previous result must not survive.
  const auto degraded = build_object_list(
    ObjectListStatus::kPoseUnavailable, candidates, scan_stamp(), ObjectListOptions{});
  EXPECT_TRUE(degraded.obstacles.empty());

  // And the healthy frame built earlier is untouched, so nothing is shared
  // between frames.
  EXPECT_EQ(healthy.obstacles.size(), 1U);
}

TEST(ObjectListBuildTest, RecoversOnTheFrameAfterAFailure)
{
  const std::vector<WorldCircleCandidate> candidates{resolved_candidate(9.0, 1.0, 0.09)};

  const auto degraded = build_object_list(
    ObjectListStatus::kLidarUnavailable, {}, scan_stamp(), ObjectListOptions{});
  ASSERT_TRUE(degraded.obstacles.empty());

  const auto recovered =
    build_object_list(ObjectListStatus::kOk, candidates, scan_stamp(), ObjectListOptions{});

  ASSERT_EQ(recovered.obstacles.size(), 1U);
  EXPECT_FLOAT_EQ(recovered.obstacles[0].center_x, 9.0F);
}

}  // namespace
