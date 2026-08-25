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
// Adapted from package "kau_object_detection", its test test_object_list.cpp,
// on 2026-08-25.
//
// The message-building assertions are carried over. The status cases that
// exercised the Gazebo world pose and the track ring are gone with those code
// paths; only the placement decision this package publishes on remains.
// ---------------------------------------------------------------------------

#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "gtest/gtest.h"
#include "kau_msgs/msg/obstacle_circle_array.hpp"
#include "kau_object_detection_lane/object_list.hpp"

namespace
{

using kau_object_detection_lane::CircleCandidate;
using kau_object_detection_lane::ObjectListOptions;
using kau_object_detection_lane::ObjectListStatus;
using kau_object_detection_lane::build_object_list;
using kau_object_detection_lane::decide_object_list_status_from_placement;

using ObstacleCircleArray = kau_msgs::msg::ObstacleCircleArray;

builtin_interfaces::msg::Time scan_stamp()
{
  builtin_interfaces::msg::Time stamp;
  stamp.sec = 1234;
  stamp.nanosec = 567000000U;
  return stamp;
}

CircleCandidate placed_candidate(
  const double x_m, const double y_m, const double radius_m)
{
  CircleCandidate candidate;
  candidate.base_center_valid = true;
  candidate.base_x_m = x_m;
  candidate.base_y_m = y_m;
  candidate.radius_m = radius_m;
  candidate.sensor_x_m = x_m + 0.027;
  candidate.sensor_y_m = y_m;
  return candidate;
}

}  // namespace

TEST(ObjectListStatusTest, AllPreconditionsHeldReportsOk)
{
  EXPECT_EQ(
    decide_object_list_status_from_placement(true, true, true),
    ObjectListStatus::kOk);
}

TEST(ObjectListStatusTest, InvalidScanDominatesEveryOtherFailure)
{
  EXPECT_EQ(
    decide_object_list_status_from_placement(false, false, false),
    ObjectListStatus::kLidarUnavailable);
  EXPECT_EQ(
    decide_object_list_status_from_placement(false, true, true),
    ObjectListStatus::kLidarUnavailable);
}

TEST(ObjectListStatusTest, MissingTransformIsReportedAsTransformUnavailable)
{
  EXPECT_EQ(
    decide_object_list_status_from_placement(true, false, false),
    ObjectListStatus::kTransformUnavailable);
}

/// A transform the lookup called valid, yet a placement stage that still
/// failed open, is an internal inconsistency of this node and is named as one
/// rather than being passed off as a missing transform.
TEST(ObjectListStatusTest, UnplacedCandidatesWithAValidTransformIsInternal)
{
  EXPECT_EQ(
    decide_object_list_status_from_placement(true, true, false),
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

TEST(ObjectListBuildTest, ConvertsBaseCentreRadiusAndConfidence)
{
  const auto message = build_object_list(
    ObjectListStatus::kOk, {placed_candidate(1.25, -0.75, 0.09)}, scan_stamp());

  ASSERT_EQ(message.obstacles.size(), 1U);
  EXPECT_FLOAT_EQ(message.obstacles[0].center_x, 1.25F);
  EXPECT_FLOAT_EQ(message.obstacles[0].center_y, -0.75F);
  EXPECT_FLOAT_EQ(message.obstacles[0].radius, 0.09F);
  EXPECT_FLOAT_EQ(message.obstacles[0].confidence, 1.0F);
}

/// The Lane-only contract: the default output frame is base_link, so a
/// consumer that never configured anything still gets vehicle-relative
/// coordinates rather than a map frame that does not exist.
TEST(ObjectListBuildTest, DefaultFrameIsBaseLink)
{
  const ObjectListOptions defaults;
  EXPECT_EQ(defaults.frame_id, std::string{"base_link"});

  const auto message = build_object_list(
    ObjectListStatus::kOk, {placed_candidate(1.0, 0.0, 0.09)}, scan_stamp());
  EXPECT_EQ(message.header.frame_id, std::string{"base_link"});
}

TEST(ObjectListBuildTest, CarriesScanStampAndConfiguredFrame)
{
  ObjectListOptions options;
  options.frame_id = "base_footprint";

  const auto message = build_object_list(
    ObjectListStatus::kOk, {placed_candidate(1.0, 0.0, 0.09)}, scan_stamp(), options);

  EXPECT_EQ(message.header.stamp.sec, 1234);
  EXPECT_EQ(message.header.stamp.nanosec, 567000000U);
  EXPECT_EQ(message.header.frame_id, std::string{"base_footprint"});
}

TEST(ObjectListBuildTest, KeepsCandidateOrder)
{
  const std::vector<CircleCandidate> candidates{
    placed_candidate(1.0, 0.0, 0.09),
    placed_candidate(2.0, 0.0, 0.09),
    placed_candidate(3.0, 0.0, 0.09),
  };

  const auto message = build_object_list(ObjectListStatus::kOk, candidates, scan_stamp());

  ASSERT_EQ(message.obstacles.size(), 3U);
  EXPECT_FLOAT_EQ(message.obstacles[0].center_x, 1.0F);
  EXPECT_FLOAT_EQ(message.obstacles[1].center_x, 2.0F);
  EXPECT_FLOAT_EQ(message.obstacles[2].center_x, 3.0F);
}

TEST(ObjectListBuildTest, DropsCandidatesWithoutResolvedBaseCentre)
{
  CircleCandidate unplaced;
  unplaced.sensor_x_m = 9.0;
  unplaced.sensor_y_m = 9.0;
  unplaced.radius_m = 0.09;
  ASSERT_FALSE(unplaced.base_center_valid);

  const auto message = build_object_list(
    ObjectListStatus::kOk, {unplaced, placed_candidate(1.0, 0.0, 0.09)}, scan_stamp());

  ASSERT_EQ(message.obstacles.size(), 1U);
  EXPECT_FLOAT_EQ(message.obstacles[0].center_x, 1.0F);
}

/// Publishing a sensor-frame value under the base frame would misplace the
/// obstacle by the mount offset instead of omitting it.
TEST(ObjectListBuildTest, NeverPublishesSensorFrameValues)
{
  const auto candidate = placed_candidate(1.0, 0.0, 0.09);
  ASSERT_NE(candidate.sensor_x_m, candidate.base_x_m);

  const auto message =
    build_object_list(ObjectListStatus::kOk, {candidate}, scan_stamp());

  ASSERT_EQ(message.obstacles.size(), 1U);
  EXPECT_FLOAT_EQ(message.obstacles[0].center_x, static_cast<float>(candidate.base_x_m));
  EXPECT_NE(message.obstacles[0].center_x, static_cast<float>(candidate.sensor_x_m));
}

TEST(ObjectListBuildTest, HonoursConfiguredConfidence)
{
  ObjectListOptions options;
  options.confidence = 0.5F;

  const auto message = build_object_list(
    ObjectListStatus::kOk, {placed_candidate(1.0, 0.0, 0.09)}, scan_stamp(), options);

  ASSERT_EQ(message.obstacles.size(), 1U);
  EXPECT_FLOAT_EQ(message.obstacles[0].confidence, 0.5F);
}

/// STATUS_OK with an empty array means "observed, nothing there". It is a
/// usable result and must not be confused with a failure.
TEST(ObjectListBuildTest, NoObservedObstacleStaysOk)
{
  const auto message = build_object_list(ObjectListStatus::kOk, {}, scan_stamp());

  EXPECT_EQ(message.status, ObstacleCircleArray::STATUS_OK);
  EXPECT_TRUE(message.obstacles.empty());
}

TEST(ObjectListBuildTest, EveryFailureStatusPublishesEmptyList)
{
  const std::vector<CircleCandidate> candidates{placed_candidate(1.0, 0.0, 0.09)};

  for (const auto status : {
    ObjectListStatus::kLidarUnavailable,
    ObjectListStatus::kPoseUnavailable,
    ObjectListStatus::kTransformUnavailable,
    ObjectListStatus::kInternalError,
  })
  {
    const auto message = build_object_list(status, candidates, scan_stamp());
    EXPECT_EQ(message.status, static_cast<std::uint8_t>(status));
    EXPECT_TRUE(message.obstacles.empty())
      << "status " << static_cast<int>(static_cast<std::uint8_t>(status))
      << " published an obstacle";
  }
}

/// The builder is stateless, so a failing frame cannot carry an obstacle from
/// the frame before it. This is the message-level half of the guarantee that
/// test_no_temporal_persistence.cpp makes for the whole pipeline.
TEST(ObjectListBuildTest, FailingFrameNeverReusesPreviousObstacles)
{
  const std::vector<CircleCandidate> candidates{placed_candidate(1.0, 0.0, 0.09)};

  const auto good = build_object_list(ObjectListStatus::kOk, candidates, scan_stamp());
  ASSERT_EQ(good.obstacles.size(), 1U);

  const auto failed =
    build_object_list(ObjectListStatus::kTransformUnavailable, candidates, scan_stamp());
  EXPECT_TRUE(failed.obstacles.empty());
}

TEST(ObjectListBuildTest, RecoversOnTheFrameAfterAFailure)
{
  const std::vector<CircleCandidate> candidates{placed_candidate(1.0, 0.0, 0.09)};

  const auto failed =
    build_object_list(ObjectListStatus::kTransformUnavailable, candidates, scan_stamp());
  ASSERT_TRUE(failed.obstacles.empty());

  const auto recovered = build_object_list(ObjectListStatus::kOk, candidates, scan_stamp());
  ASSERT_EQ(recovered.obstacles.size(), 1U);
  EXPECT_FLOAT_EQ(recovered.obstacles[0].center_x, 1.0F);
}
