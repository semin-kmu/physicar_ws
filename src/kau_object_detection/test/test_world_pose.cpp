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

#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "kau_object_detection/world_pose.hpp"

namespace
{

using kau_object_detection::WorldPoseCandidate;
using kau_object_detection::WorldPoseSample;

constexpr double kPi = 3.14159265358979323846;

WorldPoseCandidate make_candidate(
  const std::string & child_frame_id,
  const double x_m,
  const double y_m,
  const double yaw_rad,
  const double stamp_s = 1.0,
  const std::string & name = "")
{
  WorldPoseCandidate candidate;
  candidate.name = name;
  candidate.child_frame_id = child_frame_id;
  candidate.x_m = x_m;
  candidate.y_m = y_m;
  candidate.orientation_w = std::cos(0.5 * yaw_rad);
  candidate.orientation_x = 0.0;
  candidate.orientation_y = 0.0;
  candidate.orientation_z = std::sin(0.5 * yaw_rad);
  candidate.stamp_s = stamp_s;
  return candidate;
}

TEST(WorldPose, YawIsRecoveredFromAPlanarQuaternion)
{
  for (const double yaw_rad : {0.0, 0.5, -0.5, 1.5, -2.5, 3.0}) {
    const double recovered = kau_object_detection::quaternion_to_yaw_rad(
      std::cos(0.5 * yaw_rad), 0.0, 0.0, std::sin(0.5 * yaw_rad));
    EXPECT_NEAR(recovered, yaw_rad, 1e-9) << "yaw_rad=" << yaw_rad;
  }
}

TEST(WorldPose, YawMatchesTheObservedSimulatorOrientation)
{
  // Orientation sampled from the live /model/physicar/pose message. Only the
  // quaternion is reused here; no map position is assumed.
  const double yaw_rad = kau_object_detection::quaternion_to_yaw_rad(
    0.94251595010331046, 1.7967889096007855e-09, 3.8924247748246579e-09,
    -0.33416116441150662);

  EXPECT_NEAR(yaw_rad, 2.0 * std::atan2(-0.33416116441150662, 0.94251595010331046), 1e-9);
  EXPECT_LT(yaw_rad, 0.0);
  EXPECT_NEAR(kau_object_detection::radians_to_degrees(yaw_rad), -39.05, 0.05);
}

TEST(WorldPose, DegenerateQuaternionYieldsZeroYaw)
{
  EXPECT_NEAR(kau_object_detection::quaternion_to_yaw_rad(0.0, 0.0, 0.0, 0.0), 0.0, 1e-12);
}

TEST(WorldPose, RadiansToDegreesMatchesTheStandardConversion)
{
  EXPECT_NEAR(kau_object_detection::radians_to_degrees(kPi), 180.0, 1e-9);
  EXPECT_NEAR(kau_object_detection::radians_to_degrees(-kPi / 2.0), -90.0, 1e-9);
}

TEST(WorldPose, SelectsTheCandidateMatchingTheConfiguredBaseFrame)
{
  const std::vector<WorldPoseCandidate> candidates{
    make_candidate("lidar_link", 9.0, 9.0, 0.0),
    make_candidate("base_footprint", 1.5, -2.5, 0.75, 42.0),
  };

  const auto sample =
    kau_object_detection::select_world_base_pose(candidates, "base_footprint", 100.0);

  ASSERT_TRUE(sample.valid);
  EXPECT_TRUE(sample.exact_name_match);
  EXPECT_EQ(sample.source_name, "base_footprint");
  EXPECT_NEAR(sample.x_m, 1.5, 1e-12);
  EXPECT_NEAR(sample.y_m, -2.5, 1e-12);
  EXPECT_NEAR(sample.yaw_rad, 0.75, 1e-9);
  EXPECT_NEAR(sample.stamp_s, 42.0, 1e-12);
  EXPECT_NEAR(sample.received_monotonic_s, 100.0, 1e-12);
}

TEST(WorldPose, MatchesOnModelNameWhenChildFrameIsEmpty)
{
  const std::vector<WorldPoseCandidate> candidates{
    make_candidate("", 3.0, 4.0, 0.25, 1.0, "physicar"),
  };

  const auto sample =
    kau_object_detection::select_world_base_pose(candidates, "physicar", 10.0);

  ASSERT_TRUE(sample.valid);
  EXPECT_TRUE(sample.exact_name_match);
  EXPECT_EQ(sample.source_name, "physicar");
}

TEST(WorldPose, FallsBackToTheFirstEntryWhenNothingMatches)
{
  // The live message carries a single unnamed pose whose header identifies it
  // as base_footprint, so the fallback must still produce a usable sample.
  const std::vector<WorldPoseCandidate> candidates{
    make_candidate("base_footprint", 1.0, 2.0, 0.5),
  };

  const auto sample =
    kau_object_detection::select_world_base_pose(candidates, "not_present", 5.0);

  ASSERT_TRUE(sample.valid);
  EXPECT_FALSE(sample.exact_name_match);
  EXPECT_NEAR(sample.x_m, 1.0, 1e-12);
}

TEST(WorldPose, EmptyBaseFrameAlwaysUsesTheFirstUsableEntry)
{
  const std::vector<WorldPoseCandidate> candidates{
    make_candidate("base_footprint", 7.0, 8.0, 0.0),
    make_candidate("lidar_link", 1.0, 1.0, 0.0),
  };

  const auto sample = kau_object_detection::select_world_base_pose(candidates, "", 1.0);

  ASSERT_TRUE(sample.valid);
  EXPECT_FALSE(sample.exact_name_match);
  EXPECT_NEAR(sample.x_m, 7.0, 1e-12);
}

TEST(WorldPose, EmptyMessageProducesAnInvalidSample)
{
  const auto sample = kau_object_detection::select_world_base_pose({}, "base_footprint", 1.0);

  EXPECT_FALSE(sample.valid);
  EXPECT_FALSE(sample.exact_name_match);
}

TEST(WorldPose, NonFiniteOrDegenerateCandidatesAreSkipped)
{
  const double nan_value = std::numeric_limits<double>::quiet_NaN();

  WorldPoseCandidate broken_position = make_candidate("base_footprint", 1.0, 2.0, 0.0);
  broken_position.x_m = nan_value;

  WorldPoseCandidate zero_quaternion = make_candidate("base_footprint", 3.0, 4.0, 0.0);
  zero_quaternion.orientation_w = 0.0;
  zero_quaternion.orientation_z = 0.0;

  WorldPoseCandidate broken_stamp = make_candidate("base_footprint", 5.0, 6.0, 0.0);
  broken_stamp.stamp_s = nan_value;

  const WorldPoseCandidate usable = make_candidate("chassis", 8.0, 9.0, 0.0);

  const auto sample = kau_object_detection::select_world_base_pose(
    {broken_position, zero_quaternion, broken_stamp, usable}, "base_footprint", 1.0);

  ASSERT_TRUE(sample.valid);
  EXPECT_FALSE(sample.exact_name_match);
  EXPECT_NEAR(sample.x_m, 8.0, 1e-12);
}

TEST(WorldPose, AllUnusableCandidatesProduceAnInvalidSample)
{
  WorldPoseCandidate broken = make_candidate("base_footprint", 1.0, 2.0, 0.0);
  broken.y_m = std::numeric_limits<double>::infinity();

  const auto sample =
    kau_object_detection::select_world_base_pose({broken}, "base_footprint", 1.0);

  EXPECT_FALSE(sample.valid);
}

TEST(WorldPose, NonFiniteReceiptTimeProducesAnInvalidSample)
{
  const std::vector<WorldPoseCandidate> candidates{
    make_candidate("base_footprint", 1.0, 2.0, 0.0),
  };

  const auto sample = kau_object_detection::select_world_base_pose(
    candidates, "base_footprint", std::numeric_limits<double>::quiet_NaN());

  EXPECT_FALSE(sample.valid);
}

TEST(WorldPose, AgeIsInfiniteWithoutASample)
{
  const WorldPoseSample missing;

  EXPECT_FALSE(std::isfinite(kau_object_detection::world_pose_age_s(missing, 10.0)));
  EXPECT_FALSE(kau_object_detection::world_pose_is_fresh(missing, 10.0, 0.5));
}

TEST(WorldPose, FreshnessUsesTheMonotonicReceiptTime)
{
  const std::vector<WorldPoseCandidate> candidates{
    make_candidate("base_footprint", 1.0, 2.0, 0.0, 3579.695),
  };
  const auto sample =
    kau_object_detection::select_world_base_pose(candidates, "base_footprint", 100.0);
  ASSERT_TRUE(sample.valid);

  EXPECT_NEAR(kau_object_detection::world_pose_age_s(sample, 100.1), 0.1, 1e-12);
  EXPECT_TRUE(kau_object_detection::world_pose_is_fresh(sample, 100.1, 0.5));
  EXPECT_TRUE(kau_object_detection::world_pose_is_fresh(sample, 100.5, 0.5));
  EXPECT_FALSE(kau_object_detection::world_pose_is_fresh(sample, 100.6, 0.5));

  // The simulation stamp is far from the monotonic clock and must not leak into
  // the freshness decision.
  EXPECT_NEAR(sample.stamp_s, 3579.695, 1e-9);
}

TEST(WorldPose, NonPositiveOrNonFiniteTimeoutIsNeverFresh)
{
  const std::vector<WorldPoseCandidate> candidates{
    make_candidate("base_footprint", 1.0, 2.0, 0.0),
  };
  const auto sample =
    kau_object_detection::select_world_base_pose(candidates, "base_footprint", 100.0);
  ASSERT_TRUE(sample.valid);

  EXPECT_FALSE(kau_object_detection::world_pose_is_fresh(sample, 100.0, 0.0));
  EXPECT_FALSE(kau_object_detection::world_pose_is_fresh(sample, 100.0, -1.0));
  EXPECT_FALSE(
    kau_object_detection::world_pose_is_fresh(
      sample, 100.0, std::numeric_limits<double>::quiet_NaN()));
}

}  // namespace
