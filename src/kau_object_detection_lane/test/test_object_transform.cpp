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
// Adapted from package "kau_object_detection", its test
// test_object_list_transform.cpp, on 2026-08-25.
//
// The rigid-transform assertions are carried over unchanged. The track-ring
// cases are gone because this package has no ROI stage, and two cases are new:
// one pinning that the mount offset is never applied twice, and one pinning
// that the stage keeps no state between calls.
// ---------------------------------------------------------------------------

#include <cmath>
#include <limits>
#include <vector>

#include "gtest/gtest.h"
#include "kau_object_detection_lane/object_transform.hpp"

namespace
{

using kau_object_detection_lane::CircleCandidate;
using kau_object_detection_lane::ConeOccupancyCandidate;
using kau_object_detection_lane::Point2D;
using kau_object_detection_lane::SensorToBaseTransform;
using kau_object_detection_lane::make_sensor_to_base_transform;
using kau_object_detection_lane::place_candidates_in_base_frame;
using kau_object_detection_lane::transform_sensor_point_to_base;

/// Quaternion of a planar rotation by `yaw_rad`.
SensorToBaseTransform planar_transform(
  const double x_m, const double y_m, const double yaw_rad)
{
  return make_sensor_to_base_transform(
    x_m, y_m, 0.0, 0.0, std::sin(0.5 * yaw_rad), std::cos(0.5 * yaw_rad));
}

ConeOccupancyCandidate valid_candidate(const double x_m, const double y_m)
{
  ConeOccupancyCandidate candidate;
  candidate.valid = true;
  candidate.center_x_m = x_m;
  candidate.center_y_m = y_m;
  candidate.radius_m = 0.09;
  candidate.source_closest_range_m = std::hypot(x_m, y_m);
  candidate.point_count = 4U;
  candidate.first_scan_index = 10U;
  candidate.last_scan_index = 13U;
  return candidate;
}

/// The measured static mount on this vehicle: base_link <- lidar_link is a
/// pure translation of (-0.027, 0) in the plane, with identity rotation.
SensorToBaseTransform measured_static_mount()
{
  return planar_transform(-0.027, 0.0, 0.0);
}

}  // namespace

TEST(ObjectTransform, IdentityRotationKeepsSensorCoordinates)
{
  const auto transform = planar_transform(0.0, 0.0, 0.0);
  ASSERT_TRUE(transform.valid);

  const auto placed = transform_sensor_point_to_base(Point2D{1.5, -0.4}, transform);
  EXPECT_DOUBLE_EQ(placed.x_m, 1.5);
  EXPECT_DOUBLE_EQ(placed.y_m, -0.4);
}

TEST(ObjectTransform, YawAndTranslationArePlacedInTheBaseFrame)
{
  const auto transform = planar_transform(2.0, -1.0, M_PI_2);
  ASSERT_TRUE(transform.valid);
  EXPECT_NEAR(transform.yaw_rad, M_PI_2, 1e-12);

  // A point 1 m straight ahead of a sensor yawed +90 deg lands 1 m to the left.
  const auto placed = transform_sensor_point_to_base(Point2D{1.0, 0.0}, transform);
  EXPECT_NEAR(placed.x_m, 2.0, 1e-12);
  EXPECT_NEAR(placed.y_m, 0.0, 1e-12);
}

TEST(ObjectTransform, UnnormalisedQuaternionYieldsTheSameYaw)
{
  const double yaw_rad = 0.7;
  const auto normalised = planar_transform(0.0, 0.0, yaw_rad);
  const auto scaled = make_sensor_to_base_transform(
    0.0, 0.0, 0.0, 0.0, 3.0 * std::sin(0.5 * yaw_rad), 3.0 * std::cos(0.5 * yaw_rad));

  ASSERT_TRUE(scaled.valid);
  EXPECT_NEAR(scaled.yaw_rad, normalised.yaw_rad, 1e-12);
}

TEST(ObjectTransform, NonFiniteOrDegenerateTransformIsRejected)
{
  const double nan_value = std::numeric_limits<double>::quiet_NaN();

  EXPECT_FALSE(make_sensor_to_base_transform(nan_value, 0.0, 0.0, 0.0, 0.0, 1.0).valid);
  EXPECT_FALSE(make_sensor_to_base_transform(0.0, nan_value, 0.0, 0.0, 0.0, 1.0).valid);
  EXPECT_FALSE(make_sensor_to_base_transform(0.0, 0.0, nan_value, 0.0, 0.0, 1.0).valid);
  // Zero-length quaternion carries no recoverable heading.
  EXPECT_FALSE(make_sensor_to_base_transform(0.0, 0.0, 0.0, 0.0, 0.0, 0.0).valid);
}

TEST(ObjectTransform, RollAndPitchOfTheTransformAreIgnored)
{
  // Roll of pi about x, no yaw. The planar terms must come out as identity.
  const auto transform = make_sensor_to_base_transform(0.0, 0.0, 1.0, 0.0, 0.0, 0.0);
  ASSERT_TRUE(transform.valid);
  EXPECT_NEAR(std::abs(transform.yaw_rad), 0.0, 1e-12);
}

/// The tf2 lookup already spans base_link <- lidar_link, so the mount
/// translation is inside the transform. Adding it again here would put every
/// obstacle 2.7 cm too far back. This test is the guard against that
/// regression, which is why this package has no lidar_offset_* parameter.
TEST(ObjectTransform, MountOffsetIsNeverAppliedTwice)
{
  const auto transform = measured_static_mount();
  ASSERT_TRUE(transform.valid);

  // A cone measured 2 m straight ahead of the LiDAR sits 2 m - 0.027 m ahead
  // of base_link. Exactly once.
  const auto placed = transform_sensor_point_to_base(Point2D{2.0, 0.0}, transform);
  EXPECT_DOUBLE_EQ(placed.x_m, 2.0 - 0.027);
  EXPECT_DOUBLE_EQ(placed.y_m, 0.0);

  // And not 2 m - 0.054 m, which is what a second application would give.
  EXPECT_NE(placed.x_m, 2.0 - 0.054);
}

TEST(ObjectTransform, AppliedStagePlacesEveryValidCandidate)
{
  const auto transform = measured_static_mount();
  const std::vector<ConeOccupancyCandidate> candidates{
    valid_candidate(1.0, 0.0),
    valid_candidate(2.0, 0.5),
  };

  const auto result = place_candidates_in_base_frame(candidates, transform);

  EXPECT_TRUE(result.applied);
  EXPECT_EQ(result.candidates_before, 2U);
  EXPECT_EQ(result.candidates_after, 2U);
  EXPECT_EQ(result.rejected, 0U);
  ASSERT_EQ(result.accepted.size(), 2U);
  EXPECT_TRUE(result.accepted[0].base_center_valid);
  EXPECT_DOUBLE_EQ(result.accepted[0].base_x_m, 1.0 - 0.027);
  EXPECT_DOUBLE_EQ(result.accepted[1].base_y_m, 0.5);
}

/// Nothing in this package drops a candidate for being off a track boundary:
/// the ring is map data and was not forked in. What keeps walls out is the
/// upstream width and occupancy gate.
TEST(ObjectTransform, NoCandidateIsDroppedForBeingFarFromTheVehicle)
{
  const auto transform = measured_static_mount();
  const std::vector<ConeOccupancyCandidate> candidates{
    valid_candidate(0.5, 0.0),
    valid_candidate(25.0, -18.0),
  };

  const auto result = place_candidates_in_base_frame(candidates, transform);

  EXPECT_TRUE(result.applied);
  ASSERT_EQ(result.accepted.size(), 2U);
  EXPECT_TRUE(result.accepted[1].base_center_valid);
}

TEST(ObjectTransform, CarriesTheCircleThroughUnchanged)
{
  const auto transform = measured_static_mount();
  const auto source = valid_candidate(1.25, -0.75);

  const auto result = place_candidates_in_base_frame({source}, transform);
  ASSERT_EQ(result.accepted.size(), 1U);

  const CircleCandidate & carried = result.accepted.front();
  EXPECT_DOUBLE_EQ(carried.sensor_x_m, source.center_x_m);
  EXPECT_DOUBLE_EQ(carried.sensor_y_m, source.center_y_m);
  EXPECT_DOUBLE_EQ(carried.radius_m, source.radius_m);
  EXPECT_DOUBLE_EQ(carried.source_closest_range_m, source.source_closest_range_m);
  EXPECT_EQ(carried.point_count, source.point_count);
  EXPECT_EQ(carried.first_scan_index, source.first_scan_index);
  EXPECT_EQ(carried.last_scan_index, source.last_scan_index);
}

TEST(ObjectTransform, InvalidOccupancyCandidatesNeverEnter)
{
  const auto transform = measured_static_mount();
  ConeOccupancyCandidate rejected_candidate = valid_candidate(1.0, 0.0);
  rejected_candidate.valid = false;

  const auto result =
    place_candidates_in_base_frame({rejected_candidate, valid_candidate(2.0, 0.0)}, transform);

  EXPECT_EQ(result.candidates_before, 1U);
  ASSERT_EQ(result.accepted.size(), 1U);
  EXPECT_DOUBLE_EQ(result.accepted.front().sensor_x_m, 2.0);
}

TEST(ObjectTransform, InvalidTransformFailsOpenWithoutBaseCoordinates)
{
  SensorToBaseTransform missing_transform;
  ASSERT_FALSE(missing_transform.valid);

  const auto result =
    place_candidates_in_base_frame({valid_candidate(1.0, 0.0)}, missing_transform);

  // Fail open: the candidate is carried, but with no base-frame centre, which
  // is what turns the frame into STATUS_TRANSFORM_UNAVAILABLE upstream.
  EXPECT_FALSE(result.applied);
  ASSERT_EQ(result.accepted.size(), 1U);
  EXPECT_FALSE(result.accepted.front().base_center_valid);
}

/// The placement stage is the only thing between detection and the wire, so it
/// is the natural place for state to creep in. It has none: the same inputs
/// give the same outputs, and an empty call after a populated one returns
/// nothing.
TEST(ObjectTransform, PlacementKeepsNoStateBetweenCalls)
{
  const auto transform = measured_static_mount();

  const auto first = place_candidates_in_base_frame(
    {valid_candidate(1.0, 0.0), valid_candidate(2.0, 0.0)}, transform);
  ASSERT_EQ(first.accepted.size(), 2U);

  const auto empty = place_candidates_in_base_frame({}, transform);
  EXPECT_TRUE(empty.accepted.empty());
  EXPECT_EQ(empty.candidates_before, 0U);
  EXPECT_EQ(empty.candidates_after, 0U);

  const auto repeated = place_candidates_in_base_frame(
    {valid_candidate(1.0, 0.0), valid_candidate(2.0, 0.0)}, transform);
  ASSERT_EQ(repeated.accepted.size(), first.accepted.size());
  for (std::size_t index = 0U; index < first.accepted.size(); ++index) {
    EXPECT_DOUBLE_EQ(repeated.accepted[index].base_x_m, first.accepted[index].base_x_m);
    EXPECT_DOUBLE_EQ(repeated.accepted[index].base_y_m, first.accepted[index].base_y_m);
  }
}
