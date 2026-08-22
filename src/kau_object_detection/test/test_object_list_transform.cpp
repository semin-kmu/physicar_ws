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
#include <vector>

#include "gtest/gtest.h"
#include "kau_object_detection/object_list.hpp"
#include "kau_object_detection/object_list_transform.hpp"
#include "kau_object_detection/track_geometry.hpp"

namespace
{

using kau_object_detection::ConeOccupancyCandidate;
using kau_object_detection::ObjectListStatus;
using kau_object_detection::Point2D;
using kau_object_detection::SensorToTargetTransform;
using kau_object_detection::TrackRing;
using kau_object_detection::apply_object_list_transform;
using kau_object_detection::decide_object_list_status_from_placement;
using kau_object_detection::decide_object_list_status_from_transform;
using kau_object_detection::make_sensor_to_target_transform;
using kau_object_detection::make_track_ring;
using kau_object_detection::place_candidates_in_target_frame;
using kau_object_detection::transform_sensor_point_to_target;

constexpr double kTolerance = 1e-9;

/// Quaternion of a pure yaw rotation, the only rotation a planar tf2 chain
/// between map and a level LiDAR is expected to carry.
SensorToTargetTransform yaw_transform(
  const double x_m,
  const double y_m,
  const double yaw_rad)
{
  return make_sensor_to_target_transform(
    x_m, y_m, 0.0, 0.0, std::sin(0.5 * yaw_rad), std::cos(0.5 * yaw_rad));
}

/// Square ring 10 m across with a 2 m hole in the middle.
TrackRing square_ring()
{
  const std::vector<Point2D> outer{{-5.0, -5.0}, {5.0, -5.0}, {5.0, 5.0}, {-5.0, 5.0}};
  const std::vector<Point2D> inner{{-1.0, -1.0}, {1.0, -1.0}, {1.0, 1.0}, {-1.0, 1.0}};
  return make_track_ring(outer, inner);
}

ConeOccupancyCandidate valid_candidate(const double center_x_m, const double center_y_m)
{
  ConeOccupancyCandidate candidate;
  candidate.valid = true;
  candidate.center_x_m = center_x_m;
  candidate.center_y_m = center_y_m;
  candidate.radius_m = 0.09;
  candidate.source_closest_range_m = 1.0;
  candidate.point_count = 3U;
  candidate.first_scan_index = 10U;
  candidate.last_scan_index = 12U;
  return candidate;
}

TEST(ObjectListTransform, IdentityRotationKeepsSensorCoordinates)
{
  const auto transform = make_sensor_to_target_transform(0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
  ASSERT_TRUE(transform.valid);
  EXPECT_NEAR(transform.yaw_rad, 0.0, kTolerance);

  const auto point = transform_sensor_point_to_target(Point2D{2.0, -1.0}, transform);
  EXPECT_NEAR(point.x_m, 2.0, kTolerance);
  EXPECT_NEAR(point.y_m, -1.0, kTolerance);
}

TEST(ObjectListTransform, YawAndTranslationArePlacedInTheTargetFrame)
{
  // Sensor rotated a quarter turn and standing at (3, 1) in the target frame.
  const auto transform = yaw_transform(3.0, 1.0, M_PI_2);
  ASSERT_TRUE(transform.valid);
  EXPECT_NEAR(transform.yaw_rad, M_PI_2, 1e-12);

  // A point 2 m straight ahead of the sensor is 2 m along +y of the target.
  const auto point = transform_sensor_point_to_target(Point2D{2.0, 0.0}, transform);
  EXPECT_NEAR(point.x_m, 3.0, 1e-9);
  EXPECT_NEAR(point.y_m, 3.0, 1e-9);
}

TEST(ObjectListTransform, UnnormalisedQuaternionYieldsTheSameYaw)
{
  const double yaw_rad = 0.7;
  const auto normalised = yaw_transform(0.0, 0.0, yaw_rad);
  const auto scaled = make_sensor_to_target_transform(
    0.0, 0.0, 0.0, 0.0, 5.0 * std::sin(0.5 * yaw_rad), 5.0 * std::cos(0.5 * yaw_rad));

  ASSERT_TRUE(normalised.valid);
  ASSERT_TRUE(scaled.valid);
  EXPECT_NEAR(scaled.yaw_rad, normalised.yaw_rad, 1e-12);
}

TEST(ObjectListTransform, RollAndPitchOfTheTransformAreIgnored)
{
  // Quaternion of roll = pi (upside down) with no heading change. The planar
  // part of the transform must still report a zero yaw rather than a flipped
  // one, so a slightly tilted mount cannot mirror the obstacle positions.
  const auto transform = make_sensor_to_target_transform(0.0, 0.0, 1.0, 0.0, 0.0, 0.0);
  ASSERT_TRUE(transform.valid);
  EXPECT_NEAR(transform.yaw_rad, 0.0, kTolerance);
}

TEST(ObjectListTransform, NonFiniteOrDegenerateTransformIsRejected)
{
  const double nan_value = std::numeric_limits<double>::quiet_NaN();
  const double infinite_value = std::numeric_limits<double>::infinity();

  EXPECT_FALSE(make_sensor_to_target_transform(nan_value, 0.0, 0.0, 0.0, 0.0, 1.0).valid);
  EXPECT_FALSE(make_sensor_to_target_transform(0.0, infinite_value, 0.0, 0.0, 0.0, 1.0).valid);
  EXPECT_FALSE(make_sensor_to_target_transform(0.0, 0.0, 0.0, 0.0, nan_value, 1.0).valid);
  // An all-zero quaternion carries no recoverable heading.
  EXPECT_FALSE(make_sensor_to_target_transform(0.0, 0.0, 0.0, 0.0, 0.0, 0.0).valid);
}

TEST(ObjectListTransform, AppliedStagePlacesCandidatesAndDropsOffTrackOnes)
{
  const auto ring = square_ring();
  ASSERT_TRUE(ring.valid);

  // Sensor at (3, 0) looking along +x. The first candidate lands at (4, 0),
  // inside the ring; the second lands at (8, 0), well outside the outer
  // boundary and beyond the keep band.
  const auto transform = yaw_transform(3.0, 0.0, 0.0);
  const std::vector<ConeOccupancyCandidate> candidates{
    valid_candidate(1.0, 0.0), valid_candidate(5.0, 0.0)};

  const auto result = apply_object_list_transform(candidates, ring, transform);

  EXPECT_TRUE(result.applied);
  EXPECT_EQ(result.candidates_before, 2U);
  ASSERT_EQ(result.accepted.size(), 1U);
  EXPECT_EQ(result.rejected, 1U);

  const auto & accepted = result.accepted.front();
  EXPECT_TRUE(accepted.world_center_valid);
  EXPECT_NEAR(accepted.world_x_m, 4.0, 1e-9);
  EXPECT_NEAR(accepted.world_y_m, 0.0, 1e-9);
  // The circle itself is carried through untouched.
  EXPECT_NEAR(accepted.radius_m, 0.09, kTolerance);
  EXPECT_NEAR(accepted.sensor_x_m, 1.0, kTolerance);
  EXPECT_EQ(accepted.point_count, 3U);
}

TEST(ObjectListTransform, InvalidTransformFailsOpenWithoutTargetCoordinates)
{
  const auto ring = square_ring();
  const SensorToTargetTransform missing_transform;
  ASSERT_FALSE(missing_transform.valid);

  const auto result =
    apply_object_list_transform({valid_candidate(1.0, 0.0)}, ring, missing_transform);

  EXPECT_FALSE(result.applied);
  ASSERT_EQ(result.accepted.size(), 1U);
  // No target-frame coordinate was invented for the candidate, so the Object
  // List stage has nothing it could publish under the target frame.
  EXPECT_FALSE(result.accepted.front().world_center_valid);
}

TEST(ObjectListTransform, UnusableRingFailsOpenEvenWithAValidTransform)
{
  const TrackRing empty_ring;
  ASSERT_FALSE(empty_ring.valid);

  const auto result = apply_object_list_transform(
    {valid_candidate(1.0, 0.0)}, empty_ring, yaw_transform(0.0, 0.0, 0.0));

  EXPECT_FALSE(result.applied);
  EXPECT_FALSE(result.accepted.front().world_center_valid);
}

// ---------------------------------------------------------------------------
// Final output placement. The track ROI is not a rejection rule here: its
// polygons are simulator world coordinates and do not describe the Object List
// target frame, and off-track objects are not an avoidance target for now.
// ---------------------------------------------------------------------------

TEST(ObjectListPlacement, OffRingCandidateStaysInTheFinalOutput)
{
  const auto ring = square_ring();
  ASSERT_TRUE(ring.valid);

  // Sensor at (3, 0) looking along +x. The first candidate lands at (4, 0),
  // inside the ring; the second lands at (8, 0), well outside the outer
  // boundary and beyond the keep band. The ROI-gated stage drops the second
  // one; the final output stage must keep both.
  const auto transform = yaw_transform(3.0, 0.0, 0.0);
  const std::vector<ConeOccupancyCandidate> candidates{
    valid_candidate(1.0, 0.0), valid_candidate(5.0, 0.0)};

  const auto gated = apply_object_list_transform(candidates, ring, transform);
  ASSERT_EQ(gated.accepted.size(), 1U);

  const auto placed = place_candidates_in_target_frame(candidates, transform, ring);

  EXPECT_TRUE(placed.applied);
  EXPECT_EQ(placed.candidates_before, 2U);
  ASSERT_EQ(placed.accepted.size(), 2U);
  EXPECT_EQ(placed.candidates_after, 2U);
  EXPECT_EQ(placed.rejected, 0U);

  EXPECT_TRUE(placed.accepted[0].world_center_valid);
  EXPECT_NEAR(placed.accepted[0].world_x_m, 4.0, 1e-9);
  EXPECT_TRUE(placed.accepted[0].inside_track_ring);

  // The off-track candidate keeps its real target-frame centre and is still
  // published; the ring result survives only as a read-only annotation.
  EXPECT_TRUE(placed.accepted[1].world_center_valid);
  EXPECT_NEAR(placed.accepted[1].world_x_m, 8.0, 1e-9);
  EXPECT_NEAR(placed.accepted[1].world_y_m, 0.0, 1e-9);
  EXPECT_FALSE(placed.accepted[1].inside_track_ring);
}

TEST(ObjectListPlacement, AnUnusableRingNoLongerSuppressesTheOutput)
{
  const TrackRing empty_ring;
  ASSERT_FALSE(empty_ring.valid);

  const auto placed = place_candidates_in_target_frame(
    {valid_candidate(1.0, 0.0)}, yaw_transform(2.0, 0.0, 0.0), empty_ring);

  EXPECT_TRUE(placed.applied);
  ASSERT_EQ(placed.accepted.size(), 1U);
  EXPECT_TRUE(placed.accepted.front().world_center_valid);
  EXPECT_NEAR(placed.accepted.front().world_x_m, 3.0, 1e-9);
  // Nothing to annotate against, so the flag stays false without meaning
  // anything was rejected.
  EXPECT_FALSE(placed.accepted.front().inside_track_ring);
}

TEST(ObjectListPlacement, CarriesTheCircleThroughUnchanged)
{
  const auto placed = place_candidates_in_target_frame(
    {valid_candidate(1.0, 0.0)}, yaw_transform(0.0, 0.0, 0.0), square_ring());

  ASSERT_EQ(placed.accepted.size(), 1U);
  const auto & accepted = placed.accepted.front();
  EXPECT_NEAR(accepted.radius_m, 0.09, kTolerance);
  EXPECT_NEAR(accepted.sensor_x_m, 1.0, kTolerance);
  EXPECT_NEAR(accepted.sensor_y_m, 0.0, kTolerance);
  EXPECT_NEAR(accepted.source_closest_range_m, 1.0, kTolerance);
  EXPECT_EQ(accepted.point_count, 3U);
  EXPECT_EQ(accepted.first_scan_index, 10U);
  EXPECT_EQ(accepted.last_scan_index, 12U);
}

TEST(ObjectListPlacement, InvalidOccupancyCandidatesNeverEnter)
{
  ConeOccupancyCandidate rejected_upstream;
  rejected_upstream.valid = false;
  rejected_upstream.center_x_m = 1.0;

  const std::vector<ConeOccupancyCandidate> candidates{
    rejected_upstream, valid_candidate(1.0, 0.0)};
  const auto placed = place_candidates_in_target_frame(
    candidates, yaw_transform(0.0, 0.0, 0.0), square_ring());

  // The upstream cone width / occupancy gate is what rejects walls and fences,
  // and it stays the only thing that does.
  EXPECT_EQ(placed.candidates_before, 1U);
  EXPECT_EQ(placed.accepted.size(), 1U);
}

TEST(ObjectListPlacement, InvalidTransformFailsOpenWithoutTargetCoordinates)
{
  const SensorToTargetTransform missing_transform;
  const auto placed = place_candidates_in_target_frame(
    {valid_candidate(1.0, 0.0)}, missing_transform, square_ring());

  EXPECT_FALSE(placed.applied);
  ASSERT_EQ(placed.accepted.size(), 1U);
  EXPECT_FALSE(placed.accepted.front().world_center_valid);
}

TEST(ObjectListPlacementStatus, MissingTransformStillReportsTransformUnavailable)
{
  const SensorToTargetTransform missing_transform;
  const auto placed = place_candidates_in_target_frame(
    {valid_candidate(1.0, 0.0)}, missing_transform, square_ring());

  const auto status = decide_object_list_status_from_placement(
    true, missing_transform.valid, placed.applied);
  EXPECT_EQ(status, ObjectListStatus::kTransformUnavailable);
}

TEST(ObjectListPlacementStatus, AnUnusableRingNoLongerForcesAnInternalError)
{
  const TrackRing empty_ring;
  const auto placed = place_candidates_in_target_frame(
    {valid_candidate(1.0, 0.0)}, yaw_transform(0.0, 0.0, 0.0), empty_ring);

  // The ring plays no part in the published decision any more, so a frame with
  // a usable transform reports a usable observation.
  EXPECT_EQ(
    decide_object_list_status_from_placement(true, true, placed.applied),
    ObjectListStatus::kOk);
}

TEST(ObjectListPlacementStatus, BrokenScanStillOutranksTheTransform)
{
  EXPECT_EQ(
    decide_object_list_status_from_placement(false, false, false),
    ObjectListStatus::kLidarUnavailable);
}

TEST(ObjectListPlacementStatus, EveryStagePassingIsOk)
{
  EXPECT_EQ(
    decide_object_list_status_from_placement(true, true, true),
    ObjectListStatus::kOk);
}

TEST(ObjectListTransformStatus, MissingTransformIsReportedAsTransformUnavailable)
{
  EXPECT_EQ(
    decide_object_list_status_from_transform(true, false, true, false),
    ObjectListStatus::kTransformUnavailable);
  // A broken scan still outranks the transform: it is the more upstream stage.
  EXPECT_EQ(
    decide_object_list_status_from_transform(false, false, true, false),
    ObjectListStatus::kLidarUnavailable);
}

TEST(ObjectListTransformStatus, ConfigurationFaultsStayInternalErrors)
{
  EXPECT_EQ(
    decide_object_list_status_from_transform(true, true, false, false),
    ObjectListStatus::kInternalError);
  EXPECT_EQ(
    decide_object_list_status_from_transform(true, true, true, false),
    ObjectListStatus::kInternalError);
}

TEST(ObjectListTransformStatus, EveryStagePassingIsOk)
{
  EXPECT_EQ(
    decide_object_list_status_from_transform(true, true, true, true),
    ObjectListStatus::kOk);
}

/// End to end over the two stages the node runs back to back: a failed lookup
/// must reach the publisher as STATUS_TRANSFORM_UNAVAILABLE, never as an
/// "observed nothing" frame.
TEST(ObjectListTransformStatus, FailedLookupNeverReportsAnEmptyObservation)
{
  const auto ring = square_ring();
  const SensorToTargetTransform missing_transform;
  const auto result =
    apply_object_list_transform({valid_candidate(1.0, 0.0)}, ring, missing_transform);

  const auto status = decide_object_list_status_from_transform(
    true, missing_transform.valid, ring.valid, result.applied);
  EXPECT_EQ(status, ObjectListStatus::kTransformUnavailable);
  EXPECT_NE(status, ObjectListStatus::kOk);
}

}  // namespace
