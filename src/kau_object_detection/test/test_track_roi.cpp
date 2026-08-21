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
#include <cstddef>
#include <limits>
#include <vector>

#include "gtest/gtest.h"
#include "kau_object_detection/cone_occupancy.hpp"
#include "kau_object_detection/track_geometry.hpp"
#include "kau_object_detection/track_roi.hpp"
#include "kau_object_detection/world_pose.hpp"

namespace
{

using kau_object_detection::BasePose2D;
using kau_object_detection::ConeOccupancyCandidate;
using kau_object_detection::Point2D;
using kau_object_detection::SensorMountOptions;
using kau_object_detection::TrackRoiOptions;
using kau_object_detection::WorldPoseCandidate;

constexpr double kPi = 3.14159265358979323846;

/// Mount measured on the vehicle: base_footprint -> lidar_link translation is
/// [-0.027, 0, 0.200] with identity rotation. Only the planar part is used.
SensorMountOptions vehicle_mount()
{
  SensorMountOptions mount;
  mount.lidar_offset_x_m = -0.027;
  mount.lidar_offset_y_m = 0.0;
  return mount;
}

/// Synthetic occupancy candidate. No cone position from the map is used; the
/// sensor-frame centre is chosen by the test itself.
ConeOccupancyCandidate make_candidate(
  const double sensor_x_m,
  const double sensor_y_m,
  const std::size_t point_count = 3U,
  const bool valid = true)
{
  ConeOccupancyCandidate candidate;
  candidate.valid = valid;
  candidate.center_x_m = sensor_x_m;
  candidate.center_y_m = sensor_y_m;
  candidate.radius_m = 0.09;
  candidate.center_range_m = std::hypot(sensor_x_m, sensor_y_m);
  candidate.source_closest_range_m = candidate.center_range_m - 0.043;
  candidate.point_count = point_count;
  candidate.first_scan_index = 100U;
  candidate.last_scan_index = 100U + point_count - 1U;
  return candidate;
}

std::vector<Point2D> make_square(const double half_size_m)
{
  return {
    Point2D{-half_size_m, -half_size_m},
    Point2D{half_size_m, -half_size_m},
    Point2D{half_size_m, half_size_m},
    Point2D{-half_size_m, half_size_m},
  };
}

kau_object_detection::TrackRing test_ring()
{
  return kau_object_detection::make_track_ring(make_square(4.0), make_square(1.0));
}

TrackRoiOptions strict_options()
{
  TrackRoiOptions options;
  options.boundary_tolerance_m = 0.0;
  return options;
}

TEST(TrackRoi, IdentityPoseAppliesOnlyTheMountOffset)
{
  const BasePose2D base_pose{0.0, 0.0, 0.0};

  const auto world = kau_object_detection::transform_sensor_point_to_world(
    Point2D{2.0, 1.0}, base_pose, vehicle_mount());

  EXPECT_NEAR(world.x_m, 2.0 - 0.027, 1e-12);
  EXPECT_NEAR(world.y_m, 1.0, 1e-12);
}

TEST(TrackRoi, TranslationIsAddedAfterRotation)
{
  const BasePose2D base_pose{5.0, -3.0, 0.0};

  const auto world = kau_object_detection::transform_sensor_point_to_world(
    Point2D{1.0, 0.5}, base_pose, vehicle_mount());

  EXPECT_NEAR(world.x_m, 5.0 + 1.0 - 0.027, 1e-12);
  EXPECT_NEAR(world.y_m, -3.0 + 0.5, 1e-12);
}

TEST(TrackRoi, QuarterTurnRotatesTheSensorPoint)
{
  const BasePose2D base_pose{0.0, 0.0, kPi / 2.0};
  SensorMountOptions no_offset;
  no_offset.lidar_offset_x_m = 0.0;
  no_offset.lidar_offset_y_m = 0.0;

  const auto world = kau_object_detection::transform_sensor_point_to_world(
    Point2D{2.0, 0.0}, base_pose, no_offset);

  EXPECT_NEAR(world.x_m, 0.0, 1e-9);
  EXPECT_NEAR(world.y_m, 2.0, 1e-9);
}

TEST(TrackRoi, TransformMatchesTheClosedFormForAnArbitraryPose)
{
  const BasePose2D base_pose{1.4, 3.395, -1.5707};
  const auto mount = vehicle_mount();
  const Point2D sensor_point{1.25, -0.4};

  const auto world = kau_object_detection::transform_sensor_point_to_world(
    sensor_point, base_pose, mount);

  const double base_x = mount.lidar_offset_x_m + sensor_point.x_m;
  const double base_y = mount.lidar_offset_y_m + sensor_point.y_m;
  EXPECT_NEAR(
    world.x_m,
    base_pose.x_m + std::cos(base_pose.yaw_rad) * base_x -
    std::sin(base_pose.yaw_rad) * base_y,
    1e-12);
  EXPECT_NEAR(
    world.y_m,
    base_pose.y_m + std::sin(base_pose.yaw_rad) * base_x +
    std::cos(base_pose.yaw_rad) * base_y,
    1e-12);
}

TEST(TrackRoi, StaleWorldPoseFailsOpenAndKeepsEveryCandidate)
{
  const std::vector<ConeOccupancyCandidate> candidates{
    make_candidate(2.5, 0.0),
    make_candidate(0.0, 90.0),
  };

  const auto result = kau_object_detection::apply_track_roi(
    candidates, test_ring(), false, BasePose2D{0.0, 0.0, 0.0}, vehicle_mount(),
    strict_options());

  EXPECT_FALSE(result.applied);
  EXPECT_EQ(result.candidates_before, 2U);
  EXPECT_EQ(result.candidates_after, 2U);
  EXPECT_EQ(result.rejected, 0U);
  ASSERT_EQ(result.accepted.size(), 2U);
  for (const auto & accepted : result.accepted) {
    EXPECT_FALSE(accepted.world_center_valid);
    EXPECT_FALSE(accepted.inside_track_ring);
  }
  // The sensor-frame circle is carried through untouched.
  EXPECT_NEAR(result.accepted[0].sensor_x_m, 2.5, 1e-12);
  EXPECT_NEAR(result.accepted[0].radius_m, 0.09, 1e-12);
  EXPECT_EQ(result.accepted[0].point_count, 3U);
}

TEST(TrackRoi, MissingRingFailsOpenEvenWithAFreshPose)
{
  const std::vector<ConeOccupancyCandidate> candidates{make_candidate(0.0, 90.0)};
  const kau_object_detection::TrackRing empty_ring;

  const auto result = kau_object_detection::apply_track_roi(
    candidates, empty_ring, true, BasePose2D{0.0, 0.0, 0.0}, vehicle_mount());

  EXPECT_FALSE(result.applied);
  EXPECT_EQ(result.candidates_after, 1U);
  EXPECT_EQ(result.rejected, 0U);
}

TEST(TrackRoi, NonFiniteBasePoseFailsOpen)
{
  const std::vector<ConeOccupancyCandidate> candidates{make_candidate(0.0, 90.0)};
  const BasePose2D broken{std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0};

  const auto result = kau_object_detection::apply_track_roi(
    candidates, test_ring(), true, broken, vehicle_mount());

  EXPECT_FALSE(result.applied);
  EXPECT_EQ(result.candidates_after, 1U);
}

TEST(TrackRoi, FreshPoseDropsOnlyCentresOutsideTheRing)
{
  // Vehicle sits inside the ring at (2.5, 0) facing +x, so a candidate 0.5 m
  // ahead stays on the ring and one 90 m ahead is far outside the map.
  const BasePose2D base_pose{2.5, 0.0, 0.0};
  const std::vector<ConeOccupancyCandidate> candidates{
    make_candidate(0.5, 0.0, 2U),
    make_candidate(90.0, 0.0, 4U),
  };

  const auto result = kau_object_detection::apply_track_roi(
    candidates, test_ring(), true, base_pose, vehicle_mount(), strict_options());

  EXPECT_TRUE(result.applied);
  EXPECT_EQ(result.candidates_before, 2U);
  EXPECT_EQ(result.candidates_after, 1U);
  EXPECT_EQ(result.rejected, 1U);
  ASSERT_EQ(result.accepted.size(), 1U);
  EXPECT_TRUE(result.accepted[0].world_center_valid);
  EXPECT_TRUE(result.accepted[0].inside_track_ring);
  EXPECT_NEAR(result.accepted[0].world_x_m, 2.5 + 0.5 - 0.027, 1e-12);
  EXPECT_NEAR(result.accepted[0].world_y_m, 0.0, 1e-12);
  // A two-point candidate still survives the ROI stage.
  EXPECT_EQ(result.accepted[0].point_count, 2U);
}

TEST(TrackRoi, CentreInsideTheHoleIsRejected)
{
  // Facing -x from (2.5, 0), a candidate 2.5 m ahead lands in the middle of the
  // inner hole, which is not drivable.
  const BasePose2D base_pose{2.5, 0.0, kPi};
  const std::vector<ConeOccupancyCandidate> candidates{make_candidate(2.5, 0.0)};

  const auto result = kau_object_detection::apply_track_roi(
    candidates, test_ring(), true, base_pose, vehicle_mount(), strict_options());

  EXPECT_TRUE(result.applied);
  EXPECT_EQ(result.candidates_after, 0U);
  EXPECT_EQ(result.rejected, 1U);
  EXPECT_TRUE(result.accepted.empty());
}

TEST(TrackRoi, BoundaryToleranceKeepsAnEdgeCandidate)
{
  // Candidate centre lands 0.1 m outside the outer boundary.
  const BasePose2D base_pose{3.0, 0.0, 0.0};
  const std::vector<ConeOccupancyCandidate> candidates{make_candidate(1.127, 0.0)};

  const auto strict = kau_object_detection::apply_track_roi(
    candidates, test_ring(), true, base_pose, vehicle_mount(), strict_options());
  EXPECT_EQ(strict.candidates_after, 0U);
  EXPECT_EQ(strict.rejected, 1U);

  TrackRoiOptions tolerant;
  tolerant.boundary_tolerance_m = 0.15;
  const auto kept = kau_object_detection::apply_track_roi(
    candidates, test_ring(), true, base_pose, vehicle_mount(), tolerant);
  EXPECT_EQ(kept.candidates_after, 1U);
  EXPECT_EQ(kept.rejected, 0U);
  EXPECT_NEAR(kept.accepted[0].world_x_m, 4.1, 1e-9);
}

TEST(TrackRoi, InvalidOccupancyCandidatesNeverEnterTheStage)
{
  const std::vector<ConeOccupancyCandidate> candidates{
    make_candidate(0.5, 0.0, 3U, false),
    make_candidate(0.5, 0.0, 3U, true),
  };

  const auto result = kau_object_detection::apply_track_roi(
    candidates, test_ring(), true, BasePose2D{2.5, 0.0, 0.0}, vehicle_mount(),
    strict_options());

  EXPECT_EQ(result.candidates_before, 1U);
  EXPECT_EQ(result.candidates_after, 1U);
}

TEST(TrackRoi, EmptyInputProducesAnEmptyResult)
{
  const auto result = kau_object_detection::apply_track_roi(
    {}, test_ring(), true, BasePose2D{0.0, 0.0, 0.0}, vehicle_mount());

  EXPECT_TRUE(result.applied);
  EXPECT_EQ(result.candidates_before, 0U);
  EXPECT_EQ(result.candidates_after, 0U);
  EXPECT_EQ(result.rejected, 0U);
}

TEST(TrackRoi, EndToEndUsesTheWorldPoseFreshnessDecision)
{
  // A synthetic pose message drives the same freshness helper the node uses.
  WorldPoseCandidate pose_candidate;
  pose_candidate.child_frame_id = "base_footprint";
  pose_candidate.x_m = 2.5;
  pose_candidate.y_m = 0.0;
  pose_candidate.orientation_w = 1.0;
  pose_candidate.orientation_z = 0.0;
  pose_candidate.stamp_s = 3579.695;

  const auto sample = kau_object_detection::select_world_base_pose(
    {pose_candidate}, "base_footprint", 100.0);
  ASSERT_TRUE(sample.valid);

  const std::vector<ConeOccupancyCandidate> candidates{
    make_candidate(0.5, 0.0),
    make_candidate(90.0, 0.0),
  };
  const BasePose2D base_pose{sample.x_m, sample.y_m, sample.yaw_rad};

  const bool fresh_now =
    kau_object_detection::world_pose_is_fresh(sample, 100.1, 0.5);
  ASSERT_TRUE(fresh_now);
  const auto applied = kau_object_detection::apply_track_roi(
    candidates, test_ring(), fresh_now, base_pose, vehicle_mount(), strict_options());
  EXPECT_TRUE(applied.applied);
  EXPECT_EQ(applied.candidates_after, 1U);
  EXPECT_EQ(applied.rejected, 1U);

  const bool fresh_later =
    kau_object_detection::world_pose_is_fresh(sample, 101.0, 0.5);
  ASSERT_FALSE(fresh_later);
  const auto failed_open = kau_object_detection::apply_track_roi(
    candidates, test_ring(), fresh_later, base_pose, vehicle_mount(), strict_options());
  EXPECT_FALSE(failed_open.applied);
  EXPECT_EQ(failed_open.candidates_after, 2U);
  EXPECT_EQ(failed_open.rejected, 0U);
}

}  // namespace
