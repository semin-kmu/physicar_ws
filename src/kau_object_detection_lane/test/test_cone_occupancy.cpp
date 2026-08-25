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
// Forked from package "kau_object_detection", its test test_cone_occupancy.cpp, on 2026-08-25.
// Independent copy: it links only against this package's libraries. Changes:
// namespace and include paths. The assertions themselves are unchanged, so
// this is a regression test of the forked logic against the original
// behaviour.
// ---------------------------------------------------------------------------

#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

#include "gtest/gtest.h"
#include "kau_object_detection_lane/cluster_geometry.hpp"
#include "kau_object_detection_lane/cone_occupancy.hpp"
#include "kau_object_detection_lane/laser_clusterer.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace
{

using kau_object_detection_lane::Cluster2D;
using kau_object_detection_lane::ClusterGeometry2D;
using kau_object_detection_lane::ConeOccupancyCandidate;
using kau_object_detection_lane::ConeOccupancyOptions;
using kau_object_detection_lane::ConeOccupancyRejection;
using kau_object_detection_lane::ScanPoint2D;

constexpr double kPi = 3.14159265358979323846;

ScanPoint2D make_point(const std::size_t scan_index, const double x_m, const double y_m)
{
  ScanPoint2D point;
  point.scan_index = scan_index;
  point.x_m = x_m;
  point.y_m = y_m;
  point.range_m = std::hypot(x_m, y_m);
  point.angle_rad = std::atan2(y_m, x_m);
  return point;
}

ClusterGeometry2D geometry_of(
  const std::vector<ScanPoint2D> & points,
  const bool wraps_scan_boundary = false)
{
  Cluster2D cluster;
  cluster.points = points;
  cluster.wraps_scan_boundary = wraps_scan_boundary;
  return kau_object_detection_lane::compute_cluster_geometry(cluster);
}

/// Points on the near side of a cone slice of `slice_radius_m` whose axis sits
/// at `axis_range_m` straight ahead. No map position is assumed; only the cone
/// dimensions given by the vehicle team are used.
std::vector<ScanPoint2D> make_cone_slice_points(
  const double axis_range_m,
  const double slice_radius_m,
  const std::size_t point_count,
  const std::size_t first_scan_index = 100U)
{
  std::vector<ScanPoint2D> points;
  points.reserve(point_count);
  for (std::size_t index = 0U; index < point_count; ++index) {
    const double fraction = point_count == 1U ?
      0.0 :
      (static_cast<double>(index) / static_cast<double>(point_count - 1U)) - 0.5;
    // Sweep the visible half of the slice, so the middle sample is the closest.
    const double surface_angle = fraction * kPi * 0.6;
    const double x_m = axis_range_m - slice_radius_m * std::cos(surface_angle);
    const double y_m = slice_radius_m * std::sin(surface_angle);
    points.push_back(make_point(first_scan_index + index, x_m, y_m));
  }
  return points;
}

TEST(ConeOccupancy, ProvisionalCenterIsPushedAwayFromTheSensor)
{
  const ConeOccupancyOptions options;
  const auto geometry = geometry_of(
    make_cone_slice_points(3.0, options.visible_slice_radius_m, 3U));
  ASSERT_TRUE(geometry.valid);

  const auto candidate =
    kau_object_detection_lane::compute_cone_occupancy_candidate(geometry, options);

  ASSERT_TRUE(candidate.valid);
  EXPECT_EQ(candidate.rejection, ConeOccupancyRejection::kNone);
  EXPECT_NEAR(candidate.radius_m, options.nominal_cone_radius_m, 1e-12);
  EXPECT_NEAR(candidate.source_closest_range_m, geometry.closest_point.range_m, 1e-12);
  EXPECT_EQ(candidate.point_count, geometry.point_count);

  // The closest sample lies one slice radius in front of the axis, so the
  // correction recovers the axis range.
  EXPECT_NEAR(candidate.center_range_m, 3.0, 1e-9);
  EXPECT_NEAR(candidate.center_x_m, 3.0, 1e-9);
  EXPECT_NEAR(candidate.center_y_m, 0.0, 1e-9);
}

TEST(ConeOccupancy, CenterCorrectionFollowsTheClosestPointBearing)
{
  ConeOccupancyOptions options;
  options.visible_slice_radius_m = 0.043;

  const double bearing_rad = 2.0;
  const double closest_range_m = 5.0;
  const auto geometry = geometry_of({
      make_point(
        10U, closest_range_m * std::cos(bearing_rad),
        closest_range_m * std::sin(bearing_rad)),
    });
  ASSERT_TRUE(geometry.valid);

  const auto candidate =
    kau_object_detection_lane::compute_cone_occupancy_candidate(geometry, options);

  ASSERT_TRUE(candidate.valid);
  const double expected_range_m = closest_range_m + options.visible_slice_radius_m;
  EXPECT_NEAR(candidate.center_range_m, expected_range_m, 1e-9);
  EXPECT_NEAR(candidate.center_angle_rad, bearing_rad, 1e-9);
  EXPECT_NEAR(candidate.center_x_m, expected_range_m * std::cos(bearing_rad), 1e-9);
  EXPECT_NEAR(candidate.center_y_m, expected_range_m * std::sin(bearing_rad), 1e-9);
  // The centre is always farther from the sensor than the measured surface.
  EXPECT_GT(candidate.center_range_m, candidate.source_closest_range_m);
}

TEST(ConeOccupancy, TwoPointCandidateProducesAValidCircle)
{
  const ConeOccupancyOptions options;
  const auto geometry = geometry_of(
    make_cone_slice_points(8.0, options.visible_slice_radius_m, 2U));
  ASSERT_TRUE(geometry.valid);
  ASSERT_EQ(geometry.point_count, 2U);

  const auto candidate =
    kau_object_detection_lane::compute_cone_occupancy_candidate(geometry, options);

  ASSERT_TRUE(candidate.valid);
  EXPECT_EQ(candidate.point_count, 2U);
  EXPECT_NEAR(candidate.radius_m, 0.09, 1e-12);
  EXPECT_GT(candidate.center_range_m, candidate.source_closest_range_m);
  EXPECT_LE(candidate.observed_max_extent_m, 2.0 * options.visible_slice_radius_m + 1e-9);
}

TEST(ConeOccupancy, FourPointCandidateProducesAValidCircle)
{
  const ConeOccupancyOptions options;
  for (std::size_t point_count = 1U; point_count <= 4U; ++point_count) {
    const auto geometry = geometry_of(
      make_cone_slice_points(4.0, options.visible_slice_radius_m, point_count));
    ASSERT_TRUE(geometry.valid) << "point_count=" << point_count;

    const auto candidate =
      kau_object_detection_lane::compute_cone_occupancy_candidate(geometry, options);
    EXPECT_TRUE(candidate.valid) << "point_count=" << point_count;
    EXPECT_EQ(candidate.point_count, point_count);
    EXPECT_NEAR(candidate.radius_m, options.nominal_cone_radius_m, 1e-12);
  }
}

TEST(ConeOccupancy, RadiusComesFromTheConfiguredNominalRadius)
{
  ConeOccupancyOptions options;
  options.nominal_cone_radius_m = 0.12;
  const auto geometry = geometry_of({make_point(0U, 2.0, 0.0)});

  const auto candidate =
    kau_object_detection_lane::compute_cone_occupancy_candidate(geometry, options);

  ASSERT_TRUE(candidate.valid);
  EXPECT_NEAR(candidate.radius_m, 0.12, 1e-12);
}

TEST(ConeOccupancy, WideWallClusterIsRejectedByTheWidthGate)
{
  const ConeOccupancyOptions options;
  std::vector<ScanPoint2D> wall_points;
  for (std::size_t index = 0U; index < 60U; ++index) {
    const double y_m = -2.5 + 0.1 * static_cast<double>(index);
    wall_points.push_back(make_point(index, 4.0, y_m));
  }
  const auto geometry = geometry_of(wall_points);
  ASSERT_TRUE(geometry.valid);
  ASSERT_GT(
    kau_object_detection_lane::observed_extent_m(geometry),
    options.maximum_observed_width_m);

  const auto candidate =
    kau_object_detection_lane::compute_cone_occupancy_candidate(geometry, options);

  EXPECT_FALSE(candidate.valid);
  EXPECT_EQ(candidate.rejection, ConeOccupancyRejection::kObservedWidthAboveGate);
  EXPECT_NEAR(candidate.radius_m, 0.0, 1e-12);
}

TEST(ConeOccupancy, WidthGateUsesTheLargerOfWidthAndMaxExtent)
{
  ConeOccupancyOptions options;
  options.maximum_observed_width_m = 0.30;

  // A corner-shaped cluster whose first-to-last chord is small but whose hull
  // extent is large must still be rejected.
  const auto geometry = geometry_of({
      make_point(0U, 3.0, 0.0),
      make_point(1U, 3.0, 0.5),
      make_point(2U, 3.1, 0.02),
    });
  ASSERT_TRUE(geometry.valid);
  ASSERT_LT(geometry.width_m, options.maximum_observed_width_m);
  ASSERT_GT(geometry.max_extent_m, options.maximum_observed_width_m);

  const auto candidate =
    kau_object_detection_lane::compute_cone_occupancy_candidate(geometry, options);

  EXPECT_FALSE(candidate.valid);
  EXPECT_EQ(candidate.rejection, ConeOccupancyRejection::kObservedWidthAboveGate);
}

TEST(ConeOccupancy, ClusterAtTheGateBoundaryIsAccepted)
{
  ConeOccupancyOptions options;
  options.maximum_observed_width_m = 0.30;
  const auto geometry = geometry_of({
      make_point(0U, 3.0, -0.15),
      make_point(1U, 3.0, 0.15),
    });
  ASSERT_TRUE(geometry.valid);
  ASSERT_NEAR(kau_object_detection_lane::observed_extent_m(geometry), 0.30, 1e-12);

  EXPECT_TRUE(
    kau_object_detection_lane::compute_cone_occupancy_candidate(geometry, options).valid);
}

TEST(ConeOccupancy, InvalidGeometryIsRejected)
{
  const ClusterGeometry2D empty_geometry;
  ASSERT_FALSE(empty_geometry.valid);

  const auto candidate =
    kau_object_detection_lane::compute_cone_occupancy_candidate(empty_geometry);

  EXPECT_FALSE(candidate.valid);
  EXPECT_EQ(candidate.rejection, ConeOccupancyRejection::kInvalidGeometry);
}

TEST(ConeOccupancy, GeometryWithoutPointsIsRejected)
{
  ClusterGeometry2D geometry;
  geometry.valid = true;
  geometry.point_count = 0U;

  const auto candidate =
    kau_object_detection_lane::compute_cone_occupancy_candidate(geometry);

  EXPECT_FALSE(candidate.valid);
  EXPECT_EQ(candidate.rejection, ConeOccupancyRejection::kInvalidGeometry);
}

TEST(ConeOccupancy, ClosestPointAtTheSensorOriginIsRejected)
{
  ClusterGeometry2D geometry;
  geometry.valid = true;
  geometry.point_count = 1U;
  geometry.closest_point = make_point(0U, 0.0, 0.0);

  const auto candidate =
    kau_object_detection_lane::compute_cone_occupancy_candidate(geometry);

  EXPECT_FALSE(candidate.valid);
  EXPECT_EQ(candidate.rejection, ConeOccupancyRejection::kDegenerateClosestPoint);
}

TEST(ConeOccupancy, NonFiniteClosestPointIsRejected)
{
  const double nan_value = std::numeric_limits<double>::quiet_NaN();
  ClusterGeometry2D geometry;
  geometry.valid = true;
  geometry.point_count = 1U;
  geometry.closest_point = make_point(0U, 1.0, 0.0);
  geometry.closest_point.x_m = nan_value;

  const auto candidate =
    kau_object_detection_lane::compute_cone_occupancy_candidate(geometry);

  EXPECT_FALSE(candidate.valid);
  EXPECT_EQ(candidate.rejection, ConeOccupancyRejection::kDegenerateClosestPoint);
}

TEST(ConeOccupancy, NonFiniteObservedShapeIsRejected)
{
  ClusterGeometry2D geometry;
  geometry.valid = true;
  geometry.point_count = 2U;
  geometry.closest_point = make_point(0U, 2.0, 0.0);
  geometry.width_m = std::numeric_limits<double>::infinity();

  const auto candidate =
    kau_object_detection_lane::compute_cone_occupancy_candidate(geometry);

  EXPECT_FALSE(candidate.valid);
  EXPECT_EQ(candidate.rejection, ConeOccupancyRejection::kInvalidGeometry);
}

TEST(ConeOccupancy, UnusableOptionsAreRejected)
{
  const auto geometry = geometry_of({make_point(0U, 2.0, 0.0)});
  ASSERT_TRUE(geometry.valid);

  ConeOccupancyOptions zero_radius;
  zero_radius.nominal_cone_radius_m = 0.0;
  EXPECT_EQ(
    kau_object_detection_lane::compute_cone_occupancy_candidate(geometry, zero_radius).rejection,
    ConeOccupancyRejection::kInvalidOptions);

  ConeOccupancyOptions negative_slice;
  negative_slice.visible_slice_radius_m = -0.01;
  EXPECT_EQ(
    kau_object_detection_lane::compute_cone_occupancy_candidate(
      geometry, negative_slice).rejection,
    ConeOccupancyRejection::kInvalidOptions);

  ConeOccupancyOptions zero_gate;
  zero_gate.maximum_observed_width_m = 0.0;
  EXPECT_EQ(
    kau_object_detection_lane::compute_cone_occupancy_candidate(geometry, zero_gate).rejection,
    ConeOccupancyRejection::kInvalidOptions);

  ConeOccupancyOptions non_finite_gate;
  non_finite_gate.maximum_observed_width_m = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(
    kau_object_detection_lane::compute_cone_occupancy_candidate(
      geometry, non_finite_gate).rejection,
    ConeOccupancyRejection::kInvalidOptions);
}

TEST(ConeOccupancy, ZeroVisibleSliceRadiusKeepsTheMeasuredSurfaceAsCenter)
{
  ConeOccupancyOptions options;
  options.visible_slice_radius_m = 0.0;
  const auto geometry = geometry_of({make_point(0U, 2.0, 1.0)});

  const auto candidate =
    kau_object_detection_lane::compute_cone_occupancy_candidate(geometry, options);

  ASSERT_TRUE(candidate.valid);
  EXPECT_NEAR(candidate.center_x_m, 2.0, 1e-12);
  EXPECT_NEAR(candidate.center_y_m, 1.0, 1e-12);
}

TEST(ConeOccupancy, ProvenanceIsCopiedFromTheSourceGeometry)
{
  const ConeOccupancyOptions options;
  const auto geometry = geometry_of(
    make_cone_slice_points(6.0, options.visible_slice_radius_m, 3U, 718U), true);
  ASSERT_TRUE(geometry.valid);

  const auto candidate =
    kau_object_detection_lane::compute_cone_occupancy_candidate(geometry, options);

  ASSERT_TRUE(candidate.valid);
  EXPECT_EQ(candidate.first_scan_index, geometry.first_scan_index);
  EXPECT_EQ(candidate.last_scan_index, geometry.last_scan_index);
  EXPECT_TRUE(candidate.wraps_scan_boundary);
  EXPECT_NEAR(candidate.observed_width_m, geometry.width_m, 1e-12);
  EXPECT_NEAR(candidate.observed_max_extent_m, geometry.max_extent_m, 1e-12);
  EXPECT_NEAR(candidate.source_closest_x_m, geometry.closest_point.x_m, 1e-12);
  EXPECT_NEAR(candidate.source_closest_y_m, geometry.closest_point.y_m, 1e-12);
}

TEST(ConeOccupancy, BatchApiKeepsOneEntryPerGeometry)
{
  const ConeOccupancyOptions options;
  std::vector<ScanPoint2D> wall_points;
  for (std::size_t index = 0U; index < 40U; ++index) {
    wall_points.push_back(make_point(index, 4.0, -2.0 + 0.1 * static_cast<double>(index)));
  }

  const std::vector<ClusterGeometry2D> geometries{
    geometry_of(make_cone_slice_points(3.0, options.visible_slice_radius_m, 2U)),
    ClusterGeometry2D{},
    geometry_of(wall_points),
  };

  const auto candidates =
    kau_object_detection_lane::compute_all_cone_occupancy_candidates(geometries, options);

  ASSERT_EQ(candidates.size(), geometries.size());
  EXPECT_TRUE(candidates[0].valid);
  EXPECT_FALSE(candidates[1].valid);
  EXPECT_EQ(candidates[1].rejection, ConeOccupancyRejection::kInvalidGeometry);
  EXPECT_FALSE(candidates[2].valid);
  EXPECT_EQ(candidates[2].rejection, ConeOccupancyRejection::kObservedWidthAboveGate);
}

TEST(ConeOccupancy, EndToEndTwoBeamConeFromTheClustererYieldsACandidate)
{
  constexpr std::size_t kBeamCount = 720U;
  constexpr float kPiF = 3.14159265358979323846F;

  sensor_msgs::msg::LaserScan scan;
  scan.header.frame_id = "lidar_link";
  scan.angle_min = -kPiF;
  scan.angle_increment = 2.0F * kPiF / static_cast<float>(kBeamCount);
  scan.angle_max =
    scan.angle_min + scan.angle_increment * static_cast<float>(kBeamCount - 1U);
  scan.range_min = 0.1F;
  scan.range_max = 20.0F;
  scan.ranges.assign(kBeamCount, std::numeric_limits<float>::quiet_NaN());
  scan.ranges[400U] = 9.0F;
  scan.ranges[401U] = 9.0F;

  const auto clusters = kau_object_detection_lane::cluster_laser_scan(scan, {400U, 401U});
  ASSERT_EQ(clusters.size(), 1U);
  const auto candidates_clusters =
    kau_object_detection_lane::select_candidate_clusters(clusters);
  ASSERT_EQ(candidates_clusters.size(), 1U);

  const auto geometries =
    kau_object_detection_lane::compute_all_cluster_geometry(candidates_clusters);
  const auto circles =
    kau_object_detection_lane::compute_all_cone_occupancy_candidates(geometries);

  ASSERT_EQ(circles.size(), 1U);
  ASSERT_TRUE(circles[0].valid);
  EXPECT_EQ(circles[0].point_count, 2U);
  EXPECT_NEAR(circles[0].radius_m, 0.09, 1e-12);
  EXPECT_NEAR(circles[0].source_closest_range_m, 9.0, 1e-6);
  EXPECT_NEAR(circles[0].center_range_m, 9.0 + 0.043, 1e-6);
}

}  // namespace
