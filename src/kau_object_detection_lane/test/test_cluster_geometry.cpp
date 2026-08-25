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
// Forked from package "kau_object_detection", its test test_cluster_geometry.cpp, on 2026-08-25.
// Independent copy: it links only against this package's libraries. Changes:
// namespace and include paths. The assertions themselves are unchanged, so
// this is a regression test of the forked logic against the original
// behaviour.
// ---------------------------------------------------------------------------

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

#include "gtest/gtest.h"
#include "kau_object_detection_lane/cluster_geometry.hpp"
#include "kau_object_detection_lane/laser_clusterer.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace
{

using kau_object_detection_lane::Cluster2D;
using kau_object_detection_lane::ClusterGeometry2D;
using kau_object_detection_lane::ClusterGeometryOptions;
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

Cluster2D make_cluster(
  const std::vector<ScanPoint2D> & points,
  const bool wraps_scan_boundary = false)
{
  Cluster2D cluster;
  cluster.points = points;
  cluster.wraps_scan_boundary = wraps_scan_boundary;
  return cluster;
}

/// Signed area of the footprint; positive means counter-clockwise ordering.
double signed_area(const std::vector<ScanPoint2D> & vertices)
{
  double twice_area = 0.0;
  for (std::size_t index = 0U; index < vertices.size(); ++index) {
    const auto & current = vertices[index];
    const auto & next = vertices[(index + 1U) % vertices.size()];
    twice_area += current.x_m * next.y_m - next.x_m * current.y_m;
  }
  return 0.5 * twice_area;
}

/// Checks that a point lies inside the counter-clockwise footprint, on it, or
/// at most `slack_m` metres outside it.
bool footprint_contains(
  const std::vector<ScanPoint2D> & vertices,
  const ScanPoint2D & point,
  const double slack_m = 1e-9)
{
  for (std::size_t index = 0U; index < vertices.size(); ++index) {
    const auto & current = vertices[index];
    const auto & next = vertices[(index + 1U) % vertices.size()];
    const double edge_length = std::hypot(next.x_m - current.x_m, next.y_m - current.y_m);
    if (edge_length <= 0.0) {
      continue;
    }
    // Cross product in m^2 divided by the edge length in m gives the signed
    // distance in m from the point to the edge.
    const double signed_distance =
      ((next.x_m - current.x_m) * (point.y_m - current.y_m) -
      (next.y_m - current.y_m) * (point.x_m - current.x_m)) / edge_length;
    if (signed_distance < -slack_m) {
      return false;
    }
  }
  return true;
}

void expect_geometry_invariants(const ClusterGeometry2D & geometry, const Cluster2D & cluster)
{
  ASSERT_TRUE(geometry.valid);
  EXPECT_TRUE(std::isfinite(geometry.representative_x_m));
  EXPECT_TRUE(std::isfinite(geometry.representative_y_m));
  EXPECT_TRUE(std::isfinite(geometry.representative_range_m));
  EXPECT_TRUE(std::isfinite(geometry.representative_angle_rad));
  EXPECT_TRUE(std::isfinite(geometry.width_m));
  EXPECT_TRUE(std::isfinite(geometry.max_extent_m));
  EXPECT_TRUE(std::isfinite(geometry.footprint_area_m2));
  EXPECT_TRUE(std::isfinite(geometry.closest_point.x_m));
  EXPECT_TRUE(std::isfinite(geometry.closest_point.y_m));
  EXPECT_TRUE(std::isfinite(geometry.closest_point.range_m));
  EXPECT_GE(geometry.width_m, 0.0);
  EXPECT_GE(geometry.max_extent_m, 0.0);
  EXPECT_LE(geometry.width_m, geometry.max_extent_m + 1e-9);
  EXPECT_GE(geometry.footprint_area_m2, 0.0);
  EXPECT_LE(geometry.footprint_vertices.size(), cluster.points.size());
  EXPECT_EQ(geometry.point_count, cluster.points.size());
}

TEST(ClusterGeometry, EmptyClusterIsInvalid)
{
  const auto geometry = kau_object_detection_lane::compute_cluster_geometry(make_cluster({}));

  EXPECT_FALSE(geometry.valid);
  EXPECT_EQ(geometry.point_count, 0U);
  EXPECT_TRUE(geometry.footprint_vertices.empty());
  EXPECT_DOUBLE_EQ(geometry.width_m, 0.0);
  EXPECT_DOUBLE_EQ(geometry.max_extent_m, 0.0);
  EXPECT_DOUBLE_EQ(geometry.footprint_area_m2, 0.0);
}

TEST(ClusterGeometry, SinglePointClusterHasZeroWidthAndOneVertex)
{
  const auto cluster = make_cluster({make_point(7U, 1.5, 2.0)});
  const auto geometry = kau_object_detection_lane::compute_cluster_geometry(cluster);

  expect_geometry_invariants(geometry, cluster);
  EXPECT_EQ(geometry.closest_point.scan_index, 7U);
  EXPECT_NEAR(geometry.representative_x_m, 1.5, 1e-12);
  EXPECT_NEAR(geometry.representative_y_m, 2.0, 1e-12);
  EXPECT_NEAR(geometry.representative_range_m, 2.5, 1e-12);
  EXPECT_DOUBLE_EQ(geometry.width_m, 0.0);
  EXPECT_DOUBLE_EQ(geometry.max_extent_m, 0.0);
  ASSERT_EQ(geometry.footprint_vertices.size(), 1U);
  EXPECT_EQ(geometry.footprint_vertices.front().scan_index, 7U);
  EXPECT_DOUBLE_EQ(geometry.footprint_area_m2, 0.0);
}

TEST(ClusterGeometry, TwoPointClusterKeepsBothEndpoints)
{
  const auto cluster = make_cluster({make_point(3U, 1.0, 0.0), make_point(4U, 1.0, 0.4)});
  const auto geometry = kau_object_detection_lane::compute_cluster_geometry(cluster);

  expect_geometry_invariants(geometry, cluster);
  EXPECT_NEAR(geometry.width_m, 0.4, 1e-12);
  EXPECT_NEAR(geometry.max_extent_m, 0.4, 1e-12);
  EXPECT_EQ(geometry.footprint_vertices.size(), 2U);
  EXPECT_DOUBLE_EQ(geometry.footprint_area_m2, 0.0);
}

TEST(ClusterGeometry, CollinearPointsCollapseToTwoFootprintVertices)
{
  const auto cluster = make_cluster({
      make_point(0U, 1.0, -0.2),
      make_point(1U, 1.0, 0.0),
      make_point(2U, 1.0, 0.2)});
  const auto geometry = kau_object_detection_lane::compute_cluster_geometry(cluster);

  expect_geometry_invariants(geometry, cluster);
  EXPECT_NEAR(geometry.representative_x_m, 1.0, 1e-12);
  EXPECT_NEAR(geometry.representative_y_m, 0.0, 1e-12);
  EXPECT_NEAR(geometry.width_m, 0.4, 1e-12);
  EXPECT_NEAR(geometry.max_extent_m, 0.4, 1e-12);
  EXPECT_EQ(geometry.footprint_vertices.size(), 2U);
  EXPECT_DOUBLE_EQ(geometry.footprint_area_m2, 0.0);
}

TEST(ClusterGeometry, ClosestPointUsesSmallestRangeAndKeepsScanIndex)
{
  const auto cluster = make_cluster({
      make_point(10U, 2.0, 0.0),
      make_point(11U, 1.2, 0.0),
      make_point(12U, 1.6, 0.0)});
  const auto geometry = kau_object_detection_lane::compute_cluster_geometry(cluster);

  expect_geometry_invariants(geometry, cluster);
  EXPECT_EQ(geometry.closest_point.scan_index, 11U);
  EXPECT_NEAR(geometry.closest_point.range_m, 1.2, 1e-12);
  EXPECT_NEAR(geometry.closest_point.x_m, 1.2, 1e-12);
  EXPECT_EQ(geometry.first_scan_index, 10U);
  EXPECT_EQ(geometry.last_scan_index, 12U);
}

TEST(ClusterGeometry, ClosestPointTieBreaksOnSmallestScanIndex)
{
  const auto cluster = make_cluster({
      make_point(30U, 0.0, 1.0),
      make_point(31U, 1.0, 0.0),
      make_point(32U, -1.0, 0.0)});
  const auto geometry = kau_object_detection_lane::compute_cluster_geometry(cluster);

  expect_geometry_invariants(geometry, cluster);
  EXPECT_EQ(geometry.closest_point.scan_index, 30U);
}

TEST(ClusterGeometry, SquareClusterProducesCounterClockwiseFootprint)
{
  const auto cluster = make_cluster({
      make_point(0U, 1.0, 0.0),
      make_point(1U, 1.4, 0.0),
      make_point(2U, 1.4, 0.4),
      make_point(3U, 1.0, 0.4)});
  const auto geometry = kau_object_detection_lane::compute_cluster_geometry(cluster);

  expect_geometry_invariants(geometry, cluster);
  EXPECT_EQ(geometry.footprint_vertices.size(), 4U);
  EXPECT_NEAR(geometry.footprint_area_m2, 0.16, 1e-12);
  EXPECT_GT(signed_area(geometry.footprint_vertices), 0.0);
  EXPECT_NEAR(geometry.max_extent_m, std::hypot(0.4, 0.4), 1e-12);
  EXPECT_NEAR(geometry.width_m, 0.4, 1e-12);
}

TEST(ClusterGeometry, InteriorPointIsExcludedFromFootprint)
{
  const auto cluster = make_cluster({
      make_point(0U, 1.0, 0.0),
      make_point(1U, 1.4, 0.0),
      make_point(2U, 1.2, 0.2),
      make_point(3U, 1.4, 0.4),
      make_point(4U, 1.0, 0.4)});
  const auto geometry = kau_object_detection_lane::compute_cluster_geometry(cluster);

  expect_geometry_invariants(geometry, cluster);
  EXPECT_EQ(geometry.footprint_vertices.size(), 4U);
  for (const auto & vertex : geometry.footprint_vertices) {
    EXPECT_NE(vertex.scan_index, 2U);
  }
}

TEST(ClusterGeometry, FootprintContainsEveryObservedPoint)
{
  const auto cluster = make_cluster({
      make_point(0U, 1.00, -0.30),
      make_point(1U, 1.05, -0.10),
      make_point(2U, 1.02, 0.05),
      make_point(3U, 1.10, 0.20),
      make_point(4U, 0.95, 0.30),
      make_point(5U, 0.90, 0.10)});
  const auto geometry = kau_object_detection_lane::compute_cluster_geometry(cluster);

  expect_geometry_invariants(geometry, cluster);
  for (const auto & point : cluster.points) {
    EXPECT_TRUE(footprint_contains(geometry.footprint_vertices, point));
  }
}

TEST(ClusterGeometry, CornerShapedClusterHasWidthSmallerThanMaxExtent)
{
  const auto cluster = make_cluster({
      make_point(0U, 1.0, 0.0),
      make_point(1U, 1.0, 0.5),
      make_point(2U, 1.5, 0.5),
      make_point(3U, 1.4, 0.1)});
  const auto geometry = kau_object_detection_lane::compute_cluster_geometry(cluster);

  expect_geometry_invariants(geometry, cluster);
  EXPECT_NEAR(geometry.width_m, std::hypot(0.4, 0.1), 1e-12);
  EXPECT_NEAR(geometry.max_extent_m, std::hypot(0.5, 0.5), 1e-12);
  EXPECT_LT(geometry.width_m, geometry.max_extent_m);
}

TEST(ClusterGeometry, WrappedClusterGeometryIgnoresAngleDiscontinuity)
{
  // Points straddle the +/-pi scan boundary but stay adjacent in XY.
  const auto cluster = make_cluster(
    {
      make_point(718U, -1.0, -0.02),
      make_point(719U, -1.0, -0.01),
      make_point(0U, -1.0, 0.01),
      make_point(1U, -1.0, 0.02)},
    true);
  const auto geometry = kau_object_detection_lane::compute_cluster_geometry(cluster);

  expect_geometry_invariants(geometry, cluster);
  EXPECT_TRUE(geometry.wraps_scan_boundary);
  EXPECT_EQ(geometry.first_scan_index, 718U);
  EXPECT_EQ(geometry.last_scan_index, 1U);
  EXPECT_NEAR(geometry.width_m, 0.04, 1e-12);
  EXPECT_NEAR(geometry.max_extent_m, 0.04, 1e-12);
  EXPECT_NEAR(geometry.representative_x_m, -1.0, 1e-12);
  EXPECT_NEAR(geometry.representative_y_m, 0.0, 1e-12);
}

TEST(ClusterGeometry, ShapeMeasuresAreInvariantUnderRotation)
{
  const std::vector<ScanPoint2D> points = {
    make_point(0U, 1.0, 0.0),
    make_point(1U, 1.4, 0.0),
    make_point(2U, 1.4, 0.4),
    make_point(3U, 1.0, 0.4)};
  const double angle = kPi / 5.0;
  std::vector<ScanPoint2D> rotated_points;
  rotated_points.reserve(points.size());
  for (const auto & point : points) {
    rotated_points.push_back(
      make_point(
        point.scan_index,
        point.x_m * std::cos(angle) - point.y_m * std::sin(angle),
        point.x_m * std::sin(angle) + point.y_m * std::cos(angle)));
  }

  const auto original = kau_object_detection_lane::compute_cluster_geometry(make_cluster(points));
  const auto rotated =
    kau_object_detection_lane::compute_cluster_geometry(make_cluster(rotated_points));

  EXPECT_NEAR(rotated.width_m, original.width_m, 1e-9);
  EXPECT_NEAR(rotated.max_extent_m, original.max_extent_m, 1e-9);
  EXPECT_NEAR(rotated.footprint_area_m2, original.footprint_area_m2, 1e-9);
  EXPECT_EQ(rotated.footprint_vertices.size(), original.footprint_vertices.size());
  EXPECT_NEAR(rotated.representative_range_m, original.representative_range_m, 1e-9);
}

TEST(ClusterGeometry, ShapeMeasuresAreInvariantUnderTranslation)
{
  const std::vector<ScanPoint2D> points = {
    make_point(0U, 1.0, 0.0),
    make_point(1U, 1.4, 0.0),
    make_point(2U, 1.2, 0.4)};
  const double shift_x = 2.5;
  const double shift_y = -1.25;
  std::vector<ScanPoint2D> shifted_points;
  shifted_points.reserve(points.size());
  for (const auto & point : points) {
    shifted_points.push_back(
      make_point(point.scan_index, point.x_m + shift_x, point.y_m + shift_y));
  }

  const auto original = kau_object_detection_lane::compute_cluster_geometry(make_cluster(points));
  const auto shifted =
    kau_object_detection_lane::compute_cluster_geometry(make_cluster(shifted_points));

  EXPECT_NEAR(shifted.width_m, original.width_m, 1e-9);
  EXPECT_NEAR(shifted.max_extent_m, original.max_extent_m, 1e-9);
  EXPECT_NEAR(shifted.footprint_area_m2, original.footprint_area_m2, 1e-9);
  EXPECT_NEAR(shifted.representative_x_m, original.representative_x_m + shift_x, 1e-9);
  EXPECT_NEAR(shifted.representative_y_m, original.representative_y_m + shift_y, 1e-9);
}

TEST(ClusterGeometry, NonFiniteCoordinateMakesGeometryInvalid)
{
  auto broken_point = make_point(1U, 1.0, 0.0);
  broken_point.x_m = std::numeric_limits<double>::quiet_NaN();
  const auto cluster = make_cluster({make_point(0U, 1.0, 0.0), broken_point});

  const auto geometry = kau_object_detection_lane::compute_cluster_geometry(cluster);

  EXPECT_FALSE(geometry.valid);
  EXPECT_TRUE(geometry.footprint_vertices.empty());
}

TEST(ClusterGeometry, NegativeOrNonFiniteToleranceMakesGeometryInvalid)
{
  const auto cluster = make_cluster({
      make_point(0U, 1.0, 0.0),
      make_point(1U, 1.0, 0.2)});

  ClusterGeometryOptions negative_options;
  negative_options.collinear_tolerance_m = -0.001;
  EXPECT_FALSE(
    kau_object_detection_lane::compute_cluster_geometry(cluster, negative_options).valid);

  ClusterGeometryOptions non_finite_options;
  non_finite_options.collinear_tolerance_m = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(
    kau_object_detection_lane::compute_cluster_geometry(cluster, non_finite_options).valid);
}

TEST(ClusterGeometry, NearlyCollinearPointIsDroppedWithinTolerance)
{
  const std::vector<ScanPoint2D> points = {
    make_point(0U, 1.0, 0.0),
    make_point(1U, 1.0005, 0.2),
    make_point(2U, 1.0, 0.4)};

  ClusterGeometryOptions coarse_options;
  coarse_options.collinear_tolerance_m = 0.001;
  const auto coarse =
    kau_object_detection_lane::compute_cluster_geometry(make_cluster(points), coarse_options);
  EXPECT_EQ(coarse.footprint_vertices.size(), 2U);

  ClusterGeometryOptions fine_options;
  fine_options.collinear_tolerance_m = 0.0;
  const auto fine =
    kau_object_detection_lane::compute_cluster_geometry(make_cluster(points), fine_options);
  EXPECT_EQ(fine.footprint_vertices.size(), 3U);
}

TEST(ClusterGeometry, ComputationDoesNotModifyInputCluster)
{
  const auto cluster = make_cluster({
      make_point(5U, 1.0, 0.0),
      make_point(6U, 1.2, 0.1),
      make_point(7U, 1.1, 0.3)});
  const auto cluster_copy = cluster;

  const auto geometry = kau_object_detection_lane::compute_cluster_geometry(cluster);
  ASSERT_TRUE(geometry.valid);

  ASSERT_EQ(cluster.points.size(), cluster_copy.points.size());
  for (std::size_t index = 0U; index < cluster.points.size(); ++index) {
    EXPECT_EQ(cluster.points[index].scan_index, cluster_copy.points[index].scan_index);
    EXPECT_DOUBLE_EQ(cluster.points[index].x_m, cluster_copy.points[index].x_m);
    EXPECT_DOUBLE_EQ(cluster.points[index].y_m, cluster_copy.points[index].y_m);
    EXPECT_DOUBLE_EQ(cluster.points[index].range_m, cluster_copy.points[index].range_m);
    EXPECT_DOUBLE_EQ(cluster.points[index].angle_rad, cluster_copy.points[index].angle_rad);
  }
  EXPECT_EQ(cluster.wraps_scan_boundary, cluster_copy.wraps_scan_boundary);
}

Cluster2D make_dense_arc_cluster(const double radius, const std::size_t sample_count)
{
  std::vector<ScanPoint2D> points;
  points.reserve(sample_count);
  for (std::size_t index = 0U; index < sample_count; ++index) {
    const double angle = -kPi / 4.0 + (kPi / 2.0) * static_cast<double>(index) /
      static_cast<double>(sample_count - 1U);
    points.push_back(make_point(index, radius * std::cos(angle), radius * std::sin(angle)));
  }
  return make_cluster(points);
}

TEST(ClusterGeometry, DenseArcClusterStaysConsistent)
{
  const double radius = 1.0;
  const auto cluster = make_dense_arc_cluster(radius, 360U);

  ClusterGeometryOptions exact_options;
  exact_options.collinear_tolerance_m = 0.0;
  const auto geometry =
    kau_object_detection_lane::compute_cluster_geometry(cluster, exact_options);

  expect_geometry_invariants(geometry, cluster);
  EXPECT_NEAR(geometry.width_m, 2.0 * radius * std::sin(kPi / 4.0), 1e-6);
  EXPECT_NEAR(geometry.max_extent_m, geometry.width_m, 1e-6);
  EXPECT_LT(geometry.representative_range_m, radius);
  for (const auto & point : cluster.points) {
    EXPECT_TRUE(footprint_contains(geometry.footprint_vertices, point));
  }
}

TEST(ClusterGeometry, ToleranceDoesNotSimplifyAThickCluster)
{
  const double tolerance_m = 0.001;
  const auto cluster = make_dense_arc_cluster(1.0, 360U);

  ClusterGeometryOptions exact_options;
  exact_options.collinear_tolerance_m = 0.0;
  ClusterGeometryOptions tolerant_options;
  tolerant_options.collinear_tolerance_m = tolerance_m;

  const auto exact = kau_object_detection_lane::compute_cluster_geometry(cluster, exact_options);
  const auto tolerant =
    kau_object_detection_lane::compute_cluster_geometry(cluster, tolerant_options);

  expect_geometry_invariants(tolerant, cluster);
  // The arc is far thicker than the tolerance, so the hull is never collapsed
  // and keeps containing every observed point exactly.
  EXPECT_EQ(tolerant.footprint_vertices.size(), exact.footprint_vertices.size());
  EXPECT_NEAR(tolerant.footprint_area_m2, exact.footprint_area_m2, 1e-12);
  for (const auto & point : cluster.points) {
    EXPECT_TRUE(footprint_contains(tolerant.footprint_vertices, point));
  }
}

TEST(ClusterGeometry, GloballyThinClusterCollapsesWithinTolerance)
{
  const double tolerance_m = 0.001;
  const double thickness_m = 0.0004;
  std::vector<ScanPoint2D> points;
  for (std::size_t index = 0U; index < 40U; ++index) {
    const double offset = static_cast<double>(index) * 0.01;
    const double wobble = (index % 2U == 0U) ? thickness_m : -thickness_m;
    points.push_back(make_point(index, 1.0 + wobble, offset));
  }
  const auto cluster = make_cluster(points);

  ClusterGeometryOptions tolerant_options;
  tolerant_options.collinear_tolerance_m = tolerance_m;
  const auto tolerant =
    kau_object_detection_lane::compute_cluster_geometry(cluster, tolerant_options);

  expect_geometry_invariants(tolerant, cluster);
  EXPECT_EQ(tolerant.footprint_vertices.size(), 2U);
  EXPECT_DOUBLE_EQ(tolerant.footprint_area_m2, 0.0);
  // Collapsing is bounded: no observed point is farther outside than the
  // configured tolerance.
  for (const auto & point : cluster.points) {
    EXPECT_TRUE(
      footprint_contains(tolerant.footprint_vertices, point, tolerance_m + 1e-9));
  }

  ClusterGeometryOptions exact_options;
  exact_options.collinear_tolerance_m = 0.0;
  const auto exact = kau_object_detection_lane::compute_cluster_geometry(cluster, exact_options);
  EXPECT_GT(exact.footprint_vertices.size(), 2U);
  EXPECT_GT(exact.footprint_area_m2, 0.0);
}

TEST(ClusterGeometry, MatchesClustersProducedByTheClusterer)
{
  sensor_msgs::msg::LaserScan scan;
  scan.header.frame_id = "lidar_link";
  scan.angle_min = 0.0F;
  scan.angle_increment = 0.01F;
  scan.range_min = 0.1F;
  scan.range_max = 20.0F;
  scan.ranges = {2.0F, 2.0F, 2.0F, 10.0F, 10.0F, 10.0F};
  scan.angle_max =
    scan.angle_min + scan.angle_increment * static_cast<float>(scan.ranges.size() - 1U);

  kau_object_detection_lane::ScanClusteringOptions clustering_options;
  clustering_options.base_distance_threshold_m = 0.05;
  clustering_options.angular_resolution_scale = 1.0;
  clustering_options.circular_scan_tolerance_rad = 0.05;

  const auto clusters = kau_object_detection_lane::cluster_laser_scan(
    scan, {0U, 1U, 2U, 3U, 4U, 5U}, clustering_options);
  ASSERT_EQ(clusters.size(), 2U);

  const auto geometries = kau_object_detection_lane::compute_all_cluster_geometry(clusters);
  ASSERT_EQ(geometries.size(), clusters.size());

  for (std::size_t index = 0U; index < clusters.size(); ++index) {
    expect_geometry_invariants(geometries[index], clusters[index]);

    double smallest_range = clusters[index].points.front().range_m;
    for (const auto & point : clusters[index].points) {
      smallest_range = std::min(smallest_range, point.range_m);
    }
    EXPECT_NEAR(geometries[index].closest_point.range_m, smallest_range, 1e-12);
    EXPECT_EQ(geometries[index].first_scan_index, clusters[index].points.front().scan_index);
    EXPECT_EQ(geometries[index].last_scan_index, clusters[index].points.back().scan_index);
  }

  EXPECT_NEAR(geometries[0].closest_point.range_m, 2.0, 1e-9);
  EXPECT_NEAR(geometries[1].closest_point.range_m, 10.0, 1e-9);
}

TEST(ClusterGeometry, EmptyFootprintInputReturnsNoVertices)
{
  EXPECT_TRUE(kau_object_detection_lane::compute_observed_footprint({}, 0.001).empty());
  EXPECT_TRUE(
    kau_object_detection_lane::compute_observed_footprint(
      {make_point(0U, 1.0, 0.0)}, -1.0).empty());
}

}  // namespace
