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
// Forked from package "kau_object_detection", its test test_laser_scan_clusterer.cpp,
// on 2026-08-25.
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
#include "kau_object_detection_lane/laser_clusterer.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace
{

constexpr float kPi = 3.14159265358979323846F;

/// Beam count of the PhysiCar scan observed on the vehicle. Only the sampling
/// layout is reproduced here; no map-dependent distance is assumed.
constexpr std::size_t kFullCircleBeamCount = 720U;

sensor_msgs::msg::LaserScan make_scan(
  const std::vector<float> & ranges,
  const float angle_min,
  const float angle_increment)
{
  sensor_msgs::msg::LaserScan scan;
  scan.header.frame_id = "lidar_link";
  scan.angle_min = angle_min;
  scan.angle_increment = angle_increment;
  scan.angle_max =
    angle_min + angle_increment * static_cast<float>(ranges.size() - 1U);
  scan.range_min = 0.1F;
  scan.range_max = 20.0F;
  scan.ranges = ranges;
  return scan;
}

/// Closed 360-degree sweep, so that the last beam is followed by beam zero.
sensor_msgs::msg::LaserScan make_full_circle_scan(const std::vector<float> & ranges)
{
  const auto beam_count = static_cast<float>(ranges.size());
  return make_scan(ranges, -kPi, 2.0F * kPi / beam_count);
}

std::vector<float> make_unmeasured_ranges(const std::size_t beam_count)
{
  return std::vector<float>(beam_count, std::numeric_limits<float>::quiet_NaN());
}

kau_object_detection_lane::ScanClusteringOptions test_options()
{
  kau_object_detection_lane::ScanClusteringOptions options;
  options.base_distance_threshold_m = 0.05;
  options.angular_resolution_scale = 1.0;
  options.circular_scan_tolerance_rad = 0.05;
  return options;
}

kau_object_detection_lane::ClusterCandidateOptions candidate_options(
  const std::size_t minimum_cluster_points)
{
  kau_object_detection_lane::ClusterCandidateOptions options;
  options.minimum_cluster_points = minimum_cluster_points;
  return options;
}

/// Used by tests that exercise clustering or boundary merging in isolation.
kau_object_detection_lane::NeighborFilterOptions disabled_filter()
{
  kau_object_detection_lane::NeighborFilterOptions options;
  options.enabled = false;
  return options;
}

TEST(LaserScanClusterer, ConvertsPolarScanValuesAndPreservesIndices)
{
  const auto scan = make_scan({2.0F, 3.0F}, 0.0F, kPi / 2.0F);
  const auto points =
    kau_object_detection_lane::convert_scan_indices_to_points(scan, {0U, 1U});

  ASSERT_EQ(points.size(), 2U);
  EXPECT_EQ(points[0].scan_index, 0U);
  EXPECT_NEAR(points[0].x_m, 2.0, 1e-9);
  EXPECT_NEAR(points[0].y_m, 0.0, 1e-9);
  EXPECT_EQ(points[1].scan_index, 1U);
  EXPECT_NEAR(points[1].x_m, 0.0, 1e-6);
  EXPECT_NEAR(points[1].y_m, 3.0, 1e-6);
}

TEST(LaserScanClusterer, SplitsClustersAtLargeGeometricGap)
{
  const auto scan = make_scan({2.0F, 2.0F, 10.0F, 10.0F}, 0.0F, 0.01F);
  const auto clusters = kau_object_detection_lane::cluster_laser_scan(
    scan, {0U, 1U, 2U, 3U}, test_options());

  ASSERT_EQ(clusters.size(), 2U);
  EXPECT_EQ(clusters[0].points.size(), 2U);
  EXPECT_EQ(clusters[1].points.size(), 2U);
  EXPECT_EQ(clusters[0].points.front().scan_index, 0U);
  EXPECT_EQ(clusters[1].points.front().scan_index, 2U);
}

TEST(LaserScanClusterer, MissingOrNonFiniteBeamBreaksACluster)
{
  const auto nan_value = std::numeric_limits<float>::quiet_NaN();
  const auto scan = make_scan({2.0F, 2.0F, nan_value, 2.0F, 2.0F}, 0.0F, 0.01F);
  const auto clusters = kau_object_detection_lane::cluster_laser_scan(
    scan, {0U, 1U, 3U, 4U}, test_options());

  ASSERT_EQ(clusters.size(), 2U);
  EXPECT_EQ(clusters[0].points.back().scan_index, 1U);
  EXPECT_EQ(clusters[1].points.front().scan_index, 3U);
}

TEST(LaserScanClusterer, RemovesClustersSmallerThanConfiguredMinimum)
{
  const auto scan = make_scan({2.0F, 2.0F}, 0.0F, 0.01F);
  const auto clusters =
    kau_object_detection_lane::cluster_laser_scan(scan, {0U, 1U}, test_options());

  // Raw clustering reports the measured two-beam group as it was observed.
  ASSERT_EQ(clusters.size(), 1U);
  EXPECT_EQ(clusters.front().points.size(), 2U);

  // Only candidate acceptance applies the minimum size.
  EXPECT_TRUE(
    kau_object_detection_lane::select_candidate_clusters(clusters, candidate_options(3U)).empty());
  EXPECT_EQ(
    kau_object_detection_lane::select_candidate_clusters(
      clusters, candidate_options(2U)).size(), 1U);
}

TEST(LaserScanClusterer, MergesBoundaryClustersOnlyForFullCircleScans)
{
  auto scan = make_scan({1.0F, 10.0F, 10.0F, 1.0F}, 0.0F, kPi / 2.0F);
  scan.angle_max = 3.0F * kPi / 2.0F;

  const auto clusters = kau_object_detection_lane::cluster_laser_scan(
    scan, {0U, 1U, 2U, 3U}, test_options(), disabled_filter());

  ASSERT_EQ(clusters.size(), 2U);
  ASSERT_TRUE(clusters.front().wraps_scan_boundary);
  ASSERT_EQ(clusters.front().points.size(), 2U);
  EXPECT_EQ(clusters.front().points.front().scan_index, 3U);
  EXPECT_EQ(clusters.front().points.back().scan_index, 0U);
}

TEST(LaserScanClusterer, DoesNotMergeBoundaryClustersForPartialScan)
{
  auto scan = make_scan({1.0F, 10.0F, 10.0F, 1.0F}, 0.0F, kPi / 2.0F);
  scan.angle_max = kPi;

  const auto clusters = kau_object_detection_lane::cluster_laser_scan(
    scan, {0U, 1U, 2U, 3U}, test_options(), disabled_filter());

  ASSERT_EQ(clusters.size(), 3U);
  EXPECT_FALSE(clusters.front().wraps_scan_boundary);
}

TEST(LaserScanClusterer, ReturnsNoPointsForInvalidGeometry)
{
  const auto scan = make_scan({2.0F, 2.0F}, 0.0F, 0.0F);
  const auto points =
    kau_object_detection_lane::convert_scan_indices_to_points(scan, {0U, 1U});

  EXPECT_TRUE(points.empty());
}

TEST(LaserScanClusterer, AdaptiveNeighborDistanceMatchesClusteringLinkThreshold)
{
  const auto options = test_options();
  const double angle_increment = 2.0 * static_cast<double>(kPi) /
    static_cast<double>(kFullCircleBeamCount);

  const double threshold = kau_object_detection_lane::adaptive_neighbor_distance_m(
    10.0, 10.0, angle_increment, options);
  EXPECT_NEAR(
    threshold,
    options.base_distance_threshold_m +
    options.angular_resolution_scale * 10.0 * angle_increment,
    1e-12);

  // The threshold grows with range, which is what keeps far cones connected.
  EXPECT_GT(
    kau_object_detection_lane::adaptive_neighbor_distance_m(10.0, 10.0, angle_increment, options),
    kau_object_detection_lane::adaptive_neighbor_distance_m(1.0, 1.0, angle_increment, options));

  const double nan_value = std::numeric_limits<double>::quiet_NaN();
  EXPECT_LT(
    kau_object_detection_lane::adaptive_neighbor_distance_m(
      nan_value, 1.0, angle_increment, options),
    0.0);
}

TEST(LaserScanClusterer, ScanIndexFollowsWrapsOnlyForFullCircleScans)
{
  const auto full_circle_scan =
    make_full_circle_scan(std::vector<float>(kFullCircleBeamCount, 1.0F));
  const auto full_circle_topology =
    kau_object_detection_lane::make_scan_neighbor_topology(full_circle_scan, 0.05);

  ASSERT_TRUE(full_circle_topology.valid);
  EXPECT_TRUE(full_circle_topology.wraps_full_circle);
  EXPECT_EQ(full_circle_topology.beam_count, kFullCircleBeamCount);
  EXPECT_TRUE(kau_object_detection_lane::scan_index_follows(5U, 6U, full_circle_topology));
  EXPECT_TRUE(
    kau_object_detection_lane::scan_index_follows(
      kFullCircleBeamCount - 1U, 0U, full_circle_topology));
  EXPECT_FALSE(
    kau_object_detection_lane::scan_index_follows(
      0U, kFullCircleBeamCount - 1U, full_circle_topology));
  EXPECT_FALSE(kau_object_detection_lane::scan_index_follows(6U, 5U, full_circle_topology));
  EXPECT_FALSE(
    kau_object_detection_lane::scan_index_follows(
      kFullCircleBeamCount, 0U, full_circle_topology));

  const auto partial_scan = make_scan({1.0F, 1.0F, 1.0F, 1.0F}, 0.0F, 0.01F);
  const auto partial_topology =
    kau_object_detection_lane::make_scan_neighbor_topology(partial_scan, 0.05);

  ASSERT_TRUE(partial_topology.valid);
  EXPECT_FALSE(partial_topology.wraps_full_circle);
  EXPECT_FALSE(kau_object_detection_lane::scan_index_follows(3U, 0U, partial_topology));

  sensor_msgs::msg::LaserScan empty_scan = partial_scan;
  empty_scan.ranges.clear();
  EXPECT_FALSE(
    kau_object_detection_lane::make_scan_neighbor_topology(empty_scan, 0.05).valid);
}

TEST(LaserScanClusterer, SpeckleFilterRemovesIsolatedSingletonBeam)
{
  auto ranges = make_unmeasured_ranges(kFullCircleBeamCount);
  ranges[100U] = 5.0F;
  ranges[101U] = 5.0F;
  ranges[102U] = 5.0F;
  ranges[300U] = 10.0F;
  const auto scan = make_full_circle_scan(ranges);
  const std::vector<std::size_t> usable_indices{100U, 101U, 102U, 300U};

  const auto topology =
    kau_object_detection_lane::make_scan_neighbor_topology(scan, 0.05);
  const auto points =
    kau_object_detection_lane::convert_scan_indices_to_points(scan, usable_indices);
  const auto filtered = kau_object_detection_lane::filter_isolated_scan_points(
    points, topology, test_options());

  EXPECT_EQ(filtered.input_point_count, 4U);
  EXPECT_EQ(filtered.removed_speckle_count, 1U);
  ASSERT_EQ(filtered.points.size(), 3U);
  EXPECT_EQ(filtered.points.back().scan_index, 102U);

  const auto clusters =
    kau_object_detection_lane::cluster_laser_scan(scan, usable_indices, test_options());
  ASSERT_EQ(clusters.size(), 1U);
  EXPECT_EQ(clusters.front().points.size(), 3U);
  EXPECT_EQ(clusters.front().points.front().scan_index, 100U);
}

TEST(LaserScanClusterer, DisabledSpeckleFilterKeepsIsolatedSingletonBeam)
{
  auto ranges = make_unmeasured_ranges(kFullCircleBeamCount);
  ranges[300U] = 10.0F;
  const auto scan = make_full_circle_scan(ranges);

  EXPECT_TRUE(
    kau_object_detection_lane::cluster_laser_scan(scan, {300U}, test_options()).empty());

  const auto kept = kau_object_detection_lane::cluster_laser_scan(
    scan, {300U}, test_options(), disabled_filter());
  ASSERT_EQ(kept.size(), 1U);
  EXPECT_EQ(kept.front().points.size(), 1U);
}

TEST(LaserScanClusterer, KeepsAdjacentTwoPointClusterFromDistantObstacle)
{
  auto ranges = make_unmeasured_ranges(kFullCircleBeamCount);
  ranges[400U] = 10.0F;
  ranges[401U] = 10.0F;
  const auto scan = make_full_circle_scan(ranges);

  const auto clusters =
    kau_object_detection_lane::cluster_laser_scan(scan, {400U, 401U}, test_options());

  ASSERT_EQ(clusters.size(), 1U);
  ASSERT_EQ(clusters.front().points.size(), 2U);
  EXPECT_EQ(clusters.front().points.front().scan_index, 400U);
  EXPECT_EQ(clusters.front().points.back().scan_index, 401U);
  EXPECT_FALSE(clusters.front().wraps_scan_boundary);
}

TEST(LaserScanClusterer, NodeDefaultOptionsAcceptSupportedTwoPointConeCandidate)
{
  auto ranges = make_unmeasured_ranges(kFullCircleBeamCount);
  ranges[400U] = 10.0F;
  ranges[401U] = 10.0F;
  const auto scan = make_full_circle_scan(ranges);

  // Library defaults mirror the shipped node parameters.
  const auto clusters = kau_object_detection_lane::cluster_laser_scan(scan, {400U, 401U});
  ASSERT_EQ(clusters.size(), 1U);
  EXPECT_EQ(clusters.front().points.size(), 2U);

  const auto candidates = kau_object_detection_lane::select_candidate_clusters(clusters);
  ASSERT_EQ(candidates.size(), 1U);
  EXPECT_EQ(candidates.front().points.size(), 2U);
}

TEST(LaserScanClusterer, KeepsTwoPointClusterAcrossFullCircleBoundary)
{
  auto ranges = make_unmeasured_ranges(kFullCircleBeamCount);
  ranges[kFullCircleBeamCount - 1U] = 10.0F;
  ranges[0U] = 10.0F;
  const auto scan = make_full_circle_scan(ranges);
  const std::vector<std::size_t> usable_indices{0U, kFullCircleBeamCount - 1U};

  const auto topology =
    kau_object_detection_lane::make_scan_neighbor_topology(scan, 0.05);
  const auto points =
    kau_object_detection_lane::convert_scan_indices_to_points(scan, usable_indices);
  const auto filtered = kau_object_detection_lane::filter_isolated_scan_points(
    points, topology, test_options());

  EXPECT_EQ(filtered.removed_speckle_count, 0U);
  EXPECT_EQ(filtered.points.size(), 2U);

  const auto clusters =
    kau_object_detection_lane::cluster_laser_scan(scan, usable_indices, test_options());
  ASSERT_EQ(clusters.size(), 1U);
  ASSERT_EQ(clusters.front().points.size(), 2U);
  EXPECT_TRUE(clusters.front().wraps_scan_boundary);
  EXPECT_EQ(clusters.front().points.front().scan_index, kFullCircleBeamCount - 1U);
  EXPECT_EQ(clusters.front().points.back().scan_index, 0U);

  const auto candidates =
    kau_object_detection_lane::select_candidate_clusters(clusters, candidate_options(2U));
  EXPECT_EQ(candidates.size(), 1U);
}

TEST(LaserScanClusterer, PartialScanDoesNotSupportAcrossFirstAndLastIndex)
{
  auto ranges = make_unmeasured_ranges(kFullCircleBeamCount);
  ranges[kFullCircleBeamCount - 1U] = 10.0F;
  ranges[0U] = 10.0F;
  const auto scan = make_scan(ranges, 0.0F, 0.004F);
  const std::vector<std::size_t> usable_indices{0U, kFullCircleBeamCount - 1U};

  const auto topology =
    kau_object_detection_lane::make_scan_neighbor_topology(scan, 0.05);
  ASSERT_TRUE(topology.valid);
  EXPECT_FALSE(topology.wraps_full_circle);

  const auto points =
    kau_object_detection_lane::convert_scan_indices_to_points(scan, usable_indices);
  const auto filtered = kau_object_detection_lane::filter_isolated_scan_points(
    points, topology, test_options());

  EXPECT_EQ(filtered.removed_speckle_count, 2U);
  EXPECT_TRUE(filtered.points.empty());
  EXPECT_TRUE(
    kau_object_detection_lane::cluster_laser_scan(scan, usable_indices, test_options()).empty());
}

TEST(LaserScanClusterer, MissingBeamRemovesSupportAcrossTheGap)
{
  auto ranges = make_unmeasured_ranges(kFullCircleBeamCount);
  ranges[200U] = 5.0F;
  ranges[201U] = 5.0F;
  ranges[203U] = 5.0F;
  const auto scan = make_full_circle_scan(ranges);
  const std::vector<std::size_t> usable_indices{200U, 201U, 203U};

  const auto topology =
    kau_object_detection_lane::make_scan_neighbor_topology(scan, 0.05);
  const auto points =
    kau_object_detection_lane::convert_scan_indices_to_points(scan, usable_indices);
  const auto filtered = kau_object_detection_lane::filter_isolated_scan_points(
    points, topology, test_options());

  EXPECT_EQ(filtered.removed_speckle_count, 1U);
  ASSERT_EQ(filtered.points.size(), 2U);
  EXPECT_EQ(filtered.points.front().scan_index, 200U);
  EXPECT_EQ(filtered.points.back().scan_index, 201U);
}

TEST(LaserScanClusterer, RequiringTwoSupportingNeighborsDropsTwoPointCandidate)
{
  auto ranges = make_unmeasured_ranges(kFullCircleBeamCount);
  ranges[400U] = 10.0F;
  ranges[401U] = 10.0F;
  const auto scan = make_full_circle_scan(ranges);

  kau_object_detection_lane::NeighborFilterOptions strict_filter;
  strict_filter.minimum_support_neighbors = 2U;

  EXPECT_TRUE(
    kau_object_detection_lane::cluster_laser_scan(
      scan, {400U, 401U}, test_options(), strict_filter).empty());
}

TEST(LaserScanClusterer, SpeckleFilterKeepsEveryPointThatHoldsAClusteringLink)
{
  auto ranges = make_unmeasured_ranges(kFullCircleBeamCount);
  for (std::size_t index = 100U; index <= 110U; ++index) {
    ranges[index] = 3.0F;
  }
  ranges[400U] = 12.0F;
  ranges[401U] = 12.0F;
  ranges[500U] = 1.0F;
  const auto scan = make_full_circle_scan(ranges);

  std::vector<std::size_t> usable_indices;
  for (std::size_t index = 100U; index <= 110U; ++index) {
    usable_indices.push_back(index);
  }
  usable_indices.push_back(400U);
  usable_indices.push_back(401U);
  usable_indices.push_back(500U);

  const auto unfiltered = kau_object_detection_lane::cluster_laser_scan(
    scan, usable_indices, test_options(), disabled_filter());
  const auto filtered =
    kau_object_detection_lane::cluster_laser_scan(scan, usable_indices, test_options());

  // Only the unsupported singleton disappears; every multi-point cluster keeps
  // exactly the members it had before filtering.
  ASSERT_EQ(unfiltered.size(), 3U);
  ASSERT_EQ(filtered.size(), 2U);
  EXPECT_EQ(filtered[0].points.size(), 11U);
  EXPECT_EQ(filtered[1].points.size(), 2U);
  EXPECT_EQ(unfiltered[0].points.size(), 11U);
  EXPECT_EQ(unfiltered[1].points.size(), 2U);
  EXPECT_EQ(unfiltered[2].points.size(), 1U);
}

TEST(LaserScanClusterer, SelectCandidateClustersAppliesMinimumPointsOnly)
{
  const kau_object_detection_lane::Cluster2D single{
    {kau_object_detection_lane::ScanPoint2D{}}, false};
  const kau_object_detection_lane::Cluster2D pair{
    {kau_object_detection_lane::ScanPoint2D{}, kau_object_detection_lane::ScanPoint2D{}}, false};
  const std::vector<kau_object_detection_lane::Cluster2D> clusters{single, pair};

  EXPECT_EQ(kau_object_detection_lane::select_candidate_clusters(clusters).size(), 2U);
  EXPECT_EQ(
    kau_object_detection_lane::select_candidate_clusters(
      clusters, candidate_options(0U)).size(), 2U);
  EXPECT_EQ(
    kau_object_detection_lane::select_candidate_clusters(
      clusters, candidate_options(2U)).size(), 1U);
  EXPECT_TRUE(
    kau_object_detection_lane::select_candidate_clusters(clusters, candidate_options(3U)).empty());
}

}  // namespace
