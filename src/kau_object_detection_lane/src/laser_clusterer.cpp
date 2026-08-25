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
// Forked from package "kau_object_detection", its source laser_scan_clusterer.cpp, on 2026-08-25.
//
// This is an independent copy, not a link: kau_object_detection_lane builds and
// runs without that package being present. Fixes made in one package do
// NOT propagate to the other. See README.md "Source synchronisation".
//
// Changes from the original: namespace and include guard renamed to
// kau_object_detection_lane, include paths repointed at this package.
// ---------------------------------------------------------------------------

#include "kau_object_detection_lane/laser_clusterer.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

namespace kau_object_detection_lane
{
namespace
{

constexpr double kTwoPi = 6.28318530717958647692;

bool is_finite(const float value)
{
  return std::isfinite(static_cast<double>(value));
}

bool has_valid_scan_geometry(const sensor_msgs::msg::LaserScan & scan)
{
  return is_finite(scan.angle_min) &&
         is_finite(scan.angle_max) &&
         is_finite(scan.angle_increment) &&
         scan.angle_max > scan.angle_min &&
         scan.angle_increment > 0.0F &&
         is_finite(scan.range_min) &&
         is_finite(scan.range_max) &&
         scan.range_min >= 0.0F &&
         scan.range_max > scan.range_min;
}

double point_distance(const ScanPoint2D & first, const ScanPoint2D & second)
{
  return std::hypot(second.x_m - first.x_m, second.y_m - first.y_m);
}

bool points_are_adjacent(
  const ScanPoint2D & first,
  const ScanPoint2D & second,
  const double angle_increment_rad,
  const ScanClusteringOptions & options)
{
  const double threshold = adaptive_neighbor_distance_m(
    first.range_m, second.range_m, angle_increment_rad, options);
  return threshold >= 0.0 && point_distance(first, second) <= threshold;
}

/// True when `later` is the immediate next beam of `earlier` and the two lie
/// within the scaled adaptive distance, so each supports the other.
bool neighbor_supports_point(
  const ScanPoint2D & earlier,
  const ScanPoint2D & later,
  const ScanNeighborTopology & topology,
  const ScanClusteringOptions & clustering_options,
  const double neighbor_distance_scale)
{
  if (!scan_index_follows(earlier.scan_index, later.scan_index, topology)) {
    return false;
  }

  const double link_threshold = adaptive_neighbor_distance_m(
    earlier.range_m, later.range_m, topology.angle_increment_rad, clustering_options);
  if (link_threshold < 0.0) {
    return false;
  }

  const double threshold = neighbor_distance_scale * link_threshold;
  return std::isfinite(threshold) &&
         threshold >= 0.0 &&
         point_distance(earlier, later) <= threshold;
}

}  // namespace

std::vector<ScanPoint2D> convert_scan_indices_to_points(
  const sensor_msgs::msg::LaserScan & scan,
  const std::vector<std::size_t> & usable_indices)
{
  std::vector<ScanPoint2D> points;
  if (!has_valid_scan_geometry(scan)) {
    return points;
  }

  points.reserve(usable_indices.size());
  for (const auto index : usable_indices) {
    if (index >= scan.ranges.size()) {
      continue;
    }

    const float range = scan.ranges[index];
    if (!is_finite(range) || range < scan.range_min || range > scan.range_max) {
      continue;
    }

    const double angle =
      static_cast<double>(scan.angle_min) +
      static_cast<double>(index) * static_cast<double>(scan.angle_increment);
    const double range_m = static_cast<double>(range);
    if (!std::isfinite(angle)) {
      continue;
    }

    points.push_back(ScanPoint2D{
        index,
        range_m * std::cos(angle),
        range_m * std::sin(angle),
        range_m,
        angle});
  }

  std::sort(
    points.begin(), points.end(),
    [](const ScanPoint2D & first, const ScanPoint2D & second) {
      return first.scan_index < second.scan_index;
    });
  return points;
}

bool scan_covers_full_circle(
  const sensor_msgs::msg::LaserScan & scan,
  const double tolerance_rad)
{
  if (!has_valid_scan_geometry(scan) || !std::isfinite(tolerance_rad) ||
    tolerance_rad < 0.0)
  {
    return false;
  }

  const double covered_angle =
    static_cast<double>(scan.angle_max - scan.angle_min) +
    std::abs(static_cast<double>(scan.angle_increment));
  return std::abs(covered_angle - kTwoPi) <= tolerance_rad;
}

ScanNeighborTopology make_scan_neighbor_topology(
  const sensor_msgs::msg::LaserScan & scan,
  const double circular_scan_tolerance_rad)
{
  ScanNeighborTopology topology;
  if (!has_valid_scan_geometry(scan) || scan.ranges.empty()) {
    return topology;
  }

  topology.beam_count = scan.ranges.size();
  topology.angle_increment_rad = static_cast<double>(scan.angle_increment);
  topology.wraps_full_circle =
    topology.beam_count > 1U &&
    scan_covers_full_circle(scan, circular_scan_tolerance_rad);
  topology.valid = true;
  return topology;
}

bool scan_index_follows(
  const std::size_t first_index,
  const std::size_t second_index,
  const ScanNeighborTopology & topology)
{
  if (!topology.valid ||
    first_index >= topology.beam_count ||
    second_index >= topology.beam_count)
  {
    return false;
  }

  if (second_index == first_index + 1U) {
    return true;
  }

  return topology.wraps_full_circle &&
         first_index + 1U == topology.beam_count &&
         second_index == 0U;
}

double adaptive_neighbor_distance_m(
  const double first_range_m,
  const double second_range_m,
  const double angle_increment_rad,
  const ScanClusteringOptions & options)
{
  if (!std::isfinite(first_range_m) ||
    !std::isfinite(second_range_m) ||
    !std::isfinite(angle_increment_rad) ||
    options.base_distance_threshold_m < 0.0 ||
    options.angular_resolution_scale < 0.0)
  {
    return -1.0;
  }

  const double expected_angular_spacing =
    0.5 * (first_range_m + second_range_m) * std::abs(angle_increment_rad);
  const double threshold =
    options.base_distance_threshold_m +
    options.angular_resolution_scale * expected_angular_spacing;

  if (!std::isfinite(threshold) || threshold < 0.0) {
    return -1.0;
  }
  return threshold;
}

NeighborFilterResult filter_isolated_scan_points(
  const std::vector<ScanPoint2D> & points,
  const ScanNeighborTopology & topology,
  const ScanClusteringOptions & clustering_options,
  const NeighborFilterOptions & filter_options)
{
  NeighborFilterResult result;
  result.input_point_count = points.size();

  // Fail open: an unusable configuration must never silently discard
  // measurements. The clusterer keeps its own guards for that case.
  const bool filter_is_usable =
    filter_options.enabled &&
    filter_options.minimum_support_neighbors > 0U &&
    std::isfinite(filter_options.neighbor_distance_scale) &&
    filter_options.neighbor_distance_scale >= 0.0 &&
    topology.valid;
  if (!filter_is_usable) {
    result.points = points;
    return result;
  }

  const bool has_wrap_pair = topology.wraps_full_circle && points.size() > 1U;
  const bool wrap_pair_supports =
    has_wrap_pair &&
    neighbor_supports_point(
      points.back(), points.front(), topology, clustering_options,
      filter_options.neighbor_distance_scale);

  result.points.reserve(points.size());
  for (std::size_t index = 0U; index < points.size(); ++index) {
    std::size_t support_count = 0U;

    if (index > 0U &&
      neighbor_supports_point(
        points[index - 1U], points[index], topology, clustering_options,
        filter_options.neighbor_distance_scale))
    {
      ++support_count;
    }
    if (index + 1U < points.size() &&
      neighbor_supports_point(
        points[index], points[index + 1U], topology, clustering_options,
        filter_options.neighbor_distance_scale))
    {
      ++support_count;
    }
    if (wrap_pair_supports && (index == 0U || index + 1U == points.size())) {
      ++support_count;
    }

    if (support_count >= filter_options.minimum_support_neighbors) {
      result.points.push_back(points[index]);
    } else {
      ++result.removed_speckle_count;
    }
  }

  return result;
}

std::vector<Cluster2D> cluster_scan_points(
  const std::vector<ScanPoint2D> & points,
  const ScanNeighborTopology & topology,
  const ScanClusteringOptions & options)
{
  std::vector<Cluster2D> clusters;
  if (options.base_distance_threshold_m < 0.0 ||
    options.angular_resolution_scale < 0.0 ||
    !topology.valid ||
    points.empty())
  {
    return clusters;
  }

  for (const auto & point : points) {
    if (clusters.empty()) {
      clusters.push_back(Cluster2D{{point}, false});
      continue;
    }

    auto & current_cluster = clusters.back();
    const auto & previous_point = current_cluster.points.back();
    const bool consecutive_indices =
      point.scan_index == previous_point.scan_index + 1U;
    if (!consecutive_indices ||
      !points_are_adjacent(
        previous_point, point, topology.angle_increment_rad, options))
    {
      clusters.push_back(Cluster2D{{point}, false});
      continue;
    }

    current_cluster.points.push_back(point);
  }

  const bool can_merge_boundary_clusters =
    clusters.size() > 1U &&
    topology.wraps_full_circle &&
    points.front().scan_index == 0U &&
    points.back().scan_index == topology.beam_count - 1U &&
    points_are_adjacent(
      clusters.back().points.back(),
      clusters.front().points.front(),
      topology.angle_increment_rad,
      options);

  if (can_merge_boundary_clusters) {
    Cluster2D merged_cluster = clusters.back();
    clusters.pop_back();
    const Cluster2D first_cluster = clusters.front();
    clusters.erase(clusters.begin());
    merged_cluster.points.insert(
      merged_cluster.points.end(),
      first_cluster.points.begin(),
      first_cluster.points.end());
    merged_cluster.wraps_scan_boundary = true;
    clusters.insert(clusters.begin(), merged_cluster);
  }

  return clusters;
}

std::vector<Cluster2D> cluster_laser_scan(
  const sensor_msgs::msg::LaserScan & scan,
  const std::vector<std::size_t> & usable_indices,
  const ScanClusteringOptions & options,
  const NeighborFilterOptions & filter_options)
{
  std::vector<Cluster2D> clusters;
  if (options.base_distance_threshold_m < 0.0 ||
    options.angular_resolution_scale < 0.0)
  {
    return clusters;
  }

  const auto topology =
    make_scan_neighbor_topology(scan, options.circular_scan_tolerance_rad);
  if (!topology.valid) {
    return clusters;
  }

  const auto points = convert_scan_indices_to_points(scan, usable_indices);
  if (points.empty()) {
    return clusters;
  }

  const auto filtered =
    filter_isolated_scan_points(points, topology, options, filter_options);
  return cluster_scan_points(filtered.points, topology, options);
}

std::vector<Cluster2D> select_candidate_clusters(
  const std::vector<Cluster2D> & clusters,
  const ClusterCandidateOptions & options)
{
  const std::size_t minimum_points =
    std::max<std::size_t>(1U, options.minimum_cluster_points);

  std::vector<Cluster2D> accepted;
  accepted.reserve(clusters.size());
  for (const auto & cluster : clusters) {
    if (cluster.points.size() >= minimum_points) {
      accepted.push_back(cluster);
    }
  }
  return accepted;
}

}  // namespace kau_object_detection_lane
