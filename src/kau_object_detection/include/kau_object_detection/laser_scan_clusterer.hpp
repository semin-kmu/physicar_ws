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

#ifndef KAU_OBJECT_DETECTION__LASER_SCAN_CLUSTERER_HPP_
#define KAU_OBJECT_DETECTION__LASER_SCAN_CLUSTERER_HPP_

#include <cstddef>
#include <vector>

#include "sensor_msgs/msg/laser_scan.hpp"

namespace kau_object_detection
{

struct ScanPoint2D
{
  std::size_t scan_index{0U};
  double x_m{0.0};
  double y_m{0.0};
  double range_m{0.0};
  double angle_rad{0.0};
};

struct Cluster2D
{
  std::vector<ScanPoint2D> points;
  bool wraps_scan_boundary{false};
};

struct ScanClusteringOptions
{
  double base_distance_threshold_m{0.05};
  double angular_resolution_scale{1.5};
  double circular_scan_tolerance_rad{0.05};
};

/// Beam adjacency layout of one scan. Computed once per message so that the
/// speckle filter and the clusterer agree on which scan indices are immediate
/// neighbours, including the wrap between the last and the first beam.
struct ScanNeighborTopology
{
  std::size_t beam_count{0U};
  double angle_increment_rad{0.0};
  /// True only when the scan metadata describes a closed 360-degree sweep, so
  /// that beam `beam_count - 1` is followed by beam `0`.
  bool wraps_full_circle{false};
  bool valid{false};
};

/// Neighbour/speckle filtering applied to the usable scan points before
/// clustering. A point is kept when it links to its immediate previous or next
/// beam under the same adaptive distance rule the clusterer uses, so a real
/// two-beam obstacle survives while a beam with no reachable neighbour at all
/// is dropped.
struct NeighborFilterOptions
{
  bool enabled{true};
  /// Number of supporting immediate neighbours a point needs. One keeps
  /// mutually supported two-beam candidates; two would discard them again.
  std::size_t minimum_support_neighbors{1U};
  /// Support radius relative to the clustering link threshold. At 1.0 the two
  /// radii are identical, which guarantees that a point holding any clustering
  /// link is never removed as speckle.
  double neighbor_distance_scale{1.0};
};

struct NeighborFilterResult
{
  /// Supported points in ascending scan-index order.
  std::vector<ScanPoint2D> points;
  std::size_t input_point_count{0U};
  std::size_t removed_speckle_count{0U};
};

/// Acceptance rule applied to raw clusters. This is deliberately separate from
/// clustering: raw clustering reports the measured geometry, and this decides
/// which of those clusters count as obstacle candidates.
struct ClusterCandidateOptions
{
  std::size_t minimum_cluster_points{1U};
};

std::vector<ScanPoint2D> convert_scan_indices_to_points(
  const sensor_msgs::msg::LaserScan & scan,
  const std::vector<std::size_t> & usable_indices);

bool scan_covers_full_circle(
  const sensor_msgs::msg::LaserScan & scan,
  double tolerance_rad);

ScanNeighborTopology make_scan_neighbor_topology(
  const sensor_msgs::msg::LaserScan & scan,
  double circular_scan_tolerance_rad);

/// True when `second_index` is the immediate successor beam of `first_index`,
/// wrapping from the last beam to beam zero only for full-circle scans.
bool scan_index_follows(
  std::size_t first_index,
  std::size_t second_index,
  const ScanNeighborTopology & topology);

/// Adaptive distance, in metres, below which two beams count as connected.
/// Shared by the clusterer and the speckle filter so both use one formula.
/// Returns a negative value when the inputs or options are unusable.
double adaptive_neighbor_distance_m(
  double first_range_m,
  double second_range_m,
  double angle_increment_rad,
  const ScanClusteringOptions & options);

NeighborFilterResult filter_isolated_scan_points(
  const std::vector<ScanPoint2D> & points,
  const ScanNeighborTopology & topology,
  const ScanClusteringOptions & clustering_options,
  const NeighborFilterOptions & filter_options = NeighborFilterOptions{});

/// Raw adaptive adjacent-point clustering. No minimum-size rule is applied
/// here; every measured group is reported, including one- and two-point ones.
std::vector<Cluster2D> cluster_scan_points(
  const std::vector<ScanPoint2D> & points,
  const ScanNeighborTopology & topology,
  const ScanClusteringOptions & options = ScanClusteringOptions{});

std::vector<Cluster2D> cluster_laser_scan(
  const sensor_msgs::msg::LaserScan & scan,
  const std::vector<std::size_t> & usable_indices,
  const ScanClusteringOptions & options = ScanClusteringOptions{},
  const NeighborFilterOptions & filter_options = NeighborFilterOptions{});

std::vector<Cluster2D> select_candidate_clusters(
  const std::vector<Cluster2D> & clusters,
  const ClusterCandidateOptions & options = ClusterCandidateOptions{});

}  // namespace kau_object_detection

#endif  // KAU_OBJECT_DETECTION__LASER_SCAN_CLUSTERER_HPP_
