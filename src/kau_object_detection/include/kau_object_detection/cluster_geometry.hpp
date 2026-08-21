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

#ifndef KAU_OBJECT_DETECTION__CLUSTER_GEOMETRY_HPP_
#define KAU_OBJECT_DETECTION__CLUSTER_GEOMETRY_HPP_

#include <cstddef>
#include <vector>

#include "kau_object_detection/laser_scan_clusterer.hpp"

namespace kau_object_detection
{

struct ClusterGeometryOptions
{
  /// Maximum perpendicular distance, in metres, at which a whole cluster counts
  /// as collinear. The footprint is the exact convex hull; only when every hull
  /// vertex lies within this distance of the line through the two extreme
  /// vertices does the footprint collapse to those two points. This is a
  /// distance tolerance, not a raw cross-product (m^2) tolerance; see
  /// `perpendicular_distance` in the implementation for the exact test.
  double collinear_tolerance_m{0.001};
};

/// Observed obstacle shape of a single LiDAR cluster.
///
/// Every value is derived from measured scan points only. No safety margin,
/// inflation, or unobserved-side completion is applied here; those belong to
/// Local Path Planning.
struct ClusterGeometry2D
{
  bool valid{false};

  /// Measured point of the cluster with the smallest range.
  ScanPoint2D closest_point;

  /// Centroid of the observed points. This is the centre of the measured
  /// surface, not the geometric centre of the physical obstacle.
  double representative_x_m{0.0};
  double representative_y_m{0.0};
  double representative_range_m{0.0};
  double representative_angle_rad{0.0};

  /// Chord distance between the first and the last point of the cluster in
  /// scan order. This is the observed width; it is neither the true physical
  /// width of the obstacle nor a planner safety width.
  double width_m{0.0};

  /// Largest distance between two observed points, evaluated over the
  /// footprint vertices because the extreme pair always lies on the hull.
  double max_extent_m{0.0};

  /// Convex hull of the observed points, counter-clockwise, without inflation.
  /// No observed point ever lies farther than `collinear_tolerance_m` outside
  /// it, and for a non-collapsed hull every observed point is contained exactly.
  std::vector<ScanPoint2D> footprint_vertices;
  double footprint_area_m2{0.0};

  std::size_t point_count{0U};
  std::size_t first_scan_index{0U};
  std::size_t last_scan_index{0U};
  bool wraps_scan_boundary{false};
};

std::vector<ScanPoint2D> compute_observed_footprint(
  const std::vector<ScanPoint2D> & points,
  double collinear_tolerance_m);

ClusterGeometry2D compute_cluster_geometry(
  const Cluster2D & cluster,
  const ClusterGeometryOptions & options = ClusterGeometryOptions{});

std::vector<ClusterGeometry2D> compute_all_cluster_geometry(
  const std::vector<Cluster2D> & clusters,
  const ClusterGeometryOptions & options = ClusterGeometryOptions{});

}  // namespace kau_object_detection

#endif  // KAU_OBJECT_DETECTION__CLUSTER_GEOMETRY_HPP_
