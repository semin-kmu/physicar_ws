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
// Forked from package "kau_object_detection", its source cluster_geometry.cpp, on 2026-08-25.
//
// This is an independent copy, not a link: kau_object_detection_lane builds and
// runs without that package being present. Fixes made in one package do
// NOT propagate to the other. See README.md "Source synchronisation".
//
// Changes from the original: namespace and include guard renamed to
// kau_object_detection_lane, include paths repointed at this package.
// ---------------------------------------------------------------------------

#include "kau_object_detection_lane/cluster_geometry.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <utility>
#include <vector>

namespace kau_object_detection_lane
{
namespace
{

bool point_is_finite(const ScanPoint2D & point)
{
  return std::isfinite(point.x_m) &&
         std::isfinite(point.y_m) &&
         std::isfinite(point.range_m);
}

double planar_distance(const ScanPoint2D & first, const ScanPoint2D & second)
{
  return std::hypot(second.x_m - first.x_m, second.y_m - first.y_m);
}

double cross_product(
  const ScanPoint2D & origin,
  const ScanPoint2D & first,
  const ScanPoint2D & second)
{
  return (first.x_m - origin.x_m) * (second.y_m - origin.y_m) -
         (first.y_m - origin.y_m) * (second.x_m - origin.x_m);
}

/// Perpendicular distance in metres from `point` to the line through
/// `line_start` and `line_end`.
///
/// `cross_product` has unit m^2 and equals twice the triangle area; dividing it
/// by the line length in m yields a distance in m. A degenerate line falls back
/// to the point-to-point distance.
double perpendicular_distance(
  const ScanPoint2D & line_start,
  const ScanPoint2D & line_end,
  const ScanPoint2D & point)
{
  const double line_length = planar_distance(line_start, line_end);
  if (line_length <= 0.0) {
    return planar_distance(line_start, point);
  }
  return std::abs(cross_product(line_start, line_end, point)) / line_length;
}

/// Exact monotone-chain step. The middle point is dropped when it does not form
/// a strict left turn, which also removes exactly collinear points. No
/// tolerance is applied here so that the hull provably contains every input
/// point; the tolerance is a separate, global collapse decision.
void append_hull_chain(
  const std::vector<ScanPoint2D> & sorted_points,
  const bool reversed,
  std::vector<ScanPoint2D> & chain)
{
  const std::size_t count = sorted_points.size();
  for (std::size_t step = 0U; step < count; ++step) {
    const auto & point = reversed ? sorted_points[count - 1U - step] : sorted_points[step];
    while (chain.size() >= 2U &&
      cross_product(chain[chain.size() - 2U], chain.back(), point) <= 0.0)
    {
      chain.pop_back();
    }
    chain.push_back(point);
  }
}

std::vector<ScanPoint2D> sorted_unique_points(const std::vector<ScanPoint2D> & points)
{
  std::vector<ScanPoint2D> sorted = points;
  std::sort(
    sorted.begin(), sorted.end(),
    [](const ScanPoint2D & first, const ScanPoint2D & second) {
      if (first.x_m != second.x_m) {
        return first.x_m < second.x_m;
      }
      if (first.y_m != second.y_m) {
        return first.y_m < second.y_m;
      }
      return first.scan_index < second.scan_index;
    });
  sorted.erase(
    std::unique(
      sorted.begin(), sorted.end(),
      [](const ScanPoint2D & first, const ScanPoint2D & second) {
        return first.x_m == second.x_m && first.y_m == second.y_m;
      }),
    sorted.end());
  return sorted;
}

double polygon_area(const std::vector<ScanPoint2D> & vertices)
{
  if (vertices.size() < 3U) {
    return 0.0;
  }

  double twice_area = 0.0;
  for (std::size_t index = 0U; index < vertices.size(); ++index) {
    const auto & current = vertices[index];
    const auto & next = vertices[(index + 1U) % vertices.size()];
    twice_area += current.x_m * next.y_m - next.x_m * current.y_m;
  }
  return std::abs(twice_area) * 0.5;
}

/// Index pair of the two vertices that are farthest apart. The extreme pair of
/// a point set always lies on its convex hull, so evaluating the hull vertices
/// is enough and keeps the cost far below the O(n^2) over all scan points.
std::pair<std::size_t, std::size_t> extreme_vertex_pair(
  const std::vector<ScanPoint2D> & vertices)
{
  std::pair<std::size_t, std::size_t> extremes{0U, 0U};
  double largest = 0.0;
  for (std::size_t first = 0U; first < vertices.size(); ++first) {
    for (std::size_t second = first + 1U; second < vertices.size(); ++second) {
      const double distance = planar_distance(vertices[first], vertices[second]);
      if (distance > largest) {
        largest = distance;
        extremes = {first, second};
      }
    }
  }
  return extremes;
}

double largest_pairwise_distance(const std::vector<ScanPoint2D> & vertices)
{
  if (vertices.size() < 2U) {
    return 0.0;
  }
  const auto extremes = extreme_vertex_pair(vertices);
  return planar_distance(vertices[extremes.first], vertices[extremes.second]);
}

/// Collapses a hull whose points all lie within `collinear_tolerance_m` of the
/// line through its two extreme vertices down to those two vertices. The
/// decision is global, so a collapsed footprint never leaves an observed point
/// farther than the tolerance outside it.
std::vector<ScanPoint2D> collapse_collinear_hull(
  const std::vector<ScanPoint2D> & hull,
  const double collinear_tolerance_m)
{
  if (hull.size() < 3U) {
    return hull;
  }

  const auto extremes = extreme_vertex_pair(hull);
  const auto & line_start = hull[extremes.first];
  const auto & line_end = hull[extremes.second];
  for (const auto & vertex : hull) {
    if (perpendicular_distance(line_start, line_end, vertex) > collinear_tolerance_m) {
      return hull;
    }
  }
  return {line_start, line_end};
}

}  // namespace

std::vector<ScanPoint2D> compute_observed_footprint(
  const std::vector<ScanPoint2D> & points,
  const double collinear_tolerance_m)
{
  std::vector<ScanPoint2D> footprint;
  if (points.empty() || !std::isfinite(collinear_tolerance_m) ||
    collinear_tolerance_m < 0.0)
  {
    return footprint;
  }
  if (!std::all_of(points.begin(), points.end(), point_is_finite)) {
    return footprint;
  }

  const auto unique_points = sorted_unique_points(points);
  if (unique_points.size() < 3U) {
    return unique_points;
  }

  std::vector<ScanPoint2D> lower_chain;
  append_hull_chain(unique_points, false, lower_chain);
  std::vector<ScanPoint2D> upper_chain;
  append_hull_chain(unique_points, true, upper_chain);

  footprint.reserve(lower_chain.size() + upper_chain.size());
  footprint.insert(footprint.end(), lower_chain.begin(), lower_chain.end() - 1);
  footprint.insert(footprint.end(), upper_chain.begin(), upper_chain.end() - 1);
  if (footprint.size() < 3U) {
    return {unique_points.front(), unique_points.back()};
  }
  return collapse_collinear_hull(footprint, collinear_tolerance_m);
}

ClusterGeometry2D compute_cluster_geometry(
  const Cluster2D & cluster,
  const ClusterGeometryOptions & options)
{
  ClusterGeometry2D geometry;
  if (cluster.points.empty() ||
    !std::isfinite(options.collinear_tolerance_m) ||
    options.collinear_tolerance_m < 0.0)
  {
    return geometry;
  }
  if (!std::all_of(cluster.points.begin(), cluster.points.end(), point_is_finite)) {
    return geometry;
  }

  const auto & points = cluster.points;
  const auto closest = std::min_element(
    points.begin(), points.end(),
    [](const ScanPoint2D & first, const ScanPoint2D & second) {
      if (first.range_m != second.range_m) {
        return first.range_m < second.range_m;
      }
      return first.scan_index < second.scan_index;
    });

  double sum_x = 0.0;
  double sum_y = 0.0;
  for (const auto & point : points) {
    sum_x += point.x_m;
    sum_y += point.y_m;
  }
  const double point_count = static_cast<double>(points.size());

  geometry.closest_point = *closest;
  geometry.representative_x_m = sum_x / point_count;
  geometry.representative_y_m = sum_y / point_count;
  geometry.representative_range_m =
    std::hypot(geometry.representative_x_m, geometry.representative_y_m);
  geometry.representative_angle_rad =
    std::atan2(geometry.representative_y_m, geometry.representative_x_m);
  geometry.width_m = planar_distance(points.front(), points.back());
  geometry.footprint_vertices =
    compute_observed_footprint(points, options.collinear_tolerance_m);
  geometry.max_extent_m = largest_pairwise_distance(geometry.footprint_vertices);
  geometry.footprint_area_m2 = polygon_area(geometry.footprint_vertices);
  geometry.point_count = points.size();
  geometry.first_scan_index = points.front().scan_index;
  geometry.last_scan_index = points.back().scan_index;
  geometry.wraps_scan_boundary = cluster.wraps_scan_boundary;
  geometry.valid = true;
  return geometry;
}

std::vector<ClusterGeometry2D> compute_all_cluster_geometry(
  const std::vector<Cluster2D> & clusters,
  const ClusterGeometryOptions & options)
{
  std::vector<ClusterGeometry2D> geometries;
  geometries.reserve(clusters.size());
  for (const auto & cluster : clusters) {
    geometries.push_back(compute_cluster_geometry(cluster, options));
  }
  return geometries;
}

}  // namespace kau_object_detection_lane
