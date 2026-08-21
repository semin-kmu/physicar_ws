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

#include "kau_object_detection/track_geometry.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

namespace kau_object_detection
{
namespace
{

bool point_is_finite(const Point2D & point)
{
  return std::isfinite(point.x_m) && std::isfinite(point.y_m);
}

bool polygon_is_usable(const std::vector<Point2D> & polygon)
{
  if (polygon.size() < 3U) {
    return false;
  }
  return std::all_of(polygon.begin(), polygon.end(), point_is_finite);
}

/// Distance from `point` to the segment `start`-`end`.
double distance_to_segment_m(
  const Point2D & start,
  const Point2D & end,
  const Point2D & point)
{
  const double segment_x = end.x_m - start.x_m;
  const double segment_y = end.y_m - start.y_m;
  const double segment_length_squared = segment_x * segment_x + segment_y * segment_y;
  if (segment_length_squared <= 0.0) {
    return std::hypot(point.x_m - start.x_m, point.y_m - start.y_m);
  }

  double projection =
    ((point.x_m - start.x_m) * segment_x + (point.y_m - start.y_m) * segment_y) /
    segment_length_squared;
  projection = std::clamp(projection, 0.0, 1.0);

  const double closest_x = start.x_m + projection * segment_x;
  const double closest_y = start.y_m + projection * segment_y;
  return std::hypot(point.x_m - closest_x, point.y_m - closest_y);
}

}  // namespace

bool point_in_polygon(const std::vector<Point2D> & polygon, const Point2D & point)
{
  if (!polygon_is_usable(polygon) || !point_is_finite(point)) {
    return false;
  }

  bool inside = false;
  const std::size_t count = polygon.size();
  for (std::size_t index = 0U, previous = count - 1U; index < count; previous = index++) {
    const auto & current_vertex = polygon[index];
    const auto & previous_vertex = polygon[previous];

    const bool spans_scanline =
      (current_vertex.y_m > point.y_m) != (previous_vertex.y_m > point.y_m);
    if (!spans_scanline) {
      continue;
    }

    const double delta_y = previous_vertex.y_m - current_vertex.y_m;
    if (delta_y == 0.0) {
      continue;
    }
    const double crossing_x =
      current_vertex.x_m +
      (point.y_m - current_vertex.y_m) * (previous_vertex.x_m - current_vertex.x_m) / delta_y;
    if (point.x_m < crossing_x) {
      inside = !inside;
    }
  }
  return inside;
}

double distance_to_polygon_boundary_m(
  const std::vector<Point2D> & polygon,
  const Point2D & point)
{
  if (!polygon_is_usable(polygon) || !point_is_finite(point)) {
    return std::numeric_limits<double>::infinity();
  }

  double smallest = std::numeric_limits<double>::infinity();
  const std::size_t count = polygon.size();
  for (std::size_t index = 0U; index < count; ++index) {
    const auto & start = polygon[index];
    const auto & end = polygon[(index + 1U) % count];
    smallest = std::min(smallest, distance_to_segment_m(start, end, point));
  }
  return smallest;
}

TrackRing make_track_ring(
  const std::vector<Point2D> & outer,
  const std::vector<Point2D> & inner)
{
  TrackRing ring;
  if (!polygon_is_usable(outer) || !polygon_is_usable(inner)) {
    return ring;
  }
  ring.outer = outer;
  ring.inner = inner;
  ring.valid = true;
  return ring;
}

TrackRing make_track_ring(
  const std::vector<double> & outer_x,
  const std::vector<double> & outer_y,
  const std::vector<double> & inner_x,
  const std::vector<double> & inner_y)
{
  TrackRing ring;
  if (outer_x.size() != outer_y.size() || inner_x.size() != inner_y.size()) {
    return ring;
  }

  std::vector<Point2D> outer;
  outer.reserve(outer_x.size());
  for (std::size_t index = 0U; index < outer_x.size(); ++index) {
    outer.push_back(Point2D{outer_x[index], outer_y[index]});
  }

  std::vector<Point2D> inner;
  inner.reserve(inner_x.size());
  for (std::size_t index = 0U; index < inner_x.size(); ++index) {
    inner.push_back(Point2D{inner_x[index], inner_y[index]});
  }

  return make_track_ring(outer, inner);
}

bool point_is_in_track_ring(
  const TrackRing & ring,
  const Point2D & point,
  const TrackRoiOptions & options)
{
  if (!ring.valid || !point_is_finite(point)) {
    return false;
  }

  const double tolerance_m =
    std::isfinite(options.boundary_tolerance_m) && options.boundary_tolerance_m > 0.0 ?
    options.boundary_tolerance_m :
    0.0;

  // A centre close to either boundary is kept whichever side it landed on, so
  // an edge-standing cone is never dropped by centre-estimation error.
  if (tolerance_m > 0.0) {
    if (distance_to_polygon_boundary_m(ring.outer, point) <= tolerance_m ||
      distance_to_polygon_boundary_m(ring.inner, point) <= tolerance_m)
    {
      return true;
    }
  }

  return point_in_polygon(ring.outer, point) && !point_in_polygon(ring.inner, point);
}

}  // namespace kau_object_detection
