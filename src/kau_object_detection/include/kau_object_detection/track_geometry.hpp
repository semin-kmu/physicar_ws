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

#ifndef KAU_OBJECT_DETECTION__TRACK_GEOMETRY_HPP_
#define KAU_OBJECT_DETECTION__TRACK_GEOMETRY_HPP_

#include <cstddef>
#include <vector>

namespace kau_object_detection
{

struct Point2D
{
  double x_m{0.0};
  double y_m{0.0};
};

/// Drivable area of the track: everything inside `outer` and outside `inner`.
/// Both polygons come from the published route boundary and may be given
/// closed (last point equal to the first) or open; both forms behave the same.
struct TrackRing
{
  std::vector<Point2D> outer;
  std::vector<Point2D> inner;
  bool valid{false};
};

struct TrackRoiOptions
{
  /// Half-width of the keep band around either boundary, in metres. A centre
  /// within this distance of a boundary is kept even when the strict inside or
  /// outside test rejects it, so a cone standing on the edge is never dropped
  /// because of centre-estimation error.
  double boundary_tolerance_m{0.15};
};

/// Even-odd ray crossing test. Points exactly on an edge are not guaranteed a
/// particular answer here; use `point_is_in_track_ring`, which pairs this with
/// the boundary tolerance.
bool point_in_polygon(const std::vector<Point2D> & polygon, const Point2D & point);

/// Shortest distance from `point` to the closed polyline of `polygon`.
/// Returns infinity for a polygon with fewer than two distinct points.
double distance_to_polygon_boundary_m(
  const std::vector<Point2D> & polygon,
  const Point2D & point);

/// Builds a ring and validates that both polygons can form one. A ring stays
/// invalid when either polygon has fewer than three points.
TrackRing make_track_ring(
  const std::vector<Point2D> & outer,
  const std::vector<Point2D> & inner);

TrackRing make_track_ring(
  const std::vector<double> & outer_x,
  const std::vector<double> & outer_y,
  const std::vector<double> & inner_x,
  const std::vector<double> & inner_y);

/// True when the point may be on the track. Conservative by design: a point
/// within `boundary_tolerance_m` of either boundary is kept regardless of which
/// side it falls on.
bool point_is_in_track_ring(
  const TrackRing & ring,
  const Point2D & point,
  const TrackRoiOptions & options = TrackRoiOptions{});

}  // namespace kau_object_detection

#endif  // KAU_OBJECT_DETECTION__TRACK_GEOMETRY_HPP_
