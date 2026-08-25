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
// Forked from package "kau_object_detection", its source cone_occupancy.cpp, on 2026-08-25.
//
// This is an independent copy, not a link: kau_object_detection_lane builds and
// runs without that package being present. Fixes made in one package do
// NOT propagate to the other. See README.md "Source synchronisation".
//
// Changes from the original: namespace and include guard renamed to
// kau_object_detection_lane, include paths repointed at this package.
// ---------------------------------------------------------------------------

#include "kau_object_detection_lane/cone_occupancy.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

namespace kau_object_detection_lane
{
namespace
{

ConeOccupancyCandidate rejected_candidate(const ConeOccupancyRejection rejection)
{
  ConeOccupancyCandidate candidate;
  candidate.valid = false;
  candidate.rejection = rejection;
  return candidate;
}

bool options_are_usable(const ConeOccupancyOptions & options)
{
  return std::isfinite(options.nominal_cone_radius_m) &&
         options.nominal_cone_radius_m > 0.0 &&
         std::isfinite(options.visible_slice_radius_m) &&
         options.visible_slice_radius_m >= 0.0 &&
         std::isfinite(options.maximum_observed_width_m) &&
         options.maximum_observed_width_m > 0.0;
}

}  // namespace

double observed_extent_m(const ClusterGeometry2D & geometry)
{
  const double width = std::isfinite(geometry.width_m) ? geometry.width_m : 0.0;
  const double extent =
    std::isfinite(geometry.max_extent_m) ? geometry.max_extent_m : 0.0;
  return std::max(width, extent);
}

ConeOccupancyCandidate compute_cone_occupancy_candidate(
  const ClusterGeometry2D & geometry,
  const ConeOccupancyOptions & options)
{
  if (!options_are_usable(options)) {
    return rejected_candidate(ConeOccupancyRejection::kInvalidOptions);
  }
  if (!geometry.valid || geometry.point_count == 0U) {
    return rejected_candidate(ConeOccupancyRejection::kInvalidGeometry);
  }
  if (!std::isfinite(geometry.width_m) || !std::isfinite(geometry.max_extent_m)) {
    return rejected_candidate(ConeOccupancyRejection::kInvalidGeometry);
  }

  const auto & closest = geometry.closest_point;
  const double closest_norm_m = std::hypot(closest.x_m, closest.y_m);
  if (!std::isfinite(closest.x_m) || !std::isfinite(closest.y_m) ||
    !std::isfinite(closest.range_m) ||
    closest.range_m <= 0.0 ||
    !std::isfinite(closest_norm_m) ||
    closest_norm_m <= 0.0)
  {
    return rejected_candidate(ConeOccupancyRejection::kDegenerateClosestPoint);
  }

  // A wall or track boundary stays connected across many beams. Reporting it as
  // a cone-sized circle would understate the obstacle, so it is rejected here
  // rather than shrunk.
  const double extent_m = observed_extent_m(geometry);
  if (extent_m > options.maximum_observed_width_m) {
    return rejected_candidate(ConeOccupancyRejection::kObservedWidthAboveGate);
  }

  // The LiDAR only ever sees the near side of the cone. Push the closest point
  // away from the sensor along its own bearing by the visible slice radius to
  // recover a provisional cone axis.
  const double direction_x = closest.x_m / closest_norm_m;
  const double direction_y = closest.y_m / closest_norm_m;

  ConeOccupancyCandidate candidate;
  candidate.center_x_m = closest.x_m + direction_x * options.visible_slice_radius_m;
  candidate.center_y_m = closest.y_m + direction_y * options.visible_slice_radius_m;
  candidate.radius_m = options.nominal_cone_radius_m;
  candidate.center_range_m = std::hypot(candidate.center_x_m, candidate.center_y_m);
  candidate.center_angle_rad = std::atan2(candidate.center_y_m, candidate.center_x_m);
  candidate.source_closest_range_m = closest.range_m;
  candidate.source_closest_x_m = closest.x_m;
  candidate.source_closest_y_m = closest.y_m;
  candidate.point_count = geometry.point_count;
  candidate.observed_width_m = geometry.width_m;
  candidate.observed_max_extent_m = geometry.max_extent_m;
  candidate.first_scan_index = geometry.first_scan_index;
  candidate.last_scan_index = geometry.last_scan_index;
  candidate.wraps_scan_boundary = geometry.wraps_scan_boundary;

  if (!std::isfinite(candidate.center_x_m) ||
    !std::isfinite(candidate.center_y_m) ||
    !std::isfinite(candidate.center_range_m) ||
    !std::isfinite(candidate.center_angle_rad) ||
    !std::isfinite(candidate.radius_m))
  {
    return rejected_candidate(ConeOccupancyRejection::kNonFiniteResult);
  }

  candidate.rejection = ConeOccupancyRejection::kNone;
  candidate.valid = true;
  return candidate;
}

std::vector<ConeOccupancyCandidate> compute_all_cone_occupancy_candidates(
  const std::vector<ClusterGeometry2D> & geometries,
  const ConeOccupancyOptions & options)
{
  std::vector<ConeOccupancyCandidate> candidates;
  candidates.reserve(geometries.size());
  for (const auto & geometry : geometries) {
    candidates.push_back(compute_cone_occupancy_candidate(geometry, options));
  }
  return candidates;
}

}  // namespace kau_object_detection_lane
