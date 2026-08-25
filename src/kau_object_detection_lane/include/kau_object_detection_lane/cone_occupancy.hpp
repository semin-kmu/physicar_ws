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
// Forked from package "kau_object_detection", its header cone_occupancy.hpp, on 2026-08-25.
//
// This is an independent copy, not a link: kau_object_detection_lane builds and
// runs without that package being present. Fixes made in one package do
// NOT propagate to the other. See README.md "Source synchronisation".
//
// Changes from the original: namespace and include guard renamed to
// kau_object_detection_lane, include paths repointed at this package.
// ---------------------------------------------------------------------------

#ifndef KAU_OBJECT_DETECTION_LANE__CONE_OCCUPANCY_HPP_
#define KAU_OBJECT_DETECTION_LANE__CONE_OCCUPANCY_HPP_

#include <cstddef>
#include <vector>

#include "kau_object_detection_lane/cluster_geometry.hpp"

namespace kau_object_detection_lane
{

/// Why a cluster geometry did not yield a circular occupancy candidate.
/// Reported for diagnostics only; `kNone` accompanies every valid candidate.
enum class ConeOccupancyRejection
{
  kNone = 0,
  kInvalidGeometry,
  kInvalidOptions,
  kDegenerateClosestPoint,
  kObservedWidthAboveGate,
  kNonFiniteResult,
};

/// Tunables of the provisional circular occupancy model. Every value is a
/// physical property of the obstacle or of the sensor mounting, never an
/// absolute map position.
struct ConeOccupancyOptions
{
  /// Occupancy radius used for the candidate circle. A competition cone has a
  /// 0.18 x 0.18 m base footprint, so half of that edge is 0.09 m.
  double nominal_cone_radius_m{0.09};

  /// Radius of the cone cross-section that is actually visible at the LiDAR
  /// mounting height. The measured surface sits this far in front of the cone
  /// axis, so the closest point is pushed away from the sensor by this amount
  /// to recover a provisional axis centre.
  double visible_slice_radius_m{0.043};

  /// Largest observed extent, in metres, that may still be a single cone.
  /// Clusters wider than this are walls or track boundaries and are rejected
  /// instead of being reported as a cone-sized circle.
  double maximum_observed_width_m{0.30};
};

/// Provisional circular occupancy candidate derived from one measured cluster.
///
/// This is a read-only model built from the observed geometry and the
/// configured cone dimensions. It carries no safety inflation, no tracking
/// identity, and no absolute-frame transform; the coordinates stay in the
/// sensor frame the cluster was measured in.
struct ConeOccupancyCandidate
{
  bool valid{false};
  ConeOccupancyRejection rejection{ConeOccupancyRejection::kInvalidGeometry};

  /// Provisional cone axis centre in the measurement frame.
  double center_x_m{0.0};
  double center_y_m{0.0};
  /// Occupancy radius. Currently the configured nominal radius, not a fit.
  double radius_m{0.0};

  double center_range_m{0.0};
  double center_angle_rad{0.0};

  /// Provenance of the candidate, preserved for diagnostics and for the future
  /// Object List contract.
  double source_closest_range_m{0.0};
  double source_closest_x_m{0.0};
  double source_closest_y_m{0.0};
  std::size_t point_count{0U};
  double observed_width_m{0.0};
  double observed_max_extent_m{0.0};
  std::size_t first_scan_index{0U};
  std::size_t last_scan_index{0U};
  bool wraps_scan_boundary{false};
};

/// Largest observed extent of a cluster, used by the width gate. This is the
/// stricter of the chord width and the hull max extent.
double observed_extent_m(const ClusterGeometry2D & geometry);

ConeOccupancyCandidate compute_cone_occupancy_candidate(
  const ClusterGeometry2D & geometry,
  const ConeOccupancyOptions & options = ConeOccupancyOptions{});

std::vector<ConeOccupancyCandidate> compute_all_cone_occupancy_candidates(
  const std::vector<ClusterGeometry2D> & geometries,
  const ConeOccupancyOptions & options = ConeOccupancyOptions{});

}  // namespace kau_object_detection_lane

#endif  // KAU_OBJECT_DETECTION_LANE__CONE_OCCUPANCY_HPP_
