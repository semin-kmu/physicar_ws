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
// Forked from package "kau_object_detection" on 2026-08-25, condensing the
// parts of its track_geometry.hpp, track_roi.hpp and object_list_transform.hpp
// that a Localization-free pipeline still needs.
//
// This is an independent copy, not a link. See README.md "Source
// synchronisation".
//
// Removed relative to the originals, deliberately and permanently:
//   * TrackRing / TrackRoiOptions / apply_track_roi  - the boundary polygons
//     are map-frame data and cannot describe base_link.
//   * inside_track_ring annotation                   - same reason.
//   * SensorMountOptions and its offset arithmetic   - the tf2 lookup already
//     resolves base_link <- lidar_link end to end, so re-adding a mount offset
//     here would double-count it.
//   * BasePose2D / transform_sensor_point_to_world   - the Gazebo world pose
//     path is gone; the only transform left is the tf2 one below.
// ---------------------------------------------------------------------------

#ifndef KAU_OBJECT_DETECTION_LANE__OBJECT_TRANSFORM_HPP_
#define KAU_OBJECT_DETECTION_LANE__OBJECT_TRANSFORM_HPP_

#include <cstddef>
#include <vector>

#include "kau_object_detection_lane/cone_occupancy.hpp"

namespace kau_object_detection_lane
{

struct Point2D
{
  double x_m{0.0};
  double y_m{0.0};
};

/// Planar `base_link <- lidar_link` transform of one tf2 lookup.
///
/// This is the transform the published Object List is built on. It is looked up
/// as `object_list_frame_id <- scan.header.frame_id` at `scan.header.stamp`, so
/// the whole mount chain is already resolved inside it. No mount offset may be
/// added on top.
///
/// The rotation is reduced to yaw because the occupancy model is planar. Roll
/// and pitch of the transform are ignored; on this vehicle the mount rotation
/// is identity anyway.
struct SensorToBaseTransform
{
  /// False when the lookup failed or produced a value that cannot be used:
  /// a non-finite translation or a degenerate rotation quaternion.
  bool valid{false};
  double x_m{0.0};
  double y_m{0.0};
  double yaw_rad{0.0};
};

/// Reduces one tf2 transform to its planar terms and validates it.
///
/// The quaternion is not required to be normalised: yaw is recovered with a
/// scale-invariant formulation, and only a degenerate (zero-length) quaternion
/// is rejected. A transform whose translation or rotation is not finite yields
/// an invalid result rather than a silently wrong pose.
SensorToBaseTransform make_sensor_to_base_transform(
  double translation_x_m,
  double translation_y_m,
  double rotation_x,
  double rotation_y,
  double rotation_z,
  double rotation_w);

/// base = translation + R(yaw) * sensor_point.
Point2D transform_sensor_point_to_base(
  const Point2D & sensor_point,
  const SensorToBaseTransform & transform);

/// One circle candidate carried from the occupancy stage into the message
/// builder. The sensor-frame values are copied verbatim; nothing about the
/// circle itself is recomputed here.
struct CircleCandidate
{
  bool base_center_valid{false};
  double base_x_m{0.0};
  double base_y_m{0.0};

  double sensor_x_m{0.0};
  double sensor_y_m{0.0};
  double radius_m{0.0};
  double source_closest_range_m{0.0};
  std::size_t point_count{0U};
  std::size_t first_scan_index{0U};
  std::size_t last_scan_index{0U};
};

/// Copies the sensor-frame terms of one occupancy candidate into the carrier
/// the later stages work on. Nothing about the circle is recomputed; the base
/// frame fields are left unset for `place_candidates_in_base_frame` to fill in.
CircleCandidate carry_sensor_values(const ConeOccupancyCandidate & candidate);

struct PlacementResult
{
  /// False when the transform was missing or unusable. In that case every
  /// candidate is kept without a base-frame centre (fail open), which the
  /// status decision turns into a non-OK frame with an empty obstacle array.
  bool applied{false};
  std::vector<CircleCandidate> accepted;
  std::size_t candidates_before{0U};
  std::size_t candidates_after{0U};
  std::size_t rejected{0U};
};

/// Final Object List coordinate stage: places the sensor-frame circle
/// candidates in the base frame with the tf2 transform.
///
/// There is no ROI stage on this path. What keeps walls and fences from
/// becoming circles at all is the cluster width / max-extent gate and the cone
/// occupancy gate upstream, both unchanged from the original package.
///
/// A candidate whose transformed centre is not finite is counted in `rejected`
/// and dropped, because publishing it would misplace an obstacle.
///
/// This function is stateless. It is handed exactly one scan's candidates and
/// returns exactly that scan's placements: no candidate can survive into a
/// later frame.
PlacementResult place_candidates_in_base_frame(
  const std::vector<ConeOccupancyCandidate> & candidates,
  const SensorToBaseTransform & transform);

}  // namespace kau_object_detection_lane

#endif  // KAU_OBJECT_DETECTION_LANE__OBJECT_TRANSFORM_HPP_
