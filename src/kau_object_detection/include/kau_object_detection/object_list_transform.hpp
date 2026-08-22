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

#ifndef KAU_OBJECT_DETECTION__OBJECT_LIST_TRANSFORM_HPP_
#define KAU_OBJECT_DETECTION__OBJECT_LIST_TRANSFORM_HPP_

#include <vector>

#include "kau_object_detection/cone_occupancy.hpp"
#include "kau_object_detection/track_geometry.hpp"
#include "kau_object_detection/track_roi.hpp"

namespace kau_object_detection
{

/// Planar `target <- source` transform of one tf2 lookup.
///
/// This is the transform the published Object List is built on. It is looked up
/// as `object_list_frame_id <- scan.header.frame_id` at `scan.header.stamp`, so
/// the full localisation and mount chain (map -> ... -> base -> lidar) is
/// already resolved inside it. No mount offset may be added on top.
///
/// The rotation is reduced to yaw because both the occupancy model and the
/// track boundary are planar. Roll and pitch of the transform are ignored, in
/// the same way the existing simulator path ignores them.
struct SensorToTargetTransform
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
SensorToTargetTransform make_sensor_to_target_transform(
  double translation_x_m,
  double translation_y_m,
  double rotation_x,
  double rotation_y,
  double rotation_z,
  double rotation_w);

/// target = translation + R(yaw) * sensor_point.
Point2D transform_sensor_point_to_target(
  const Point2D & sensor_point,
  const SensorToTargetTransform & transform);

/// Final Object List coordinate stage: places the sensor-frame circle
/// candidates in the target frame with the tf2 transform.
///
/// The track ROI is deliberately *not* used to drop candidates here. The
/// configured boundary polygons are expressed in the simulator world frame and
/// do not describe the Object List target frame, so applying them would reject
/// every candidate. Off-track objects are not an avoidance target for now, and
/// what keeps walls and fences from becoming circles at all is the cluster
/// width / max-extent gate and the cone occupancy gate upstream, both unchanged.
///
/// `applied` follows the transform alone: an invalid transform makes the stage
/// fail open, leaving every candidate without a target-frame centre, which the
/// status decision turns into a non-OK frame with an empty obstacle array. A
/// candidate whose transformed centre is not finite is still counted in
/// `rejected` and dropped, because publishing it would misplace an obstacle.
///
/// `inside_track_ring` is still filled in whenever the ring is usable, as a
/// read-only annotation for the diagnostics. Nothing acts on it.
TrackRoiResult place_candidates_in_target_frame(
  const std::vector<ConeOccupancyCandidate> & candidates,
  const SensorToTargetTransform & transform,
  const TrackRing & ring,
  const TrackRoiOptions & options = TrackRoiOptions{});

/// Track-ROI-gated variant of the stage above.
///
/// Retained, with its tests, for the day the real target-frame outer/inner
/// boundaries are available: at that point the ROI can be put back in front of
/// the publisher by swapping this in. It does not gate the published Object
/// List today.
TrackRoiResult apply_object_list_transform(
  const std::vector<ConeOccupancyCandidate> & candidates,
  const TrackRing & ring,
  const SensorToTargetTransform & transform,
  const TrackRoiOptions & options = TrackRoiOptions{});

}  // namespace kau_object_detection

#endif  // KAU_OBJECT_DETECTION__OBJECT_LIST_TRANSFORM_HPP_
