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

#include "kau_object_detection/object_list_transform.hpp"

#include <cmath>
#include <vector>

namespace kau_object_detection
{
namespace
{

/// The tf2 lookup already contains every link between the target frame and the
/// scan frame, so the planar mount offset of the simulator path must not be
/// applied a second time.
constexpr SensorMountOptions kNoAdditionalMountOffset{0.0, 0.0};

/// Smallest squared quaternion norm still treated as a rotation. Below this a
/// quaternion carries no recoverable heading.
constexpr double kMinimumQuaternionNormSquared = 1e-12;

}  // namespace

SensorToTargetTransform make_sensor_to_target_transform(
  const double translation_x_m,
  const double translation_y_m,
  const double rotation_x,
  const double rotation_y,
  const double rotation_z,
  const double rotation_w)
{
  SensorToTargetTransform transform;

  if (!std::isfinite(translation_x_m) || !std::isfinite(translation_y_m) ||
    !std::isfinite(rotation_x) || !std::isfinite(rotation_y) ||
    !std::isfinite(rotation_z) || !std::isfinite(rotation_w))
  {
    return transform;
  }

  const double norm_squared = rotation_x * rotation_x + rotation_y * rotation_y +
    rotation_z * rotation_z + rotation_w * rotation_w;
  if (!(norm_squared >= kMinimumQuaternionNormSquared)) {
    return transform;
  }

  // Scale-invariant yaw: both arguments carry the same quaternion norm factor,
  // so atan2 returns the same angle for a normalised and an unnormalised input.
  const double sin_yaw_cos_pitch =
    2.0 * (rotation_w * rotation_z + rotation_x * rotation_y);
  const double cos_yaw_cos_pitch =
    rotation_w * rotation_w + rotation_x * rotation_x -
    rotation_y * rotation_y - rotation_z * rotation_z;
  const double yaw_rad = std::atan2(sin_yaw_cos_pitch, cos_yaw_cos_pitch);
  if (!std::isfinite(yaw_rad)) {
    return transform;
  }

  transform.valid = true;
  transform.x_m = translation_x_m;
  transform.y_m = translation_y_m;
  transform.yaw_rad = yaw_rad;
  return transform;
}

Point2D transform_sensor_point_to_target(
  const Point2D & sensor_point,
  const SensorToTargetTransform & transform)
{
  return transform_sensor_point_to_world(
    sensor_point,
    BasePose2D{transform.x_m, transform.y_m, transform.yaw_rad},
    kNoAdditionalMountOffset);
}

TrackRoiResult place_candidates_in_target_frame(
  const std::vector<ConeOccupancyCandidate> & candidates,
  const SensorToTargetTransform & transform,
  const TrackRing & ring,
  const TrackRoiOptions & options)
{
  TrackRoiResult result;
  result.accepted.reserve(candidates.size());

  std::vector<const ConeOccupancyCandidate *> usable;
  usable.reserve(candidates.size());
  for (const auto & candidate : candidates) {
    if (candidate.valid) {
      usable.push_back(&candidate);
    }
  }
  result.candidates_before = usable.size();

  // Fail open. Without a transform there is no target frame coordinate to give
  // a candidate, so none is invented; the caller reports the frame as
  // STATUS_TRANSFORM_UNAVAILABLE and publishes nothing.
  if (!transform.valid) {
    for (const auto * candidate : usable) {
      result.accepted.push_back(carry_sensor_values(*candidate));
    }
    result.candidates_after = result.accepted.size();
    return result;
  }

  result.applied = true;
  for (const auto * candidate : usable) {
    auto carried = carry_sensor_values(*candidate);
    const auto target_center = transform_sensor_point_to_target(
      Point2D{candidate->center_x_m, candidate->center_y_m}, transform);
    if (!std::isfinite(target_center.x_m) || !std::isfinite(target_center.y_m)) {
      ++result.rejected;
      continue;
    }

    carried.world_center_valid = true;
    carried.world_x_m = target_center.x_m;
    carried.world_y_m = target_center.y_m;
    // Annotation only. A candidate outside the ring is still published: the
    // ring is not expressed in this frame and is not a rejection rule here.
    carried.inside_track_ring =
      ring.valid && point_is_in_track_ring(ring, target_center, options);
    result.accepted.push_back(carried);
  }

  result.candidates_after = result.accepted.size();
  return result;
}

TrackRoiResult apply_object_list_transform(
  const std::vector<ConeOccupancyCandidate> & candidates,
  const TrackRing & ring,
  const SensorToTargetTransform & transform,
  const TrackRoiOptions & options)
{
  // Same ROI stage as the simulator path, driven by the tf2 transform instead
  // of the Gazebo world pose. `transform.valid` takes the place of the pose
  // freshness gate and the mount offset is zero, because the lookup already
  // resolved the target -> scan frame chain end to end.
  return apply_track_roi(
    candidates, ring, transform.valid,
    BasePose2D{transform.x_m, transform.y_m, transform.yaw_rad},
    kNoAdditionalMountOffset, options);
}

}  // namespace kau_object_detection
