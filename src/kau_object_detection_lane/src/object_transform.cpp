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
// parts of its track_roi.cpp and object_list_transform.cpp that a
// Localization-free pipeline still needs. See the header for what was removed.
// ---------------------------------------------------------------------------

#include "kau_object_detection_lane/object_transform.hpp"

#include <cmath>
#include <vector>

namespace kau_object_detection_lane
{
namespace
{

/// Smallest squared quaternion norm still treated as a rotation. Below this a
/// quaternion carries no recoverable heading.
constexpr double kMinimumQuaternionNormSquared = 1e-12;

}  // namespace

SensorToBaseTransform make_sensor_to_base_transform(
  const double translation_x_m,
  const double translation_y_m,
  const double rotation_x,
  const double rotation_y,
  const double rotation_z,
  const double rotation_w)
{
  SensorToBaseTransform transform;

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

Point2D transform_sensor_point_to_base(
  const Point2D & sensor_point,
  const SensorToBaseTransform & transform)
{
  // No mount offset is added here. The tf2 lookup that produced `transform`
  // already spans base_link <- lidar_link, so the sensor mount translation is
  // inside `transform.x_m` / `transform.y_m` already.
  const double cos_yaw = std::cos(transform.yaw_rad);
  const double sin_yaw = std::sin(transform.yaw_rad);

  return Point2D{
    transform.x_m + cos_yaw * sensor_point.x_m - sin_yaw * sensor_point.y_m,
    transform.y_m + sin_yaw * sensor_point.x_m + cos_yaw * sensor_point.y_m};
}

CircleCandidate carry_sensor_values(const ConeOccupancyCandidate & candidate)
{
  CircleCandidate carried;
  carried.sensor_x_m = candidate.center_x_m;
  carried.sensor_y_m = candidate.center_y_m;
  carried.radius_m = candidate.radius_m;
  carried.source_closest_range_m = candidate.source_closest_range_m;
  carried.point_count = candidate.point_count;
  carried.first_scan_index = candidate.first_scan_index;
  carried.last_scan_index = candidate.last_scan_index;
  return carried;
}

PlacementResult place_candidates_in_base_frame(
  const std::vector<ConeOccupancyCandidate> & candidates,
  const SensorToBaseTransform & transform)
{
  PlacementResult result;
  result.accepted.reserve(candidates.size());

  std::vector<const ConeOccupancyCandidate *> usable;
  usable.reserve(candidates.size());
  for (const auto & candidate : candidates) {
    if (candidate.valid) {
      usable.push_back(&candidate);
    }
  }
  result.candidates_before = usable.size();

  // Fail open. Without a transform there is no base frame coordinate to give a
  // candidate, so none is invented; the caller reports the frame as
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
    const auto base_center = transform_sensor_point_to_base(
      Point2D{candidate->center_x_m, candidate->center_y_m}, transform);
    if (!std::isfinite(base_center.x_m) || !std::isfinite(base_center.y_m)) {
      ++result.rejected;
      continue;
    }

    carried.base_center_valid = true;
    carried.base_x_m = base_center.x_m;
    carried.base_y_m = base_center.y_m;
    result.accepted.push_back(carried);
  }

  result.candidates_after = result.accepted.size();
  return result;
}

}  // namespace kau_object_detection_lane
