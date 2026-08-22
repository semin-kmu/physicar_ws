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

#include "kau_object_detection/track_roi.hpp"

#include <cmath>
#include <cstddef>
#include <vector>

namespace kau_object_detection
{
namespace
{

bool base_pose_is_usable(const BasePose2D & base_pose, const SensorMountOptions & mount)
{
  return std::isfinite(base_pose.x_m) &&
         std::isfinite(base_pose.y_m) &&
         std::isfinite(base_pose.yaw_rad) &&
         std::isfinite(mount.lidar_offset_x_m) &&
         std::isfinite(mount.lidar_offset_y_m);
}

}  // namespace

WorldCircleCandidate carry_sensor_values(const ConeOccupancyCandidate & candidate)
{
  WorldCircleCandidate carried;
  carried.sensor_x_m = candidate.center_x_m;
  carried.sensor_y_m = candidate.center_y_m;
  carried.radius_m = candidate.radius_m;
  carried.source_closest_range_m = candidate.source_closest_range_m;
  carried.point_count = candidate.point_count;
  carried.first_scan_index = candidate.first_scan_index;
  carried.last_scan_index = candidate.last_scan_index;
  return carried;
}

Point2D transform_sensor_point_to_world(
  const Point2D & sensor_point,
  const BasePose2D & base_pose,
  const SensorMountOptions & mount)
{
  const double base_x_m = mount.lidar_offset_x_m + sensor_point.x_m;
  const double base_y_m = mount.lidar_offset_y_m + sensor_point.y_m;
  const double cos_yaw = std::cos(base_pose.yaw_rad);
  const double sin_yaw = std::sin(base_pose.yaw_rad);

  return Point2D{
    base_pose.x_m + cos_yaw * base_x_m - sin_yaw * base_y_m,
    base_pose.y_m + sin_yaw * base_x_m + cos_yaw * base_y_m};
}

TrackRoiResult apply_track_roi(
  const std::vector<ConeOccupancyCandidate> & candidates,
  const TrackRing & ring,
  const bool world_pose_fresh,
  const BasePose2D & base_pose,
  const SensorMountOptions & mount,
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

  // Fail open. Without a fresh pose or a usable ring there is no trustworthy
  // way to place a candidate on the map, and dropping real obstacles is worse
  // than keeping off-track ones.
  const bool can_apply =
    world_pose_fresh && ring.valid && base_pose_is_usable(base_pose, mount);
  if (!can_apply) {
    for (const auto * candidate : usable) {
      result.accepted.push_back(carry_sensor_values(*candidate));
    }
    result.candidates_after = result.accepted.size();
    return result;
  }

  result.applied = true;
  for (const auto * candidate : usable) {
    auto carried = carry_sensor_values(*candidate);
    const auto world_center = transform_sensor_point_to_world(
      Point2D{candidate->center_x_m, candidate->center_y_m}, base_pose, mount);
    if (!std::isfinite(world_center.x_m) || !std::isfinite(world_center.y_m)) {
      ++result.rejected;
      continue;
    }

    carried.world_center_valid = true;
    carried.world_x_m = world_center.x_m;
    carried.world_y_m = world_center.y_m;
    carried.inside_track_ring = point_is_in_track_ring(ring, world_center, options);
    if (!carried.inside_track_ring) {
      ++result.rejected;
      continue;
    }

    result.accepted.push_back(carried);
  }

  result.candidates_after = result.accepted.size();
  return result;
}

}  // namespace kau_object_detection
