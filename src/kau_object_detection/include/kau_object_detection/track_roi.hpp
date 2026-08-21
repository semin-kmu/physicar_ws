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

#ifndef KAU_OBJECT_DETECTION__TRACK_ROI_HPP_
#define KAU_OBJECT_DETECTION__TRACK_ROI_HPP_

#include <cstddef>
#include <vector>

#include "kau_object_detection/cone_occupancy.hpp"
#include "kau_object_detection/track_geometry.hpp"

namespace kau_object_detection
{

/// Planar part of the base_footprint -> lidar_link mount. The measured z offset
/// is not carried here because the occupancy model and the track boundary are
/// both planar; the rotation between the two frames is identity on this
/// vehicle, so only the translation is needed.
struct SensorMountOptions
{
  double lidar_offset_x_m{-0.027};
  double lidar_offset_y_m{0.0};
};

/// Pose of the vehicle base in the world frame, reduced to the planar terms
/// this transform needs. Kept free of the world pose sampling type so the
/// transform stays independently testable.
struct BasePose2D
{
  double x_m{0.0};
  double y_m{0.0};
  double yaw_rad{0.0};
};

/// world = base + R(yaw) * (mount_offset + sensor_point), with identity
/// rotation between base_footprint and lidar_link.
Point2D transform_sensor_point_to_world(
  const Point2D & sensor_point,
  const BasePose2D & base_pose,
  const SensorMountOptions & mount = SensorMountOptions{});

/// One circle candidate carried through the ROI stage. The sensor-frame values
/// are copied verbatim from the occupancy stage; nothing about the circle
/// itself is recomputed here.
struct WorldCircleCandidate
{
  bool world_center_valid{false};
  double world_x_m{0.0};
  double world_y_m{0.0};
  bool inside_track_ring{false};

  double sensor_x_m{0.0};
  double sensor_y_m{0.0};
  double radius_m{0.0};
  double source_closest_range_m{0.0};
  std::size_t point_count{0U};
  std::size_t first_scan_index{0U};
  std::size_t last_scan_index{0U};
};

struct TrackRoiResult
{
  /// False when the world pose was stale or missing, or the ring was not
  /// configured. In that case every candidate is kept unchanged (fail-open).
  bool applied{false};
  std::vector<WorldCircleCandidate> accepted;
  std::size_t candidates_before{0U};
  std::size_t candidates_after{0U};
  std::size_t rejected{0U};
};

/// Read-only track ROI stage.
///
/// Only valid occupancy candidates enter. When `world_pose_fresh` is false or
/// the ring is unusable, the stage fails open: every candidate is kept, world
/// centres stay unset, and `applied` is false. Otherwise each centre is
/// transformed to the world frame and candidates whose centre is outside the
/// drivable ring are dropped.
TrackRoiResult apply_track_roi(
  const std::vector<ConeOccupancyCandidate> & candidates,
  const TrackRing & ring,
  bool world_pose_fresh,
  const BasePose2D & base_pose,
  const SensorMountOptions & mount = SensorMountOptions{},
  const TrackRoiOptions & options = TrackRoiOptions{});

}  // namespace kau_object_detection

#endif  // KAU_OBJECT_DETECTION__TRACK_ROI_HPP_
