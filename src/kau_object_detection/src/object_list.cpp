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

#include "kau_object_detection/object_list.hpp"

#include <vector>

namespace kau_object_detection
{

ObjectListStatus decide_object_list_status(
  const bool scan_valid,
  const bool world_pose_fresh,
  const bool track_ring_valid,
  const bool roi_applied)
{
  if (!scan_valid) {
    return ObjectListStatus::kLidarUnavailable;
  }
  if (!world_pose_fresh) {
    return ObjectListStatus::kPoseUnavailable;
  }
  if (!track_ring_valid) {
    return ObjectListStatus::kInternalError;
  }
  // The ROI stage refused the transform for a reason the three checks above do
  // not cover, such as a non-finite base pose or mount offset.
  if (!roi_applied) {
    return ObjectListStatus::kInternalError;
  }
  return ObjectListStatus::kOk;
}

ObjectListStatus decide_object_list_status_from_transform(
  const bool scan_valid,
  const bool transform_available,
  const bool track_ring_valid,
  const bool roi_applied)
{
  if (!scan_valid) {
    return ObjectListStatus::kLidarUnavailable;
  }
  // The tf2 lookup is the only source of the target-frame coordinates on this
  // path, so its failure is reported for what it is instead of being folded
  // into the pose status of the simulator path.
  if (!transform_available) {
    return ObjectListStatus::kTransformUnavailable;
  }
  if (!track_ring_valid) {
    return ObjectListStatus::kInternalError;
  }
  // The transform was available and the ring is usable, yet the ROI stage still
  // failed open. That leaves this node's own configuration as the cause.
  if (!roi_applied) {
    return ObjectListStatus::kInternalError;
  }
  return ObjectListStatus::kOk;
}

ObjectListStatus decide_object_list_status_from_placement(
  const bool scan_valid,
  const bool transform_available,
  const bool candidates_placed)
{
  if (!scan_valid) {
    return ObjectListStatus::kLidarUnavailable;
  }
  // The tf2 lookup is the only source of the target-frame coordinates, so its
  // failure is reported for what it is and never as an empty observation.
  if (!transform_available) {
    return ObjectListStatus::kTransformUnavailable;
  }
  // The transform was usable, yet the placement stage still failed open. That
  // leaves this node itself as the cause.
  if (!candidates_placed) {
    return ObjectListStatus::kInternalError;
  }
  return ObjectListStatus::kOk;
}

kau_msgs::msg::ObstacleCircleArray build_object_list(
  const ObjectListStatus status,
  const std::vector<WorldCircleCandidate> & candidates,
  const builtin_interfaces::msg::Time & stamp,
  const ObjectListOptions & options)
{
  kau_msgs::msg::ObstacleCircleArray message;
  message.header.stamp = stamp;
  message.header.frame_id = options.frame_id;
  message.status = static_cast<std::uint8_t>(status);

  if (status != ObjectListStatus::kOk) {
    return message;
  }

  message.obstacles.reserve(candidates.size());
  for (const auto & candidate : candidates) {
    // A candidate without a resolved world centre only carries sensor-frame
    // values. Publishing those under the map frame would misplace an obstacle
    // instead of omitting it, so it is dropped.
    if (!candidate.world_center_valid) {
      continue;
    }

    kau_msgs::msg::ObstacleCircle circle;
    circle.center_x = static_cast<float>(candidate.world_x_m);
    circle.center_y = static_cast<float>(candidate.world_y_m);
    circle.radius = static_cast<float>(candidate.radius_m);
    circle.confidence = options.confidence;
    message.obstacles.push_back(circle);
  }

  return message;
}

}  // namespace kau_object_detection
