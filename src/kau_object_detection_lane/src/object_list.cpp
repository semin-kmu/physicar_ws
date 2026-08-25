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
// Forked from package "kau_object_detection", its source object_list.cpp, on
// 2026-08-25. See the header for what was removed.
// ---------------------------------------------------------------------------

#include "kau_object_detection_lane/object_list.hpp"

#include <vector>

namespace kau_object_detection_lane
{

ObjectListStatus decide_object_list_status_from_placement(
  const bool scan_valid,
  const bool transform_available,
  const bool candidates_placed)
{
  if (!scan_valid) {
    return ObjectListStatus::kLidarUnavailable;
  }
  // The tf2 lookup is the only source of the base-frame coordinates, so its
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
  const std::vector<CircleCandidate> & candidates,
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
    // A candidate without a resolved base centre only carries sensor-frame
    // values. Publishing those under the base frame would misplace an obstacle
    // instead of omitting it, so it is dropped.
    if (!candidate.base_center_valid) {
      continue;
    }

    kau_msgs::msg::ObstacleCircle circle;
    circle.center_x = static_cast<float>(candidate.base_x_m);
    circle.center_y = static_cast<float>(candidate.base_y_m);
    circle.radius = static_cast<float>(candidate.radius_m);
    circle.confidence = options.confidence;
    message.obstacles.push_back(circle);
  }

  return message;
}

}  // namespace kau_object_detection_lane
