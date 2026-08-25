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
// Forked from package "kau_object_detection", its header object_list.hpp, on
// 2026-08-25. This is an independent copy, not a link. See README.md
// "Source synchronisation".
//
// Removed relative to the original, deliberately and permanently:
//   * decide_object_list_status              - Gazebo world pose path.
//   * decide_object_list_status_from_transform - track-ROI-gated variant.
// Only the placement decision the node actually publishes on is kept, so there
// is exactly one status rule in this package and it cannot drift from the node.
//
// The default frame_id is base_link here, not map.
// ---------------------------------------------------------------------------

#ifndef KAU_OBJECT_DETECTION_LANE__OBJECT_LIST_HPP_
#define KAU_OBJECT_DETECTION_LANE__OBJECT_LIST_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "kau_msgs/msg/obstacle_circle_array.hpp"
#include "kau_object_detection_lane/object_transform.hpp"

namespace kau_object_detection_lane
{

/// Output status of one Object List frame. The values mirror the constants of
/// `kau_msgs/msg/ObstacleCircleArray` one for one so that the enum and the wire
/// representation can never drift apart.
enum class ObjectListStatus : std::uint8_t
{
  kOk = 0U,
  kLidarUnavailable = 1U,
  /// Not reachable on this path. There is no vehicle pose source in a
  /// Localization-free pipeline, so nothing can report a stale pose. The
  /// constant is kept so the enum stays aligned with the message.
  kPoseUnavailable = 2U,
  /// The tf2 lookup of `object_list_frame_id <- scan.header.frame_id` at the
  /// scan measurement time failed: unknown frame, disconnected tree, the stamp
  /// is outside the buffer, or the transform itself is unusable.
  kTransformUnavailable = 3U,
  kInternalError = 4U,
};

/// Fixed parts of every published Object List frame.
struct ObjectListOptions
{
  /// Carried on every frame, including failures, so a consumer never has to
  /// branch on the status to know which frame the message speaks about.
  ///
  /// base_link, not map: this package exists to publish obstacles the vehicle
  /// can use without any localisation running.
  std::string frame_id{"base_link"};

  /// Confidence written on every accepted circle. v1 approves candidates in a
  /// binary way, so the only value in use is 1.0.
  float confidence{1.0F};
};

/// Picks the status of one frame published from the tf2 placement path.
///
/// A frame is usable exactly when the scan validated and the base frame
/// transform was obtained at the scan measurement time.
///
/// `candidates_placed` is the last guard. It is false only when the placement
/// stage failed open despite a transform it reported as valid, which would be
/// an internal inconsistency of this node rather than a missing transform.
ObjectListStatus decide_object_list_status_from_placement(
  bool scan_valid,
  bool transform_available,
  bool candidates_placed);

/// Builds one Object List message.
///
/// Only candidates whose base centre was actually resolved become circles, and
/// only on a `kOk` frame. Every other status yields an empty obstacle array:
/// this function is stateless, so a failing frame can never carry a candidate
/// from an earlier one.
kau_msgs::msg::ObstacleCircleArray build_object_list(
  ObjectListStatus status,
  const std::vector<CircleCandidate> & candidates,
  const builtin_interfaces::msg::Time & stamp,
  const ObjectListOptions & options = ObjectListOptions{});

}  // namespace kau_object_detection_lane

#endif  // KAU_OBJECT_DETECTION_LANE__OBJECT_LIST_HPP_
