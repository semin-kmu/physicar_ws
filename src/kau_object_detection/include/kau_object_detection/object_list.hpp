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

#ifndef KAU_OBJECT_DETECTION__OBJECT_LIST_HPP_
#define KAU_OBJECT_DETECTION__OBJECT_LIST_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "kau_msgs/msg/obstacle_circle_array.hpp"
#include "kau_object_detection/track_roi.hpp"

namespace kau_object_detection
{

/// Output status of one Object List frame. The values mirror the constants of
/// `kau_msgs/msg/ObstacleCircleArray` one for one so that the enum and the wire
/// representation can never drift apart.
enum class ObjectListStatus : std::uint8_t
{
  kOk = 0U,
  kLidarUnavailable = 1U,
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
  std::string frame_id{"map"};

  /// Confidence written on every accepted circle. v1 approves candidates in a
  /// binary way, so the only value in use is 1.0.
  float confidence{1.0F};
};

/// Picks the status of one frame from the Gazebo world pose preconditions.
///
/// Simulator-only diagnostic path. The published Object List is decided by
/// `decide_object_list_status_from_transform`; this overload is kept so the
/// Gazebo world pose behaviour stays described and testable.
///
/// The checks are ordered from the most upstream failure to the most
/// downstream one, so the reported status names the earliest stage that broke.
/// `roi_applied` is the final guard: the track ROI stage fails open and hands
/// back sensor-frame candidates when it cannot place them on the map, and
/// reporting that as `kOk` would publish an empty list that reads as
/// "no obstacles" while the real answer is "position unknown".
ObjectListStatus decide_object_list_status(
  bool scan_valid,
  bool world_pose_fresh,
  bool track_ring_valid,
  bool roi_applied);

/// Track-ROI-gated status decision of the tf2 transform path.
///
/// Retained, with its tests, for the day the real target-frame track boundaries
/// are available; `decide_object_list_status_from_placement` below is what the
/// node publishes on today. It replaces the world
/// pose freshness check with the outcome of the `object_list_frame_id <-
/// scan.header.frame_id` lookup at `scan.header.stamp`: without that transform
/// there is no way to place a candidate in the target frame, and the frame is
/// reported as `kTransformUnavailable` rather than as an empty observation.
///
/// The remaining checks keep the meaning they have above: an unusable track
/// ring or a ROI stage that failed open is a configuration problem of this
/// node, so it stays `kInternalError`.
ObjectListStatus decide_object_list_status_from_transform(
  bool scan_valid,
  bool transform_available,
  bool track_ring_valid,
  bool roi_applied);

/// Picks the status of one frame published from the tf2 placement path.
///
/// This is the decision the node actually publishes on. The track ROI no longer
/// gates the output, so the track ring plays no part in it: a frame is usable
/// exactly when the scan validated and the target frame transform was obtained
/// at the scan measurement time.
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
/// Only candidates whose world centre was actually resolved become circles, and
/// only on a `kOk` frame. Every other status yields an empty obstacle array:
/// this function is stateless, so a failing frame can never carry a candidate
/// from an earlier one.
kau_msgs::msg::ObstacleCircleArray build_object_list(
  ObjectListStatus status,
  const std::vector<WorldCircleCandidate> & candidates,
  const builtin_interfaces::msg::Time & stamp,
  const ObjectListOptions & options = ObjectListOptions{});

}  // namespace kau_object_detection

#endif  // KAU_OBJECT_DETECTION__OBJECT_LIST_HPP_
