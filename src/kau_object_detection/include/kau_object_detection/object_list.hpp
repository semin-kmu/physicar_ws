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
  /// Reserved for the future tf2 based transform. Nothing in this package
  /// produces it yet; the constant exists so the contract is complete.
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

/// Picks the status of one frame from the pipeline preconditions.
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
