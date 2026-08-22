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

#ifndef KAU_OBJECT_DETECTION__OBJECT_LIST_DIAGNOSTIC_HPP_
#define KAU_OBJECT_DETECTION__OBJECT_LIST_DIAGNOSTIC_HPP_

#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "kau_msgs/msg/obstacle_circle_array.hpp"
#include "kau_object_detection/object_list.hpp"
#include "kau_object_detection/track_roi.hpp"

namespace kau_object_detection
{

/// Builds the raw diagnostic counterpart of one Object List frame.
///
/// Diagnostics only. The frame is published on a separate topic so the effect
/// of the temporal tracker and of the EMA smoothing can be measured against the
/// candidates that entered them, on the very same scan. Local Path Planning
/// consumes the final Object List and must never subscribe to this frame.
///
/// `pre_tracking_candidates` are the candidates as they existed after the tf2
/// placement stage and before tracker association, outlier rejection and EMA
/// smoothing. They are therefore already target-frame coordinates: "raw" here
/// means unsmoothed, never unprojected LiDAR range.
///
/// The message is built by `build_object_list`, so status handling, frame id,
/// confidence and the drop of a candidate without a resolved target-frame
/// centre are the very same rules the published Object List follows. Passing
/// the same `status` and `stamp` the final frame is published with therefore
/// yields two messages a consumer can pair on `header.stamp` alone, and a
/// non-OK frame yields an empty obstacle array on both.
///
/// The function is stateless and takes its candidates by const reference: it
/// cannot carry a candidate over from an earlier frame, and it cannot perturb
/// the vector the tracker is fed with.
kau_msgs::msg::ObstacleCircleArray build_raw_diagnostic_object_list(
  ObjectListStatus status,
  const std::vector<WorldCircleCandidate> & pre_tracking_candidates,
  const builtin_interfaces::msg::Time & stamp,
  const ObjectListOptions & options = ObjectListOptions{});

}  // namespace kau_object_detection

#endif  // KAU_OBJECT_DETECTION__OBJECT_LIST_DIAGNOSTIC_HPP_
