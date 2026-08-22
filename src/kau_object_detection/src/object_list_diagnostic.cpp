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

#include "kau_object_detection/object_list_diagnostic.hpp"

#include <vector>

#include "kau_object_detection/object_list.hpp"

namespace kau_object_detection
{

kau_msgs::msg::ObstacleCircleArray build_raw_diagnostic_object_list(
  const ObjectListStatus status,
  const std::vector<WorldCircleCandidate> & pre_tracking_candidates,
  const builtin_interfaces::msg::Time & stamp,
  const ObjectListOptions & options)
{
  // Deliberately the same builder as the published Object List. The two frames
  // may only differ in the candidates they were given, never in how those
  // candidates are turned into a message; otherwise the comparison would be
  // measuring this function instead of the smoothing.
  return build_object_list(status, pre_tracking_candidates, stamp, options);
}

}  // namespace kau_object_detection
