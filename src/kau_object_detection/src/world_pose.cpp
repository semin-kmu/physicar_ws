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

#include "kau_object_detection/world_pose.hpp"

#include <cmath>
#include <cstddef>
#include <limits>
#include <string>
#include <vector>

namespace kau_object_detection
{
namespace
{

constexpr double kRadiansToDegrees = 57.29577951308232087680;

bool candidate_is_usable(const WorldPoseCandidate & candidate)
{
  if (!std::isfinite(candidate.x_m) || !std::isfinite(candidate.y_m) ||
    !std::isfinite(candidate.orientation_w) ||
    !std::isfinite(candidate.orientation_x) ||
    !std::isfinite(candidate.orientation_y) ||
    !std::isfinite(candidate.orientation_z) ||
    !std::isfinite(candidate.stamp_s))
  {
    return false;
  }

  const double norm = std::sqrt(
    candidate.orientation_w * candidate.orientation_w +
    candidate.orientation_x * candidate.orientation_x +
    candidate.orientation_y * candidate.orientation_y +
    candidate.orientation_z * candidate.orientation_z);
  return std::isfinite(norm) && norm > 1e-9;
}

bool candidate_matches(const WorldPoseCandidate & candidate, const std::string & base_frame)
{
  if (base_frame.empty()) {
    return false;
  }
  return candidate.child_frame_id == base_frame || candidate.name == base_frame;
}

WorldPoseSample make_sample(
  const WorldPoseCandidate & candidate,
  const bool exact_name_match,
  const double received_monotonic_s)
{
  WorldPoseSample sample;
  sample.valid = true;
  sample.exact_name_match = exact_name_match;
  sample.source_name =
    candidate.child_frame_id.empty() ? candidate.name : candidate.child_frame_id;
  sample.x_m = candidate.x_m;
  sample.y_m = candidate.y_m;
  sample.yaw_rad = quaternion_to_yaw_rad(
    candidate.orientation_w, candidate.orientation_x,
    candidate.orientation_y, candidate.orientation_z);
  sample.stamp_s = candidate.stamp_s;
  sample.received_monotonic_s = received_monotonic_s;
  return sample;
}

}  // namespace

double quaternion_to_yaw_rad(
  const double orientation_w,
  const double orientation_x,
  const double orientation_y,
  const double orientation_z)
{
  const double sin_yaw = 2.0 *
    (orientation_w * orientation_z + orientation_x * orientation_y);
  const double cos_yaw = 1.0 - 2.0 *
    (orientation_y * orientation_y + orientation_z * orientation_z);
  if (!std::isfinite(sin_yaw) || !std::isfinite(cos_yaw) ||
    (sin_yaw == 0.0 && cos_yaw == 0.0))
  {
    return 0.0;
  }
  return std::atan2(sin_yaw, cos_yaw);
}

double radians_to_degrees(const double radians)
{
  return radians * kRadiansToDegrees;
}

WorldPoseSample select_world_base_pose(
  const std::vector<WorldPoseCandidate> & candidates,
  const std::string & base_frame,
  const double received_monotonic_s)
{
  WorldPoseSample sample;
  if (!std::isfinite(received_monotonic_s)) {
    return sample;
  }

  for (const auto & candidate : candidates) {
    if (candidate_matches(candidate, base_frame) && candidate_is_usable(candidate)) {
      return make_sample(candidate, true, received_monotonic_s);
    }
  }

  // The topic is already scoped to one model, so the first usable entry is the
  // vehicle base. The fallback stays visible through `exact_name_match`.
  for (const auto & candidate : candidates) {
    if (candidate_is_usable(candidate)) {
      return make_sample(candidate, false, received_monotonic_s);
    }
  }

  return sample;
}

double world_pose_age_s(const WorldPoseSample & sample, const double now_monotonic_s)
{
  if (!sample.valid || !std::isfinite(now_monotonic_s) ||
    !std::isfinite(sample.received_monotonic_s))
  {
    return std::numeric_limits<double>::infinity();
  }
  return now_monotonic_s - sample.received_monotonic_s;
}

bool world_pose_is_fresh(
  const WorldPoseSample & sample,
  const double now_monotonic_s,
  const double timeout_s)
{
  if (!std::isfinite(timeout_s) || timeout_s <= 0.0) {
    return false;
  }
  const double age_s = world_pose_age_s(sample, now_monotonic_s);
  return std::isfinite(age_s) && age_s >= 0.0 && age_s <= timeout_s;
}

}  // namespace kau_object_detection
