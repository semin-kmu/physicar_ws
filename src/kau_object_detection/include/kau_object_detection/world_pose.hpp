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

#ifndef KAU_OBJECT_DETECTION__WORLD_POSE_HPP_
#define KAU_OBJECT_DETECTION__WORLD_POSE_HPP_

#include <string>
#include <vector>

namespace kau_object_detection
{

/// One pose entry taken from a simulator pose message, reduced to the fields
/// this package needs. Keeping this free of any Gazebo type lets the selection
/// and freshness logic be unit tested without a running simulator.
struct WorldPoseCandidate
{
  std::string name;
  std::string child_frame_id;
  double x_m{0.0};
  double y_m{0.0};
  double orientation_w{1.0};
  double orientation_x{0.0};
  double orientation_y{0.0};
  double orientation_z{0.0};
  /// Simulation timestamp carried by the message, in seconds.
  double stamp_s{0.0};
};

/// Latest known world pose of the vehicle base.
///
/// This is a read-only diagnostic sample. Nothing in this package transforms
/// perception output with it; the absolute output frame is still a team
/// interface decision.
struct WorldPoseSample
{
  bool valid{false};
  /// True when a candidate matched the configured base frame or model name.
  /// False means the first entry of the message was used as a fallback.
  bool exact_name_match{false};
  std::string source_name;

  double x_m{0.0};
  double y_m{0.0};
  double yaw_rad{0.0};

  /// Simulation clock timestamp of the message.
  double stamp_s{0.0};
  /// Local monotonic receipt time. Freshness is measured against this, never
  /// against `stamp_s`, because the simulation clock and the local monotonic
  /// clock have no common epoch.
  double received_monotonic_s{0.0};
};

/// Yaw in radians from a quaternion. Returns 0 for a degenerate quaternion;
/// callers that need validity should use `select_world_base_pose`.
double quaternion_to_yaw_rad(
  double orientation_w,
  double orientation_x,
  double orientation_y,
  double orientation_z);

double radians_to_degrees(double radians);

/// Picks the vehicle base pose out of a pose message.
///
/// A candidate whose `child_frame_id` or `name` equals `base_frame` wins. When
/// `base_frame` is empty, or nothing matches, the first usable entry is taken
/// and `exact_name_match` is cleared so the fallback stays visible in the logs.
WorldPoseSample select_world_base_pose(
  const std::vector<WorldPoseCandidate> & candidates,
  const std::string & base_frame,
  double received_monotonic_s);

/// Seconds since the sample was received, or infinity when there is no sample.
double world_pose_age_s(const WorldPoseSample & sample, double now_monotonic_s);

bool world_pose_is_fresh(
  const WorldPoseSample & sample,
  double now_monotonic_s,
  double timeout_s);

}  // namespace kau_object_detection

#endif  // KAU_OBJECT_DETECTION__WORLD_POSE_HPP_
