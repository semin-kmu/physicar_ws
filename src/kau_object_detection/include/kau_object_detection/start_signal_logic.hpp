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

#ifndef KAU_OBJECT_DETECTION__START_SIGNAL_LOGIC_HPP_
#define KAU_OBJECT_DETECTION__START_SIGNAL_LOGIC_HPP_

#include <cstdint>

namespace kau_object_detection
{

/// What one camera frame told us about the start signal.
enum class FrameOutcome : std::uint8_t
{
  /// The lamp was found inside the region of interest.
  kGreen,
  /// The frame was readable and no green lamp was in the region of interest.
  /// Red, yellow, an absent signal and an unrecognised one all land here.
  kNotGreen,
  /// The frame arrived but could not be judged, for instance because the JPEG
  /// payload failed to decode. It breaks the consecutive-green run and, unlike
  /// the two outcomes above, does not count as the camera being alive.
  kUnusable,
};

/// Tunables of the start permission decision. Both come from YAML.
struct StartSignalOptions
{
  /// Consecutive green frames required before the start is permitted. At the
  /// roughly 15 Hz camera rate the default spans about 0.33 s.
  int green_confirmation_frames{5};

  /// A gap this long without a usable frame clears the run and keeps the
  /// permission false. It has no effect once the permission has latched.
  double camera_timeout_s{0.5};
};

/// Start permission state machine.
///
/// The permission starts false, turns true only after `green_confirmation_frames`
/// consecutive green frames, and then latches: nothing afterwards can take it
/// back, not a red lamp, not a disappearing signal, not a dead camera. The class
/// holds no ROS or OpenCV types so the whole policy is testable on its own.
///
/// Time is supplied by the caller in seconds and only ever compared against
/// itself, so any monotonic source works. The node passes its ROS clock, which
/// makes the timeout follow simulation time.
class StartPermissionLatch
{
public:
  /// Throws std::invalid_argument when the options cannot describe a usable
  /// policy, so a misconfigured node fails at start-up instead of silently
  /// permitting or forbidding the start.
  explicit StartPermissionLatch(const StartSignalOptions & options);

  /// Records one camera frame observed at `now_s`.
  void observe_frame(FrameOutcome outcome, double now_s);

  /// Advances time without a new frame. The publish timer calls this so a
  /// camera that stops delivering is noticed even though no callback runs.
  void advance_time(double now_s);

  /// The value published on the start permission topic.
  bool start_permitted() const {return latched_;}

  bool latched() const {return latched_;}

  /// False before the first usable frame and after the camera timeout expires.
  /// Reported for diagnostics only; the permission never depends on it once
  /// latched.
  bool camera_active() const {return camera_active_;}

  /// Length of the current consecutive-green run. Frozen once latched.
  int consecutive_green_frames() const {return consecutive_green_frames_;}

  const StartSignalOptions & options() const {return options_;}

private:
  void refresh_camera_activity(double now_s);

  StartSignalOptions options_;
  bool latched_{false};
  bool camera_active_{false};
  bool has_usable_frame_{false};
  int consecutive_green_frames_{0};
  double last_usable_frame_s_{0.0};
};

}  // namespace kau_object_detection

#endif  // KAU_OBJECT_DETECTION__START_SIGNAL_LOGIC_HPP_
