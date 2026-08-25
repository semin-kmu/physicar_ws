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
// Forked from package "kau_object_detection", its header start_signal_logic.hpp, on 2026-08-25.
//
// This is an independent copy, not a link: kau_object_detection_lane builds and
// runs without that package being present. Fixes made in one package do
// NOT propagate to the other. See README.md "Source synchronisation".
//
// Changes from the original: namespace and include guard renamed to
// kau_object_detection_lane, include paths repointed at this package.
//
// 2026-08-25, diverged further from the original: the consecutive-green run was
// replaced by a sliding window. The original demanded
// `green_confirmation_frames` greens in a row, so a single dropped or
// mis-classified frame sent the count back to zero and the vehicle sometimes
// never left the line. This class now latches on `green_required_frames` greens
// within the last `green_window_frames` usable frames, which tolerates the odd
// miss without weakening the latch.
//
// Only the temporal decision changed. The spatial detection - ROI, HSV gates,
// area, aspect and fill - lives in green_lamp_detector and was not touched, so
// the effect of this change can be attributed to the window alone.
// ---------------------------------------------------------------------------

#ifndef KAU_OBJECT_DETECTION_LANE__START_SIGNAL_LOGIC_HPP_
#define KAU_OBJECT_DETECTION_LANE__START_SIGNAL_LOGIC_HPP_

#include <cstdint>
#include <deque>

namespace kau_object_detection_lane
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
  /// payload failed to decode. It enters the window as a non-green sample and,
  /// unlike the two outcomes above, does not count as the camera being alive.
  kUnusable,
};

/// Tunables of the start permission decision. All three come from YAML.
struct StartSignalOptions
{
  /// How many of the most recent samples the decision looks at. At the roughly
  /// 15 Hz camera rate the default spans about 0.53 s.
  int green_window_frames{8};

  /// Green samples required inside that window before the start is permitted.
  /// Must be at least 1 and at most `green_window_frames`.
  int green_required_frames{4};

  /// A gap this long without a usable frame clears the window and keeps the
  /// permission false. It has no effect once the permission has latched.
  double camera_timeout_s{0.5};
};

/// Start permission state machine.
///
/// The permission starts false, turns true once `green_required_frames` of the
/// last `green_window_frames` samples were green, and then latches: nothing
/// afterwards can take it back, not a red lamp, not a disappearing signal, not
/// a dead camera. The class holds no ROS or OpenCV types so the whole policy is
/// testable on its own.
///
/// The window does not have to be full. Four greens as the first four samples
/// latch immediately; the window only ever bounds how far back a green stays
/// relevant.
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

  /// Green samples currently inside the window. Frozen once latched.
  int green_hits() const {return green_hits_;}

  /// Samples currently inside the window, at most `green_window_frames`. Tells
  /// an operator whether a low hit count is "not green" or "not enough frames
  /// yet". Frozen once latched.
  int window_samples() const {return static_cast<int>(window_.size());}

  const StartSignalOptions & options() const {return options_;}

private:
  void refresh_camera_activity(double now_s);

  /// Appends one sample, evicting the oldest when the window is full, and
  /// latches when the hit count reaches the threshold. O(1): the count is
  /// maintained incrementally rather than recomputed.
  void push_sample(bool green);

  /// Drops every sample. Used by the camera timeout so greens seen before a
  /// long outage cannot combine with greens seen after it.
  void clear_window();

  StartSignalOptions options_;
  bool latched_{false};
  bool camera_active_{false};
  bool has_usable_frame_{false};
  /// Most recent samples, oldest at the front. Bounded by
  /// `options_.green_window_frames`.
  std::deque<bool> window_;
  /// Number of `true` entries in `window_`, kept in step with every push and
  /// eviction.
  int green_hits_{0};
  double last_usable_frame_s_{0.0};
};

}  // namespace kau_object_detection_lane

#endif  // KAU_OBJECT_DETECTION_LANE__START_SIGNAL_LOGIC_HPP_
