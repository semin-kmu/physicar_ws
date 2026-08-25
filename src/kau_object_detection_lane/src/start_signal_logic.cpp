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
// Forked from package "kau_object_detection", its source start_signal_logic.cpp,
// on 2026-08-25.
//
// This is an independent copy, not a link: kau_object_detection_lane builds and
// runs without that package being present. Fixes made in one package do
// NOT propagate to the other. See README.md "Source synchronisation".
//
// Changes from the original: namespace and include guard renamed to
// kau_object_detection_lane, include paths repointed at this package.
//
// 2026-08-25, diverged further: consecutive-green run replaced by a sliding
// window. See the header for the rationale.
// ---------------------------------------------------------------------------

#include "kau_object_detection_lane/start_signal_logic.hpp"

#include <cmath>
#include <stdexcept>

namespace kau_object_detection_lane
{

StartPermissionLatch::StartPermissionLatch(const StartSignalOptions & options)
: options_(options)
{
  if (options_.green_window_frames < 1) {
    throw std::invalid_argument("green_window_frames must be at least 1");
  }
  if (options_.green_required_frames < 1) {
    throw std::invalid_argument("green_required_frames must be at least 1");
  }
  // A threshold above the window can never be reached, which would forbid the
  // start for the whole race with no other symptom. Refusing to start is the
  // only honest response.
  if (options_.green_required_frames > options_.green_window_frames) {
    throw std::invalid_argument(
            "green_required_frames must not exceed green_window_frames");
  }
  if (!std::isfinite(options_.camera_timeout_s) || options_.camera_timeout_s <= 0.0) {
    throw std::invalid_argument("camera_timeout_s must be finite and positive");
  }
}

void StartPermissionLatch::refresh_camera_activity(const double now_s)
{
  has_usable_frame_ = true;
  last_usable_frame_s_ = now_s;
  camera_active_ = true;
}

void StartPermissionLatch::clear_window()
{
  window_.clear();
  green_hits_ = 0;
}

void StartPermissionLatch::push_sample(const bool green)
{
  // Evict first, so the window never exceeds its configured size and the hit
  // count always describes exactly the samples still inside it.
  if (window_.size() >= static_cast<std::size_t>(options_.green_window_frames)) {
    if (window_.front()) {
      --green_hits_;
    }
    window_.pop_front();
  }

  window_.push_back(green);
  if (green) {
    ++green_hits_;
  }

  // The window does not have to be full: the threshold is a count of greens,
  // not a ratio, so four greens as the first four samples latch immediately.
  if (green_hits_ >= options_.green_required_frames) {
    latched_ = true;
  }
}

void StartPermissionLatch::observe_frame(const FrameOutcome outcome, const double now_s)
{
  // Once the start is permitted the decision is final. Frames keep arriving and
  // camera liveness is still tracked for diagnostics, but nothing they say can
  // revoke it, and the window is frozen so the reported hit count stays at the
  // value that latched it.
  if (latched_) {
    if (outcome != FrameOutcome::kUnusable) {
      refresh_camera_activity(now_s);
    }
    return;
  }

  switch (outcome) {
    case FrameOutcome::kGreen:
      refresh_camera_activity(now_s);
      push_sample(true);
      break;

    case FrameOutcome::kNotGreen:
      // Red, yellow, an absent signal and an unrecognised one all land here.
      refresh_camera_activity(now_s);
      push_sample(false);
      break;

    case FrameOutcome::kUnusable:
      // The camera delivered bytes we could not judge. It enters the window as
      // a non-green sample - the conservative choice, since it can push an
      // older green out of the window and so only ever makes latching harder -
      // and deliberately does not count as the camera being alive.
      push_sample(false);
      break;
  }
}

void StartPermissionLatch::advance_time(const double now_s)
{
  if (latched_) {
    return;
  }

  if (!has_usable_frame_) {
    // Nothing has ever arrived, so there is no window to clear and the camera
    // was never active in the first place.
    camera_active_ = false;
    return;
  }

  const double age_s = now_s - last_usable_frame_s_;

  // A clock that moved backwards, which a simulator reset can do, is re-anchored
  // rather than reported as a gap that never happened.
  if (age_s < 0.0) {
    last_usable_frame_s_ = now_s;
    return;
  }

  if (age_s > options_.camera_timeout_s) {
    camera_active_ = false;
    // The whole window goes, not just the count. Greens observed before a long
    // outage say nothing about the signal after it, and letting them combine
    // with greens seen after recovery could permit a start on evidence that is
    // seconds old.
    clear_window();
  }
}

}  // namespace kau_object_detection_lane
