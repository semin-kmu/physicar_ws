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

#include "kau_object_detection/start_signal_logic.hpp"

#include <cmath>
#include <stdexcept>

namespace kau_object_detection
{

StartPermissionLatch::StartPermissionLatch(const StartSignalOptions & options)
: options_(options)
{
  if (options_.green_confirmation_frames < 1) {
    throw std::invalid_argument("green_confirmation_frames must be at least 1");
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

void StartPermissionLatch::observe_frame(const FrameOutcome outcome, const double now_s)
{
  // Once the start is permitted the decision is final. Frames keep arriving and
  // are still tracked for diagnostics, but nothing they say can revoke it.
  if (latched_) {
    if (outcome != FrameOutcome::kUnusable) {
      refresh_camera_activity(now_s);
    }
    return;
  }

  switch (outcome) {
    case FrameOutcome::kGreen:
      refresh_camera_activity(now_s);
      ++consecutive_green_frames_;
      if (consecutive_green_frames_ >= options_.green_confirmation_frames) {
        latched_ = true;
      }
      break;

    case FrameOutcome::kNotGreen:
      refresh_camera_activity(now_s);
      consecutive_green_frames_ = 0;
      break;

    case FrameOutcome::kUnusable:
      // The camera delivered bytes we could not judge. That breaks the run and
      // deliberately does not count as the camera being alive.
      consecutive_green_frames_ = 0;
      break;
  }
}

void StartPermissionLatch::advance_time(const double now_s)
{
  if (latched_) {
    return;
  }

  if (!has_usable_frame_) {
    // Nothing has ever arrived, so there is no run to clear and the camera was
    // never active in the first place.
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
    consecutive_green_frames_ = 0;
  }
}

}  // namespace kau_object_detection
