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
// Forked from package "kau_object_detection", its header obstacle_markers.hpp, on 2026-08-25.
//
// This is an independent copy, not a link: kau_object_detection_lane builds and
// runs without that package being present. Fixes made in one package do
// NOT propagate to the other. See README.md "Source synchronisation".
//
// Changes from the original: namespace and include guard renamed to
// kau_object_detection_lane, include paths repointed at this package.
// ---------------------------------------------------------------------------

#ifndef KAU_OBJECT_DETECTION_LANE__OBSTACLE_MARKERS_HPP_
#define KAU_OBJECT_DETECTION_LANE__OBSTACLE_MARKERS_HPP_

#include <cstddef>
#include <string>

#include "kau_msgs/msg/obstacle_circle_array.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace kau_object_detection_lane
{

/// Fixed parts of the RViz visualisation of one Object List frame.
///
/// Visualisation only. Nothing in this struct is part of the Object List
/// contract published on `object_list_topic`.
struct ObstacleMarkerOptions
{
  /// Marker namespace. One namespace for the whole stream, so an RViz display
  /// filter and the DELETE bookkeeping below both address the same set.
  std::string ns{"obstacle_circles"};

  /// Seconds an ADD marker survives in RViz without being refreshed. The
  /// explicit DELETE markers are the primary cleanup; this is the safety net
  /// for the case where the node stops publishing altogether.
  ///
  /// Must be finite and positive. A value that is not is ignored and leaves the
  /// marker without an expiry, which is why the node validates the parameter
  /// and refuses to start on a bad one.
  double lifetime_s{0.3};

  /// Height of the cylinder and of its centre above the ground plane. The disc
  /// is drawn flat and just off the floor so it never z-fights with a map or
  /// ground grid.
  double height_m{0.06};
  double center_z_m{0.03};

  /// Orange-red fill. Alpha stays well above the 0.8 minimum so the circle
  /// reads as a solid obstacle while the map underneath stays visible.
  float color_r{1.0F};
  float color_g{0.35F};
  float color_b{0.0F};
  float color_a{0.85F};
};

/// One frame of obstacle markers plus the bookkeeping the caller needs.
///
/// `add_marker_count` is the number of ADD markers in `markers`, which is what
/// the caller stores and passes back as `previous_marker_count` on the next
/// frame. It is not the same as `object_list.obstacles.size()`: a circle with a
/// non-positive radius or a non-finite coordinate is skipped.
struct ObstacleMarkerFrame
{
  visualization_msgs::msg::MarkerArray markers;
  std::size_t add_marker_count{0U};
};

/// Builds the RViz markers of one already-built Object List frame.
///
/// Visualisation only, and deliberately driven by the very message that is
/// published on `/perception/obstacles`: what the planner receives and what
/// RViz draws can therefore never disagree.
///
/// Every accepted circle becomes one CYLINDER marker with a diameter of
/// `2 * radius`, at `(center_x, center_y)` of `object_list.header.frame_id`,
/// carrying `object_list.header.stamp`. Marker ids run from 0 upwards without
/// gaps, so the ids of one frame are always `[0, add_marker_count)`.
///
/// A circle is skipped when its radius is not positive or when any of its
/// centre coordinates or its radius is not finite: drawing a degenerate circle
/// would misreport the perception result rather than visualise it. A frame
/// whose status is not `STATUS_OK` yields no ADD marker at all, mirroring the
/// rule that a non-OK Object List carries no usable obstacle.
///
/// `previous_marker_count` is the `add_marker_count` of the previous frame.
/// Every id in `[add_marker_count, previous_marker_count)` gets a DELETE
/// marker, so a shrinking obstacle count, an empty list and a non-OK status all
/// clear the circles RViz still shows. The function is stateless: it neither
/// reads nor writes tracker or smoothing state, and it takes the Object List by
/// const reference, so it cannot perturb the message being published.
ObstacleMarkerFrame build_obstacle_markers(
  const kau_msgs::msg::ObstacleCircleArray & object_list,
  std::size_t previous_marker_count,
  const ObstacleMarkerOptions & options = ObstacleMarkerOptions{});

}  // namespace kau_object_detection_lane

#endif  // KAU_OBJECT_DETECTION_LANE__OBSTACLE_MARKERS_HPP_
