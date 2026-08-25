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
// Forked from package "kau_object_detection", its source obstacle_markers.cpp, on 2026-08-25.
//
// This is an independent copy, not a link: kau_object_detection_lane builds and
// runs without that package being present. Fixes made in one package do
// NOT propagate to the other. See README.md "Source synchronisation".
//
// Changes from the original: namespace and include guard renamed to
// kau_object_detection_lane, include paths repointed at this package.
// ---------------------------------------------------------------------------

#include "kau_object_detection_lane/obstacle_markers.hpp"

#include <cmath>
#include <cstdint>

namespace kau_object_detection_lane
{
namespace
{

/// True when a circle can be drawn as it was measured. A non-positive radius or
/// a non-finite value has no place on the map: it would either collapse the
/// cylinder or push it somewhere the obstacle never was.
bool circle_is_drawable(const kau_msgs::msg::ObstacleCircle & circle)
{
  if (!std::isfinite(circle.center_x) || !std::isfinite(circle.center_y) ||
    !std::isfinite(circle.radius))
  {
    return false;
  }
  return circle.radius > 0.0F;
}

/// Fills the marker lifetime from a duration in seconds.
///
/// A value that is not finite and positive is left at the default zero, which
/// RViz reads as "never expires". The node validates the parameter up front, so
/// this is the defensive branch rather than the expected one.
void set_lifetime(visualization_msgs::msg::Marker & marker, const double lifetime_s)
{
  if (!std::isfinite(lifetime_s) || lifetime_s <= 0.0) {
    return;
  }

  const double whole_seconds = std::floor(lifetime_s);
  marker.lifetime.sec = static_cast<std::int32_t>(whole_seconds);
  marker.lifetime.nanosec =
    static_cast<std::uint32_t>((lifetime_s - whole_seconds) * 1e9);
}

/// Header and namespace every marker of one frame shares, ADD and DELETE alike.
/// A DELETE is addressed by namespace and id, so it has to carry the same pair
/// the ADD it removes was published with.
visualization_msgs::msg::Marker make_base_marker(
  const kau_msgs::msg::ObstacleCircleArray & object_list,
  const ObstacleMarkerOptions & options,
  const std::size_t id)
{
  visualization_msgs::msg::Marker marker;
  marker.header = object_list.header;
  marker.ns = options.ns;
  marker.id = static_cast<std::int32_t>(id);
  return marker;
}

}  // namespace

ObstacleMarkerFrame build_obstacle_markers(
  const kau_msgs::msg::ObstacleCircleArray & object_list,
  const std::size_t previous_marker_count,
  const ObstacleMarkerOptions & options)
{
  ObstacleMarkerFrame frame;

  // A non-OK frame carries no usable obstacle by contract, so it draws nothing
  // and only clears what an earlier frame left behind.
  const bool status_ok =
    object_list.status == kau_msgs::msg::ObstacleCircleArray::STATUS_OK;

  if (status_ok) {
    frame.markers.markers.reserve(object_list.obstacles.size());
    for (const auto & circle : object_list.obstacles) {
      if (!circle_is_drawable(circle)) {
        continue;
      }

      auto marker = make_base_marker(object_list, options, frame.add_marker_count);
      marker.type = visualization_msgs::msg::Marker::CYLINDER;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.position.x = static_cast<double>(circle.center_x);
      marker.pose.position.y = static_cast<double>(circle.center_y);
      marker.pose.position.z = options.center_z_m;
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;
      // A CYLINDER is scaled by its diameter, not by its radius.
      marker.scale.x = 2.0 * static_cast<double>(circle.radius);
      marker.scale.y = 2.0 * static_cast<double>(circle.radius);
      marker.scale.z = options.height_m;
      marker.color.r = options.color_r;
      marker.color.g = options.color_g;
      marker.color.b = options.color_b;
      marker.color.a = options.color_a;
      set_lifetime(marker, options.lifetime_s);

      frame.markers.markers.push_back(marker);
      ++frame.add_marker_count;
    }
  }

  // Ids the previous frame drew and this one does not reach. Without these the
  // circles of a larger previous frame would stay on screen for the whole
  // lifetime window, which is exactly the stale-obstacle picture the planner
  // never sees.
  for (std::size_t id = frame.add_marker_count; id < previous_marker_count; ++id) {
    auto marker = make_base_marker(object_list, options, id);
    marker.action = visualization_msgs::msg::Marker::DELETE;
    frame.markers.markers.push_back(marker);
  }

  return frame;
}

}  // namespace kau_object_detection_lane
