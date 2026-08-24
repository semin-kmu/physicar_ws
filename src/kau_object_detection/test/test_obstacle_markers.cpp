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

#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "gtest/gtest.h"
#include "kau_msgs/msg/obstacle_circle_array.hpp"
#include "kau_object_detection/object_list.hpp"
#include "kau_object_detection/obstacle_markers.hpp"
#include "kau_object_detection/obstacle_tracker.hpp"
#include "kau_object_detection/track_roi.hpp"

namespace
{

using kau_object_detection::ObjectListOptions;
using kau_object_detection::ObjectListStatus;
using kau_object_detection::ObstacleMarkerOptions;
using kau_object_detection::ObstacleMeasurement;
using kau_object_detection::ObstacleTracker;
using kau_object_detection::ObstacleTrackerOptions;
using kau_object_detection::WorldCircleCandidate;
using kau_object_detection::build_object_list;
using kau_object_detection::build_obstacle_markers;

using Marker = visualization_msgs::msg::Marker;
using ObstacleCircleArray = kau_msgs::msg::ObstacleCircleArray;

builtin_interfaces::msg::Time scan_stamp()
{
  builtin_interfaces::msg::Time stamp;
  stamp.sec = 1234;
  stamp.nanosec = 567000000U;
  return stamp;
}

WorldCircleCandidate placed_candidate(
  const double x_m, const double y_m, const double radius_m)
{
  WorldCircleCandidate candidate;
  candidate.world_center_valid = true;
  candidate.world_x_m = x_m;
  candidate.world_y_m = y_m;
  candidate.radius_m = radius_m;
  candidate.inside_track_ring = true;
  return candidate;
}

/// One `STATUS_OK` frame carrying the given circles verbatim, built the way the
/// node builds the frame it publishes.
ObstacleCircleArray ok_object_list(const std::vector<WorldCircleCandidate> & candidates)
{
  ObjectListOptions options;
  options.frame_id = "map";
  return build_object_list(ObjectListStatus::kOk, candidates, scan_stamp(), options);
}

}  // namespace

TEST(ObstacleMarkersTest, TwoCirclesBecomeTwoAddMarkers)
{
  const auto object_list = ok_object_list(
    {placed_candidate(1.25, -0.75, 0.09), placed_candidate(-2.0, 3.5, 0.12)});
  ASSERT_EQ(object_list.obstacles.size(), 2U);

  const auto frame = build_obstacle_markers(object_list, 0U, ObstacleMarkerOptions{});

  EXPECT_EQ(frame.add_marker_count, 2U);
  ASSERT_EQ(frame.markers.markers.size(), 2U);

  for (std::size_t index = 0U; index < frame.markers.markers.size(); ++index) {
    const auto & marker = frame.markers.markers[index];
    const auto & circle = object_list.obstacles[index];

    EXPECT_EQ(marker.action, Marker::ADD);
    EXPECT_EQ(marker.type, Marker::CYLINDER);
    EXPECT_EQ(marker.ns, "obstacle_circles");
    // Contiguous ids from zero, so the delete range of the next frame is exact.
    EXPECT_EQ(marker.id, static_cast<std::int32_t>(index));

    EXPECT_EQ(marker.header.frame_id, object_list.header.frame_id);
    EXPECT_EQ(marker.header.stamp.sec, object_list.header.stamp.sec);
    EXPECT_EQ(marker.header.stamp.nanosec, object_list.header.stamp.nanosec);

    EXPECT_DOUBLE_EQ(marker.pose.position.x, static_cast<double>(circle.center_x));
    EXPECT_DOUBLE_EQ(marker.pose.position.y, static_cast<double>(circle.center_y));
    EXPECT_DOUBLE_EQ(marker.pose.position.z, 0.03);

    // Identity orientation: a flat disc has no meaningful heading.
    EXPECT_DOUBLE_EQ(marker.pose.orientation.x, 0.0);
    EXPECT_DOUBLE_EQ(marker.pose.orientation.y, 0.0);
    EXPECT_DOUBLE_EQ(marker.pose.orientation.z, 0.0);
    EXPECT_DOUBLE_EQ(marker.pose.orientation.w, 1.0);

    // Diameter, not radius.
    EXPECT_DOUBLE_EQ(marker.scale.x, 2.0 * static_cast<double>(circle.radius));
    EXPECT_DOUBLE_EQ(marker.scale.y, 2.0 * static_cast<double>(circle.radius));
    EXPECT_DOUBLE_EQ(marker.scale.z, 0.06);

    EXPECT_GE(marker.color.a, 0.8F);
    EXPECT_GT(marker.color.r, marker.color.g);
    EXPECT_GT(marker.color.r, marker.color.b);

    // 0.3 s safety net on every drawn circle.
    EXPECT_EQ(marker.lifetime.sec, 0);
    EXPECT_EQ(marker.lifetime.nanosec, 300000000U);
  }
}

TEST(ObstacleMarkersTest, ShrinkingObstacleCountDeletesTheLeftoverId)
{
  const auto first_frame = build_obstacle_markers(
    ok_object_list({placed_candidate(1.0, 1.0, 0.09), placed_candidate(2.0, 2.0, 0.09)}),
    0U, ObstacleMarkerOptions{});
  ASSERT_EQ(first_frame.add_marker_count, 2U);

  const auto second_frame = build_obstacle_markers(
    ok_object_list({placed_candidate(1.0, 1.0, 0.09)}),
    first_frame.add_marker_count, ObstacleMarkerOptions{});

  EXPECT_EQ(second_frame.add_marker_count, 1U);
  ASSERT_EQ(second_frame.markers.markers.size(), 2U);
  EXPECT_EQ(second_frame.markers.markers[0].action, Marker::ADD);
  EXPECT_EQ(second_frame.markers.markers[0].id, 0);
  // The id the previous frame drew and this one no longer fills.
  EXPECT_EQ(second_frame.markers.markers[1].action, Marker::DELETE);
  EXPECT_EQ(second_frame.markers.markers[1].id, 1);
  EXPECT_EQ(second_frame.markers.markers[1].ns, "obstacle_circles");
}

TEST(ObstacleMarkersTest, NonOkStatusDeletesEveryPreviousMarker)
{
  ObjectListOptions options;
  options.frame_id = "map";
  // A non-OK frame carries no obstacle by contract; the markers must go too.
  const auto failed_frame_message = build_object_list(
    ObjectListStatus::kTransformUnavailable,
    {placed_candidate(1.0, 1.0, 0.09)}, scan_stamp(), options);
  ASSERT_TRUE(failed_frame_message.obstacles.empty());

  const auto frame = build_obstacle_markers(failed_frame_message, 3U, ObstacleMarkerOptions{});

  EXPECT_EQ(frame.add_marker_count, 0U);
  ASSERT_EQ(frame.markers.markers.size(), 3U);
  for (std::size_t index = 0U; index < frame.markers.markers.size(); ++index) {
    EXPECT_EQ(frame.markers.markers[index].action, Marker::DELETE);
    EXPECT_EQ(frame.markers.markers[index].id, static_cast<std::int32_t>(index));
    EXPECT_EQ(frame.markers.markers[index].header.frame_id, "map");
  }
}

TEST(ObstacleMarkersTest, EmptyOkListDeletesEveryPreviousMarker)
{
  const auto frame = build_obstacle_markers(ok_object_list({}), 2U, ObstacleMarkerOptions{});

  EXPECT_EQ(frame.add_marker_count, 0U);
  ASSERT_EQ(frame.markers.markers.size(), 2U);
  EXPECT_EQ(frame.markers.markers[0].action, Marker::DELETE);
  EXPECT_EQ(frame.markers.markers[1].action, Marker::DELETE);
}

TEST(ObstacleMarkersTest, SteadyEmptyStreamProducesNothingToPublish)
{
  const auto frame = build_obstacle_markers(ok_object_list({}), 0U, ObstacleMarkerOptions{});

  EXPECT_EQ(frame.add_marker_count, 0U);
  EXPECT_TRUE(frame.markers.markers.empty());
}

TEST(ObstacleMarkersTest, DegenerateCirclesAreNotDrawn)
{
  ObstacleCircleArray object_list;
  object_list.header.frame_id = "map";
  object_list.header.stamp = scan_stamp();
  object_list.status = ObstacleCircleArray::STATUS_OK;

  const auto degenerate_circle =
    [](const float x, const float y, const float radius) {
      kau_msgs::msg::ObstacleCircle circle;
      circle.center_x = x;
      circle.center_y = y;
      circle.radius = radius;
      circle.confidence = 1.0F;
      return circle;
    };

  const float not_a_number = std::numeric_limits<float>::quiet_NaN();
  const float infinity = std::numeric_limits<float>::infinity();
  object_list.obstacles.push_back(degenerate_circle(1.0F, 1.0F, 0.0F));
  object_list.obstacles.push_back(degenerate_circle(1.0F, 1.0F, -0.09F));
  object_list.obstacles.push_back(degenerate_circle(not_a_number, 1.0F, 0.09F));
  object_list.obstacles.push_back(degenerate_circle(1.0F, infinity, 0.09F));
  object_list.obstacles.push_back(degenerate_circle(1.0F, 1.0F, not_a_number));

  const auto frame = build_obstacle_markers(object_list, 0U, ObstacleMarkerOptions{});

  EXPECT_EQ(frame.add_marker_count, 0U);
  EXPECT_TRUE(frame.markers.markers.empty());
}

TEST(ObstacleMarkersTest, DrawableCirclesKeepContiguousIdsAroundADroppedOne)
{
  ObstacleCircleArray object_list = ok_object_list(
    {placed_candidate(1.0, 1.0, 0.09), placed_candidate(2.0, 2.0, 0.09)});
  ASSERT_EQ(object_list.obstacles.size(), 2U);

  kau_msgs::msg::ObstacleCircle degenerate;
  degenerate.center_x = 3.0F;
  degenerate.center_y = 3.0F;
  degenerate.radius = 0.0F;
  degenerate.confidence = 1.0F;
  object_list.obstacles.insert(object_list.obstacles.begin() + 1, degenerate);

  const auto frame = build_obstacle_markers(object_list, 0U, ObstacleMarkerOptions{});

  EXPECT_EQ(frame.add_marker_count, 2U);
  ASSERT_EQ(frame.markers.markers.size(), 2U);
  EXPECT_EQ(frame.markers.markers[0].id, 0);
  EXPECT_EQ(frame.markers.markers[1].id, 1);
  EXPECT_DOUBLE_EQ(frame.markers.markers[1].pose.position.x, 2.0);
}

TEST(ObstacleMarkersTest, InvalidLifetimeLeavesTheMarkerWithoutAnExpiry)
{
  // The node refuses to start on such a value; the builder still must not turn
  // it into a nonsensical expiry.
  ObstacleMarkerOptions options;
  options.lifetime_s = -1.0;

  const auto frame = build_obstacle_markers(
    ok_object_list({placed_candidate(1.0, 1.0, 0.09)}), 0U, options);

  ASSERT_EQ(frame.markers.markers.size(), 1U);
  EXPECT_EQ(frame.markers.markers[0].lifetime.sec, 0);
  EXPECT_EQ(frame.markers.markers[0].lifetime.nanosec, 0U);
}

TEST(ObstacleMarkersTest, MarkerBuildLeavesObjectListTrackerAndSmoothingUntouched)
{
  const std::vector<ObstacleMeasurement> measurements{
    ObstacleMeasurement{1.0, 1.0, 0.09},
    ObstacleMeasurement{2.0, 2.0, 0.09},
  };

  ObstacleTrackerOptions tracker_options;
  tracker_options.minimum_confirmation_frames = 1U;

  ObstacleTracker control_tracker{tracker_options};
  ObstacleTracker observed_tracker{tracker_options};
  control_tracker.update(measurements, 0.0);
  observed_tracker.update(measurements, 0.0);

  const auto object_list = ok_object_list(
    {placed_candidate(1.0, 1.0, 0.09), placed_candidate(2.0, 2.0, 0.09)});
  const auto object_list_before_markers = object_list;

  const auto frame = build_obstacle_markers(object_list, 0U, ObstacleMarkerOptions{});
  ASSERT_EQ(frame.add_marker_count, 2U);

  // The published message is untouched: same status, header and circles.
  EXPECT_EQ(object_list.status, object_list_before_markers.status);
  EXPECT_EQ(object_list.header.frame_id, object_list_before_markers.header.frame_id);
  EXPECT_EQ(object_list.header.stamp.sec, object_list_before_markers.header.stamp.sec);
  EXPECT_EQ(object_list.header.stamp.nanosec, object_list_before_markers.header.stamp.nanosec);
  ASSERT_EQ(object_list.obstacles.size(), object_list_before_markers.obstacles.size());
  for (std::size_t index = 0U; index < object_list.obstacles.size(); ++index) {
    EXPECT_FLOAT_EQ(
      object_list.obstacles[index].center_x, object_list_before_markers.obstacles[index].center_x);
    EXPECT_FLOAT_EQ(
      object_list.obstacles[index].center_y, object_list_before_markers.obstacles[index].center_y);
    EXPECT_FLOAT_EQ(
      object_list.obstacles[index].radius, object_list_before_markers.obstacles[index].radius);
    EXPECT_FLOAT_EQ(
      object_list.obstacles[index].confidence,
      object_list_before_markers.obstacles[index].confidence);
  }

  // The tracker that saw a marker build in between still smooths identically:
  // the visualisation path holds no reference to tracker or EMA state.
  const std::vector<ObstacleMeasurement> next_measurements{
    ObstacleMeasurement{1.1, 1.05, 0.09},
    ObstacleMeasurement{2.05, 2.1, 0.09},
  };
  const auto control_update = control_tracker.update(next_measurements, 0.1);
  const auto observed_update = observed_tracker.update(next_measurements, 0.1);

  ASSERT_EQ(observed_update.obstacles.size(), control_update.obstacles.size());
  EXPECT_EQ(observed_tracker.active_track_count(), control_tracker.active_track_count());
  for (std::size_t index = 0U; index < control_update.obstacles.size(); ++index) {
    EXPECT_DOUBLE_EQ(observed_update.obstacles[index].x_m, control_update.obstacles[index].x_m);
    EXPECT_DOUBLE_EQ(observed_update.obstacles[index].y_m, control_update.obstacles[index].y_m);
    EXPECT_EQ(
      observed_update.obstacles[index].track_id, control_update.obstacles[index].track_id);
  }
}
