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
#include "kau_object_detection_lane/object_list.hpp"
#include "kau_object_detection_lane/obstacle_markers.hpp"
#include "kau_object_detection_lane/object_transform.hpp"

namespace
{

using kau_object_detection_lane::ObjectListOptions;
using kau_object_detection_lane::ObjectListStatus;
using kau_object_detection_lane::ObstacleMarkerOptions;
using kau_object_detection_lane::CircleCandidate;
using kau_object_detection_lane::build_object_list;
using kau_object_detection_lane::build_obstacle_markers;

using Marker = visualization_msgs::msg::Marker;
using ObstacleCircleArray = kau_msgs::msg::ObstacleCircleArray;

builtin_interfaces::msg::Time scan_stamp()
{
  builtin_interfaces::msg::Time stamp;
  stamp.sec = 1234;
  stamp.nanosec = 567000000U;
  return stamp;
}

CircleCandidate placed_candidate(
  const double x_m, const double y_m, const double radius_m)
{
  CircleCandidate candidate;
  candidate.base_center_valid = true;
  candidate.base_x_m = x_m;
  candidate.base_y_m = y_m;
  candidate.radius_m = radius_m;
  return candidate;
}

/// One `STATUS_OK` frame carrying the given circles verbatim, built the way the
/// node builds the frame it publishes.
ObstacleCircleArray ok_object_list(const std::vector<CircleCandidate> & candidates)
{
  ObjectListOptions options;
  options.frame_id = "base_link";
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
  options.frame_id = "base_link";
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
    EXPECT_EQ(frame.markers.markers[index].header.frame_id, "base_link");
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
  object_list.header.frame_id = "base_link";
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

TEST(ObstacleMarkersTest, MarkerBuildLeavesTheObjectListUntouched)
{
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
}

/// The original package proved that a marker build cannot perturb tracker or
/// EMA state. Here the stronger statement holds: there is no such state to
/// perturb, and the marker builder is a pure function of its two arguments.
TEST(ObstacleMarkersTest, MarkerBuildIsAPureFunctionOfItsArguments)
{
  const auto object_list = ok_object_list(
    {placed_candidate(1.0, 1.0, 0.09), placed_candidate(2.0, 2.0, 0.09)});

  const auto first = build_obstacle_markers(object_list, 0U, ObstacleMarkerOptions{});
  const auto second = build_obstacle_markers(object_list, 0U, ObstacleMarkerOptions{});

  ASSERT_EQ(first.add_marker_count, second.add_marker_count);
  ASSERT_EQ(first.markers.markers.size(), second.markers.markers.size());
  for (std::size_t index = 0U; index < first.markers.markers.size(); ++index) {
    EXPECT_EQ(first.markers.markers[index].id, second.markers.markers[index].id);
    EXPECT_EQ(first.markers.markers[index].action, second.markers.markers[index].action);
    EXPECT_DOUBLE_EQ(
      first.markers.markers[index].pose.position.x,
      second.markers.markers[index].pose.position.x);
    EXPECT_DOUBLE_EQ(
      first.markers.markers[index].pose.position.y,
      second.markers.markers[index].pose.position.y);
    EXPECT_DOUBLE_EQ(
      first.markers.markers[index].scale.x, second.markers.markers[index].scale.x);
  }
}
