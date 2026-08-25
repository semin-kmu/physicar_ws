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
// The defining property of this package: every LaserScan is judged on its own.
//
// New in kau_object_detection_lane; there is no counterpart in the source
// package, which deliberately does keep tracker and EMA state.
//
// These tests drive the whole detection chain the node runs - validate,
// speckle filter, polar/Cartesian, cluster, geometry, occupancy gate, tf2
// placement, message build - and assert that no obstacle, and no smoothing
// state, survives from one scan into the next.
//
// The chain is exercised directly rather than through rclcpp so the test needs
// no ROS graph. The node adds only the subscription, the tf2 lookup and the
// publisher on top of exactly these calls, in exactly this order.
// ---------------------------------------------------------------------------

#include <cmath>
#include <cstddef>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "gtest/gtest.h"
#include "kau_msgs/msg/obstacle_circle_array.hpp"
#include "kau_object_detection_lane/cluster_geometry.hpp"
#include "kau_object_detection_lane/cone_occupancy.hpp"
#include "kau_object_detection_lane/laser_clusterer.hpp"
#include "kau_object_detection_lane/object_list.hpp"
#include "kau_object_detection_lane/object_transform.hpp"
#include "kau_object_detection_lane/scan_validation.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace
{

using kau_object_detection_lane::ClusterCandidateOptions;
using kau_object_detection_lane::ClusterGeometryOptions;
using kau_object_detection_lane::ConeOccupancyOptions;
using kau_object_detection_lane::NeighborFilterOptions;
using kau_object_detection_lane::ObjectListOptions;
using kau_object_detection_lane::ScanClusteringOptions;
using kau_object_detection_lane::ScanValidationOptions;
using kau_object_detection_lane::SensorToBaseTransform;
using kau_object_detection_lane::build_object_list;
using kau_object_detection_lane::cluster_scan_points;
using kau_object_detection_lane::compute_all_cluster_geometry;
using kau_object_detection_lane::compute_all_cone_occupancy_candidates;
using kau_object_detection_lane::convert_scan_indices_to_points;
using kau_object_detection_lane::decide_object_list_status_from_placement;
using kau_object_detection_lane::filter_isolated_scan_points;
using kau_object_detection_lane::make_scan_neighbor_topology;
using kau_object_detection_lane::make_sensor_to_base_transform;
using kau_object_detection_lane::place_candidates_in_base_frame;
using kau_object_detection_lane::select_candidate_clusters;
using kau_object_detection_lane::validate_laser_scan;

using ObstacleCircleArray = kau_msgs::msg::ObstacleCircleArray;

constexpr std::size_t kBeamCount = 360U;

/// The measured static mount: base_link <- lidar_link, pure translation.
SensorToBaseTransform static_mount()
{
  return make_sensor_to_base_transform(-0.027, 0.0, 0.0, 0.0, 0.0, 1.0);
}

SensorToBaseTransform missing_transform()
{
  return SensorToBaseTransform{};
}

ScanValidationOptions validation_options()
{
  ScanValidationOptions options;
  options.expected_frame_id = "lidar_link";
  return options;
}

/// A full-circle scan with everything out of range except a short arc of beams
/// at `range_m`, which reads as one cone-sized cluster.
sensor_msgs::msg::LaserScan scan_with_cone(
  const int stamp_sec, const double range_m, const bool with_cone = true)
{
  sensor_msgs::msg::LaserScan scan;
  scan.header.frame_id = "lidar_link";
  scan.header.stamp.sec = stamp_sec;
  scan.header.stamp.nanosec = 0U;
  scan.angle_min = 0.0F;
  scan.angle_increment = static_cast<float>(2.0 * M_PI / static_cast<double>(kBeamCount));
  scan.angle_max = static_cast<float>(2.0 * M_PI) - scan.angle_increment;
  scan.range_min = 0.05F;
  scan.range_max = 12.0F;
  // Out-of-range beams are dropped by the validator, leaving only the arc.
  scan.ranges.assign(kBeamCount, 50.0F);

  if (with_cone) {
    // Four adjacent beams straight ahead. At 1 m the beam spacing is about
    // 1.7 cm, so they link into one cluster well inside the width gate.
    for (std::size_t index = 0U; index < 4U; ++index) {
      scan.ranges[index] = static_cast<float>(range_m);
    }
  }
  return scan;
}

builtin_interfaces::msg::Time stamp_of(const sensor_msgs::msg::LaserScan & scan)
{
  return scan.header.stamp;
}

/// One full pass of the node's detection chain over one scan.
///
/// Every intermediate value is a local: the function holds no state, exactly
/// as the node's scan callback holds none.
ObstacleCircleArray process_one_scan(
  const sensor_msgs::msg::LaserScan & scan,
  const SensorToBaseTransform & transform)
{
  const ScanClusteringOptions clustering_options;
  const NeighborFilterOptions filter_options;
  const ClusterCandidateOptions candidate_options;
  const ClusterGeometryOptions geometry_options;
  const ConeOccupancyOptions occupancy_options;
  const ObjectListOptions object_list_options;

  const auto validation = validate_laser_scan(scan, validation_options());
  if (!validation.valid) {
    return build_object_list(
      kau_object_detection_lane::ObjectListStatus::kLidarUnavailable, {},
      stamp_of(scan), object_list_options);
  }

  const auto topology = make_scan_neighbor_topology(
    scan, clustering_options.circular_scan_tolerance_rad);
  const auto points = convert_scan_indices_to_points(scan, validation.usable_indices);
  const auto filtered =
    filter_isolated_scan_points(points, topology, clustering_options, filter_options);
  const auto raw_clusters = cluster_scan_points(filtered.points, topology, clustering_options);
  const auto clusters = select_candidate_clusters(raw_clusters, candidate_options);
  const auto geometries = compute_all_cluster_geometry(clusters, geometry_options);
  const auto candidates =
    compute_all_cone_occupancy_candidates(geometries, occupancy_options);

  const auto placement = place_candidates_in_base_frame(candidates, transform);
  const auto status =
    decide_object_list_status_from_placement(true, transform.valid, placement.applied);

  return build_object_list(status, placement.accepted, stamp_of(scan), object_list_options);
}

}  // namespace

/// A cone that has just come into view is published on the very first scan
/// that sees it. There is no confirmation-frame delay to wait out.
TEST(NoTemporalPersistence, FirstScanCandidateIsPublishedImmediately)
{
  const auto message = process_one_scan(scan_with_cone(1, 1.0), static_mount());

  EXPECT_EQ(message.status, ObstacleCircleArray::STATUS_OK);
  ASSERT_FALSE(message.obstacles.empty())
    << "a candidate visible in the first scan must reach the wire on that scan";
  EXPECT_GT(message.obstacles[0].radius, 0.0F);
}

/// The obstacle disappears from the output the moment it disappears from the
/// scan. A tracker with a track timeout would have held it for several frames.
TEST(NoTemporalPersistence, EmptyScanAfterAPopulatedOneCarriesNothingOver)
{
  const auto populated = process_one_scan(scan_with_cone(1, 1.0), static_mount());
  ASSERT_FALSE(populated.obstacles.empty());

  const auto empty =
    process_one_scan(scan_with_cone(2, 1.0, /*with_cone=*/false), static_mount());

  EXPECT_EQ(empty.status, ObstacleCircleArray::STATUS_OK);
  EXPECT_TRUE(empty.obstacles.empty())
    << "an empty scan must publish an empty list, not the previous obstacles";
}

/// A failed transform empties the list on the spot and never republishes the
/// last good frame.
TEST(NoTemporalPersistence, TransformFailureCarriesNothingOver)
{
  const auto populated = process_one_scan(scan_with_cone(1, 1.0), static_mount());
  ASSERT_FALSE(populated.obstacles.empty());

  const auto failed = process_one_scan(scan_with_cone(2, 1.0), missing_transform());

  EXPECT_EQ(failed.status, ObstacleCircleArray::STATUS_TRANSFORM_UNAVAILABLE);
  EXPECT_TRUE(failed.obstacles.empty());
}

/// And recovery is immediate: the next scan whose transform resolves is
/// STATUS_OK again, with no re-confirmation period.
TEST(NoTemporalPersistence, TransformRecoveryIsImmediate)
{
  const auto failed = process_one_scan(scan_with_cone(1, 1.0), missing_transform());
  ASSERT_EQ(failed.status, ObstacleCircleArray::STATUS_TRANSFORM_UNAVAILABLE);

  const auto recovered = process_one_scan(scan_with_cone(2, 1.0), static_mount());

  EXPECT_EQ(recovered.status, ObstacleCircleArray::STATUS_OK);
  EXPECT_FALSE(recovered.obstacles.empty())
    << "recovery must not wait for confirmation frames";
}

/// Each published frame carries the measurement time of the scan it was built
/// from, so two scans can never be blended into one output.
TEST(NoTemporalPersistence, EachFrameCarriesItsOwnScanStamp)
{
  const auto first = process_one_scan(scan_with_cone(11, 1.0), static_mount());
  const auto second = process_one_scan(scan_with_cone(22, 1.0), static_mount());

  EXPECT_EQ(first.header.stamp.sec, 11);
  EXPECT_EQ(second.header.stamp.sec, 22);
  EXPECT_NE(first.header.stamp.sec, second.header.stamp.sec);
}

/// The published centre follows the measurement exactly. An EMA at alpha 0.2
/// would put the second frame roughly four fifths of the way back towards the
/// first; this asserts the output sits on the new measurement instead.
TEST(NoTemporalPersistence, MovingObstacleIsReportedWithoutSmoothingLag)
{
  const auto near_frame = process_one_scan(scan_with_cone(1, 1.0), static_mount());
  ASSERT_FALSE(near_frame.obstacles.empty());
  const float near_x = near_frame.obstacles[0].center_x;

  // The same cone, now measured 0.5 m closer.
  const auto moved_frame = process_one_scan(scan_with_cone(2, 0.5), static_mount());
  ASSERT_FALSE(moved_frame.obstacles.empty());
  const float moved_x = moved_frame.obstacles[0].center_x;

  // The reported jump is the full 0.5 m the measurement moved, to within the
  // beam quantisation of the cluster centroid.
  EXPECT_NEAR(near_x - moved_x, 0.5F, 0.05F)
    << "the output lagged the measurement, which means smoothing crept in";

  // And explicitly not the smoothed value an alpha=0.2 EMA would have given.
  const float ema_alpha = 0.2F;
  const float smoothed = ema_alpha * moved_x + (1.0F - ema_alpha) * near_x;
  EXPECT_GT(std::abs(moved_x - smoothed), 0.2F);
}

/// Replaying the identical scan gives the identical message: the chain is a
/// pure function of one scan plus one transform, with no accumulator anywhere.
TEST(NoTemporalPersistence, IdenticalScansProduceIdenticalOutput)
{
  const auto first = process_one_scan(scan_with_cone(1, 1.0), static_mount());
  // Interleave a different scan so any accumulator would be disturbed.
  const auto interleaved = process_one_scan(scan_with_cone(2, 3.0), static_mount());
  ASSERT_FALSE(interleaved.obstacles.empty());
  const auto repeat = process_one_scan(scan_with_cone(1, 1.0), static_mount());

  ASSERT_EQ(first.obstacles.size(), repeat.obstacles.size());
  for (std::size_t index = 0U; index < first.obstacles.size(); ++index) {
    EXPECT_FLOAT_EQ(first.obstacles[index].center_x, repeat.obstacles[index].center_x);
    EXPECT_FLOAT_EQ(first.obstacles[index].center_y, repeat.obstacles[index].center_y);
    EXPECT_FLOAT_EQ(first.obstacles[index].radius, repeat.obstacles[index].radius);
  }
}

/// A structurally broken scan reports the LiDAR as unavailable with an empty
/// array, and does not fall back on the last good frame.
TEST(NoTemporalPersistence, InvalidScanCarriesNothingOver)
{
  const auto populated = process_one_scan(scan_with_cone(1, 1.0), static_mount());
  ASSERT_FALSE(populated.obstacles.empty());

  auto broken = scan_with_cone(2, 1.0);
  broken.header.frame_id = "some_other_frame";

  const auto message = process_one_scan(broken, static_mount());
  EXPECT_EQ(message.status, ObstacleCircleArray::STATUS_LIDAR_UNAVAILABLE);
  EXPECT_TRUE(message.obstacles.empty());
}
