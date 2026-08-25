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
// Localization-free LiDAR Object Detection.
//
// Derived from package "kau_object_detection", its source
// laser_scan_clusterer_node.cpp, on 2026-08-25. This is an independent copy,
// not a link. See README.md "Source synchronisation".
//
// One scan in, one Object List out, in base_link:
//
//   /scan_filtered  (lidar_link)
//     -> validate -> speckle filter -> polar/Cartesian -> adjacent clustering
//     -> cluster geometry / width gate -> cone occupancy gate
//     -> tf2 base_link <- lidar_link at scan.header.stamp
//     -> publish
//
// Removed relative to the original, deliberately and permanently:
//   * ObstacleTracker, association, outlier gate, EMA smoothing, track
//     timeout, minimum confirmation frames. Every scan is judged on its own.
//     Rationale in README.md "Why there is no tracker".
//   * Gazebo Transport world pose subscription and its diagnostics.
//   * Track ROI, track ring, amet_2026_track.yaml.
//   * The raw-vs-smoothed diagnostic twin topic: with no smoothing stage there
//     is nothing for it to be the twin of.
//   * Every map-frame code path. base_link is the only output frame contract.
//
// This node reads no TF other than base_link <- lidar_link, which is a static
// URDF transform. It never looks up map or odom, so it runs unchanged whether
// or not AMCL, Cartographer or the EKF are running.
// ---------------------------------------------------------------------------

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <iomanip>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "kau_msgs/msg/obstacle_circle_array.hpp"
#include "kau_object_detection_lane/cluster_geometry.hpp"
#include "kau_object_detection_lane/cone_occupancy.hpp"
#include "kau_object_detection_lane/laser_clusterer.hpp"
#include "kau_object_detection_lane/object_list.hpp"
#include "kau_object_detection_lane/object_transform.hpp"
#include "kau_object_detection_lane/obstacle_markers.hpp"
#include "kau_object_detection_lane/scan_validation.hpp"
#include "rclcpp/create_timer.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2/exceptions.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace kau_object_detection_lane
{

class LaneLaserObjectDetectorNode : public rclcpp::Node
{
public:
  LaneLaserObjectDetectorNode()
  : Node("lane_laser_object_detector")
  {
    const auto input_topic = declare_parameter<std::string>("input_topic", "/scan_filtered");
    validation_options_.expected_frame_id =
      declare_parameter<std::string>("expected_frame_id", "lidar_link");

    const auto minimum_sample_count =
      declare_parameter<std::int64_t>("minimum_sample_count", 2);
    const auto sample_count_tolerance =
      declare_parameter<std::int64_t>("sample_count_tolerance", 1);
    validation_options_.require_intensities =
      declare_parameter<bool>("require_intensities", false);

    clustering_options_.base_distance_threshold_m =
      declare_parameter<double>("base_distance_threshold_m", 0.05);
    clustering_options_.angular_resolution_scale =
      declare_parameter<double>("angular_resolution_scale", 1.5);
    clustering_options_.circular_scan_tolerance_rad =
      declare_parameter<double>("circular_scan_tolerance_rad", 0.05);
    filter_options_.enabled =
      declare_parameter<bool>("speckle_filter_enabled", true);
    const auto minimum_support_neighbors =
      declare_parameter<std::int64_t>("minimum_support_neighbors", 1);
    filter_options_.neighbor_distance_scale =
      declare_parameter<double>("neighbor_distance_scale", 1.0);
    const auto minimum_cluster_points =
      declare_parameter<std::int64_t>("minimum_cluster_points", 1);
    geometry_options_.collinear_tolerance_m =
      declare_parameter<double>("collinear_tolerance_m", 0.001);
    occupancy_options_.nominal_cone_radius_m =
      declare_parameter<double>("nominal_cone_radius_m", 0.09);
    occupancy_options_.visible_slice_radius_m =
      declare_parameter<double>("visible_slice_radius_m", 0.043);
    occupancy_options_.maximum_observed_width_m =
      declare_parameter<double>("maximum_observed_width_m", 0.30);

    log_circle_candidates_ = declare_parameter<bool>("log_circle_candidates", true);
    summary_period_seconds_ = declare_parameter<double>("summary_period_seconds", 2.0);

    const auto object_list_topic =
      declare_parameter<std::string>("object_list_topic", "/perception/obstacles");
    const auto obstacle_marker_topic = declare_parameter<std::string>(
      "obstacle_marker_topic", "/perception/obstacle_markers");
    obstacle_marker_enabled_ =
      declare_parameter<bool>("obstacle_marker_enabled", true);
    obstacle_marker_options_.lifetime_s =
      declare_parameter<double>("obstacle_marker_lifetime_s", 0.3);

    // base_link is the Lane-only contract. The parameter exists so the frame
    // can be pointed at base_footprint on a vehicle whose URDF puts the planar
    // origin there, not so the node can be pushed back onto map: nothing else
    // in this package understands a map frame.
    object_list_options_.frame_id =
      declare_parameter<std::string>("object_list_frame_id", "base_link");
    object_list_options_.confidence = static_cast<float>(
      declare_parameter<double>("object_list_confidence", 1.0));
    object_list_scan_timeout_s_ =
      declare_parameter<double>("object_list_scan_timeout_s", 0.3);
    object_list_transform_timeout_s_ =
      declare_parameter<double>("object_list_transform_timeout_s", 0.05);
    const auto watchdog_period_s =
      declare_parameter<double>("object_list_watchdog_period_s", 0.1);
    watchdog_publish_period_s_ =
      declare_parameter<double>("object_list_watchdog_publish_period_s", 0.2);

    if (minimum_sample_count < 2) {
      throw std::invalid_argument("minimum_sample_count must be at least 2");
    }
    if (sample_count_tolerance < 0 || minimum_cluster_points < 1) {
      throw std::invalid_argument("sample-count parameters must be non-negative");
    }
    if (minimum_support_neighbors < 0) {
      throw std::invalid_argument("minimum_support_neighbors must be non-negative");
    }
    if (!std::isfinite(filter_options_.neighbor_distance_scale) ||
      filter_options_.neighbor_distance_scale < 0.0)
    {
      throw std::invalid_argument("neighbor_distance_scale must be finite and non-negative");
    }
    if (!std::isfinite(occupancy_options_.nominal_cone_radius_m) ||
      occupancy_options_.nominal_cone_radius_m <= 0.0)
    {
      throw std::invalid_argument("nominal_cone_radius_m must be finite and positive");
    }
    if (!std::isfinite(occupancy_options_.visible_slice_radius_m) ||
      occupancy_options_.visible_slice_radius_m < 0.0)
    {
      throw std::invalid_argument("visible_slice_radius_m must be finite and non-negative");
    }
    if (!std::isfinite(occupancy_options_.maximum_observed_width_m) ||
      occupancy_options_.maximum_observed_width_m <= 0.0)
    {
      throw std::invalid_argument("maximum_observed_width_m must be finite and positive");
    }
    if (!std::isfinite(object_list_scan_timeout_s_) || object_list_scan_timeout_s_ <= 0.0) {
      throw std::invalid_argument("object_list_scan_timeout_s must be finite and positive");
    }
    if (!std::isfinite(object_list_transform_timeout_s_) ||
      object_list_transform_timeout_s_ < 0.0)
    {
      throw std::invalid_argument(
              "object_list_transform_timeout_s must be finite and non-negative");
    }
    if (!std::isfinite(watchdog_period_s) || watchdog_period_s <= 0.0) {
      throw std::invalid_argument("object_list_watchdog_period_s must be finite and positive");
    }
    if (!std::isfinite(watchdog_publish_period_s_) || watchdog_publish_period_s_ < 0.0) {
      throw std::invalid_argument(
              "object_list_watchdog_publish_period_s must be finite and non-negative");
    }
    if (!std::isfinite(object_list_options_.confidence) ||
      object_list_options_.confidence < 0.0F || object_list_options_.confidence > 1.0F)
    {
      throw std::invalid_argument("object_list_confidence must be within [0, 1]");
    }
    if (object_list_options_.frame_id.empty()) {
      throw std::invalid_argument("object_list_frame_id must not be empty");
    }
    // A non-positive lifetime would mean "never expires" to RViz, which is the
    // opposite of the safety net the parameter exists for.
    if (!std::isfinite(obstacle_marker_options_.lifetime_s) ||
      obstacle_marker_options_.lifetime_s <= 0.0)
    {
      throw std::invalid_argument("obstacle_marker_lifetime_s must be finite and positive");
    }
    if (watchdog_publish_period_s_ >= object_list_scan_timeout_s_) {
      throw std::invalid_argument(
              "object_list_watchdog_publish_period_s must stay below "
              "object_list_scan_timeout_s so a consumer sees the state in time");
    }
    if (clustering_options_.base_distance_threshold_m < 0.0 ||
      clustering_options_.angular_resolution_scale < 0.0 ||
      clustering_options_.circular_scan_tolerance_rad < 0.0 ||
      geometry_options_.collinear_tolerance_m < 0.0 ||
      summary_period_seconds_ <= 0.0)
    {
      throw std::invalid_argument("clustering parameters must be non-negative");
    }

    validation_options_.minimum_sample_count =
      static_cast<std::size_t>(minimum_sample_count);
    validation_options_.sample_count_tolerance =
      static_cast<std::size_t>(sample_count_tolerance);
    candidate_options_.minimum_cluster_points =
      static_cast<std::size_t>(minimum_cluster_points);
    filter_options_.minimum_support_neighbors =
      static_cast<std::size_t>(minimum_support_neighbors);

    // Only source of the published coordinates. The listener runs its own
    // thread, so the bounded lookup wait inside the scan callback is serviced.
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_, this, true);

    scan_subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
      input_topic,
      rclcpp::SensorDataQoS(),
      std::bind(&LaneLaserObjectDetectorNode::scan_callback, this, std::placeholders::_1));

    // The Object List is a live perception stream: a late frame is worthless to
    // the planner, so the newest one always wins and none are retained.
    object_list_publisher_ = create_publisher<kau_msgs::msg::ObstacleCircleArray>(
      object_list_topic,
      rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile());

    // Same QoS as the Object List: the markers are the same live perception
    // frame, so a late one is worth no more than a late obstacle list.
    if (obstacle_marker_enabled_) {
      obstacle_marker_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        obstacle_marker_topic,
        rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile());
    }

    // Node clock, never a wall timer: the simulator drives /clock and a wall
    // timer would measure the LiDAR gap against the wrong time base.
    last_scan_activity_ = now();
    last_summary_time_ = now();
    watchdog_timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration::from_seconds(watchdog_period_s),
      std::bind(&LaneLaserObjectDetectorNode::watchdog_callback, this));

    RCLCPP_INFO(
      get_logger(),
      "ready input=%s expected_frame=%s qos=sensor_data(best_effort) "
      "base_threshold_m=%.3f angular_scale=%.3f speckle_filter=%d "
      "min_support_neighbors=%zu neighbor_scale=%.3f min_points=%zu "
      "collinear_tolerance_m=%.4f cone_radius_m=%.3f "
      "visible_slice_radius_m=%.3f max_observed_width_m=%.3f",
      input_topic.c_str(), validation_options_.expected_frame_id.c_str(),
      clustering_options_.base_distance_threshold_m,
      clustering_options_.angular_resolution_scale,
      filter_options_.enabled ? 1 : 0,
      filter_options_.minimum_support_neighbors,
      filter_options_.neighbor_distance_scale,
      candidate_options_.minimum_cluster_points,
      geometry_options_.collinear_tolerance_m,
      occupancy_options_.nominal_cone_radius_m,
      occupancy_options_.visible_slice_radius_m,
      occupancy_options_.maximum_observed_width_m);

    RCLCPP_INFO(
      get_logger(),
      "object list topic=%s frame_id=%s qos=best_effort/keep_last(1)/volatile "
      "confidence=%.2f scan_timeout_s=%.3f watchdog_period_s=%.3f "
      "watchdog_publish_period_s=%.3f transform=tf2 target_frame=%s "
      "source_frame=scan.header.frame_id transform_time=scan.header.stamp "
      "transform_timeout_s=%.3f",
      object_list_topic.c_str(), object_list_options_.frame_id.c_str(),
      static_cast<double>(object_list_options_.confidence),
      object_list_scan_timeout_s_, watchdog_period_s, watchdog_publish_period_s_,
      object_list_options_.frame_id.c_str(), object_list_transform_timeout_s_);

    // Stated explicitly at start-up so a log reader can tell this node from the
    // map-frame one without diffing parameter dumps.
    RCLCPP_INFO(
      get_logger(),
      "temporal tracking: none. no association, no outlier gate, no EMA, "
      "no track timeout, no confirmation frames. every scan is published on "
      "its own measurements only, and no obstacle survives into a later frame");

    RCLCPP_INFO(
      get_logger(),
      "obstacle markers enabled=%d topic=%s type=visualization_msgs/MarkerArray "
      "qos=best_effort/keep_last(1)/volatile ns=%s lifetime_s=%.3f "
      "source=published_object_list frame_id=object_list.header.frame_id "
      "consumer=visualisation_only",
      obstacle_marker_enabled_ ? 1 : 0, obstacle_marker_topic.c_str(),
      obstacle_marker_options_.ns.c_str(), obstacle_marker_options_.lifetime_s);
  }

private:
  void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan)
  {
    const auto validation = validate_laser_scan(*scan, validation_options_);
    if (!validation.valid) {
      ++invalid_message_count_;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "invalid scan frame=%s samples=%zu reason=%s",
        scan->header.frame_id.c_str(), scan->ranges.size(), validation.reason.c_str());
      // A structurally broken scan is a LiDAR failure, not an empty world. The
      // watchdog clock is deliberately not refreshed here: only a usable scan
      // counts as the sensor being alive.
      publish_object_list(ObjectListStatus::kLidarUnavailable, {}, scan->header.stamp);
      return;
    }

    // Only a scan that passed validation proves the sensor is delivering.
    last_scan_activity_ = now();

    // Explicit pipeline so the read-only diagnostics can report each stage
    // separately: range validation, speckle filtering, raw clustering, and
    // candidate acceptance.
    const auto topology = make_scan_neighbor_topology(
      *scan, clustering_options_.circular_scan_tolerance_rad);
    const auto points = convert_scan_indices_to_points(*scan, validation.usable_indices);
    const auto filtered =
      filter_isolated_scan_points(points, topology, clustering_options_, filter_options_);
    const auto raw_clusters =
      cluster_scan_points(filtered.points, topology, clustering_options_);
    const auto clusters = select_candidate_clusters(raw_clusters, candidate_options_);
    const auto geometries = compute_all_cluster_geometry(clusters, geometry_options_);
    const auto circle_candidates =
      compute_all_cone_occupancy_candidates(geometries, occupancy_options_);

    // --- Final Object List coordinate path ---------------------------------
    // tf2 is the single source of the published coordinates. The lookup is
    // base_link <- scan.header.frame_id at the scan measurement time, so the
    // whole mount chain is resolved inside it and no offset is added on top.
    const auto transform_lookup = lookup_object_list_transform(*scan);

    // Placement in the base frame. The sensor-frame circles above are never
    // recomputed here; the stage only places their centres.
    const auto placement =
      place_candidates_in_base_frame(circle_candidates, transform_lookup.transform);

    const auto status = decide_object_list_status_from_placement(
      true, transform_lookup.transform.valid, placement.applied);

    // Straight from this scan's placement to the wire. There is no tracker to
    // consult and no previous frame to merge with, which is the whole point of
    // this node: what goes out is exactly what this scan measured.
    publish_object_list(status, placement.accepted, scan->header.stamp);

    ++valid_message_count_since_summary_;
    for (const auto & geometry : geometries) {
      if (!geometry.valid) {
        ++invalid_geometry_count_;
      }
    }

    log_periodic_summary(
      *scan, validation, filtered, raw_clusters, clusters, geometries,
      circle_candidates, placement, transform_lookup);
  }

  /// Outcome of one Object List transform lookup. `failure_reason` is
  /// diagnostics only; the published contract is decided by
  /// `transform.valid` alone.
  struct ObjectListTransformLookup
  {
    SensorToBaseTransform transform;
    std::string failure_reason;
  };

  /// Looks up `object_list_frame_id <- scan.header.frame_id` at the scan
  /// measurement time.
  ///
  /// The source frame is whatever the scan itself declares, so no sensor frame
  /// name is hard coded here, and the target frame is the configured Object
  /// List frame. The lookup time is always `scan.header.stamp`: asking for the
  /// latest transform would place this frame's beams with the pose the vehicle
  /// had at some other moment.
  ///
  /// Every tf2 failure - unknown frame, disconnected tree, a stamp the buffer
  /// cannot serve, extrapolation, or a lookup that timed out - leaves the
  /// transform invalid. The caller turns that into STATUS_TRANSFORM_UNAVAILABLE
  /// with an empty obstacle array; no earlier frame is ever reused. Recovery is
  /// automatic: the next scan whose lookup succeeds publishes STATUS_OK again,
  /// because nothing about the failure is latched.
  ObjectListTransformLookup lookup_object_list_transform(
    const sensor_msgs::msg::LaserScan & scan)
  {
    ObjectListTransformLookup lookup;

    if (scan.header.frame_id.empty()) {
      lookup.failure_reason = "scan header carries no frame_id";
    } else {
      try {
        const auto transform_message = tf_buffer_->lookupTransform(
          object_list_options_.frame_id, scan.header.frame_id,
          rclcpp::Time(scan.header.stamp, RCL_ROS_TIME),
          rclcpp::Duration::from_seconds(object_list_transform_timeout_s_));
        lookup.transform = make_sensor_to_base_transform(
          transform_message.transform.translation.x,
          transform_message.transform.translation.y,
          transform_message.transform.rotation.x,
          transform_message.transform.rotation.y,
          transform_message.transform.rotation.z,
          transform_message.transform.rotation.w);
        if (!lookup.transform.valid) {
          lookup.failure_reason = "transform has no usable planar part";
        }
      } catch (const tf2::TransformException & exception) {
        lookup.failure_reason = exception.what();
      }
    }

    if (!lookup.transform.valid) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "no %s <- %s transform at scan stamp %d.%09u: %s; "
        "publishing STATUS_TRANSFORM_UNAVAILABLE",
        object_list_options_.frame_id.c_str(), scan.header.frame_id.c_str(),
        scan.header.stamp.sec, scan.header.stamp.nanosec,
        lookup.failure_reason.c_str());
    }

    return lookup;
  }

  /// Builds and publishes one Object List frame. The message is rebuilt from
  /// the arguments every time, so a failing frame can never carry an obstacle
  /// from an earlier one.
  void publish_object_list(
    const ObjectListStatus status,
    const std::vector<CircleCandidate> & candidates,
    const builtin_interfaces::msg::Time & stamp)
  {
    const auto message =
      build_object_list(status, candidates, stamp, object_list_options_);
    object_list_publisher_->publish(message);

    // Drawn from the frame that has just gone out, after it has gone out: the
    // planner is never made to wait on visualisation work.
    publish_obstacle_markers(message);

    last_object_list_publish_ = now();
    if (status != last_object_list_status_) {
      RCLCPP_INFO(
        get_logger(),
        "object list status %u -> %u obstacles=%zu",
        static_cast<unsigned int>(last_object_list_status_),
        static_cast<unsigned int>(status), message.obstacles.size());
      last_object_list_status_ = status;
    }
  }

  /// Publishes the RViz markers of one Object List frame.
  ///
  /// Visualisation only. The frame is the message just published on
  /// `object_list_topic`, taken by const reference, so the drawn circles are by
  /// construction the circles Local Path Planning receives.
  ///
  /// `previous_obstacle_marker_count_` is the only state this path keeps. It
  /// carries the ADD marker count of the previous frame so the builder can
  /// delete the ids this frame no longer fills, which is what clears the stale
  /// circles when the obstacle count drops, when the list is empty, or when the
  /// status turns non-OK. It is bookkeeping for RViz alone and never feeds back
  /// into the Object List.
  void publish_obstacle_markers(const kau_msgs::msg::ObstacleCircleArray & object_list)
  {
    if (!obstacle_marker_publisher_) {
      return;
    }

    const auto frame = build_obstacle_markers(
      object_list, previous_obstacle_marker_count_, obstacle_marker_options_);
    previous_obstacle_marker_count_ = frame.add_marker_count;

    // An empty array happens on a steady no-obstacle stream: nothing to draw
    // and nothing left to delete.
    if (frame.markers.markers.empty()) {
      return;
    }
    obstacle_marker_publisher_->publish(frame.markers);
  }

  /// Reports the LiDAR as unavailable while no usable scan arrives.
  ///
  /// The scan callback is the only healthy publisher, so this timer exists
  /// purely to keep the contract alive when that callback stops firing.
  void watchdog_callback()
  {
    const auto current_time = now();

    // With use_sim_time the node clock reads zero until the first /clock
    // message, so the anchor taken in the constructor can predate the
    // simulation epoch. Re-anchor once rather than reporting the whole
    // simulation runtime as the scan gap.
    if (last_scan_activity_.nanoseconds() == 0 && current_time.nanoseconds() != 0) {
      last_scan_activity_ = current_time;
      last_object_list_publish_ = current_time;
      return;
    }

    // A simulator reset can move the clock backwards. Re-anchor instead of
    // reporting a failure that never happened.
    if (current_time < last_scan_activity_) {
      last_scan_activity_ = current_time;
      last_object_list_publish_ = current_time;
      return;
    }

    const double scan_age_s = (current_time - last_scan_activity_).seconds();
    if (scan_age_s <= object_list_scan_timeout_s_) {
      return;
    }

    // The scan callback publishes on every frame, so rate limiting against the
    // last publication keeps the watchdog quiet while the pipeline is healthy
    // and still refreshes the state well inside the timeout once it is not.
    if (current_time >= last_object_list_publish_) {
      const double since_publish_s = (current_time - last_object_list_publish_).seconds();
      if (since_publish_s < watchdog_publish_period_s_) {
        return;
      }
    }

    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "no usable scan for %.3f s (timeout %.3f s); publishing STATUS_LIDAR_UNAVAILABLE",
      scan_age_s, object_list_scan_timeout_s_);

    // No scan means no measurement timestamp to speak for, so the node clock is
    // the only honest stamp available.
    publish_object_list(ObjectListStatus::kLidarUnavailable, {}, now());
  }

  /// Read-only periodic pipeline summary. Rate limited on the node clock so it
  /// follows simulation time and never floods the terminal at scan rate.
  void log_periodic_summary(
    const sensor_msgs::msg::LaserScan & scan,
    const ScanValidationResult & validation,
    const NeighborFilterResult & filtered,
    const std::vector<Cluster2D> & raw_clusters,
    const std::vector<Cluster2D> & clusters,
    const std::vector<ClusterGeometry2D> & geometries,
    const std::vector<ConeOccupancyCandidate> & circle_candidates,
    const PlacementResult & placement,
    const ObjectListTransformLookup & transform_lookup)
  {
    const auto current_time = now();
    if (current_time < last_summary_time_) {
      last_summary_time_ = current_time;
      return;
    }
    if ((current_time - last_summary_time_).seconds() < summary_period_seconds_) {
      return;
    }

    std::size_t valid_geometry_count = 0U;
    for (const auto & geometry : geometries) {
      if (geometry.valid) {
        ++valid_geometry_count;
      }
    }
    std::size_t valid_candidate_count = 0U;
    for (const auto & candidate : circle_candidates) {
      if (candidate.valid) {
        ++valid_candidate_count;
      }
    }

    RCLCPP_INFO(
      get_logger(),
      "frames=%zu invalid=%zu frame=%s beams=%zu usable=%zu non_finite=%zu "
      "below_min=%zu above_max=%zu speckle_removed=%zu raw_clusters=%zu "
      "candidate_clusters=%zu valid_geometry=%zu invalid_geometry=%zu "
      "circle_candidates=%zu transform_valid=%d placed=%zu rejected=%zu "
      "target_frame=%s scan_stamp=%d.%09u",
      valid_message_count_since_summary_, invalid_message_count_,
      scan.header.frame_id.c_str(), scan.ranges.size(),
      validation.usable_indices.size(), validation.non_finite_count,
      validation.below_minimum_count, validation.above_maximum_count,
      filtered.removed_speckle_count, raw_clusters.size(), clusters.size(),
      valid_geometry_count, invalid_geometry_count_, valid_candidate_count,
      transform_lookup.transform.valid ? 1 : 0,
      placement.candidates_after, placement.rejected,
      object_list_options_.frame_id.c_str(),
      scan.header.stamp.sec, scan.header.stamp.nanosec);

    if (log_circle_candidates_) {
      log_nearest_circle_candidates(placement.accepted);
    }

    valid_message_count_since_summary_ = 0U;
    last_summary_time_ = current_time;
  }

  /// Logs the closest published circle so a human can sanity-check the sign and
  /// magnitude of the base-frame coordinates against what is in front of the
  /// vehicle. Diagnostics only.
  void log_nearest_circle_candidates(const std::vector<CircleCandidate> & candidates)
  {
    const CircleCandidate * nearest = nullptr;
    double nearest_range_m = 0.0;
    for (const auto & candidate : candidates) {
      if (!candidate.base_center_valid) {
        continue;
      }
      const double range_m = std::hypot(candidate.sensor_x_m, candidate.sensor_y_m);
      if (nearest == nullptr || range_m < nearest_range_m) {
        nearest = &candidate;
        nearest_range_m = range_m;
      }
    }
    if (nearest == nullptr) {
      RCLCPP_INFO(get_logger(), "nearest circle: none in this frame");
      return;
    }

    RCLCPP_INFO(
      get_logger(),
      "nearest circle: sensor=(%.3f, %.3f) %s=(%.3f, %.3f) radius=%.3f "
      "points=%zu closest_range=%.3f",
      nearest->sensor_x_m, nearest->sensor_y_m,
      object_list_options_.frame_id.c_str(),
      nearest->base_x_m, nearest->base_y_m, nearest->radius_m,
      nearest->point_count, nearest->source_closest_range_m);
  }

  ScanValidationOptions validation_options_;
  ScanClusteringOptions clustering_options_;
  NeighborFilterOptions filter_options_;
  ClusterCandidateOptions candidate_options_;
  ClusterGeometryOptions geometry_options_;
  ConeOccupancyOptions occupancy_options_;
  ObjectListOptions object_list_options_;

  bool log_circle_candidates_{true};
  double summary_period_seconds_{2.0};
  std::size_t valid_message_count_since_summary_{0U};
  std::size_t invalid_message_count_{0U};
  std::size_t invalid_geometry_count_{0U};
  rclcpp::Time last_summary_time_{0, 0, RCL_ROS_TIME};

  double object_list_scan_timeout_s_{0.3};
  double object_list_transform_timeout_s_{0.05};
  double watchdog_publish_period_s_{0.2};
  rclcpp::Time last_scan_activity_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_object_list_publish_{0, 0, RCL_ROS_TIME};
  ObjectListStatus last_object_list_status_{ObjectListStatus::kOk};

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
  rclcpp::Publisher<kau_msgs::msg::ObstacleCircleArray>::SharedPtr object_list_publisher_;

  /// RViz visualisation of the published Object List. Null while disabled,
  /// which is the only thing that can switch it off; the Object List itself is
  /// not affected either way.
  bool obstacle_marker_enabled_{true};
  ObstacleMarkerOptions obstacle_marker_options_;
  /// ADD marker count of the previous frame, kept so the ids this frame no
  /// longer fills can be deleted instead of lingering in RViz.
  std::size_t previous_obstacle_marker_count_{0U};
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    obstacle_marker_publisher_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
};

}  // namespace kau_object_detection_lane

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<kau_object_detection_lane::LaneLaserObjectDetectorNode>());
  rclcpp::shutdown();
  return 0;
}
