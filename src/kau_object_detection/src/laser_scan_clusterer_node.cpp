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

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "kau_msgs/msg/obstacle_circle_array.hpp"
#include "kau_object_detection/cluster_geometry.hpp"
#include "kau_object_detection/cone_occupancy.hpp"
#include "kau_object_detection/laser_scan_clusterer.hpp"
#include "kau_object_detection/laser_scan_validator.hpp"
#include "kau_object_detection/object_list.hpp"
#include "kau_object_detection/object_list_transform.hpp"
#include "kau_object_detection/obstacle_tracker.hpp"
#include "kau_object_detection/track_geometry.hpp"
#include "kau_object_detection/track_roi.hpp"
#include "kau_object_detection/world_pose.hpp"
#include "rclcpp/create_timer.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2/exceptions.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"

#ifdef KAU_OBJECT_DETECTION_WITH_GZ_TRANSPORT
#include <gz/msgs/pose_v.pb.h>
#include <gz/transport/Node.hh>
#endif

namespace kau_object_detection
{

class LaserScanClustererNode : public rclcpp::Node
{
public:
  LaserScanClustererNode()
  : Node("laser_scan_clusterer"), last_summary_time_(std::chrono::steady_clock::now())
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
    log_circle_candidates_ =
      declare_parameter<bool>("log_circle_candidates", true);
    const auto world_pose_topic =
      declare_parameter<std::string>("world_pose_topic", "/model/physicar/pose");
    world_pose_base_frame_ =
      declare_parameter<std::string>("world_pose_base_frame", "base_footprint");
    world_pose_timeout_s_ =
      declare_parameter<double>("world_pose_timeout_s", 0.5);
    mount_options_.lidar_offset_x_m =
      declare_parameter<double>("lidar_offset_x_m", -0.027);
    mount_options_.lidar_offset_y_m =
      declare_parameter<double>("lidar_offset_y_m", 0.0);
    roi_options_.boundary_tolerance_m =
      declare_parameter<double>("track_boundary_tolerance_m", 0.15);
    const auto track_outer_x =
      declare_parameter<std::vector<double>>("track_outer_x", std::vector<double>{});
    const auto track_outer_y =
      declare_parameter<std::vector<double>>("track_outer_y", std::vector<double>{});
    const auto track_inner_x =
      declare_parameter<std::vector<double>>("track_inner_x", std::vector<double>{});
    const auto track_inner_y =
      declare_parameter<std::vector<double>>("track_inner_y", std::vector<double>{});
    log_cluster_geometry_ =
      declare_parameter<bool>("log_cluster_geometry", true);
    const auto geometry_log_cluster_count =
      declare_parameter<std::int64_t>("geometry_log_cluster_count", 3);
    summary_period_seconds_ =
      declare_parameter<double>("summary_period_seconds", 2.0);

    // Object List output. The diagnostic logging above keeps its own period;
    // these parameters only govern the published contract.
    const auto object_list_topic =
      declare_parameter<std::string>("object_list_topic", "/perception/obstacles");
    object_list_options_.frame_id =
      declare_parameter<std::string>("object_list_frame_id", "map");
    object_list_options_.confidence = static_cast<float>(
      declare_parameter<double>("object_list_confidence", 1.0));
    object_list_pose_timeout_s_ =
      declare_parameter<double>("object_list_pose_timeout_s", 0.3);
    object_list_scan_timeout_s_ =
      declare_parameter<double>("object_list_scan_timeout_s", 0.3);
    // How long the Object List transform lookup may wait for a TF that covers
    // the scan measurement time. Only a short wait is useful: a scan whose
    // transform never arrives has to be reported, not held.
    object_list_transform_timeout_s_ =
      declare_parameter<double>("object_list_transform_timeout_s", 0.05);
    const auto watchdog_period_s =
      declare_parameter<double>("object_list_watchdog_period_s", 0.1);
    watchdog_publish_period_s_ =
      declare_parameter<double>("object_list_watchdog_publish_period_s", 0.2);

    // Temporal tracking. Runs on the map-frame circle candidates only; nothing
    // upstream of the track ROI stage is affected by these values.
    temporal_tracking_enabled_ =
      declare_parameter<bool>("temporal_tracking_enabled", true);
    ObstacleTrackerOptions tracker_options;
    tracker_options.association_distance_m =
      declare_parameter<double>("association_distance_m", 0.30);
    tracker_options.outlier_distance_m =
      declare_parameter<double>("outlier_distance_m", 0.20);
    tracker_options.smoothing_alpha =
      declare_parameter<double>("smoothing_alpha", 0.4);
    const auto minimum_confirmation_frames =
      declare_parameter<std::int64_t>("minimum_confirmation_frames", 2);
    tracker_options.track_timeout_s =
      declare_parameter<double>("track_timeout_s", 0.7);

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
    if (geometry_log_cluster_count < 0) {
      throw std::invalid_argument("geometry_log_cluster_count must be non-negative");
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
    if (!std::isfinite(world_pose_timeout_s_) || world_pose_timeout_s_ <= 0.0) {
      throw std::invalid_argument("world_pose_timeout_s must be finite and positive");
    }
    if (!std::isfinite(mount_options_.lidar_offset_x_m) ||
      !std::isfinite(mount_options_.lidar_offset_y_m))
    {
      throw std::invalid_argument("lidar mount offsets must be finite");
    }
    if (!std::isfinite(roi_options_.boundary_tolerance_m) ||
      roi_options_.boundary_tolerance_m < 0.0)
    {
      throw std::invalid_argument("track_boundary_tolerance_m must be finite and non-negative");
    }
    if (!std::isfinite(object_list_pose_timeout_s_) || object_list_pose_timeout_s_ <= 0.0) {
      throw std::invalid_argument("object_list_pose_timeout_s must be finite and positive");
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
    if (watchdog_publish_period_s_ >= object_list_scan_timeout_s_) {
      throw std::invalid_argument(
              "object_list_watchdog_publish_period_s must stay below "
              "object_list_scan_timeout_s so a consumer sees the state in time");
    }
    // Both freshness rules belong to the simulator-only Gazebo diagnostic now
    // that tf2 places the published circles. The stricter Object List rule is
    // still reported next to the looser ROI one so the two stay comparable.
    if (object_list_pose_timeout_s_ > world_pose_timeout_s_) {
      RCLCPP_WARN(
        get_logger(),
        "object_list_pose_timeout_s=%.3f exceeds world_pose_timeout_s=%.3f; "
        "the simulator world pose diagnostic reports the ROI as applied while "
        "already calling the same pose too old for the Object List",
        object_list_pose_timeout_s_, world_pose_timeout_s_);
    }

    if (!std::isfinite(tracker_options.association_distance_m) ||
      tracker_options.association_distance_m <= 0.0)
    {
      throw std::invalid_argument("association_distance_m must be finite and positive");
    }
    if (!std::isfinite(tracker_options.outlier_distance_m) ||
      tracker_options.outlier_distance_m <= 0.0)
    {
      throw std::invalid_argument("outlier_distance_m must be finite and positive");
    }
    if (!std::isfinite(tracker_options.smoothing_alpha) ||
      tracker_options.smoothing_alpha <= 0.0 || tracker_options.smoothing_alpha > 1.0)
    {
      throw std::invalid_argument("smoothing_alpha must be within (0, 1]");
    }
    if (minimum_confirmation_frames < 1) {
      throw std::invalid_argument("minimum_confirmation_frames must be at least 1");
    }
    if (!std::isfinite(tracker_options.track_timeout_s) ||
      tracker_options.track_timeout_s <= 0.0)
    {
      throw std::invalid_argument("track_timeout_s must be finite and positive");
    }
    // The two gates answer different questions, so neither implies the other.
    // An outlier gate at or above the association gate can never fire, which
    // silently disables the outlier rejection instead of tuning it.
    if (tracker_options.outlier_distance_m >= tracker_options.association_distance_m) {
      RCLCPP_WARN(
        get_logger(),
        "outlier_distance_m=%.3f is not below association_distance_m=%.3f; "
        "every associated measurement passes the outlier gate, so momentary "
        "outliers are smoothed in instead of being rejected",
        tracker_options.outlier_distance_m, tracker_options.association_distance_m);
    }
    tracker_options.minimum_confirmation_frames =
      static_cast<std::size_t>(minimum_confirmation_frames);
    obstacle_tracker_ = ObstacleTracker(tracker_options);

    // An unconfigured or malformed track leaves the ring invalid, which makes
    // the ROI stage fail open instead of dropping obstacles.
    track_ring_ = make_track_ring(track_outer_x, track_outer_y, track_inner_x, track_inner_y);
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
    geometry_log_cluster_count_ =
      static_cast<std::size_t>(geometry_log_cluster_count);

    // Final Object List coordinate source. The listener runs its own thread, so
    // the bounded lookup wait inside the scan callback is actually serviced.
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_, this, true);

    scan_subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
      input_topic,
      rclcpp::SensorDataQoS(),
      std::bind(&LaserScanClustererNode::scan_callback, this, std::placeholders::_1));

    // The Object List is a live perception stream: a late frame is worthless to
    // the planner, so the newest one always wins and none are retained.
    object_list_publisher_ = create_publisher<kau_msgs::msg::ObstacleCircleArray>(
      object_list_topic,
      rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile());

    // Node clock, never a wall timer: the simulator drives /clock and a wall
    // timer would measure the LiDAR gap against the wrong time base.
    last_scan_activity_ = now();
    watchdog_timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration::from_seconds(watchdog_period_s),
      std::bind(&LaserScanClustererNode::watchdog_callback, this));

    subscribe_world_pose(world_pose_topic);

    RCLCPP_INFO(
      get_logger(),
      "ready input=%s expected_frame=%s qos=sensor_data(best_effort) "
      "base_threshold_m=%.3f angular_scale=%.3f speckle_filter=%d "
      "min_support_neighbors=%zu neighbor_scale=%.3f min_points=%zu "
      "collinear_tolerance_m=%.4f geometry_log_clusters=%zu "
      "cone_radius_m=%.3f visible_slice_radius_m=%.3f max_observed_width_m=%.3f "
      "world_pose_topic=%s world_pose_base_frame=%s world_pose_timeout_s=%.3f "
      "world_pose_transport=%s lidar_offset_m=(%.3f,%.3f) track_ring_valid=%d "
      "track_outer_points=%zu track_inner_points=%zu track_boundary_tolerance_m=%.3f",
      input_topic.c_str(), validation_options_.expected_frame_id.c_str(),
      clustering_options_.base_distance_threshold_m,
      clustering_options_.angular_resolution_scale,
      filter_options_.enabled ? 1 : 0,
      filter_options_.minimum_support_neighbors,
      filter_options_.neighbor_distance_scale,
      candidate_options_.minimum_cluster_points,
      geometry_options_.collinear_tolerance_m,
      log_cluster_geometry_ ? geometry_log_cluster_count_ : 0U,
      occupancy_options_.nominal_cone_radius_m,
      occupancy_options_.visible_slice_radius_m,
      occupancy_options_.maximum_observed_width_m,
      world_pose_topic.c_str(), world_pose_base_frame_.c_str(), world_pose_timeout_s_,
      world_pose_transport_state_.c_str(),
      mount_options_.lidar_offset_x_m, mount_options_.lidar_offset_y_m,
      track_ring_.valid ? 1 : 0, track_ring_.outer.size(), track_ring_.inner.size(),
      roi_options_.boundary_tolerance_m);

    RCLCPP_INFO(
      get_logger(),
      "object list topic=%s frame_id=%s qos=best_effort/keep_last(1)/volatile "
      "confidence=%.2f pose_timeout_s=%.3f scan_timeout_s=%.3f "
      "watchdog_period_s=%.3f watchdog_publish_period_s=%.3f "
      "transform=tf2 target_frame=%s source_frame=scan.header.frame_id "
      "transform_time=scan.header.stamp transform_timeout_s=%.3f",
      object_list_topic.c_str(), object_list_options_.frame_id.c_str(),
      static_cast<double>(object_list_options_.confidence),
      object_list_pose_timeout_s_, object_list_scan_timeout_s_,
      watchdog_period_s, watchdog_publish_period_s_,
      object_list_options_.frame_id.c_str(), object_list_transform_timeout_s_);

    const auto & applied_tracker_options = obstacle_tracker_.options();
    RCLCPP_INFO(
      get_logger(),
      "temporal tracking enabled=%d association_distance_m=%.3f "
      "outlier_distance_m=%.3f smoothing_alpha=%.2f "
      "minimum_confirmation_frames=%zu track_timeout_s=%.3f",
      temporal_tracking_enabled_ ? 1 : 0,
      applied_tracker_options.association_distance_m,
      applied_tracker_options.outlier_distance_m,
      applied_tracker_options.smoothing_alpha,
      applied_tracker_options.minimum_confirmation_frames,
      applied_tracker_options.track_timeout_s);
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
      obstacle_tracker_.reset();
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
    // tf2 is the single source of the published coordinates from here on. The
    // Gazebo world pose below is diagnostics only and never reaches this path.
    const auto transform_lookup = lookup_object_list_transform(*scan);

    // Placement in the target frame. The sensor-frame circles above are never
    // recomputed here; the stage only places their centres.
    //
    // The track ROI is not a rejection rule on this path. Its boundary polygons
    // are simulator world coordinates and do not describe the target frame, so
    // gating on them would drop every candidate. Off-track objects are not an
    // avoidance target for now, and what stops walls and fences from becoming
    // circles is the cluster width / max-extent gate and the cone occupancy
    // gate upstream, both untouched. The ring is still evaluated per candidate
    // as a read-only annotation.
    const auto placement = place_candidates_in_target_frame(
      circle_candidates, transform_lookup.transform, track_ring_, roi_options_);

    // Object List output. This runs before the diagnostic summary below, which
    // returns early on its own period; publishing after it would throttle the
    // contract down to the summary rate.
    const auto object_list_status = decide_object_list_status_from_placement(
      true, transform_lookup.transform.valid, placement.applied);

    // Temporal tracking. Only placed candidates with a resolved target-frame
    // centre are handed over, and only on a STATUS_OK frame. On any other
    // status the map coordinates cannot be trusted, so the tracker is cleared
    // instead of being carried across the gap; that also keeps the contract
    // that a non-OK frame never republishes an earlier result.
    ObstacleTrackerUpdate tracker_update;
    std::vector<WorldCircleCandidate> published_candidates;
    if (!temporal_tracking_enabled_) {
      published_candidates = placement.accepted;
    } else if (object_list_status != ObjectListStatus::kOk) {
      obstacle_tracker_.reset();
    } else {
      std::vector<ObstacleMeasurement> measurements;
      std::vector<const WorldCircleCandidate *> measurement_sources;
      measurements.reserve(placement.accepted.size());
      measurement_sources.reserve(placement.accepted.size());
      for (const auto & candidate : placement.accepted) {
        if (!candidate.world_center_valid) {
          continue;
        }
        measurements.push_back(
          ObstacleMeasurement{candidate.world_x_m, candidate.world_y_m, candidate.radius_m});
        measurement_sources.push_back(&candidate);
      }

      tracker_update = obstacle_tracker_.update(measurements, scan_timestamp_s(*scan));

      published_candidates.reserve(tracker_update.obstacles.size());
      for (const auto & obstacle : tracker_update.obstacles) {
        // The candidate keeps its radius, validity and provenance verbatim.
        // Only the map centre is replaced by the smoothed estimate.
        WorldCircleCandidate candidate = *measurement_sources[obstacle.measurement_index];
        candidate.world_x_m = obstacle.x_m;
        candidate.world_y_m = obstacle.y_m;
        published_candidates.push_back(candidate);
      }
    }

    publish_object_list(object_list_status, published_candidates, scan->header.stamp);

    ++valid_message_count_since_summary_;

    for (const auto & geometry : geometries) {
      if (!geometry.valid) {
        ++invalid_geometry_count_;
      }
    }

    const auto now = std::chrono::steady_clock::now();
    const double elapsed_seconds =
      std::chrono::duration<double>(now - last_summary_time_).count();
    if (elapsed_seconds < summary_period_seconds_) {
      return;
    }

    std::size_t clustered_point_count = 0U;
    std::ostringstream cluster_sizes;
    cluster_sizes << "[";
    for (std::size_t index = 0U; index < clusters.size(); ++index) {
      const auto point_count = clusters[index].points.size();
      clustered_point_count += point_count;
      if (index > 0U) {
        cluster_sizes << ",";
      }
      cluster_sizes << point_count;
    }
    cluster_sizes << "]";

    const double measured_rate =
      static_cast<double>(valid_message_count_since_summary_) / elapsed_seconds;

    // --- Simulator-only Gazebo world pose diagnostics ----------------------
    // Everything from here to the end of this block is read-only reporting. The
    // frame published above was placed by tf2 alone; the Gazebo ground truth is
    // sampled and transformed again here purely so the two can be compared, and
    // at the summary period rather than on every scan.
    const double now_monotonic_s = monotonic_now_s();
    WorldPoseSample world_pose;
    std::size_t world_pose_messages = 0U;
    {
      const std::lock_guard<std::mutex> lock(world_pose_mutex_);
      world_pose = latest_world_pose_;
      world_pose_messages = world_pose_message_count_since_summary_;
      world_pose_message_count_since_summary_ = 0U;
    }
    const bool world_pose_available =
      world_pose_is_fresh(world_pose, now_monotonic_s, world_pose_timeout_s_);
    const bool object_list_pose_fresh =
      world_pose_is_fresh(world_pose, now_monotonic_s, object_list_pose_timeout_s_);
    // Ground-truth ROI. Same stage, fed by the Gazebo pose and the measured
    // mount offset instead of by tf2. Its result is logged and then discarded.
    const auto simulator_roi = apply_track_roi(
      circle_candidates, track_ring_, world_pose_available,
      BasePose2D{world_pose.x_m, world_pose.y_m, world_pose.yaw_rad},
      mount_options_, roi_options_);
    const double world_pose_age_seconds = world_pose_age_s(world_pose, now_monotonic_s);
    const double world_pose_rate =
      static_cast<double>(world_pose_messages) / elapsed_seconds;

    // How many published candidates the track ROI would have dropped if its
    // polygons described the target frame. Read-only: it is the number that
    // says whether the ring can be put back in front of the publisher once the
    // real target-frame boundaries land.
    std::size_t off_track_ring_candidates = 0U;
    if (track_ring_.valid) {
      for (const auto & candidate : placement.accepted) {
        if (!candidate.inside_track_ring) {
          ++off_track_ring_candidates;
        }
      }
    }

    // Read-only pose/scan alignment diagnostic. Nothing here selects,
    // interpolates or rejects a pose; the value is reported and never acted on.
    const double scan_stamp_seconds = scan_timestamp_s(*scan);
    const bool scan_stamp_usable = simulation_stamp_is_usable(scan_stamp_seconds);
    const bool world_pose_stamp_usable = simulation_stamp_is_usable(world_pose.stamp_s);
    const std::string scan_stamp_text =
      format_diagnostic_stamp(scan_stamp_seconds, scan_stamp_usable, 3);
    const std::string world_pose_stamp_text =
      format_diagnostic_stamp(world_pose.stamp_s, world_pose_stamp_usable, 3);
    // Both stamps come from the simulation clock, so the difference is only
    // meaningful when each of them is a real measurement.
    const std::string world_pose_minus_scan_text = format_diagnostic_stamp(
      1000.0 * (world_pose.stamp_s - scan_stamp_seconds),
      world_pose_stamp_usable && scan_stamp_usable, 1);

    RCLCPP_INFO(
      get_logger(),
      "scan frame=%s usable_points=%zu speckle_removed_points=%zu raw_clusters=%zu "
      "accepted_candidates=%zu clustered_points=%zu "
      "cluster_sizes=%s rate_hz=%.2f invalid_messages=%zu invalid_geometry=%zu "
      "object_list_transform_available=%d object_list_target_frame=%s "
      "object_list_transform_x_m=%.3f object_list_transform_y_m=%.3f "
      "object_list_transform_yaw_deg=%.2f object_list_transform_failure=%s "
      "object_list_valid_occupancy_candidates=%zu "
      "object_list_candidates_before_tracking=%zu "
      "object_list_non_finite_rejected=%zu "
      "track_roi_final_output_applied=0 track_roi_would_reject_candidates=%zu "
      "raw_circle_candidates=%zu tracker_enabled=%d tracker_active_tracks=%zu "
      "tracker_confirmed_observations=%zu tracker_rejected_outliers=%zu "
      "smoothed_output_candidates=%zu "
      "sim_world_pose_available=%d sim_world_pose_age_s=%.3f sim_world_base_x_m=%.3f "
      "sim_world_base_y_m=%.3f sim_world_base_yaw_deg=%.2f sim_world_pose_rate_hz=%.2f "
      "sim_object_list_pose_fresh=%d sim_track_roi_diagnostic_applied=%d "
      "sim_circle_candidates_after_roi=%zu sim_track_roi_rejected_candidates=%zu "
      "scan_stamp_s=%s sim_world_pose_stamp_s=%s sim_world_pose_minus_scan_ms=%s",
      scan->header.frame_id.c_str(), validation.usable_indices.size(),
      filtered.removed_speckle_count, raw_clusters.size(), clusters.size(),
      clustered_point_count, cluster_sizes.str().c_str(), measured_rate,
      invalid_message_count_, invalid_geometry_count_,
      transform_lookup.transform.valid ? 1 : 0, object_list_options_.frame_id.c_str(),
      transform_lookup.transform.x_m, transform_lookup.transform.y_m,
      radians_to_degrees(transform_lookup.transform.yaw_rad),
      transform_lookup.transform.valid ? "none" : transform_lookup.failure_reason.c_str(),
      placement.candidates_before, placement.candidates_after, placement.rejected,
      off_track_ring_candidates,
      circle_candidates.size(), temporal_tracking_enabled_ ? 1 : 0,
      tracker_update.active_tracks, tracker_update.confirmed_observations,
      tracker_update.rejected_outliers, published_candidates.size(),
      world_pose_available ? 1 : 0,
      std::isfinite(world_pose_age_seconds) ? world_pose_age_seconds : -1.0,
      world_pose_available ? world_pose.x_m : 0.0,
      world_pose_available ? world_pose.y_m : 0.0,
      world_pose_available ? radians_to_degrees(world_pose.yaw_rad) : 0.0,
      world_pose_rate, object_list_pose_fresh ? 1 : 0,
      simulator_roi.applied ? 1 : 0, simulator_roi.candidates_after, simulator_roi.rejected,
      scan_stamp_text.c_str(), world_pose_stamp_text.c_str(),
      world_pose_minus_scan_text.c_str());

    log_nearest_cluster_geometry(geometries);
    log_nearest_circle_candidates(placement.accepted);

    valid_message_count_since_summary_ = 0U;
    last_summary_time_ = now;
  }

  /// Outcome of one Object List transform lookup. `failure_reason` is
  /// diagnostics only; the published contract is decided by
  /// `transform.valid` alone.
  struct ObjectListTransformLookup
  {
    SensorToTargetTransform transform;
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
  /// with an empty obstacle array; no earlier frame is ever reused.
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
        lookup.transform = make_sensor_to_target_transform(
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

  /// Measurement time of one scan, in seconds. The tracker orders its frames
  /// on this value, so it must be the sensor stamp and never a node clock read.
  static double scan_timestamp_s(const sensor_msgs::msg::LaserScan & scan)
  {
    return static_cast<double>(scan.header.stamp.sec) +
           (static_cast<double>(scan.header.stamp.nanosec) * 1e-9);
  }

  static double monotonic_now_s()
  {
    return std::chrono::duration<double>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
  }

  /// True when a simulation timestamp can be reported as a measured value. A
  /// stamp of zero is the unset default on both the scan header and the world
  /// pose sample, so it is treated as absent rather than as time zero.
  static bool simulation_stamp_is_usable(const double stamp_s)
  {
    return std::isfinite(stamp_s) && stamp_s > 0.0;
  }

  /// Renders one diagnostic timestamp field. An absent or unusable stamp is
  /// reported as `n/a`, never as a zero, so a log reader can never mistake a
  /// missing value for a measured one.
  static std::string format_diagnostic_stamp(
    const double value,
    const bool usable,
    const int precision)
  {
    if (!usable) {
      return std::string("n/a");
    }
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(precision) << value;
    return stream.str();
  }

  /// Builds and publishes one Object List frame. The message is rebuilt from
  /// the arguments every time, so a failing frame can never carry an obstacle
  /// from an earlier one.
  void publish_object_list(
    const ObjectListStatus status,
    const std::vector<WorldCircleCandidate> & candidates,
    const builtin_interfaces::msg::Time & stamp)
  {
    const auto message =
      build_object_list(status, candidates, stamp, object_list_options_);
    object_list_publisher_->publish(message);

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
    // the only honest stamp available. The tracker is cleared as well: its
    // tracks are only as good as the scan stream that fed them.
    obstacle_tracker_.reset();
    publish_object_list(ObjectListStatus::kLidarUnavailable, {}, now());
  }

  /// Stores the newest world pose. Called from a Gazebo Transport thread, so
  /// every access to `latest_world_pose_` is guarded by `world_pose_mutex_`.
  void store_world_pose(const std::vector<WorldPoseCandidate> & candidates)
  {
    const auto sample =
      select_world_base_pose(candidates, world_pose_base_frame_, monotonic_now_s());
    if (!sample.valid) {
      return;
    }

    const std::lock_guard<std::mutex> lock(world_pose_mutex_);
    latest_world_pose_ = sample;
    ++world_pose_message_count_since_summary_;
  }

#ifdef KAU_OBJECT_DETECTION_WITH_GZ_TRANSPORT
  void subscribe_world_pose(const std::string & topic)
  {
    gz_node_ = std::make_unique<gz::transport::Node>();
    if (gz_node_->Subscribe(topic, &LaserScanClustererNode::world_pose_callback, this)) {
      world_pose_transport_state_ = "gz_transport";
      return;
    }

    world_pose_transport_state_ = "gz_transport_subscribe_failed";
    RCLCPP_WARN(
      get_logger(),
      "could not subscribe to Gazebo world pose topic %s; "
      "world pose diagnostics stay unavailable",
      topic.c_str());
  }

  /// Converts one Gazebo `Pose_V` into the transport-free representation the
  /// tested selection logic works on. The per-pose header carries the
  /// simulation stamp on this platform; the message header is the fallback.
  void world_pose_callback(const gz::msgs::Pose_V & message)
  {
    const double message_stamp_s = message.has_header() && message.header().has_stamp() ?
      static_cast<double>(message.header().stamp().sec()) +
      1e-9 * static_cast<double>(message.header().stamp().nsec()) :
      0.0;

    std::vector<WorldPoseCandidate> candidates;
    candidates.reserve(static_cast<std::size_t>(message.pose_size()));
    for (int index = 0; index < message.pose_size(); ++index) {
      const auto & pose = message.pose(index);

      WorldPoseCandidate candidate;
      candidate.name = pose.name();
      candidate.x_m = pose.position().x();
      candidate.y_m = pose.position().y();
      candidate.orientation_w = pose.orientation().w();
      candidate.orientation_x = pose.orientation().x();
      candidate.orientation_y = pose.orientation().y();
      candidate.orientation_z = pose.orientation().z();
      candidate.stamp_s = message_stamp_s;

      if (pose.has_header()) {
        if (pose.header().has_stamp()) {
          candidate.stamp_s =
            static_cast<double>(pose.header().stamp().sec()) +
            1e-9 * static_cast<double>(pose.header().stamp().nsec());
        }
        for (int entry = 0; entry < pose.header().data_size(); ++entry) {
          const auto & data = pose.header().data(entry);
          if (data.key() == "child_frame_id" && data.value_size() > 0) {
            candidate.child_frame_id = data.value(0);
          }
        }
      }

      candidates.push_back(candidate);
    }

    store_world_pose(candidates);
  }
#else
  void subscribe_world_pose(const std::string & topic)
  {
    world_pose_transport_state_ = "unavailable_at_build_time";
    RCLCPP_WARN(
      get_logger(),
      "built without gz-transport13; world pose topic %s is not subscribed",
      topic.c_str());
  }
#endif

  /// Read-only diagnostic output of the measured obstacle shape for the
  /// clusters closest to the sensor. No message is published here.
  void log_nearest_cluster_geometry(const std::vector<ClusterGeometry2D> & geometries) const
  {
    if (!log_cluster_geometry_ || geometry_log_cluster_count_ == 0U) {
      return;
    }

    std::vector<const ClusterGeometry2D *> ordered_geometries;
    ordered_geometries.reserve(geometries.size());
    for (const auto & geometry : geometries) {
      if (geometry.valid) {
        ordered_geometries.push_back(&geometry);
      }
    }
    std::sort(
      ordered_geometries.begin(), ordered_geometries.end(),
      [](const ClusterGeometry2D * first, const ClusterGeometry2D * second) {
        return first->closest_point.range_m < second->closest_point.range_m;
      });

    const std::size_t logged_count =
      std::min(geometry_log_cluster_count_, ordered_geometries.size());
    for (std::size_t index = 0U; index < logged_count; ++index) {
      const auto & geometry = *ordered_geometries[index];
      RCLCPP_INFO(
        get_logger(),
        "  near[%zu] points=%zu closest=(%.3f,%.3f) closest_range_m=%.3f "
        "closest_index=%zu representative=(%.3f,%.3f) representative_range_m=%.3f "
        "width_m=%.3f max_extent_m=%.3f footprint_vertices=%zu footprint_area_m2=%.4f "
        "scan_indices=[%zu..%zu] wraps=%d",
        index, geometry.point_count, geometry.closest_point.x_m, geometry.closest_point.y_m,
        geometry.closest_point.range_m, geometry.closest_point.scan_index,
        geometry.representative_x_m, geometry.representative_y_m,
        geometry.representative_range_m, geometry.width_m, geometry.max_extent_m,
        geometry.footprint_vertices.size(), geometry.footprint_area_m2,
        geometry.first_scan_index, geometry.last_scan_index,
        geometry.wraps_scan_boundary ? 1 : 0);
    }
  }

  /// Read-only diagnostic output of the provisional circular occupancy model
  /// for the candidates closest to the sensor. No message is published here and
  /// no safety inflation is applied; that stays with Local Path Planning.
  void log_nearest_circle_candidates(
    const std::vector<WorldCircleCandidate> & candidates) const
  {
    if (!log_circle_candidates_ || geometry_log_cluster_count_ == 0U) {
      return;
    }

    std::vector<const WorldCircleCandidate *> ordered_candidates;
    ordered_candidates.reserve(candidates.size());
    for (const auto & candidate : candidates) {
      ordered_candidates.push_back(&candidate);
    }
    std::sort(
      ordered_candidates.begin(), ordered_candidates.end(),
      [](const WorldCircleCandidate * first, const WorldCircleCandidate * second) {
        return first->source_closest_range_m < second->source_closest_range_m;
      });

    const std::size_t logged_count =
      std::min(geometry_log_cluster_count_, ordered_candidates.size());
    for (std::size_t index = 0U; index < logged_count; ++index) {
      const auto & candidate = *ordered_candidates[index];
      RCLCPP_INFO(
        get_logger(),
        "  circle[%zu] sensor_center=(%.3f,%.3f) radius_m=%.3f "
        "source_closest_range_m=%.3f points=%zu scan_indices=[%zu..%zu] "
        "world_center_valid=%d world_center=(%.3f,%.3f) in_track_ring=%d",
        index, candidate.sensor_x_m, candidate.sensor_y_m, candidate.radius_m,
        candidate.source_closest_range_m, candidate.point_count,
        candidate.first_scan_index, candidate.last_scan_index,
        candidate.world_center_valid ? 1 : 0,
        candidate.world_x_m, candidate.world_y_m,
        candidate.inside_track_ring ? 1 : 0);
    }
  }

  ScanValidationOptions validation_options_;
  ScanClusteringOptions clustering_options_;
  NeighborFilterOptions filter_options_;
  ClusterCandidateOptions candidate_options_;
  ClusterGeometryOptions geometry_options_;
  ConeOccupancyOptions occupancy_options_;
  SensorMountOptions mount_options_;
  TrackRoiOptions roi_options_;
  TrackRing track_ring_;
  bool log_cluster_geometry_{true};
  bool log_circle_candidates_{true};
  std::string world_pose_base_frame_{"base_footprint"};
  std::string world_pose_transport_state_{"unavailable_at_build_time"};
  double world_pose_timeout_s_{0.5};
  mutable std::mutex world_pose_mutex_;
  WorldPoseSample latest_world_pose_;
  std::size_t world_pose_message_count_since_summary_{0U};
#ifdef KAU_OBJECT_DETECTION_WITH_GZ_TRANSPORT
  std::unique_ptr<gz::transport::Node> gz_node_;
#endif
  std::size_t geometry_log_cluster_count_{3U};
  double summary_period_seconds_{2.0};
  std::size_t valid_message_count_since_summary_{0U};
  std::size_t invalid_message_count_{0U};
  std::size_t invalid_geometry_count_{0U};
  std::chrono::steady_clock::time_point last_summary_time_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;

  bool temporal_tracking_enabled_{true};
  ObstacleTracker obstacle_tracker_;

  ObjectListOptions object_list_options_;
  /// Simulator-only Gazebo pose freshness rule. Diagnostics only.
  double object_list_pose_timeout_s_{0.3};
  double object_list_scan_timeout_s_{0.3};
  double object_list_transform_timeout_s_{0.05};
  /// tf2 source of the published Object List coordinates.
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  double watchdog_publish_period_s_{0.2};
  /// Node clock, so the watchdog follows simulation time.
  rclcpp::Time last_scan_activity_{0, 0U, RCL_ROS_TIME};
  rclcpp::Time last_object_list_publish_{0, 0U, RCL_ROS_TIME};
  ObjectListStatus last_object_list_status_{ObjectListStatus::kInternalError};
  rclcpp::Publisher<kau_msgs::msg::ObstacleCircleArray>::SharedPtr object_list_publisher_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
};

}  // namespace kau_object_detection

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<kau_object_detection::LaserScanClustererNode>());
  rclcpp::shutdown();
  return 0;
}
