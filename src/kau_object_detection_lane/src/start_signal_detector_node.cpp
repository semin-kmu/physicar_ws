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
// Forked from package "kau_object_detection", its source
// start_signal_detector_node.cpp, on 2026-08-25. This is an independent copy,
// not a link. See README.md "Source synchronisation".
//
// The spatial detection algorithm and its ROI / HSV / area / aspect / fill
// gates are carried over UNCHANGED and deliberately so. They live in
// green_lamp_detector and nothing here touches them.
//
// 2026-08-25: the TEMPORAL confirmation policy did change. The original
// required `green_confirmation_frames` consecutive greens, so one dropped or
// mis-classified frame reset the count and the vehicle sometimes never left the
// line. It now latches on `green_required_frames` greens within the last
// `green_window_frames` usable frames. Only the window changed, so the effect
// of that change can be attributed to it alone.
//
// Topic, message type, QoS, publish period, camera timeout and the permanent
// latch are all unchanged. Wider traffic-light hardening - ROI adjustment,
// RGB/HSV combination, black panel checks, dynamic ROI - is a separate task
// that has NOT been done here.
// ---------------------------------------------------------------------------

#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <opencv2/imgcodecs.hpp>

#include "kau_object_detection_lane/green_lamp_detector.hpp"
#include "kau_object_detection_lane/start_signal_logic.hpp"
#include "rclcpp/create_timer.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "std_msgs/msg/bool.hpp"

namespace kau_object_detection_lane
{

/// Publishes the one-time permission to leave the start line.
///
/// The camera is used for this and nothing else; obstacle perception stays
/// LiDAR-first. The node never commands motion, it only states whether the
/// start signal has been seen.
class StartSignalDetectorNode : public rclcpp::Node
{
public:
  StartSignalDetectorNode()
  : Node("start_signal_detector")
  {
    const auto input_topic = declare_parameter<std::string>(
      "input_topic", "/camera/image_raw/compressed");
    const auto output_topic = declare_parameter<std::string>(
      "output_topic", "/perception/start_permission");

    // Sliding-window confirmation. Declared as int64 because that is the ROS
    // integer parameter type, then range-checked before the narrowing cast so a
    // value outside int cannot wrap into something that passes the latch's own
    // validation.
    const auto green_window_frames =
      declare_parameter<std::int64_t>("green_window_frames", 8);
    const auto green_required_frames =
      declare_parameter<std::int64_t>("green_required_frames", 4);

    StartSignalOptions signal_options;
    signal_options.green_window_frames = to_frame_count(
      green_window_frames, "green_window_frames");
    signal_options.green_required_frames = to_frame_count(
      green_required_frames, "green_required_frames");
    signal_options.camera_timeout_s =
      declare_parameter<double>("camera_timeout_s", 0.5);
    const auto publish_period_s =
      declare_parameter<double>("publish_period_s", 0.1);

    lamp_options_.roi.x_min = static_cast<int>(declare_parameter<std::int64_t>("roi_x_min", 280));
    lamp_options_.roi.x_max = static_cast<int>(declare_parameter<std::int64_t>("roi_x_max", 450));
    lamp_options_.roi.y_min = static_cast<int>(declare_parameter<std::int64_t>("roi_y_min", 135));
    lamp_options_.roi.y_max = static_cast<int>(declare_parameter<std::int64_t>("roi_y_max", 265));
    lamp_options_.hue_min = static_cast<int>(declare_parameter<std::int64_t>("hue_min", 45));
    lamp_options_.hue_max = static_cast<int>(declare_parameter<std::int64_t>("hue_max", 85));
    lamp_options_.saturation_min =
      static_cast<int>(declare_parameter<std::int64_t>("saturation_min", 150));
    lamp_options_.value_min =
      static_cast<int>(declare_parameter<std::int64_t>("value_min", 150));
    lamp_options_.min_area_px =
      static_cast<int>(declare_parameter<std::int64_t>("min_area_px", 60));
    lamp_options_.max_area_px =
      static_cast<int>(declare_parameter<std::int64_t>("max_area_px", 5000));
    lamp_options_.min_aspect_ratio = declare_parameter<double>("min_aspect_ratio", 0.6);
    lamp_options_.max_aspect_ratio = declare_parameter<double>("max_aspect_ratio", 1.6);
    lamp_options_.min_fill_ratio = declare_parameter<double>("min_fill_ratio", 0.6);
    log_detection_detail_ = declare_parameter<bool>("log_detection_detail", false);

    if (!std::isfinite(publish_period_s) || publish_period_s <= 0.0) {
      throw std::invalid_argument("publish_period_s must be finite and positive");
    }

    // Both throw on a configuration that cannot describe a usable policy, so a
    // typo in YAML stops the node instead of silently forbidding the start.
    validate_green_lamp_options(lamp_options_);
    latch_ = std::make_unique<StartPermissionLatch>(signal_options);

    image_subscription_ = create_subscription<sensor_msgs::msg::CompressedImage>(
      input_topic,
      rclcpp::SensorDataQoS(),
      std::bind(&StartSignalDetectorNode::image_callback, this, std::placeholders::_1));

    // The state machine owns a decision the vehicle must not miss, so the
    // permission is delivered reliably and only the newest value matters.
    permission_publisher_ = create_publisher<std_msgs::msg::Bool>(
      output_topic,
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile());

    // Node clock, never a wall timer, so the timeout follows simulation time.
    publish_timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration::from_seconds(publish_period_s),
      std::bind(&StartSignalDetectorNode::publish_timer_callback, this));

    RCLCPP_INFO(
      get_logger(),
      "ready input=%s output=%s qos_in=sensor_data(best_effort) "
      "qos_out=reliable/keep_last(1)/volatile green_window_frames=%d "
      "green_required_frames=%d camera_timeout_s=%.3f publish_period_s=%.3f",
      input_topic.c_str(), output_topic.c_str(),
      signal_options.green_window_frames, signal_options.green_required_frames,
      signal_options.camera_timeout_s, publish_period_s);
    RCLCPP_INFO(
      get_logger(),
      "start confirmation: %d green frames within the last %d usable frames. "
      "Not consecutive: a dropped or mis-classified frame no longer resets the "
      "count. A camera timeout clears the window; the latch is permanent",
      signal_options.green_required_frames, signal_options.green_window_frames);
    RCLCPP_INFO(
      get_logger(),
      "green lamp gate roi=[%d,%d]x[%d,%d] hue=[%d,%d] saturation_min=%d "
      "value_min=%d area=[%d,%d] aspect=[%.2f,%.2f] min_fill_ratio=%.2f",
      lamp_options_.roi.x_min, lamp_options_.roi.x_max,
      lamp_options_.roi.y_min, lamp_options_.roi.y_max,
      lamp_options_.hue_min, lamp_options_.hue_max,
      lamp_options_.saturation_min, lamp_options_.value_min,
      lamp_options_.min_area_px, lamp_options_.max_area_px,
      lamp_options_.min_aspect_ratio, lamp_options_.max_aspect_ratio,
      lamp_options_.min_fill_ratio);
  }

private:
  /// Narrows one ROS integer parameter to the `int` the policy uses.
  ///
  /// The latch validates the values themselves (at least 1, required at most
  /// window). This only rejects magnitudes `int` cannot hold, which would
  /// otherwise wrap into a value that passes that validation while meaning
  /// something entirely different.
  static int to_frame_count(const std::int64_t value, const char * name)
  {
    if (value < 1 || value > std::numeric_limits<int>::max()) {
      throw std::invalid_argument(
              std::string(name) + " must be at least 1 and fit in an int");
    }
    return static_cast<int>(value);
  }

  void image_callback(const sensor_msgs::msg::CompressedImage::ConstSharedPtr image)
  {
    const double now_s = now().seconds();

    // cv::imdecode reads the JPEG payload straight out of the message, so no
    // image transport plugin or bridge is involved.
    const cv::Mat encoded(1, static_cast<int>(image->data.size()), CV_8UC1,
      const_cast<std::uint8_t *>(image->data.data()));
    const cv::Mat frame = cv::imdecode(encoded, cv::IMREAD_COLOR);

    if (frame.empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "could not decode compressed image format=%s bytes=%zu",
        image->format.c_str(), image->data.size());
      latch_->observe_frame(FrameOutcome::kUnusable, now_s);
      report_state_change();
      report_green_window();
      return;
    }

    const auto detection = detect_green_lamp(frame, lamp_options_);
    latch_->observe_frame(
      detection.detected ? FrameOutcome::kGreen : FrameOutcome::kNotGreen, now_s);

    if (log_detection_detail_) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "frame %dx%d candidates=%d detected=%d area=%d aspect=%.2f fill=%.2f "
        "center=(%d,%d) green_hits=%d/%d samples=%d",
        frame.cols, frame.rows, detection.candidate_count, detection.detected ? 1 : 0,
        detection.area_px, detection.aspect_ratio, detection.fill_ratio,
        detection.center_x, detection.center_y, latch_->green_hits(),
        latch_->options().green_window_frames, latch_->window_samples());
    }

    report_state_change();
    report_green_window();
  }

  void publish_timer_callback()
  {
    latch_->advance_time(now().seconds());

    std_msgs::msg::Bool message;
    message.data = latch_->start_permitted();
    permission_publisher_->publish(message);

    report_state_change();
    report_green_window();
  }

  /// Read-only progress line for the operator watching the terminal.
  ///
  /// `green_window` is how many of the samples currently inside the window were
  /// green, never a running total of the whole run: a green that has slid out
  /// of the window no longer counts. `samples` is how many samples the window
  /// holds, so a low hit count early on reads as "not enough frames yet" rather
  /// than "not green". It is printed the moment the hit count moves and
  /// otherwise once a second, which is the same throttle policy the
  /// consecutive-run version used.
  ///
  /// Nothing here touches the published value, the topic, the QoS or the latch.
  void report_green_window()
  {
    const int hits = latch_->green_hits();
    const int window = latch_->options().green_window_frames;
    const int required = latch_->options().green_required_frames;
    const int samples = latch_->window_samples();

    if (hits != reported_hits_) {
      reported_hits_ = hits;
      if (latch_->latched()) {
        RCLCPP_INFO(
          get_logger(), "start_permission=true (latched, green_hits=%d/%d)",
          hits, window);
      } else {
        RCLCPP_INFO(
          get_logger(),
          "start_permission=false green_window=[%d/%d] required=%d samples=%d",
          hits, window, required, samples);
      }
      return;
    }

    // Unchanged value: keep a once-a-second heartbeat so a stalled window is
    // still visible. After the latch the window is frozen, so this is the only
    // line that keeps reporting.
    if (latch_->latched()) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "start_permission=true (latched, green_hits=%d/%d)", hits, window);
    } else {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "start_permission=false green_window=[%d/%d] required=%d samples=%d",
        hits, window, required, samples);
    }
  }

  /// Logs only when something actually changed, so a node that runs for the
  /// whole race does not bury its two interesting lines in frame-rate noise.
  void report_state_change()
  {
    if (latch_->latched() != reported_latched_) {
      reported_latched_ = latch_->latched();
      RCLCPP_INFO(
        get_logger(),
        "start permission latched: green detected in %d of the last %d usable "
        "frames; publishing true from now on",
        latch_->green_hits(), latch_->options().green_window_frames);
    }

    if (latch_->camera_active() != reported_camera_active_) {
      reported_camera_active_ = latch_->camera_active();
      if (reported_camera_active_) {
        RCLCPP_INFO(get_logger(), "camera stream active");
      } else {
        RCLCPP_WARN(
          get_logger(),
          "no usable camera frame within %.3f s; start permission stays %s",
          latch_->options().camera_timeout_s, latch_->latched() ? "true" : "false");
      }
    }
  }

  GreenLampOptions lamp_options_;
  std::unique_ptr<StartPermissionLatch> latch_;
  bool log_detection_detail_{false};
  bool reported_latched_{false};
  int reported_hits_{0};
  bool reported_camera_active_{false};
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr image_subscription_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr permission_publisher_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

}  // namespace kau_object_detection_lane

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<kau_object_detection_lane::StartSignalDetectorNode>());
  rclcpp::shutdown();
  return 0;
}
