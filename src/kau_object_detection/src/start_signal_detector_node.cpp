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

#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <opencv2/imgcodecs.hpp>

#include "kau_object_detection/green_lamp_detector.hpp"
#include "kau_object_detection/start_signal_logic.hpp"
#include "rclcpp/create_timer.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "std_msgs/msg/bool.hpp"

namespace kau_object_detection
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

    StartSignalOptions signal_options;
    signal_options.green_confirmation_frames = static_cast<int>(
      declare_parameter<std::int64_t>("green_confirmation_frames", 5));
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
      "qos_out=reliable/keep_last(1)/volatile green_confirmation_frames=%d "
      "camera_timeout_s=%.3f publish_period_s=%.3f",
      input_topic.c_str(), output_topic.c_str(),
      signal_options.green_confirmation_frames, signal_options.camera_timeout_s,
      publish_period_s);
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
      report_green_streak();
      return;
    }

    const auto detection = detect_green_lamp(frame, lamp_options_);
    latch_->observe_frame(
      detection.detected ? FrameOutcome::kGreen : FrameOutcome::kNotGreen, now_s);

    if (log_detection_detail_) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "frame %dx%d candidates=%d detected=%d area=%d aspect=%.2f fill=%.2f "
        "center=(%d,%d) consecutive_green=%d",
        frame.cols, frame.rows, detection.candidate_count, detection.detected ? 1 : 0,
        detection.area_px, detection.aspect_ratio, detection.fill_ratio,
        detection.center_x, detection.center_y, latch_->consecutive_green_frames());
    }

    report_state_change();
    report_green_streak();
  }

  void publish_timer_callback()
  {
    latch_->advance_time(now().seconds());

    std_msgs::msg::Bool message;
    message.data = latch_->start_permitted();
    permission_publisher_->publish(message);

    report_state_change();
    report_green_streak();
  }

  /// Read-only progress line for the operator watching the terminal.
  ///
  /// `green_streak` is the length of the current run of consecutive green
  /// frames, never a running total: a single non-green frame puts it back to
  /// zero. It is printed the moment the value moves and otherwise once a
  /// second, so the terminal shows the live state without one line per frame.
  ///
  /// Nothing here touches the published value, the topic, the QoS or the latch.
  void report_green_streak()
  {
    const int streak = latch_->consecutive_green_frames();
    const int required = latch_->options().green_confirmation_frames;

    if (streak != reported_streak_) {
      reported_streak_ = streak;
      if (latch_->latched()) {
        RCLCPP_INFO(
          get_logger(), "start_permission=true (latched, green_streak=[%d/%d])",
          streak, required);
      } else {
        RCLCPP_INFO(
          get_logger(), "start_permission=false green_streak=[%d/%d]", streak, required);
      }
      return;
    }

    // Unchanged value: keep a once-a-second heartbeat so a stalled run is still
    // visible. After the latch the streak is frozen, so this is the only line
    // that keeps reporting.
    if (latch_->latched()) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "start_permission=true (latched, green_streak=[%d/%d])", streak, required);
    } else {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "start_permission=false green_streak=[%d/%d]", streak, required);
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
        "start permission latched after %d consecutive green frames; "
        "publishing true from now on",
        latch_->consecutive_green_frames());
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
  int reported_streak_{0};
  bool reported_camera_active_{false};
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr image_subscription_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr permission_publisher_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

}  // namespace kau_object_detection

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<kau_object_detection::StartSignalDetectorNode>());
  rclcpp::shutdown();
  return 0;
}
