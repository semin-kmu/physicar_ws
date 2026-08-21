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

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include "kau_object_detection/laser_scan_validator.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace kau_object_detection
{

class LaserScanValidatorNode : public rclcpp::Node
{
public:
  LaserScanValidatorNode()
  : Node("laser_scan_validator"), last_summary_time_(std::chrono::steady_clock::now())
  {
    const auto input_topic = declare_parameter<std::string>("input_topic", "/scan_filtered");
    options_.expected_frame_id =
      declare_parameter<std::string>("expected_frame_id", "lidar_link");

    const auto minimum_sample_count =
      declare_parameter<std::int64_t>("minimum_sample_count", 2);
    const auto sample_count_tolerance =
      declare_parameter<std::int64_t>("sample_count_tolerance", 1);
    options_.require_intensities =
      declare_parameter<bool>("require_intensities", false);
    summary_period_seconds_ =
      declare_parameter<double>("summary_period_seconds", 2.0);

    if (minimum_sample_count < 2) {
      throw std::invalid_argument("minimum_sample_count must be at least 2");
    }
    if (sample_count_tolerance < 0) {
      throw std::invalid_argument("sample_count_tolerance must be non-negative");
    }
    if (summary_period_seconds_ <= 0.0) {
      throw std::invalid_argument("summary_period_seconds must be positive");
    }

    options_.minimum_sample_count = static_cast<std::size_t>(minimum_sample_count);
    options_.sample_count_tolerance = static_cast<std::size_t>(sample_count_tolerance);

    scan_subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
      input_topic,
      rclcpp::SensorDataQoS(),
      std::bind(&LaserScanValidatorNode::scan_callback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "ready input=%s expected_frame=%s qos=sensor_data(best_effort)",
      input_topic.c_str(), options_.expected_frame_id.c_str());
  }

private:
  void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan)
  {
    const auto result = validate_laser_scan(*scan, options_);
    if (!result.valid) {
      ++invalid_message_count_;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "invalid scan frame=%s samples=%zu reason=%s",
        scan->header.frame_id.c_str(), scan->ranges.size(), result.reason.c_str());
      return;
    }

    ++valid_message_count_since_summary_;
    const auto now = std::chrono::steady_clock::now();
    const double elapsed_seconds =
      std::chrono::duration<double>(now - last_summary_time_).count();
    if (elapsed_seconds < summary_period_seconds_) {
      return;
    }

    const double measured_rate =
      static_cast<double>(valid_message_count_since_summary_) / elapsed_seconds;
    RCLCPP_INFO(
      get_logger(),
      "valid scan frame=%s samples=%zu usable=%zu non_finite=%zu below_min=%zu "
      "above_max=%zu rate_hz=%.2f invalid_messages=%zu",
      scan->header.frame_id.c_str(), scan->ranges.size(), result.usable_indices.size(),
      result.non_finite_count, result.below_minimum_count, result.above_maximum_count,
      measured_rate, invalid_message_count_);

    valid_message_count_since_summary_ = 0U;
    last_summary_time_ = now;
  }

  ScanValidationOptions options_;
  double summary_period_seconds_{2.0};
  std::size_t valid_message_count_since_summary_{0U};
  std::size_t invalid_message_count_{0U};
  std::chrono::steady_clock::time_point last_summary_time_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
};

}  // namespace kau_object_detection

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<kau_object_detection::LaserScanValidatorNode>());
  rclcpp::shutdown();
  return 0;
}
