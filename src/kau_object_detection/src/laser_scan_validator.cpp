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

#include "kau_object_detection/laser_scan_validator.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <limits>
#include <string>

namespace kau_object_detection
{
namespace
{

ScanValidationResult invalid_result(const std::string & reason)
{
  ScanValidationResult result;
  result.reason = reason;
  return result;
}

bool is_finite(const float value)
{
  return std::isfinite(static_cast<double>(value));
}

}  // namespace

ScanValidationResult validate_laser_scan(
  const sensor_msgs::msg::LaserScan & scan,
  const ScanValidationOptions & options)
{
  if (scan.header.frame_id.empty()) {
    return invalid_result("frame_id is empty");
  }

  if (!options.expected_frame_id.empty() &&
    scan.header.frame_id != options.expected_frame_id)
  {
    return invalid_result(
      "frame_id does not match expected frame " + options.expected_frame_id);
  }

  if (scan.ranges.size() < options.minimum_sample_count) {
    return invalid_result("ranges contains fewer samples than configured minimum");
  }

  if (options.require_intensities && scan.intensities.size() != scan.ranges.size()) {
    return invalid_result("intensities size does not match ranges size");
  }

  if (!scan.intensities.empty() && scan.intensities.size() != scan.ranges.size()) {
    return invalid_result("non-empty intensities size does not match ranges size");
  }

  if (!is_finite(scan.angle_min) || !is_finite(scan.angle_max) ||
    !is_finite(scan.angle_increment))
  {
    return invalid_result("angle metadata contains a non-finite value");
  }

  if (scan.angle_max <= scan.angle_min) {
    return invalid_result("angle_max must be greater than angle_min");
  }

  if (scan.angle_increment <= 0.0F) {
    return invalid_result("angle_increment must be positive");
  }

  const double interval_count =
    static_cast<double>(scan.angle_max - scan.angle_min) /
    static_cast<double>(scan.angle_increment);
  if (!std::isfinite(interval_count) || interval_count < 0.0) {
    return invalid_result("angle metadata does not define a usable scan geometry");
  }

  const auto rounded_interval_count = std::llround(interval_count);
  if (rounded_interval_count < 0) {
    return invalid_result("angle metadata produced a negative sample count");
  }

  const auto expected_sample_count =
    static_cast<std::size_t>(rounded_interval_count) + 1U;
  const auto count_difference =
    expected_sample_count > scan.ranges.size() ?
    expected_sample_count - scan.ranges.size() :
    scan.ranges.size() - expected_sample_count;
  if (count_difference > options.sample_count_tolerance) {
    return invalid_result("ranges size is inconsistent with angle metadata");
  }

  if (!is_finite(scan.range_min) || !is_finite(scan.range_max)) {
    return invalid_result("range metadata contains a non-finite value");
  }

  if (scan.range_min < 0.0F || scan.range_max <= scan.range_min) {
    return invalid_result("range_min and range_max do not define a valid interval");
  }

  if (!is_finite(scan.scan_time) || !is_finite(scan.time_increment) ||
    scan.scan_time < 0.0F || scan.time_increment < 0.0F)
  {
    return invalid_result("scan timing metadata must be finite and non-negative");
  }

  ScanValidationResult result;
  result.valid = true;
  result.reason = "valid";
  result.usable_indices.reserve(scan.ranges.size());

  for (std::size_t index = 0U; index < scan.ranges.size(); ++index) {
    const float range = scan.ranges[index];
    if (!is_finite(range)) {
      ++result.non_finite_count;
    } else if (range < scan.range_min) {
      ++result.below_minimum_count;
    } else if (range > scan.range_max) {
      ++result.above_maximum_count;
    } else {
      result.usable_indices.push_back(index);
    }
  }

  return result;
}

}  // namespace kau_object_detection
