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
#include <cstddef>
#include <limits>
#include <string>

#include "gtest/gtest.h"
#include "kau_object_detection/laser_scan_validator.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace
{

constexpr float kPi = 3.14159265358979323846F;

sensor_msgs::msg::LaserScan make_scan(const std::size_t sample_count = 720U)
{
  sensor_msgs::msg::LaserScan scan;
  scan.header.frame_id = "lidar_link";
  scan.header.stamp.sec = 42;
  scan.angle_min = -kPi;
  scan.angle_max = kPi;
  scan.angle_increment =
    (scan.angle_max - scan.angle_min) / static_cast<float>(sample_count - 1U);
  scan.time_increment = 0.0F;
  scan.scan_time = 0.0F;
  scan.range_min = 0.1F;
  scan.range_max = 16.0F;
  scan.ranges.assign(sample_count, 2.0F);
  scan.intensities.assign(sample_count, 1.0F);
  return scan;
}

kau_object_detection::ScanValidationOptions physicar_options()
{
  kau_object_detection::ScanValidationOptions options;
  options.expected_frame_id = "lidar_link";
  options.minimum_sample_count = 2U;
  options.sample_count_tolerance = 1U;
  options.require_intensities = false;
  return options;
}

TEST(LaserScanValidator, AcceptsObservedPhysicarGeometryAndZeroTiming)
{
  const auto scan = make_scan();
  const auto result =
    kau_object_detection::validate_laser_scan(scan, physicar_options());

  ASSERT_TRUE(result.valid) << result.reason;
  EXPECT_EQ(result.usable_indices.size(), 720U);
  EXPECT_EQ(result.usable_indices.front(), 0U);
  EXPECT_EQ(result.usable_indices.back(), 719U);
  EXPECT_EQ(result.non_finite_count, 0U);
}

TEST(LaserScanValidator, RejectsEmptyFrame)
{
  auto scan = make_scan();
  scan.header.frame_id.clear();

  const auto result =
    kau_object_detection::validate_laser_scan(scan, physicar_options());
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.reason, "frame_id is empty");
}

TEST(LaserScanValidator, RejectsUnexpectedFrame)
{
  auto scan = make_scan();
  scan.header.frame_id = "other_lidar";

  const auto result =
    kau_object_detection::validate_laser_scan(scan, physicar_options());
  EXPECT_FALSE(result.valid);
  EXPECT_NE(result.reason.find("expected frame"), std::string::npos);
}

TEST(LaserScanValidator, RejectsInvalidAngleGeometry)
{
  auto scan = make_scan();
  scan.angle_increment = 0.0F;

  const auto result =
    kau_object_detection::validate_laser_scan(scan, physicar_options());
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.reason, "angle_increment must be positive");
}

TEST(LaserScanValidator, RejectsInconsistentSampleCount)
{
  auto scan = make_scan();
  scan.ranges.resize(100U);
  scan.intensities.resize(100U);

  const auto result =
    kau_object_detection::validate_laser_scan(scan, physicar_options());
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.reason, "ranges size is inconsistent with angle metadata");
}

TEST(LaserScanValidator, AllowsEmptyIntensitiesWhenNotRequired)
{
  auto scan = make_scan();
  scan.intensities.clear();

  const auto result =
    kau_object_detection::validate_laser_scan(scan, physicar_options());
  EXPECT_TRUE(result.valid) << result.reason;
}

TEST(LaserScanValidator, RejectsMismatchedNonEmptyIntensities)
{
  auto scan = make_scan();
  scan.intensities.resize(10U);

  const auto result =
    kau_object_detection::validate_laser_scan(scan, physicar_options());
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.reason, "non-empty intensities size does not match ranges size");
}

TEST(LaserScanValidator, ClassifiesInvalidBeamsAndPreservesIndices)
{
  auto scan = make_scan(8U);
  scan.ranges = {
    1.0F,
    std::numeric_limits<float>::quiet_NaN(),
    std::numeric_limits<float>::infinity(),
    -std::numeric_limits<float>::infinity(),
    0.01F,
    16.1F,
    0.1F,
    16.0F};

  const auto result =
    kau_object_detection::validate_laser_scan(scan, physicar_options());

  ASSERT_TRUE(result.valid) << result.reason;
  EXPECT_EQ(result.non_finite_count, 3U);
  EXPECT_EQ(result.below_minimum_count, 1U);
  EXPECT_EQ(result.above_maximum_count, 1U);
  ASSERT_EQ(result.usable_indices.size(), 3U);
  EXPECT_EQ(result.usable_indices[0], 0U);
  EXPECT_EQ(result.usable_indices[1], 6U);
  EXPECT_EQ(result.usable_indices[2], 7U);
}

TEST(LaserScanValidator, DoesNotModifyInputMessage)
{
  const auto scan = make_scan();
  const auto original_ranges = scan.ranges;
  const auto original_stamp = scan.header.stamp;

  const auto result =
    kau_object_detection::validate_laser_scan(scan, physicar_options());

  ASSERT_TRUE(result.valid) << result.reason;
  EXPECT_EQ(scan.ranges, original_ranges);
  EXPECT_EQ(scan.header.stamp.sec, original_stamp.sec);
  EXPECT_EQ(scan.header.stamp.nanosec, original_stamp.nanosec);
}

}  // namespace
