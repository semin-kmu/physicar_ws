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
// Forked from package "kau_object_detection", its header laser_scan_validator.hpp, on 2026-08-25.
//
// This is an independent copy, not a link: kau_object_detection_lane builds and
// runs without that package being present. Fixes made in one package do
// NOT propagate to the other. See README.md "Source synchronisation".
//
// Changes from the original: namespace and include guard renamed to
// kau_object_detection_lane, include paths repointed at this package.
// ---------------------------------------------------------------------------

#ifndef KAU_OBJECT_DETECTION_LANE__SCAN_VALIDATION_HPP_
#define KAU_OBJECT_DETECTION_LANE__SCAN_VALIDATION_HPP_

#include <cstddef>
#include <string>
#include <vector>

#include "sensor_msgs/msg/laser_scan.hpp"

namespace kau_object_detection_lane
{

struct ScanValidationOptions
{
  std::string expected_frame_id;
  std::size_t minimum_sample_count{2U};
  std::size_t sample_count_tolerance{1U};
  bool require_intensities{false};
};

struct ScanValidationResult
{
  bool valid{false};
  std::string reason;
  std::vector<std::size_t> usable_indices;
  std::size_t non_finite_count{0U};
  std::size_t below_minimum_count{0U};
  std::size_t above_maximum_count{0U};
};

ScanValidationResult validate_laser_scan(
  const sensor_msgs::msg::LaserScan & scan,
  const ScanValidationOptions & options = ScanValidationOptions{});

}  // namespace kau_object_detection_lane

#endif  // KAU_OBJECT_DETECTION_LANE__SCAN_VALIDATION_HPP_
