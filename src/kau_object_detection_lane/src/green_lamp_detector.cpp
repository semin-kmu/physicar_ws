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
// Forked from package "kau_object_detection", its source green_lamp_detector.cpp, on 2026-08-25.
//
// This is an independent copy, not a link: kau_object_detection_lane builds and
// runs without that package being present. Fixes made in one package do
// NOT propagate to the other. See README.md "Source synchronisation".
//
// Changes from the original: namespace and include guard renamed to
// kau_object_detection_lane, include paths repointed at this package.
// ---------------------------------------------------------------------------

#include "kau_object_detection_lane/green_lamp_detector.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <opencv2/imgproc.hpp>

namespace kau_object_detection_lane
{

void validate_green_lamp_options(const GreenLampOptions & options)
{
  if (options.roi.x_min < 0 || options.roi.y_min < 0 ||
    options.roi.x_max <= options.roi.x_min ||
    options.roi.y_max <= options.roi.y_min)
  {
    throw std::invalid_argument(
            "green lamp roi must be a non-empty window with non-negative origin");
  }
  if (options.hue_min < 0 || options.hue_max > 179 || options.hue_min > options.hue_max) {
    throw std::invalid_argument("hue gate must satisfy 0 <= hue_min <= hue_max <= 179");
  }
  if (options.saturation_min < 0 || options.saturation_min > 255 ||
    options.value_min < 0 || options.value_min > 255)
  {
    throw std::invalid_argument("saturation_min and value_min must be within 0..255");
  }
  if (options.min_area_px < 1 || options.max_area_px < options.min_area_px) {
    throw std::invalid_argument("area gate must satisfy 1 <= min_area_px <= max_area_px");
  }
  if (!std::isfinite(options.min_aspect_ratio) || !std::isfinite(options.max_aspect_ratio) ||
    options.min_aspect_ratio <= 0.0 || options.max_aspect_ratio < options.min_aspect_ratio)
  {
    throw std::invalid_argument(
            "aspect gate must satisfy 0 < min_aspect_ratio <= max_aspect_ratio");
  }
  if (!std::isfinite(options.min_fill_ratio) ||
    options.min_fill_ratio < 0.0 || options.min_fill_ratio > 1.0)
  {
    throw std::invalid_argument("min_fill_ratio must be within 0..1");
  }
}

GreenLampDetection detect_green_lamp(
  const cv::Mat & bgr,
  const GreenLampOptions & options)
{
  GreenLampDetection detection;

  if (bgr.empty() || bgr.type() != CV_8UC3) {
    return detection;
  }

  // Clip the configured window to the frame. A window that falls entirely
  // outside leaves nothing to inspect, which reads as "not green".
  const int x_min = std::max(0, options.roi.x_min);
  const int y_min = std::max(0, options.roi.y_min);
  const int x_max = std::min(bgr.cols, options.roi.x_max);
  const int y_max = std::min(bgr.rows, options.roi.y_max);
  if (x_max <= x_min || y_max <= y_min) {
    return detection;
  }

  const cv::Rect window(x_min, y_min, x_max - x_min, y_max - y_min);
  const cv::Mat roi = bgr(window);

  cv::Mat hsv;
  cv::cvtColor(roi, hsv, cv::COLOR_BGR2HSV);

  cv::Mat mask;
  cv::inRange(
    hsv,
    cv::Scalar(options.hue_min, options.saturation_min, options.value_min),
    cv::Scalar(options.hue_max, 255, 255),
    mask);

  cv::Mat labels;
  cv::Mat stats;
  cv::Mat centroids;
  const int component_count =
    cv::connectedComponentsWithStats(mask, labels, stats, centroids, 8, CV_32S);

  // Label 0 is the background.
  detection.candidate_count = std::max(0, component_count - 1);

  int best_area = 0;
  for (int label = 1; label < component_count; ++label) {
    const int area = stats.at<int>(label, cv::CC_STAT_AREA);
    if (area < options.min_area_px || area > options.max_area_px) {
      continue;
    }

    const int width = stats.at<int>(label, cv::CC_STAT_WIDTH);
    const int height = stats.at<int>(label, cv::CC_STAT_HEIGHT);
    if (width <= 0 || height <= 0) {
      continue;
    }

    const double aspect_ratio = static_cast<double>(width) / static_cast<double>(height);
    if (aspect_ratio < options.min_aspect_ratio || aspect_ratio > options.max_aspect_ratio) {
      continue;
    }

    const double fill_ratio =
      static_cast<double>(area) / static_cast<double>(width * height);
    if (fill_ratio < options.min_fill_ratio) {
      continue;
    }

    if (area <= best_area) {
      continue;
    }

    best_area = area;
    detection.detected = true;
    detection.area_px = area;
    detection.aspect_ratio = aspect_ratio;
    detection.fill_ratio = fill_ratio;
    detection.center_x =
      window.x + static_cast<int>(std::lround(centroids.at<double>(label, 0)));
    detection.center_y =
      window.y + static_cast<int>(std::lround(centroids.at<double>(label, 1)));
  }

  return detection;
}

}  // namespace kau_object_detection_lane
