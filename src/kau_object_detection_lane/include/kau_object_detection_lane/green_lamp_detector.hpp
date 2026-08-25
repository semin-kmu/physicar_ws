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
// Forked from package "kau_object_detection", its header green_lamp_detector.hpp, on 2026-08-25.
//
// This is an independent copy, not a link: kau_object_detection_lane builds and
// runs without that package being present. Fixes made in one package do
// NOT propagate to the other. See README.md "Source synchronisation".
//
// Changes from the original: namespace and include guard renamed to
// kau_object_detection_lane, include paths repointed at this package.
// ---------------------------------------------------------------------------

#ifndef KAU_OBJECT_DETECTION_LANE__GREEN_LAMP_DETECTOR_HPP_
#define KAU_OBJECT_DETECTION_LANE__GREEN_LAMP_DETECTOR_HPP_

#include <opencv2/core.hpp>

namespace kau_object_detection_lane
{

/// Fixed image window the start signal is searched in, in pixels of the full
/// camera frame. The vehicle stands at a known start pose, so the signal always
/// falls in the same place and a fixed window is enough. Everything outside it
/// is ignored, which is what keeps roadside grass out of the decision.
struct GreenLampRoi
{
  int x_min{280};
  /// Wide enough to clear the lamp measured at bbox (412, 217, 28x28) on the
  /// real camera. A narrower window clips the lamp and the area gate then
  /// rejects the sliver that is left.
  int x_max{450};
  int y_min{135};
  int y_max{265};
};

/// Colour and shape gates of the lit green lamp. Every value is a tunable and
/// belongs in YAML; the defaults here only keep the struct usable on its own.
struct GreenLampOptions
{
  GreenLampRoi roi;

  /// OpenCV hue is 0..179, so these are half of the usual degree values.
  int hue_min{45};
  int hue_max{85};
  int saturation_min{150};
  int value_min{150};

  /// Connected component size gates, in pixels.
  int min_area_px{60};
  int max_area_px{5000};

  /// Bounding box width over height. A lamp is round, so this sits near 1.
  double min_aspect_ratio{0.6};
  double max_aspect_ratio{1.6};

  /// Component area over bounding box area. A filled circle inscribed in its
  /// bounding box reaches pi/4, about 0.785; scattered foliage stays far below.
  double min_fill_ratio{0.6};
};

/// Outcome of one frame. `detected` is the only value the state machine needs;
/// the rest exists so a human can tune the gates from the logs.
struct GreenLampDetection
{
  bool detected{false};

  /// Largest component that passed every gate.
  int area_px{0};
  double aspect_ratio{0.0};
  double fill_ratio{0.0};

  /// Centre of the accepted component in full-frame pixel coordinates.
  int center_x{0};
  int center_y{0};

  /// Components that passed the colour mask before the shape gates ran.
  int candidate_count{0};
};

/// Throws std::invalid_argument when the gates cannot describe a usable lamp.
void validate_green_lamp_options(const GreenLampOptions & options);

/// Finds the lit green lamp inside the region of interest.
///
/// `bgr` is a decoded 8-bit 3-channel image. The region of interest is clipped
/// to the image, and an empty or non-conforming image simply yields no
/// detection rather than an error, because a bad frame must read as "not green"
/// and never as a permission to start.
GreenLampDetection detect_green_lamp(
  const cv::Mat & bgr,
  const GreenLampOptions & options = GreenLampOptions{});

}  // namespace kau_object_detection_lane

#endif  // KAU_OBJECT_DETECTION_LANE__GREEN_LAMP_DETECTOR_HPP_
