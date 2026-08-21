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

#include <stdexcept>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "gtest/gtest.h"
#include "kau_object_detection/green_lamp_detector.hpp"

namespace
{

using kau_object_detection::GreenLampOptions;
using kau_object_detection::detect_green_lamp;
using kau_object_detection::validate_green_lamp_options;

/// Real camera geometry: 480x360 with the signal to the right of the vehicle.
constexpr int kFrameWidth = 480;
constexpr int kFrameHeight = 360;

/// BGR of the lit lamp measured on the live camera: hue 60, saturation 255,
/// value 255 converts to pure green.
const cv::Scalar kLampBgr{0, 255, 0};

/// BGR of the roadside grass, measured at hue 33. Yellow-green, and bright
/// enough to pass a naive brightness test.
const cv::Scalar kGrassBgr{0, 200, 140};

/// Red lamp, the state the detector must never accept.
const cv::Scalar kRedBgr{0, 0, 255};

GreenLampOptions default_options()
{
  GreenLampOptions options;
  options.roi.x_min = 280;
  options.roi.x_max = 450;
  options.roi.y_min = 135;
  options.roi.y_max = 265;
  options.hue_min = 45;
  options.hue_max = 85;
  options.saturation_min = 150;
  options.value_min = 150;
  options.min_area_px = 60;
  options.max_area_px = 5000;
  options.min_aspect_ratio = 0.6;
  options.max_aspect_ratio = 1.6;
  options.min_fill_ratio = 0.6;
  return options;
}

/// Dark frame with a mid-grey ground, so nothing but the drawn shapes can pass
/// the colour gate.
cv::Mat blank_frame()
{
  return cv::Mat(kFrameHeight, kFrameWidth, CV_8UC3, cv::Scalar(60, 60, 60));
}

cv::Mat frame_with_circle(const cv::Point & center, const int radius, const cv::Scalar & bgr)
{
  cv::Mat frame = blank_frame();
  cv::circle(frame, center, radius, bgr, cv::FILLED);
  return frame;
}

// ---------------------------------------------------------------------------
// Detection inside the region of interest
// ---------------------------------------------------------------------------

TEST(GreenLampDetectorTest, DetectsAGreenCircleInsideTheRoi)
{
  // Centre of the measured lamp on the live camera.
  const cv::Mat frame = frame_with_circle({338, 211}, 8, kLampBgr);

  const auto detection = detect_green_lamp(frame, default_options());

  EXPECT_TRUE(detection.detected);
  EXPECT_GE(detection.area_px, 60);
  EXPECT_NEAR(detection.aspect_ratio, 1.0, 0.2);
  EXPECT_GE(detection.fill_ratio, 0.6);
  EXPECT_NEAR(detection.center_x, 338, 2);
  EXPECT_NEAR(detection.center_y, 211, 2);
}

TEST(GreenLampDetectorTest, ReportsFullFrameCoordinates)
{
  // Deliberately off the ROI centre: the reported centre must be in frame
  // pixels, not pixels relative to the window.
  const cv::Mat frame = frame_with_circle({300, 160}, 9, kLampBgr);

  const auto detection = detect_green_lamp(frame, default_options());

  ASSERT_TRUE(detection.detected);
  EXPECT_NEAR(detection.center_x, 300, 2);
  EXPECT_NEAR(detection.center_y, 160, 2);
}

// ---------------------------------------------------------------------------
// Real start pose geometry
// ---------------------------------------------------------------------------

/// Lamp measured on the real PhysiCar camera: bbox (412, 217, 28x28),
/// centre (425.7, 230.4), 591 px. A radius 14 disc reproduces it closely.
cv::Mat frame_with_measured_lamp()
{
  return frame_with_circle({426, 230}, 14, kLampBgr);
}

TEST(GreenLampDetectorTest, DetectsTheLampMeasuredAtTheRealStartPose)
{
  const auto detection = detect_green_lamp(frame_with_measured_lamp(), default_options());

  EXPECT_TRUE(detection.detected);
  EXPECT_NEAR(detection.area_px, 591, 120);
  EXPECT_NEAR(detection.aspect_ratio, 1.0, 0.15);
  EXPECT_NEAR(detection.center_x, 426, 2);
  EXPECT_NEAR(detection.center_y, 230, 2);
}

TEST(GreenLampDetectorTest, TheOldWindowClippedTheMeasuredLampIntoRejection)
{
  // Regression guard. The window used to end at x = 415, which left a 3 px
  // slice of the lamp and the area gate then threw it away: the signal was
  // green and the permission still stayed false.
  GreenLampOptions narrow = default_options();
  narrow.roi.x_max = 415;

  EXPECT_FALSE(detect_green_lamp(frame_with_measured_lamp(), narrow).detected);
  EXPECT_TRUE(detect_green_lamp(frame_with_measured_lamp(), default_options()).detected);
}

TEST(GreenLampDetectorTest, WidenedWindowStillStopsShortOfTheFrameEdge)
{
  // x_max = 450 on a 480 wide frame leaves a margin, so anything painted at
  // the right edge stays out of the decision.
  cv::Mat frame = blank_frame();
  cv::rectangle(frame, cv::Rect(455, 0, 25, kFrameHeight), kLampBgr, cv::FILLED);

  EXPECT_FALSE(detect_green_lamp(frame, default_options()).detected);
}

// ---------------------------------------------------------------------------
// The region of interest is a hard boundary
// ---------------------------------------------------------------------------

TEST(GreenLampDetectorTest, IgnoresAGreenCircleLeftOfTheRoi)
{
  const cv::Mat frame = frame_with_circle({100, 200}, 12, kLampBgr);

  EXPECT_FALSE(detect_green_lamp(frame, default_options()).detected);
}

TEST(GreenLampDetectorTest, IgnoresAGreenCircleRightOfTheRoi)
{
  // Clear of x_max = 450 by a full radius, so the window alone rejects it.
  const cv::Mat frame = frame_with_circle({470, 200}, 8, kLampBgr);

  EXPECT_FALSE(detect_green_lamp(frame, default_options()).detected);
}

TEST(GreenLampDetectorTest, IgnoresAGreenCircleAboveTheRoi)
{
  const cv::Mat frame = frame_with_circle({340, 60}, 12, kLampBgr);

  EXPECT_FALSE(detect_green_lamp(frame, default_options()).detected);
}

TEST(GreenLampDetectorTest, IgnoresAGreenCircleBelowTheRoi)
{
  const cv::Mat frame = frame_with_circle({340, 330}, 12, kLampBgr);

  EXPECT_FALSE(detect_green_lamp(frame, default_options()).detected);
}

TEST(GreenLampDetectorTest, LargeGreenFieldOutsideTheRoiNeverPermitsTheStart)
{
  // The whole left half of the frame is lamp-coloured. Only the window counts.
  cv::Mat frame = blank_frame();
  cv::rectangle(frame, cv::Rect(0, 0, 270, kFrameHeight), kLampBgr, cv::FILLED);

  EXPECT_FALSE(detect_green_lamp(frame, default_options()).detected);
}

// ---------------------------------------------------------------------------
// Size gates
// ---------------------------------------------------------------------------

TEST(GreenLampDetectorTest, RejectsATooSmallGreenBlobInsideTheRoi)
{
  // Radius 3 gives roughly 29 px, the size of the largest measured grass
  // fragment and far below the 192 px of the real lamp.
  const cv::Mat frame = frame_with_circle({338, 211}, 3, kLampBgr);

  const auto detection = detect_green_lamp(frame, default_options());

  EXPECT_FALSE(detection.detected);
  EXPECT_GE(detection.candidate_count, 1) << "the blob must reach the size gate";
}

TEST(GreenLampDetectorTest, RejectsATooLargeGreenRegionInsideTheRoi)
{
  GreenLampOptions options = default_options();
  options.max_area_px = 2000;

  cv::Mat frame = blank_frame();
  cv::rectangle(frame, cv::Rect(285, 140, 120, 115), kLampBgr, cv::FILLED);

  const auto detection = detect_green_lamp(frame, options);

  EXPECT_FALSE(detection.detected);
  EXPECT_GE(detection.candidate_count, 1);
}

// ---------------------------------------------------------------------------
// Shape gates
// ---------------------------------------------------------------------------

TEST(GreenLampDetectorTest, RejectsAnElongatedGreenStripe)
{
  // Wide and thin: a painted line or a foliage edge, never a lamp.
  cv::Mat frame = blank_frame();
  cv::rectangle(frame, cv::Rect(290, 200, 110, 8), kLampBgr, cv::FILLED);

  const auto detection = detect_green_lamp(frame, default_options());

  EXPECT_FALSE(detection.detected);
  EXPECT_GE(detection.candidate_count, 1);
}

TEST(GreenLampDetectorTest, RejectsASparseRingThatFillsItsBoundingBoxPoorly)
{
  // A hollow ring spans a square bounding box but fills little of it.
  cv::Mat frame = blank_frame();
  cv::circle(frame, {338, 211}, 24, kLampBgr, 2);

  EXPECT_FALSE(detect_green_lamp(frame, default_options()).detected);
}

// ---------------------------------------------------------------------------
// Colour gates
// ---------------------------------------------------------------------------

TEST(GreenLampDetectorTest, RejectsGrassColouredFoliageInsideTheRoi)
{
  // Same size and shape as the lamp, but at the measured grass hue.
  const cv::Mat frame = frame_with_circle({338, 211}, 8, kGrassBgr);

  const auto detection = detect_green_lamp(frame, default_options());

  EXPECT_FALSE(detection.detected);
  EXPECT_EQ(detection.candidate_count, 0) << "grass must not even reach the shape gates";
}

TEST(GreenLampDetectorTest, RejectsARedLampInsideTheRoi)
{
  const cv::Mat frame = frame_with_circle({338, 211}, 8, kRedBgr);

  EXPECT_FALSE(detect_green_lamp(frame, default_options()).detected);
}

TEST(GreenLampDetectorTest, RejectsADimGreenCircle)
{
  // Right hue, but the lamp is not lit.
  const cv::Mat frame = frame_with_circle({338, 211}, 8, cv::Scalar(0, 90, 0));

  EXPECT_FALSE(detect_green_lamp(frame, default_options()).detected);
}

TEST(GreenLampDetectorTest, RejectsAWashedOutGreenCircle)
{
  // Bright but barely saturated, the way an overexposed white surface reads.
  const cv::Mat frame = frame_with_circle({338, 211}, 8, cv::Scalar(220, 255, 220));

  EXPECT_FALSE(detect_green_lamp(frame, default_options()).detected);
}

// ---------------------------------------------------------------------------
// Degenerate input
// ---------------------------------------------------------------------------

TEST(GreenLampDetectorTest, EmptyImageIsNotGreen)
{
  EXPECT_FALSE(detect_green_lamp(cv::Mat{}, default_options()).detected);
}

TEST(GreenLampDetectorTest, SingleChannelImageIsNotGreen)
{
  const cv::Mat gray(kFrameHeight, kFrameWidth, CV_8UC1, cv::Scalar(255));

  EXPECT_FALSE(detect_green_lamp(gray, default_options()).detected);
}

TEST(GreenLampDetectorTest, RoiEntirelyOutsideTheImageIsNotGreen)
{
  GreenLampOptions options = default_options();
  options.roi.x_min = 900;
  options.roi.x_max = 1000;

  const cv::Mat frame = frame_with_circle({338, 211}, 8, kLampBgr);

  EXPECT_FALSE(detect_green_lamp(frame, options).detected);
}

TEST(GreenLampDetectorTest, RoiIsClippedToASmallerFrame)
{
  // A frame smaller than the configured window must not read out of bounds.
  cv::Mat frame(200, 320, CV_8UC3, cv::Scalar(60, 60, 60));
  cv::circle(frame, {300, 160}, 8, kLampBgr, cv::FILLED);

  EXPECT_NO_THROW(detect_green_lamp(frame, default_options()));
}

// ---------------------------------------------------------------------------
// Option validation
// ---------------------------------------------------------------------------

TEST(GreenLampDetectorTest, AcceptsTheShippedDefaults)
{
  EXPECT_NO_THROW(validate_green_lamp_options(default_options()));
}

TEST(GreenLampDetectorTest, RejectsAnEmptyRoi)
{
  GreenLampOptions options = default_options();
  options.roi.x_max = options.roi.x_min;

  EXPECT_THROW(validate_green_lamp_options(options), std::invalid_argument);
}

TEST(GreenLampDetectorTest, RejectsAnInvertedHueWindow)
{
  GreenLampOptions options = default_options();
  options.hue_min = 90;
  options.hue_max = 40;

  EXPECT_THROW(validate_green_lamp_options(options), std::invalid_argument);
}

TEST(GreenLampDetectorTest, RejectsAHueBeyondTheOpenCvRange)
{
  GreenLampOptions options = default_options();
  options.hue_max = 200;

  EXPECT_THROW(validate_green_lamp_options(options), std::invalid_argument);
}

TEST(GreenLampDetectorTest, RejectsAnInvertedAreaGate)
{
  GreenLampOptions options = default_options();
  options.min_area_px = 900;
  options.max_area_px = 100;

  EXPECT_THROW(validate_green_lamp_options(options), std::invalid_argument);
}

TEST(GreenLampDetectorTest, RejectsANonPositiveAspectGate)
{
  GreenLampOptions options = default_options();
  options.min_aspect_ratio = 0.0;

  EXPECT_THROW(validate_green_lamp_options(options), std::invalid_argument);
}

TEST(GreenLampDetectorTest, RejectsAFillRatioOutsideZeroToOne)
{
  GreenLampOptions options = default_options();
  options.min_fill_ratio = 1.5;

  EXPECT_THROW(validate_green_lamp_options(options), std::invalid_argument);
}

}  // namespace
