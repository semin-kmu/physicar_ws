#include "backup_object_detection/start_signal_node.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <exception>
#include <functional>
#include <limits>
#include <stdexcept>
#include <string>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "rclcpp/create_timer.hpp"

namespace backup_object_detection
{
namespace
{

/// ROS 정수 파라미터를 int 로 좁힌다. int 를 넘는 값이 되감겨 검증을
/// 통과하는 일을 막는다.
int toIntParam(const std::int64_t value, const char * name)
{
  if (value < 1 || value > std::numeric_limits<int>::max()) {
    throw std::invalid_argument(std::string(name) + " must be at least 1 and fit in an int");
  }
  return static_cast<int>(value);
}

}  // namespace

void validateGreenLampParams(const GreenLampParams & params)
{
  if (params.roi_x_min < 0 || params.roi_y_min < 0 ||
    params.roi_x_max <= params.roi_x_min ||
    params.roi_y_max <= params.roi_y_min)
  {
    throw std::invalid_argument("roi must be a non-empty window with non-negative origin");
  }
  if (params.hue_min < 0 || params.hue_max > 179 || params.hue_min > params.hue_max) {
    throw std::invalid_argument("hue gate must satisfy 0 <= hue_min <= hue_max <= 179");
  }
  if (params.saturation_min < 0 || params.saturation_min > 255 ||
    params.value_min < 0 || params.value_min > 255)
  {
    throw std::invalid_argument("saturation_min and value_min must be within 0..255");
  }
  if (params.min_area_px < 1 || params.max_area_px < params.min_area_px) {
    throw std::invalid_argument("area gate must satisfy 1 <= min_area_px <= max_area_px");
  }
  if (!std::isfinite(params.min_aspect_ratio) || !std::isfinite(params.max_aspect_ratio) ||
    params.min_aspect_ratio <= 0.0 || params.max_aspect_ratio < params.min_aspect_ratio)
  {
    throw std::invalid_argument("aspect gate must satisfy 0 < min_aspect_ratio <= max_aspect_ratio");
  }
  if (!std::isfinite(params.min_fill_ratio) ||
    params.min_fill_ratio < 0.0 || params.min_fill_ratio > 1.0)
  {
    throw std::invalid_argument("min_fill_ratio must be within 0..1");
  }
}

GreenLampDetection detectGreenLamp(const cv::Mat & bgr, const GreenLampParams & params)
{
  GreenLampDetection detection;

  if (bgr.empty() || bgr.type() != CV_8UC3) {
    return detection;
  }

  // 설정 창을 프레임에 맞춰 자른다. 완전히 밖이면 볼 것이 없다 = 초록 아님.
  const int x_min = std::max(0, params.roi_x_min);
  const int y_min = std::max(0, params.roi_y_min);
  const int x_max = std::min(bgr.cols, params.roi_x_max);
  const int y_max = std::min(bgr.rows, params.roi_y_max);
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
    cv::Scalar(params.hue_min, params.saturation_min, params.value_min),
    cv::Scalar(params.hue_max, 255, 255),
    mask);

  cv::Mat labels;
  cv::Mat stats;
  cv::Mat centroids;
  const int component_count =
    cv::connectedComponentsWithStats(mask, labels, stats, centroids, 8, CV_32S);

  // 라벨 0 은 배경.
  detection.candidate_count = std::max(0, component_count - 1);

  int best_area = 0;
  for (int label = 1; label < component_count; ++label) {
    const int area = stats.at<int>(label, cv::CC_STAT_AREA);
    if (area < params.min_area_px || area > params.max_area_px) {
      continue;
    }

    const int width = stats.at<int>(label, cv::CC_STAT_WIDTH);
    const int height = stats.at<int>(label, cv::CC_STAT_HEIGHT);
    if (width <= 0 || height <= 0) {
      continue;
    }

    const double aspect_ratio = static_cast<double>(width) / static_cast<double>(height);
    if (aspect_ratio < params.min_aspect_ratio || aspect_ratio > params.max_aspect_ratio) {
      continue;
    }

    const double fill_ratio = static_cast<double>(area) / static_cast<double>(width * height);
    if (fill_ratio < params.min_fill_ratio) {
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
    detection.center_x = window.x + static_cast<int>(std::lround(centroids.at<double>(label, 0)));
    detection.center_y = window.y + static_cast<int>(std::lround(centroids.at<double>(label, 1)));
  }

  return detection;
}

StartSignalNode::StartSignalNode()
: Node("backup_start_signal_detector")
{
  input_topic_ = declare_parameter<std::string>("input_topic", "/camera/image_raw/compressed");
  const auto output_topic =
    declare_parameter<std::string>("output_topic", "/perception/start_permission");

  publish_hz_ = declare_parameter<double>("publish_hz", 10.0);
  unsubscribe_after_latch_ = declare_parameter<bool>("unsubscribe_after_latch", true);

  const auto confirm_window = declare_parameter<std::int64_t>("confirm_window", 8);
  const auto confirm_count = declare_parameter<std::int64_t>("confirm_count", 4);
  latch_once_true_ = declare_parameter<bool>("latch_once_true", true);
  camera_timeout_s_ = declare_parameter<double>("camera_timeout_s", 0.5);

  lamp_.roi_x_min = static_cast<int>(declare_parameter<std::int64_t>("roi_x_min", 280));
  lamp_.roi_x_max = static_cast<int>(declare_parameter<std::int64_t>("roi_x_max", 450));
  lamp_.roi_y_min = static_cast<int>(declare_parameter<std::int64_t>("roi_y_min", 135));
  lamp_.roi_y_max = static_cast<int>(declare_parameter<std::int64_t>("roi_y_max", 265));
  lamp_.hue_min = static_cast<int>(declare_parameter<std::int64_t>("hue_min", 45));
  lamp_.hue_max = static_cast<int>(declare_parameter<std::int64_t>("hue_max", 85));
  lamp_.saturation_min = static_cast<int>(declare_parameter<std::int64_t>("saturation_min", 150));
  lamp_.value_min = static_cast<int>(declare_parameter<std::int64_t>("value_min", 150));
  lamp_.min_area_px = static_cast<int>(declare_parameter<std::int64_t>("min_area_px", 60));
  lamp_.max_area_px = static_cast<int>(declare_parameter<std::int64_t>("max_area_px", 5000));
  lamp_.min_aspect_ratio = declare_parameter<double>("min_aspect_ratio", 0.6);
  lamp_.max_aspect_ratio = declare_parameter<double>("max_aspect_ratio", 1.6);
  lamp_.min_fill_ratio = declare_parameter<double>("min_fill_ratio", 0.6);

  log_detection_detail_ = declare_parameter<bool>("log_detection_detail", false);
  const auto log_period_s = declare_parameter<double>("log_period_s", 2.0);

  if (!std::isfinite(publish_hz_) || publish_hz_ <= 0.0) {
    throw std::invalid_argument("publish_hz must be finite and positive");
  }
  if (!std::isfinite(camera_timeout_s_) || camera_timeout_s_ <= 0.0) {
    throw std::invalid_argument("camera_timeout_s must be finite and positive");
  }
  if (!std::isfinite(log_period_s) || log_period_s <= 0.0) {
    throw std::invalid_argument("log_period_s must be finite and positive");
  }

  confirm_window_ = toIntParam(confirm_window, "confirm_window");
  confirm_count_ = toIntParam(confirm_count, "confirm_count");
  // 윈도우보다 큰 문턱은 영원히 도달하지 못한다. 다른 증상 없이 레이스 내내
  // 출발을 막으므로 기동 자체를 거부한다.
  if (confirm_count_ > confirm_window_) {
    throw std::invalid_argument("confirm_count must not exceed confirm_window");
  }
  validateGreenLampParams(lamp_);

  log_period_ms_ = static_cast<std::int64_t>(log_period_s * 1000.0);

  image_subscription_ = create_subscription<sensor_msgs::msg::CompressedImage>(
    input_topic_,
    rclcpp::SensorDataQoS(),
    std::bind(&StartSignalNode::onImage, this, std::placeholders::_1));

  permission_publisher_ = create_publisher<std_msgs::msg::Bool>(
    output_topic,
    rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile());

  publish_timer_ = rclcpp::create_timer(
    this, get_clock(), rclcpp::Duration::from_seconds(1.0 / publish_hz_),
    std::bind(&StartSignalNode::onTimer, this));

  RCLCPP_INFO(
    get_logger(),
    "[기동][start] in=%s out=%s %.1f Hz 확인 %d/%d latch=%d 구독해제=%d timeout %.2f s",
    input_topic_.c_str(), output_topic.c_str(), publish_hz_, confirm_count_, confirm_window_,
    latch_once_true_ ? 1 : 0, unsubscribe_after_latch_ ? 1 : 0, camera_timeout_s_);
  RCLCPP_INFO(
    get_logger(),
    "[기동][start] 램프 게이트 roi=[%d,%d]x[%d,%d] hue=[%d,%d] s>=%d v>=%d "
    "area=[%d,%d] aspect=[%.2f,%.2f] fill>=%.2f",
    lamp_.roi_x_min, lamp_.roi_x_max, lamp_.roi_y_min, lamp_.roi_y_max,
    lamp_.hue_min, lamp_.hue_max, lamp_.saturation_min, lamp_.value_min,
    lamp_.min_area_px, lamp_.max_area_px,
    lamp_.min_aspect_ratio, lamp_.max_aspect_ratio, lamp_.min_fill_ratio);
}

void StartSignalNode::clearWindow()
{
  window_.clear();
  green_hits_ = 0;
}

void StartSignalNode::pushSample(const bool green)
{
  // 먼저 밀어낸다. 윈도우가 설정 크기를 넘지 않고, 카운트가 항상 윈도우
  // 안의 표본만을 가리킨다.
  if (window_.size() >= static_cast<std::size_t>(confirm_window_)) {
    if (window_.front()) {
      --green_hits_;
    }
    window_.pop_front();
  }

  window_.push_back(green);
  if (green) {
    ++green_hits_;
  }

  // 윈도우가 다 찰 필요는 없다. 문턱은 비율이 아니라 개수다.
  const bool confirmed = green_hits_ >= confirm_count_;
  if (confirmed) {
    permitted_ = true;
    if (latch_once_true_) {
      latched_ = true;
    }
  } else if (!latched_) {
    permitted_ = false;
  }
}

void StartSignalNode::observeFrame(const FrameOutcome outcome, const double now_s)
{
  // latch 후엔 결정이 최종이다. 윈도우를 동결해 보고되는 hit 수가 latch 시점
  // 값으로 남는다.
  if (latched_) {
    if (outcome != FrameOutcome::kUnusable) {
      has_usable_frame_ = true;
      last_usable_frame_s_ = now_s;
      camera_active_ = true;
    }
    return;
  }

  switch (outcome) {
    case FrameOutcome::kGreen:
    case FrameOutcome::kNotGreen:
      has_usable_frame_ = true;
      last_usable_frame_s_ = now_s;
      camera_active_ = true;
      pushSample(outcome == FrameOutcome::kGreen);
      break;

    case FrameOutcome::kUnusable:
      // 판정 불가 프레임은 비-green 표본으로만 넣는다. 오래된 green 을 밀어내
      // latch 를 더 어렵게만 만드는 보수적인 선택이다.
      pushSample(false);
      break;
  }
}

void StartSignalNode::applyCameraTimeout(const double now_s)
{
  if (latched_) {
    return;
  }

  if (!has_usable_frame_) {
    camera_active_ = false;
    return;
  }

  const double age_s = now_s - last_usable_frame_s_;

  // 시뮬 리셋으로 시계가 뒤로 간 경우는 없던 공백으로 보고하지 않고 다시 맞춘다.
  if (age_s < 0.0) {
    last_usable_frame_s_ = now_s;
    return;
  }

  if (age_s > camera_timeout_s_) {
    camera_active_ = false;
    // 카운트만이 아니라 윈도우 전체를 버린다. 긴 공백 전의 green 이 공백 후의
    // green 과 합산되면 수 초 묵은 근거로 출발할 수 있다.
    clearWindow();
    permitted_ = false;
  }
}

void StartSignalNode::onImage(const sensor_msgs::msg::CompressedImage::ConstSharedPtr image)
{
  try {
    const double now_s = now().seconds();

    // cv::imdecode 로 JPEG 페이로드를 바로 읽는다. image_transport 나 bridge 를
    // 거치지 않는다.
    const cv::Mat encoded(
      1, static_cast<int>(image->data.size()), CV_8UC1,
      const_cast<std::uint8_t *>(image->data.data()));
    const cv::Mat frame = cv::imdecode(encoded, cv::IMREAD_COLOR);

    if (frame.empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), log_period_ms_,
        "[검출][start] 압축 이미지 디코드 실패 format=%s bytes=%zu",
        image->format.c_str(), image->data.size());
      observeFrame(FrameOutcome::kUnusable, now_s);
      reportState();
      return;
    }

    const auto detection = detectGreenLamp(frame, lamp_);
    observeFrame(
      detection.detected ? FrameOutcome::kGreen : FrameOutcome::kNotGreen, now_s);

    if (log_detection_detail_) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), log_period_ms_,
        "[검출][start] %dx%d 후보 %d 검출 %d area=%d aspect=%.2f fill=%.2f "
        "center=(%d,%d) green %d/%d 표본 %d",
        frame.cols, frame.rows, detection.candidate_count, detection.detected ? 1 : 0,
        detection.area_px, detection.aspect_ratio, detection.fill_ratio,
        detection.center_x, detection.center_y,
        green_hits_, confirm_window_, static_cast<int>(window_.size()));
    }

    reportState();

    if (unsubscribe_after_latch_ && latched_) {
      // 자기 콜백 안에서 구독을 파괴하지 않는다. 타이머에서 처리한다.
      unsubscribe_pending_ = true;
    }
  } catch (const std::exception & error) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), log_period_ms_,
      "[검출][start] 콜백 예외 (%s)", error.what());
  }
}

void StartSignalNode::onTimer()
{
  try {
    if (unsubscribe_pending_ && image_subscription_) {
      image_subscription_.reset();
      unsubscribe_pending_ = false;
      RCLCPP_INFO(
        get_logger(), "[상태][start] latch 완료, %s 구독 해제", input_topic_.c_str());
    }

    applyCameraTimeout(now().seconds());

    std_msgs::msg::Bool message;
    message.data = latched_ || permitted_;
    permission_publisher_->publish(message);

    reportState();
  } catch (const std::exception & error) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), log_period_ms_,
      "[상태][start] 타이머 예외 (%s)", error.what());
  }
}

void StartSignalNode::reportState()
{
  if (latched_ != reported_latched_) {
    reported_latched_ = latched_;
    RCLCPP_INFO(
      get_logger(),
      "[상태][start] start_permission latch, 최근 %d 표본 중 green %d 개",
      static_cast<int>(window_.size()), green_hits_);
  }

  if (camera_active_ != reported_camera_active_) {
    reported_camera_active_ = camera_active_;
    if (camera_active_) {
      RCLCPP_INFO(get_logger(), "[상태][start] 카메라 스트림 활성");
    } else {
      RCLCPP_WARN(
        get_logger(),
        "[상태][start] %.2f s 동안 판정 가능한 프레임 없음, permission=%s",
        camera_timeout_s_, (latched_ || permitted_) ? "true" : "false");
    }
  }

  if (green_hits_ != reported_hits_) {
    reported_hits_ = green_hits_;
    RCLCPP_INFO(
      get_logger(), "[검출][start] green %d/%d 필요 %d 표본 %d",
      green_hits_, confirm_window_, confirm_count_, static_cast<int>(window_.size()));
    return;
  }

  RCLCPP_INFO_THROTTLE(
    get_logger(), *get_clock(), log_period_ms_,
    "[검출][start] permission=%s green %d/%d 필요 %d 표본 %d",
    (latched_ || permitted_) ? "true" : "false",
    green_hits_, confirm_window_, confirm_count_, static_cast<int>(window_.size()));
}

}  // namespace backup_object_detection
