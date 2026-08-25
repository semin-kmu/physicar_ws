// backup_start_signal_detector
//
// kau_object_detection/start_signal_detector_node 이식.
// /camera/image_raw/compressed -> /perception/start_permission (10 Hz 타이머).
// permission 이 latch 되면 구독을 해제한다 -- 레이스 내내 도는 이미지 처리
// 하나가 사라진다.

#ifndef BACKUP_OBJECT_DETECTION__START_SIGNAL_NODE_HPP_
#define BACKUP_OBJECT_DETECTION__START_SIGNAL_NODE_HPP_

#include <cstdint>
#include <deque>
#include <string>

#include <opencv2/core.hpp>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "std_msgs/msg/bool.hpp"

namespace backup_object_detection
{

/// 점등된 초록 램프의 색·형상 게이트. 전부 yaml 값이다.
struct GreenLampParams
{
  int roi_x_min{280};
  int roi_x_max{450};
  int roi_y_min{135};
  int roi_y_max{265};

  int hue_min{45};        // OpenCV hue 는 0~179
  int hue_max{85};
  int saturation_min{150};
  int value_min{150};

  int min_area_px{60};
  int max_area_px{5000};
  double min_aspect_ratio{0.6};
  double max_aspect_ratio{1.6};
  double min_fill_ratio{0.6};
};

/// 한 프레임 판정 결과. detected 외 나머지는 게이트 튜닝용 로그 재료다.
struct GreenLampDetection
{
  bool detected{false};
  int area_px{0};
  double aspect_ratio{0.0};
  double fill_ratio{0.0};
  int center_x{0};
  int center_y{0};
  int candidate_count{0};
};

/// bgr 은 디코드된 8UC3. 빈 프레임이나 창 밖은 "초록 아님" 으로 읽힌다.
GreenLampDetection detectGreenLamp(const cv::Mat & bgr, const GreenLampParams & params);

/// 쓸 수 없는 게이트 설정이면 std::invalid_argument.
void validateGreenLampParams(const GreenLampParams & params);

class StartSignalNode : public rclcpp::Node
{
public:
  StartSignalNode();

private:
  /// 한 프레임의 판정 결과.
  enum class FrameOutcome : std::uint8_t
  {
    kGreen,
    kNotGreen,
    kUnusable,   // 디코드 실패. 윈도우엔 비-green 으로 들어가되 카메라 생존으로는 세지 않는다
  };

  void onImage(const sensor_msgs::msg::CompressedImage::ConstSharedPtr image);
  void onTimer();

  void observeFrame(FrameOutcome outcome, double now_s);
  void pushSample(bool green);
  void clearWindow();
  void applyCameraTimeout(double now_s);
  void reportState();

  std::string input_topic_;
  double publish_hz_{10.0};
  bool unsubscribe_after_latch_{true};

  int confirm_window_{5};
  int confirm_count_{3};
  bool latch_once_true_{true};
  double camera_timeout_s_{0.5};

  GreenLampParams lamp_;
  bool log_detection_detail_{false};
  std::int64_t log_period_ms_{2000};

  std::deque<bool> window_;
  int green_hits_{0};
  bool permitted_{false};
  bool latched_{false};
  bool camera_active_{false};
  bool has_usable_frame_{false};
  double last_usable_frame_s_{0.0};

  bool reported_latched_{false};
  bool reported_camera_active_{false};
  int reported_hits_{0};
  /// 구독 해제는 이미지 콜백 안에서 하지 않고 타이머로 미룬다.
  bool unsubscribe_pending_{false};

  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr image_subscription_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr permission_publisher_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

}  // namespace backup_object_detection

#endif  // BACKUP_OBJECT_DETECTION__START_SIGNAL_NODE_HPP_
