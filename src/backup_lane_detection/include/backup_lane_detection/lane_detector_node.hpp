// backup_lane_detector -- 카메라 한 대로 흰 차선 기하를 낸다. 경로는 만들지 않는다.
//
// 이미지 콜백 구동. 타이머가 없다 -- 입력이 끊기면 발행도 끊기고
// 하류의 timeout 게이트가 고장을 잡는다.

#ifndef BACKUP_LANE_DETECTION__LANE_DETECTOR_NODE_HPP_
#define BACKUP_LANE_DETECTION__LANE_DETECTOR_NODE_HPP_

#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/header.hpp>

#include <opencv2/core.hpp>

#include "backup_msgs/msg/lane_geometry.hpp"

#include "backup_lane_detection/homography.hpp"
#include "backup_lane_detection/line_fit.hpp"
#include "backup_lane_detection/white_mask.hpp"

namespace backup_lane_detection
{

class LaneDetectorNode : public rclcpp::Node
{
public:
  explicit LaneDetectorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // yaml 이름 그대로.
  struct Params
  {
    std::string input_topic;
    std::string output_topic;
    std::string debug_image_topic;
    bool publish_debug_image = false;

    bool publish_bev_debug = false;
    int bev_debug_width = 240;
    int bev_debug_height = 320;

    bool undistort = false;
    int image_width = 480;
    int image_height = 360;
    double fx = 0.0;
    double fy = 0.0;
    double cx = 0.0;
    double cy = 0.0;

    double camera_height_m = 0.0;
    double camera_tilt_deg = 0.0;
    double tilt_bias_deg = 0.0;
    bool publish_tilt_on_start = true;
    std::string tilt_topic;
    bool use_camera_pan = false;
    double camera_to_rear_axle_m = 0.0;

    int roi_row_min = 0;
    int roi_row_max = 0;
    double detect_min_m = 0.0;
    double detect_max_m = 0.0;

    double lane_width_m = 0.0;
    double tophat_kernel_ratio = 3.0;
    int tophat_strips = 4;

    double run_merge_grid_m = 0.005;

    double inflection_enter_deg = 20.0;
    double inflection_exit_deg = 15.0;
    double inflection_residual_m = 0.03;
    int min_transition_points = 4;

    bool use_motion_prediction = true;
    std::string speed_topic;
    std::string steer_topic;
    double wheelbase_m = 0.18;

    double filter_q_lateral = 0.0;
    double filter_q_heading = 0.0;
    double filter_r_lateral = 0.0;
    double filter_r_heading = 0.0;
    double gate_lateral_m = 0.0;
    double gate_heading_rad = 0.0;

    int valid_on_frames = 2;
    int valid_off_frames = 3;
    double track_half_width_m = 0.35;

    double log_period_s = 2.0;
  };

  // 유효 판정 히스테리시스.
  struct SideState
  {
    int on = 0;
    int off = 0;
    bool valid = false;

    void update(bool detected, int on_frames, int off_frames)
    {
      if (detected) {
        off = 0;
        if (++on >= on_frames) {valid = true;}
      } else {
        on = 0;
        if (++off >= off_frames) {valid = false;}
      }
    }
  };

  void declareParams();
  void loadParams();
  void buildGeometry();
  void logStartup();
  void publishTiltOnce();

  void onImage(const sensor_msgs::msg::CompressedImage::ConstSharedPtr msg);
  void onSpeed(const std_msgs::msg::Float64::ConstSharedPtr msg);
  void onSteer(const std_msgs::msg::Float64::ConstSharedPtr msg);

  void process(const cv::Mat & bgr, const std_msgs::msg::Header & header);

  // 검출점 -> 지면좌표. 밴드 밖은 버린다.
  void toGround(const std::vector<RunPoint> & runs, std::vector<Point2> & out) const;
  // 행 런과 열 런이 같은 지점을 두 번 낸다. 지면 격자로 병합한다.
  void mergeGrid(std::vector<Point2> & pts) const;
  void classify(
    const std::vector<Point2> & pts,
    std::vector<Point2> & left, std::vector<Point2> & right) const;

  void updateFilter(const Line & center, bool have_center, double dt);
  void publishDebugImage(
    const cv::Mat & roi, const cv::Mat & mask,
    const std::vector<RunPoint> & runs, const std_msgs::msg::Header & header);

  Params p_;

  GroundProjector proj_;
  RowTable rows_;
  std::vector<Strip> strips_;
  MaskParams mask_p_;
  RunParams run_p_;
  FitParams fit_p_;

  int roi_row0_ = 0;          // 포함
  int roi_row1_ = 0;          // 미포함
  double tilt_rad_ = 0.0;
  double inflection_enter_rad_ = 0.0;
  double inflection_exit_rad_ = 0.0;

  // 이전 프레임 해 -- IRLS 초기값. 결정성과 시간 연속성의 근거다.
  Line prev_left_;
  Line prev_right_;

  // 중앙선 상태 (x=0 에서의 횡오프셋, 헤딩)
  bool filter_init_ = false;
  double f_y_ = 0.0;
  double f_psi_ = 0.0;
  double f_var_y_ = 0.0;
  double f_var_psi_ = 0.0;
  rclcpp::Time last_stamp_;
  bool have_stamp_ = false;

  double speed_ = 0.0;        // [m/s]
  double steer_ = 0.0;        // [rad]

  // 히스테리시스로 붙잡은 직전 해를 발행하는 동안 자차가 나아간 거리 [m].
  // valid_length 에서 빼야 하류 Ld 가 외삽 구간에 들어가지 않는다.
  double stale_dist_ = 0.0;

  SideState left_state_;
  SideState right_state_;
  uint8_t prev_lost_ = 0;
  uint8_t lost_first_ = 0;
  double last_delta_psi_ = 0.0;
  bool inflected_ = false;

  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr image_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr speed_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr steer_sub_;
  rclcpp::Publisher<backup_msgs::msg::LaneGeometry>::SharedPtr lane_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr tilt_pub_;
};

}  // namespace backup_lane_detection

#endif  // BACKUP_LANE_DETECTION__LANE_DETECTOR_NODE_HPP_
