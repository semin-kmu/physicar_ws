#include "backup_lane_detection/lane_detector_node.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <backup_common/motion.hpp>
#include <backup_common/vehicle.hpp>

namespace backup_lane_detection
{

using backup_common::kDeg2Rad;
using backup_common::kRad2Deg;
using backup_common::wrapAngle;
using LaneGeometry = backup_msgs::msg::LaneGeometry;

namespace
{
// 내부 토픽 · 차량 출력 QoS (규약 §8)
rclcpp::QoS internalQoS()
{
  return rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();
}
}  // namespace

LaneDetectorNode::LaneDetectorNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("backup_lane_detector", options)
{
  declareParams();
  loadParams();
  buildGeometry();

  lane_pub_ = create_publisher<LaneGeometry>(p_.output_topic, internalQoS());

  if (p_.publish_debug_image) {
    debug_pub_ = create_publisher<sensor_msgs::msg::Image>(p_.debug_image_topic, internalQoS());
  }

  image_sub_ = create_subscription<sensor_msgs::msg::CompressedImage>(
    p_.input_topic, rclcpp::SensorDataQoS(),
    std::bind(&LaneDetectorNode::onImage, this, std::placeholders::_1));

  if (p_.use_motion_prediction) {
    speed_sub_ = create_subscription<std_msgs::msg::Float64>(
      p_.speed_topic, internalQoS(),
      std::bind(&LaneDetectorNode::onSpeed, this, std::placeholders::_1));
    steer_sub_ = create_subscription<std_msgs::msg::Float64>(
      p_.steer_topic, internalQoS(),
      std::bind(&LaneDetectorNode::onSteer, this, std::placeholders::_1));
  }

  publishTiltOnce();
  logStartup();
}

// ================================================================
// 파라미터
// ================================================================

void LaneDetectorNode::declareParams()
{
  declare_parameter<std::string>("input_topic", "/camera/image_raw/compressed");
  declare_parameter<std::string>("output_topic", "/backup/lane/geometry");
  declare_parameter<std::string>("debug_image_topic", "/backup/lane/debug_image");
  declare_parameter<bool>("publish_debug_image", false);

  declare_parameter<bool>("publish_bev_debug", false);
  declare_parameter<int>("bev_debug_width", 240);
  declare_parameter<int>("bev_debug_height", 320);

  declare_parameter<bool>("undistort", false);
  declare_parameter<int>("image_width", 480);
  declare_parameter<int>("image_height", 360);
  declare_parameter<double>("fx", 261.558954);
  declare_parameter<double>("fy", 261.558954);
  declare_parameter<double>("cx", 231.819776);
  declare_parameter<double>("cy", 169.603322);

  declare_parameter<double>("camera_height_m", 0.1465);
  declare_parameter<double>("camera_tilt_deg", 10.0);
  declare_parameter<double>("tilt_bias_deg", 0.284);
  declare_parameter<bool>("publish_tilt_on_start", true);
  declare_parameter<std::string>("tilt_topic", "/camera/tilt");
  declare_parameter<bool>("use_camera_pan", false);
  declare_parameter<double>("camera_to_rear_axle_m", 0.195);

  declare_parameter<int>("roi_row_min", 140);
  declare_parameter<int>("roi_row_max", 250);
  declare_parameter<double>("detect_min_m", 0.45);
  declare_parameter<double>("detect_max_m", 2.00);

  declare_parameter<int>("tophat_threshold", 30);
  declare_parameter<int>("saturation_max", 60);
  declare_parameter<double>("lane_width_m", 0.075);
  declare_parameter<double>("tophat_kernel_ratio", 3.0);
  declare_parameter<bool>("tophat_bidirectional", true);
  declare_parameter<int>("tophat_strips", 4);

  declare_parameter<bool>("use_clahe", false);
  declare_parameter<double>("clahe_clip", 2.0);
  declare_parameter<int>("clahe_grid", 8);
  declare_parameter<int>("morph_open", 2);
  declare_parameter<int>("morph_close", 3);

  declare_parameter<bool>("run_scan_rows", true);
  declare_parameter<bool>("run_scan_cols", true);
  declare_parameter<double>("run_merge_grid_m", 0.005);
  declare_parameter<double>("run_width_tol_lo", 0.5);
  declare_parameter<double>("run_width_tol_hi", 2.0);
  declare_parameter<int>("min_run_pixels", 3);
  declare_parameter<int>("row_step", 1);
  declare_parameter<int>("col_step", 2);

  declare_parameter<int>("irls_iterations", 4);
  declare_parameter<double>("irls_huber_delta_m", 0.02);
  declare_parameter<int>("min_points_per_line", 8);
  declare_parameter<bool>("seed_from_previous", true);

  declare_parameter<double>("inflection_enter_deg", 20.0);
  declare_parameter<double>("inflection_exit_deg", 15.0);
  declare_parameter<double>("inflection_residual_m", 0.03);
  declare_parameter<int>("min_transition_points", 4);

  declare_parameter<bool>("use_motion_prediction", true);
  declare_parameter<std::string>("speed_topic", "/speed");
  declare_parameter<std::string>("steer_topic", "/steering");
  declare_parameter<double>("wheelbase_m", 0.18);

  declare_parameter<double>("filter_q_lateral", 0.0004);
  declare_parameter<double>("filter_q_heading", 0.0003);
  declare_parameter<double>("filter_r_lateral", 0.0025);
  declare_parameter<double>("filter_r_heading", 0.0020);
  declare_parameter<double>("gate_lateral_m", 0.12);
  declare_parameter<double>("gate_heading_rad", 0.35);

  declare_parameter<int>("valid_on_frames", 2);
  declare_parameter<int>("valid_off_frames", 3);
  declare_parameter<double>("track_half_width_m", 0.35);

  declare_parameter<double>("log_period_s", 2.0);
}

void LaneDetectorNode::loadParams()
{
  p_.input_topic = get_parameter("input_topic").as_string();
  p_.output_topic = get_parameter("output_topic").as_string();
  p_.debug_image_topic = get_parameter("debug_image_topic").as_string();
  p_.publish_debug_image = get_parameter("publish_debug_image").as_bool();

  p_.publish_bev_debug = get_parameter("publish_bev_debug").as_bool();
  p_.bev_debug_width = static_cast<int>(get_parameter("bev_debug_width").as_int());
  p_.bev_debug_height = static_cast<int>(get_parameter("bev_debug_height").as_int());

  p_.undistort = get_parameter("undistort").as_bool();
  p_.image_width = static_cast<int>(get_parameter("image_width").as_int());
  p_.image_height = static_cast<int>(get_parameter("image_height").as_int());
  p_.fx = get_parameter("fx").as_double();
  p_.fy = get_parameter("fy").as_double();
  p_.cx = get_parameter("cx").as_double();
  p_.cy = get_parameter("cy").as_double();

  p_.camera_height_m = get_parameter("camera_height_m").as_double();
  p_.camera_tilt_deg = get_parameter("camera_tilt_deg").as_double();
  p_.tilt_bias_deg = get_parameter("tilt_bias_deg").as_double();
  p_.publish_tilt_on_start = get_parameter("publish_tilt_on_start").as_bool();
  p_.tilt_topic = get_parameter("tilt_topic").as_string();
  p_.use_camera_pan = get_parameter("use_camera_pan").as_bool();
  p_.camera_to_rear_axle_m = get_parameter("camera_to_rear_axle_m").as_double();

  p_.roi_row_min = static_cast<int>(get_parameter("roi_row_min").as_int());
  p_.roi_row_max = static_cast<int>(get_parameter("roi_row_max").as_int());
  p_.detect_min_m = get_parameter("detect_min_m").as_double();
  p_.detect_max_m = get_parameter("detect_max_m").as_double();

  p_.lane_width_m = get_parameter("lane_width_m").as_double();
  p_.tophat_kernel_ratio = get_parameter("tophat_kernel_ratio").as_double();
  p_.tophat_strips = static_cast<int>(get_parameter("tophat_strips").as_int());

  mask_p_.tophat_threshold = static_cast<int>(get_parameter("tophat_threshold").as_int());
  mask_p_.saturation_max = static_cast<int>(get_parameter("saturation_max").as_int());
  mask_p_.bidirectional = get_parameter("tophat_bidirectional").as_bool();
  mask_p_.use_clahe = get_parameter("use_clahe").as_bool();
  mask_p_.clahe_clip = get_parameter("clahe_clip").as_double();
  mask_p_.clahe_grid = static_cast<int>(get_parameter("clahe_grid").as_int());
  mask_p_.morph_open = static_cast<int>(get_parameter("morph_open").as_int());
  mask_p_.morph_close = static_cast<int>(get_parameter("morph_close").as_int());

  run_p_.scan_rows = get_parameter("run_scan_rows").as_bool();
  run_p_.scan_cols = get_parameter("run_scan_cols").as_bool();
  run_p_.tol_lo = get_parameter("run_width_tol_lo").as_double();
  run_p_.tol_hi = get_parameter("run_width_tol_hi").as_double();
  run_p_.min_run_pixels = static_cast<int>(get_parameter("min_run_pixels").as_int());
  run_p_.row_step = static_cast<int>(get_parameter("row_step").as_int());
  run_p_.col_step = static_cast<int>(get_parameter("col_step").as_int());
  p_.run_merge_grid_m = get_parameter("run_merge_grid_m").as_double();

  fit_p_.iterations = static_cast<int>(get_parameter("irls_iterations").as_int());
  fit_p_.huber_delta = get_parameter("irls_huber_delta_m").as_double();
  fit_p_.min_points = static_cast<int>(get_parameter("min_points_per_line").as_int());
  fit_p_.seed_from_previous = get_parameter("seed_from_previous").as_bool();

  p_.inflection_enter_deg = get_parameter("inflection_enter_deg").as_double();
  p_.inflection_exit_deg = get_parameter("inflection_exit_deg").as_double();
  p_.inflection_residual_m = get_parameter("inflection_residual_m").as_double();
  p_.min_transition_points = static_cast<int>(get_parameter("min_transition_points").as_int());

  p_.use_motion_prediction = get_parameter("use_motion_prediction").as_bool();
  p_.speed_topic = get_parameter("speed_topic").as_string();
  p_.steer_topic = get_parameter("steer_topic").as_string();
  p_.wheelbase_m = get_parameter("wheelbase_m").as_double();

  p_.filter_q_lateral = get_parameter("filter_q_lateral").as_double();
  p_.filter_q_heading = get_parameter("filter_q_heading").as_double();
  p_.filter_r_lateral = get_parameter("filter_r_lateral").as_double();
  p_.filter_r_heading = get_parameter("filter_r_heading").as_double();
  p_.gate_lateral_m = get_parameter("gate_lateral_m").as_double();
  p_.gate_heading_rad = get_parameter("gate_heading_rad").as_double();

  p_.valid_on_frames = static_cast<int>(get_parameter("valid_on_frames").as_int());
  p_.valid_off_frames = static_cast<int>(get_parameter("valid_off_frames").as_int());
  p_.track_half_width_m = get_parameter("track_half_width_m").as_double();

  p_.log_period_s = get_parameter("log_period_s").as_double();

  inflection_enter_rad_ = p_.inflection_enter_deg * kDeg2Rad;
  inflection_exit_rad_ = p_.inflection_exit_deg * kDeg2Rad;

  if (p_.undistort) {
    RCLCPP_WARN(
      get_logger(),
      "[기동][lane] undistort=true 는 무시한다. 드라이버가 이미 폈다 (이중 보정)");
  }
  if (p_.use_camera_pan) {
    RCLCPP_WARN(get_logger(), "[기동][lane] pan 은 지원하지 않는다. use_camera_pan 무시");
  }
  if (p_.publish_bev_debug) {
    RCLCPP_WARN(
      get_logger(),
      "[기동][lane] publish_bev_debug 는 구현하지 않았다 (전체 warp 금지). 무시");
  }
}

// ================================================================
// 기하 사전계산 -- 카메라 고정이므로 기동 시 1회
// ================================================================

void LaneDetectorNode::buildGeometry()
{
  tilt_rad_ = (p_.camera_tilt_deg + p_.tilt_bias_deg) * kDeg2Rad;

  proj_.build(p_.fx, p_.fy, p_.cx, p_.cy, p_.camera_height_m, tilt_rad_);

  // 지평선 위 행은 지면과 만나지 않는다. ROI 상단을 그 아래로 민다.
  const int horizon = static_cast<int>(std::ceil(proj_.horizon_row)) + 1;
  roi_row0_ = std::max({0, p_.roi_row_min, horizon});
  roi_row1_ = std::min(p_.image_height, p_.roi_row_max + 1);

  if (roi_row1_ <= roi_row0_) {
    RCLCPP_ERROR(
      get_logger(), "[기동][lane] ROI 가 비었다 (행 %d~%d, 지평선 %.1f)",
      roi_row0_, roi_row1_, proj_.horizon_row);
    roi_row1_ = roi_row0_;
    return;
  }
  if (roi_row0_ > p_.roi_row_min) {
    RCLCPP_WARN(
      get_logger(), "[기동][lane] roi_row_min %d 이 지평선 위라 %d 로 올렸다",
      p_.roi_row_min, roi_row0_);
  }

  rows_ = buildRowTable(
    roi_row0_, roi_row1_ - 1, p_.fx, p_.fy, p_.cy, p_.camera_height_m,
    tilt_rad_, p_.camera_to_rear_axle_m, p_.lane_width_m);

  strips_ = buildStrips(rows_, p_.tophat_strips, p_.tophat_kernel_ratio);
}

void LaneDetectorNode::publishTiltOnce()
{
  if (!p_.publish_tilt_on_start) {return;}

  // 기동 시 1회. 드라이버가 늦게 떠도 받도록 latch 한다
  // (반복 발행은 웹UI 슬라이더를 되돌린다).
  tilt_pub_ = create_publisher<std_msgs::msg::Float64>(
    p_.tilt_topic, rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());

  std_msgs::msg::Float64 m;
  // /camera/tilt 규약: 절대각 [rad], + 가 아래. bias 는 광학 실측편차라 빼고 보낸다.
  m.data = p_.camera_tilt_deg * kDeg2Rad;
  tilt_pub_->publish(m);
}

void LaneDetectorNode::logStartup()
{
  if (rows_.rows.empty()) {return;}

  // 테이블은 행 중심(v+0.5)이지만 로그는 ROI 가 실제로 덮는 구간, 즉
  // 위 끝 행의 윗변과 아래 끝 행의 아랫변을 낸다 (설계문서 표와 같은 값).
  const RowGeometry far_row = rowGeometry(
    roi_row0_, p_.fx, p_.fy, p_.cy, p_.camera_height_m, tilt_rad_,
    p_.camera_to_rear_axle_m, p_.lane_width_m);
  const RowGeometry near_row = rowGeometry(
    roi_row1_, p_.fx, p_.fy, p_.cy, p_.camera_height_m, tilt_rad_,
    p_.camera_to_rear_axle_m, p_.lane_width_m);

  RCLCPP_INFO(
    get_logger(),
    "[기동][lane] tilt %.3f deg (bias %.3f), 지평선 행 %.1f, 후륜축 오프셋 %.3f m",
    p_.camera_tilt_deg + p_.tilt_bias_deg, p_.tilt_bias_deg,
    proj_.horizon_row, p_.camera_to_rear_axle_m);

  RCLCPP_INFO(
    get_logger(),
    "[기동][lane] ROI 행 %d~%d -> 후륜축 %.2f~%.2f m, 검출밴드 %.2f~%.2f m",
    roi_row0_, roi_row1_ - 1, near_row.x_rear, far_row.x_rear,
    p_.detect_min_m, p_.detect_max_m);

  for (size_t i = 0; i < strips_.size(); ++i) {
    const Strip & s = strips_[i];
    RCLCPP_INFO(
      get_logger(), "[기동][lane] 스트립 %zu: 행 %d~%d, 선폭 %.1f px, 커널 %d",
      i, roi_row0_ + s.row0, roi_row0_ + s.row1 - 1, s.lane_px, s.kernel);
  }

  // 규약 검산 행. 틸트 10.0 + bias 0.284, 실차 cy 169.603 에서
  // 각각 1.60 m / 13.9 px,  0.80 m / 31.5 px,  0.56 m / 49.9 px 여야 한다.
  const double check_rows[3] = {149.8, 184.8, 223.2};
  for (const double v : check_rows) {
    const RowGeometry g = rowGeometry(
      v, p_.fx, p_.fy, p_.cy, p_.camera_height_m, tilt_rad_,
      p_.camera_to_rear_axle_m, p_.lane_width_m);
    RCLCPP_INFO(
      get_logger(), "[기동][lane] 검산 행 %.1f -> 후륜축 %.2f m, 선폭 %.1f px",
      v, g.x_rear, g.lane_px_row);
  }

  RCLCPP_INFO(
    get_logger(),
    "[기동][lane] IRLS %d회 huber %.3f m, 최소 %d점, 시드 %s / 필터 %s / 히스테리시스 on %d off %d",
    fit_p_.iterations, fit_p_.huber_delta, fit_p_.min_points,
    fit_p_.seed_from_previous ? "이전해" : "없음",
    p_.use_motion_prediction ? "예측보정" : "EMA",
    p_.valid_on_frames, p_.valid_off_frames);
}

// ================================================================
// 콜백
// ================================================================

void LaneDetectorNode::onSpeed(const std_msgs::msg::Float64::ConstSharedPtr msg)
{
  speed_ = msg->data;
}

void LaneDetectorNode::onSteer(const std_msgs::msg::Float64::ConstSharedPtr msg)
{
  steer_ = msg->data;
}

void LaneDetectorNode::onImage(const sensor_msgs::msg::CompressedImage::ConstSharedPtr msg)
{
  // 예외를 콜백 밖으로 던지지 않는다 (규약 §11).
  try {
    if (msg->data.empty()) {return;}

    const cv::Mat buf(
      1, static_cast<int>(msg->data.size()), CV_8UC1,
      const_cast<uint8_t *>(msg->data.data()));
    const cv::Mat bgr = cv::imdecode(buf, cv::IMREAD_COLOR);
    if (bgr.empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), static_cast<int>(p_.log_period_s * 1000.0),
        "[검출][lane] imdecode 실패");
      return;
    }

    if (bgr.cols != p_.image_width || bgr.rows != p_.image_height) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), static_cast<int>(p_.log_period_s * 1000.0),
        "[검출][lane] 영상 %dx%d 이 보정값 %dx%d 과 다르다. 기하가 틀어진다",
        bgr.cols, bgr.rows, p_.image_width, p_.image_height);
      return;
    }

    process(bgr, msg->header);
  } catch (const std::exception & e) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), static_cast<int>(p_.log_period_s * 1000.0),
      "[검출][lane] 콜백 예외: %s", e.what());
  }
}

// ================================================================
// 점 처리
// ================================================================

void LaneDetectorNode::toGround(
  const std::vector<RunPoint> & runs, std::vector<Point2> & out) const
{
  out.clear();
  out.reserve(runs.size());

  for (const RunPoint & r : runs) {
    double x = 0.0;
    double y = 0.0;
    if (!proj_.project(r.u, r.v, x, y)) {continue;}
    x += p_.camera_to_rear_axle_m;
    if (x < p_.detect_min_m || x > p_.detect_max_m) {continue;}
    out.push_back({x, y});
  }
}

void LaneDetectorNode::mergeGrid(std::vector<Point2> & pts) const
{
  const double g = p_.run_merge_grid_m;
  if (g <= 0.0 || pts.empty()) {return;}

  struct Acc
  {
    double x = 0.0;
    double y = 0.0;
    int n = 0;
  };
  std::unordered_map<int64_t, Acc> cells;
  cells.reserve(pts.size() * 2);

  for (const Point2 & p : pts) {
    const int64_t ix = static_cast<int64_t>(std::llround(p.x / g));
    const int64_t iy = static_cast<int64_t>(std::llround(p.y / g));
    // 검출 밴드가 수 m 라 |iy| 는 수천을 넘지 않는다. 곱셈 키가 단사다.
    Acc & a = cells[ix * 1000000 + iy];
    a.x += p.x;
    a.y += p.y;
    ++a.n;
  }

  pts.clear();
  pts.reserve(cells.size());
  for (const auto & kv : cells) {
    const double n = static_cast<double>(kv.second.n);
    pts.push_back({kv.second.x / n, kv.second.y / n});
  }
}

void LaneDetectorNode::classify(
  const std::vector<Point2> & pts,
  std::vector<Point2> & left, std::vector<Point2> & right) const
{
  left.clear();
  right.clear();

  // 이전 중앙선을 기준으로 가른다. 없으면 y 부호 (출발 시 중앙 가정).
  const bool use_center = filter_init_;
  const double a = use_center ? std::tan(f_psi_) : 0.0;
  const double b = use_center ? f_y_ : 0.0;

  for (const Point2 & p : pts) {
    if (p.y - (a * p.x + b) >= 0.0) {
      left.push_back(p);
    } else {
      right.push_back(p);
    }
  }
}

// ================================================================
// 필터 -- 예측 보정 (상태 y@x=0, psi)
// ================================================================

void LaneDetectorNode::updateFilter(const Line & center, bool have_center, double dt)
{
  if (filter_init_) {
    // 예측. use_motion_prediction 이 꺼지면 이 블록만 빠지고
    // 아래 보정이 그대로 EMA (이득은 Q,R 의 정상상태 값) 가 된다.
    if (p_.use_motion_prediction && dt > 0.0) {
      const backup_common::Pose2 m =
        backup_common::integrateBicycle(speed_, steer_, p_.wheelbase_m, dt);
      double x0 = 0.0;
      double y0 = f_y_;
      double x1 = 1.0;
      double y1 = f_y_ + std::tan(f_psi_);
      backup_common::transformPoint(m, x0, y0);
      backup_common::transformPoint(m, x1, y1);
      const double dx = x1 - x0;
      if (std::fabs(dx) > 1e-6) {
        const double a = (y1 - y0) / dx;
        f_psi_ = std::atan(a);
        f_y_ = y0 - a * x0;
      }
    }
    f_var_y_ += p_.filter_q_lateral;
    f_var_psi_ += p_.filter_q_heading;
  }

  if (!have_center) {return;}

  const double meas_y = center.b;
  const double meas_psi = center.heading();

  if (!filter_init_) {
    f_y_ = meas_y;
    f_psi_ = meas_psi;
    f_var_y_ = p_.filter_r_lateral;
    f_var_psi_ = p_.filter_r_heading;
    filter_init_ = true;
    return;
  }

  const double dy = meas_y - f_y_;
  const double dpsi = wrapAngle(meas_psi - f_psi_);

  if (std::fabs(dy) > p_.gate_lateral_m || std::fabs(dpsi) > p_.gate_heading_rad) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), static_cast<int>(p_.log_period_s * 1000.0),
      "[필터][lane] 게이트 이탈 측정 폐기 (dy %.3f m, dpsi %.3f rad)", dy, dpsi);
    return;
  }

  const double ky = f_var_y_ / (f_var_y_ + p_.filter_r_lateral);
  f_y_ += ky * dy;
  f_var_y_ *= (1.0 - ky);

  const double kp = f_var_psi_ / (f_var_psi_ + p_.filter_r_heading);
  f_psi_ = wrapAngle(f_psi_ + kp * dpsi);
  f_var_psi_ *= (1.0 - kp);
}

// ================================================================
// 본 처리
// ================================================================

void LaneDetectorNode::process(const cv::Mat & bgr, const std_msgs::msg::Header & header)
{
  if (roi_row1_ <= roi_row0_ || rows_.rows.empty()) {return;}

  // 시각은 this->now() 계열만 쓴다 (std::chrono 금지). 첫 프레임의 dt 는 버린다.
  const rclcpp::Time stamp(header.stamp);
  double dt = 0.0;
  if (have_stamp_) {dt = (stamp - last_stamp_).seconds();}
  last_stamp_ = stamp;
  have_stamp_ = true;
  if (dt < 0.0) {dt = 0.0;}

  const cv::Mat roi = bgr.rowRange(roi_row0_, roi_row1_);

  cv::Mat mask;
  cv::Mat tophat;
  buildWhiteMask(roi, strips_, mask_p_, mask, tophat);

  std::vector<RunPoint> runs;
  extractRuns(mask, tophat, rows_, roi_row0_, run_p_, runs);

  std::vector<Point2> ground;
  toGround(runs, ground);
  mergeGrid(ground);

  std::vector<Point2> left_pts;
  std::vector<Point2> right_pts;
  classify(ground, left_pts, right_pts);

  const auto by_x = [](const Point2 & a, const Point2 & b) {return a.x < b.x;};
  std::sort(left_pts.begin(), left_pts.end(), by_x);
  std::sort(right_pts.begin(), right_pts.end(), by_x);

  Line fit_left;
  Line fit_right;
  const bool ok_left = fitIRLS(left_pts, fit_p_, &prev_left_, fit_left);
  const bool ok_right = fitIRLS(right_pts, fit_p_, &prev_right_, fit_right);

  left_state_.update(ok_left, p_.valid_on_frames, p_.valid_off_frames);
  right_state_.update(ok_right, p_.valid_on_frames, p_.valid_off_frames);

  // 시드는 히스테리시스가 살아 있는 동안만 붙잡는다. 무효로 떨어지면
  // 낡은 해가 다음 복귀 프레임을 끌어당기므로 버린다.
  if (ok_left) {
    prev_left_ = fit_left;
  } else if (!left_state_.valid) {
    prev_left_ = Line{};
  }
  if (ok_right) {
    prev_right_ = fit_right;
  } else if (!right_state_.valid) {
    prev_right_ = Line{};
  }

  if (ok_left || ok_right) {
    stale_dist_ = 0.0;
  } else {
    stale_dist_ += std::fabs(speed_) * dt;
  }

  // 히스테리시스를 통과한 쪽만 쓴다. 한 프레임 깜빡임으로 복원 모드가
  // 뒤집히면 중앙선이 반폭만큼 점프한다 (모드 전환 불연속).
  const Line & pub_left = ok_left ? fit_left : prev_left_;
  const Line & pub_right = ok_right ? fit_right : prev_right_;
  const bool use_left = left_state_.valid && pub_left.valid;
  const bool use_right = right_state_.valid && pub_right.valid;

  Line center;
  bool have_center = false;
  const double hw = p_.track_half_width_m;

  if (use_left && use_right) {
    // 각 x 에서의 중점.
    center.a = 0.5 * (pub_left.a + pub_right.a);
    center.b = 0.5 * (pub_left.b + pub_right.b);
    center.confidence = 0.5f * (pub_left.confidence + pub_right.confidence);
    have_center = true;
  } else if (use_left) {
    // 한쪽만 보이면 트랙 반폭만큼 수직으로 옮겨 중앙선을 복원한다.
    center.a = pub_left.a;
    center.b = pub_left.b - hw * std::sqrt(1.0 + pub_left.a * pub_left.a);
    center.confidence = pub_left.confidence;
    have_center = true;
  } else if (use_right) {
    center.a = pub_right.a;
    center.b = pub_right.b + hw * std::sqrt(1.0 + pub_right.a * pub_right.a);
    center.confidence = pub_right.confidence;
    have_center = true;
  }
  center.valid = have_center;

  // 이번 프레임 피팅이 실패했으면 측정이 없다 -- 예측만 돌린다.
  updateFilter(center, have_center && (ok_left || ok_right), dt);

  // ---- 변곡점 ----
  // 점이 많은 쪽에서 찾는다. 좌우 차선은 같은 지점에서 꺾이고,
  // 중앙선 점군을 합성하면 반폭 오프셋 오차가 잔차에 섞인다.
  bool inflect_now = false;
  Split split;
  bool second = false;
  const std::vector<Point2> * src = nullptr;
  bool src_is_left = true;

  if (ok_left && (!ok_right || left_pts.size() >= right_pts.size())) {
    src = &left_pts;
    src_is_left = true;
  } else if (ok_right) {
    src = &right_pts;
    src_is_left = false;
  }

  if (src != nullptr) {
    const Line & single = src_is_left ? fit_left : fit_right;
    if (single.rms > p_.inflection_residual_m) {
      if (splitAtInflection(
          *src, 0, src->size(), fit_p_, p_.inflection_residual_m,
          p_.min_transition_points, split))
      {
        const double th = inflected_ ? inflection_exit_rad_ : inflection_enter_rad_;
        inflect_now = std::fabs(split.delta_psi) >= th;

        if (inflect_now) {
          // 시야 안에 변곡점이 하나 더 있으면 연속 코너다.
          Split s2;
          const size_t k = static_cast<size_t>(split.a.n);
          if (split.b.rms > p_.inflection_residual_m &&
            splitAtInflection(
              *src, k, src->size(), fit_p_, p_.inflection_residual_m,
              p_.min_transition_points, s2))
          {
            second = std::fabs(s2.delta_psi) >= inflection_enter_rad_;
          }
          if (!second && split.a.rms > p_.inflection_residual_m &&
            splitAtInflection(
              *src, 0, k, fit_p_, p_.inflection_residual_m,
              p_.min_transition_points, s2))
          {
            second = std::fabs(s2.delta_psi) >= inflection_enter_rad_;
          }
        }
      }
    }
  }
  inflected_ = inflect_now;
  if (inflect_now) {last_delta_psi_ = split.delta_psi;}

  // ---- 손실 상태 ----
  uint8_t lost = LaneGeometry::LOST_NONE;
  if (!use_left && !use_right) {
    lost = LaneGeometry::LOST_BOTH;
  } else if (!use_left) {
    lost = LaneGeometry::LOST_LEFT;
  } else if (!use_right) {
    lost = LaneGeometry::LOST_RIGHT;
  }

  if (lost == LaneGeometry::LOST_BOTH) {
    if (prev_lost_ != LaneGeometry::LOST_BOTH) {
      if (prev_lost_ == LaneGeometry::LOST_LEFT || prev_lost_ == LaneGeometry::LOST_RIGHT) {
        lost_first_ = prev_lost_;
      } else {
        // 동시 손실. 좌회전이면 안쪽인 좌측이 먼저 시야를 벗어난다.
        lost_first_ = (last_delta_psi_ > 0.0) ? LaneGeometry::LOST_LEFT :
          (last_delta_psi_ < 0.0 ? LaneGeometry::LOST_RIGHT : LaneGeometry::LOST_NONE);
      }
    }
  } else {
    lost_first_ = LaneGeometry::LOST_NONE;
  }
  prev_lost_ = lost;

  // ---- valid_length: 실제 검출 최원거리. 외삽 구간을 넣지 않는다 ----
  // 붙잡은 직전 해를 쓰는 프레임에서는 그동안 나아간 거리만큼 줄인다.
  double valid_length = 0.0;
  if (use_left) {valid_length = std::max(valid_length, pub_left.x1);}
  if (use_right) {valid_length = std::max(valid_length, pub_right.x1);}
  valid_length = std::max(0.0, valid_length - stale_dist_);

  // ---- 발행 ----
  LaneGeometry out;
  out.header.stamp = header.stamp;
  out.header.frame_id = "base_link";

  const auto fill = [](const Line & l, backup_msgs::msg::LaneLine & m) {
      m.x0 = l.x0;
      m.y0 = l.at(l.x0);
      m.x1 = l.x1;
      m.y1 = l.at(l.x1);
      m.confidence = l.confidence;
    };

  if (use_left) {fill(pub_left, out.left);}
  if (use_right) {fill(pub_right, out.right);}
  out.left_valid = use_left;
  out.right_valid = use_right;

  out.center_valid = filter_init_ && (use_left || use_right);
  if (filter_init_) {
    Line fc;
    fc.a = std::tan(f_psi_);
    fc.b = f_y_;
    fc.x0 = p_.detect_min_m;
    fc.x1 = std::max(valid_length, p_.detect_min_m);
    fc.confidence = center.confidence;
    fill(fc, out.center);
  }

  out.inflection_valid = inflect_now;
  if (inflect_now) {
    // 교점은 검출한 쪽 차선 위다. 중앙선으로 반폭 옮긴다.
    const double th = 0.5 * (split.a.heading() + split.b.heading());
    const double sgn = src_is_left ? 1.0 : -1.0;
    out.inflection_x = split.x + sgn * hw * std::sin(th);
    out.inflection_y = split.y - sgn * hw * std::cos(th);
    out.delta_psi = split.delta_psi;
    out.arc_length = split.arc_length;
  }
  out.second_inflection = second;

  out.lost_state = lost;
  out.lost_first = lost_first_;
  out.valid_length = valid_length;

  lane_pub_->publish(out);

  if (lost == LaneGeometry::LOST_BOTH) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), static_cast<int>(p_.log_period_s * 1000.0),
      "[검출][lane] 양쪽 차선 미검출 (점 %zu, 먼저 끊긴 쪽 %u)", ground.size(), lost_first_);
  } else {
    RCLCPP_DEBUG_THROTTLE(
      get_logger(), *get_clock(), static_cast<int>(p_.log_period_s * 1000.0),
      "[검출][lane] 점 %zu (좌 %zu 우 %zu), 유효거리 %.2f m, y %.3f m psi %.1f deg, 변곡 %s",
      ground.size(), left_pts.size(), right_pts.size(), valid_length,
      f_y_, f_psi_ * kRad2Deg, inflect_now ? "예" : "아니오");
  }

  if (p_.publish_debug_image && debug_pub_) {
    publishDebugImage(roi, mask, runs, out.header);
  }
}

void LaneDetectorNode::publishDebugImage(
  const cv::Mat & roi, const cv::Mat & mask,
  const std::vector<RunPoint> & runs, const std_msgs::msg::Header & header)
{
  cv::Mat dbg = roi.clone();
  dbg.setTo(cv::Scalar(0, 255, 0), mask);

  for (const RunPoint & r : runs) {
    const cv::Point pt(
      static_cast<int>(std::lround(r.u)),
      static_cast<int>(std::lround(r.v)) - roi_row0_);
    if (pt.y < 0 || pt.y >= dbg.rows || pt.x < 0 || pt.x >= dbg.cols) {continue;}
    cv::circle(dbg, pt, 1, cv::Scalar(0, 0, 255), -1);
  }

  debug_pub_->publish(*cv_bridge::CvImage(header, "bgr8", dbg).toImageMsg());
}

}  // namespace backup_lane_detection
