#include "backup_steer_controller/steer_controller_node.hpp"

#include <algorithm>
#include <cmath>

#include <backup_common/motion.hpp>
#include <backup_common/vehicle.hpp>
#include <backup_msgs/msg/lane_geometry.hpp>

namespace backup_steer_controller
{

using backup_msgs::msg::LaneGeometry;

namespace
{
// 내부 토픽 · 차량 출력 QoS (규약 §8)
rclcpp::QoS controlQos()
{
  return rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();
}
}  // namespace

SteerControllerNode::SteerControllerNode()
: rclcpp::Node("backup_steer_controller")
{
  path_topic_ = declare_parameter<std::string>("path_topic", "/backup/path");
  speed_topic_ = declare_parameter<std::string>("speed_topic", "/speed");
  output_topic_ = declare_parameter<std::string>("output_topic", "/steering");
  debug_topic_ = declare_parameter<std::string>("debug_topic", "/backup/debug/steer");
  publish_debug_ = declare_parameter<bool>("publish_debug", true);

  control_hz_ = declare_parameter<double>("control_hz", 50.0);
  path_timeout_s_ = declare_parameter<double>("path_timeout_s", 0.4);

  wheelbase_m_ = declare_parameter<double>("wheelbase_m", 0.18);
  const double max_steer_deg = declare_parameter<double>("max_steer_deg", 20.0);
  const double max_steer_rate_deg_s = declare_parameter<double>("max_steer_rate_deg_s", 300.0);
  max_steer_rad_ = max_steer_deg * backup_common::kDeg2Rad;
  max_steer_rate_rad_s_ = max_steer_rate_deg_s * backup_common::kDeg2Rad;

  lookahead_k_v_ = declare_parameter<double>("lookahead_k_v", 0.85);
  lookahead_min_m_ = declare_parameter<double>("lookahead_min_m", 0.80);
  lookahead_max_m_ = declare_parameter<double>("lookahead_max_m", 1.20);
  clamp_to_valid_length_ = declare_parameter<bool>("clamp_to_valid_length", true);
  valid_length_margin_m_ = declare_parameter<double>("valid_length_margin_m", 0.10);

  use_dead_reckoning_ = declare_parameter<bool>("use_dead_reckoning", true);
  max_dead_reckon_s_ = declare_parameter<double>("max_dead_reckon_s", 0.30);
  arclength_lut_samples_ = declare_parameter<int>("arclength_lut_samples", 64);

  saturation_report_ = declare_parameter<bool>("saturation_report", true);
  log_period_s_ = declare_parameter<double>("log_period_s", 2.0);

  if (control_hz_ <= 0.0) {control_hz_ = 50.0;}
  if (arclength_lut_samples_ < 1) {arclength_lut_samples_ = 1;}
  if (wheelbase_m_ <= 0.0) {wheelbase_m_ = 0.18;}

  const rclcpp::Time t0 = now();
  path_rx_ = t0;
  path_stamp_ = t0;
  prev_tick_ = t0;

  steer_pub_ = create_publisher<std_msgs::msg::Float64>(output_topic_, controlQos());
  if (publish_debug_) {
    debug_pub_ = create_publisher<backup_msgs::msg::ControlDebug>(debug_topic_, controlQos());
  }

  path_sub_ = create_subscription<backup_msgs::msg::BackupPath>(
    path_topic_, controlQos(),
    std::bind(&SteerControllerNode::onPath, this, std::placeholders::_1));

  speed_sub_ = create_subscription<std_msgs::msg::Float64>(
    speed_topic_, controlQos(),
    std::bind(&SteerControllerNode::onSpeed, this, std::placeholders::_1));

  timer_ = rclcpp::create_timer(
    this, get_clock(), rclcpp::Duration::from_seconds(1.0 / control_hz_),
    std::bind(&SteerControllerNode::onTimer, this));

  logStartup();
}

void SteerControllerNode::logStartup() const
{
  RCLCPP_INFO(
    get_logger(),
    "[기동][steer] %.0f Hz, L %.3f m, delta 한계 %.1f deg, 슬루 %.0f deg/s",
    control_hz_, wheelbase_m_, max_steer_rad_ * backup_common::kRad2Deg,
    max_steer_rate_rad_s_ * backup_common::kRad2Deg);
  RCLCPP_INFO(
    get_logger(),
    "[기동][steer] Ld = %.2f*v, %.2f~%.2f m, valid 클램프 %s(여유 %.2f m), "
    "데드레커닝 %s(최대 %.2f s), LUT %d/seg, timeout %.2f s",
    lookahead_k_v_, lookahead_min_m_, lookahead_max_m_,
    clamp_to_valid_length_ ? "on" : "off", valid_length_margin_m_,
    use_dead_reckoning_ ? "on" : "off", max_dead_reckon_s_,
    arclength_lut_samples_, path_timeout_s_);
  RCLCPP_INFO(
    get_logger(), "[기동][steer] 입력 %s / %s, 출력 %s [rad]",
    path_topic_.c_str(), speed_topic_.c_str(), output_topic_.c_str());
}

// /speed 는 speed_controller 의 **명령** 속도다. 실측이 아니다 --
// 백업 스택은 odom 비의존이 전제라 피드백을 받지 않는다.
// 차량이 명령을 못 따라가면 Ld 와 데드레커닝이 그만큼 어긋난다.
void SteerControllerNode::onSpeed(const std_msgs::msg::Float64::SharedPtr msg)
{
  try {
    v_ = std::max(0.0, msg->data);
  } catch (const std::exception & e) {
    RCLCPP_WARN(get_logger(), "[제어][steer] speed 콜백 예외: %s", e.what());
  }
}

void SteerControllerNode::onPath(const backup_msgs::msg::BackupPath::SharedPtr msg)
{
  try {
    const size_t per = static_cast<size_t>(backup_common::kCtrlPerSeg);
    const size_t n = msg->ctrl_x.size();
    if (n == 0 || n != msg->ctrl_y.size() || n % per != 0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), static_cast<int>(log_period_s_ * 1000.0),
        "[경로][steer] 제어점 길이 불량 (%zu)", n);
      return;
    }

    path_.clear();
    for (size_t k = 0; k + per <= n; k += per) {
      backup_common::Segment s;
      for (size_t i = 0; i < per; ++i) {
        s.p[i] = {msg->ctrl_x[k + i], msg->ctrl_y[k + i]};
      }
      path_.addSegment(s);
    }
    // 호길이 LUT 는 여기서 1 회만. 50 Hz 에서는 보간만 한다.
    path_.buildLut(arclength_lut_samples_);

    valid_length_ = msg->valid_length;
    kappa_saturated_ = msg->kappa_saturated;
    lost_state_ = msg->lost_state;
    lost_first_ = msg->lost_first;
    path_source_ = msg->source;

    path_stamp_ = rclcpp::Time(msg->header.stamp, get_clock()->get_clock_type());
    path_rx_ = now();
    have_path_ = true;
  } catch (const std::exception & e) {
    RCLCPP_WARN(get_logger(), "[경로][steer] 경로 콜백 예외: %s", e.what());
  }
}

// 먼저 끊긴 쪽으로 최대 조향. LOST_LEFT 는 좌(+), LOST_RIGHT 는 우(-).
double SteerControllerNode::lostBothSteer() const
{
  if (lost_first_ == LaneGeometry::LOST_LEFT) {return max_steer_rad_;}
  if (lost_first_ == LaneGeometry::LOST_RIGHT) {return -max_steer_rad_;}
  if (last_kappa_sign_ > 0.0) {return max_steer_rad_;}
  if (last_kappa_sign_ < 0.0) {return -max_steer_rad_;}
  return 0.0;
}

SteerControllerNode::Tick SteerControllerNode::computeTick(const rclcpp::Time & now_t)
{
  Tick t;

  // 1. LOST_BOTH -- 기하가 없다. 먼저 끊긴 쪽으로 전타한다.
  if (have_path_ && lost_state_ == LaneGeometry::LOST_BOTH) {
    t.delta_target = lostBothSteer();
    t.lookahead_m = 0.0;
    t.reason = "LOST_BOTH 전타";
    return t;
  }

  // 2. 경로 없음 / 수신 끊김 -- delta 유지. 0 으로 튕기면 차선을 벗어난다.
  //    속도 게이트가 /speed 를 0 으로 만든다.
  if (!have_path_ || path_.empty() ||
    (now_t - path_rx_).seconds() > path_timeout_s_)
  {
    t.delta_target = delta_cmd_;
    t.reason = have_path_ ? "경로 수신 끊김 -- delta 유지" : "경로 미수신 -- delta 유지";
    return t;
  }

  // 3. 경로 stamp -> now 보상.
  //    강체 변환은 호길이를 보존하므로 LUT 를 다시 만들 필요가 없다.
  //    이동한 호길이만큼 조회점을 밀고, 그 점 하나만 현재 body frame 으로 옮긴다.
  if (use_dead_reckoning_) {
    t.dead_reckon_dt =
      std::max(0.0, std::min((now_t - path_stamp_).seconds(), max_dead_reckon_s_));
  }
  const double s_travel = v_ * t.dead_reckon_dt;

  // 4. Ld = clamp(k_v * v, min, max), 외삽 구간을 쫓지 않게 valid_length 로 절단.
  double ld = std::max(lookahead_min_m_, std::min(lookahead_k_v_ * v_, lookahead_max_m_));
  if (clamp_to_valid_length_) {
    const double reach = valid_length_ - valid_length_margin_m_ - s_travel;
    // 절단은 하되 lookahead_min_m 아래로는 내리지 않는다 (루프 게인 2L/Ld^2 폭주 방지).
    ld = std::min(ld, std::max(reach, lookahead_min_m_));
  }

  // 5. 목표점. 조회는 호길이로 한다.
  const double s_query = s_travel + ld;
  backup_common::Point2 target = path_.pointAtArcLength(s_query);
  if (t.dead_reckon_dt > 0.0) {
    backup_common::deadReckonPoint(
      v_, delta_cmd_, wheelbase_m_, t.dead_reckon_dt, target.x, target.y);
  }

  // 6. Pure Pursuit. kappa = 2*sin(alpha)/Ld 는 Ld 가 현(직선거리)일 때만 맞다.
  //    호 1.2 m / R 1.39 m 면 현은 1.163 m -- 곡률에 3.1 % 오차가 붙는다.
  double ld_eff = std::hypot(target.x, target.y);
  // 게인 2L/Ld^2 가 폭주하지 않게 하한을 유지한다.
  ld_eff = std::max(lookahead_min_m_, std::min(ld_eff, lookahead_max_m_));
  t.lookahead_m = ld_eff;

  const double delta_pp =
    backup_common::purePursuitSteer(target.x, target.y, wheelbase_m_, ld_eff);
  t.alpha_rad = std::atan2(target.y, target.x);
  t.saturated = std::fabs(delta_pp) > max_steer_rad_;
  t.delta_target = backup_common::clampAbs(delta_pp, max_steer_rad_);
  t.tracking_ok = true;

  const double k = path_.curvatureAtArcLength(s_query);
  if (k != 0.0) {last_kappa_sign_ = (k > 0.0) ? 1.0 : -1.0;}

  return t;
}

void SteerControllerNode::onTimer()
{
  try {
    const rclcpp::Time now_t = now();

    // 첫 tick 은 /clock 이 아직 오지 않아 dt 가 0 이거나 음수일 수 있다.
    double dt = have_prev_tick_ ? (now_t - prev_tick_).seconds() : 0.0;
    if (dt <= 0.0) {dt = 1.0 / control_hz_;}
    prev_tick_ = now_t;
    have_prev_tick_ = true;

    const Tick t = computeTick(now_t);

    // 슬루 제한은 채터 방지용이지 필터가 아니다.
    const double step = max_steer_rate_rad_s_ * dt;
    delta_cmd_ += backup_common::clampAbs(t.delta_target - delta_cmd_, step);
    delta_cmd_ = backup_common::clampAbs(delta_cmd_, max_steer_rad_);

    publishSteer(delta_cmd_);
    publishDebug(now_t, t);

    if (!t.tracking_ok) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), static_cast<int>(log_period_s_ * 1000.0),
        "[게이트][steer] %s -> %.1f deg", t.reason,
        delta_cmd_ * backup_common::kRad2Deg);
    } else {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), static_cast<int>(log_period_s_ * 1000.0),
        "[제어][steer] delta %.1f deg, Ld %.2f m, v %.2f m/s, dr %.0f ms%s",
        delta_cmd_ * backup_common::kRad2Deg, t.lookahead_m, v_,
        t.dead_reckon_dt * 1000.0, t.saturated ? ", 포화" : "");
    }
  } catch (const std::exception & e) {
    // 조향은 0 으로 튕기지 않는다. 마지막 값을 유지한 채 경고만 낸다.
    publishSteer(delta_cmd_);
    RCLCPP_WARN(get_logger(), "[제어][steer] tick 예외: %s", e.what());
  }
}

void SteerControllerNode::publishSteer(double rad)
{
  std_msgs::msg::Float64 m;
  m.data = rad;
  steer_pub_->publish(m);
}

void SteerControllerNode::publishDebug(const rclcpp::Time & now_t, const Tick & t)
{
  if (!publish_debug_ || !debug_pub_) {return;}

  backup_msgs::msg::ControlDebug d;
  d.header.stamp = now_t;
  d.header.frame_id = "base_link";
  d.path_source = path_source_;
  d.tracking_ok = t.tracking_ok;
  d.cmd_steer_deg = delta_cmd_ * backup_common::kRad2Deg;
  d.lookahead_m = t.lookahead_m;
  // 목표점 방위. 경로 접선 헤딩은 Path 가 노출하지 않는다.
  d.heading_error_rad = t.alpha_rad;
  // 경로 stamp 시점 원점 기준. 표시용이라 보상하지 않는다.
  d.cross_track_m = path_.empty() ? 0.0 : path_.crossTrackAtOrigin();
  d.delta_saturated = saturation_report_ && t.saturated;
  d.kappa_saturated = kappa_saturated_;
  d.dead_reckon_dt = t.dead_reckon_dt;
  debug_pub_->publish(d);
}

}  // namespace backup_steer_controller
