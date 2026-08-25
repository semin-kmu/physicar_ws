#include "backup_speed_controller/speed_controller_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <thread>

#include <backup_msgs/msg/lane_geometry.hpp>

namespace backup_speed_controller
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

SpeedControllerNode::SpeedControllerNode()
: rclcpp::Node("backup_speed_controller")
{
  path_topic_ = declare_parameter<std::string>("path_topic", "/backup/path");
  permission_topic_ =
    declare_parameter<std::string>("permission_topic", "/perception/start_permission");
  output_topic_ = declare_parameter<std::string>("output_topic", "/speed");
  debug_topic_ = declare_parameter<std::string>("debug_topic", "/backup/debug/speed");
  publish_debug_ = declare_parameter<bool>("publish_debug", true);

  control_hz_ = declare_parameter<double>("control_hz", 50.0);
  path_timeout_s_ = declare_parameter<double>("path_timeout_s", 0.4);

  prof_.v_max = declare_parameter<double>("v_max", 1.40);
  prof_.v_min = declare_parameter<double>("v_min", 0.50);
  a_accel_ = declare_parameter<double>("a_accel", 1.5);
  prof_.a_decel = declare_parameter<double>("a_decel", 0.5);
  max_accel_step_ = declare_parameter<double>("max_accel_step", 0.030);
  max_decel_step_ = declare_parameter<double>("max_decel_step", 0.010);

  prof_.a_lat_max = declare_parameter<double>("a_lat_max", 3.0);
  preview_m_ = declare_parameter<double>("preview_m", 1.60);
  prof_.t_lat_s = declare_parameter<double>("t_lat_s", 0.10);

  require_start_permission_ = declare_parameter<bool>("require_start_permission", true);
  lost_hold_s_ = declare_parameter<double>("lost_hold_s", 0.6);

  use_dead_reckoning_ = declare_parameter<bool>("use_dead_reckoning", true);
  max_dead_reckon_s_ = declare_parameter<double>("max_dead_reckon_s", 0.30);

  shutdown_zero_count_ = declare_parameter<int>("shutdown_zero_count", 3);
  shutdown_zero_interval_ms_ = declare_parameter<int>("shutdown_zero_interval_ms", 20);
  shutdown_linger_ms_ = declare_parameter<int>("shutdown_linger_ms", 200);
  log_period_s_ = declare_parameter<double>("log_period_s", 2.0);

  if (control_hz_ <= 0.0) {control_hz_ = 50.0;}

  const rclcpp::Time t0 = now();
  path_rx_ = t0;
  path_stamp_ = t0;
  lost_since_ = t0;
  prev_tick_ = t0;

  speed_pub_ = create_publisher<std_msgs::msg::Float64>(output_topic_, controlQos());
  if (publish_debug_) {
    debug_pub_ = create_publisher<backup_msgs::msg::ControlDebug>(debug_topic_, controlQos());
  }

  path_sub_ = create_subscription<backup_msgs::msg::BackupPath>(
    path_topic_, controlQos(),
    std::bind(&SpeedControllerNode::onPath, this, std::placeholders::_1));

  // 발행자(start_signal_detector)와 같은 QoS 여야 매칭된다.
  permission_sub_ = create_subscription<std_msgs::msg::Bool>(
    permission_topic_, controlQos(),
    std::bind(&SpeedControllerNode::onPermission, this, std::placeholders::_1));

  timer_ = rclcpp::create_timer(
    this, get_clock(), rclcpp::Duration::from_seconds(1.0 / control_hz_),
    std::bind(&SpeedControllerNode::onTimer, this));

  // 컨텍스트가 아직 살아 있는 시점이어야 0 이 실제로 나간다.
  auto ctx = get_node_base_interface()->get_context();
  ctx->add_pre_shutdown_callback([this]() {publishShutdownZeros();});

  logStartup();
}

void SpeedControllerNode::logStartup() const
{
  RCLCPP_INFO(
    get_logger(),
    "[기동][speed] %.0f Hz, v %.2f~%.2f m/s, a %.2f/-%.2f m/s^2, preview %.2f m, "
    "a_lat %.2f m/s^2, timeout %.2f s, permission %s",
    control_hz_, prof_.v_min, prof_.v_max, a_accel_, prof_.a_decel, preview_m_,
    prof_.a_lat_max, path_timeout_s_, require_start_permission_ ? "필수" : "무시");
  RCLCPP_INFO(
    get_logger(), "[기동][speed] 입력 %s / %s, 출력 %s",
    path_topic_.c_str(), permission_topic_.c_str(), output_topic_.c_str());
}

void SpeedControllerNode::onPermission(const std_msgs::msg::Bool::SharedPtr msg)
{
  try {
    const bool prev = permission_ && permission_seen_;
    permission_ = msg->data;
    permission_seen_ = true;
    if (permission_ != prev) {
      RCLCPP_INFO(
        get_logger(), "[게이트][speed] start permission %s",
        permission_ ? "true" : "false");
    }
  } catch (const std::exception & e) {
    RCLCPP_WARN(get_logger(), "[게이트][speed] permission 콜백 예외: %s", e.what());
  }
}

void SpeedControllerNode::onPath(const backup_msgs::msg::BackupPath::SharedPtr msg)
{
  try {
    const size_t per = static_cast<size_t>(backup_common::kCtrlPerSeg);
    const size_t n = msg->ctrl_x.size();
    if (n == 0 || n != msg->ctrl_y.size() || n % per != 0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), static_cast<int>(log_period_s_ * 1000.0),
        "[경로][speed] 제어점 길이 불량 (%zu)", n);
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
    // 표본수는 backup_common 기본값을 쓴다 (yaml 에 없는 상수를 만들지 않는다).
    path_.buildLut();

    valid_length_ = msg->valid_length;
    corner_entry_s_ = msg->corner_entry_s;
    corner_kappa_max_ = msg->corner_kappa_max;
    kappa_saturated_ = msg->kappa_saturated;
    stop_request_ = msg->stop_request;
    lost_state_ = msg->lost_state;
    path_source_ = msg->source;

    path_stamp_ = rclcpp::Time(msg->header.stamp, get_clock()->get_clock_type());
    path_rx_ = now();
    have_path_ = true;
  } catch (const std::exception & e) {
    RCLCPP_WARN(get_logger(), "[경로][speed] 경로 콜백 예외: %s", e.what());
  }
}

void SpeedControllerNode::updateLostTimer(const rclcpp::Time & now_t)
{
  const bool lost_both = have_path_ && lost_state_ == LaneGeometry::LOST_BOTH;
  if (lost_both && !lost_active_) {
    lost_active_ = true;
    lost_since_ = now_t;
  } else if (!lost_both) {
    lost_active_ = false;
  }
}

SpeedControllerNode::GateResult SpeedControllerNode::evaluateGates(const rclcpp::Time & now_t)
{
  GateResult g;

  // 1. permission -- 한 번도 못 받았으면 0 이지 v_min 이 아니다.
  if (require_start_permission_ && !(permission_seen_ && permission_)) {
    g.target = 0.0;
    g.hard_zero = true;
    g.reason = "start permission 없음";
    return g;
  }

  // 2. stop_request -- 0 으로 감속
  if (have_path_ && stop_request_) {
    g.target = 0.0;
    g.reason = "stop_request";
    return g;
  }

  // 3. 조향 포화 -- 조향으로 더 할 게 없어 속도로 넘어온 것
  if (have_path_ && kappa_saturated_) {
    g.target = prof_.v_min;
    g.reason = "kappa_saturated";
    return g;
  }

  // 4. 양쪽 차선 손실 -- v_min 유지, lost_hold_s 초과 시 0
  if (lost_active_) {
    if ((now_t - lost_since_).seconds() > lost_hold_s_) {
      g.target = 0.0;
      g.hard_zero = true;
      g.reason = "LOST_BOTH lost_hold_s 초과";
    } else {
      g.target = prof_.v_min;
      g.reason = "LOST_BOTH";
    }
    return g;
  }

  // 5. 경로 수신 끊김
  if (!have_path_ || (now_t - path_rx_).seconds() > path_timeout_s_) {
    g.target = 0.0;
    g.hard_zero = true;
    g.reason = "경로 수신 끊김";
    return g;
  }

  g.normal = true;
  return g;
}

double SpeedControllerNode::profileTarget(double s_travel, double * kappa_ahead_out)
{
  // 경로 근거가 닿는 길이를 넘어서 곡률을 보지 않는다.
  const double reach = (valid_length_ > 0.0) ? valid_length_ : path_.totalLength();
  const double s_end = std::min(s_travel + preview_m_, reach);
  const double kappa_ahead = path_.maxAbsCurvatureUpTo(s_end);
  if (kappa_ahead_out) {*kappa_ahead_out = kappa_ahead;}

  double v_target = clampSpeed(std::min(prof_.v_max, curveSpeed(kappa_ahead, prof_)), prof_);

  // 코너 진입 -- 제동거리 안에 들어오면 코너 통과속도로 미리 내린다.
  if (corner_entry_s_ >= 0.0 && corner_kappa_max_ > 0.0) {
    const double v_corner = clampSpeed(curveSpeed(corner_kappa_max_, prof_), prof_);
    const double d_brake = brakeDistance(cmd_speed_, v_corner, prof_);
    if (corner_entry_s_ - s_travel <= d_brake) {
      v_target = std::min(v_target, v_corner);
    }
  }
  return v_target;
}

void SpeedControllerNode::onTimer()
{
  try {
    // 종료 시퀀스가 시작되면 executor 는 아직 살아 있다. 0 사이에 다른 값이
    // 끼어들지 않게 여기서 막는다.
    if (zeros_sent_.load()) {
      publishSpeed(0.0);
      return;
    }

    const rclcpp::Time now_t = now();

    // 첫 tick 은 /clock 이 아직 오지 않아 dt 가 0 이거나 음수일 수 있다.
    double dt = have_prev_tick_ ? (now_t - prev_tick_).seconds() : 0.0;
    if (dt <= 0.0) {dt = 1.0 / control_hz_;}
    prev_tick_ = now_t;
    have_prev_tick_ = true;

    // 경로 stamp -> 이번 tick. 그동안 진행한 호길이만큼 전방 구간을 민다.
    double dt_dr = 0.0;
    if (use_dead_reckoning_ && have_path_) {
      dt_dr = std::max(0.0, std::min((now_t - path_stamp_).seconds(), max_dead_reckon_s_));
    }
    const double s_travel = cmd_speed_ * dt_dr;

    updateLostTimer(now_t);
    const GateResult g = evaluateGates(now_t);

    double kappa_ahead = 0.0;
    double target = g.target;
    if (g.normal) {
      target = profileTarget(s_travel, &kappa_ahead);
    }

    if (g.hard_zero) {
      cmd_speed_ = 0.0;
    } else {
      // a_* 에서 유도한 tick 폭과 yaml 상한 중 작은 쪽.
      const double up = std::min(a_accel_ * dt, max_accel_step_);
      const double down = std::min(prof_.a_decel * dt, max_decel_step_);
      cmd_speed_ = rateLimit(cmd_speed_, target, up, down);
    }
    cmd_speed_ = std::max(0.0, std::min(cmd_speed_, prof_.v_max));

    publishSpeed(cmd_speed_);
    publishDebug(now_t, g, target, kappa_ahead, dt_dr);

    if (!g.normal) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), static_cast<int>(log_period_s_ * 1000.0),
        "[게이트][speed] %s -> %.2f m/s", g.reason, cmd_speed_);
    } else {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), static_cast<int>(log_period_s_ * 1000.0),
        "[제어][speed] v %.2f -> %.2f m/s, kappa %.3f 1/m, corner %.2f m, dr %.0f ms",
        cmd_speed_, target, kappa_ahead, corner_entry_s_, dt_dr * 1000.0);
    }
  } catch (const std::exception & e) {
    // 예외 tick 에서도 차량은 정지 쪽으로 간다.
    cmd_speed_ = 0.0;
    publishSpeed(0.0);
    RCLCPP_WARN(get_logger(), "[제어][speed] tick 예외: %s", e.what());
  }
}

void SpeedControllerNode::publishSpeed(double v)
{
  std_msgs::msg::Float64 m;
  m.data = v;
  speed_pub_->publish(m);
}

void SpeedControllerNode::publishDebug(
  const rclcpp::Time & now_t, const GateResult & g,
  double v_target, double kappa_ahead, double dt_dr)
{
  if (!publish_debug_ || !debug_pub_) {return;}

  backup_msgs::msg::ControlDebug d;
  d.header.stamp = now_t;
  d.header.frame_id = "base_link";
  d.path_source = path_source_;
  d.tracking_ok = g.normal;
  d.cmd_speed = cmd_speed_;
  d.v_target = v_target;
  d.kappa_ahead = kappa_ahead;
  d.kappa_saturated = kappa_saturated_;
  d.dead_reckon_dt = dt_dr;
  debug_pub_->publish(d);
}

void SpeedControllerNode::publishShutdownZeros()
{
  if (zeros_sent_.exchange(true)) {return;}
  if (!speed_pub_) {return;}

  std_msgs::msg::Float64 m;
  m.data = 0.0;
  for (int i = 0; i < shutdown_zero_count_; ++i) {
    speed_pub_->publish(m);
    // sim time 이 이미 멈춰 있을 수 있어 여기서만 벽시계 sleep 을 쓴다.
    std::this_thread::sleep_for(std::chrono::milliseconds(shutdown_zero_interval_ms_));
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(shutdown_linger_ms_));
  RCLCPP_INFO(get_logger(), "[종료][speed] /speed 0 %d 회 발행 완료", shutdown_zero_count_);
}

}  // namespace backup_speed_controller
