#include "simple_drive/control_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <thread>

namespace simple_drive
{

namespace
{
constexpr double kDeg2Rad = M_PI / 180.0;

rclcpp::QoS controlQos()
{
  return rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();
}

// 목표를 향해 step 이하로만 움직인다.
double rateLimit(double current, double target, double step)
{
  const double d = target - current;
  if (std::fabs(d) <= step) {return target;}
  return current + std::copysign(step, d);
}
}  // namespace


ControlNode::ControlNode()
: rclcpp::Node("simple_control")
{
  const auto steer_cmd_topic =
    declare_parameter<std::string>("steer_cmd_topic", "/simple/cmd_steer_deg");
  const auto speed_cmd_topic =
    declare_parameter<std::string>("speed_cmd_topic", "/simple/cmd_speed");
  const auto steer_topic =
    declare_parameter<std::string>("steer_topic", "/steering");
  const auto speed_topic =
    declare_parameter<std::string>("speed_topic", "/speed");

  control_hz_ = declare_parameter<double>("control_hz", 50.0);
  cmd_timeout_s_ = declare_parameter<double>("cmd_timeout_s", 0.3);
  max_steer_deg_ = declare_parameter<double>("max_steer_deg", 20.0);
  max_steer_rate_deg_s_ = declare_parameter<double>("max_steer_rate_deg_s", 600.0);
  v_max_ = declare_parameter<double>("v_max", 0.60);
  a_accel_ = declare_parameter<double>("a_accel", 1.5);
  a_decel_ = declare_parameter<double>("a_decel", 1.5);
  shutdown_zero_count_ = static_cast<int>(declare_parameter<int>("shutdown_zero_count", 5));
  shutdown_zero_interval_ms_ =
    static_cast<int>(declare_parameter<int>("shutdown_zero_interval_ms", 20));

  if (control_hz_ <= 0.0) {throw std::invalid_argument("control_hz must be > 0");}
  if (max_steer_deg_ <= 0.0) {throw std::invalid_argument("max_steer_deg must be > 0");}
  if (max_steer_rate_deg_s_ <= 0.0) {
    throw std::invalid_argument("max_steer_rate_deg_s must be > 0");
  }
  if (v_max_ <= 0.0) {throw std::invalid_argument("v_max must be > 0");}
  if (a_accel_ <= 0.0 || a_decel_ <= 0.0) {
    throw std::invalid_argument("a_accel / a_decel must be > 0");
  }

  steer_cmd_sub_ = create_subscription<std_msgs::msg::Float64>(
    steer_cmd_topic, controlQos(),
    std::bind(&ControlNode::onSteerCmd, this, std::placeholders::_1));
  speed_cmd_sub_ = create_subscription<std_msgs::msg::Float64>(
    speed_cmd_topic, controlQos(),
    std::bind(&ControlNode::onSpeedCmd, this, std::placeholders::_1));

  steer_pub_ = create_publisher<std_msgs::msg::Float64>(steer_topic, controlQos());
  speed_pub_ = create_publisher<std_msgs::msg::Float64>(speed_topic, controlQos());

  const rclcpp::Time zero(0, 0, get_clock()->get_clock_type());
  steer_stamp_ = zero;
  speed_stamp_ = zero;
  last_tick_ = now();

  timer_ = rclcpp::create_timer(
    this, get_clock(), rclcpp::Duration::from_seconds(1.0 / control_hz_),
    std::bind(&ControlNode::onTimer, this));

  // 컨텍스트가 아직 살아 있는 시점이어야 0 이 실제로 나간다.
  auto ctx = get_node_base_interface()->get_context();
  ctx->add_pre_shutdown_callback([this]() {publishShutdownZeros();});

  RCLCPP_INFO(
    get_logger(),
    "[기동][control] %s / %s -> %s (rad) / %s (m/s) | %.0f Hz "
    "조향 한계 %.1f deg 속도제한 %.0f deg/s v_max %.2f m/s 워치독 %.2f s",
    steer_cmd_topic.c_str(), speed_cmd_topic.c_str(),
    steer_topic.c_str(), speed_topic.c_str(),
    control_hz_, max_steer_deg_, max_steer_rate_deg_s_, v_max_, cmd_timeout_s_);
}


void ControlNode::onSteerCmd(const std_msgs::msg::Float64::SharedPtr msg)
{
  // NaN 은 0(직진)으로 떨군다. 클램프에 그대로 넣으면 std::min/max 의
  // NaN 비교가 전부 false 라 부호 없는 +limit 이 나온다
  // (backup_common::clampAbs 주석의 실제 사고 사례).
  cmd_steer_deg_ = std::isfinite(msg->data) ? msg->data : 0.0;
  steer_stamp_ = now();
}


void ControlNode::onSpeedCmd(const std_msgs::msg::Float64::SharedPtr msg)
{
  cmd_speed_ = std::isfinite(msg->data) ? msg->data : 0.0;
  speed_stamp_ = now();
}


void ControlNode::onTimer()
{
  const rclcpp::Time t = now();
  double dt = (t - last_tick_).seconds();
  last_tick_ = t;
  if (!std::isfinite(dt) || dt <= 0.0 || dt > 5.0 / control_hz_) {
    dt = 1.0 / control_hz_;
  }

  const bool fresh =
    (t - steer_stamp_).seconds() < cmd_timeout_s_ &&
    (t - speed_stamp_).seconds() < cmd_timeout_s_;

  if (fresh == timed_out_) {
    // 상태가 바뀐 순간만 로그. 50 Hz 로 도배하지 않는다.
    if (!fresh) {
      RCLCPP_WARN(get_logger(), "[제어] 판단 지시 끊김 %.2f s -- 정지", cmd_timeout_s_);
    } else {
      RCLCPP_INFO(get_logger(), "[제어] 판단 지시 복구");
    }
    timed_out_ = !fresh;
  }

  const double target_steer = fresh ?
    std::max(-max_steer_deg_, std::min(max_steer_deg_, cmd_steer_deg_)) : 0.0;
  const double target_speed = fresh ?
    std::max(0.0, std::min(v_max_, cmd_speed_)) : 0.0;

  out_steer_deg_ = rateLimit(out_steer_deg_, target_steer, max_steer_rate_deg_s_ * dt);

  const double a = (target_speed >= out_speed_) ? a_accel_ : a_decel_;
  out_speed_ = rateLimit(out_speed_, target_speed, a * dt);

  std_msgs::msg::Float64 s;
  s.data = out_steer_deg_ * kDeg2Rad;
  steer_pub_->publish(s);

  std_msgs::msg::Float64 v;
  v.data = out_speed_;
  speed_pub_->publish(v);
}


void ControlNode::publishShutdownZeros()
{
  if (zeros_sent_.exchange(true)) {return;}
  if (!steer_pub_ || !speed_pub_) {return;}

  std_msgs::msg::Float64 m;
  m.data = 0.0;
  for (int i = 0; i < shutdown_zero_count_; ++i) {
    steer_pub_->publish(m);
    speed_pub_->publish(m);
    // sim time 이 이미 멈춰 있을 수 있어 여기서만 벽시계 sleep 을 쓴다.
    std::this_thread::sleep_for(std::chrono::milliseconds(shutdown_zero_interval_ms_));
  }
  RCLCPP_INFO(
    get_logger(), "[종료][control] /steering, /speed 0 %d 회 발행 완료",
    shutdown_zero_count_);
}

}  // namespace simple_drive
