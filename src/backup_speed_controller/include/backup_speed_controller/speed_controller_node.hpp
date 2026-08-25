// backup_speed_controller -- 50 Hz. /backup/path 의 곡률에서 /speed 를 만든다.
//
// 게이트는 우선순위 순으로 평가하고 먼저 걸리는 것이 이긴다.
// 확신이 없으면 0 이다 (permission 미수신도 0).

#ifndef BACKUP_SPEED_CONTROLLER__SPEED_CONTROLLER_NODE_HPP_
#define BACKUP_SPEED_CONTROLLER__SPEED_CONTROLLER_NODE_HPP_

#include <atomic>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>

#include <backup_common/bezier.hpp>
#include <backup_msgs/msg/backup_path.hpp>
#include <backup_msgs/msg/control_debug.hpp>

#include "backup_speed_controller/profile.hpp"

namespace backup_speed_controller
{

class SpeedControllerNode : public rclcpp::Node
{
public:
  SpeedControllerNode();

  // 종료 시 0 을 반복 발행. 컨텍스트가 아직 유효한 pre-shutdown 에서 부른다.
  // 여기서만 std::this_thread::sleep_for 를 쓴다 -- sim time 이 이미 멈춰 있다.
  void publishShutdownZeros();

private:
  // 게이트 판정 결과. hard 는 레이트 리밋을 무시하고 즉시 0.
  struct GateResult
  {
    double target = 0.0;
    bool hard_zero = false;
    bool normal = false;
    const char * reason = "";
  };

  void onPath(const backup_msgs::msg::BackupPath::SharedPtr msg);
  void onPermission(const std_msgs::msg::Bool::SharedPtr msg);
  void onTimer();

  void updateLostTimer(const rclcpp::Time & now);
  GateResult evaluateGates(const rclcpp::Time & now);
  double profileTarget(double s_travel, double * kappa_ahead_out);
  void publishSpeed(double v);
  void publishDebug(
    const rclcpp::Time & now, const GateResult & g,
    double v_target, double kappa_ahead, double dt_dr);
  void logStartup() const;

  // --- 파라미터 ---
  std::string path_topic_, permission_topic_, output_topic_, debug_topic_;
  bool publish_debug_ = true;
  double control_hz_ = 50.0;
  double path_timeout_s_ = 0.4;

  ProfileParams prof_;
  double a_accel_ = 1.5;
  double max_accel_step_ = 0.030;
  double max_decel_step_ = 0.010;
  double preview_m_ = 1.60;

  bool require_start_permission_ = true;
  double lost_hold_s_ = 0.6;

  bool use_dead_reckoning_ = true;
  double max_dead_reckon_s_ = 0.30;

  int shutdown_zero_count_ = 3;
  int shutdown_zero_interval_ms_ = 20;
  int shutdown_linger_ms_ = 200;
  double log_period_s_ = 2.0;

  // --- 경로 상태 ---
  backup_common::Path path_;
  bool have_path_ = false;
  rclcpp::Time path_rx_;
  rclcpp::Time path_stamp_;
  double valid_length_ = 0.0;
  double corner_entry_s_ = -1.0;
  double corner_kappa_max_ = 0.0;
  bool kappa_saturated_ = false;
  bool stop_request_ = false;
  uint8_t lost_state_ = 0;
  uint8_t path_source_ = 0;

  // --- 게이트 상태 ---
  bool permission_ = false;
  bool permission_seen_ = false;
  bool lost_active_ = false;
  rclcpp::Time lost_since_;

  // --- tick 상태 ---
  double cmd_speed_ = 0.0;
  bool have_prev_tick_ = false;
  rclcpp::Time prev_tick_;

  std::atomic<bool> zeros_sent_{false};

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr speed_pub_;
  rclcpp::Publisher<backup_msgs::msg::ControlDebug>::SharedPtr debug_pub_;
  rclcpp::Subscription<backup_msgs::msg::BackupPath>::SharedPtr path_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr permission_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace backup_speed_controller

#endif  // BACKUP_SPEED_CONTROLLER__SPEED_CONTROLLER_NODE_HPP_
