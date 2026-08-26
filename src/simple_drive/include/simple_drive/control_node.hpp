// ====================================================================
// simple_drive / control_node
//
// 제어. 판단이 내린 지시를 실제 액추에이터 명령으로 바꾼다.
// 여기에는 어떤 주행 판단도 없다 -- 단위변환 / 레이트 리밋 /
// 클램프 / 워치독 / 종료 0 발행뿐이다.
//
// 입력:
//   /simple/cmd_steer_deg  Float64  [deg] 좌측 +
//   /simple/cmd_speed      Float64  [m/s]
//
// 출력 (플랫폼 계약. 기존 backup_*_controller 와 같은 토픽/단위):
//   /steering  Float64  [rad] 좌측 +
//   /speed     Float64  [m/s]
//
// ★ backup_steer_controller / backup_speed_controller 와 절대 동시에
//   띄우지 않는다. 두 publisher 값이 섞여 차량에 간다.
// ====================================================================

#ifndef SIMPLE_DRIVE__CONTROL_NODE_HPP_
#define SIMPLE_DRIVE__CONTROL_NODE_HPP_

#include <atomic>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

namespace simple_drive
{

class ControlNode : public rclcpp::Node
{
public:
  ControlNode();

private:
  void onSteerCmd(const std_msgs::msg::Float64::SharedPtr msg);
  void onSpeedCmd(const std_msgs::msg::Float64::SharedPtr msg);
  void onTimer();
  void publishShutdownZeros();

  // --- 파라미터 ---
  double control_hz_ = 50.0;
  double cmd_timeout_s_ = 0.3;
  double max_steer_deg_ = 20.0;
  // 하드웨어 조향속도는 600 deg/s. 밴뱅은 전타 전환이 빨라야 의미가
  // 있으므로 기존 Pure Pursuit(300)보다 높게 잡는다.
  double max_steer_rate_deg_s_ = 600.0;
  double v_max_ = 0.60;
  double a_accel_ = 1.5;
  double a_decel_ = 1.5;
  int shutdown_zero_count_ = 5;
  int shutdown_zero_interval_ms_ = 20;

  // --- 상태 ---
  double cmd_steer_deg_ = 0.0;
  double cmd_speed_ = 0.0;
  rclcpp::Time steer_stamp_;
  rclcpp::Time speed_stamp_;
  double out_steer_deg_ = 0.0;
  double out_speed_ = 0.0;
  rclcpp::Time last_tick_;
  bool timed_out_ = true;
  std::atomic<bool> zeros_sent_{false};

  // --- 배선 ---
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr steer_cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr speed_cmd_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr steer_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr speed_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace simple_drive

#endif  // SIMPLE_DRIVE__CONTROL_NODE_HPP_
