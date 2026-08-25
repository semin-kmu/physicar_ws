// backup_steer_controller -- 50 Hz. Pure Pursuit 단일 모드.
//
// 모드 전환도 delta PID 도 없다 -- 기하는 전부 플래너가 처리했다.
// 출력 /steering 은 rad (기존 kau_control 과 동일).

#ifndef BACKUP_STEER_CONTROLLER__STEER_CONTROLLER_NODE_HPP_
#define BACKUP_STEER_CONTROLLER__STEER_CONTROLLER_NODE_HPP_

#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

#include <backup_common/bezier.hpp>
#include <backup_msgs/msg/backup_path.hpp>
#include <backup_msgs/msg/control_debug.hpp>

namespace backup_steer_controller
{

class SteerControllerNode : public rclcpp::Node
{
public:
  SteerControllerNode();

private:
  // 한 tick 의 산출. 디버그 발행이 같은 값을 다시 계산하지 않게 모아 둔다.
  struct Tick
  {
    double delta_target = 0.0;    // [rad] 슬루 제한 전
    double lookahead_m = 0.0;     // [m] 공식 분모로 실제 쓴 현(직선거리)
    double alpha_rad = 0.0;       // 목표점 방위. 좌측 +
    double dead_reckon_dt = 0.0;
    bool saturated = false;
    bool tracking_ok = false;
    const char * reason = "";
  };

  void onPath(const backup_msgs::msg::BackupPath::SharedPtr msg);
  void onSpeed(const std_msgs::msg::Float64::SharedPtr msg);
  void onTimer();

  Tick computeTick(const rclcpp::Time & now);
  double lostBothSteer() const;
  void publishSteer(double rad);
  void publishDebug(const rclcpp::Time & now, const Tick & t);
  void logStartup() const;

  // --- 파라미터 ---
  std::string path_topic_, speed_topic_, output_topic_, debug_topic_;
  bool publish_debug_ = true;
  double control_hz_ = 50.0;
  double path_timeout_s_ = 0.4;

  double wheelbase_m_ = 0.18;
  double max_steer_rad_ = 0.0;
  double max_steer_rate_rad_s_ = 0.0;

  double lookahead_k_v_ = 0.85;
  double lookahead_min_m_ = 0.80;
  double lookahead_max_m_ = 1.20;
  bool clamp_to_valid_length_ = true;
  double valid_length_margin_m_ = 0.10;

  bool use_dead_reckoning_ = true;
  double max_dead_reckon_s_ = 0.30;
  int arclength_lut_samples_ = 64;

  bool saturation_report_ = true;
  double log_period_s_ = 2.0;

  // --- 경로 상태 ---
  backup_common::Path path_;
  bool have_path_ = false;
  rclcpp::Time path_rx_;
  rclcpp::Time path_stamp_;
  double valid_length_ = 0.0;
  bool kappa_saturated_ = false;
  uint8_t lost_state_ = 0;
  uint8_t lost_first_ = 0;
  uint8_t path_source_ = 0;

  // --- tick 상태 ---
  double v_ = 0.0;                // [m/s] /speed 명령값. 실측이 아니다
  double delta_cmd_ = 0.0;        // [rad] 마지막으로 발행한 값
  double last_kappa_sign_ = 0.0;  // LOST_BOTH 에서 lost_first 가 없을 때의 대안
  bool have_prev_tick_ = false;
  rclcpp::Time prev_tick_;

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr steer_pub_;
  rclcpp::Publisher<backup_msgs::msg::ControlDebug>::SharedPtr debug_pub_;
  rclcpp::Subscription<backup_msgs::msg::BackupPath>::SharedPtr path_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr speed_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace backup_steer_controller

#endif  // BACKUP_STEER_CONTROLLER__STEER_CONTROLLER_NODE_HPP_
