// 경로 생성 단독 소유. lane 콜백 구동 -- 차선이 끊기면 경로도 끊겨
// 하류 timeout 게이트가 작동한다. 낡은 경로를 재발행하지 않는다.

#ifndef BACKUP_PATH_PLANNER__PATH_PLANNER_NODE_HPP_
#define BACKUP_PATH_PLANNER__PATH_PLANNER_NODE_HPP_

#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

#include <backup_common/bezier.hpp>
#include <backup_common/motion.hpp>
#include <backup_common/vehicle.hpp>

#include "backup_msgs/msg/backup_path.hpp"
#include "backup_msgs/msg/lane_geometry.hpp"
#include "backup_msgs/msg/obstacle_circle_array.hpp"

#include "backup_path_planner/avoidance.hpp"
#include "backup_path_planner/race_line.hpp"

namespace backup_path_planner
{

// 생성 파라미터용 EMA. 제어점이 아니라 파라미터에 건다 (제어점 필터는 형상을 깬다).
class Ema
{
public:
  double update(double v, double alpha)
  {
    if (!init_) {val_ = v; init_ = true;} else {val_ += alpha * (v - val_);}
    return val_;
  }

  double updateAngle(double v, double alpha)
  {
    if (!init_) {
      val_ = v;
      init_ = true;
    } else {
      val_ = backup_common::wrapAngle(val_ + alpha * backup_common::wrapAngle(v - val_));
    }
    return val_;
  }

  void reset() {init_ = false; val_ = 0.0;}
  double value() const {return val_;}

private:
  bool init_ = false;
  double val_ = 0.0;
};

struct PlannerParams
{
  std::string lane_topic, obstacle_topic, speed_topic, steer_topic, output_topic;
  double timer_hz = 0.0;
  double lane_timeout_s = 0.4;

  double wheelbase_m = 0.18;
  double vehicle_width_m = 0.205;
  double vehicle_length_m = 0.280;
  double rear_axle_to_front_bumper_m = 0.230;
  double max_steer_deg = 20.0;

  double track_width_m = 0.700;
  double lane_margin_m = 0.05;

  bool use_dead_reckoning = true;
  double max_dead_reckon_s = 0.30;

  bool enable_race_line = true;
  double d_sight_m = 1.60;
  double t_lat_s = 0.10;
  double race_line_r_max_m = 5.0;
  int width_iterations = 2;

  double obstacle_radius_m = 0.142;
  double avoid_clearance_m = 0.10;
  double avoid_trigger_m = 1.40;
  double avoid_lateral_gate_m = 0.30;
  double avoid_offset_rate_m_per_s = 0.60;

  int degree = 5;
  bool anchor_at_vehicle = true;
  double tangent_epsilon_m = 0.05;
  bool zero_end_curvature = true;

  double param_ema_alpha = 0.5;
  double source_min_hold_s = 0.3;

  bool log_kappa_saturation = true;
  double log_period_s = 2.0;
};

// 데드레커닝까지 끝난 차선 기하. 전부 현재 base_link 기준.
struct LaneWork
{
  Point2 foot{};              // 자차 원점을 중앙선에 내린 발
  Point2 u_c{1.0, 0.0};       // 중앙선 진행 방향
  bool center_valid = false;

  Point2 corner{};
  bool inflection_valid = false;
  double delta_psi = 0.0;
  bool second_inflection = false;

  double valid_length = 0.0;
  uint8_t lost_state = 0;
  uint8_t lost_first = 0;
  float confidence = 0.0f;
};

class PathPlannerNode : public rclcpp::Node
{
public:
  explicit PathPlannerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  using LaneGeometry = backup_msgs::msg::LaneGeometry;
  using BackupPath = backup_msgs::msg::BackupPath;
  using ObstacleCircleArray = backup_msgs::msg::ObstacleCircleArray;
  using Float64 = std_msgs::msg::Float64;

  void declareParams();
  void logStartup();

  void onLane(const LaneGeometry::ConstSharedPtr msg);
  void onObstacles(const ObstacleCircleArray::ConstSharedPtr msg);
  void onTimer();

  void plan(const LaneGeometry & lane);

  bool buildLaneWork(const LaneGeometry & lane, double dr_dt, LaneWork & w) const;
  uint8_t holdSource(uint8_t desired, const rclcpp::Time & now, const LaneWork & w);
  std::vector<AvoidTarget> freshObstacles(const rclcpp::Time & now) const;
  void logSaturation(bool sat, double s_pos, double required_steer, const rclcpp::Time & now);

  PlannerParams p_;
  backup_common::VehicleParams veh_;
  RaceLineConfig race_cfg_;
  AvoidConfig avoid_cfg_;

  rclcpp::Subscription<LaneGeometry>::SharedPtr lane_sub_;
  rclcpp::Subscription<ObstacleCircleArray>::SharedPtr obs_sub_;
  rclcpp::Subscription<Float64>::SharedPtr speed_sub_;
  rclcpp::Subscription<Float64>::SharedPtr steer_sub_;
  rclcpp::Publisher<BackupPath>::SharedPtr path_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  double v_ = 0.0;            // /speed [m/s]
  double steer_ = 0.0;        // /steering [rad]
  bool have_speed_ = false;

  ObstacleCircleArray obstacles_;
  bool have_obstacles_ = false;

  LaneGeometry last_lane_;
  bool have_lane_ = false;

  Ema ema_lat_, ema_psi_, ema_corner_n_, ema_dpsi_, ema_radius_;

  uint8_t base_source_ = backup_msgs::msg::BackupPath::SRC_LANE_CENTER;
  rclcpp::Time source_since_;
  rclcpp::Time last_plan_;
  bool have_last_plan_ = false;

  double avoid_offset_ = 0.0;

  bool sat_active_ = false;
  rclcpp::Time sat_start_;
  double sat_max_steer_ = 0.0;
  double sat_s_ = 0.0;
};

}  // namespace backup_path_planner

#endif  // BACKUP_PATH_PLANNER__PATH_PLANNER_NODE_HPP_
