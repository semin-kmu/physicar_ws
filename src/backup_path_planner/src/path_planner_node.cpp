#include "backup_path_planner/path_planner_node.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <string>
#include <vector>

namespace backup_path_planner
{

using backup_common::kRad2Deg;

namespace
{

float laneConfidence(const backup_msgs::msg::LaneGeometry & lane)
{
  if (lane.center_valid) {return lane.center.confidence;}
  int n = 0;
  float s = 0.0f;
  if (lane.left_valid) {s += lane.left.confidence; ++n;}
  if (lane.right_valid) {s += lane.right.confidence; ++n;}
  return (n > 0) ? s / static_cast<float>(n) : 0.0f;
}

}  // namespace

PathPlannerNode::PathPlannerNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("backup_path_planner", options)
{
  declareParams();

  veh_.wheelbase = p_.wheelbase_m;
  veh_.width = p_.vehicle_width_m;
  veh_.length = p_.vehicle_length_m;
  veh_.rear_axle_to_front_bumper = p_.rear_axle_to_front_bumper_m;
  veh_.max_steer_deg = p_.max_steer_deg;

  race_cfg_.d_sight_m = p_.d_sight_m;
  race_cfg_.t_lat_s = p_.t_lat_s;
  race_cfg_.r_max_m = p_.race_line_r_max_m;
  race_cfg_.track_width_m = p_.track_width_m;
  race_cfg_.lane_margin_m = p_.lane_margin_m;
  race_cfg_.width_iterations = p_.width_iterations;

  avoid_cfg_.obstacle_radius_m = p_.obstacle_radius_m;
  avoid_cfg_.clearance_m = p_.avoid_clearance_m;
  avoid_cfg_.trigger_m = p_.avoid_trigger_m;
  avoid_cfg_.lateral_gate_m = p_.avoid_lateral_gate_m;
  avoid_cfg_.vehicle_width_m = p_.vehicle_width_m;
  avoid_cfg_.track_width_m = p_.track_width_m;
  avoid_cfg_.lane_margin_m = p_.lane_margin_m;

  // 기본 생성 rclcpp::Time 은 SYSTEM_TIME 이라 now() 와 빼면 던진다.
  source_since_ = this->now();
  last_plan_ = this->now();
  sat_start_ = this->now();

  const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();

  path_pub_ = create_publisher<BackupPath>(p_.output_topic, qos);

  lane_sub_ = create_subscription<LaneGeometry>(
    p_.lane_topic, qos, std::bind(&PathPlannerNode::onLane, this, std::placeholders::_1));
  obs_sub_ = create_subscription<ObstacleCircleArray>(
    p_.obstacle_topic, qos,
    std::bind(&PathPlannerNode::onObstacles, this, std::placeholders::_1));
  speed_sub_ = create_subscription<Float64>(
    p_.speed_topic, qos,
    [this](const Float64::ConstSharedPtr m) {v_ = m->data; have_speed_ = true;});
  steer_sub_ = create_subscription<Float64>(
    p_.steer_topic, qos, [this](const Float64::ConstSharedPtr m) {steer_ = m->data;});

  if (p_.timer_hz > 0.0) {
    timer_ = rclcpp::create_timer(
      this, this->get_clock(), rclcpp::Duration::from_seconds(1.0 / p_.timer_hz),
      std::bind(&PathPlannerNode::onTimer, this));
  }

  logStartup();
}

void PathPlannerNode::declareParams()
{
  p_.lane_topic = declare_parameter<std::string>("lane_topic", "/backup/lane/geometry");
  p_.obstacle_topic =
    declare_parameter<std::string>("obstacle_topic", "/backup/perception/obstacles");
  p_.speed_topic = declare_parameter<std::string>("speed_topic", "/speed");
  p_.steer_topic = declare_parameter<std::string>("steer_topic", "/steering");
  p_.output_topic = declare_parameter<std::string>("output_topic", "/backup/path");

  p_.timer_hz = declare_parameter<double>("timer_hz", 0.0);
  p_.lane_timeout_s = declare_parameter<double>("lane_timeout_s", 0.4);

  p_.wheelbase_m = declare_parameter<double>("wheelbase_m", 0.18);
  p_.vehicle_width_m = declare_parameter<double>("vehicle_width_m", 0.205);
  p_.vehicle_length_m = declare_parameter<double>("vehicle_length_m", 0.280);
  p_.rear_axle_to_front_bumper_m =
    declare_parameter<double>("rear_axle_to_front_bumper_m", 0.230);
  p_.max_steer_deg = declare_parameter<double>("max_steer_deg", 20.0);

  p_.track_width_m = declare_parameter<double>("track_width_m", 0.700);
  p_.lane_margin_m = declare_parameter<double>("lane_margin_m", 0.05);

  p_.use_dead_reckoning = declare_parameter<bool>("use_dead_reckoning", true);
  p_.max_dead_reckon_s = declare_parameter<double>("max_dead_reckon_s", 0.30);

  p_.enable_race_line = declare_parameter<bool>("enable_race_line", true);
  p_.d_sight_m = declare_parameter<double>("d_sight_m", 1.60);
  p_.t_lat_s = declare_parameter<double>("t_lat_s", 0.10);
  p_.race_line_r_max_m = declare_parameter<double>("race_line_r_max_m", 5.0);
  p_.width_iterations = static_cast<int>(declare_parameter<int64_t>("width_iterations", 2));

  p_.obstacle_radius_m = declare_parameter<double>("obstacle_radius_m", 0.142);
  p_.avoid_clearance_m = declare_parameter<double>("avoid_clearance_m", 0.10);
  p_.avoid_trigger_m = declare_parameter<double>("avoid_trigger_m", 1.40);
  p_.avoid_lateral_gate_m = declare_parameter<double>("avoid_lateral_gate_m", 0.30);
  p_.avoid_offset_rate_m_per_s = declare_parameter<double>("avoid_offset_rate_m_per_s", 0.60);

  p_.degree = static_cast<int>(declare_parameter<int64_t>("degree", 5));
  p_.anchor_at_vehicle = declare_parameter<bool>("anchor_at_vehicle", true);
  p_.tangent_epsilon_m = declare_parameter<double>("tangent_epsilon_m", 0.05);
  p_.zero_end_curvature = declare_parameter<bool>("zero_end_curvature", true);

  p_.param_ema_alpha = declare_parameter<double>("param_ema_alpha", 0.5);
  p_.source_min_hold_s = declare_parameter<double>("source_min_hold_s", 0.3);

  p_.log_kappa_saturation = declare_parameter<bool>("log_kappa_saturation", true);
  p_.log_period_s = declare_parameter<double>("log_period_s", 2.0);

  if (p_.degree != backup_common::kDegree) {
    RCLCPP_WARN(
      get_logger(), "[기동][plan] degree %d 는 미지원. quintic(5) 로 강제한다", p_.degree);
    p_.degree = backup_common::kDegree;
  }
  p_.param_ema_alpha = std::min(std::max(p_.param_ema_alpha, 0.0), 1.0);
}

void PathPlannerNode::logStartup()
{
  RCLCPP_INFO(
    get_logger(), "[기동][plan] 구동 %s, %s + %s -> %s",
    (p_.timer_hz > 0.0) ? "타이머" : "lane 콜백",
    p_.lane_topic.c_str(), p_.obstacle_topic.c_str(), p_.output_topic.c_str());
  RCLCPP_INFO(
    get_logger(),
    "[기동][plan] 트랙 %.3f m, 차선여유 %.3f m, d_sight %.2f m, R_min %.4f m, kappa_max %.4f 1/m",
    p_.track_width_m, p_.lane_margin_m, p_.d_sight_m, veh_.rMin(), veh_.kappaMax());
  RCLCPP_INFO(
    get_logger(),
    "[기동][plan] DR %s(<=%.2f s), race_line %s, EMA a=%.2f, hold %.2f s, anchor %s",
    p_.use_dead_reckoning ? "on" : "off", p_.max_dead_reckon_s,
    p_.enable_race_line ? "on" : "off", p_.param_ema_alpha, p_.source_min_hold_s,
    p_.anchor_at_vehicle ? "veh" : "lane");
}

void PathPlannerNode::onObstacles(const ObstacleCircleArray::ConstSharedPtr msg)
{
  obstacles_ = *msg;
  have_obstacles_ = true;
}

void PathPlannerNode::onLane(const LaneGeometry::ConstSharedPtr msg)
{
  last_lane_ = *msg;
  have_lane_ = true;
  if (p_.timer_hz > 0.0) {return;}
  try {
    plan(*msg);
  } catch (const std::exception & e) {
    RCLCPP_WARN(get_logger(), "[계획][plan] 콜백 예외: %s", e.what());
  }
}

void PathPlannerNode::onTimer()
{
  if (!have_lane_) {return;}
  try {
    plan(last_lane_);
  } catch (const std::exception & e) {
    RCLCPP_WARN(get_logger(), "[계획][plan] 타이머 예외: %s", e.what());
  }
}

std::vector<AvoidTarget> PathPlannerNode::freshObstacles(const rclcpp::Time & now) const
{
  std::vector<AvoidTarget> out;
  if (!have_obstacles_) {return out;}
  if (obstacles_.status != ObstacleCircleArray::STATUS_OK) {return out;}
  const rclcpp::Time stamp(obstacles_.header.stamp, now.get_clock_type());
  const double age = (now - stamp).seconds();
  if (age < 0.0 || age > p_.lane_timeout_s) {return out;}
  out.reserve(obstacles_.obstacles.size());
  for (const auto & o : obstacles_.obstacles) {
    out.push_back({static_cast<double>(o.center_x), static_cast<double>(o.center_y)});
  }
  return out;
}

bool PathPlannerNode::buildLaneWork(
  const LaneGeometry & lane, double dr_dt, LaneWork & w) const
{
  backup_common::Pose2 motion;
  if (p_.use_dead_reckoning && dr_dt > 0.0) {
    motion = backup_common::integrateBicycle(v_, steer_, p_.wheelbase_m, dr_dt);
  }

  double cx0 = lane.center.x0, cy0 = lane.center.y0;
  double cx1 = lane.center.x1, cy1 = lane.center.y1;
  bool center_valid = lane.center_valid;
  if (!center_valid && lane.left_valid && lane.right_valid) {
    cx0 = 0.5 * (lane.left.x0 + lane.right.x0);
    cy0 = 0.5 * (lane.left.y0 + lane.right.y0);
    cx1 = 0.5 * (lane.left.x1 + lane.right.x1);
    cy1 = 0.5 * (lane.left.y1 + lane.right.y1);
    center_valid = true;
  }

  w.valid_length = lane.valid_length;
  w.lost_state = lane.lost_state;
  w.lost_first = lane.lost_first;
  w.confidence = laneConfidence(lane);
  w.center_valid = center_valid;
  if (!center_valid) {return false;}

  backup_common::transformPoint(motion, cx0, cy0);
  backup_common::transformPoint(motion, cx1, cy1);

  Point2 u = normalized(Point2{cx1 - cx0, cy1 - cy0});
  if (u.x < 0.0) {u = Point2{-u.x, -u.y};}  // 항상 전방을 향한다
  w.u_c = u;
  w.foot = Point2{cx0, cy0} + u * (-(cx0 * u.x + cy0 * u.y));

  if (lane.inflection_valid) {
    double ix = lane.inflection_x, iy = lane.inflection_y;
    backup_common::transformPoint(motion, ix, iy);
    w.corner = Point2{ix, iy};
    // 꼭짓점이 뒤에 있으면 코너를 이미 지난 것이다.
    w.inflection_valid = ix > 0.0;
  }
  w.delta_psi = lane.delta_psi;
  w.second_inflection = lane.second_inflection;
  return true;
}

uint8_t PathPlannerNode::holdSource(
  uint8_t desired, const rclcpp::Time & now, const LaneWork & w)
{
  if (desired == base_source_) {return base_source_;}
  const bool can_hold = (base_source_ == BackupPath::SRC_RACE_LINE)
    ? w.inflection_valid : w.center_valid;
  if (can_hold && (now - source_since_).seconds() < p_.source_min_hold_s) {
    return base_source_;
  }
  base_source_ = desired;
  source_since_ = now;
  return base_source_;
}

void PathPlannerNode::logSaturation(
  bool sat, double s_pos, double required_steer, const rclcpp::Time & now)
{
  if (!p_.log_kappa_saturation) {return;}
  const int period_ms = static_cast<int>(p_.log_period_s * 1000.0);

  if (sat && !sat_active_) {
    sat_active_ = true;
    sat_start_ = now;
    sat_s_ = s_pos;
    sat_max_steer_ = required_steer;
    // 상태 전이는 THROTTLE 로 묻지 않는다. 통과 가능 여부의 근거 로그다.
    RCLCPP_WARN(
      get_logger(), "[포화][plan] kappa 포화 진입 s=%.2f m 요구 delta=%.1f deg (한계 %.1f)",
      s_pos, required_steer * kRad2Deg, p_.max_steer_deg);
  } else if (sat) {
    sat_max_steer_ = std::max(sat_max_steer_, required_steer);
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), period_ms,
      "[포화][plan] kappa 포화 지속 %.2f s 최대 요구 delta=%.1f deg",
      (now - sat_start_).seconds(), sat_max_steer_ * kRad2Deg);
  } else if (sat_active_) {
    sat_active_ = false;
    RCLCPP_WARN(
      get_logger(),
      "[포화][plan] kappa 포화 해제 지속 %.2f s 진입 s=%.2f m 최대 요구 delta=%.1f deg",
      (now - sat_start_).seconds(), sat_s_, sat_max_steer_ * kRad2Deg);
  }
}

void PathPlannerNode::plan(const LaneGeometry & lane)
{
  const rclcpp::Time now = this->now();
  const int period_ms = static_cast<int>(p_.log_period_s * 1000.0);

  // use_sim_time 직후에는 /clock 이 오기 전까지 now() 가 0 이다.
  if (now.nanoseconds() == 0) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), period_ms, "[기동][plan] /clock 대기");
    return;
  }

  const rclcpp::Time stamp(lane.header.stamp, now.get_clock_type());
  double dr_dt = (now - stamp).seconds();
  if (dr_dt < 0.0) {dr_dt = 0.0;}
  if (p_.use_dead_reckoning && dr_dt > p_.max_dead_reckon_s) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), period_ms,
      "[계획][plan] 차선 기하 %.0f ms 지연 (>%.0f), 폐기", dr_dt * 1e3,
      p_.max_dead_reckon_s * 1e3);
    return;
  }

  double dt_pub = have_last_plan_ ? (now - last_plan_).seconds() : 0.0;
  if (dt_pub < 0.0 || dt_pub > p_.lane_timeout_s) {dt_pub = 0.0;}  // 첫 tick / 공백 복귀는 버린다
  last_plan_ = now;
  have_last_plan_ = true;

  LaneWork w;
  const bool ok = buildLaneWork(lane, dr_dt, w);
  const double alpha = p_.param_ema_alpha;

  Segment seg;
  uint8_t source = BackupPath::SRC_LANE_CENTER;
  bool sat_geo = false;
  double required_steer = 0.0;
  double corner_entry_s = -1.0;
  float corner_kappa_max = 0.0f;
  bool stop_request = false;

  const double path_len = std::max(w.valid_length, 5.0 * p_.tangent_epsilon_m);

  if (!ok) {
    // 중앙선 근거가 없다. 진행방향 직선만 낸다 -- lost_state 는 그대로 승계된다.
    ema_lat_.reset();
    ema_psi_.reset();
    ema_corner_n_.reset();
    ema_dpsi_.reset();
    ema_radius_.reset();
    w.confidence = 0.0f;
    seg = centerLineSegment({0.0, 0.0}, {1.0, 0.0}, path_len, p_.tangent_epsilon_m, true);
    base_source_ = BackupPath::SRC_LANE_CENTER;
    source_since_ = now;
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), period_ms, "[계획][plan] 중앙선 무효, 직진 경로");
  } else {
    // 생성 파라미터에 EMA. 중앙선 횡오프셋과 헤딩이 직선 모드의 생성 파라미터다.
    const Point2 n_raw = leftNormal(w.u_c);
    const double lat_raw = -(w.foot.x * n_raw.x + w.foot.y * n_raw.y);
    const double psi = ema_psi_.updateAngle(std::atan2(w.u_c.y, w.u_c.x), alpha);
    const double lat = ema_lat_.update(lat_raw, alpha);
    w.u_c = Point2{std::cos(psi), std::sin(psi)};
    w.foot = leftNormal(w.u_c) * (-lat);

    uint8_t desired = BackupPath::SRC_LANE_CENTER;
    if (w.inflection_valid) {
      desired = (w.second_inflection || !p_.enable_race_line)
        ? BackupPath::SRC_DEGRADED : BackupPath::SRC_RACE_LINE;
    }
    source = holdSource(desired, now, w);
    if (source == BackupPath::SRC_RACE_LINE && !w.inflection_valid) {
      source = BackupPath::SRC_LANE_CENTER;
    }

    if (!w.inflection_valid) {
      ema_corner_n_.reset();
      ema_dpsi_.reset();
      ema_radius_.reset();
    }

    if (source == BackupPath::SRC_RACE_LINE) {
      // 진입점 횡오프셋에 EMA. 종방향은 접근하며 줄어들므로 필터하면 지연이 된다.
      const Point2 nrm = leftNormal(w.u_c);
      const Point2 rel = w.corner - w.foot;
      const double s_c = rel.x * w.u_c.x + rel.y * w.u_c.y;
      const double n_c = ema_corner_n_.update(rel.x * nrm.x + rel.y * nrm.y, alpha);
      const Point2 corner = w.foot + w.u_c * s_c + nrm * n_c;

      const double dpsi = ema_dpsi_.updateAngle(w.delta_psi, alpha);
      double r_raw = solveRaceRadius(std::fabs(dpsi), v_, race_cfg_, veh_);
      r_raw = ema_radius_.update(r_raw, alpha);

      sat_geo = r_raw < veh_.rMin();
      required_steer = std::atan(p_.wheelbase_m / std::max(r_raw, 1e-6));
      // 조향으로 못 내는 곡률은 애초에 만들지 않는다. 부족분은 플래그로 알린다.
      const RaceLine rl =
        raceLineFromRadius(corner, w.u_c, dpsi, std::max(r_raw, veh_.rMin()));
      seg = raceLineSegment(
        rl, corner, p_.tangent_epsilon_m, p_.anchor_at_vehicle, p_.zero_end_curvature);

      double t_entry = 0.0;
      corner_entry_s = arcLengthToPoint(seg, rl.entry, t_entry);
      corner_kappa_max = static_cast<float>(maxAbsCurvatureFrom(seg, t_entry));
    } else {
      seg = centerLineSegment(
        w.foot, w.u_c, path_len, p_.tangent_epsilon_m, p_.anchor_at_vehicle);
      if (source == BackupPath::SRC_DEGRADED) {
        const double k_req = seg.maxAbsCurvature();
        required_steer = veh_.steerOfKappa(k_req);
        double beta = 1.0;
        seg = clampToFeasibleCurvature(seg, veh_.kappaMax(), beta);
        sat_geo = beta < 1.0;
      }
    }
  }

  // 회피 -- 기준 경로 위에 횡 offset 을 중첩한다.
  const AvoidDecision dec =
    decideAvoidance(seg, freshObstacles(now), w.foot, w.u_c, avoid_cfg_);
  avoid_offset_ = rateLimitOffset(
    avoid_offset_, dec.active ? dec.offset : 0.0, p_.avoid_offset_rate_m_per_s, dt_pub);
  const bool avoiding = std::fabs(avoid_offset_) > 1e-4;
  if (avoiding) {
    applyLateralOffset(seg, avoid_offset_);
    source = BackupPath::SRC_AVOID;
  }
  stop_request = dec.stop_request;

  const double seg_len = seg.length();
  const double kmax = seg.maxAbsCurvature();
  if (kmax > veh_.kappaMax()) {
    sat_geo = true;
    required_steer = std::max(required_steer, veh_.steerOfKappa(kmax));
  }
  logSaturation(sat_geo, std::max(corner_entry_s, 0.0), required_steer, now);

  BackupPath out;
  // stamp 는 "이 경로가 유효한 base_link 시각" 이다. DR 로 now 까지 옮겼으므로
  // now 를 넣는다. 원본 stamp 를 넣으면 하류 DR 이 같은 구간을 두 번 보상한다.
  out.header.stamp = p_.use_dead_reckoning ? now : stamp;
  out.header.frame_id = "base_link";
  out.source = source;
  out.degree = static_cast<uint8_t>(backup_common::kDegree);
  out.total_length = seg_len;
  out.ctrl_x.resize(backup_common::kCtrlPerSeg);
  out.ctrl_y.resize(backup_common::kCtrlPerSeg);
  for (int i = 0; i < backup_common::kCtrlPerSeg; ++i) {
    out.ctrl_x[i] = seg.p[i].x;
    out.ctrl_y[i] = seg.p[i].y;
  }
  out.seg_length = {seg_len};
  out.seg_kappa_max = {static_cast<float>(kmax)};
  out.confidence = w.confidence;
  out.valid_length = w.valid_length;  // 외삽 구간을 늘리지 않는다. 그대로 승계
  out.corner_entry_s = corner_entry_s;
  out.corner_kappa_max = corner_kappa_max;
  out.kappa_saturated = sat_geo;
  out.avoiding = avoiding;
  out.stop_request = stop_request;
  out.lost_state = w.lost_state;
  out.lost_first = w.lost_first;
  path_pub_->publish(out);

  RCLCPP_INFO_THROTTLE(
    get_logger(), *get_clock(), period_ms,
    "[계획][plan] src=%u len=%.2f kmax=%.2f valid=%.2f off=%+.3f dr=%.0f ms v=%.2f",
    static_cast<unsigned>(source), seg_len, kmax, w.valid_length, avoid_offset_,
    dr_dt * 1e3, v_);

  if (!have_speed_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), period_ms, "[계획][plan] /speed 미수신, v=0 으로 DR");
  }
}

}  // namespace backup_path_planner
