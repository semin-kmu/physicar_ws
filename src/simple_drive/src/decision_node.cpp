#include "simple_drive/decision_node.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>

namespace simple_drive
{

namespace
{
// 제어 지시는 놓치면 안 되지만 쌓이면 늦은 값을 쓴다.
rclcpp::QoS controlQos()
{
  return rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();
}
}  // namespace


DecisionNode::DecisionNode()
: rclcpp::Node("simple_decision")
{
  const auto path_topic =
    declare_parameter<std::string>("path_topic", "/backup/lane/path");
  const auto left_topic =
    declare_parameter<std::string>("left_topic", "/backup/lane/left");
  const auto right_topic =
    declare_parameter<std::string>("right_topic", "/backup/lane/right");
  const auto obstacle_topic =
    declare_parameter<std::string>("obstacle_topic", "/backup/perception/obstacles");
  const auto permission_topic =
    declare_parameter<std::string>("permission_topic", "/perception/start_permission");
  const auto steer_topic =
    declare_parameter<std::string>("steer_topic", "/simple/cmd_steer_deg");
  const auto speed_topic =
    declare_parameter<std::string>("speed_topic", "/simple/cmd_speed");
  const auto debug_topic =
    declare_parameter<std::string>("debug_topic", "/simple/debug");
  const auto state_topic =
    declare_parameter<std::string>("state_topic", "/simple/state");
  publish_debug_ = declare_parameter<bool>("publish_debug", true);

  control_hz_ = declare_parameter<double>("control_hz", 50.0);
  path_timeout_s_ = declare_parameter<double>("path_timeout_s", 0.4);
  obstacle_timeout_s_ = declare_parameter<double>("obstacle_timeout_s", 0.5);

  max_steer_deg_ = declare_parameter<double>("max_steer_deg", 20.0);

  LaneResolver::Params lp;
  lp.lookahead_m = declare_parameter<double>("lookahead_m", 0.28);
  lp.valid_length_margin_m = declare_parameter<double>("valid_length_margin_m", 0.05);
  lp.samples_per_seg = static_cast<int>(declare_parameter<int>("samples_per_seg", 32));
  lp.lane_width_m = declare_parameter<double>("lane_width_m", 0.317);
  lp.white_min_valid_length_m =
    declare_parameter<double>("white_min_valid_length_m", 0.15);
  lp.pair_max_disagree_m = declare_parameter<double>("white_pair_max_disagree_m", 0.10);
  if (lp.lane_width_m <= 0.0) {
    throw std::invalid_argument("lane_width_m must be > 0");
  }
  lanes_.setParams(lp);

  v_straight_ = declare_parameter<double>("v_straight", 0.45);
  v_corner_ = declare_parameter<double>("v_corner", 0.30);
  v_avoid_ = declare_parameter<double>("v_avoid", 0.30);
  v_lost_ = declare_parameter<double>("v_lost", 0.25);
  lost_grace_s_ = declare_parameter<double>("lost_grace_s", 2.0);
  require_permission_ = declare_parameter<bool>("require_permission", true);
  require_red_to_green_ = declare_parameter<bool>("require_red_to_green", true);

  BangBang::Params bp;
  bp.enter_m = declare_parameter<double>("bang.enter_m", 0.05);
  bp.exit_m = declare_parameter<double>("bang.exit_m", 0.02);
  bp.min_hold_s = declare_parameter<double>("bang.min_hold_s", 0.06);
  if (bp.exit_m > bp.enter_m) {
    // 이탈 임계가 진입보다 크면 히스테리시스가 뒤집혀 한 번 꺾은 뒤
    // 영원히 안 놓는다. 조용히 고치지 않고 세운다.
    throw std::invalid_argument("bang.exit_m must be <= bang.enter_m");
  }
  bang_.setParams(bp);

  AvoidFsm::Params ap;
  ap.trigger_x_min_m = declare_parameter<double>("avoid.trigger_x_min_m", 0.15);
  ap.trigger_x_max_m = declare_parameter<double>("avoid.trigger_x_max_m", 0.90);
  ap.trigger_y_abs_m = declare_parameter<double>("avoid.trigger_y_abs_m", 0.20);
  ap.dodge_deg = declare_parameter<double>("avoid.dodge_deg", 20.0);
  ap.dodge_s = declare_parameter<double>("avoid.dodge_s", 1.05);
  ap.counter_s = declare_parameter<double>("avoid.counter_s", 1.05);
  ap.hold_s = declare_parameter<double>("avoid.hold_s", 1.00);
  if (ap.trigger_x_min_m >= ap.trigger_x_max_m) {
    throw std::invalid_argument("avoid.trigger_x_min_m must be < avoid.trigger_x_max_m");
  }
  if (ap.dodge_deg < 0.0 || ap.dodge_deg > max_steer_deg_) {
    throw std::invalid_argument("avoid.dodge_deg must be within [0, max_steer_deg]");
  }
  avoid_.setParams(ap);

  if (control_hz_ <= 0.0) {throw std::invalid_argument("control_hz must be > 0");}

  const auto lane_qos = rclcpp::QoS(1).reliable();
  path_sub_ = create_subscription<backup_msgs::msg::BackupPath>(
    path_topic, lane_qos,
    std::bind(&DecisionNode::onPath, this, std::placeholders::_1));
  left_sub_ = create_subscription<backup_msgs::msg::BackupPath>(
    left_topic, lane_qos,
    std::bind(&DecisionNode::onLeft, this, std::placeholders::_1));
  right_sub_ = create_subscription<backup_msgs::msg::BackupPath>(
    right_topic, lane_qos,
    std::bind(&DecisionNode::onRight, this, std::placeholders::_1));
  obstacle_sub_ = create_subscription<backup_msgs::msg::ObstacleCircleArray>(
    obstacle_topic, controlQos(),
    std::bind(&DecisionNode::onObstacles, this, std::placeholders::_1));
  permission_sub_ = create_subscription<std_msgs::msg::Bool>(
    permission_topic, controlQos(),
    std::bind(&DecisionNode::onPermission, this, std::placeholders::_1));

  steer_pub_ = create_publisher<std_msgs::msg::Float64>(steer_topic, controlQos());
  speed_pub_ = create_publisher<std_msgs::msg::Float64>(speed_topic, controlQos());
  state_pub_ = create_publisher<std_msgs::msg::String>(state_topic, controlQos());
  if (publish_debug_) {
    debug_pub_ = create_publisher<backup_msgs::msg::ControlDebug>(debug_topic, controlQos());
  }

  last_tick_ = now();
  path_stamp_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
  left_stamp_ = path_stamp_;
  right_stamp_ = path_stamp_;
  obstacle_stamp_ = path_stamp_;

  timer_ = rclcpp::create_timer(
    this, get_clock(), rclcpp::Duration::from_seconds(1.0 / control_hz_),
    std::bind(&DecisionNode::onTimer, this));

  RCLCPP_INFO(
    get_logger(),
    "[기동][decision] center=%s left=%s right=%s obstacles=%s permission=%s "
    "-> %s / %s | %.0f Hz Ld %.2f m 밴뱅 %.0f deg (enter %.3f / exit %.3f m) "
    "흰선폴백 w %.3f m (최소 %.2f m) 유예 %.1f s "
    "회피 %.1f deg %.2f+%.2f+%.2f s 속도 %.2f/%.2f/%.2f/%.2f m/s",
    path_topic.c_str(), left_topic.c_str(), right_topic.c_str(),
    obstacle_topic.c_str(), permission_topic.c_str(),
    steer_topic.c_str(), speed_topic.c_str(),
    control_hz_, lp.lookahead_m, max_steer_deg_, bp.enter_m, bp.exit_m,
    lp.lane_width_m, lp.white_min_valid_length_m, lost_grace_s_,
    ap.dodge_deg, ap.dodge_s, ap.counter_s, ap.hold_s,
    v_straight_, v_corner_, v_avoid_, v_lost_);
}


void DecisionNode::onPath(const backup_msgs::msg::BackupPath::SharedPtr msg)
{
  path_ = msg;
  path_stamp_ = now();
}


void DecisionNode::onLeft(const backup_msgs::msg::BackupPath::SharedPtr msg)
{
  left_ = msg;
  left_stamp_ = now();
}


void DecisionNode::onRight(const backup_msgs::msg::BackupPath::SharedPtr msg)
{
  right_ = msg;
  right_stamp_ = now();
}


void DecisionNode::onObstacles(const backup_msgs::msg::ObstacleCircleArray::SharedPtr msg)
{
  obstacles_ = msg;
  obstacle_stamp_ = now();
}


void DecisionNode::onPermission(const std_msgs::msg::Bool::SharedPtr msg)
{
  permission_ = msg->data;

  if (started_) {return;}

  if (!permission_) {
    // 빨강을 봤다. 이제부터의 상승엣지가 진짜 출발 신호다.
    if (!armed_) {
      RCLCPP_INFO(get_logger(), "[판단] 정지 신호 확인 -- 출발 신호 대기");
      armed_ = true;
    }
    return;
  }

  // 여기부터 permission_ == true.
  // 기동했을 때 이미 초록이면 armed_ 가 false 라 출발하지 않는다.
  // 빨강 -> 초록 전이를 직접 봐야 출발한다는 것이 규칙이기 때문이다.
  if (armed_ || !require_red_to_green_) {
    started_ = true;
    RCLCPP_INFO(
      get_logger(), "[판단] 출발 신호 확인 (%s) -- 출발",
      require_red_to_green_ ? "빨강->초록 전이" : "레벨");
  }
}


const backup_msgs::msg::BackupPath * DecisionNode::freshOr(
  const backup_msgs::msg::BackupPath::SharedPtr & msg,
  const rclcpp::Time & stamp, const rclcpp::Time & nowt, double timeout_s) const
{
  if (!msg) {return nullptr;}
  if ((nowt - stamp).seconds() >= timeout_s) {return nullptr;}
  return msg.get();
}


void DecisionNode::onTimer()
{
  const rclcpp::Time t = now();
  double dt = (t - last_tick_).seconds();
  last_tick_ = t;
  // 시뮬 시간 점프나 첫 tick 에서 dt 가 터무니없이 크면 타이머 상태가
  // 한 번에 다 소모된다. 공칭 주기로 되돌린다.
  if (!std::isfinite(dt) || dt <= 0.0 || dt > 5.0 / control_hz_) {
    dt = 1.0 / control_hz_;
  }

  const bool obstacle_fresh =
    obstacles_ && (t - obstacle_stamp_).seconds() < obstacle_timeout_s_;

  // --- 1. 회피 (차선추종보다 우선) ---
  // 회피 FSM 은 타이머 오픈루프다. 차선 근거를 전혀 쓰지 않으므로
  // 경로가 끊겨도 시퀀스는 끝까지 돈다. (2026-08-26 실기동에서
  // 회피 도중 NO_PATH 로 정지해 버린 것이 이 분리를 안 한 탓이다.)
  const double avoid_deg =
    avoid_.update(obstacle_fresh ? obstacles_.get() : nullptr, dt);

  // --- 2. 차선 근거 해석 (중앙선 -> 흰선 -> 없음) ---
  const LaneEstimate lane = lanes_.resolve(
    freshOr(path_, path_stamp_, t, path_timeout_s_),
    freshOr(left_, left_stamp_, t, path_timeout_s_),
    freshOr(right_, right_stamp_, t, path_timeout_s_));

  if (lane.valid) {
    lost_s_ = 0.0;
  } else {
    lost_s_ += dt;
  }

  if (lane.origin != last_origin_) {
    RCLCPP_INFO(
      get_logger(), "[판단] 차선 근거 %s -> %s",
      toString(last_origin_), toString(lane.origin));
    last_origin_ = lane.origin;
  }

  // --- 3. 밴뱅 ---
  // 근거가 없으면 e 를 NaN 으로 준다 -> BangBang 이 0 도로 떨군다.
  const double e = lane.valid ? lane.lateral : std::numeric_limits<double>::quiet_NaN();
  const int bang = bang_.update(e, dt);

  // --- 4. 합성 ---
  std::string state;
  double steer_deg = 0.0;
  double speed = 0.0;

  if (require_permission_ && !started_) {
    // armed_ 가 false 면 아직 빨강을 못 봤다 -- 지금 초록이어도 안 나간다.
    state = armed_ ? "WAIT_GREEN" : "WAIT_RED_FIRST";
    bang_.reset();
    avoid_.reset();
  } else if (avoid_.ownsSteer()) {
    state = std::string("AVOID_") + toString(avoid_.state());
    steer_deg = avoid_deg;
    speed = v_avoid_;
    // 회피가 끝난 뒤 밴뱅이 회피 직전 상태를 그대로 이어받지 않게 한다.
    bang_.reset();
  } else if (!lane.valid) {
    // 유예: 근거가 끊겨도 곧바로 세우지 않는다. 0 도로 저속 전진하며
    // 차선이 다시 화각에 들어오기를 기다린다.
    if (lost_s_ < lost_grace_s_) {
      state = "LOST_GRACE";
      speed = v_lost_;
    } else {
      state = "LOST_STOP";
      speed = 0.0;
    }
  } else if (bang != 0) {
    state = (bang > 0) ? "TURN_LEFT" : "TURN_RIGHT";
    steer_deg = bang * max_steer_deg_;
    speed = v_corner_;
  } else {
    state = "STRAIGHT";
    speed = v_straight_;
  }

  // 상태만으로는 흰선으로 버티는 중인지 안 보인다. 근거를 같이 싣는다.
  const std::string tagged = state + "|" + toString(lane.origin);

  std_msgs::msg::Float64 s;
  s.data = steer_deg;
  steer_pub_->publish(s);

  std_msgs::msg::Float64 v;
  v.data = speed;
  speed_pub_->publish(v);

  std_msgs::msg::String st;
  st.data = tagged;
  state_pub_->publish(st);

  if (state != last_logged_state_) {
    RCLCPP_INFO(
      get_logger(), "[판단] %s -> %s (%s) | steer %.1f deg v %.2f m/s e %.3f m",
      last_logged_state_.empty() ? "(초기)" : last_logged_state_.c_str(),
      state.c_str(), toString(lane.origin), steer_deg, speed,
      lane.valid ? lane.lateral : 0.0);
    last_logged_state_ = state;
  }

  if (debug_pub_) {
    backup_msgs::msg::ControlDebug d;
    d.header.stamp = t;
    d.header.frame_id = "base_link";
    d.path_source = static_cast<std::uint8_t>(lane.origin);
    d.tracking_ok = lane.valid;
    d.cmd_speed = speed;
    d.v_target = speed;
    d.kappa_ahead = 0.0;  // 곡률은 판단에 안 쓴다
    d.cmd_steer_deg = steer_deg;
    d.lookahead_m = lane.s;
    d.heading_error_rad = lane.heading;
    d.cross_track_m = lane.valid ? lane.lateral : 0.0;
    d.delta_saturated = (steer_deg != 0.0);
    d.kappa_saturated = false;
    d.dead_reckon_dt = lost_s_;  // 근거가 끊긴 시간을 여기에 싣는다
    debug_pub_->publish(d);
  }
}

}  // namespace simple_drive
