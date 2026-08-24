// Copyright 2026 KAU AMET Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * 차가 멈춰 있는 동안 AMCL 필터를 강제로 돌린다.
 *
 * AMCL 은 update_min_d / update_min_a 를 넘게 **움직여야** 필터를 갱신한다.
 * 멈추면 갱신이 아예 없고 map -> odom 이 마지막 갱신 값에 얼어붙는다.
 *
 * 그래서 "회전하다 멈추면 그 방향으로 틀어진 채 유지"되는 증상이 생긴다.
 * 감속 구간에서 남은 회전량이 update_min_a 미만이면 마지막 갱신은 고속 회전
 * 중의 것이고, 그때의 각속도 비례 오차가 그대로 남는다. 라이다가 10 Hz 라
 * 80 deg/s 로 돌면 스캔 사이 8 도가 비는데 그 오차가 얼어붙는 것이다.
 *
 * nav2_amcl 은 /request_nomotion_update (std_srvs/Empty) 로 "움직이지 않았지만
 * 한 번 갱신하라" 를 받는다. 이 노드는 정지를 감지해서 그걸 대신 호출한다.
 *
 * 실측 (2026-08-24, sim. yaw 를 8 도 틀어 심고 반복 호출):
 *   0 회  10 cm 이내 38.2 %
 *   4 회             86.7 %   <- 여기서 사실상 끝난다
 *  12 회             97.4 %
 *
 * **무한정 호출하지 않는다.** 두 가지 이유가 있다.
 *
 * 하나, 같은 스캔으로 반복 갱신하면 파티클이 한 점으로 몰려 과신하게 된다
 * (particle depletion). 위 실측에서 xy 표준편차가 23.8 -> 4.2 cm 로 줄었다.
 * 그 정도로 쪼그라들면 나중에 차가 실제로 옮겨졌을 때 못 따라간다.
 *
 * 둘, **이미 맞는 자세에서는 반복 갱신이 오히려 해롭다.** 정답 자세를 심고
 * 돌리면 96.2 -> 74.3 % 로 밀린다 (12 회). 최적점 근처는 우도 기울기가 평평해서
 * 복원력보다 리샘플링 노이즈가 커 랜덤워크가 되기 때문이다. 멀리 있을 때만
 * 기울기가 끌어당긴다.
 *
 * 그래서 "틀어졌으면 복구되고, 맞았으면 크게 안 망가지는" 구간인 max_calls 회
 * (기본 4) 만 호출하고 멈춘다. rate_hz 2.0 이므로 정지 후 2 초면 끝난다.
 *
 * Cartographer 와는 무관하다. amcl.launch.py 에서만 띄운다.
 */

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <std_srvs/srv/empty.hpp>

namespace
{

class NomotionUpdater : public rclcpp::Node
{
public:
  NomotionUpdater()
  : rclcpp::Node("nomotion_updater")
  {
    const auto odom_topic = declare_parameter<std::string>("odom_topic", "/odom");
    service_name_ = declare_parameter<std::string>(
      "service_name", "/request_nomotion_update");
    rate_hz_ = declare_parameter<double>("rate_hz", 2.0);
    linear_threshold_ = declare_parameter<double>("linear_threshold", 0.02);
    angular_threshold_ = declare_parameter<double>("angular_threshold", 0.05);
    settle_sec_ = declare_parameter<double>("settle_sec", 0.3);
    max_calls_ = declare_parameter<int>("max_calls", 4);
    reset_distance_ = declare_parameter<double>("reset_distance", 0.10);
    reset_angle_ = declare_parameter<double>("reset_angle", 0.10);

    client_ = create_client<std_srvs::srv::Empty>(service_name_);

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, rclcpp::SensorDataQoS(),
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {onOdom(*msg);});

    const auto period = std::chrono::duration<double>(1.0 / std::max(0.1, rate_hz_));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() {onTimer();});

    RCLCPP_INFO(
      get_logger(),
      "정지 시 %s 를 %.1f Hz 로 최대 %d 회 호출한다 "
      "(정지 판정: |v| < %.3f m/s, |w| < %.3f rad/s 가 %.1f s 지속. "
      "예산은 %.2f m 또는 %.2f rad 실제로 움직여야 되살아난다)",
      service_name_.c_str(), rate_hz_, max_calls_,
      linear_threshold_, angular_threshold_, settle_sec_,
      reset_distance_, reset_angle_);
  }

private:
  void onOdom(const nav_msgs::msg::Odometry & msg)
  {
    const double v = std::hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y);
    const double w = std::abs(msg.twist.twist.angular.z);
    const bool still = v < linear_threshold_ && w < angular_threshold_;

    const double x = msg.pose.pose.position.x;
    const double y = msg.pose.pose.position.y;
    const auto & q = msg.pose.pose.orientation;
    const double yaw = std::atan2(
      2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));

    if (!still) {
      stopped_since_.reset();
      // **호출 예산은 여기서 되돌리지 않는다.** 정지 중에도 EKF twist 에는
      // 노이즈가 남아서 순간적으로 임계값을 넘는다. 속도로 리셋하면 예산이
      // 계속 되살아나 사실상 무제한이 되고, 같은 스캔으로 끝없이 갱신해
      // 파티클이 한 점으로 몰린다. 실제로 움직였는지는 아래에서 자세로 본다.
      return;
    }
    if (!stopped_since_.has_value()) {
      stopped_since_ = now();
    }
    if (!budget_anchor_.has_value()) {
      budget_anchor_ = Pose2{x, y, yaw};
      return;
    }
    // 예산을 되살리는 조건: 마지막으로 예산을 채운 지점에서 실제로 이만큼
    // 움직였을 때만. 노이즈는 이 문턱을 못 넘는다.
    const double moved = std::hypot(x - budget_anchor_->x, y - budget_anchor_->y);
    const double turned = std::abs(wrap(yaw - budget_anchor_->yaw));
    if (moved > reset_distance_ || turned > reset_angle_) {
      budget_anchor_ = Pose2{x, y, yaw};
      calls_done_ = 0;
    }
  }

  static double wrap(double a)
  {
    while (a > M_PI) {a -= 2.0 * M_PI;}
    while (a < -M_PI) {a += 2.0 * M_PI;}
    return a;
  }

  void onTimer()
  {
    if (!stopped_since_.has_value()) {
      return;
    }
    if ((now() - *stopped_since_).seconds() < settle_sec_) {
      return;
    }
    if (max_calls_ > 0 && calls_done_ >= max_calls_) {
      return;
    }
    if (!client_->service_is_ready()) {
      // AMCL 이 아직 안 떴거나 활성화 전이다. 조용히 넘긴다 — 뜨면 알아서 붙는다.
      return;
    }
    // 응답을 기다리지 않는다. 타이머 콜백에서 future 를 블록하면 executor 가
    // 자기 자신을 기다리는 교착이 된다.
    client_->async_send_request(std::make_shared<std_srvs::srv::Empty::Request>());
    ++calls_done_;
    if (calls_done_ == 1) {
      RCLCPP_DEBUG(get_logger(), "정지 감지. 강제 갱신 시작.");
    }
  }

  std::string service_name_;
  double rate_hz_{2.0};
  double linear_threshold_{0.02};
  double angular_threshold_{0.05};
  double settle_sec_{0.3};
  int max_calls_{4};

  struct Pose2
  {
    double x;
    double y;
    double yaw;
  };

  std::optional<rclcpp::Time> stopped_since_;
  std::optional<Pose2> budget_anchor_;
  int calls_done_{0};
  double reset_distance_{0.10};
  double reset_angle_{0.10};

  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr client_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NomotionUpdater>());
  rclcpp::shutdown();
  return 0;
}
