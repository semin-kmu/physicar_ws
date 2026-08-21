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
 * RViz 의 2D Pose Estimate 를 Cartographer 초기 위치로 넘겨주는 릴레이.
 *
 * AMCL 과 달리 Cartographer 는 /initialpose 를 직접 받지 않는다. 초기 위치를
 * 주는 유일한 경로는 /start_trajectory 서비스이고, 그마저도 실행 중인
 * trajectory 를 먼저 /finish_trajectory 로 닫아야 한다. 이 노드가 그 3단계
 * (상태 조회 -> 종료 -> 재시작) 를 대신한다.
 *
 *     /initialpose (PoseWithCovarianceStamped, frame_id: map)
 *         -> /get_trajectory_states
 *         -> /finish_trajectory  (ACTIVE 인 것만)
 *         -> /start_trajectory   (use_initial_pose=true)
 *
 * relative_to_trajectory_id 는 0 으로 고정한다. pbstream 을 frozen 으로 올리면
 * 저장된 지도가 trajectory 0 이 되고, 우리가 찍는 pose 는 그 지도 좌표계
 * (= map 프레임) 기준이기 때문이다.
 *
 * 구독 콜백 안에서 서비스 응답을 기다리면 executor 스레드가 자기 자신을
 * 기다리는 꼴이라 교착에 빠진다. 그래서 콜백은 pose 를 넘기기만 하고,
 * 실제 3단계는 워커 스레드가 처리한다. 워커가 future 에서 블록하는 동안
 * 응답을 실제로 꺼내주는 쪽은 메인 스레드의 spin 이므로 이 분리가 필요하다.
 */

#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include <cartographer_ros_msgs/msg/trajectory_states.hpp>
#include <cartographer_ros_msgs/srv/finish_trajectory.hpp>
#include <cartographer_ros_msgs/srv/get_trajectory_states.hpp>
#include <cartographer_ros_msgs/srv/start_trajectory.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

namespace kau_localization
{

using cartographer_ros_msgs::msg::TrajectoryStates;
using cartographer_ros_msgs::srv::FinishTrajectory;
using cartographer_ros_msgs::srv::GetTrajectoryStates;
using cartographer_ros_msgs::srv::StartTrajectory;
using geometry_msgs::msg::PoseWithCovarianceStamped;

namespace
{
// 지도가 실린 trajectory. frozen 으로 로드된 pbstream 이 항상 0 번이다.
constexpr int32_t kMapTrajectoryId = 0;

// cartographer_ros_msgs/StatusResponse 는 grpc 상태 코드를 따른다.
constexpr int32_t kStatusOk = 0;

constexpr std::chrono::seconds kServiceTimeout{10};
}  // namespace

class InitialPoseRelay : public rclcpp::Node
{
public:
  InitialPoseRelay()
  : Node("initial_pose_relay")
  {
    config_dir_ = declare_parameter<std::string>("configuration_directory", "");
    config_basename_ = declare_parameter<std::string>(
      "configuration_basename", "physicar_2d_localization.lua");

    if (config_dir_.empty()) {
      RCLCPP_ERROR(
        get_logger(),
        "configuration_directory 가 비어 있다. "
        "/start_trajectory 는 이 값이 없으면 실패한다.");
    }

    get_states_ = create_client<GetTrajectoryStates>("/get_trajectory_states");
    finish_ = create_client<FinishTrajectory>("/finish_trajectory");
    start_ = create_client<StartTrajectory>("/start_trajectory");

    subscription_ = create_subscription<PoseWithCovarianceStamped>(
      "/initialpose", 1,
      [this](const PoseWithCovarianceStamped::SharedPtr msg) {onInitialPose(msg);});

    worker_ = std::thread(&InitialPoseRelay::run, this);

    RCLCPP_INFO(get_logger(), "RViz 의 2D Pose Estimate 를 기다린다 (/initialpose).");
  }

  ~InitialPoseRelay() override
  {
    stop();
  }

  /// 워커를 깨워 종료시키고 join 한다. 소멸자보다 먼저 불러도 안전하다.
  void stop()
  {
    {
      const std::lock_guard<std::mutex> lock(mutex_);
      stopping_ = true;
    }
    condition_.notify_all();
    if (worker_.joinable()) {
      worker_.join();
    }
  }

private:
  void onInitialPose(const PoseWithCovarianceStamped::SharedPtr msg)
  {
    if (!msg->header.frame_id.empty() && msg->header.frame_id != "map") {
      RCLCPP_WARN(
        get_logger(),
        "frame_id 가 '%s' 다. RViz 의 Fixed Frame 을 map 으로 두고 찍어야 한다.",
        msg->header.frame_id.c_str());
      return;
    }

    {
      const std::lock_guard<std::mutex> lock(mutex_);
      // 처리 중에 들어온 pose 는 쌓지 않고 버린다. 큐에 넣어두면 사용자가
      // 여러 번 찍었을 때 지난 위치로 되돌아가는 꼴이 된다.
      if (busy_ || pending_.has_value()) {
        RCLCPP_WARN(get_logger(), "이전 재측위 처리 중이라 이번 pose 는 무시한다.");
        return;
      }
      pending_ = msg->pose.pose;
    }
    condition_.notify_one();
  }

  void run()
  {
    while (true) {
      geometry_msgs::msg::Pose pose;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        condition_.wait(lock, [this] {return stopping_ || pending_.has_value();});
        if (stopping_) {
          return;
        }
        pose = *pending_;
        pending_.reset();
        busy_ = true;
      }

      relocalize(pose);

      {
        const std::lock_guard<std::mutex> lock(mutex_);
        busy_ = false;
      }
    }
  }

  void relocalize(const geometry_msgs::msg::Pose & pose)
  {
    RCLCPP_INFO(
      get_logger(), "재측위 요청: x=%.2f y=%.2f", pose.position.x, pose.position.y);

    const auto states = call(get_states_, std::make_shared<GetTrajectoryStates::Request>());
    if (!states) {
      return;
    }

    // 지도(trajectory 0, FROZEN)는 건드리면 안 되고,
    // 현재 위치 추정 중인 ACTIVE trajectory 만 닫는다.
    const auto & trajectories = states->trajectory_states;
    for (std::size_t i = 0; i < trajectories.trajectory_id.size(); ++i) {
      if (trajectories.trajectory_state[i] != TrajectoryStates::ACTIVE) {
        continue;
      }
      const int32_t id = trajectories.trajectory_id[i];
      RCLCPP_INFO(get_logger(), "trajectory %d 종료.", id);

      auto request = std::make_shared<FinishTrajectory::Request>();
      request->trajectory_id = id;
      if (!call(finish_, request)) {
        return;
      }
    }

    auto request = std::make_shared<StartTrajectory::Request>();
    request->configuration_directory = config_dir_;
    request->configuration_basename = config_basename_;
    request->use_initial_pose = true;
    request->initial_pose = pose;
    request->relative_to_trajectory_id = kMapTrajectoryId;

    const auto started = call(start_, request);
    if (!started) {
      return;
    }

    if (started->status.code != kStatusOk) {
      RCLCPP_ERROR(
        get_logger(), "start_trajectory 실패 (%d): %s",
        started->status.code, started->status.message.c_str());
      return;
    }

    RCLCPP_INFO(
      get_logger(), "trajectory %d 시작. 재측위 완료.", started->trajectory_id);
  }

  /// 서비스 동기 호출. 실패하면 로그를 남기고 nullptr 을 준다.
  ///
  /// 워커 스레드에서만 부른다. 메인 스레드의 spin 이 응답을 처리해야
  /// 여기서 기다리는 블록이 풀린다.
  template<typename ServiceT>
  typename ServiceT::Response::SharedPtr call(
    const std::shared_ptr<rclcpp::Client<ServiceT>> & client,
    const typename ServiceT::Request::SharedPtr & request)
  {
    const std::string name = client->get_service_name();

    if (!client->wait_for_service(kServiceTimeout)) {
      RCLCPP_ERROR(get_logger(), "%s 서비스가 없다. cartographer_node 확인.", name.c_str());
      return nullptr;
    }

    auto future = client->async_send_request(request);
    if (future.wait_for(kServiceTimeout) != std::future_status::ready) {
      // 응답을 안 지우면 요청이 클라이언트에 계속 남는다.
      client->remove_pending_request(future.request_id);
      RCLCPP_ERROR(get_logger(), "%s 호출이 시간 초과됐다.", name.c_str());
      return nullptr;
    }

    return future.get();
  }

  std::string config_dir_;
  std::string config_basename_;

  rclcpp::Client<GetTrajectoryStates>::SharedPtr get_states_;
  rclcpp::Client<FinishTrajectory>::SharedPtr finish_;
  rclcpp::Client<StartTrajectory>::SharedPtr start_;
  rclcpp::Subscription<PoseWithCovarianceStamped>::SharedPtr subscription_;

  std::mutex mutex_;
  std::condition_variable condition_;
  std::optional<geometry_msgs::msg::Pose> pending_;
  bool busy_{false};
  bool stopping_{false};
  std::thread worker_;
};

}  // namespace kau_localization

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<kau_localization::InitialPoseRelay>();
  rclcpp::spin(node);
  // spin 이 풀린 뒤 워커를 먼저 정리해야 rclcpp 핸들이 살아 있는 상태로 join 된다.
  node->stop();
  rclcpp::shutdown();
  return 0;
}
