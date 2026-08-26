// ====================================================================
// simple_drive / decision_node
//
// 판단. 인지 출력만 보고 "지금 몇 도로 얼마나 빨리" 를 정한다.
// 실제 발행(레이트 리밋 / 워치독 / 단위변환)은 control_node 몫이다.
//
// 입력 (전부 기존 backup 인지 노드가 이미 내는 것):
//   /backup/lane/path            BackupPath              BEV 노란 중앙선
//   /backup/lane/left            BackupPath              좌 흰선
//   /backup/lane/right           BackupPath              우 흰선
//   /backup/perception/obstacles ObstacleCircleArray     라이다 원
//   /perception/start_permission Bool                    출발 신호
//
// 출력:
//   /simple/cmd_steer_deg  Float64  [deg] 좌측 +. 0 / +-20 만 나온다
//   /simple/cmd_speed      Float64  [m/s]
//   /simple/debug          ControlDebug
//   /simple/state          String   "STRAIGHT|center" 처럼 상태|차선근거
//
// 조향은 두 값만 쓴다: 0 또는 +-max_steer_deg. 사이값은 만들지 않는다.
// 이 스택의 전제가 "비례제어 튜닝을 포기하고 저속 + 전타 스위칭으로
// 간다" 이기 때문이다.
//
// --------------------------------------------------------------
// 차선 근거의 3 단 폴백 (실기동 2026-08-26 에서 나온 요구)
// --------------------------------------------------------------
// 회피로 전타를 주면 카메라 방위가 20 도 넘게 돌아 노란 중앙선이
// 화각 밖으로 나간다. 그때 /backup/lane/path 는 끊기지만 좌/우 흰선
// (/backup/lane/left, /right)은 14 Hz 로 계속 나온다 -- 실측 확인.
//
//   1) 중앙선      /backup/lane/path 를 그대로 쓴다
//   2) 흰선 복원   흰선을 lane_width_m 만큼 안쪽으로 밀어 중앙선을
//                  복원한다. 좌 = e - w, 우 = e + w, 둘 다면 평균
//   3) 유예        셋 다 없으면 lost_grace_s 동안 0 도 저속 전진.
//                  그 뒤에도 없으면 정지
//
// 유예 구간에서 "마지막 조향 유지" 가 아니라 0 도인 이유:
// 근거가 끊기는 시점은 대개 방금 크게 꺾은 직후다. 전타를 그대로
// 들고 가면 v=0.25 로 2 초만 지나도 방위가 58 도 더 돈다. NaN 을
// 0 으로 떨구는 것과 같은 판단이다 -- 모르면 직진이 전타보다 안전하다.
// ====================================================================

#ifndef SIMPLE_DRIVE__DECISION_NODE_HPP_
#define SIMPLE_DRIVE__DECISION_NODE_HPP_

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>

#include <backup_msgs/msg/backup_path.hpp>
#include <backup_msgs/msg/control_debug.hpp>
#include <backup_msgs/msg/obstacle_circle_array.hpp>

#include "simple_drive/avoid_fsm.hpp"
#include "simple_drive/bang_bang.hpp"
#include "simple_drive/lane_source.hpp"
#include "simple_drive/path_sample.hpp"

namespace simple_drive
{

class DecisionNode : public rclcpp::Node
{
public:
  DecisionNode();

private:
  void onPath(const backup_msgs::msg::BackupPath::SharedPtr msg);
  void onLeft(const backup_msgs::msg::BackupPath::SharedPtr msg);
  void onRight(const backup_msgs::msg::BackupPath::SharedPtr msg);
  void onObstacles(const backup_msgs::msg::ObstacleCircleArray::SharedPtr msg);
  void onPermission(const std_msgs::msg::Bool::SharedPtr msg);
  void onTimer();

  // 신선한 것만 골라 넘긴다. 오래된 경로는 없는 것과 같다.
  const backup_msgs::msg::BackupPath * freshOr(
    const backup_msgs::msg::BackupPath::SharedPtr & msg,
    const rclcpp::Time & stamp, const rclcpp::Time & nowt, double timeout_s) const;

  // --- 파라미터 ---
  double control_hz_ = 50.0;
  double path_timeout_s_ = 0.4;
  double obstacle_timeout_s_ = 0.5;

  double max_steer_deg_ = 20.0;

  double v_straight_ = 0.45;
  double v_corner_ = 0.30;
  double v_avoid_ = 0.30;
  double v_lost_ = 0.25;
  double lost_grace_s_ = 2.0;
  bool require_permission_ = true;
  // 레벨(초록)만 보면 이미 초록인 상태로 기동했을 때 전이를 못 보고
  // 나간다. 빨강을 한 번 본 뒤의 상승엣지를 요구한다.
  bool require_red_to_green_ = true;

  // --- 상태 ---
  BangBang bang_;
  AvoidFsm avoid_;
  LaneResolver lanes_;
  LaneOrigin last_origin_ = LaneOrigin::None;

  backup_msgs::msg::BackupPath::SharedPtr path_;
  backup_msgs::msg::BackupPath::SharedPtr left_;
  backup_msgs::msg::BackupPath::SharedPtr right_;
  rclcpp::Time path_stamp_;
  rclcpp::Time left_stamp_;
  rclcpp::Time right_stamp_;
  backup_msgs::msg::ObstacleCircleArray::SharedPtr obstacles_;
  rclcpp::Time obstacle_stamp_;
  bool permission_ = false;
  bool armed_ = false;    // 빨강(false)을 한 번 봤다
  bool started_ = false;  // 초록 상승엣지를 봤다. 한 번 서면 안 풀린다
  rclcpp::Time last_tick_;
  double lost_s_ = 0.0;  // 차선 근거가 끊긴 뒤 경과 시간

  std::string last_logged_state_;

  // --- 배선 ---
  rclcpp::Subscription<backup_msgs::msg::BackupPath>::SharedPtr path_sub_;
  rclcpp::Subscription<backup_msgs::msg::BackupPath>::SharedPtr left_sub_;
  rclcpp::Subscription<backup_msgs::msg::BackupPath>::SharedPtr right_sub_;
  rclcpp::Subscription<backup_msgs::msg::ObstacleCircleArray>::SharedPtr obstacle_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr permission_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr steer_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr speed_pub_;
  rclcpp::Publisher<backup_msgs::msg::ControlDebug>::SharedPtr debug_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool publish_debug_ = true;
};

}  // namespace simple_drive

#endif  // SIMPLE_DRIVE__DECISION_NODE_HPP_
