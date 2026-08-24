// ====================================================================
// ros_bridge.hpp
//
// 구독 전담 rclcpp 노드. Qt 를 전혀 모른다 (QPointF 만 자료형으로 쓴다).
//
// **관측 전용.** publisher 도 service client 도 만들지 않는다.
// 이 클래스에 발행 코드가 생기면 docs/09 계약 104·128 위반이다.
//
// 구독은 두 종류로 나뉜다.
//
//   표시용 구독  타입을 아는 메시지. 실제로 화면에 그린다.
//   감시용 구독  generic subscription. 역직렬화하지 않고 수신 시각만 찍는다.
//                노드가 "제 주기로 돌고 있는가" 만 알면 되므로 내용이
//                필요 없고, 그 덕에 감시 대상을 config 만으로 늘릴 수 있다.
//
// 스레드: rclcpp executor 가 별도 스레드에서 spin 하고, Qt 메인 스레드는
//         snapshot() 만 호출한다. 공유 상태는 mu_ 하나로 지킨다.
// ====================================================================

#ifndef KAU_GUI__ROS_BRIDGE_HPP_
#define KAU_GUI__ROS_BRIDGE_HPP_

#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/generic_subscription.hpp>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/float64.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "kau_msgs/msg/steer_debug.hpp"

#include "kau_gui/types.hpp"


namespace kau_gui
{

class RosBridge : public rclcpp::Node
{
public:
    RosBridge();

    // Qt 메인 스레드가 렌더 직전에 한 번 호출한다.
    Snapshot snapshot();

    double renderHz() const
    {
        return render_hz_;
    }

    double vehicleLengthM() const
    {
        return veh_len_;
    }

    double vehicleWidthM() const
    {
        return veh_wid_;
    }

private:
    // --- 표시용 콜백 ---
    void onScan(const sensor_msgs::msg::LaserScan::SharedPtr m);
    void onMarkers(const visualization_msgs::msg::MarkerArray::SharedPtr m);
    void onGrid(const nav_msgs::msg::OccupancyGrid::SharedPtr m);
    void onLookahead(const geometry_msgs::msg::PointStamped::SharedPtr m);
    void onOdom(const nav_msgs::msg::Odometry::SharedPtr m);
    void onSpeed(const std_msgs::msg::Float64::SharedPtr m);
    void onSteering(const std_msgs::msg::Float64::SharedPtr m);
    void onSteerDebug(const kau_msgs::msg::SteerDebug::SharedPtr m);

    void onPath(
        const nav_msgs::msg::Path::SharedPtr m, Latest<Polyline> * dst);

    // --- 주기 작업 ---
    void onPoseTimer();     // TF map->base 조회 + 맵 파일 폴백
    void onGraphTimer();    // ROS graph 노드 목록 + 등급 산출

    // --- 도우미 ---
    double stamp() const;

    bool loadMapFile(const std::string & yaml_path, MapImage * out);

    void declareAll();

    void makeDisplaySubs();

    void makeWatchSubs();

    // --- 설정 ---
    double render_hz_    = 10.0;
    double history_s_    = 30.0;
    double tf_timeout_   = 0.5;
    double stale_ratio_  = 3.0;
    double min_stale_s_  = 0.3;
    double veh_len_      = 0.30;
    double veh_wid_      = 0.20;

    std::string map_frame_;
    std::string base_frame_;

    // 감시 대상. 네 배열의 같은 index 가 한 노드를 이룬다.
    std::vector<std::string> watch_names_;
    std::vector<std::string> watch_topics_;
    std::vector<std::string> watch_types_;
    std::vector<double>      watch_rates_;

    // --- 상태 (mu_ 로 보호) ---
    std::mutex mu_;

    Snapshot snap_;

    // 감시 토픽별 수신 주기.
    std::map<std::string, RateMeter> rates_;

    // 맵을 topic 으로 받았는가. auto 모드 파일 폴백 판정에 쓴다.
    bool        map_from_topic_ = false;
    bool        map_file_tried_ = false;
    double      start_t_        = 0.0;
    double      map_wait_s_     = 5.0;
    std::string map_yaml_;

    // --- ROS ---
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr
        marker_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr global_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr local_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr lane_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr
        lookahead_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr speed_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr steering_sub_;
    rclcpp::Subscription<kau_msgs::msg::SteerDebug>::SharedPtr debug_sub_;

    std::vector<rclcpp::GenericSubscription::SharedPtr> watch_subs_;

    rclcpp::TimerBase::SharedPtr pose_timer_;
    rclcpp::TimerBase::SharedPtr graph_timer_;

    std::unique_ptr<tf2_ros::Buffer>            tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace kau_gui

#endif  // KAU_GUI__ROS_BRIDGE_HPP_
