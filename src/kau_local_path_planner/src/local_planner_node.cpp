// ====================================================================
// local_planner_node.cpp
//
// ROS2 wiring: 구독(/path/global, /lane/center, /perception/obstacles,
// /steering, TF map->base_footprint), local_planner 호출,
// 발행(/path/local = kau_msgs/KauPath, /viz/path/local = nav_msgs/Path).
//
// 알려진 단순화 (KAU_AMET_ROS 세션에서 시간 제약으로 결정, 후속 확인 필요):
//   - /path/global, /lane/center 가 항상 map frame 으로 온다고 가정한다.
//     문서상 Lane Detection 은 localization 상실 시 base_link 로 강등할
//     수 있는데, 그 경우의 frame 변환(제어점에 강체변환 적용, §7.1)은
//     아직 구현하지 않았다 -- frame_id 가 기대와 다르면 경고만 내고
//     그대로 사용한다.
// ====================================================================

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <yaml-cpp/yaml.h>

#include "kau_control/kau_path.hpp"
#include "kau_msgs/msg/kau_path.hpp"
#include "kau_msgs/msg/obstacle_circle_array.hpp"
#include "kau_local_path_planner/local_planner.hpp"

namespace
{

using kau::local_path_planner::Frame;
using kau::local_path_planner::LocalPlanner;
using kau::local_path_planner::Obstacle;
using kau::local_path_planner::PlannerParams;
using kau::local_path_planner::PlanStatus;
using kau::local_path_planner::Point2;
using kau::local_path_planner::RoadBoundary;
using kau::local_path_planner::SimToMap;
using kau::local_path_planner::VehicleFootprint;

// Python: config.END_RATIO -- Curve -> KauPath 발행 시에는 쓰지 않지만
// 다른 상수들과 나란히 두어 출처를 분명히 한다.
constexpr double kEndRatio = 0.80;

kau_msgs::msg::KauPath curveToKauPath(
    const kau::control::Curve & cv, uint8_t source, const std::string & frame_id,
    const rclcpp::Time & stamp, float confidence, double valid_length)
{
    kau_msgs::msg::KauPath msg;
    msg.header.frame_id = frame_id;
    msg.header.stamp = stamp;
    msg.source = source;
    msg.degree = static_cast<uint8_t>(kau::bezier::DEGREE);
    msg.is_closed = cv.closed();
    msg.s_offset = 0.0;
    msg.total_length = cv.length();

    const int nseg = cv.nseg();
    msg.ctrl_x.reserve(static_cast<std::size_t>(nseg) * kau::bezier::NCTRL);
    msg.ctrl_y.reserve(static_cast<std::size_t>(nseg) * kau::bezier::NCTRL);
    msg.seg_length.reserve(static_cast<std::size_t>(nseg));
    msg.seg_kappa_max.reserve(static_cast<std::size_t>(nseg));

    for (int i = 0; i < nseg; ++i)
    {
        for (const auto & pt : cv.seg(i))
        {
            msg.ctrl_x.push_back(pt.x);
            msg.ctrl_y.push_back(pt.y);
        }
        msg.seg_length.push_back(cv.segLen(i));
        msg.seg_kappa_max.push_back(
            static_cast<float>(kau::bezier::kappaMaxExact(cv.seg(i))));
    }

    msg.confidence = confidence;
    msg.valid_length = valid_length;
    return msg;
}

// RViz 시각화용 nav_msgs/Path. KauPath 는 Bezier 제어점이라 RViz 가
// 그리지 못하므로, 곡선을 호길이 step 간격으로 샘플링해 폴리라인으로
// 편다. 단위 주의: 내부 계산과 KauPath 는 전부 cm 이지만 ROS 표준
// 메시지(RViz 포함)는 m 이므로 여기서 /100 한다.
nav_msgs::msg::Path curveToNavPath(
    const kau::control::Curve & cv, const std::string & frame_id,
    const rclcpp::Time & stamp, double step_cm)
{
    nav_msgs::msg::Path msg;
    msg.header.frame_id = frame_id;
    msg.header.stamp = stamp;

    const double total = cv.length();
    if (total <= 0.0 || step_cm <= 0.0)
    {
        return msg;
    }

    // 끝점을 반드시 포함하도록 개수를 올림해 균등 분할한다.
    const int nsample =
        std::max(2, static_cast<int>(std::ceil(total / step_cm)) + 1);
    msg.poses.reserve(static_cast<std::size_t>(nsample));

    for (int i = 0; i < nsample; ++i)
    {
        const double s =
            total * static_cast<double>(i) / static_cast<double>(nsample - 1);
        const Point2 pt = cv.point(s);

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, cv.heading(s));

        geometry_msgs::msg::PoseStamped pose;
        pose.header = msg.header;
        pose.pose.position.x = pt.x / 100.0;
        pose.pose.position.y = pt.y / 100.0;
        pose.pose.position.z = 0.0;
        pose.pose.orientation = tf2::toMsg(q);

        msg.poses.push_back(std::move(pose));
    }

    return msg;
}

}  // namespace

class LocalPlannerNode : public rclcpp::Node
{
public:
    LocalPlannerNode()
    : rclcpp::Node("local_planner_node")
    {
        declareParameters();

        map_frame_ = get_parameter("map_frame").as_string();
        base_frame_ = get_parameter("base_frame").as_string();

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        boundary_ = loadBoundary();

        rclcpp::QoS global_qos(rclcpp::KeepLast(1));
        global_qos.reliable().transient_local();
        global_path_sub_ = create_subscription<kau_msgs::msg::KauPath>(
            "/path/global", global_qos,
            std::bind(&LocalPlannerNode::onGlobalPath, this, std::placeholders::_1));

        rclcpp::QoS lane_qos(rclcpp::KeepLast(1));
        lane_qos.reliable();
        lane_sub_ = create_subscription<kau_msgs::msg::KauPath>(
            "/lane/center", lane_qos,
            std::bind(&LocalPlannerNode::onLaneCenter, this, std::placeholders::_1));

        rclcpp::QoS obstacle_qos(rclcpp::KeepLast(1));
        obstacle_qos.best_effort().durability_volatile();
        obstacle_sub_ = create_subscription<kau_msgs::msg::ObstacleCircleArray>(
            "/perception/obstacles", obstacle_qos,
            std::bind(&LocalPlannerNode::onObstacles, this, std::placeholders::_1));

        steering_sub_ = create_subscription<std_msgs::msg::Float64>(
            "/steering", 10,
            std::bind(&LocalPlannerNode::onSteering, this, std::placeholders::_1));

        rclcpp::QoS local_qos(rclcpp::KeepLast(1));
        local_qos.reliable();
        local_path_pub_ = create_publisher<kau_msgs::msg::KauPath>(
            "/path/local", local_qos);

        viz_spacing_cm_ = get_parameter("viz_spacing_cm").as_double();
        if (get_parameter("publish_viz_path").as_bool())
        {
            const std::string viz_topic = get_parameter("viz_topic").as_string();

            // RViz Path 디스플레이 기본 QoS(reliable/volatile) 및 기존
            // /viz/path/global, /viz/path/global_avoidance 발행측과 맞춘다.
            // transient_local 로 두면 RViz 기본 설정에서 매칭되지 않는다.
            rclcpp::QoS viz_qos(rclcpp::KeepLast(1));
            viz_qos.reliable().durability_volatile();
            viz_path_pub_ = create_publisher<nav_msgs::msg::Path>(
                viz_topic, viz_qos);

            RCLCPP_INFO(get_logger(),
                "RViz 시각화 경로 발행: %s (nav_msgs/Path, spacing=%.1fcm)",
                viz_topic.c_str(), viz_spacing_cm_);
        }

        const double plan_hz = get_parameter("plan_hz").as_double();
        timer_ = create_wall_timer(
            std::chrono::duration<double>(1.0 / plan_hz),
            std::bind(&LocalPlannerNode::onTimer, this));

        RCLCPP_INFO(get_logger(),
            "local_planner_node 시작. plan_hz=%.1f map_frame=%s base_frame=%s",
            plan_hz, map_frame_.c_str(), base_frame_.c_str());
    }

private:
    void declareParameters()
    {
        declare_parameter<double>("plan_hz", 5.0);
        declare_parameter<std::string>("map_frame", "map");
        declare_parameter<std::string>("base_frame", "base_footprint");

        // RViz 시각화용 nav_msgs/Path 발행 (내용은 /path/local 과 동일,
        // 표현만 폴리라인 + m 단위). 제어단은 /path/local 을 쓴다.
        // 토픽/파라미터 이름은 kau_global_path, kau_lane_detection 의
        // /viz/path/* + viz_topic/viz_spacing_cm 규약을 그대로 따른다.
        declare_parameter<bool>("publish_viz_path", true);
        declare_parameter<std::string>("viz_topic", "/viz/path/local");
        declare_parameter<double>("viz_spacing_cm", 5.0);

        // amet_2026_track.yaml 절대 경로 (launch 가 kau_object_detection 의
        // share 경로로 채워준다) + sim->map 변환 (kau_global_path 확인값).
        declare_parameter<std::string>("track_yaml_path", "");
        declare_parameter<double>("sim_to_map_ox", 3.68);
        declare_parameter<double>("sim_to_map_oy", 1.39);
        declare_parameter<double>("sim_to_map_rot_deg", 0.0);

        // 차량 제원 (docs/경로_형식.md 부록 B 확정값).
        declare_parameter<double>("kappa_max_vehicle", 0.020221);   // 1/cm
        declare_parameter<double>("body_radius_cm", 11.0353);
        // 2026-08-25: hard-gate(road/obstacle) 전용 회전 사각형 차체
        // (후륜축 기준, 실제 치수). body_radius_cm(3분할 원 근사)는 candidate
        // 생성(offset 목표값 계산)에 그대로 남아있다.
        declare_parameter<double>("body_front_cm", 23.0);
        declare_parameter<double>("rear_overhang_cm", 5.0);
        declare_parameter<double>("half_width_cm", 10.0);
        declare_parameter<double>("wheelbase_cm", 18.0);

        // PlannerParams -- KAU_AMET_Test 세션 최종 튜닝값.
        declare_parameter<double>("l_plan", 300.0);
        declare_parameter<std::string>("ref_mode", "fuse");
        declare_parameter<double>("w_lane", 1.0);
        declare_parameter<double>("kappa_margin", 0.95);
        declare_parameter<int>("bound_depth", 2);
        declare_parameter<double>("cusp_guard", 0.6);
        declare_parameter<double>("obs_margin", 4.0);
        declare_parameter<double>("lane_gate", 60.0);
        declare_parameter<double>("preview", 450.0);
        declare_parameter<double>("w_obstacle", 4.0);
        declare_parameter<double>("w_ref", 1.0);
        declare_parameter<double>("w_kappa", 2.0);
        declare_parameter<double>("w_end", 0.6);
        declare_parameter<double>("w_continuity", 10.0);
        declare_parameter<double>("w_path_preview", 2.0);
        declare_parameter<double>("clear_target", 15.0);
        declare_parameter<double>("d_scale", 18.0);
    }

    PlannerParams loadPlannerParams()
    {
        PlannerParams p;
        p.l_plan = get_parameter("l_plan").as_double();
        p.ref_mode = get_parameter("ref_mode").as_string();
        p.w_lane = get_parameter("w_lane").as_double();
        p.kappa_margin = get_parameter("kappa_margin").as_double();
        p.bound_depth = static_cast<int>(get_parameter("bound_depth").as_int());
        p.cusp_guard = get_parameter("cusp_guard").as_double();
        p.obs_margin = get_parameter("obs_margin").as_double();
        p.lane_gate = get_parameter("lane_gate").as_double();
        p.preview = get_parameter("preview").as_double();
        p.w_obstacle = get_parameter("w_obstacle").as_double();
        p.w_ref = get_parameter("w_ref").as_double();
        p.w_kappa = get_parameter("w_kappa").as_double();
        p.w_end = get_parameter("w_end").as_double();
        p.w_continuity = get_parameter("w_continuity").as_double();
        p.w_path_preview = get_parameter("w_path_preview").as_double();
        p.clear_target = get_parameter("clear_target").as_double();
        p.d_scale = get_parameter("d_scale").as_double();
        return p;
    }

    // amet_2026_track.yaml 은 다른 노드(laser_scan_clusterer) 이름으로
    // 스코프돼 있어 표준 ROS2 parameter 파일 로딩(우리 노드 이름과 불일치)
    // 으로는 못 읽는다. yaml-cpp 로 직접 읽는다 (파일 자체는 복제하지
    // 않고 kau_object_detection 의 share 경로를 그대로 참조).
    RoadBoundary loadBoundary()
    {
        const std::string path = get_parameter("track_yaml_path").as_string();
        if (path.empty())
        {
            throw std::runtime_error(
                "track_yaml_path 파라미터가 비어있다 -- launch 파일에서 "
                "kau_object_detection 의 amet_2026_track.yaml 경로를 넘길 것");
        }
        const YAML::Node root = YAML::LoadFile(path);
        const YAML::Node params = root["laser_scan_clusterer"]["ros__parameters"];

        SimToMap transform;
        transform.ox = get_parameter("sim_to_map_ox").as_double();
        transform.oy = get_parameter("sim_to_map_oy").as_double();
        transform.rot_deg = get_parameter("sim_to_map_rot_deg").as_double();

        return kau::local_path_planner::makeRoadBoundary(
            params["track_outer_x"].as<std::vector<double>>(),
            params["track_outer_y"].as<std::vector<double>>(),
            params["track_inner_x"].as<std::vector<double>>(),
            params["track_inner_y"].as<std::vector<double>>(),
            transform);
    }

    void onGlobalPath(const kau_msgs::msg::KauPath::SharedPtr msg)
    {
        std::string reason;
        if (!kau::control::validateKauPath(*msg, reason))
        {
            RCLCPP_ERROR(get_logger(), "/path/global 무결성 검사 실패: %s",
                        reason.c_str());
            return;
        }
        if (msg->header.frame_id != map_frame_)
        {
            RCLCPP_WARN(get_logger(),
                "/path/global frame_id='%s' (기대: '%s') -- 변환 미구현, 그대로 사용",
                msg->header.frame_id.c_str(), map_frame_.c_str());
        }

        kau::control::Curve global_path = kau::control::curveFromKauPath(*msg);
        VehicleFootprint body_footprint{
            get_parameter("body_front_cm").as_double(),
            get_parameter("rear_overhang_cm").as_double(),
            get_parameter("half_width_cm").as_double()};
        planner_ = std::make_unique<LocalPlanner>(
            std::move(global_path), boundary_, std::vector<Obstacle>{},
            loadPlannerParams(), get_parameter("kappa_max_vehicle").as_double(),
            get_parameter("body_radius_cm").as_double(), body_footprint);
        RCLCPP_INFO(get_logger(),
            "/path/global 수신, LocalPlanner 생성 완료 (nseg=%d, length=%.1fcm)",
            static_cast<int>(msg->seg_length.size()), msg->total_length);
    }

    void onLaneCenter(const kau_msgs::msg::KauPath::SharedPtr msg)
    {
        std::string reason;
        if (!kau::control::validateKauPath(*msg, reason))
        {
            lane_curve_.reset();
            return;
        }
        if (msg->header.frame_id != map_frame_)
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                "/lane/center frame_id='%s' (기대: '%s') -- 변환 미구현, 그대로 사용",
                msg->header.frame_id.c_str(), map_frame_.c_str());
        }
        lane_curve_ = kau::control::curveFromKauPath(*msg);
        lane_confidence_ = msg->confidence;
    }

    void onObstacles(const kau_msgs::msg::ObstacleCircleArray::SharedPtr msg)
    {
        // STATUS_OK 가 아니면 obstacles 는 항상 비어있다는 게 메시지 자체의
        // 계약이다 (kau_msgs/msg/ObstacleCircleArray.msg) -- 여기서 이전
        // 주기 장애물을 유지하려 하지 않고, 매번 msg->obstacles 를 그대로
        // 신뢰한다 (실패 시 자연히 "장애물 없음"으로 취급됨).
        std::vector<Obstacle> obstacles;
        obstacles.reserve(msg->obstacles.size());
        for (const auto & o : msg->obstacles)
        {
            Obstacle ob;
            ob.center = Point2{
                static_cast<double>(o.center_x) * 100.0,
                static_cast<double>(o.center_y) * 100.0};
            ob.radius = static_cast<double>(o.radius) * 100.0;
            obstacles.push_back(ob);
        }
        latest_obstacles_ = std::move(obstacles);
    }

    void onSteering(const std_msgs::msg::Float64::SharedPtr msg)
    {
        current_steer_rad_ = msg->data;
    }

    void onTimer()
    {
        if (!planner_)
        {
            return;   // /path/global 아직 미수신
        }

        geometry_msgs::msg::TransformStamped tf;
        try
        {
            tf = tf_buffer_->lookupTransform(
                map_frame_, base_frame_, tf2::TimePointZero);
        }
        catch (const tf2::TransformException & ex)
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "TF %s -> %s 조회 실패: %s", map_frame_.c_str(),
                base_frame_.c_str(), ex.what());
            return;
        }

        const double x = tf.transform.translation.x * 100.0;   // m -> cm
        const double y = tf.transform.translation.y * 100.0;
        const double yaw = tf2::getYaw(tf.transform.rotation);

        // kappa0 = tan(delta) / L (문서 6.3 ⑤, Python: car.steer 와 동일 역할).
        const double wheelbase_cm = get_parameter("wheelbase_cm").as_double();
        const double kappa0 = std::tan(current_steer_rad_) / wheelbase_cm;

        planner_->updateObstacles(latest_obstacles_);

        const kau::control::Curve * lane_ptr =
            lane_curve_.has_value() ? &*lane_curve_ : nullptr;
        const auto result = planner_->plan(x, y, yaw, kappa0, lane_ptr, lane_confidence_);

        if (!result.path.has_value())
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "local path 없음 (status=%d) -- /path/local 미발행",
                static_cast<int>(result.status));
            return;
        }

        const uint8_t src = kau_msgs::msg::KauPath::SRC_LOCAL;
        auto msg = curveToKauPath(
            *result.path, src, map_frame_, now(),
            /*confidence=*/result.status == PlanStatus::kOk ? 1.0f : 0.5f,
            /*valid_length=*/0.0);
        local_path_pub_->publish(msg);

        if (viz_path_pub_)
        {
            viz_path_pub_->publish(curveToNavPath(
                *result.path, map_frame_, msg.header.stamp, viz_spacing_cm_));
        }

        if (result.status != PlanStatus::kOk)
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "local path status=%d clearance=%.2fcm kappa_max=%.5f",
                static_cast<int>(result.status), result.clearance,
                result.kappa_max);
        }
    }

    std::string map_frame_;
    std::string base_frame_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    RoadBoundary boundary_;
    std::unique_ptr<LocalPlanner> planner_;

    std::optional<kau::control::Curve> lane_curve_;
    float lane_confidence_ = 0.0f;
    std::vector<Obstacle> latest_obstacles_;
    double current_steer_rad_ = 0.0;
    double viz_spacing_cm_ = 5.0;

    rclcpp::Subscription<kau_msgs::msg::KauPath>::SharedPtr global_path_sub_;
    rclcpp::Subscription<kau_msgs::msg::KauPath>::SharedPtr lane_sub_;
    rclcpp::Subscription<kau_msgs::msg::ObstacleCircleArray>::SharedPtr obstacle_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr steering_sub_;
    rclcpp::Publisher<kau_msgs::msg::KauPath>::SharedPtr local_path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr viz_path_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LocalPlannerNode>());
    rclcpp::shutdown();
    return 0;
}
