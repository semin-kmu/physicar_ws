// ====================================================================
// local_planner_node.cpp (lane-only 재설계, 2026-08-25)
//
// map/global path/localization 완전 배제. 구독: /lane/left, /lane/right,
// /lane/center (KauPath, base_link), /perception/obstacles
// (kau_msgs/ObstacleCircleArray, base_link), odom_topic(nav_msgs/Odometry,
// hold-last/continuity/앵커 재정렬용 상대 오도메트리 -- 절대 위치가 아니라
// 두 시각 사이의 상대 변위만 쓴다). 발행: /path/local (KauPath, base_link).
//
// TF/map 을 전혀 안 쓴다 -- 옛 kau_local_path_planner(map+global path 기반)
// 는 건드리지 않고 이 패키지만 별도로 둔 이유이기도 하다.
// ====================================================================

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "kau_control/kau_path.hpp"
#include "kau_msgs/msg/kau_path.hpp"
#include "kau_msgs/msg/obstacle_circle_array.hpp"
#include "kau_local_path_planner_lane/local_planner.hpp"

namespace
{

using kau::local_path_planner_lane::Frame;
using kau::local_path_planner_lane::LocalPlanner;
using kau::local_path_planner_lane::Obstacle;
using kau::local_path_planner_lane::OdomDelta;
using kau::local_path_planner_lane::PlannerParams;
using kau::local_path_planner_lane::PlanStatus;
using kau::local_path_planner_lane::Point2;
using kau::local_path_planner_lane::VehicleFootprint;
using kau::local_path_planner_lane::WheelFootprint;

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
        base_frame_ = get_parameter("base_frame").as_string();

        const double rear_axle_offset =
            get_parameter("rear_axle_offset_cm").as_double();
        VehicleFootprint body_footprint{
            get_parameter("body_front_cm").as_double(),
            get_parameter("rear_overhang_cm").as_double(),
            get_parameter("half_width_cm").as_double(),
            rear_axle_offset};
        WheelFootprint wheels{
            rear_axle_offset,
            get_parameter("wheelbase_cm").as_double(),
            get_parameter("track_width_cm").as_double(),
            get_parameter("wheel_width_cm").as_double(),
            static_cast<int>(get_parameter("min_wheels_on").as_int())};

        planner_ = std::make_unique<LocalPlanner>(
            loadPlannerParams(), get_parameter("kappa_max_vehicle").as_double(),
            get_parameter("body_radius_cm").as_double(), body_footprint, wheels);

        rclcpp::QoS lane_qos(rclcpp::KeepLast(1));
        lane_qos.reliable();
        lane_center_sub_ = create_subscription<kau_msgs::msg::KauPath>(
            "/lane/center", lane_qos,
            std::bind(&LocalPlannerNode::onLaneCenter, this, std::placeholders::_1));
        lane_left_sub_ = create_subscription<kau_msgs::msg::KauPath>(
            "/lane/left", lane_qos,
            std::bind(&LocalPlannerNode::onLaneLeft, this, std::placeholders::_1));
        lane_right_sub_ = create_subscription<kau_msgs::msg::KauPath>(
            "/lane/right", lane_qos,
            std::bind(&LocalPlannerNode::onLaneRight, this, std::placeholders::_1));

        rclcpp::QoS obstacle_qos(rclcpp::KeepLast(1));
        obstacle_qos.best_effort().durability_volatile();
        obstacle_sub_ = create_subscription<kau_msgs::msg::ObstacleCircleArray>(
            "/perception/obstacles", obstacle_qos,
            std::bind(&LocalPlannerNode::onObstacles, this, std::placeholders::_1));

        const std::string odom_topic = get_parameter("odom_topic").as_string();
        rclcpp::QoS odom_qos(rclcpp::KeepLast(1));
        odom_qos.best_effort();
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            odom_topic, odom_qos,
            std::bind(&LocalPlannerNode::onOdom, this, std::placeholders::_1));

        rclcpp::QoS local_qos(rclcpp::KeepLast(1));
        local_qos.reliable();
        local_path_pub_ = create_publisher<kau_msgs::msg::KauPath>(
            "/path/local", local_qos);

        viz_spacing_cm_ = get_parameter("viz_spacing_cm").as_double();
        if (get_parameter("publish_viz_path").as_bool())
        {
            const std::string viz_topic = get_parameter("viz_topic").as_string();
            rclcpp::QoS viz_qos(rclcpp::KeepLast(1));
            viz_qos.reliable().durability_volatile();
            viz_path_pub_ = create_publisher<nav_msgs::msg::Path>(viz_topic, viz_qos);
        }

        const double plan_hz = get_parameter("plan_hz").as_double();
        timer_ = create_wall_timer(
            std::chrono::duration<double>(1.0 / plan_hz),
            std::bind(&LocalPlannerNode::onTimer, this));

        RCLCPP_INFO(get_logger(),
            "local_planner_node(lane-only) 시작. plan_hz=%.1f base_frame=%s "
            "odom_topic=%s (map/global path 배제)",
            plan_hz, base_frame_.c_str(), odom_topic.c_str());
    }

private:
    void declareParameters()
    {
        declare_parameter<double>("plan_hz", 5.0);
        declare_parameter<std::string>("base_frame", "base_footprint");
        // 2026-08-25: "/odometry/filtered" -> "/odom".
        //
        // 그 토픽은 이 시스템에 **존재하지 않는다** (kau_gui/README.md 142:
        // "`/odometry/filtered` 는 존재하지 않는다"). 실제 발행자는
        // physicar_bringup 의 ekf_filter_node 이고 토픽은 `/odom` 이다.
        //
        // 조용히 안 뜨는 것으로 끝나지 않는다. odom 이 없으면 plan() 의
        // odom_delta 가 항상 nullopt 라 previous_path_ 가 한 번도 재정렬되지
        // 않고, 그러면
        //   - computeAnchor 의 최근접점이 매 틱 s~0 에 머물러 kappa0 가
        //     자기가 직전에 쓴 값을 다시 읽는다 -> 0 에서 영원히 안 움직인다
        //     (코너 한복판에서도 "지금 직진 중"으로 계획한다)
        //   - continuity 항(w_continuity=10.0, 최대 가중치)이 옛 ego frame
        //     기준으로 채점돼 매 틱 코너 바깥으로 끌어당긴다
        // 합성 원호 폐루프 실측(완벽 추종 가정, 40틱): R=100cm 코너에서
        // ok 7/40 · 횡오차 206cm 발산 -> odom 연결만으로 40/40 · 19cm.
        declare_parameter<std::string>("odom_topic", "/odom");

        declare_parameter<bool>("publish_viz_path", true);
        declare_parameter<std::string>("viz_topic", "/viz/path/local");
        declare_parameter<double>("viz_spacing_cm", 5.0);

        declare_parameter<double>("kappa_max_vehicle", 0.020221);
        declare_parameter<double>("body_radius_cm", 11.0353);
        declare_parameter<double>("body_front_cm", 23.0);
        declare_parameter<double>("rear_overhang_cm", 5.0);
        declare_parameter<double>("half_width_cm", 10.0);
        declare_parameter<double>("wheelbase_cm", 18.0);

        declare_parameter<double>("rear_axle_offset_cm", -9.0);
        declare_parameter<double>("track_width_cm", 16.0);
        declare_parameter<double>("wheel_width_cm", 3.5);
        declare_parameter<int>("min_wheels_on", 1);

        declare_parameter<double>("l_plan", 300.0);
        declare_parameter<double>("kappa_margin", 0.95);
        declare_parameter<int>("bound_depth", 2);
        declare_parameter<double>("cusp_guard", 0.6);
        declare_parameter<double>("obs_margin", 4.0);
        declare_parameter<double>("w_obstacle", 4.0);
        declare_parameter<double>("w_ref", 1.0);
        declare_parameter<double>("w_kappa", 2.0);
        declare_parameter<double>("w_end", 0.6);
        declare_parameter<double>("w_continuity", 10.0);
        declare_parameter<double>("w_road", 6.0);
        declare_parameter<double>("kappa_barrier_knee", 0.60);
        declare_parameter<double>("kappa_barrier_cap", 0.0);   // 0 = 꺼짐
        declare_parameter<double>("clear_target", 15.0);
        declare_parameter<double>("d_scale", 18.0);

        declare_parameter<double>("anchor_blend_lo_cm", 5.0);
        declare_parameter<double>("anchor_blend_hi_cm", 15.0);
        declare_parameter<double>("anchor_max_age_sec", 0.5);
        declare_parameter<double>("validated_horizon_cm", 145.0);
    }

    PlannerParams loadPlannerParams()
    {
        PlannerParams p;
        p.l_plan = get_parameter("l_plan").as_double();
        p.kappa_margin = get_parameter("kappa_margin").as_double();
        p.bound_depth = static_cast<int>(get_parameter("bound_depth").as_int());
        p.cusp_guard = get_parameter("cusp_guard").as_double();
        p.obs_margin = get_parameter("obs_margin").as_double();
        p.w_obstacle = get_parameter("w_obstacle").as_double();
        p.w_ref = get_parameter("w_ref").as_double();
        p.w_kappa = get_parameter("w_kappa").as_double();
        p.w_end = get_parameter("w_end").as_double();
        p.w_continuity = get_parameter("w_continuity").as_double();
        p.w_road = get_parameter("w_road").as_double();
        p.kappa_barrier_knee = get_parameter("kappa_barrier_knee").as_double();
        p.kappa_barrier_cap = get_parameter("kappa_barrier_cap").as_double();
        p.clear_target = get_parameter("clear_target").as_double();
        p.d_scale = get_parameter("d_scale").as_double();
        p.anchor_blend_lo_cm = get_parameter("anchor_blend_lo_cm").as_double();
        p.anchor_blend_hi_cm = get_parameter("anchor_blend_hi_cm").as_double();
        p.anchor_max_age_sec = get_parameter("anchor_max_age_sec").as_double();
        p.validated_horizon_cm = get_parameter("validated_horizon_cm").as_double();
        return p;
    }

    static std::optional<kau::control::Curve> toCurve(
        const kau_msgs::msg::KauPath::SharedPtr & msg)
    {
        if (!msg)
        {
            return std::nullopt;
        }
        std::string reason;
        if (!kau::control::validateKauPath(*msg, reason))
        {
            return std::nullopt;
        }
        return kau::control::curveFromKauPath(*msg);
    }

    void onLaneCenter(const kau_msgs::msg::KauPath::SharedPtr msg) { lane_center_ = msg; }
    void onLaneLeft(const kau_msgs::msg::KauPath::SharedPtr msg) { lane_left_ = msg; }
    void onLaneRight(const kau_msgs::msg::KauPath::SharedPtr msg) { lane_right_ = msg; }

    void onObstacles(const kau_msgs::msg::ObstacleCircleArray::SharedPtr msg)
    {
        // STATUS_OK 가 아니면 obstacles 는 항상 비어있다는 게 메시지 계약이다
        // (kau_msgs/ObstacleCircleArray.msg) -- 그대로 신뢰한다.
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

    void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        latest_odom_ = msg;
    }

    void onTimer()
    {
        // odom_delta: 직전 plan() 시각의 odom pose 기준 이번 odom pose 상대
        // 변위 (map 아니라 odom frame 내 상대량). 옵션이라 odom 이 없으면
        // 무보정으로 그대로 동작한다(재계획 주기가 짧다는 근사).
        std::optional<OdomDelta> odom_delta;
        if (!latest_odom_)
        {
            // odom 이 없으면 previous_path_ 가 재정렬되지 않아 kappa0 가 0 에
            // 고정되고(코너를 직진으로 계획한다) continuity 항이 옛 frame 을
            // 기준으로 채점된다 -- 곡선에서 횡오차가 발산한다. 조용히 넘어가면
            // 안 되는 상태라 토픽 이름과 함께 계속 경고한다.
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                "odom(%s) 수신 없음 -- 경로 재정렬/kappa0 갱신이 멈춘다 "
                "(곡선에서 횡오차 발산). 토픽 이름을 확인할 것.",
                get_parameter("odom_topic").as_string().c_str());
        }
        if (latest_odom_)
        {
            const double x = latest_odom_->pose.pose.position.x * 100.0;
            const double y = latest_odom_->pose.pose.position.y * 100.0;
            const double yaw = tf2::getYaw(latest_odom_->pose.pose.orientation);
            if (have_prev_odom_)
            {
                const double dxw = x - prev_odom_x_;
                const double dyw = y - prev_odom_y_;
                const double c = std::cos(-prev_odom_yaw_);
                const double s = std::sin(-prev_odom_yaw_);
                OdomDelta d;
                d.dx = dxw * c - dyw * s;
                d.dy = dxw * s + dyw * c;
                d.dyaw = kau::control::wrapPi(yaw - prev_odom_yaw_);
                odom_delta = d;
            }
            prev_odom_x_ = x;
            prev_odom_y_ = y;
            prev_odom_yaw_ = yaw;
            have_prev_odom_ = true;
        }

        planner_->updateObstacles(latest_obstacles_);

        const auto left = toCurve(lane_left_);
        const auto right = toCurve(lane_right_);
        const auto center = toCurve(lane_center_);

        const auto result = planner_->plan(
            left ? &*left : nullptr, right ? &*right : nullptr,
            center ? &*center : nullptr, odom_delta, now().seconds());

        if (!result.path.has_value())
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "local path 없음 (status=%d) -- /path/local 미발행",
                static_cast<int>(result.status));
            return;
        }

        const uint8_t src = kau_msgs::msg::KauPath::SRC_LOCAL;
        auto msg = curveToKauPath(
            *result.path, src, base_frame_, now(),
            result.status == PlanStatus::kOk ? 1.0f : 0.5f,
            result.valid_length_cm);
        local_path_pub_->publish(msg);

        if (viz_path_pub_)
        {
            viz_path_pub_->publish(curveToNavPath(
                *result.path, base_frame_, msg.header.stamp, viz_spacing_cm_));
        }

        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
            "plan st=%d d=%+.1f k0=%+.5f kmax=%.5f obs=%+.1f wheels_on=%d "
            "road_full=%+.1f road_cmt=%+.1f e=%.1f a=%.2f vl=%.0f alive=%d "
            "%.1fms",
            static_cast<int>(result.status), result.chosen_offset,
            result.kappa0, result.kappa_max, result.clearance,
            result.min_wheels_on, result.road_clear_full,
            result.road_clear_committed, result.anchor_e_cm,
            result.anchor_alpha, result.valid_length_cm, result.alive,
            result.calc_ms);

        if (result.min_wheels_on < 4)
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "바퀴 노면 이탈: %d/4 남음", result.min_wheels_on);
        }
        if (result.status != PlanStatus::kOk)
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "local path status=%d clearance=%.2fcm kappa_max=%.5f",
                static_cast<int>(result.status), result.clearance,
                result.kappa_max);
        }
    }

    std::string base_frame_;
    std::unique_ptr<LocalPlanner> planner_;

    kau_msgs::msg::KauPath::SharedPtr lane_center_;
    kau_msgs::msg::KauPath::SharedPtr lane_left_;
    kau_msgs::msg::KauPath::SharedPtr lane_right_;
    std::vector<Obstacle> latest_obstacles_;
    nav_msgs::msg::Odometry::SharedPtr latest_odom_;
    bool have_prev_odom_ = false;
    double prev_odom_x_ = 0.0;
    double prev_odom_y_ = 0.0;
    double prev_odom_yaw_ = 0.0;
    double viz_spacing_cm_ = 5.0;

    rclcpp::Subscription<kau_msgs::msg::KauPath>::SharedPtr lane_center_sub_;
    rclcpp::Subscription<kau_msgs::msg::KauPath>::SharedPtr lane_left_sub_;
    rclcpp::Subscription<kau_msgs::msg::KauPath>::SharedPtr lane_right_sub_;
    rclcpp::Subscription<kau_msgs::msg::ObstacleCircleArray>::SharedPtr obstacle_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
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
