// ====================================================================
// steer_controller_node.cpp
//
// 조향 제어. 경로를 보고 Pure Pursuit 로 /steering [rad] 을 낸다.
//
//     Ld  = clamp(k_v * v, ld_min, ld_max)         v 는 /speed 구독
//     tgt = 최근접점에서 Ld 앞의 경로점
//     delta = atan(2 * L * sin(alpha) / Ld)  ->  ±max_steer clamp
//
// 속도는 speed_controller 가 따로 낸다. 이 노드는 조향만 책임진다.
// 단위: 내부 cm / deg, 발행 직전에만 rad.
//
// 발행: /steering (제어) · /viz/path/{tracked,lookahead} (RViz)
//       /debug/steer (kau_msgs/SteerDebug, 디버깅 GUI 전용)
// ====================================================================

#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/float64.hpp>

#include "kau_msgs/msg/steer_debug.hpp"

#include "kau_control/params.hpp"
#include "kau_control/path_tracker.hpp"
#include "kau_control/pure_pursuit.hpp"


namespace kau
{
namespace control
{

class SteerController : public rclcpp::Node
{
public:
    SteerController()
    : rclcpp::Node("steer_controller"),
      tracker_(this)
    {
        declare_parameter<double>("control_hz", 50.0);
        declare_parameter<double>("vehicle.wheelbase_cm", 18.0);
        declare_parameter<double>("vehicle.max_steer_deg", 20.0);
        declare_parameter<double>("controller.k_v", 0.5);
        declare_parameter<double>("controller.ld_min_cm", 30.0);
        declare_parameter<double>("controller.ld_max_cm", 175.0);
        declare_parameter<std::string>("speed_topic", "/speed");
        declare_parameter<bool>("viz.enabled", true);
        declare_parameter<int>("viz.samples_per_seg", 20);

        // 디버깅 GUI 전용 발행. 끄면 publisher 자체를 만들지 않는다.
        declare_parameter<bool>("debug.enabled", true);
        declare_parameter<std::string>("debug.topic", "/debug/steer");

        load();


        steer_pub_ = create_publisher<std_msgs::msg::Float64>("/steering", 10);

        // RViz 는 KauPath 를 못 그린다 -> 시각화는 별도 topic 으로 분리 발행
        viz_path_pub_ =
            create_publisher<nav_msgs::msg::Path>("/viz/path/tracked", 1);

        viz_target_pub_ =
            create_publisher<geometry_msgs::msg::PointStamped>(
                "/viz/path/lookahead", 1);

        // 관측 전용이므로 BEST_EFFORT. 제어 tick 이 GUI 구독자 때문에
        // 재전송 비용을 떠안지 않게 한다 (docs/09 section 10).
        if (get_parameter("debug.enabled").as_bool())
        {
            debug_pub_ = create_publisher<kau_msgs::msg::SteerDebug>(
                get_parameter("debug.topic").as_string(),
                rclcpp::QoS(1).best_effort());
        }

        // Ld 가 속도에 비례하므로 현재 속도 명령을 본다.
        // 첫 메시지 전에는 v=0 -> Ld=ld_min 이라 안전한 쪽으로 붙는다.
        speed_sub_ = create_subscription<std_msgs::msg::Float64>(
            get_parameter("speed_topic").as_string(), 10,
            [this](std_msgs::msg::Float64::SharedPtr m)
            {
                v_ = m->data;
            });


        const auto period = std::chrono::duration<double>(
            1.0 / get_parameter("control_hz").as_double());

        timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period),
            [this]()
            {
                onTimer();
            });

        // ros2 param set 으로 튜닝값을 바꾸면 즉시 반영한다.
        // topic / control_hz 는 굳었으므로 바꾸려면 노드를 다시 띄운다.
        param_cb_ = add_on_set_parameters_callback(
            [this](const std::vector<rclcpp::Parameter> &)
            {
                rcl_interfaces::msg::SetParametersResult r;

                r.successful = true;

                load();

                tracker_.load();

                return r;
            });

        RCLCPP_INFO(
            get_logger(), "steer_controller 시작. Ld=%.0f~%.0f cm k_v=%.2f",
            ctrl_.ld_min, ctrl_.ld_max, ctrl_.k_v);
    }

private:
    void load()
    {
        vehicle_.wheelbase =
            get_parameter("vehicle.wheelbase_cm").as_double();
        vehicle_.max_steer =
            get_parameter("vehicle.max_steer_deg").as_double();

        ctrl_.k_v    = get_parameter("controller.k_v").as_double();
        ctrl_.ld_min = get_parameter("controller.ld_min_cm").as_double();
        ctrl_.ld_max = get_parameter("controller.ld_max_cm").as_double();

        viz_enabled_ = get_parameter("viz.enabled").as_bool();
        viz_samples_ =
            static_cast<int>(get_parameter("viz.samples_per_seg").as_int());
    }

    void onTimer()
    {
        const TrackResult r = tracker_.update();

        if (!r.ok || r.goal)
        {
            stop(r.reason);

            return;
        }

        publishVizPath();


        double ld = pure_pursuit::lookaheadDistance(v_, ctrl_);

        // Lane Detection 은 실제로 본 구간까지만 valid_length 로 알려준다.
        // 그 너머는 외삽이므로 LookAhead 점을 거기 두면 안 된다.
        // (그 외 source 는 valid_length = 0 이라 이 절이 걸리지 않는다)
        const double vl = tracker_.validLength();

        if (vl > 0.0)
        {
            const double usable = vl - r.s - LD_VALID_MARGIN_CM;

            if (usable < ctrl_.ld_min)
            {
                stop("관측 구간이 짧아 LookAhead 를 둘 곳이 없다");

                return;
            }

            ld = std::min(ld, usable);
        }


        const Curve & cv = *tracker_.curve();

        const Point2 tgt = cv.point(cv.lookahead(r.s, ld));

        // 조향 포화는 deg 가 +-max_steer 에 붙는 것으로 드러난다. clamp 전
        // 원출력은 밖으로 내보내지 않는다 (SteerDebug.msg 주석 참고).
        const double deg = std::clamp(
            pure_pursuit::steerCommand(
                r.x, r.y, r.yaw, tgt.x, tgt.y, ld, vehicle_),
            -vehicle_.max_steer, vehicle_.max_steer);

        publishSteering(deg2rad(deg));

        publishVizTarget(tgt);

        publishDebug(r, deg, ld);

        RCLCPP_DEBUG_THROTTLE(
            get_logger(), *get_clock(), 500,
            "s=%.1f cte=%.1f cm Ld=%.0f steer=%.1f deg (v=%.2f)",
            r.s, r.cte, ld, deg, v_);
    }

    // 어떤 경로로 빠져나가든 0 을 발행한다. 조용히 return 하면 차량이
    // 직전 조향각을 그대로 물고 간다.
    void stop(const std::string & reason)
    {
        publishSteering(0.0);

        // 실패 tick 에도 debug 는 계속 내보낸다. 발행이 멈추면 GUI 가
        // "노드가 죽음" 과 "추종 불가" 를 구분할 수 없다.
        publishDebugIdle();

        if (!reason.empty())
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000, "조향 0: %s",
                reason.c_str());
        }
    }

    void publishSteering(double rad)
    {
        std_msgs::msg::Float64 m;

        m.data = rad;

        steer_pub_->publish(m);
    }

    // 경로가 바뀔 때만 다시 그린다 (50 Hz 로 Path 를 쏘면 낭비다).
    void publishVizPath()
    {
        if (!viz_enabled_ || viz_id_ == tracker_.pathId())
        {
            return;
        }

        viz_id_ = tracker_.pathId();

        nav_msgs::msg::Path p;

        p.header.frame_id = tracker_.frame();

        p.header.stamp = now();

        for (const Point2 & q : tracker_.curve()->sample(viz_samples_))
        {
            geometry_msgs::msg::PoseStamped ps;

            ps.header = p.header;

            ps.pose.position.x = q.x / CM_PER_M;

            ps.pose.position.y = q.y / CM_PER_M;

            ps.pose.orientation.w = 1.0;

            p.poses.push_back(ps);
        }

        viz_path_pub_->publish(p);
    }

    // 값은 전부 위에서 이미 구한 것이다. 여기서 새로 계산하지 않는다.
    void publishDebug(const TrackResult & r, double cmd_deg, double ld)
    {
        if (!debug_pub_)
        {
            return;
        }

        kau_msgs::msg::SteerDebug m;

        m.header.frame_id = tracker_.frame();

        m.header.stamp = now();

        m.tracking_ok = true;

        m.path_source = tracker_.source();

        m.cmd_steer_deg = cmd_deg;

        m.lookahead_cm = ld;

        m.heading_error_rad = r.head_err;

        m.cross_track_cm = r.cte;

        m.s_cm = r.s;

        debug_pub_->publish(m);
    }

    // 추종 불가 tick. 수치는 전부 0 이고 tracking_ok 로만 구분한다.
    void publishDebugIdle()
    {
        if (!debug_pub_)
        {
            return;
        }

        kau_msgs::msg::SteerDebug m;

        m.header.frame_id = tracker_.frame();

        m.header.stamp = now();

        m.tracking_ok = false;

        m.path_source = tracker_.source();

        debug_pub_->publish(m);
    }

    void publishVizTarget(const Point2 & tgt)
    {
        if (!viz_enabled_)
        {
            return;
        }

        geometry_msgs::msg::PointStamped m;

        m.header.frame_id = tracker_.frame();

        m.header.stamp = now();

        m.point.x = tgt.x / CM_PER_M;

        m.point.y = tgt.y / CM_PER_M;

        viz_target_pub_->publish(m);
    }


    static constexpr double LD_VALID_MARGIN_CM = 5.0;

    PathTracker      tracker_;
    VehicleParams    vehicle_;
    ControllerParams ctrl_;

    double   v_           = 0.0;   // m/s, /speed 로 받은 현재 명령
    bool     viz_enabled_ = true;
    int      viz_samples_ = 20;
    uint32_t viz_id_      = 0;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr steer_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr    viz_path_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr
        viz_target_pub_;

    // debug.enabled=false 면 null 로 남는다.
    rclcpp::Publisher<kau_msgs::msg::SteerDebug>::SharedPtr debug_pub_;

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr speed_sub_;

    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
        param_cb_;
};

}  // namespace control
}  // namespace kau


int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    // 설정 오류(path.mode 등)는 노드 생성자가 던진다. 그대로 두면 terminate
    // 로 죽어 종료 코드가 134 가 되고, supervisor 의 사망 판정이 오염된다
    // (kau_state_machine 의 state_machine.py main() 과 같은 취지).
    // 이유를 남기고 1 로 끝낸다.
    try
    {
        rclcpp::spin(std::make_shared<kau::control::SteerController>());
    }
    catch (const std::exception & e)
    {
        RCLCPP_FATAL(
            rclcpp::get_logger("steer_controller"), "기동 실패: %s", e.what());

        rclcpp::shutdown();

        return 1;
    }

    rclcpp::shutdown();

    return 0;
}
