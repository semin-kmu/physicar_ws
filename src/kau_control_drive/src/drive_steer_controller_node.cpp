// ====================================================================
// drive_steer_controller_node.cpp
//
// 조향 제어 -- /path/drive 하나가 유일한 입력이다. 그 토픽은
// kau_path_arbiter 가 장애물 유무에 따라 차선(/lane/center)과 회피
// 경로(/path/local) 중 하나를 골라 낸 것이다.
//
//     Ld  = clamp(k_v * v, ld_min, ld_max), valid_length 로 다시 절단
//     tgt = 최근접점에서 Ld 앞의 경로점
//     delta = atan(2 * L * sin(alpha) / Ld)  ->  clamp  ->  슬루 제한
//
// kau_control_lane/lane_steer_controller 와 알고리즘은 같고, 다른 점 셋:
//   1. 경로 소스가 /path/drive (소스별 timeout, drive_tracker.hpp)
//   2. 조향 슬루 제한을 실제로 건다 (steer_limiter.hpp -- 기존 두 제어기는
//      max_steer_rate 를 선언만 하고 쓰지 않는다)
//   3. 곡선 수학을 복사하지 않고 kau_control 헤더를 그대로 쓴다
//
// 경로가 끊기면 다른 소스로 내려가지 않고 조향 0 을 낸다.
// 단위: 내부 cm / deg, 발행 직전에만 rad.
// ====================================================================

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/float64.hpp>

#include "kau_msgs/msg/steer_debug.hpp"

#include "kau_control/params.hpp"
#include "kau_control/pure_pursuit.hpp"
#include "kau_control_drive/drive_tracker.hpp"
#include "kau_control_drive/steer_limiter.hpp"

namespace kau
{
namespace control_drive
{

// Ld 를 valid_length 안으로 자를 때 남겨 둘 여유 [cm].
// 경로 끝 정확히에 목표점을 두면 수치적으로 접선이 불안정하다.
constexpr double kLdValidMarginCm = 5.0;

class DriveSteerController : public rclcpp::Node
{
public:
    DriveSteerController()
    : rclcpp::Node("drive_steer_controller"),
      tracker_(this),
      limiter_(SteerLimiterParams{})
    {
        control_hz_ = declare_parameter<double>("control_hz", 50.0);
        vehicle_.wheelbase = declare_parameter<double>("vehicle.wheelbase_cm", 18.0);
        vehicle_.max_steer = declare_parameter<double>("vehicle.max_steer_deg", 20.0);
        vehicle_.max_steer_rate =
            declare_parameter<double>("vehicle.max_steer_rate_dps", 600.0);
        ctrl_.k_v = declare_parameter<double>("controller.k_v", 0.5);
        ctrl_.ld_min = declare_parameter<double>("controller.ld_min_cm", 25.0);
        ctrl_.ld_max = declare_parameter<double>("controller.ld_max_cm", 80.0);

        SteerLimiterParams lp;
        lp.max_steer_deg = vehicle_.max_steer;
        lp.max_steer_rate_dps = vehicle_.max_steer_rate;
        lp.transition_rate_dps =
            declare_parameter<double>("controller.transition_rate_dps", 150.0);
        lp.transition_sec =
            declare_parameter<double>("controller.transition_sec", 0.4);
        limiter_ = SteerLimiter(lp);

        speed_topic_ = declare_parameter<std::string>("speed_topic", "/speed");
        viz_enabled_ = declare_parameter<bool>("viz.enabled", true);
        debug_enabled_ = declare_parameter<bool>("debug.enabled", true);

        steer_pub_ = create_publisher<std_msgs::msg::Float64>("/steering", 10);
        if (viz_enabled_)
        {
            viz_path_pub_ = create_publisher<nav_msgs::msg::Path>("/viz/path/tracked", 1);
            viz_target_pub_ = create_publisher<geometry_msgs::msg::PointStamped>(
                "/viz/path/lookahead", 1);
        }
        if (debug_enabled_)
        {
            debug_pub_ = create_publisher<kau_msgs::msg::SteerDebug>(
                declare_parameter<std::string>("debug.topic", "/debug/steer"),
                rclcpp::QoS(1).best_effort());
        }

        // Ld 가 속도에 비례하므로 현재 속도 명령을 본다.
        // 첫 메시지 전에는 v=0 -> Ld=ld_min 이라 안전한 쪽으로 붙는다.
        speed_sub_ = create_subscription<std_msgs::msg::Float64>(
            speed_topic_, 10,
            [this](std_msgs::msg::Float64::SharedPtr m) { v_ = m->data; });

        last_tick_ = now();
        timer_ = create_wall_timer(
            std::chrono::duration<double>(1.0 / control_hz_),
            [this]() { onTimer(); });

        RCLCPP_INFO(get_logger(),
            "drive_steer_controller 시작. %.0fHz, 슬루 %.0fdeg/s "
            "(전환 직후 %.0fdeg/s x %.2fs)",
            control_hz_, vehicle_.max_steer_rate, lp.transition_rate_dps,
            lp.transition_sec);
    }

private:
    void publishSteer(double deg)
    {
        std_msgs::msg::Float64 m;
        m.data = deg * M_PI / 180.0;   // 발행은 rad
        steer_pub_->publish(m);
    }

    void onTimer()
    {
        const rclcpp::Time t = now();
        const double dt = std::max(0.0, (t - last_tick_).seconds());
        last_tick_ = t;

        const bool src_changed = tracker_.consumeSourceChanged();
        const TrackResult r = tracker_.update();
        if (!r.ok)
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "조향 0 발행: %s", r.reason.c_str());
            publishSteer(limiter_.releaseToZero());
            publishDebug(t, r, 0.0, 0.0);
            return;
        }

        const Curve * cv = tracker_.curve();
        double ld = kau::control::pure_pursuit::lookaheadDistance(v_, ctrl_);

        // valid_length 안으로 자른다. 그 너머는 발행측이 "믿지 말라" 고
        // 표시한 구간이라 목표점을 두면 안 된다 (KauPath.valid_length 계약).
        const double vl = tracker_.valid_length();
        if (vl > 0.0)
        {
            const double usable = vl - r.s - kLdValidMarginCm;
            if (usable < ctrl_.ld_min)
            {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                    "믿을 수 있는 앞 구간이 %.1fcm 뿐 (ld_min %.1f) -- 조향 0.",
                    usable, ctrl_.ld_min);
                publishSteer(limiter_.releaseToZero());
                publishDebug(t, r, 0.0, 0.0);
                return;
            }
            ld = std::min(ld, usable);
        }

        const Point2 tgt = cv->point(cv->wrapS(r.s + ld));
        const double want = kau::control::pure_pursuit::steerCommand(
            r.x, r.y, r.yaw, tgt.x, tgt.y, ld, vehicle_);
        const double cmd = limiter_.apply(want, dt, src_changed, t.seconds());
        publishSteer(cmd);

        if (viz_target_pub_)
        {
            geometry_msgs::msg::PointStamped p;
            p.header.stamp = t;
            p.header.frame_id = tracker_.frame();
            p.point.x = tgt.x / 100.0;
            p.point.y = tgt.y / 100.0;
            viz_target_pub_->publish(p);
        }
        publishDebug(t, r, cmd, ld);
    }

    void publishDebug(const rclcpp::Time & t, const TrackResult & r,
                      double cmd_deg, double ld)
    {
        if (!debug_pub_)
        {
            return;
        }
        kau_msgs::msg::SteerDebug d;
        d.header.stamp = t;
        d.header.frame_id = tracker_.frame();
        d.tracking_ok = r.ok;
        d.path_source = tracker_.source();
        d.cmd_steer_deg = cmd_deg;
        d.lookahead_cm = ld;
        d.heading_error_rad = r.heading_error;
        d.cross_track_cm = r.cte;
        d.s_cm = r.s;
        debug_pub_->publish(d);
    }

    DriveTracker tracker_;
    SteerLimiter limiter_;
    kau::control::VehicleParams vehicle_;
    kau::control::ControllerParams ctrl_;

    double control_hz_ = 50.0;
    double v_ = 0.0;
    bool viz_enabled_ = true;
    bool debug_enabled_ = true;
    std::string speed_topic_;
    rclcpp::Time last_tick_;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr steer_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr viz_path_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr viz_target_pub_;
    rclcpp::Publisher<kau_msgs::msg::SteerDebug>::SharedPtr debug_pub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr speed_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace control_drive
}  // namespace kau

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<kau::control_drive::DriveSteerController>());
    rclcpp::shutdown();
    return 0;
}
