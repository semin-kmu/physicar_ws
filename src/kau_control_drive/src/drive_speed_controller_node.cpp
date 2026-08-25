// ====================================================================
// drive_speed_controller_node.cpp
//
// 속도 제어 -- /path/drive 하나가 유일한 입력이다.
//
//     kappa_win = 자차 앞 [look_min, look] 구간의 최대 곡률
//     v_target  = SpeedParams::target(kappa_win)        (곡률 클수록 느리게)
//     rate limit + EMA  ->  /speed
//
// kau_control_lane/lane_speed_controller 와 알고리즘은 같다. 다른 점:
// 경로 소스가 /path/drive 이고, 곡률 창을 valid_length 안으로 자른다
// (믿지 말라고 표시된 구간의 곡률로 감속하면 근거 없는 감속이 된다).
//
// 속도 되먹임(PID)은 붙이지 않았다 -- kau_control 의 speed PID 는
// `/odometry/filtered` 를 feedback_topic 기본값으로 쓰는데 그 토픽은 이
// 시스템에 존재하지 않는다 (kau_gui/README.md 142). 되먹임 없이 목표속도를
// 그대로 내는 개루프이며, 붙이려면 feedback_topic 을 /odom 으로 두고
// 여기에 PID 를 추가할 것 (kau_control/pid.hpp 재사용).
//
// 경로가 끊기면 0 을 낸다.
// ====================================================================

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

#include "kau_control/params.hpp"
#include "kau_control_drive/drive_tracker.hpp"

namespace kau
{
namespace control_drive
{

class DriveSpeedController : public rclcpp::Node
{
public:
    DriveSpeedController()
    : rclcpp::Node("drive_speed_controller"),
      tracker_(this)
    {
        control_hz_ = declare_parameter<double>("control_hz", 50.0);
        vehicle_.wheelbase = declare_parameter<double>("vehicle.wheelbase_cm", 18.0);
        vehicle_.max_steer = declare_parameter<double>("vehicle.max_steer_deg", 20.0);

        speed_.v_min = declare_parameter<double>("speed.v_min", 0.2);
        speed_.v_max = declare_parameter<double>("speed.v_max", 0.5);
        speed_.look_min = declare_parameter<double>("speed.look_min_cm", 20.0);
        speed_.look_max = declare_parameter<double>("speed.look_max_cm", 150.0);
        speed_.min_span = declare_parameter<double>("speed.min_span_cm", 20.0);
        speed_.margin = declare_parameter<double>("speed.margin", 0.8);
        speed_.accel_max = declare_parameter<double>("speed.accel_max", 1.0);
        speed_.decel_max = declare_parameter<double>("speed.decel_max", 2.0);
        speed_.ema_tau = declare_parameter<double>("speed.ema_tau", 0.4);

        speed_pub_ = create_publisher<std_msgs::msg::Float64>("/speed", 10);

        last_tick_ = now();
        timer_ = create_wall_timer(
            std::chrono::duration<double>(1.0 / control_hz_),
            [this]() { onTimer(); });

        RCLCPP_INFO(get_logger(),
            "drive_speed_controller 시작. %.0fHz v=[%.2f, %.2f] m/s",
            control_hz_, speed_.v_min, speed_.v_max);
    }

private:
    void publishSpeed(double v)
    {
        std_msgs::msg::Float64 m;
        m.data = v;
        speed_pub_->publish(m);
    }

    void onTimer()
    {
        const rclcpp::Time t = now();
        const double dt = std::max(1e-3, (t - last_tick_).seconds());
        last_tick_ = t;

        (void)tracker_.consumeSourceChanged();
        const TrackResult r = tracker_.update();
        if (!r.ok)
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "속도 0 발행: %s", r.reason.c_str());
            v_cmd_ = 0.0;
            v_ema_ = 0.0;
            publishSpeed(0.0);
            return;
        }

        const Curve * cv = tracker_.curve();

        // 곡률 창: 현재 속도가 빠를수록 멀리 본다.
        const double look = std::clamp(
            speed_.look_k() * v_ema_ * kau::control::CM_PER_M,
            speed_.look_min, speed_.look_max);
        double s0 = r.s + speed_.look_min;
        double s1 = r.s + look;

        // valid_length 안으로 자른다. 믿지 말라고 표시된 구간의 곡률로
        // 감속하면 근거 없는 감속이다 (KauPath.valid_length 계약).
        const double vl = tracker_.valid_length();
        const double limit = (vl > 0.0) ? std::min(cv->length(), vl) : cv->length();
        s1 = std::min(s1, limit);
        s0 = std::min(s0, std::max(0.0, s1 - speed_.min_span));
        if (s1 - s0 < speed_.min_span)
        {
            // 창이 퇴화했다 (경로가 짧거나 끝에 다 왔다). 최저속으로 간다.
            s0 = r.s;
            s1 = std::min(limit, r.s + speed_.min_span);
        }

        const double kappa_win =
            (s1 > s0) ? cv->kappaMaxOver(s0, s1) : vehicle_.kappa_max();
        const double want = speed_.target(kappa_win, vehicle_);

        v_cmd_ = speed_.rateLimit(v_cmd_, want, dt);
        const double a = speed_.emaAlpha(dt);
        v_ema_ = v_ema_ + a * (v_cmd_ - v_ema_);
        publishSpeed(v_ema_);

        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
            "속도 %.2f m/s (목표 %.2f, kappa_win %.5f, 창 %.0f~%.0fcm, src=%u)",
            v_ema_, want, kappa_win, s0, s1, tracker_.source());
    }

    DriveTracker tracker_;
    kau::control::VehicleParams vehicle_;
    kau::control::SpeedParams speed_;

    double control_hz_ = 50.0;
    double v_cmd_ = 0.0;
    double v_ema_ = 0.0;
    rclcpp::Time last_tick_;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr speed_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace control_drive
}  // namespace kau

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<kau::control_drive::DriveSpeedController>());
    rclcpp::shutdown();
    return 0;
}
