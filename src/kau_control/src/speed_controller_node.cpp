// ====================================================================
// speed_controller_node.cpp
//
// 속도 제어. 앞으로 갈 구간의 곡률로 목표속도를 정하고 PID 로 /speed 를 낸다.
//
//     kappa_win = 전방 window 의 최대 |kappa|      (직선이면 ~0)
//     v_ref     = law(kappa_win) -> [v_min, v_max] -> 가감속 제한
//     /speed    = clamp(v_ref + PID(v_ref - v_meas), 0, v_max)
//
// 직선에서는 v_max, 곡선에서는 곡률이 클수록 낮아진다 (params.hpp 의 sqrt law).
//
// 출발 게이트 (start_gate.*)
//     신호등이 초록이 되기 전에는 /speed 를 0 으로 묶는다. 허가는
//     /perception/start_permission (std_msgs/Bool) 으로 들어온다.
//
//         초기값 · 미수신 · false  -> 0 발행
//         true 1회 수신            -> latch. 이후 값은 로깅만 하고 무시
//
//     대기 중에도 곡률 -> 목표속도 -> EMA 는 계속 돌린다 (warm). 허가가
//     떨어지면 EMA 수렴값이 이미 서 있어 차가운 시작이 없고, v_ref 만
//     0 에서 accel_max 로 올라간다.
//
// 속도 피드백(v_meas)
//     아직 오도메트리 노드가 없다. topic 이름은 speed.feedback_topic
//     parameter 로 두었으니 노드가 생기면 YAML 한 줄만 바꾸면 된다.
//     피드백이 없거나 stale 이면
//         require_feedback = true   -> 정지 (0 발행)
//         require_feedback = false  -> PID 보정 없이 v_ref 만 발행
// ====================================================================

#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>

#include "kau_control/params.hpp"
#include "kau_control/path_tracker.hpp"
#include "kau_control/pid.hpp"


namespace kau
{
namespace control
{

class SpeedController : public rclcpp::Node
{
public:
    SpeedController()
    : rclcpp::Node("speed_controller"),
      tracker_(this)
    {
        declare_parameter<double>("control_hz", 50.0);
        declare_parameter<double>("vehicle.wheelbase_cm", 18.0);
        declare_parameter<double>("vehicle.max_steer_deg", 20.0);

        declare_parameter<double>("speed.v_min", 0.5);
        declare_parameter<double>("speed.v_max", 2.0);
        declare_parameter<double>("speed.look_min_cm", 20.0);
        declare_parameter<double>("speed.look_max_cm", 150.0);
        declare_parameter<double>("speed.min_span_cm", 20.0);
        declare_parameter<double>("speed.margin", 0.8);
        declare_parameter<bool>("speed.use_sqrt_law", true);
        declare_parameter<double>("speed.accel_max", 1.0);
        declare_parameter<double>("speed.decel_max", 2.0);
        declare_parameter<double>("speed.ema_tau", 0.4);

        // 오도메트리 노드가 확정되면 이 값만 바꾼다 (nav_msgs/Odometry).
        declare_parameter<std::string>(
            "speed.feedback_topic", "/odometry/filtered");
        declare_parameter<double>("speed.feedback_timeout", 0.2);
        declare_parameter<bool>("speed.require_feedback", false);

        // 출발 게이트. state_machine 이 /state_machine/speed_limit 을 내기
        // 시작하면 permission_topic 구독을 그 토픽으로 갈아끼운다
        // (kau_state_machine/docs/01 계약 14). 게이트 판정부는 그대로 쓴다.
        declare_parameter<std::string>(
            "start_gate.permission_topic", "/perception/start_permission");
        declare_parameter<bool>("start_gate.required", true);

        declare_parameter<double>("pid.kp", 0.5);
        declare_parameter<double>("pid.ki", 0.5);
        declare_parameter<double>("pid.kd", 0.0);
        declare_parameter<double>("pid.i_max", 0.5);
        declare_parameter<double>("pid.out_max", 0.5);

        load();

        v_ref_ = 0.0;


        speed_pub_ = create_publisher<std_msgs::msg::Float64>("/speed", 10);

        fb_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            get_parameter("speed.feedback_topic").as_string(), 10,
            [this](nav_msgs::msg::Odometry::SharedPtr m)
            {
                v_meas_  = m->twist.twist.linear.x;

                fb_stamp_ = now();
            });


        // 발행자(start_signal_detector)와 같은 QoS 여야 매칭된다:
        // RELIABLE / KEEP_LAST(1) / VOLATILE.
        //
        // 미수신 대비 timeout 은 두지 않는다. 초기값이 false 이고 latch 전에는
        // true 를 봐야만 열리므로, 안 오면 안 열리는 것이 이미 기본값이다.
        permission_sub_ = create_subscription<std_msgs::msg::Bool>(
            get_parameter("start_gate.permission_topic").as_string(),
            rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile(),
            [this](std_msgs::msg::Bool::SharedPtr m)
            {
                // 소비 측 latch. 검출 노드가 재시작하면 그쪽 latch 가 풀려
                // false 를 다시 내는데, 그것을 그대로 따르면 트랙 한복판에서
                // 멈춘다. 주행 중 신호등은 정의상 오검출이다 (docs/01 4-3).
                if (start_permitted_)
                {
                    return;
                }

                if (m->data)
                {
                    start_permitted_ = true;

                    RCLCPP_INFO(
                        get_logger(),
                        "출발 허가 수신 (latch). 주행을 시작한다");
                }
            });


        const auto period = std::chrono::duration<double>(
            1.0 / get_parameter("control_hz").as_double());

        timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period),
            [this]()
            {
                onTimer();
            });

        last_tick_ = now();

        // ros2 param set 으로 게인을 바꾸면 즉시 반영한다 (PID 튜닝용).
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
            get_logger(),
            "speed_controller 시작. v=%.2f~%.2f m/s 피드백=%s (%s) "
            "출발게이트=%s",
            sp_.v_min, sp_.v_max,
            get_parameter("speed.feedback_topic").as_string().c_str(),
            require_fb_ ? "필수" : "없으면 v_ref 만 발행",
            gate_required_
                ? get_parameter("start_gate.permission_topic")
                    .as_string().c_str()
                : "없음 (start_gate.required=false)");
    }

private:
    void load()
    {
        vehicle_.wheelbase =
            get_parameter("vehicle.wheelbase_cm").as_double();
        vehicle_.max_steer =
            get_parameter("vehicle.max_steer_deg").as_double();

        sp_.v_min     = get_parameter("speed.v_min").as_double();
        sp_.v_max     = get_parameter("speed.v_max").as_double();
        sp_.look_min  = get_parameter("speed.look_min_cm").as_double();
        sp_.look_max  = get_parameter("speed.look_max_cm").as_double();
        sp_.min_span  = get_parameter("speed.min_span_cm").as_double();
        sp_.margin    = get_parameter("speed.margin").as_double();
        sp_.use_sqrt  = get_parameter("speed.use_sqrt_law").as_bool();
        sp_.accel_max = get_parameter("speed.accel_max").as_double();
        sp_.decel_max = get_parameter("speed.decel_max").as_double();
        sp_.ema_tau   = get_parameter("speed.ema_tau").as_double();

        fb_timeout_ = get_parameter("speed.feedback_timeout").as_double();
        require_fb_ = get_parameter("speed.require_feedback").as_bool();

        gate_required_ = get_parameter("start_gate.required").as_bool();

        PidParams pp;

        pp.kp      = get_parameter("pid.kp").as_double();
        pp.ki      = get_parameter("pid.ki").as_double();
        pp.kd      = get_parameter("pid.kd").as_double();
        pp.i_max   = get_parameter("pid.i_max").as_double();
        pp.out_max = get_parameter("pid.out_max").as_double();

        pid_.configure(pp);
    }

    void onTimer()
    {
        const rclcpp::Time t = now();

        double dt = (t - last_tick_).seconds();

        last_tick_ = t;

        // ROS timer 는 밀린다. 실제 경과시간을 쓰되 말이 안 되면 공칭값.
        if (dt <= 0.0 || dt > 1.0)
        {
            dt = 1.0 / get_parameter("control_hz").as_double();
        }


        const TrackResult r = tracker_.update();

        if (!r.ok || r.goal)
        {
            if (r.goal && !goal_logged_)
            {
                RCLCPP_INFO(get_logger(), "경로 종점 도달. 정지한다.");

                goal_logged_ = true;
            }

            stop(r.reason);

            return;
        }

        goal_logged_ = false;


        // --- 목표속도: 전방 window 의 최대 곡률 ---
        double lo = 0.0;

        double hi = 0.0;

        sp_.window(v_ref_, lo, hi);

        const double kappa_win =
            tracker_.curve()->kappaMaxOver(r.s + lo, r.s + hi);

        // EMA(목표 평활) -> 가감속 제한(하드 슬루 상한).
        //
        // 순서를 뒤집으면 안 된다. rateLimit 이 이미 accel_max*dt 로 잘라 놓은
        // 증분에 EMA 가 다시 alpha 배만 다가가므로 실효 가속이
        // alpha * accel_max 로 줄어든다 (50 Hz / tau 0.4 s 에서 0.049 m/s^2).
        //
        // EMA 식은 원본 시뮬(full_simulation.py)과 같다.
        v_want_ +=
            sp_.emaAlpha(dt) * (sp_.target(kappa_win, vehicle_) - v_want_);


        // --- 출발 게이트 ---
        // 여기가 게이트 자리다. 위의 목표속도 계산(곡률 -> target -> EMA)은
        // 대기 중에도 이미 돌았으므로 v_want_ 는 계속 warm 이고, 허가 즉시
        // 그 값으로 붙는다 (docs/01 4-3 "min 은 EMA 뒤").
        //
        // v_ref_ 를 0 으로 잡아 두는 것이 급가속 방지의 전부다. 허가가
        // 떨어지면 아래 rateLimit 이 0 에서 accel_max 로 끌어올린다
        // (accel_max 1.5 에서 0.5 m/s 까지 0.33 s). 게이트를 rateLimit
        // **뒤**에 두면 v_ref_ 가 대기 중 v_max 까지 차올라 허가 순간
        // 계단으로 튄다 -- 순서를 바꾸면 안 되는 이유다.
        //
        // 경로 이상 · 종점 정지(stop())는 이 위에서 이미 처리됐다. 그쪽은
        // rateLimit 을 거치지 않고 즉시 0 이라 안전 경로가 느려지지 않는다.
        if (gate_required_ && !start_permitted_)
        {
            v_ref_ = 0.0;

            // 게이트로 눌려 있는 동안 오차(v_ref - v_meas)를 적분하면
            // 허가 순간 포화된 적분항이 그대로 실린다. 대기 중 적분 상태에
            // 보존할 정보가 없으므로 얼리지 않고 매 tick 씻는다.
            pid_.reset();

            publishSpeed(0.0);

            RCLCPP_INFO_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "출발 허가 대기 중 (%s). v_want=%.2f m/s 로 대기",
                get_parameter("start_gate.permission_topic")
                    .as_string().c_str(),
                v_want_);

            return;
        }

        v_ref_ = sp_.rateLimit(v_ref_, v_want_, dt);


        // --- PID 보정 ---
        const bool fb_ok =
            fb_stamp_.nanoseconds() != 0 &&
            (t - fb_stamp_).seconds() < fb_timeout_;

        double u = v_ref_;

        if (fb_ok)
        {
            u += pid_.step(v_ref_ - v_meas_, dt);
        }
        else if (require_fb_)
        {
            stop("속도 피드백이 없다 (speed.feedback_topic 확인)");

            return;
        }
        else
        {
            // 피드백이 붙기 전. 적분기를 얼려 두고 v_ref 만 내보낸다.
            pid_.reset();
        }

        publishSpeed(std::clamp(u, 0.0, sp_.v_max));

        RCLCPP_DEBUG_THROTTLE(
            get_logger(), *get_clock(), 500,
            "s=%.1f kappa=%.4f v_ref=%.2f v_meas=%.2f cmd=%.2f m/s%s",
            r.s, kappa_win, v_ref_, v_meas_, std::clamp(u, 0.0, sp_.v_max),
            fb_ok ? "" : " (피드백 없음)");
    }

    // 어떤 경로로 빠져나가든 0 을 발행한다. 조용히 return 하면 driver 의
    // cmd_timeout(1 s) 워치독이 끝날 때까지 직전 속도로 계속 굴러간다.
    void stop(const std::string & reason)
    {
        publishSpeed(0.0);

        v_ref_ = 0.0;

        v_want_ = 0.0;

        pid_.reset();

        if (!reason.empty())
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000, "정지: %s", reason.c_str());
        }
    }

    void publishSpeed(double v_ms)
    {
        std_msgs::msg::Float64 m;

        m.data = v_ms;

        speed_pub_->publish(m);
    }


    PathTracker   tracker_;
    VehicleParams vehicle_;
    SpeedParams   sp_;
    Pid           pid_;

    double v_want_     = 0.0;   // m/s, EMA 만 거친 목표속도
    double v_ref_      = 0.0;   // m/s, 위에 가감속 제한까지 건 최종 목표속도
    double v_meas_     = 0.0;   // m/s, 피드백
    double fb_timeout_ = 0.2;
    bool   require_fb_ = false;
    bool   goal_logged_ = false;

    // 출발 허가. **반드시 false 로 시작한다.** true 를 한 번 보면 latch 되고
    // 그 뒤 들어오는 값은 무시한다.
    bool   start_permitted_ = false;
    bool   gate_required_   = true;

    rclcpp::Time fb_stamp_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_tick_{0, 0, RCL_ROS_TIME};

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr speed_pub_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr fb_sub_;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr permission_sub_;

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
        rclcpp::spin(std::make_shared<kau::control::SpeedController>());
    }
    catch (const std::exception & e)
    {
        RCLCPP_FATAL(
            rclcpp::get_logger("speed_controller"), "기동 실패: %s", e.what());

        rclcpp::shutdown();

        return 1;
    }

    rclcpp::shutdown();

    return 0;
}
