// ====================================================================
// path_follower_node.cpp
//
// 경로 추종 제어 노드. KauPath 를 받아 /speed, /steering 을 발행한다.
//
// 원본: KAU_AMET_Test / src/kau_controller/test/runner.py 의 제어 loop.
//       시뮬의 while 문이 여기서 timer callback 이 된다.
//       차량 모델(vehicle.py) · 노이즈 · 시각화 · 시나리오는 시뮬 전용이라
//       옮기지 않았다. 실차/Gazebo 가 그 역할을 한다.
//
// 단위 규약 (매우 중요)
//     내부 계산  : cm / deg / (1/cm)   <- 레퍼런스와 동일
//     ROS 경계   : m  / rad
//     변환은 pose 수신 직후와 명령 발행 직전 **두 곳에서만** 한다.
//
// TF 소유권
//     이 노드는 TF 를 발행하지 않는다. map->odom 은 Cartographer,
//     odom->base_footprint 는 EKF 가 소유한다 (kau_localization/README.md).
// ====================================================================

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/float64.hpp>

#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

// tf2::getYaw 가 쓰는 fromMsg(Quaternion) 의 정의가 여기 있다.
// tf2/utils.h 만 넣으면 선언만 보여서 링크 단계에서 터진다.
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "kau_msgs/msg/kau_path.hpp"

#include "kau_control/curve.hpp"
#include "kau_control/kau_path.hpp"
#include "kau_control/params.hpp"
#include "kau_control/pure_pursuit.hpp"


namespace kau
{
namespace control
{

class PathFollower : public rclcpp::Node
{
public:
    PathFollower()
    : rclcpp::Node("path_follower")
    {
        declareParams();

        loadParams();


        tf_buffer_ =
            std::make_unique<tf2_ros::Buffer>(get_clock());

        tf_listener_ =
            std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);


        // QoS 는 topic 마다 다르다 (kau_msgs/README.md).
        //
        //     /path/global   TRANSIENT_LOCAL, RELIABLE, depth 1  (latched)
        //     /path/local    RELIABLE, depth 1                   (volatile)
        //     /lane/center   RELIABLE, depth 1                   (volatile)
        //
        // **TRANSIENT_LOCAL 구독자는 VOLATILE 발행자와 호환되지 않는다.**
        // 반대 방향(VOLATILE 구독자 <- TRANSIENT_LOCAL 발행자)만 성립한다.
        // 전부 transient_local 로 구독하면 /lane/center 와 /path/local 이
        // "incompatible QoS. No messages will be sent" 로 조용히 끊긴다.
        auto qos_of = [](bool latched)
            {
                auto q = rclcpp::QoS(1).reliable();

                return latched ? q.transient_local() : q.durability_volatile();
            };

        path_sub_ = create_subscription<kau_msgs::msg::KauPath>(
            path_topic_, qos_of(path_latched_),
            [this](kau_msgs::msg::KauPath::SharedPtr msg)
            {
                onPath(std::move(msg), false);
            });

        if (!fallback_topic_.empty() && fallback_topic_ != path_topic_)
        {
            fallback_sub_ = create_subscription<kau_msgs::msg::KauPath>(
                fallback_topic_, qos_of(fallback_latched_),
                [this](kau_msgs::msg::KauPath::SharedPtr msg)
                {
                    onPath(std::move(msg), true);
                });
        }


        speed_pub_ =
            create_publisher<std_msgs::msg::Float64>("/speed", 10);

        steer_pub_ =
            create_publisher<std_msgs::msg::Float64>("/steering", 10);

        // RViz 는 KauPath 를 못 그린다 -> 시각화는 별도 topic 으로 분리 발행
        viz_path_pub_ =
            create_publisher<nav_msgs::msg::Path>("/viz/path/tracked", 1);

        viz_target_pub_ =
            create_publisher<geometry_msgs::msg::PointStamped>(
                "/viz/path/lookahead", 1);


        const auto period = std::chrono::duration<double>(1.0 / control_hz_);

        timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period),
            [this]()
            {
                onTimer();
            });

        last_tick_ = now();

        // ros2 param set 으로 튜닝값을 바꾸면 즉시 반영한다.
        // 이게 없으면 값만 바뀌고 노드는 계속 옛 값으로 돈다.
        param_cb_ = add_on_set_parameters_callback(
            [this](const std::vector<rclcpp::Parameter> & params)
            {
                return onParamChange(params);
            });


        RCLCPP_INFO(
            get_logger(),
            "path_follower 시작. path=%s fallback=%s %.0f Hz "
            "(wheelbase %.1f cm, 후륜축 offset %.1f cm)",
            path_topic_.c_str(),
            fallback_topic_.empty() ? "(없음)" : fallback_topic_.c_str(),
            control_hz_, vehicle_.wheelbase, vehicle_.rear_axle_offset);
    }

private:
    // ----------------------------------------------------------------
    // parameter
    // ----------------------------------------------------------------

    void declareParams()
    {
        declare_parameter<std::string>("path_topic", "/path/local");
        declare_parameter<std::string>("fallback_path_topic", "/path/global");
        declare_parameter<std::string>("map_frame", "map");
        declare_parameter<std::string>("base_frame", "base_footprint");

        // "tf"       : map -> base_frame TF 로 차량 pose 를 얻는다 (전역 경로용)
        // "identity" : 경로가 이미 차량 프레임에 있다. TF 를 보지 않는다.
        //              Lane Detection 의 /lane/center (base_link, s_offset=0,
        //              localization 비의존) 를 그대로 따라갈 때 쓴다.
        declare_parameter<std::string>("pose_source", "tf");

        // 발행측 durability 와 반드시 맞춰야 한다. 틀리면 조용히 안 받는다.
        declare_parameter<bool>("path_latched", false);
        declare_parameter<bool>("fallback_path_latched", true);

        declare_parameter<double>("control_hz", 100.0);
        declare_parameter<double>("tf_timeout", 0.2);
        declare_parameter<double>("path_timeout", 1.0);
        declare_parameter<double>("cte_abort_cm", 200.0);
        declare_parameter<double>("goal_tol_cm", 10.0);

        declare_parameter<double>("vehicle.wheelbase_cm", 18.0);
        declare_parameter<double>("vehicle.max_steer_deg", 20.0);
        declare_parameter<double>("vehicle.rear_axle_offset_cm", -9.0);

        declare_parameter<double>("controller.k_v", 0.5);
        declare_parameter<double>("controller.ld_min_cm", 30.0);
        declare_parameter<double>("controller.ld_max_cm", 175.0);

        declare_parameter<bool>("speed.enabled", true);
        declare_parameter<double>("speed.constant", 1.0);
        declare_parameter<double>("speed.v_min", 0.5);
        declare_parameter<double>("speed.v_max", 2.0);
        declare_parameter<double>("speed.look_min_cm", 20.0);
        declare_parameter<double>("speed.look_max_cm", 150.0);
        declare_parameter<double>("speed.min_span_cm", 20.0);
        declare_parameter<double>("speed.margin", 0.8);
        declare_parameter<bool>("speed.use_sqrt_law", true);
        declare_parameter<double>("speed.ema_tau", 0.4);

        declare_parameter<bool>("viz.enabled", true);
        declare_parameter<int>("viz.samples_per_seg", 20);
    }

    void loadParams()
    {
        path_topic_     = get_parameter("path_topic").as_string();
        fallback_topic_ = get_parameter("fallback_path_topic").as_string();
        map_frame_      = get_parameter("map_frame").as_string();
        base_frame_     = get_parameter("base_frame").as_string();
        pose_source_    = get_parameter("pose_source").as_string();

        path_latched_     = get_parameter("path_latched").as_bool();
        fallback_latched_ = get_parameter("fallback_path_latched").as_bool();

        vehicle_frame_mode_ = (pose_source_ == "identity");

        control_hz_   = get_parameter("control_hz").as_double();
        tf_timeout_   = get_parameter("tf_timeout").as_double();
        path_timeout_ = get_parameter("path_timeout").as_double();
        cte_abort_    = get_parameter("cte_abort_cm").as_double();
        goal_tol_     = get_parameter("goal_tol_cm").as_double();

        vehicle_.wheelbase =
            get_parameter("vehicle.wheelbase_cm").as_double();
        vehicle_.max_steer =
            get_parameter("vehicle.max_steer_deg").as_double();
        vehicle_.rear_axle_offset =
            get_parameter("vehicle.rear_axle_offset_cm").as_double();

        ctrl_.k_v    = get_parameter("controller.k_v").as_double();
        ctrl_.ld_min = get_parameter("controller.ld_min_cm").as_double();
        ctrl_.ld_max = get_parameter("controller.ld_max_cm").as_double();

        speed_enabled_  = get_parameter("speed.enabled").as_bool();
        speed_constant_ = get_parameter("speed.constant").as_double();

        sp_.v_min    = get_parameter("speed.v_min").as_double();
        sp_.v_max    = get_parameter("speed.v_max").as_double();
        sp_.look_min = get_parameter("speed.look_min_cm").as_double();
        sp_.look_max = get_parameter("speed.look_max_cm").as_double();
        sp_.min_span = get_parameter("speed.min_span_cm").as_double();
        sp_.margin   = get_parameter("speed.margin").as_double();
        sp_.use_sqrt = get_parameter("speed.use_sqrt_law").as_bool();
        sp_.ema_tau  = get_parameter("speed.ema_tau").as_double();

        viz_enabled_ = get_parameter("viz.enabled").as_bool();
        viz_samples_ = static_cast<int>(
            get_parameter("viz.samples_per_seg").as_int());

        // 주행 중 parameter 를 바꿔도 속도를 튕기지 않는다 (최초 1회만 초기화)
        if (!params_loaded_)
        {
            v_ema_ = speed_enabled_ ? sp_.v_min : speed_constant_;

            params_loaded_ = true;
        }
    }

    // 구독 topic / QoS / 타이머 주기처럼 재구성이 필요한 항목은 거부하고,
    // 튜닝값만 즉시 반영한다. 거부한 항목은 노드를 다시 띄워야 바뀐다.
    rcl_interfaces::msg::SetParametersResult onParamChange(
        const std::vector<rclcpp::Parameter> & params)
    {
        rcl_interfaces::msg::SetParametersResult result;

        result.successful = true;

        static const std::vector<std::string> fixed = {
            "path_topic", "fallback_path_topic", "control_hz", "pose_source",
            "path_latched", "fallback_path_latched",
        };

        for (const rclcpp::Parameter & p : params)
        {
            if (std::find(fixed.begin(), fixed.end(), p.get_name()) !=
                fixed.end())
            {
                result.successful = false;

                result.reason =
                    p.get_name() + " 은(는) 실행 중 바꿀 수 없다. "
                    "노드를 다시 띄울 것.";

                return result;
            }
        }

        // 선언된 값은 이미 갱신된 상태이므로 그대로 다시 읽는다.
        loadParams();

        RCLCPP_INFO(
            get_logger(),
            "parameter 갱신: k_v=%.2f Ld=%.0f~%.0f cm speed=%s v=%.2f~%.2f",
            ctrl_.k_v, ctrl_.ld_min, ctrl_.ld_max,
            speed_enabled_ ? "곡률기반" : "상수", sp_.v_min, sp_.v_max);

        return result;
    }

    // ----------------------------------------------------------------
    // 경로 수신
    // ----------------------------------------------------------------

    void onPath(kau_msgs::msg::KauPath::SharedPtr msg, bool is_fallback)
    {
        // 주 경로가 살아 있는 동안은 fallback 을 무시한다.
        if (is_fallback && haveFreshPrimary())
        {
            return;
        }

        std::string reason;

        if (!validateKauPath(*msg, reason))
        {
            RCLCPP_ERROR_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "KauPath 무결성 검사 실패 -> 무시: %s", reason.c_str());

            return;
        }

        const std::string & want =
            vehicle_frame_mode_ ? base_frame_ : map_frame_;

        if (msg->header.frame_id != want)
        {
            RCLCPP_ERROR_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "경로 frame_id='%s' 이지만 이 노드는 '%s' 기준으로 추종한다 "
                "(pose_source=%s). 발행측이 프레임을 맞추거나 "
                "map_frame / base_frame / pose_source 를 바꿀 것.",
                msg->header.frame_id.c_str(), want.c_str(),
                pose_source_.c_str());

            return;
        }


        Curve cv = curveFromKauPath(*msg);

        if (!cv.windowSafe())
        {
            // 폐곡선 segment 1개가 전장의 절반 이상 -> 8.4 최소표현이 뒤집힌다
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 5000,
                "폐곡선인데 segment 가 %d 개뿐이다. window 추적이 불안정하다 "
                "(nseg >= 3 필요).", cv.nseg());
        }

        if (!cv.isRegular())
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 5000,
                "퇴화 segment 를 포함한 경로다 (|r'| ~ 0). 헤딩이 튈 수 있다.");
        }


        curve_ = std::make_shared<Curve>(std::move(cv));

        path_source_   = msg->source;

        valid_length_  = msg->valid_length;

        path_conf_     = msg->confidence;

        // 새 경로 -> 다음 tick 에 전역 재탐색 (8.3)
        track_ = TrackState{};

        // 종점이 실제로 옮겨갔을 때만 "도착" 을 푼다. 같은 경로를 주기
        // 재발행하는 발행자를 만나면 매번 리셋되어 도착 후 다시 출발한다.
        const Point2 goal = curve_->point(curve_->length());

        if (!goal_valid_ ||
            std::hypot(goal.x - goal_.x, goal.y - goal_.y) > goal_tol_)
        {
            goal_reached_ = false;
        }

        goal_      = goal;

        goal_valid_ = true;

        const rclcpp::Time stamp = now();

        if (is_fallback)
        {
            fallback_stamp_ = stamp;
        }
        else
        {
            primary_stamp_ = stamp;
        }

        publishVizPath();

        // Lane / local path 는 10 Hz 이상으로 계속 들어온다. 매번 찍으면
        // 로그가 이것만 남아 DEBUG 가 안 보인다.
        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 3000,
            "경로 수신 (%s): nseg=%d 길이=%.1f cm %s conf=%.2f valid=%.0f cm",
            is_fallback ? "fallback" : "primary",
            curve_->nseg(), curve_->length(),
            curve_->closed() ? "폐곡선" : "개곡선",
            path_conf_, valid_length_);
    }

    bool haveFreshPrimary() const
    {
        if (primary_stamp_.nanoseconds() == 0)
        {
            return false;
        }

        return (now() - primary_stamp_).seconds() < path_timeout_;
    }

    // ----------------------------------------------------------------
    // 제어 loop  (runner.py 88~142 줄)
    // ----------------------------------------------------------------

    void onTimer()
    {
        const rclcpp::Time tick = now();

        // 시뮬은 dt 고정 100 Hz. ROS timer 는 밀리므로 실제 경과시간을 쓴다.
        double dt = (tick - last_tick_).seconds();

        last_tick_ = tick;

        if (dt <= 0.0 || dt > 1.0)
        {
            dt = 1.0 / control_hz_;
        }


        if (!curve_ || curve_->empty())
        {
            stop("경로 없음");

            return;
        }

        if (goal_reached_)
        {
            stop("");   // 도착 후에는 조용히 0 유지 (watchdog 때문에 계속 발행)

            return;
        }


        // --- 1. 포즈: 시뮬의 car.pose 자리 ---
        //
        // 기준점 보정: Pure Pursuit 기준점은 후륜축 중심인데 PhysiCar 의
        // base_footprint / base_link 는 휠베이스 중앙이다. 보정을 빼면
        // 코너에서 계속 안쪽으로 파고든다.
        const double off = vehicle_.rear_axle_offset;      // cm, 보통 -9

        double x   = 0.0;

        double y   = 0.0;

        double yaw = 0.0;

        if (vehicle_frame_mode_)
        {
            // 경로가 이미 차량 프레임에 있다. 차량은 그 프레임의 원점이므로
            // TF 를 볼 필요가 없다 (Lane Detection 경로 = localization 비의존).
            // 후륜축만 차량 원점에서 뒤로 물러나 있다.
            x = off;

            // 경로가 stale 하면 TF 대신 이게 유일한 안전장치다.
            if (primary_stamp_.nanoseconds() == 0)
            {
                stop("경로를 아직 못 받았다");

                return;
            }

            const double age = (tick - primary_stamp_).seconds();

            // 기동 직후 node clock 이 system -> sim 으로 넘어가는 순간
            // 두 시각의 기준이 달라 age 가 말도 안 되게 튄다. 시계가 안
            // 잡힌 것이지 경로가 끊긴 게 아니므로, 다시 찍고 한 tick 쉰다.
            if (age < 0.0 || age > CLOCK_SANITY_S)
            {
                primary_stamp_ = tick;

                stop("");

                return;
            }

            if (age > path_timeout_)
            {
                stop("경로가 " + std::to_string(age) + " s 갱신되지 않았다");

                return;
            }
        }
        else
        {
            geometry_msgs::msg::TransformStamped tf;

            try
            {
                tf = tf_buffer_->lookupTransform(
                    map_frame_, base_frame_, tf2::TimePointZero);
            }
            catch (const tf2::TransformException & ex)
            {
                stop(std::string("TF 조회 실패: ") + ex.what());

                return;
            }

            const double tf_age =
                (tick - rclcpp::Time(tf.header.stamp)).seconds();

            if (tf_age > tf_timeout_)
            {
                stop("TF 가 " + std::to_string(tf_age) +
                     " s 지연됐다 (측위 끊김)");

                return;
            }

            yaw = tf2::getYaw(tf.transform.rotation);

            // --- 2. 단위 변환 (m -> cm) ---
            x = tf.transform.translation.x * CM_PER_M + off * std::cos(yaw);

            y = tf.transform.translation.y * CM_PER_M + off * std::sin(yaw);
        }


        // --- 3. 최근접점: 첫 질의만 전역(8.3), 이후 국소 window(8.4) ---
        track_ = curve_->nearest(Point2{x, y}, track_);

        if (!track_.valid)
        {
            stop("최근접점 탐색 실패");

            return;
        }


        double cte = 0.0;

        double head_err = 0.0;

        curve_->errors(x, y, yaw, track_, cte, head_err);

        if (std::abs(cte) > cte_abort_)
        {
            stop("cross track error " + std::to_string(cte) +
                 " cm -> 발산 판정");

            return;
        }

        // Lane 경로는 차량과 같이 움직이는 rolling horizon 이라 종점이 곧
        // "앞으로 본 데까지" 일 뿐이다. 도착으로 처리하면 매번 멈춘다.
        if (path_source_ != kau_msgs::msg::KauPath::SRC_LANE &&
            curve_->finished(track_, goal_tol_))
        {
            goal_reached_ = true;

            stop("");

            RCLCPP_INFO(get_logger(), "경로 종점 도달. 정지한다.");

            return;
        }


        // --- 4. 속도: 전방 window 의 최대 곡률 -> 목표 속도 (EMA 평활) ---
        double v_cmd = speed_constant_;

        if (speed_enabled_)
        {
            double lo = 0.0;

            double hi = 0.0;

            sp_.window(v_ema_, lo, hi);

            const double kappa_win =
                curve_->kappaMaxOver(track_.s + lo, track_.s + hi);

            const double v_target = sp_.target(kappa_win, vehicle_);

            v_ema_ += (v_target - v_ema_) * sp_.ema_alpha(dt);

            v_cmd = v_ema_;
        }
        else
        {
            v_ema_ = speed_constant_;
        }


        // --- 5. 조향: Pure Pursuit ---
        double ld = pure_pursuit::lookaheadDistance(v_cmd, ctrl_);

        // Lane Detection 은 실제로 본 구간까지만 valid_length 로 알려준다.
        // 그 너머는 외삽이므로 LookAhead 점을 거기 두면 안 된다.
        // (그 외 source 는 valid_length = 0 이라 이 절이 걸리지 않는다)
        if (valid_length_ > 0.0)
        {
            const double usable = valid_length_ - track_.s - ld_valid_margin_;

            if (usable < ctrl_.ld_min)
            {
                stop("관측 구간이 " + std::to_string(valid_length_) +
                     " cm 뿐이라 LookAhead 를 둘 곳이 없다");

                return;
            }

            ld = std::min(ld, usable);
        }

        const Point2 tgt = curve_->point(curve_->lookahead(track_.s, ld));

        double steer_deg = pure_pursuit::steerCommand(
            x, y, yaw, tgt.x, tgt.y, ld, vehicle_);

        steer_deg = std::min(
            std::max(steer_deg, -vehicle_.max_steer), vehicle_.max_steer);


        // --- 6. 발행: 여기서만 deg -> rad, cm -> m ---
        publishSteering(deg2rad(steer_deg));

        publishSpeed(v_cmd);

        publishVizTarget(tgt);


        RCLCPP_DEBUG_THROTTLE(
            get_logger(), *get_clock(), 500,
            "s=%.1f cte=%.1f cm head_err=%.1f deg Ld=%.0f "
            "steer=%.1f deg v=%.2f m/s",
            track_.s, cte, rad2deg(head_err), ld, steer_deg, v_cmd);
    }

    // ----------------------------------------------------------------
    // 발행
    // ----------------------------------------------------------------

    // 정지. reason 이 비어 있지 않으면 throttle 로 한 번 알린다.
    //
    // 주의: 그냥 return 하면 안 된다. /speed 는 driver 의 cmd_timeout(1 s)
    // 워치독이 있어 갱신을 멈추면 1 초간 직전 속도로 계속 굴러간다.
    // 어떤 경로로 빠져나가든 반드시 0 을 발행한다.
    void stop(const std::string & reason)
    {
        publishSpeed(0.0);

        publishSteering(0.0);

        v_ema_ = speed_enabled_ ? sp_.v_min : speed_constant_;

        if (!reason.empty())
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "정지: %s", reason.c_str());
        }
    }

    void publishSpeed(double v_ms)
    {
        std_msgs::msg::Float64 m;

        m.data = v_ms;

        speed_pub_->publish(m);
    }

    void publishSteering(double rad)
    {
        std_msgs::msg::Float64 m;

        m.data = rad;

        steer_pub_->publish(m);
    }

    void publishVizPath()
    {
        if (!viz_enabled_ || !viz_path_pub_ || !curve_)
        {
            return;
        }

        nav_msgs::msg::Path p;

        p.header.frame_id = vehicle_frame_mode_ ? base_frame_ : map_frame_;

        p.header.stamp = now();

        for (const Point2 & q : curve_->sample(viz_samples_))
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

    void publishVizTarget(const Point2 & tgt)
    {
        if (!viz_enabled_ || !viz_target_pub_)
        {
            return;
        }

        geometry_msgs::msg::PointStamped m;

        m.header.frame_id = vehicle_frame_mode_ ? base_frame_ : map_frame_;

        m.header.stamp = now();

        m.point.x = tgt.x / CM_PER_M;

        m.point.y = tgt.y / CM_PER_M;

        viz_target_pub_->publish(m);
    }

    // ----------------------------------------------------------------

    std::string path_topic_;
    std::string fallback_topic_;
    std::string map_frame_;
    std::string base_frame_;

    double control_hz_   = 100.0;
    double tf_timeout_   = 0.2;
    double path_timeout_ = 1.0;
    double cte_abort_    = 200.0;
    double goal_tol_     = 10.0;

    bool   speed_enabled_  = true;
    double speed_constant_ = 1.0;

    bool viz_enabled_ = true;
    int  viz_samples_ = 20;

    VehicleParams    vehicle_;
    ControllerParams ctrl_;
    SpeedParams      sp_;

    bool        params_loaded_      = false;
    bool        path_latched_       = false;
    bool        fallback_latched_   = true;
    std::string pose_source_        = "tf";
    bool        vehicle_frame_mode_ = false;
    double      ld_valid_margin_    = 5.0;   // cm, valid_length 안쪽 여유

    // 이보다 큰 경로 age 는 "끊김" 이 아니라 "시계가 아직 안 잡힘" 으로 본다
    static constexpr double CLOCK_SANITY_S = 60.0;

    std::shared_ptr<Curve> curve_;
    TrackState             track_;
    double                 v_ema_        = 0.0;
    bool                   goal_reached_ = false;
    uint8_t                path_source_  = 0;
    double                 valid_length_ = 0.0;
    double                 path_conf_    = 1.0;
    Point2                 goal_{};
    bool                   goal_valid_   = false;

    rclcpp::Time primary_stamp_{0, 0, RCL_ROS_TIME};
    rclcpp::Time fallback_stamp_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_tick_{0, 0, RCL_ROS_TIME};

    std::unique_ptr<tf2_ros::Buffer>                    tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener>         tf_listener_;

    rclcpp::Subscription<kau_msgs::msg::KauPath>::SharedPtr path_sub_;
    rclcpp::Subscription<kau_msgs::msg::KauPath>::SharedPtr fallback_sub_;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr speed_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr steer_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr    viz_path_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr
        viz_target_pub_;

    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
        param_cb_;
};

}  // namespace control
}  // namespace kau


int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<kau::control::PathFollower>());

    rclcpp::shutdown();

    return 0;
}
