// ====================================================================
// path_tracker.hpp
//
// speed_controller / steer_controller 가 공유하는 "경로 추종 상태".
//
//     KauPath 구독 (주 경로 + fallback)  ->  Curve
//     차량 pose (TF map->base, 또는 경로가 이미 차량 프레임이면 원점)
//     최근접점 추적 (8.3 전역 -> 8.4 국소 window) + cross track error
//
// 두 노드가 이걸 각자 하나씩 들고 돈다. 서로를 구독하지 않는다.
//
// 단위 규약: 내부는 cm / rad. ROS 경계에서만 m 로 바꾼다.
// TF 는 발행하지 않는다 (map->odom 은 Cartographer, odom->base 는 EKF).
// ====================================================================

#ifndef KAU_CONTROL__PATH_TRACKER_HPP_
#define KAU_CONTROL__PATH_TRACKER_HPP_

#include <cmath>
#include <cstdio>
#include <memory>
#include <string>
#include <utility>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

// tf2::getYaw 가 쓰는 fromMsg(Quaternion) 의 정의가 여기 있다.
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "kau_msgs/msg/kau_path.hpp"

#include "kau_control/curve.hpp"
#include "kau_control/kau_path.hpp"
#include "kau_control/params.hpp"


namespace kau
{
namespace control
{

// update() 한 tick 의 결과.
//
//     ok == false  -> 노드는 즉시 0 을 발행해야 한다 (reason 이 비면 조용히)
//     goal == true -> 종점 도달. 역시 0.
struct TrackResult
{
    bool ok   = false;

    bool goal = false;

    std::string reason;

    double x   = 0.0;   // cm, 후륜축 기준
    double y   = 0.0;   // cm
    double yaw = 0.0;   // rad

    double s   = 0.0;   // cm, 경로 시작점부터의 호길이
    double cte = 0.0;   // cm, 좌측 +
};


class PathTracker
{
public:
    explicit PathTracker(rclcpp::Node * node)
    : node_(node)
    {
        node_->declare_parameter<std::string>("path_topic", "/path/local");
        node_->declare_parameter<std::string>(
            "fallback_path_topic", "/path/global");
        node_->declare_parameter<std::string>("map_frame", "map");
        node_->declare_parameter<std::string>("base_frame", "base_footprint");

        // "tf"       : map -> base_frame TF 로 pose 를 얻는다 (전역 경로용)
        // "identity" : 경로가 이미 차량 프레임에 있다. TF 를 보지 않는다.
        node_->declare_parameter<std::string>("pose_source", "tf");

        // 발행측 durability 와 반드시 맞춰야 한다. 틀리면 조용히 안 받는다.
        node_->declare_parameter<bool>("path_latched", false);
        node_->declare_parameter<bool>("fallback_path_latched", true);

        node_->declare_parameter<double>("tf_timeout", 0.2);
        node_->declare_parameter<double>("path_timeout", 1.0);
        node_->declare_parameter<double>("cte_abort_cm", 200.0);
        node_->declare_parameter<double>("goal_tol_cm", 10.0);
        node_->declare_parameter<double>("vehicle.rear_axle_offset_cm", -9.0);

        load();


        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());

        tf_listener_ =
            std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);


        // QoS 는 topic 마다 다르다 (kau_msgs/README.md).
        // TRANSIENT_LOCAL 구독자는 VOLATILE 발행자와 호환되지 않는다.
        // 전부 latched 로 구독하면 /lane/center 가 조용히 끊긴다.
        auto qos_of = [](bool latched)
            {
                auto q = rclcpp::QoS(1).reliable();

                return latched ? q.transient_local() : q.durability_volatile();
            };

        path_sub_ = node_->create_subscription<kau_msgs::msg::KauPath>(
            path_topic_, qos_of(path_latched_),
            [this](kau_msgs::msg::KauPath::SharedPtr m)
            {
                onPath(std::move(m), false);
            });

        if (!fallback_topic_.empty() && fallback_topic_ != path_topic_)
        {
            fallback_sub_ =
                node_->create_subscription<kau_msgs::msg::KauPath>(
                    fallback_topic_, qos_of(fallback_latched_),
                    [this](kau_msgs::msg::KauPath::SharedPtr m)
                    {
                        onPath(std::move(m), true);
                    });
        }
    }

    // 튜닝값만 다시 읽는다. topic / QoS 는 구독 생성 시점에 굳는다.
    void load()
    {
        path_topic_       = node_->get_parameter("path_topic").as_string();
        fallback_topic_   =
            node_->get_parameter("fallback_path_topic").as_string();
        map_frame_        = node_->get_parameter("map_frame").as_string();
        base_frame_       = node_->get_parameter("base_frame").as_string();
        path_latched_     = node_->get_parameter("path_latched").as_bool();
        fallback_latched_ =
            node_->get_parameter("fallback_path_latched").as_bool();

        identity_ =
            (node_->get_parameter("pose_source").as_string() == "identity");

        tf_timeout_   = node_->get_parameter("tf_timeout").as_double();
        path_timeout_ = node_->get_parameter("path_timeout").as_double();
        cte_abort_    = node_->get_parameter("cte_abort_cm").as_double();
        goal_tol_     = node_->get_parameter("goal_tol_cm").as_double();
        rear_offset_  =
            node_->get_parameter("vehicle.rear_axle_offset_cm").as_double();
    }


    TrackResult update()
    {
        TrackResult r;

        const rclcpp::Time t = node_->now();


        if (!curve_ || curve_->empty())
        {
            r.reason = "경로 없음";

            return r;
        }

        // 주 경로만 신선도를 본다. fallback 전역 경로는 latched 1 회 발행이라
        // age 가 계속 자라므로 여기서 걸면 영원히 못 달린다.
        if (!from_fallback_)
        {
            const double age = (t - stamp_).seconds();

            // 기동 직후 node clock 이 system -> sim 으로 넘어가면 두 시각의
            // 기준이 달라 age 가 말도 안 되게 튄다. 끊긴 게 아니라 시계가
            // 아직 안 잡힌 것이므로 다시 찍고 한 tick 쉰다.
            if (age < 0.0 || age > CLOCK_SANITY_S)
            {
                stamp_ = t;

                return r;   // ok=false, reason 없음 -> 조용히 0
            }

            if (age > path_timeout_)
            {
                r.reason =
                    "경로가 " + fmt(age) + " s 갱신되지 않았다";

                return r;
            }
        }


        // Pure Pursuit 기준점은 후륜축 중심인데 base_frame 원점은 휠베이스
        // 중앙이다. 이 보정을 빼면 코너에서 계속 안쪽으로 파고든다.
        const double off = rear_offset_;

        if (identity_)
        {
            // 차량이 곧 경로 프레임의 원점이다. 후륜축만 뒤로 물러나 있다.
            r.x = off;
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
                r.reason = std::string("TF 조회 실패: ") + ex.what();

                return r;
            }

            const double tf_age =
                (t - rclcpp::Time(tf.header.stamp)).seconds();

            if (tf_age > tf_timeout_)
            {
                r.reason =
                    "TF 가 " + fmt(tf_age) + " s 지연됐다 (측위 끊김)";

                return r;
            }

            r.yaw = tf2::getYaw(tf.transform.rotation);

            r.x = tf.transform.translation.x * CM_PER_M +
                  off * std::cos(r.yaw);

            r.y = tf.transform.translation.y * CM_PER_M +
                  off * std::sin(r.yaw);
        }


        // 첫 질의만 전역(8.3), 이후 국소 window(8.4)
        track_ = curve_->nearest(Point2{r.x, r.y}, track_);

        if (!track_.valid)
        {
            r.reason = "최근접점 탐색 실패";

            return r;
        }

        double head_err = 0.0;

        curve_->errors(r.x, r.y, r.yaw, track_, r.cte, head_err);

        if (std::abs(r.cte) > cte_abort_)
        {
            r.reason =
                "cross track error " + fmt(r.cte) + " cm -> 발산 판정";

            return r;
        }

        // lane 경로는 차량과 같이 움직이는 rolling horizon 이라 종점이 곧
        // "앞으로 본 데까지" 일 뿐이다. 도착으로 처리하면 매번 멈춘다.
        r.goal = (source_ != kau_msgs::msg::KauPath::SRC_LANE) &&
                 curve_->finished(track_, goal_tol_);

        r.s  = track_.s;

        r.ok = true;

        return r;
    }


    const std::shared_ptr<Curve> & curve() const
    {
        return curve_;
    }

    // Lane Detection 이 "실제로 본 구간" [cm]. 그 외 source 는 0.
    double validLength() const
    {
        return valid_length_;
    }

    // 경로가 새로 들어올 때마다 증가. 시각화 재발행 판정용.
    uint32_t pathId() const
    {
        return path_id_;
    }

    // 경로와 pose 의 기준 프레임 (시각화용).
    const std::string & frame() const
    {
        return identity_ ? base_frame_ : map_frame_;
    }

private:
    void onPath(kau_msgs::msg::KauPath::SharedPtr msg, bool is_fallback)
    {
        // 주 경로가 살아 있는 동안은 fallback 을 무시한다.
        if (is_fallback && !from_fallback_ && curve_ &&
            (node_->now() - stamp_).seconds() < path_timeout_)
        {
            return;
        }

        std::string reason;

        if (!validateKauPath(*msg, reason))
        {
            RCLCPP_ERROR_THROTTLE(
                node_->get_logger(), *node_->get_clock(), 2000,
                "KauPath 무결성 검사 실패 -> 무시: %s", reason.c_str());

            return;
        }

        const std::string & want = frame();

        if (msg->header.frame_id != want)
        {
            RCLCPP_ERROR_THROTTLE(
                node_->get_logger(), *node_->get_clock(), 2000,
                "경로 frame_id='%s' 이지만 '%s' 기준으로 추종한다. "
                "발행측이 프레임을 맞추거나 pose_source 를 바꿀 것.",
                msg->header.frame_id.c_str(), want.c_str());

            return;
        }


        Curve cv = curveFromKauPath(*msg);

        if (!cv.windowSafe())
        {
            // 폐곡선 segment 1 개가 전장의 절반 이상 -> 8.4 최소표현이 뒤집힌다
            RCLCPP_WARN_THROTTLE(
                node_->get_logger(), *node_->get_clock(), 5000,
                "폐곡선인데 segment 가 %d 개뿐이다 (nseg >= 3 필요).",
                cv.nseg());
        }

        curve_ = std::make_shared<Curve>(std::move(cv));

        source_        = msg->source;

        valid_length_  = msg->valid_length;

        from_fallback_ = is_fallback;

        stamp_         = node_->now();

        ++path_id_;

        // 새 경로 -> 다음 tick 에 전역 재탐색 (8.3)
        track_ = TrackState{};

        RCLCPP_INFO_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 3000,
            "경로 수신 (%s): nseg=%d 길이=%.1f cm %s valid=%.0f cm",
            is_fallback ? "fallback" : "primary",
            curve_->nseg(), curve_->length(),
            curve_->closed() ? "폐곡선" : "개곡선", valid_length_);
    }

    static std::string fmt(double v)
    {
        char buf[32];

        std::snprintf(buf, sizeof(buf), "%.2f", v);

        return std::string(buf);
    }


    // 이보다 큰 age 는 "끊김" 이 아니라 "시계가 아직 안 잡힘" 으로 본다
    static constexpr double CLOCK_SANITY_S = 60.0;

    rclcpp::Node * node_;

    std::string path_topic_;
    std::string fallback_topic_;
    std::string map_frame_;
    std::string base_frame_;

    bool   path_latched_     = false;
    bool   fallback_latched_ = true;
    bool   identity_         = false;

    double tf_timeout_   = 0.2;
    double path_timeout_ = 1.0;
    double cte_abort_    = 200.0;
    double goal_tol_     = 10.0;
    double rear_offset_  = -9.0;

    std::shared_ptr<Curve> curve_;
    TrackState             track_;

    uint8_t  source_        = 0;
    double   valid_length_  = 0.0;
    bool     from_fallback_ = false;
    uint32_t path_id_       = 0;

    rclcpp::Time stamp_{0, 0, RCL_ROS_TIME};

    std::unique_ptr<tf2_ros::Buffer>            tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::Subscription<kau_msgs::msg::KauPath>::SharedPtr path_sub_;
    rclcpp::Subscription<kau_msgs::msg::KauPath>::SharedPtr fallback_sub_;
};

}  // namespace control
}  // namespace kau

#endif  // KAU_CONTROL__PATH_TRACKER_HPP_
