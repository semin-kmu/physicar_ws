// ====================================================================
// lane_tracker.hpp
//
// lane_speed_controller / lane_steer_controller 가 공유하는 "차선 추종 상태".
//
//     /lane/center (KauPath) 구독  ->  Curve
//     차량 pose (경로가 이미 차량 프레임이면 원점, 아니면 TF map->base)
//     최근접점 추적 (8.3 전역 -> 8.4 국소 window) + cross track error
//
// kau_control 의 path_tracker.hpp 와 알고리즘은 **같다.** 다른 점은 하나뿐:
// 경로 소스가 lane **하나**다. local / global 우선순위도, 그 fallback 도
// 없다. 차선을 놓치면 다른 경로로 내려가지 않고 그냥 선다 -- 그것이 이
// 패키지의 전제다 (README §1).
//
// 두 노드가 이걸 각자 하나씩 들고 돈다. 서로를 구독하지 않는다.
//
// pose_source
//     identity  경로가 차량 프레임(base_link)으로 들어온다. 차량이 곧 그
//               프레임의 원점이므로 TF 를 보지 않는다. **측위 불필요.**
//               kau_lane_detection 의 path_frame_id 가 base_link 여야 한다.
//     tf        경로가 map 프레임으로 들어온다. map->base TF 를 본다.
//               kau_lane_detection 의 path_frame_id 가 map 일 때 (기본 설정).
//
// 단위 규약: 내부는 cm / rad. ROS 경계에서만 m 로 바꾼다.
// TF 는 발행하지 않는다.
// ====================================================================

#ifndef KAU_CONTROL_LANE__LANE_TRACKER_HPP_
#define KAU_CONTROL_LANE__LANE_TRACKER_HPP_

#include <cmath>
#include <cstdio>
#include <memory>
#include <stdexcept>
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

#include "kau_control_lane/curve.hpp"
#include "kau_control_lane/kau_path.hpp"
#include "kau_control_lane/params.hpp"


namespace kau
{
namespace control_lane
{

// update() 한 tick 의 결과.
//
//     ok == false  -> 노드는 즉시 0 을 발행해야 한다 (reason 이 비면 조용히)
//
// goal 은 없다. 차선 경로는 차량과 같이 움직이는 rolling horizon 이라
// 종점이 "앞으로 본 데까지" 일 뿐이다. 도착으로 처리하면 매번 멈춘다.
struct TrackResult
{
    bool ok = false;

    std::string reason;

    double x   = 0.0;   // cm, 후륜축 기준
    double y   = 0.0;   // cm
    double yaw = 0.0;   // rad

    double s   = 0.0;   // cm, 경로 시작점부터의 호길이
    double cte = 0.0;   // cm, 좌측 +

    // rad, 차량 yaw - 경로 접선. 좌측 +.
    // 제어에는 쓰지 않는다 (Pure Pursuit 는 LookAhead 점만 본다). cte 와
    // 같은 호출에서 어차피 나오는 값이라 디버깅용으로 같이 실어 보낸다.
    double head_err = 0.0;
};


class LaneTracker
{
public:
    explicit LaneTracker(rclcpp::Node * node)
    : node_(node)
    {
        node_->declare_parameter<std::string>("lane.topic", "/lane/center");
        node_->declare_parameter<bool>("lane.latched", false);
        node_->declare_parameter<double>("lane.timeout", 0.5);
        node_->declare_parameter<std::string>("lane.pose_source", "identity");
        node_->declare_parameter<std::string>("lane.frame", "base_link");

        node_->declare_parameter<std::string>("map_frame", "map");
        node_->declare_parameter<std::string>("base_frame", "base_footprint");

        node_->declare_parameter<double>("tf_timeout", 0.2);
        node_->declare_parameter<double>("cte_abort_cm", 40.0);
        node_->declare_parameter<double>("vehicle.rear_axle_offset_cm", -9.0);


        topic_   = node_->get_parameter("lane.topic").as_string();
        latched_ = node_->get_parameter("lane.latched").as_bool();
        frame_   = node_->get_parameter("lane.frame").as_string();

        const std::string pose_source =
            node_->get_parameter("lane.pose_source").as_string();

        // 오타를 조용히 넘기면 "왜 경로를 안 따라가지" 로 시간을 날린다.
        // 기동을 접는 편이 낫다.
        if (pose_source != "identity" && pose_source != "tf")
        {
            RCLCPP_FATAL(
                node_->get_logger(),
                "lane.pose_source '%s' 를 모른다. identity | tf",
                pose_source.c_str());

            throw std::runtime_error(
                "lane.pose_source 를 모른다: " + pose_source);
        }

        identity_ = (pose_source == "identity");

        if (topic_.empty())
        {
            RCLCPP_FATAL(node_->get_logger(), "lane.topic 이 비어 있다.");

            throw std::runtime_error("lane.topic 이 비었다");
        }

        load();


        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());

        tf_listener_ =
            std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);


        // QoS 는 발행측과 맞아야 한다 (kau_msgs/README.md).
        // TRANSIENT_LOCAL 구독자는 VOLATILE 발행자와 호환되지 않는다.
        // /lane/center 는 VOLATILE 이므로 latched 를 켜면 조용히 끊긴다.
        auto qos = rclcpp::QoS(1).reliable();

        if (latched_)
        {
            qos = qos.transient_local();
        }
        else
        {
            qos = qos.durability_volatile();
        }

        sub_ = node_->create_subscription<kau_msgs::msg::KauPath>(
            topic_, qos,
            [this](kau_msgs::msg::KauPath::SharedPtr m)
            {
                onPath(std::move(m));
            });

        RCLCPP_INFO(
            node_->get_logger(),
            "차선 경로 구독: %s [%s] frame=%s timeout=%.2f s",
            topic_.c_str(), identity_ ? "identity" : "tf",
            frame().c_str(), timeout_);
    }


    // 튜닝값만 다시 읽는다.
    // topic / QoS / pose_source / frame 은 구독 생성 시점에 굳는다.
    void load()
    {
        map_frame_   = node_->get_parameter("map_frame").as_string();
        base_frame_  = node_->get_parameter("base_frame").as_string();

        tf_timeout_  = node_->get_parameter("tf_timeout").as_double();
        cte_abort_   = node_->get_parameter("cte_abort_cm").as_double();
        rear_offset_ =
            node_->get_parameter("vehicle.rear_axle_offset_cm").as_double();

        // timeout 은 주행 중에도 바꿀 수 있게 열어 둔다.
        timeout_ = node_->get_parameter("lane.timeout").as_double();
    }


    TrackResult update()
    {
        TrackResult r;

        const rclcpp::Time t = node_->now();

        if (!curve_ || curve_->empty())
        {
            r.reason = "차선 경로 없음";

            return r;
        }

        // 신선도. timeout <= 0 이면 만료 없음.
        if (timeout_ > 0.0)
        {
            const double age = (t - stamp_).seconds();

            // 기동 직후 node clock 이 system -> sim 으로 넘어가면 두 시각의
            // 기준이 달라 age 가 말도 안 되게 튄다. 끊긴 게 아니라 시계가
            // 아직 안 잡힌 것이므로 다시 찍고 한 tick 쉰다.
            if (age < 0.0 || age > CLOCK_SANITY_S)
            {
                stamp_ = t;

                return r;
            }

            if (age > timeout_)
            {
                r.reason =
                    "차선 경로가 " + fmt(age) + " s 갱신되지 않았다";

                return r;
            }
        }

        std::string pose_why;

        if (!pose(t, r.x, r.y, r.yaw, pose_why))
        {
            r.reason = pose_why;

            return r;
        }


        // 첫 질의만 전역(8.3), 이후 국소 window(8.4)
        track_ = curve_->nearest(Point2{r.x, r.y}, track_);

        if (!track_.valid)
        {
            r.reason = "최근접점 탐색 실패";

            return r;
        }

        curve_->errors(r.x, r.y, r.yaw, track_, r.cte, r.head_err);

        if (std::abs(r.cte) > cte_abort_)
        {
            r.reason = "cross track error " + fmt(r.cte) + " cm -> 발산 판정";

            return r;
        }

        r.s = track_.s;

        r.ok = true;

        return r;
    }


    const std::shared_ptr<Curve> & curve() const
    {
        return curve_;
    }

    // Lane Detection 이 "실제로 본 구간" [cm].
    double validLength() const
    {
        return valid_length_;
    }

    // 경로가 새로 들어올 때마다 증가. 시각화 재발행 판정용.
    uint32_t pathId() const
    {
        return path_id_;
    }

    // 지금 따라가는 경로의 기준 프레임 (시각화용).
    const std::string & frame() const
    {
        if (!identity_)
        {
            return map_frame_;
        }

        // lane.frame 은 경로 발행측 프레임이 TF 조회용 base_frame 과 다를 때만
        // 채운다 (예: lane 은 base_link, base_frame 은 base_footprint --
        // 둘은 z 만 달라 평면 추종 결과가 같다).
        return frame_.empty() ? base_frame_ : frame_;
    }

    // 발행측이 붙인 KauPath.source (정상이면 SRC_LANE).
    // 다른 값이 와도 정직하게 그대로 내보낸다.
    uint8_t source() const
    {
        return source_;
    }

private:
    // 차량 pose. 못 얻으면 why 에 이유를 담고 false.
    bool pose(
        const rclcpp::Time & t,
        double & x,
        double & y,
        double & yaw,
        std::string & why) const
    {
        // Pure Pursuit 기준점은 후륜축 중심인데 base_frame 원점은 휠베이스
        // 중앙이다. 이 보정을 빼면 코너에서 계속 안쪽으로 파고든다.
        const double off = rear_offset_;

        if (identity_)
        {
            // 차량이 곧 경로 프레임의 원점이다. 후륜축만 뒤로 물러나 있다.
            x   = off;
            y   = 0.0;
            yaw = 0.0;

            return true;
        }

        geometry_msgs::msg::TransformStamped tf;

        try
        {
            tf = tf_buffer_->lookupTransform(
                map_frame_, base_frame_, tf2::TimePointZero);
        }
        catch (const tf2::TransformException & ex)
        {
            why = std::string("TF 조회 실패: ") + ex.what();

            return false;
        }

        const double tf_age = (t - rclcpp::Time(tf.header.stamp)).seconds();

        if (tf_age > tf_timeout_)
        {
            why = "TF 가 " + fmt(tf_age) + " s 지연됐다 (측위 끊김)";

            return false;
        }

        yaw = tf2::getYaw(tf.transform.rotation);

        x = tf.transform.translation.x * CM_PER_M + off * std::cos(yaw);

        y = tf.transform.translation.y * CM_PER_M + off * std::sin(yaw);

        return true;
    }


    void onPath(kau_msgs::msg::KauPath::SharedPtr msg)
    {
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
            // 여기서 막히면 차가 영영 안 움직인다. 고칠 곳을 같이 적는다 --
            // 프레임 짝이 맞는 조합은 둘뿐이다.
            //     lane.pose_source=identity <-> path_frame_id=base_link
            //     lane.pose_source=tf       <-> path_frame_id=map
            RCLCPP_ERROR_THROTTLE(
                node_->get_logger(), *node_->get_clock(), 2000,
                "경로 frame_id='%s' 이지만 '%s' 기준으로 추종한다. "
                "kau_lane_detection 의 path_frame_id 를 '%s' 로 맞추거나, "
                "이 노드의 lane.pose_source 를 '%s' 로 바꿀 것.",
                msg->header.frame_id.c_str(), want.c_str(), want.c_str(),
                identity_ ? "tf" : "identity");

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

        source_ = msg->source;

        valid_length_ = msg->valid_length;

        stamp_ = node_->now();

        // 매 프레임 새로 지은 곡선이다. 직전 s 가 그대로 유효하지 않으므로
        // window 추적 상태를 버린다 (다음 질의는 전역 탐색).
        track_ = TrackState{};

        ++path_id_;

        RCLCPP_INFO_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 3000,
            "차선 경로 수신: nseg=%d 길이=%.1f cm %s valid=%.0f cm",
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

    std::string topic_;
    std::string frame_;
    bool        latched_  = false;
    bool        identity_ = true;

    std::string map_frame_;
    std::string base_frame_;

    double timeout_      = 0.5;
    double tf_timeout_   = 0.2;
    double cte_abort_    = 40.0;
    double rear_offset_  = -9.0;

    std::shared_ptr<Curve> curve_;
    TrackState             track_;

    rclcpp::Time stamp_{0, 0, RCL_ROS_TIME};

    uint8_t  source_       = 0;
    double   valid_length_ = 0.0;
    uint32_t path_id_      = 0;

    rclcpp::Subscription<kau_msgs::msg::KauPath>::SharedPtr sub_;

    std::unique_ptr<tf2_ros::Buffer>            tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace control_lane
}  // namespace kau

#endif  // KAU_CONTROL_LANE__LANE_TRACKER_HPP_
