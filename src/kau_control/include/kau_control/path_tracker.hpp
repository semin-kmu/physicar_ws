// ====================================================================
// path_tracker.hpp
//
// speed_controller / steer_controller 가 공유하는 "경로 추종 상태".
//
//     KauPath 구독 (3 순위: local -> global -> lane)  ->  Curve
//     차량 pose (TF map->base, 또는 경로가 이미 차량 프레임이면 원점)
//     최근접점 추적 (8.3 전역 -> 8.4 국소 window) + cross track error
//
// 두 노드가 이걸 각자 하나씩 들고 돈다. 서로를 구독하지 않는다.
//
// 경로 우선순위 (매 tick 위에서부터 훑어 첫 번째로 "쓸 수 있는" 것을 고른다)
//     1. path_topic          기본 /path/local   map 프레임, TF 필요
//     2. fallback_path_topic 기본 /path/global  map 프레임, TF 필요
//     3. lane_path_topic     기본 /lane/center  lane_frame, TF 불필요
//
// "쓸 수 있다" = 경로를 받아뒀고 + timeout 안에 갱신됐고 + pose 를 얻을 수
// 있다. 1·2 는 map 프레임이라 측위(map->base TF)가 끊기면 경로가 아무리
// 신선해도 못 쓴다. 그때 3 번(lane, identity pose)으로 내려간다.
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

    // rad, 차량 yaw - 경로 접선. 좌측 +.
    // 제어에는 쓰지 않는다 (Pure Pursuit 는 LookAhead 점만 본다). cte 와
    // 같은 호출에서 어차피 나오는 값이라 디버깅용으로 같이 실어 보낸다.
    double head_err = 0.0;
};


class PathTracker
{
public:
    explicit PathTracker(rclcpp::Node * node)
    : node_(node)
    {
        // --- 1 순위: local path ---
        node_->declare_parameter<std::string>("path_topic", "/path/local");
        node_->declare_parameter<bool>("path_latched", false);
        node_->declare_parameter<double>("path_timeout", 1.0);

        // --- 2 순위: global path ---
        node_->declare_parameter<std::string>(
            "fallback_path_topic", "/path/global");
        node_->declare_parameter<bool>("fallback_path_latched", true);

        // latched 1 회 발행이면 age 가 계속 자라므로 기본은 만료 없음(0).
        // global_path 를 rate>0 으로 띄우고 "발행자가 죽으면 lane 으로" 를
        // 원하면 여기에 초 단위 값을 준다.
        node_->declare_parameter<double>("fallback_path_timeout", 0.0);

        // --- 3 순위: lane detection ---
        // 빈 문자열이면 이 순위를 아예 쓰지 않는다.
        node_->declare_parameter<std::string>(
            "lane_path_topic", "/lane/center");
        node_->declare_parameter<bool>("lane_path_latched", false);
        node_->declare_parameter<double>("lane_path_timeout", 0.5);

        // lane 경로가 실려 오는 프레임. kau_lane_detection 의 path_frame_id 와
        // 반드시 같아야 한다. base_frame 과 별개인 이유는 TF 조회용 프레임
        // (base_footprint) 과 lane 발행 프레임 (base_link) 이 다를 수 있기
        // 때문이다. 둘은 z 만 다르므로 평면 추종 결과는 같다.
        node_->declare_parameter<std::string>("lane_frame", "base_link");

        node_->declare_parameter<std::string>("map_frame", "map");
        node_->declare_parameter<std::string>("base_frame", "base_footprint");

        // "tf"       : map -> base_frame TF 로 pose 를 얻는다 (전역 경로용)
        // "identity" : 경로가 이미 차량 프레임에 있다. TF 를 보지 않는다.
        //
        // 1·2 순위는 pose_source 를 따르고, 3 순위(lane)만 따로 둔다.
        // lane 경로는 차량 프레임으로 오는 것이 존재 이유이기 때문이다.
        node_->declare_parameter<std::string>("pose_source", "tf");
        node_->declare_parameter<std::string>(
            "lane_pose_source", "identity");

        node_->declare_parameter<double>("tf_timeout", 0.2);
        node_->declare_parameter<double>("cte_abort_cm", 200.0);
        node_->declare_parameter<double>("goal_tol_cm", 10.0);
        node_->declare_parameter<double>("vehicle.rear_axle_offset_cm", -9.0);

        load();


        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());

        tf_listener_ =
            std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);


        const bool identity =
            (node_->get_parameter("pose_source").as_string() == "identity");

        const bool lane_identity =
            (node_->get_parameter("lane_pose_source").as_string() ==
             "identity");

        slots_[PRIMARY] = {
            "primary",
            node_->get_parameter("path_topic").as_string(),
            node_->get_parameter("path_latched").as_bool(),
            node_->get_parameter("path_timeout").as_double(),
            identity};

        slots_[FALLBACK] = {
            "fallback",
            node_->get_parameter("fallback_path_topic").as_string(),
            node_->get_parameter("fallback_path_latched").as_bool(),
            node_->get_parameter("fallback_path_timeout").as_double(),
            identity};

        slots_[LANE] = {
            "lane",
            node_->get_parameter("lane_path_topic").as_string(),
            node_->get_parameter("lane_path_latched").as_bool(),
            node_->get_parameter("lane_path_timeout").as_double(),
            lane_identity,
            true};


        // QoS 는 topic 마다 다르다 (kau_msgs/README.md).
        // TRANSIENT_LOCAL 구독자는 VOLATILE 발행자와 호환되지 않는다.
        // 전부 latched 로 구독하면 /lane/center 가 조용히 끊긴다.
        auto qos_of = [](bool latched)
            {
                auto q = rclcpp::QoS(1).reliable();

                return latched ? q.transient_local() : q.durability_volatile();
            };

        for (int i = 0; i < NSLOT; ++i)
        {
            Slot & s = slots_[i];

            if (s.topic.empty())
            {
                continue;
            }

            // 같은 topic 을 두 순위에 걸면 위쪽만 남긴다.
            bool dup = false;

            for (int j = 0; j < i; ++j)
            {
                dup = dup || (slots_[j].sub && slots_[j].topic == s.topic);
            }

            if (dup)
            {
                s.topic.clear();

                continue;
            }

            s.sub = node_->create_subscription<kau_msgs::msg::KauPath>(
                s.topic, qos_of(s.latched),
                [this, i](kau_msgs::msg::KauPath::SharedPtr m)
                {
                    onPath(std::move(m), i);
                });
        }

        RCLCPP_INFO(
            node_->get_logger(),
            "경로 우선순위: 1) %s  2) %s  3) %s",
            slotDesc(PRIMARY).c_str(), slotDesc(FALLBACK).c_str(),
            slotDesc(LANE).c_str());
    }

    // 튜닝값만 다시 읽는다. topic / QoS / pose_source 는 구독 생성 시점에 굳는다.
    void load()
    {
        map_frame_    = node_->get_parameter("map_frame").as_string();
        base_frame_   = node_->get_parameter("base_frame").as_string();
        lane_frame_   = node_->get_parameter("lane_frame").as_string();

        tf_timeout_   = node_->get_parameter("tf_timeout").as_double();
        cte_abort_    = node_->get_parameter("cte_abort_cm").as_double();
        goal_tol_     = node_->get_parameter("goal_tol_cm").as_double();
        rear_offset_  =
            node_->get_parameter("vehicle.rear_axle_offset_cm").as_double();

        // timeout 은 주행 중에도 바꿀 수 있게 열어 둔다 (topic 과 달리 굳지 않음).
        slots_[PRIMARY].timeout =
            node_->get_parameter("path_timeout").as_double();
        slots_[FALLBACK].timeout =
            node_->get_parameter("fallback_path_timeout").as_double();
        slots_[LANE].timeout =
            node_->get_parameter("lane_path_timeout").as_double();
    }


    TrackResult update()
    {
        TrackResult r;

        const rclcpp::Time t = node_->now();


        // --- 우선순위대로 "쓸 수 있는" 경로 고르기 ---
        int    chosen = -1;
        double px = 0.0;
        double py = 0.0;
        double pyaw = 0.0;

        std::string why;      // 가장 높은 순위가 못 쓰인 이유
        bool        quiet = false;

        for (int i = 0; i < NSLOT; ++i)
        {
            Slot & s = slots_[i];

            if (!s.curve || s.curve->empty())
            {
                continue;
            }

            // 신선도. timeout <= 0 이면 만료 없음 (latched 1 회 발행용).
            if (s.timeout > 0.0)
            {
                const double age = (t - s.stamp).seconds();

                // 기동 직후 node clock 이 system -> sim 으로 넘어가면 두 시각의
                // 기준이 달라 age 가 말도 안 되게 튄다. 끊긴 게 아니라 시계가
                // 아직 안 잡힌 것이므로 다시 찍고 한 tick 쉰다.
                if (age < 0.0 || age > CLOCK_SANITY_S)
                {
                    s.stamp = t;

                    quiet = true;

                    continue;
                }

                if (age > s.timeout)
                {
                    if (why.empty())
                    {
                        why = std::string(s.name) + " 경로가 " + fmt(age) +
                              " s 갱신되지 않았다";
                    }

                    continue;
                }
            }

            std::string pose_why;

            if (!pose(s, t, px, py, pyaw, pose_why))
            {
                if (why.empty())
                {
                    why = pose_why;
                }

                continue;
            }

            chosen = i;

            break;
        }

        if (chosen < 0)
        {
            if (!quiet || !why.empty())
            {
                r.reason = why.empty() ? "경로 없음" : why;
            }

            active_ = -1;

            return r;
        }


        if (chosen != active_)
        {
            const std::string note =
                why.empty() ? std::string() : " · 상위 순위 불가: " + why;

            RCLCPP_WARN(
                node_->get_logger(), "경로 소스 -> %s (%s)%s",
                slots_[chosen].name, slots_[chosen].topic.c_str(),
                note.c_str());

            active_ = chosen;

            // 새 곡선이라 직전 s 가 무의미하다 -> 다음 질의는 전역 탐색 (8.3)
            track_ = TrackState{};

            ++path_id_;
        }

        Slot & s = slots_[chosen];

        curve_ = s.curve;

        r.x   = px;
        r.y   = py;
        r.yaw = pyaw;


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
            r.reason =
                "cross track error " + fmt(r.cte) + " cm -> 발산 판정";

            return r;
        }

        // lane 경로는 차량과 같이 움직이는 rolling horizon 이라 종점이 곧
        // "앞으로 본 데까지" 일 뿐이다. 도착으로 처리하면 매번 멈춘다.
        r.goal = (s.source != kau_msgs::msg::KauPath::SRC_LANE) &&
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
        return active_ < 0 ? 0.0 : slots_[active_].valid_length;
    }

    // 경로가 새로 들어오거나 소스가 바뀔 때마다 증가. 시각화 재발행 판정용.
    uint32_t pathId() const
    {
        return path_id_;
    }

    // 지금 따라가는 경로의 기준 프레임 (시각화용).
    const std::string & frame() const
    {
        return frameOf(active_ < 0 ? slots_[PRIMARY] : slots_[active_]);
    }

    // 지금 따라가는 순위 이름 ("primary" / "fallback" / "lane" / "-").
    const char * activeName() const
    {
        return active_ < 0 ? "-" : slots_[active_].name;
    }

    // 지금 따라가는 경로의 KauPath.source (SRC_GLOBAL / SRC_LOCAL / SRC_LANE).
    // 순위(primary/fallback/lane)와 별개다. 설정에 따라 primary 순위로
    // global 경로가 올 수도 있으므로 발행측이 붙인 값을 그대로 쓴다.
    uint8_t source() const
    {
        return active_ < 0 ? 0 : slots_[active_].source;
    }

private:
    enum : int
    {
        PRIMARY  = 0,
        FALLBACK = 1,
        LANE     = 2,
        NSLOT    = 3
    };

    // 경로 한 순위. topic 이 비면 그 순위는 없는 것으로 친다.
    struct Slot
    {
        const char * name = "";

        std::string topic;

        bool   latched  = false;

        double timeout  = 0.0;    // s, 0 이하면 만료 없음

        bool   identity = false;  // true 면 TF 대신 차량 원점

        bool   lane     = false;  // true 면 프레임 기준이 lane_frame

        std::shared_ptr<Curve> curve;

        rclcpp::Time stamp{0, 0, RCL_ROS_TIME};

        uint8_t source       = 0;

        double  valid_length = 0.0;

        rclcpp::Subscription<kau_msgs::msg::KauPath>::SharedPtr sub;
    };


    // 이 순위가 기대하는 경로 프레임.
    const std::string & frameOf(const Slot & s) const
    {
        if (!s.identity)
        {
            return map_frame_;
        }

        return s.lane ? lane_frame_ : base_frame_;
    }


    std::string slotDesc(int i) const
    {
        const Slot & s = slots_[i];

        if (s.topic.empty())
        {
            return "(없음)";
        }

        return s.topic + (s.identity ? " [identity]" : " [tf]");
    }


    // 차량 pose. 못 얻으면 why 에 이유를 담고 false.
    bool pose(
        const Slot & s,
        const rclcpp::Time & t,
        double & x,
        double & y,
        double & yaw,
        std::string & why) const
    {
        // Pure Pursuit 기준점은 후륜축 중심인데 base_frame 원점은 휠베이스
        // 중앙이다. 이 보정을 빼면 코너에서 계속 안쪽으로 파고든다.
        const double off = rear_offset_;

        if (s.identity)
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


    void onPath(kau_msgs::msg::KauPath::SharedPtr msg, int idx)
    {
        Slot & s = slots_[idx];

        std::string reason;

        if (!validateKauPath(*msg, reason))
        {
            RCLCPP_ERROR_THROTTLE(
                node_->get_logger(), *node_->get_clock(), 2000,
                "[%s] KauPath 무결성 검사 실패 -> 무시: %s",
                s.name, reason.c_str());

            return;
        }

        const std::string & want = frameOf(s);

        if (msg->header.frame_id != want)
        {
            RCLCPP_ERROR_THROTTLE(
                node_->get_logger(), *node_->get_clock(), 2000,
                "[%s] 경로 frame_id='%s' 이지만 '%s' 기준으로 추종한다. "
                "발행측이 프레임을 맞추거나 pose_source 를 바꿀 것.",
                s.name, msg->header.frame_id.c_str(), want.c_str());

            return;
        }


        Curve cv = curveFromKauPath(*msg);

        if (!cv.windowSafe())
        {
            // 폐곡선 segment 1 개가 전장의 절반 이상 -> 8.4 최소표현이 뒤집힌다
            RCLCPP_WARN_THROTTLE(
                node_->get_logger(), *node_->get_clock(), 5000,
                "[%s] 폐곡선인데 segment 가 %d 개뿐이다 (nseg >= 3 필요).",
                s.name, cv.nseg());
        }

        s.curve = std::make_shared<Curve>(std::move(cv));

        s.source       = msg->source;

        s.valid_length = msg->valid_length;

        s.stamp        = node_->now();

        // 지금 따라가는 순위가 갱신된 것이라면 곡선이 통째로 바뀐 것이므로
        // window 추적 상태를 버린다. 다른 순위의 갱신은 건드리지 않는다
        // (그쪽 메시지가 올 때마다 전역 재탐색을 하면 낭비다).
        if (idx == active_)
        {
            track_ = TrackState{};

            ++path_id_;
        }

        RCLCPP_INFO_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 3000,
            "경로 수신 (%s): nseg=%d 길이=%.1f cm %s valid=%.0f cm",
            s.name, s.curve->nseg(), s.curve->length(),
            s.curve->closed() ? "폐곡선" : "개곡선", s.valid_length);
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

    std::string map_frame_;
    std::string base_frame_;
    std::string lane_frame_;

    double tf_timeout_   = 0.2;
    double cte_abort_    = 200.0;
    double goal_tol_     = 10.0;
    double rear_offset_  = -9.0;

    Slot slots_[NSLOT];

    int  active_ = -1;          // 지금 따라가는 순위. -1 = 없음

    std::shared_ptr<Curve> curve_;
    TrackState             track_;

    uint32_t path_id_ = 0;

    std::unique_ptr<tf2_ros::Buffer>            tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace control
}  // namespace kau

#endif  // KAU_CONTROL__PATH_TRACKER_HPP_
