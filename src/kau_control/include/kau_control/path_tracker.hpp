// ====================================================================
// path_tracker.hpp
//
// speed_controller / steer_controller 가 공유하는 "경로 추종 상태".
//
//     KauPath 구독 (프리셋 순서대로)  ->  Curve
//     차량 pose (TF map->base, 또는 경로가 이미 차량 프레임이면 원점)
//     최근접점 추적 (8.3 전역 -> 8.4 국소 window) + cross track error
//
// 두 노드가 이걸 각자 하나씩 들고 돈다. 서로를 구독하지 않는다.
//
// 경로 소스는 셋이고 이름이 곧 yaml 의 path.sources.<이름> 키다.
//     local   기본 /path/local
//     global  기본 /path/global
//     lane    기본 /lane/center
//
// 쓸 소스와 그 순서는 path.mode 프리셋이 정한다 (modes() 참고).
//     normal      local -> global -> lane   평상시 주행
//     steer_test  global 만                 조향 제어기 시험
//     lane_only   lane 만                   차선 추종 단독
//
// 프리셋에 없는 소스는 구독조차 하지 않는다. 매 tick 프리셋 순서대로 훑어
// 첫 번째로 "쓸 수 있는" 것을 고른다.
//
// "쓸 수 있다" = 경로를 받아뒀고 + timeout 안에 갱신됐고 + pose 를 얻을 수
// 있다. pose_source=tf 인 소스는 측위(map->base TF)가 끊기면 경로가 아무리
// 신선해도 못 쓴다. identity 인 소스는 TF 를 보지 않으므로 살아남는다.
//
// 단위 규약: 내부는 cm / rad. ROS 경계에서만 m 로 바꾼다.
// TF 는 발행하지 않는다 (map->odom 은 Cartographer, odom->base 는 EKF).
// ====================================================================

#ifndef KAU_CONTROL__PATH_TRACKER_HPP_
#define KAU_CONTROL__PATH_TRACKER_HPP_

#include <cmath>
#include <cstdio>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

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
        // 경로 소스 프리셋. 쓸 소스와 그 순서를 정한다 (modes()).
        node_->declare_parameter<std::string>("path.mode", "normal");

        // 소스별 설정. 이름이 곧 yaml 의 path.sources.<이름> 키다.
        //            소스     topic            latched timeout pose      frame
        declareSource(LOCAL,  "/path/local",  false,  1.0,   "tf", "");
        declareSource(GLOBAL, "/path/global", true,   3.0,   "tf", "");
        declareSource(LANE,   "/lane/center", false,  0.5,   "tf", "base_link");

        node_->declare_parameter<std::string>("map_frame", "map");
        node_->declare_parameter<std::string>("base_frame", "base_footprint");

        node_->declare_parameter<double>("tf_timeout", 0.2);
        node_->declare_parameter<double>("cte_abort_cm", 200.0);
        node_->declare_parameter<double>("goal_tol_cm", 10.0);
        node_->declare_parameter<double>("vehicle.rear_axle_offset_cm", -9.0);


        for (int i = 0; i < NSRC; ++i)
        {
            slots_[i] = loadSource(i);
        }

        load();


        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());

        tf_listener_ =
            std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);


        // --- 프리셋 해석 ---
        //
        // 모르는 이름을 조용히 넘기면 "왜 경로를 안 따라가지" 로 시간을
        // 날린다. 기동을 접는 편이 낫다.
        const std::string mode = node_->get_parameter("path.mode").as_string();

        const auto it = modes().find(mode);

        if (it == modes().end())
        {
            RCLCPP_FATAL(
                node_->get_logger(),
                "path.mode '%s' 를 모른다. 있는 것: %s",
                mode.c_str(), modeList().c_str());

            throw std::runtime_error("path.mode 를 모른다: " + mode);
        }


        // QoS 는 topic 마다 다르다 (kau_msgs/README.md).
        // TRANSIENT_LOCAL 구독자는 VOLATILE 발행자와 호환되지 않는다.
        // 전부 latched 로 구독하면 /lane/center 가 조용히 끊긴다.
        auto qos_of = [](bool latched)
            {
                auto q = rclcpp::QoS(1).reliable();

                return latched ? q.transient_local() : q.durability_volatile();
            };


        // --- 프리셋에 든 소스만 구독한다 ---
        for (int i : it->second)
        {
            Slot & s = slots_[i];

            if (s.topic.empty())
            {
                RCLCPP_FATAL(
                    node_->get_logger(),
                    "path.mode '%s' 가 %s 소스를 쓰는데 "
                    "path.sources.%s.topic 이 비어 있다.",
                    mode.c_str(), s.name, s.name);

                throw std::runtime_error(
                    std::string("path.sources.") + s.name + ".topic 이 비었다");
            }

            // 같은 topic 을 두 순위에 걸면 위쪽만 남긴다.
            bool dup = false;

            for (int j : order_)
            {
                dup = dup || (slots_[j].topic == s.topic);
            }

            if (dup)
            {
                RCLCPP_WARN(
                    node_->get_logger(),
                    "%s 소스가 상위 순위와 같은 topic(%s) 이라 건너뛴다.",
                    s.name, s.topic.c_str());

                continue;
            }

            s.sub = node_->create_subscription<kau_msgs::msg::KauPath>(
                s.topic, qos_of(s.latched),
                [this, i](kau_msgs::msg::KauPath::SharedPtr m)
                {
                    onPath(std::move(m), i);
                });

            order_.push_back(i);
        }

        RCLCPP_INFO(
            node_->get_logger(),
            "경로 소스 (path.mode=%s):%s",
            mode.c_str(), orderDesc().c_str());
    }


    // 튜닝값만 다시 읽는다.
    // path.mode / topic / QoS / pose_source 는 구독 생성 시점에 굳는다.
    void load()
    {
        map_frame_    = node_->get_parameter("map_frame").as_string();
        base_frame_   = node_->get_parameter("base_frame").as_string();

        tf_timeout_   = node_->get_parameter("tf_timeout").as_double();
        cte_abort_    = node_->get_parameter("cte_abort_cm").as_double();
        goal_tol_     = node_->get_parameter("goal_tol_cm").as_double();
        rear_offset_  =
            node_->get_parameter("vehicle.rear_axle_offset_cm").as_double();

        // timeout 은 주행 중에도 바꿀 수 있게 열어 둔다 (topic 과 달리 굳지 않음).
        for (int i = 0; i < NSRC; ++i)
        {
            slots_[i].timeout =
                node_->get_parameter(key(i, "timeout")).as_double();
        }
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

        for (int i : order_)
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
    // 아직 아무것도 못 고른 동안은 프리셋 1 순위의 프레임을 쓴다.
    const std::string & frame() const
    {
        const int i = active_ >= 0 ? active_
                    : order_.empty() ? LOCAL : order_.front();

        return frameOf(slots_[i]);
    }

    // 지금 따라가는 소스 이름 ("local" / "global" / "lane" / "-").
    const char * activeName() const
    {
        return active_ < 0 ? "-" : slots_[active_].name;
    }

    // 지금 따라가는 경로의 KauPath.source (SRC_GLOBAL / SRC_LOCAL / SRC_LANE).
    // 소스 이름과 대개 일치하지만 발행측이 붙인 값을 그대로 쓴다 -- 예컨대
    // global 소스에 local 경로를 물려 시험하는 경우까지 정직하게 나간다.
    uint8_t source() const
    {
        return active_ < 0 ? 0 : slots_[active_].source;
    }

private:
    // 경로 소스. 이름이 곧 yaml 의 path.sources.<이름> 키다.
    enum : int
    {
        LOCAL  = 0,
        GLOBAL = 1,
        LANE   = 2,
        NSRC   = 3
    };

    static constexpr const char * SRC_NAME[NSRC] = {"local", "global", "lane"};


    // 프리셋 -> 소스 순서. 목록에 없는 소스는 구독조차 하지 않는다.
    //
    // 새 조합이 필요하면 여기에 한 줄 추가한다. yaml 은 이름만 고르므로
    // 오타는 기동 시 FATAL 로 걸린다.
    static const std::map<std::string, std::vector<int>> & modes()
    {
        static const std::map<std::string, std::vector<int>> M = {
            {"normal",     {LOCAL, GLOBAL, LANE}},   // 평상시 주행
            {"steer_test", {GLOBAL}},                // 조향 제어기 시험
            {"lane_only",  {LANE}},                  // 차선 추종 단독
        };

        return M;
    }

    static std::string modeList()
    {
        std::string out;

        for (const auto & kv : modes())
        {
            out += (out.empty() ? "" : " | ") + kv.first;
        }

        return out;
    }


    // 경로 소스 하나.
    struct Slot
    {
        const char * name = "";

        std::string topic;

        bool   latched  = false;

        double timeout  = 0.0;    // s, 0 이하면 만료 없음

        bool   identity = false;  // true 면 TF 대신 차량 원점

        // identity 일 때 경로가 실려 오는 프레임. 비우면 base_frame.
        std::string frame;

        std::shared_ptr<Curve> curve;

        rclcpp::Time stamp{0, 0, RCL_ROS_TIME};

        uint8_t source       = 0;

        double  valid_length = 0.0;

        rclcpp::Subscription<kau_msgs::msg::KauPath>::SharedPtr sub;
    };


    static std::string key(int i, const char * field)
    {
        return std::string("path.sources.") + SRC_NAME[i] + "." + field;
    }


    void declareSource(
        int i,
        const char * topic,
        bool latched,
        double timeout,
        const char * pose_source,
        const char * frame)
    {
        node_->declare_parameter<std::string>(key(i, "topic"), topic);
        node_->declare_parameter<bool>(key(i, "latched"), latched);
        node_->declare_parameter<double>(key(i, "timeout"), timeout);
        node_->declare_parameter<std::string>(
            key(i, "pose_source"), pose_source);
        node_->declare_parameter<std::string>(key(i, "frame"), frame);
    }


    Slot loadSource(int i) const
    {
        Slot s;

        s.name    = SRC_NAME[i];
        s.topic   = node_->get_parameter(key(i, "topic")).as_string();
        s.latched = node_->get_parameter(key(i, "latched")).as_bool();
        s.timeout = node_->get_parameter(key(i, "timeout")).as_double();
        s.frame   = node_->get_parameter(key(i, "frame")).as_string();

        s.identity =
            node_->get_parameter(key(i, "pose_source")).as_string() ==
            "identity";

        return s;
    }


    // 이 소스가 기대하는 경로 프레임.
    const std::string & frameOf(const Slot & s) const
    {
        if (!s.identity)
        {
            return map_frame_;
        }

        // frame 은 경로 발행측 프레임이 TF 조회용 base_frame 과 다를 때만
        // 채운다 (예: lane 은 base_link, base_frame 은 base_footprint --
        // 둘은 z 만 달라 평면 추종 결과가 같다).
        return s.frame.empty() ? base_frame_ : s.frame;
    }


    // " 1) local /path/local [tf]  2) global /path/global [tf]" 꼴.
    std::string orderDesc() const
    {
        std::string out;

        for (size_t k = 0; k < order_.size(); ++k)
        {
            const Slot & s = slots_[order_[k]];

            out += "  " + std::to_string(k + 1) + ") " + s.name + " " +
                   s.topic + (s.identity ? " [identity]" : " [tf]");
        }

        return out.empty() ? "  (없음)" : out;
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

    double tf_timeout_   = 0.2;
    double cte_abort_    = 200.0;
    double goal_tol_     = 10.0;
    double rear_offset_  = -9.0;

    Slot slots_[NSRC];

    // path.mode 가 정한 소스 순서. 여기 없는 소스는 구독조차 하지 않는다.
    std::vector<int> order_;

    int  active_ = -1;          // 지금 따라가는 소스. -1 = 없음

    std::shared_ptr<Curve> curve_;
    TrackState             track_;

    uint32_t path_id_ = 0;

    std::unique_ptr<tf2_ros::Buffer>            tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace control
}  // namespace kau

#endif  // KAU_CONTROL__PATH_TRACKER_HPP_
