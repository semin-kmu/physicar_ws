// ====================================================================
// path_arbiter_node.cpp
//
// 구독: /lane/center, /path/local (kau_msgs/KauPath),
//       /perception/obstacles (kau_msgs/ObstacleCircleArray)
// 발행: /path/drive (KauPath), /debug/path_arbiter (std_msgs/String)
//
// 발행은 **이벤트 구동** 이다 -- 고른 소스가 발행할 때 그대로 중계한다.
// 타이머로 재발행하지 않는다: 같은 경로를 새 stamp 로 다시 내면 죽은
// 소스가 살아있는 것처럼 보여 제어기의 timeout 안전망이 무력해진다.
// 소스가 끊기면 이 노드도 조용해지고, 제어기가 스스로 선다
// (kau_control_lane 의 "끊기면 다른 데로 안 내려가고 선다" 전제와 같다).
//
// 모드 판정만 타이머로 돈다 -- 회피 중에 /path/local 이 죽으면 발행
// 이벤트가 아예 안 와서 히스테리시스 해제가 멈추기 때문이다.
// ====================================================================

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "kau_msgs/msg/kau_path.hpp"
#include "kau_msgs/msg/obstacle_circle_array.hpp"
#include "kau_path_arbiter/arbiter.hpp"

using kau::path_arbiter::Arbiter;
using kau::path_arbiter::ArbiterParams;
using kau::path_arbiter::Decision;
using kau::path_arbiter::Mode;
using kau::path_arbiter::obstacleRelevant;
using kau::path_arbiter::Source;

class PathArbiterNode : public rclcpp::Node
{
public:
    PathArbiterNode()
    : rclcpp::Node("path_arbiter_node")
    {
        ArbiterParams p;
        p.trigger_min_x_cm = declare_parameter<double>("trigger_min_x_cm", 0.0);
        p.trigger_max_x_cm = declare_parameter<double>("trigger_max_x_cm", 150.0);
        p.trigger_half_width_cm =
            declare_parameter<double>("trigger_half_width_cm", 40.0);
        p.engage_sec = declare_parameter<double>("engage_sec", 0.0);
        p.release_sec = declare_parameter<double>("release_sec", 2.0);
        p.lane_timeout_sec = declare_parameter<double>("lane_timeout_sec", 0.5);
        p.local_timeout_sec = declare_parameter<double>("local_timeout_sec", 1.0);
        p.fallback_to_lane_on_local_loss =
            declare_parameter<bool>("fallback_to_lane_on_local_loss", false);
        params_ = p;
        arbiter_ = std::make_unique<Arbiter>(p);

        output_frame_ = declare_parameter<std::string>("output_frame", "base_footprint");
        lane_topic_ = declare_parameter<std::string>("lane_topic", "/lane/center");
        local_topic_ = declare_parameter<std::string>("local_topic", "/path/local");
        output_topic_ = declare_parameter<std::string>("output_topic", "/path/drive");

        rclcpp::QoS path_qos(rclcpp::KeepLast(1));
        path_qos.reliable();
        lane_sub_ = create_subscription<kau_msgs::msg::KauPath>(
            lane_topic_, path_qos,
            [this](kau_msgs::msg::KauPath::SharedPtr m) { onLane(std::move(m)); });
        local_sub_ = create_subscription<kau_msgs::msg::KauPath>(
            local_topic_, path_qos,
            [this](kau_msgs::msg::KauPath::SharedPtr m) { onLocal(std::move(m)); });

        rclcpp::QoS obs_qos(rclcpp::KeepLast(1));
        obs_qos.best_effort().durability_volatile();
        obstacle_sub_ = create_subscription<kau_msgs::msg::ObstacleCircleArray>(
            "/perception/obstacles", obs_qos,
            [this](kau_msgs::msg::ObstacleCircleArray::SharedPtr m)
            { onObstacles(std::move(m)); });

        drive_pub_ = create_publisher<kau_msgs::msg::KauPath>(output_topic_, path_qos);
        debug_pub_ = create_publisher<std_msgs::msg::String>(
            "/debug/path_arbiter", rclcpp::QoS(1).best_effort());

        obstacle_timeout_sec_ =
            declare_parameter<double>("obstacle_timeout_sec", 1.0);

        const double hz = declare_parameter<double>("mode_hz", 20.0);
        timer_ = create_wall_timer(
            std::chrono::duration<double>(1.0 / hz),
            [this]() { onModeTimer(); });

        RCLCPP_INFO(get_logger(),
            "path_arbiter 시작. %s(장애물 없음) | %s(장애물) -> %s [%s], "
            "release=%.1fs 관심영역 x[%.0f,%.0f] |y|<=%.0fcm",
            lane_topic_.c_str(), local_topic_.c_str(), output_topic_.c_str(),
            output_frame_.c_str(), p.release_sec, p.trigger_min_x_cm,
            p.trigger_max_x_cm, p.trigger_half_width_cm);
    }

private:
    // base_footprint <-> base_link 는 URDF 상 순수 Z 오프셋이라
    // (physicar.urdf.xacro base_joint: origin xyz="0 0 wheel_radius" rpy="0 0 0")
    // 평면 좌표가 동일하다. 그래서 제어점을 건드리지 않고 frame_id 만 바꿔
    // 중계할 수 있다.
    //
    // 단, 그 전제가 깨지는 frame 이 오면 절대 바꿔 달면 안 된다. lane
    // detection 의 path_frame_id 기본값이 "map" 이라 실수로 map 좌표가
    // 들어올 수 있는데, 그걸 base_footprint 라고 이름만 바꿔 내보내면
    // 제어기가 절대좌표를 자차 상대좌표로 읽는다 (차가 즉시 튄다).
    bool planarEquivalent(const std::string & frame) const
    {
        return frame == output_frame_ || frame == "base_link" ||
               frame == "base_footprint";
    }

    void onLane(kau_msgs::msg::KauPath::SharedPtr msg)
    {
        if (!checkFrame(msg->header.frame_id, lane_topic_))
        {
            return;
        }
        lane_ = std::move(msg);
        lane_rx_sec_ = now().seconds();
        publishIfSelected(Source::kLane);
    }

    void onLocal(kau_msgs::msg::KauPath::SharedPtr msg)
    {
        if (!checkFrame(msg->header.frame_id, local_topic_))
        {
            return;
        }
        local_ = std::move(msg);
        local_rx_sec_ = now().seconds();
        publishIfSelected(Source::kLocal);
    }

    bool checkFrame(const std::string & frame, const std::string & topic)
    {
        if (planarEquivalent(frame))
        {
            return true;
        }
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
            "%s 의 frame_id 가 '%s' 다. base_link/base_footprint 가 아니면 "
            "평면 좌표가 같다고 볼 수 없어 중계하지 않는다 "
            "(kau_lane_detection 의 path_frame_id 를 base_link 로 둘 것).",
            topic.c_str(), frame.c_str());
        return false;
    }

    void onObstacles(kau_msgs::msg::ObstacleCircleArray::SharedPtr msg)
    {
        const bool valid = (msg->status == kau_msgs::msg::ObstacleCircleArray::STATUS_OK);
        bool relevant = false;
        if (valid)
        {
            for (const auto & o : msg->obstacles)
            {
                // 메시지는 m, 이 노드는 cm.
                if (obstacleRelevant(
                        static_cast<double>(o.center_x) * 100.0,
                        static_cast<double>(o.center_y) * 100.0,
                        static_cast<double>(o.radius) * 100.0, params_))
                {
                    relevant = true;
                    break;
                }
            }
        }
        else
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                "/perception/obstacles status=%u -- 장애물 정보 없음(비었다는 뜻이 "
                "아니다). 현재 모드를 유지한다.", msg->status);
        }
        obstacles_rx_sec_ = now().seconds();
        arbiter_->updateObstacles(valid, relevant, obstacles_rx_sec_);
    }

    Decision currentDecision()
    {
        const double t = now().seconds();
        const bool lane_fresh = lane_ && lane_rx_sec_ > 0.0 &&
            (t - lane_rx_sec_) <= params_.lane_timeout_sec;
        const bool local_fresh = local_ && local_rx_sec_ > 0.0 &&
            (t - local_rx_sec_) <= params_.local_timeout_sec;
        return arbiter_->decide(lane_fresh, local_fresh, t);
    }

    void publishIfSelected(Source arrived)
    {
        const Decision d = currentDecision();
        if (d.source != arrived)
        {
            return;
        }
        const auto & src = (d.source == Source::kLane) ? lane_ : local_;
        if (!src)
        {
            return;
        }
        kau_msgs::msg::KauPath out = *src;
        // 제어점은 그대로 둔다 (평면 동치이므로 변환 불필요). source 필드도
        // 원본을 유지해 하류가 무엇을 따라가는지 알 수 있게 한다.
        out.header.frame_id = output_frame_;
        drive_pub_->publish(out);
        announce(d);
    }

    void onModeTimer()
    {
        // 장애물 토픽 자체가 안 오면 이 노드는 조용히 LANE 에 머문다.
        // 그건 "장애물이 없다" 가 아니라 **회피 기능이 통째로 죽어 있다** 는
        // 뜻이다 (인지 노드나 플래너가 안 떠 있는 경우). 조용히 넘어가면
        // 장애물 없는 트랙에서는 잘 도는 것처럼 보이다가 실전에서 그대로
        // 들이받으므로, 계속 경고한다.
        const double t = now().seconds();
        const bool obstacles_alive = obstacles_rx_sec_ > 0.0 &&
            (t - obstacles_rx_sec_) <= obstacle_timeout_sec_;
        if (!obstacles_alive)
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                "/perception/obstacles %s -- 회피 모드로 넘어갈 수단이 없다. "
                "차선만 따라간다 (장애물 인지/플래너가 떠 있는지 확인할 것).",
                obstacles_rx_sec_ > 0.0 ? "두절" : "수신 없음");
        }

        const Decision d = currentDecision();
        if (d.source == Source::kNone)
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "발행 없음 (%s) -- 제어기가 timeout 으로 선다.", d.reason);
        }
        announce(d);
    }

    void announce(const Decision & d)
    {
        const std::string s =
            std::string(d.mode == Mode::kAvoid ? "AVOID" : "LANE") + " " + d.reason;
        if (s != last_announce_)
        {
            RCLCPP_INFO(get_logger(), "경로 소스: %s", s.c_str());
            last_announce_ = s;
        }
        std_msgs::msg::String m;
        m.data = s;
        debug_pub_->publish(m);
    }

    ArbiterParams params_;
    std::unique_ptr<Arbiter> arbiter_;
    std::string output_frame_, lane_topic_, local_topic_, output_topic_;
    std::string last_announce_;

    kau_msgs::msg::KauPath::SharedPtr lane_, local_;
    double lane_rx_sec_ = -1.0;
    double local_rx_sec_ = -1.0;
    double obstacles_rx_sec_ = -1.0;
    double obstacle_timeout_sec_ = 1.0;

    rclcpp::Subscription<kau_msgs::msg::KauPath>::SharedPtr lane_sub_, local_sub_;
    rclcpp::Subscription<kau_msgs::msg::ObstacleCircleArray>::SharedPtr obstacle_sub_;
    rclcpp::Publisher<kau_msgs::msg::KauPath>::SharedPtr drive_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr debug_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathArbiterNode>());
    rclcpp::shutdown();
    return 0;
}
