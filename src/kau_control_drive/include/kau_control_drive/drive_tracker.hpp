// ====================================================================
// drive_tracker.hpp
//
// drive_steer_controller / drive_speed_controller 가 공유하는 추종 상태.
//
//     /path/drive (KauPath) 구독  ->  Curve
//     최근접점 추적 (전역 -> 국소 window) + cross track error
//
// kau_control_lane/lane_tracker.hpp 와 알고리즘은 같다. 다른 점 셋:
//
//  1. 곡선 수학을 **복사하지 않는다.** kau_control 의 bezier/curve/kau_path
//     를 그대로 쓴다 (kau_control_lane 은 세 헤더를 사본으로 들고 있고
//     namespace 만 갈라 두었다 -- 세 번째 사본을 만들지 않는다).
//
//  2. pose_source 분기가 없다. /path/drive 는 kau_path_arbiter 가 내는
//     것이고, 그 노드는 자차 프레임(base_link/base_footprint, 평면 동치)이
//     아닌 경로를 아예 중계하지 않는다. 즉 "경로가 map 으로 올 수도 있다" 는
//     경우가 계약상 존재하지 않으므로 TF 를 보지 않는다.
//
//  3. **소스별 timeout.** /path/drive 는 장애물 유무에 따라 차선(14Hz)과
//     회피 경로(5Hz)를 오간다. 하나의 timeout 으로 둘을 덮으면, 14Hz 에
//     맞추면 회피 모드에서 멀쩡한 경로를 죽은 것으로 보고, 5Hz 에 맞추면
//     차선 모드에서 두절을 1 초 가까이 못 알아챈다. KauPath.source 를 보고
//     갈라 쓴다.
//
// 단위 규약: 내부는 cm / rad. TF 는 보지도 발행하지도 않는다.
// ====================================================================

#ifndef KAU_CONTROL_DRIVE__DRIVE_TRACKER_HPP_
#define KAU_CONTROL_DRIVE__DRIVE_TRACKER_HPP_

#include <memory>
#include <string>
#include <utility>

#include <rclcpp/rclcpp.hpp>

#include "kau_msgs/msg/kau_path.hpp"

#include "kau_control/curve.hpp"
#include "kau_control/kau_path.hpp"

namespace kau
{
namespace control_drive
{

using kau::bezier::Point2;
using kau::control::Curve;
using kau::control::TrackState;

// update() 한 tick 의 결과.
//
//     ok == false  -> 노드는 즉시 안전값(조향 0 / 속도 0)을 발행해야 한다.
//
// goal 은 없다. /path/drive 는 차량과 같이 움직이는 rolling horizon 이라
// 종점이 "앞으로 본 데까지" 일 뿐이다. 도착으로 처리하면 매번 멈춘다.
struct TrackResult
{
    bool        ok = false;
    std::string reason;

    double x   = 0.0;   // cm, 자차 (항상 원점)
    double y   = 0.0;
    double yaw = 0.0;   // rad

    double s        = 0.0;   // cm, 경로 위 자차 호길이
    double cte      = 0.0;   // cm, 좌측 +
    double heading_error = 0.0;   // rad, 좌측 +
};

class DriveTracker
{
public:
    explicit DriveTracker(rclcpp::Node * node)
    : node_(node)
    {
        topic_ = node_->declare_parameter<std::string>("drive.topic", "/path/drive");
        frame_ = node_->declare_parameter<std::string>("drive.frame", "base_footprint");
        // 소스별 timeout. 근거는 헤더 주석 3번.
        timeout_lane_ = node_->declare_parameter<double>("drive.timeout_lane", 0.5);
        timeout_local_ = node_->declare_parameter<double>("drive.timeout_local", 1.0);

        rclcpp::QoS qos(rclcpp::KeepLast(1));
        qos.reliable();
        sub_ = node_->create_subscription<kau_msgs::msg::KauPath>(
            topic_, qos,
            [this](kau_msgs::msg::KauPath::SharedPtr m) { onPath(std::move(m)); });

        RCLCPP_INFO(node_->get_logger(),
            "경로 입력: %s [%s]  timeout lane=%.2fs local=%.2fs",
            topic_.c_str(), frame_.c_str(), timeout_lane_, timeout_local_);
    }

    TrackResult update()
    {
        TrackResult r;
        if (!curve_)
        {
            r.reason = "경로 수신 없음";
            return r;
        }
        const double age = (node_->now() - stamp_).seconds();
        const double limit = timeoutForSource();
        if (age > limit)
        {
            r.reason = "경로 두절 (" + fmt(age) + "s > " + fmt(limit) + "s, source=" +
                       std::to_string(static_cast<int>(source_)) + ")";
            return r;
        }

        // 경로가 자차 프레임이므로 자차는 항상 원점이다.
        r.x = 0.0;
        r.y = 0.0;
        r.yaw = 0.0;

        // 첫 질의는 전역, 이후 국소 window (곡선이 바뀌면 onPath 가 리셋한다).
        track_ = curve_->nearest(Point2{r.x, r.y}, track_);
        if (!track_.valid)
        {
            r.reason = "최근접점 탐색 실패";
            return r;
        }

        r.s = track_.s;
        const Point2 p = curve_->point(track_.s);
        const double th = curve_->heading(track_.s);
        // 좌측이 + 인 부호 규약 (kau_control_lane 과 동일).
        r.cte = -std::sin(th) * (r.x - p.x) + std::cos(th) * (r.y - p.y);
        r.heading_error = kau::control::wrapPi(r.yaw - th);
        r.ok = true;
        return r;
    }

    const Curve * curve() const { return curve_.get(); }
    uint8_t source() const { return source_; }
    double valid_length() const { return valid_length_; }
    const std::string & frame() const { return frame_; }

    // 직전 update 이후 경로 소스가 바뀌었으면 true 를 한 번만 돌려준다.
    // 조향 제어기가 전환 구간에 슬루 제한을 조이는 데 쓴다.
    bool consumeSourceChanged()
    {
        const bool c = source_changed_;
        source_changed_ = false;
        return c;
    }

private:
    double timeoutForSource() const
    {
        return (source_ == kau_msgs::msg::KauPath::SRC_LOCAL) ? timeout_local_
                                                              : timeout_lane_;
    }

    void onPath(kau_msgs::msg::KauPath::SharedPtr msg)
    {
        // frame 이 다르면 좌표계를 잘못 읽는 것이라 조용히 넘기면 안 된다.
        // (arbiter 가 한 번 걸러 주지만, 이 노드를 다른 발행자에 직접 물릴
        //  수도 있으므로 여기서도 본다.)
        if (msg->header.frame_id != frame_)
        {
            RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                "%s 의 frame_id 가 '%s' 인데 drive.frame 은 '%s' 다. "
                "topic 과 frame 은 짝이다 -- 둘을 같이 맞출 것.",
                topic_.c_str(), msg->header.frame_id.c_str(), frame_.c_str());
            return;
        }
        std::string reason;
        if (!kau::control::validateKauPath(*msg, reason))
        {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                "KauPath 검증 실패: %s", reason.c_str());
            return;
        }
        Curve cv = kau::control::curveFromKauPath(*msg);
        if (!cv.windowSafe())
        {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                "window 추적이 불안전한 곡선 (nseg=%d)", cv.nseg());
            return;
        }

        if (curve_ && msg->source != source_)
        {
            source_changed_ = true;
            RCLCPP_INFO(node_->get_logger(), "경로 소스 전환: %s -> %s",
                srcName(source_), srcName(msg->source));
        }
        curve_ = std::make_shared<Curve>(std::move(cv));
        source_ = msg->source;
        valid_length_ = msg->valid_length;
        stamp_ = node_->now();
        // 매 프레임 새로 지은 곡선이다. 직전 s 가 그대로 유효하지 않으므로
        // window 추적 상태를 버린다 (다음 질의는 전역 탐색).
        track_ = TrackState{};
    }

    static const char * srcName(uint8_t s)
    {
        switch (s)
        {
            case kau_msgs::msg::KauPath::SRC_GLOBAL: return "GLOBAL";
            case kau_msgs::msg::KauPath::SRC_LOCAL:  return "LOCAL(회피)";
            case kau_msgs::msg::KauPath::SRC_LANE:   return "LANE(차선)";
            default: return "?";
        }
    }

    static std::string fmt(double v)
    {
        char b[32];
        std::snprintf(b, sizeof(b), "%.2f", v);
        return std::string(b);
    }

    rclcpp::Node * node_;
    std::string topic_, frame_;
    double timeout_lane_ = 0.5;
    double timeout_local_ = 1.0;

    rclcpp::Subscription<kau_msgs::msg::KauPath>::SharedPtr sub_;
    std::shared_ptr<Curve> curve_;
    TrackState track_;
    rclcpp::Time stamp_;
    uint8_t source_ = 0;
    double valid_length_ = 0.0;
    bool source_changed_ = false;
};

}  // namespace control_drive
}  // namespace kau

#endif  // KAU_CONTROL_DRIVE__DRIVE_TRACKER_HPP_
