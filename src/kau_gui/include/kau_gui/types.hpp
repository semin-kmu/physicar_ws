// ====================================================================
// types.hpp
//
// ROS 스레드와 Qt 스레드가 주고받는 자료구조.
//
// 두 계열로 나뉜다. 이 구분이 이 GUI 설계의 핵심이다.
//
//   latch  : 맵 계열 (스캔 · 경로 · 장애물 · 자세). 어차피 최신 한 장만
//            그리므로 최신값만 덮어쓴다.
//   Series : 플롯 계열. 콜백에서 **전량** 링버퍼에 쌓는다. 렌더가 10 Hz
//            라고 50 Hz 신호를 10 Hz 로 샘플링하면 조향 떨림 · cte 스파이크
//            처럼 정작 봐야 할 순간 변화가 화면에서 사라진다.
//
// 단위: 이 GUI 내부는 표시 단위를 그대로 쓴다 (길이 m, 각 deg, 오차 cm).
//       ROS 경계에서 ros_bridge 가 변환한다.
// ====================================================================

#ifndef KAU_GUI__TYPES_HPP_
#define KAU_GUI__TYPES_HPP_

#include <cstddef>
#include <deque>
#include <string>
#include <vector>

#include <QImage>
#include <QPointF>


namespace kau_gui
{

// --------------------------------------------------------------------
// 시계열 링버퍼
//
// 시간축은 전부 GUI 노드 clock 기준 초. 메시지 stamp 를 쓰지 않는 이유는
// 발행 노드마다 sim/system clock 이 섞일 수 있어 축이 어긋나기 때문이다.
// --------------------------------------------------------------------

struct Sample
{
    double t = 0.0;

    double v = 0.0;
};


class Series
{
public:
    void setWindow(double seconds)
    {
        window_ = seconds;
    }

    void push(double t, double v)
    {
        data_.push_back(Sample{t, v});

        // 창 밖으로 나간 것만 버린다. 개수 제한을 따로 두지 않는 이유는
        // 창 길이 x 최고 발행주기(50 Hz)로 상한이 이미 정해지기 때문이다.
        while (!data_.empty() && (t - data_.front().t) > window_)
        {
            data_.pop_front();
        }

        last_  = v;

        stamp_ = t;

        got_   = true;
    }

    const std::deque<Sample> & data() const
    {
        return data_;
    }

    bool empty() const
    {
        return data_.empty();
    }

    // 마지막 값. 아직 하나도 못 받았으면 got()==false.
    double last() const
    {
        return last_;
    }

    double stamp() const
    {
        return stamp_;
    }

    bool got() const
    {
        return got_;
    }

private:
    std::deque<Sample> data_;

    double window_ = 30.0;

    double last_   = 0.0;

    double stamp_  = 0.0;

    bool   got_    = false;
};


// --------------------------------------------------------------------
// 토픽 수신 주기 실측
//
// 노드가 "살아 있는가" 가 아니라 "제 주기로 돌고 있는가" 를 본다.
// heartbeat 가 아직 없으므로 이것이 유일한 근거다.
// --------------------------------------------------------------------

class RateMeter
{
public:
    void mark(double t)
    {
        stamps_.push_back(t);

        while (stamps_.size() > MAX_MARKS)
        {
            stamps_.pop_front();
        }

        last_ = t;
    }

    // 관측 표본이 2 개 미만이면 -1. "아직 모름" 과 "0 Hz" 는 다르다.
    double hz() const
    {
        if (stamps_.size() < 2)
        {
            return -1.0;
        }

        const double span = stamps_.back() - stamps_.front();

        if (span <= 1e-6)
        {
            return -1.0;
        }

        return static_cast<double>(stamps_.size() - 1) / span;
    }

    double last() const
    {
        return last_;
    }

    bool got() const
    {
        return last_ > 0.0;
    }

private:
    static constexpr std::size_t MAX_MARKS = 24;

    std::deque<double> stamps_;

    double last_ = 0.0;
};


// --------------------------------------------------------------------
// 노드 감시 결과
// --------------------------------------------------------------------

enum class Health
{
    ABSENT,     // ROS graph 에 없다. 미기동
    STALE,      // 노드는 있으나 대표 토픽이 끊겼다
    OK
};


struct NodeStatus
{
    std::string name;

    Health health = Health::ABSENT;

    // 실측 Hz. 음수면 표시하지 않는다 (표본 부족 또는 판정 대상 아님).
    double hz = -1.0;

    // 기대 주기가 0 이면 주기 판정을 하지 않는 노드다.
    bool rate_checked = false;
};


// --------------------------------------------------------------------
// 맵 배경
//
// img 는 화면 방향 그대로다 (row 0 = 월드 y 최대). OccupancyGrid 는
// row 0 이 y 최소이므로 ros_bridge 가 뒤집어서 넣는다.
// --------------------------------------------------------------------

struct MapImage
{
    QImage img;

    double resolution = 0.05;   // m/px

    double origin_x   = 0.0;    // m, 이미지 좌하단의 월드 좌표

    double origin_y   = 0.0;

    bool   valid      = false;

    double widthM() const
    {
        return img.width() * resolution;
    }

    double heightM() const
    {
        return img.height() * resolution;
    }
};


// --------------------------------------------------------------------
// 맵 패널 표시물 (전부 map 프레임 · m)
// --------------------------------------------------------------------

struct Obstacle
{
    double x = 0.0;

    double y = 0.0;

    double r = 0.0;
};


struct Pose2D
{
    double x   = 0.0;

    double y   = 0.0;

    double yaw = 0.0;   // rad

    bool   valid = false;
};


// 최신 한 장만 유지하는 표시물. stamp 는 GUI clock 기준 수신 시각.
template<typename T>
struct Latest
{
    T      value{};

    double stamp = 0.0;

    bool   got   = false;

    void set(const T & v, double t)
    {
        value = v;

        stamp = t;

        got   = true;
    }

    // 수신한 적이 있고, 마지막 수신이 timeout 안이면 살아 있다고 본다.
    bool fresh(double now, double timeout) const
    {
        return got && (now - stamp) <= timeout;
    }
};


using Polyline = std::vector<QPointF>;


// --------------------------------------------------------------------
// Qt 스레드가 한 프레임 그릴 때 받아가는 전량 복사본
//
// 스냅샷을 뜬 뒤에는 lock 을 잡지 않는다. 복사량은 스캔 1000 점 기준
// 수십 KB 라 10 Hz 에서 문제되지 않는다.
// --------------------------------------------------------------------

struct Snapshot
{
    double now = 0.0;           // GUI clock 기준 초

    // --- 맵 패널 ---
    MapImage map;

    Latest<Polyline> scan;
    Latest<Polyline> path_global;
    Latest<Polyline> path_local;
    Latest<Polyline> path_lane;

    Latest<std::vector<Obstacle>> obstacles;

    Latest<QPointF> lookahead;

    Pose2D pose;

    // --- 플롯 패널 ---
    Series speed_target;        // m/s
    Series speed_real;          // m/s
    Series steer_raw;           // deg, clamp 전
    Series steer_cmd;           // deg, clamp 후
    Series lookahead_cm;        // cm
    Series heading_err_deg;     // deg
    Series cross_track_cm;      // cm

    // steer_controller 가 지금 경로를 추종하고 있는가.
    // false 면 조향 0 이 나가는 중이라 위 4 개 플롯이 의미 없다.
    bool tracking_ok = false;

    bool steer_debug_alive = false;

    // --- 상단 패널 ---
    std::vector<NodeStatus> nodes;
};

}  // namespace kau_gui

#endif  // KAU_GUI__TYPES_HPP_
