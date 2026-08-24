#include "kau_gui/ros_bridge.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <utility>

#include <QDir>
#include <QFileInfo>

#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>


namespace kau_gui
{

namespace
{

constexpr double RAD2DEG = 180.0 / M_PI;

// 관측 전용 구독. RELIABLE 발행자와도 호환되고, 재전송 트래픽을 발행측에
// 지우지 않는다 (docs/09 section 10-1).
rclcpp::QoS observeQos()
{
    return rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
}

// latched(TRANSIENT_LOCAL) 발행자를 받으려면 구독자도 맞춰야 한다.
// /map, /viz/path/global 이 여기 해당한다.
rclcpp::QoS latchedQos()
{
    return rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
}

// map yaml 한 줄에서 "key: value" 를 뽑는다. 필요한 키가 3 개뿐이라
// yaml 라이브러리를 끌어오지 않는다.
bool yamlValue(
    const std::string & line, const std::string & key, std::string * out)
{
    const std::size_t pos = line.find(key + ":");

    if (pos != 0)
    {
        return false;      // 들여쓴 하위 키를 잘못 집지 않도록 줄 머리만
    }

    std::string v = line.substr(key.size() + 1);

    const std::size_t b = v.find_first_not_of(" \t\"'");

    if (b == std::string::npos)
    {
        return false;
    }

    const std::size_t e = v.find_last_not_of(" \t\"'\r\n");

    *out = v.substr(b, e - b + 1);

    return true;
}

}  // namespace


// ====================================================================
// 생성
// ====================================================================

RosBridge::RosBridge()
: rclcpp::Node("kau_gui")
{
    declareAll();

    start_t_ = stamp();

    snap_.speed_target.setWindow(history_s_);
    snap_.speed_real.setWindow(history_s_);
    snap_.steer_raw.setWindow(history_s_);
    snap_.steer_cmd.setWindow(history_s_);
    snap_.lookahead_cm.setWindow(history_s_);
    snap_.heading_err_deg.setWindow(history_s_);
    snap_.cross_track_cm.setWindow(history_s_);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());

    tf_listener_ =
        std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    makeDisplaySubs();

    makeWatchSubs();


    // 차량 자세는 20 Hz 로 갱신한다. 렌더(10 Hz)보다 빠르게 두어 매 프레임
    // 항상 최신 TF 가 잡혀 있게 한다.
    pose_timer_ = create_wall_timer(
        std::chrono::milliseconds(50),
        [this]()
        {
            onPoseTimer();
        });

    // graph 조회는 비싸다. 1 Hz 면 노드 죽음을 1 초 안에 잡는다.
    graph_timer_ = create_wall_timer(
        std::chrono::seconds(1),
        [this]()
        {
            onGraphTimer();
        });


    RCLCPP_INFO(
        get_logger(),
        "[kau_gui] 관측 전용 기동. render=%.0f Hz history=%.0f s "
        "감시노드=%zu 감시구독=%zu",
        render_hz_, history_s_, watch_names_.size(), watch_subs_.size());
}


void RosBridge::declareAll()
{
    render_hz_  = declare_parameter<double>("render_hz", 10.0);
    history_s_  = declare_parameter<double>("history_s", 30.0);

    map_frame_  = declare_parameter<std::string>("frames.map", "map");
    base_frame_ = declare_parameter<std::string>("frames.base", "base_link");
    tf_timeout_ = declare_parameter<double>("frames.tf_timeout_s", 0.5);

    declare_parameter<std::string>("map.source", "auto");
    declare_parameter<std::string>("map.topic", "/map");
    map_yaml_   = declare_parameter<std::string>("map.yaml_path", "");
    map_wait_s_ = declare_parameter<double>("map.wait_s", 5.0);

    declare_parameter<std::string>("topics.scan", "/scan_filtered");
    declare_parameter<std::string>(
        "topics.obstacles", "/perception/obstacle_markers");
    declare_parameter<std::string>("topics.path_global", "/viz/path/global");
    declare_parameter<std::string>("topics.path_local", "/viz/path/local");
    declare_parameter<std::string>("topics.path_lane", "/viz/path/lane");
    declare_parameter<std::string>("topics.lookahead", "/viz/path/lookahead");
    declare_parameter<std::string>("topics.speed_cmd", "/speed");
    declare_parameter<std::string>("topics.odom", "/odom");
    declare_parameter<std::string>("topics.steering", "/steering");
    declare_parameter<std::string>("topics.steer_debug", "/debug/steer");

    veh_len_ = declare_parameter<double>("vehicle.length_m", 0.30);
    veh_wid_ = declare_parameter<double>("vehicle.width_m", 0.20);

    stale_ratio_ = declare_parameter<double>("watch.stale_ratio", 3.0);
    min_stale_s_ = declare_parameter<double>("watch.min_stale_s", 0.3);

    watch_names_ = declare_parameter<std::vector<std::string>>(
        "watch.names", std::vector<std::string>{});
    watch_topics_ = declare_parameter<std::vector<std::string>>(
        "watch.topics", std::vector<std::string>{});
    watch_types_ = declare_parameter<std::vector<std::string>>(
        "watch.types", std::vector<std::string>{});
    watch_rates_ = declare_parameter<std::vector<double>>(
        "watch.rates", std::vector<double>{});

    // 네 배열의 index 가 한 노드를 이룬다. 길이가 어긋나면 엉뚱한 토픽으로
    // 등급을 매기게 되므로 조용히 넘어가지 않는다.
    if (watch_names_.size() != watch_topics_.size() ||
        watch_names_.size() != watch_types_.size() ||
        watch_names_.size() != watch_rates_.size())
    {
        throw std::runtime_error(
            "watch.names / topics / types / rates 의 길이가 서로 다르다");
    }
}


void RosBridge::makeDisplaySubs()
{
    auto topic = [this](const char * key)
        {
            return get_parameter(std::string("topics.") + key).as_string();
        };

    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        topic("scan"), observeQos(),
        [this](sensor_msgs::msg::LaserScan::SharedPtr m)
        {
            onScan(m);
        });

    marker_sub_ = create_subscription<visualization_msgs::msg::MarkerArray>(
        topic("obstacles"), observeQos(),
        [this](visualization_msgs::msg::MarkerArray::SharedPtr m)
        {
            onMarkers(m);
        });

    // 발행측이 latched 다. 구독도 맞춰야 GUI 를 나중에 띄워도 전역 경로가
    // 한 번 들어온다.
    global_sub_ = create_subscription<nav_msgs::msg::Path>(
        topic("path_global"), latchedQos(),
        [this](nav_msgs::msg::Path::SharedPtr m)
        {
            onPath(m, &snap_.path_global);
        });

    local_sub_ = create_subscription<nav_msgs::msg::Path>(
        topic("path_local"), observeQos(),
        [this](nav_msgs::msg::Path::SharedPtr m)
        {
            onPath(m, &snap_.path_local);
        });

    lane_sub_ = create_subscription<nav_msgs::msg::Path>(
        topic("path_lane"), observeQos(),
        [this](nav_msgs::msg::Path::SharedPtr m)
        {
            onPath(m, &snap_.path_lane);
        });

    lookahead_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
        topic("lookahead"), observeQos(),
        [this](geometry_msgs::msg::PointStamped::SharedPtr m)
        {
            onLookahead(m);
        });

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        topic("odom"), observeQos(),
        [this](nav_msgs::msg::Odometry::SharedPtr m)
        {
            onOdom(m);
        });

    speed_sub_ = create_subscription<std_msgs::msg::Float64>(
        topic("speed_cmd"), observeQos(),
        [this](std_msgs::msg::Float64::SharedPtr m)
        {
            onSpeed(m);
        });

    steering_sub_ = create_subscription<std_msgs::msg::Float64>(
        topic("steering"), observeQos(),
        [this](std_msgs::msg::Float64::SharedPtr m)
        {
            onSteering(m);
        });

    debug_sub_ = create_subscription<kau_msgs::msg::SteerDebug>(
        topic("steer_debug"), observeQos(),
        [this](kau_msgs::msg::SteerDebug::SharedPtr m)
        {
            onSteerDebug(m);
        });


    const std::string src = get_parameter("map.source").as_string();

    if (src == "topic" || src == "auto")
    {
        grid_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
            get_parameter("map.topic").as_string(), latchedQos(),
            [this](nav_msgs::msg::OccupancyGrid::SharedPtr m)
            {
                onGrid(m);
            });
    }

    if (src == "file" && !map_yaml_.empty())
    {
        MapImage mi;

        if (loadMapFile(map_yaml_, &mi))
        {
            std::lock_guard<std::mutex> lk(mu_);

            snap_.map = std::move(mi);
        }

        map_file_tried_ = true;
    }
}


// 감시 토픽마다 generic subscription 을 하나씩. 역직렬화하지 않으므로
// 큰 메시지를 감시해도 부하가 수신 시각 기록뿐이다.
//
// 주기 판정을 하지 않는 노드(rate <= 0)는 구독 자체를 만들지 않는다.
// graph 존재 여부만으로 판정하기 때문이다.
void RosBridge::makeWatchSubs()
{
    for (std::size_t i = 0; i < watch_names_.size(); ++i)
    {
        const std::string & topic = watch_topics_[i];

        const std::string & type = watch_types_[i];

        if (topic.empty() || type.empty() || watch_rates_[i] <= 0.0)
        {
            continue;
        }

        if (rates_.count(topic) > 0)
        {
            continue;      // 두 노드가 같은 토픽을 보면 구독은 하나로
        }

        rates_.emplace(topic, RateMeter{});

        try
        {
            // BEST_EFFORT / VOLATILE 구독자는 어떤 발행자 QoS 와도 맞는다.
            //
            // 콜백 인자를 auto 로 받는 이유: rclcpp 버전에 따라
            // SerializedMessage 가 const 로 오기도 한다. 어차피 쓰지 않으므로
            // 타입을 고정하지 않는다.
            watch_subs_.push_back(
                create_generic_subscription(
                    topic, type, observeQos(),
                    [this, topic](auto)
                    {
                        std::lock_guard<std::mutex> lk(mu_);

                        rates_[topic].mark(stamp());
                    }));
        }
        catch (const std::exception & e)
        {
            // 타입 이름 오타 등. 감시 하나가 실패했다고 GUI 전체를 죽이지
            // 않는다. 해당 노드는 STALE 로 남아 눈에 띈다.
            RCLCPP_ERROR(
                get_logger(), "[kau_gui] 감시 구독 실패 %s (%s): %s",
                topic.c_str(), type.c_str(), e.what());
        }
    }
}


double RosBridge::stamp() const
{
    return now().seconds();
}


// ====================================================================
// 표시용 콜백
// ====================================================================

void RosBridge::onScan(const sensor_msgs::msg::LaserScan::SharedPtr m)
{
    // 스캔은 센서 프레임이다. map 으로 옮겨야 배경 지도 위에 얹힌다.
    geometry_msgs::msg::TransformStamped tf;

    try
    {
        tf = tf_buffer_->lookupTransform(
            map_frame_, m->header.frame_id, tf2::TimePointZero);
    }
    catch (const tf2::TransformException &)
    {
        // 측위가 없으면 절대좌표를 만들 수 없다. 직전 점군을 그대로 두면
        // 낡은 것이 계속 그려지므로 갱신하지 않고 stale 로 늙게 둔다.
        return;
    }

    const double tx  = tf.transform.translation.x;
    const double ty  = tf.transform.translation.y;
    const double yaw = tf2::getYaw(tf.transform.rotation);

    const double c = std::cos(yaw);
    const double s = std::sin(yaw);

    Polyline pts;

    pts.reserve(m->ranges.size());

    for (std::size_t i = 0; i < m->ranges.size(); ++i)
    {
        const float r = m->ranges[i];

        if (!std::isfinite(r) || r < m->range_min || r > m->range_max)
        {
            continue;
        }

        const double a =
            m->angle_min + m->angle_increment * static_cast<double>(i);

        const double lx = r * std::cos(a);
        const double ly = r * std::sin(a);

        pts.emplace_back(tx + c * lx - s * ly, ty + s * lx + c * ly);
    }

    std::lock_guard<std::mutex> lk(mu_);

    snap_.scan.set(pts, stamp());
}


void RosBridge::onMarkers(
    const visualization_msgs::msg::MarkerArray::SharedPtr m)
{
    std::vector<Obstacle> out;

    for (const auto & mk : m->markers)
    {
        // 발행측이 사라진 장애물을 DELETE 로 지운다. 그건 그릴 것이 없다.
        if (mk.action != visualization_msgs::msg::Marker::ADD)
        {
            continue;
        }

        // obstacle_markers 는 원판을 CYLINDER 로 낸다. scale.x = 지름.
        out.push_back(
            Obstacle{
                mk.pose.position.x, mk.pose.position.y, mk.scale.x * 0.5});
    }

    std::lock_guard<std::mutex> lk(mu_);

    snap_.obstacles.set(out, stamp());
}


void RosBridge::onGrid(const nav_msgs::msg::OccupancyGrid::SharedPtr m)
{
    const int w = static_cast<int>(m->info.width);

    const int h = static_cast<int>(m->info.height);

    if (w <= 0 || h <= 0)
    {
        return;
    }

    QImage img(w, h, QImage::Format_Grayscale8);

    // OccupancyGrid 는 row 0 이 월드 y 최소다. MapImage 규약은 row 0 이
    // 월드 y 최대(화면 위)이므로 뒤집어 담는다.
    for (int row = 0; row < h; ++row)
    {
        uchar * line = img.scanLine(row);

        const std::size_t src = static_cast<std::size_t>(h - 1 - row) *
                                static_cast<std::size_t>(w);

        for (int col = 0; col < w; ++col)
        {
            const int8_t v = m->data[src + static_cast<std::size_t>(col)];

            // -1 미탐색 / 0 자유 / 100 점유
            line[col] = (v < 0)
                ? static_cast<uchar>(128)
                : static_cast<uchar>(255 - (v * 255) / 100);
        }
    }

    MapImage mi;

    mi.img        = std::move(img);
    mi.resolution = m->info.resolution;
    mi.origin_x   = m->info.origin.position.x;
    mi.origin_y   = m->info.origin.position.y;
    mi.valid      = true;

    std::lock_guard<std::mutex> lk(mu_);

    snap_.map = std::move(mi);

    map_from_topic_ = true;
}


void RosBridge::onPath(
    const nav_msgs::msg::Path::SharedPtr m, Latest<Polyline> * dst)
{
    Polyline pts;

    pts.reserve(m->poses.size());

    for (const auto & ps : m->poses)
    {
        pts.emplace_back(ps.pose.position.x, ps.pose.position.y);
    }

    std::lock_guard<std::mutex> lk(mu_);

    dst->set(pts, stamp());
}


void RosBridge::onLookahead(
    const geometry_msgs::msg::PointStamped::SharedPtr m)
{
    std::lock_guard<std::mutex> lk(mu_);

    snap_.lookahead.set(QPointF(m->point.x, m->point.y), stamp());
}


void RosBridge::onOdom(const nav_msgs::msg::Odometry::SharedPtr m)
{
    std::lock_guard<std::mutex> lk(mu_);

    snap_.speed_real.push(stamp(), m->twist.twist.linear.x);
}


void RosBridge::onSpeed(const std_msgs::msg::Float64::SharedPtr m)
{
    std::lock_guard<std::mutex> lk(mu_);

    snap_.speed_target.push(stamp(), m->data);
}


void RosBridge::onSteering(const std_msgs::msg::Float64::SharedPtr m)
{
    std::lock_guard<std::mutex> lk(mu_);

    // /steering 은 rad. 플롯은 deg 로 통일한다. max_steer_deg(20) 와 눈으로
    // 바로 대조하려면 deg 여야 한다.
    snap_.steer_cmd.push(stamp(), m->data * RAD2DEG);
}


void RosBridge::onSteerDebug(const kau_msgs::msg::SteerDebug::SharedPtr m)
{
    const double t = stamp();

    std::lock_guard<std::mutex> lk(mu_);

    snap_.steer_debug_alive = true;

    snap_.tracking_ok = m->tracking_ok;

    // 추종 실패 tick 의 0 을 그대로 쌓으면 플롯이 0 으로 끌려 내려가
    // 오차가 사라진 것처럼 보인다. 값을 넣지 않고 선을 끊는다.
    if (!m->tracking_ok)
    {
        return;
    }

    snap_.steer_raw.push(t, m->raw_steer_deg);

    snap_.lookahead_cm.push(t, m->lookahead_cm);

    snap_.heading_err_deg.push(t, m->heading_error_rad * RAD2DEG);

    snap_.cross_track_cm.push(t, m->cross_track_cm);
}


// ====================================================================
// 주기 작업
// ====================================================================

void RosBridge::onPoseTimer()
{
    // auto 모드: 토픽으로 맵이 안 오면 파일로 폴백한다. map_server 없이
    // GUI 만 띄워도 배경이 나오게 하기 위한 것이다.
    if (!map_from_topic_ && !map_file_tried_ && !map_yaml_.empty() &&
        get_parameter("map.source").as_string() == "auto" &&
        (stamp() - start_t_) > map_wait_s_)
    {
        map_file_tried_ = true;

        MapImage mi;

        if (loadMapFile(map_yaml_, &mi))
        {
            RCLCPP_INFO(
                get_logger(), "[kau_gui] 맵 토픽 미수신 -> 파일 사용: %s",
                map_yaml_.c_str());

            std::lock_guard<std::mutex> lk(mu_);

            snap_.map = std::move(mi);
        }
    }


    geometry_msgs::msg::TransformStamped tf;

    try
    {
        tf = tf_buffer_->lookupTransform(
            map_frame_, base_frame_, tf2::TimePointZero);
    }
    catch (const tf2::TransformException &)
    {
        std::lock_guard<std::mutex> lk(mu_);

        snap_.pose.valid = false;

        return;
    }

    const double age = (now() - rclcpp::Time(tf.header.stamp)).seconds();

    Pose2D p;

    p.x     = tf.transform.translation.x;
    p.y     = tf.transform.translation.y;
    p.yaw   = tf2::getYaw(tf.transform.rotation);

    // TF 는 조회되는데 낡았다 = 측위가 멈춘 것이다. 좌표는 그대로 두고
    // valid 만 내려 화면에서 STALE 로 보이게 한다.
    p.valid = (age >= 0.0 && age <= tf_timeout_);

    std::lock_guard<std::mutex> lk(mu_);

    snap_.pose = p;
}


void RosBridge::onGraphTimer()
{
    // 노드 이름공간은 쓰지 않는 것이 팀 규약이라(docs/07 section 2)
    // 마지막 조각만 비교한다.
    std::vector<std::string> alive;

    for (const std::string & full : get_node_names())
    {
        const std::size_t pos = full.find_last_of('/');

        alive.push_back(
            pos == std::string::npos ? full : full.substr(pos + 1));
    }

    const double t = stamp();

    std::vector<NodeStatus> out;

    out.reserve(watch_names_.size());


    std::lock_guard<std::mutex> lk(mu_);

    for (std::size_t i = 0; i < watch_names_.size(); ++i)
    {
        NodeStatus st;

        st.name = watch_names_[i];

        if (std::find(alive.begin(), alive.end(), st.name) == alive.end())
        {
            st.health = Health::ABSENT;

            out.push_back(st);

            continue;
        }

        const std::string & topic = watch_topics_[i];

        const double want = watch_rates_[i];

        // 대표 토픽이 없거나(진단 로그 전용) latched 라 주기가 없는 노드는
        // 존재만으로 정상 처리한다. Hz 칸은 비운다.
        if (topic.empty() || want <= 0.0)
        {
            st.health       = Health::OK;

            st.rate_checked = false;

            out.push_back(st);

            continue;
        }

        st.rate_checked = true;

        auto it = rates_.find(topic);

        if (it == rates_.end() || !it->second.got())
        {
            st.health = Health::STALE;

            out.push_back(st);

            continue;
        }

        st.hz = it->second.hz();

        // 기대 주기의 stale_ratio 배 동안 소식이 없으면 끊긴 것으로 본다.
        // 다만 하한을 둔다. 50 Hz 토픽은 3 배가 60 ms 라, 무선 지터만으로도
        // 빨간불이 깜빡여 정작 진짜 고장이 묻힌다.
        const double limit = std::max(stale_ratio_ / want, min_stale_s_);

        st.health =
            ((t - it->second.last()) <= limit) ? Health::OK : Health::STALE;

        out.push_back(st);
    }

    snap_.nodes = std::move(out);
}


// ====================================================================
// 맵 파일
// ====================================================================

bool RosBridge::loadMapFile(const std::string & yaml_path, MapImage * out)
{
    std::ifstream f(yaml_path);

    if (!f)
    {
        RCLCPP_WARN(
            get_logger(), "[kau_gui] 맵 yaml 을 열 수 없다: %s",
            yaml_path.c_str());

        return false;
    }

    std::string image;

    double res = 0.05;

    double ox = 0.0;

    double oy = 0.0;

    std::string line;

    while (std::getline(f, line))
    {
        std::string v;

        if (yamlValue(line, "image", &v))
        {
            image = v;
        }
        else if (yamlValue(line, "resolution", &v))
        {
            res = std::atof(v.c_str());
        }
        else if (yamlValue(line, "origin", &v))
        {
            // "[-3.79, -1.76, 0.0]" 형태
            for (char & c : v)
            {
                if (c == '[' || c == ']' || c == ',')
                {
                    c = ' ';
                }
            }

            std::istringstream is(v);

            is >> ox >> oy;
        }
    }

    if (res <= 0.0)
    {
        RCLCPP_WARN(
            get_logger(), "[kau_gui] 맵 resolution 이 잘못됐다: %f", res);

        return false;
    }


    // map yaml 의 image 는 절대경로인 경우가 많고 그 경로는 지도를 만든 PC
    // 기준이다. 없으면 yaml 옆에서 같은 파일명을 찾는다.
    QString img_path = QString::fromStdString(image);

    if (!QFileInfo::exists(img_path))
    {
        const QFileInfo yi(QString::fromStdString(yaml_path));

        img_path = yi.absoluteDir().filePath(QFileInfo(img_path).fileName());
    }

    QImage img;

    if (!img.load(img_path))
    {
        RCLCPP_WARN(
            get_logger(), "[kau_gui] 맵 이미지를 읽을 수 없다: %s",
            img_path.toStdString().c_str());

        return false;
    }

    out->img        = img.convertToFormat(QImage::Format_Grayscale8);
    out->resolution = res;
    out->origin_x   = ox;
    out->origin_y   = oy;
    out->valid      = true;

    return true;
}


// ====================================================================
// 스냅샷
// ====================================================================

Snapshot RosBridge::snapshot()
{
    std::lock_guard<std::mutex> lk(mu_);

    snap_.now = stamp();

    return snap_;
}

}  // namespace kau_gui
