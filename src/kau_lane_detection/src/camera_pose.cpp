// ====================================================================
// camera_pose.cpp
//
// 카메라 자세 구동. pan 각 이력/램프 발행과 tilt 고정 자세.
//
// 설계 근거와 실측표는 CLAUDE.md 참고.
// ====================================================================

#include "kau_lane_detection/kau_lane_detection_node.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <functional>

namespace geom = kau_lane::geom;


// ================================================================
// JointState Callback
//
// physicar_driver_node 가 50Hz 로 camera_pan_joint 실측 각도를
// 낸다 (rad, publish_joint_states 참고). pan 명령을 보냈다고
// 그 즉시 그 각도에 가 있다고 가정하지 않고, 이 콜백이 채우는
// actual_pan_deg_ 로 updatePanSearch() 가 정착을 판정한다.
// ================================================================

void KauLaneDetectionNode::jointStateCallback(
    const sensor_msgs::msg::JointState::SharedPtr msg)
{
    for (
        std::size_t i = 0;
        i < msg->name.size() && i < msg->position.size();
        ++i
    )
    {
        if (msg->name[i] == "camera_pan_joint")
        {
            actual_pan_deg_ =
                msg->position[i] * 180.0 / M_PI;

            joint_state_received_ = true;


            // 영상 시각의 각도를 복원하려면 스탬프가 있어야 한다.
            // (선언부 pan_history_ 주석 참고)
            const rclcpp::Time stamp(msg->header.stamp);

            // 스탬프가 뒤로 간 경우(시뮬 재시작 등)는 이력을 버린다.
            // 정렬이 깨지면 보간이 엉뚱한 구간을 집는다.
            if (
                !pan_history_.empty() &&
                stamp < pan_history_.back().stamp
            )
            {
                pan_history_.clear();
            }

            pan_history_.push_back({stamp, actual_pan_deg_});

            while (
                pan_history_.size() > 1 &&
                (stamp - pan_history_.front().stamp).seconds() >
                    kPanHistorySec
            )
            {
                pan_history_.pop_front();
            }

            break;
        }
    }
}


// ================================================================
// 영상 스탬프에서의 pan 각도 [deg]
//
// 이력을 스탬프로 이분탐색해 앞뒤 두 샘플 사이를 선형보간한다.
// 범위 밖이면 끝값으로 자른다 (외삽하지 않는다 — 서보 거동을
// 모르는 상태에서 외삽하면 없는 정보를 지어내는 것이다).
//
// 시계 주의: /joint_states 와 영상은 스탬프 출처가 같다 (시뮬은
// Gazebo, 실기는 시스템 클록). 그래서 use_sim_time 설정과 무관하게
// 성립한다 — CLAUDE.md §9 와 같은 근거다. 노드 클록을 섞으면
// 안 된다.
// ================================================================

double KauLaneDetectionNode::panAngleAt(
    const rclcpp::Time & stamp,
    bool * out_extrapolated) const
{
    if (out_extrapolated)
    {
        *out_extrapolated = false;
    }


    // 이력이 없으면 명령각으로 물러난다 (드라이버 기동 전 등).
    if (pan_history_.empty())
    {
        if (out_extrapolated)
        {
            *out_extrapolated = true;
        }

        return pan_cmd_deg_;
    }


    if (stamp <= pan_history_.front().stamp)
    {
        if (out_extrapolated)
        {
            *out_extrapolated = true;
        }

        return pan_history_.front().deg;
    }


    // 영상이 가장 최근 관절 샘플보다 앞서는 경우. 예전 동작과
    // 같아지는 지점이라 개선의 여지가 없다 — 그래서 표시해 둔다.
    if (stamp >= pan_history_.back().stamp)
    {
        if (out_extrapolated)
        {
            *out_extrapolated = true;
        }

        return pan_history_.back().deg;
    }


    // stamp 를 감싸는 첫 샘플 (이력은 스탬프 오름차순이다).
    const auto hi =
        std::lower_bound(
            pan_history_.begin(),
            pan_history_.end(),
            stamp,
            [](const PanSample & s, const rclcpp::Time & t)
            {
                return s.stamp < t;
            }
        );

    if (hi == pan_history_.begin())
    {
        return hi->deg;
    }

    const auto lo = std::prev(hi);


    const double span = (hi->stamp - lo->stamp).seconds();

    if (span <= 1e-9)
    {
        return hi->deg;
    }

    const double u = (stamp - lo->stamp).seconds() / span;

    return lo->deg + u * (hi->deg - lo->deg);
}


// ================================================================
// Pan 목표각 설정
//
// 상태기계는 목표각만 정한다. 실제 발행은 publishPanRamped 가
// pan_rate_deg_s_ 로 나눠서 한다.
//
// > 한때 이 분리를 썼다가 실기에서 "꺾다 말다" 하는 움직임이 나와
// > 스텝 방식으로 되돌린 적이 있다. 원인은 램프 자체가 아니라
// > **램프에 정착 대기(settled)가 없었던 것**이다 — 서보가 쫓아가는
// > 도중의 영상으로 재검출 판정을 반복했다. 지금은 (a) settled 가
// > "목표 도달 && 서보 정착" 을 둘 다 보고, (b) 재검출 기준이
// > found_count>0 에서 창 6개 x 2프레임으로 올라가 있어 그 실패
// > 모드가 막혀 있다. 그래서 램프를 다시 쓴다.
//
// physicar_driver_node::apply_pan 은 절대각[rad]을 받아 내부에서
// ±30도로 한 번 더 클램프한다. 여기서도 pan_max_deg_ 로 먼저
// 클램프해 설정상의 상한을 지킨다.
//
// pan_sign_ 은 하드웨어 배선용 뒤집개다. 탐색 방향(어느 쪽을
// 볼 것인가)은 pan_search_dir_ 가 정하며 이것과 무관하다.
// ================================================================

void KauLaneDetectionNode::commandPan(double goal_deg)
{
    pan_goal_deg_ =
        std::clamp(
            goal_deg,
            -pan_max_deg_,
            pan_max_deg_
        );
}


void KauLaneDetectionNode::publishPanRamped(
    const rclcpp::Time & frame_stamp)
{
    // 첫 프레임과 스탬프가 튄 프레임은 공칭 주기로 대체한다.
    double dt = 1.0 / 14.0;

    if (pan_ramp_time_init_)
    {
        dt = (frame_stamp - pan_ramp_last_stamp_).seconds();

        if (dt <= 0.0 || dt > 1.0)
        {
            dt = 1.0 / 14.0;
        }
    }

    pan_ramp_last_stamp_ = frame_stamp;

    pan_ramp_time_init_ = true;


    const double max_step =
        std::max(0.0, pan_rate_deg_s_) * dt;

    const double remain = pan_goal_deg_ - pan_cmd_deg_;


    // rate <= 0 이면 램프를 끄고 즉시 목표로 간다 (예전 스텝 동작).
    if (max_step <= 0.0 || std::abs(remain) <= max_step)
    {
        pan_cmd_deg_ = pan_goal_deg_;
    }
    else
    {
        pan_cmd_deg_ += std::copysign(max_step, remain);
    }


    std_msgs::msg::Float64 msg;

    msg.data =
        pan_sign_ * pan_cmd_deg_ * M_PI / 180.0;

    camera_pan_publisher_->publish(msg);
}


// ================================================================
// Pan 램프 발행
//
// 목표각을 향해 한 프레임분(pan_rate_deg_s_ * dt)만 다가가서
// 발행한다. 매 프레임 무조건 호출된다 — 목표가 안 바뀐 프레임에도
// 램프가 진행 중일 수 있기 때문이다.
//
// dt 는 노드 클록이 아니라 영상 프레임 스탬프로 잰다. use_sim_time
// 이 false 인데 센서가 sim time 을 달고 오면 노드 클록 기준 dt 가
// 터무니없이 나와 램프가 한 프레임에 끝나 버린다 (CLAUDE.md §9
// 시계 주의와 같은 함정).
// ================================================================

// ================================================================
// 기동 tilt 발행
//
// pan 과 달리 램프도 상태기계도 없다. BEV 가 전제하는 고정 자세를
// 세우는 것이 전부다.
//
// 반복하는 이유: 드라이버(실기)나 gz 브리지(시뮬)가 이 노드보다
// 늦게 뜨면 첫 발행이 조용히 유실된다. 반대로 계속 쏘면 웹UI 의
// 틸트 슬라이더를 매초 되돌려 버리므로 정해진 횟수만 쏘고 멈춘다.
//
// 부호는 /camera/tilt 규약 그대로 **+ 가 아래**다
// (camera_tilt_joint axis = +Y, URDF/SDF 공통).
// ================================================================

void KauLaneDetectionNode::publishCameraTilt()
{
    // 남이 이미 다른 각을 잡았으면 되돌리지 않는다.
    if (camera_tilt_yielded_)
    {
        camera_tilt_timer_->cancel();

        return;
    }

    std_msgs::msg::Float64 msg;

    msg.data = camera_tilt_deg_ * M_PI / 180.0;

    camera_tilt_last_sent_rad_ = msg.data;

    camera_tilt_publisher_->publish(msg);

    if (--camera_tilt_ticks_left_ <= 0)
    {
        camera_tilt_timer_->cancel();
    }
}


// ================================================================
// /camera/tilt 에코 — 남의 명령이면 양보한다
//
// 예전에는 이 노드가 기동 후 camera_tilt_repeat_s 동안 1 Hz 로
// 계속 쏘기만 해서, 그 사이에
//
//   ros2 topic pub -1 /camera/tilt std_msgs/msg/Float64 "{data: 0.1745}"
//
// 로 손수 세운 각이 다음 틱에 camera_tilt_deg 로 되돌아갔다.
// 틸트를 훑어 보려면 매번 yaml 을 고치고 재시작해야 했다.
//
// 이제는 우리가 마지막으로 보낸 값과 다른 값이 토픽에 오면
// **그 즉시 손을 뗀다** (타이머도 끈다). 우리 발행도 같은 토픽으로
// 돌아오므로 비교 대상은 "마지막으로 우리가 보낸 각" 이다.
//
// 대신 BEV 세 행은 그 각을 모른다. 그래서 새 각 기준 유도값을
// 로그로 찍어 준다 — 그대로 ros2 param set 하면 기하가 맞는다.
//
// 다시 이 노드가 주인이 되려면 camera_tilt_deg 를 param set 한다
// (refreshCameraTilt 가 양보를 풀고 즉시 발행한다).
// ================================================================

void KauLaneDetectionNode::tiltEchoCallback(
    const std_msgs::msg::Float64::SharedPtr msg)
{
    if (!camera_tilt_enable_)
    {
        return;
    }

    // 우리가 보낸 것과 같으면 (부동소수 왕복 오차 허용) 무시한다.
    if (
        std::isfinite(camera_tilt_last_sent_rad_) &&
        std::abs(msg->data - camera_tilt_last_sent_rad_) < 1e-6
    )
    {
        return;
    }


    const double external_deg = msg->data * 180.0 / M_PI;

    // 같은 외부 각이 반복해서 오면 한 번만 경고한다.
    if (
        camera_tilt_yielded_ &&
        std::isfinite(camera_tilt_external_deg_) &&
        std::abs(external_deg - camera_tilt_external_deg_) < 1e-6
    )
    {
        return;
    }


    camera_tilt_yielded_ = true;

    // camera_tilt_deg_ 는 **파라미터 캐시**다. 여기에 외부 각을 덮으면
    // 다음 프레임 refreshCameraTilt() 가 "param 이 바뀌었다" 로 오인해
    // 원래 각을 되쏘고 양보가 즉시 풀린다 (실측으로 잡은 버그).
    camera_tilt_external_deg_ = external_deg;


    double van = 0.0;

    double top = 0.0;

    double bot = 0.0;

    if (tiltDerivedBevRows(external_deg, van, top, bot))
    {
        RCLCPP_WARN(
            this->get_logger(),
            "외부 /camera/tilt %+.2f deg 를 받아 기동 tilt 를 양보한다. "
            "BEV 세 행은 %+.2f deg 를 전제하므로 지평선이 %.1f px "
            "어긋난 상태다. 이 각으로 맞추려면:\n"
            "  ros2 param set /kau_lane_detection_node bev_vanishing_y %.2f\n"
            "  ros2 param set /kau_lane_detection_node bev_src_top_y %.2f\n"
            "  ros2 param set /kau_lane_detection_node bev_src_bottom_y %.2f",
            external_deg,
            camera_tilt_bev_ref_deg_,
            std::abs(bev_vanishing_y_ - van),
            van,
            top,
            bot
        );
    }
    else
    {
        RCLCPP_WARN(
            this->get_logger(),
            "외부 /camera/tilt %+.2f deg 를 받아 기동 tilt 를 양보한다. "
            "BEV 세 행은 %+.2f deg 를 전제한다 — 유도값은 "
            "CameraInfo 수신 후에 찍는다.",
            external_deg,
            camera_tilt_bev_ref_deg_
        );
    }
}


// ================================================================
// camera_tilt_deg 런타임 재독 (bev_* 와 같은 idiom)
//
// 생성자에서 한 번만 읽던 값이라 ros2 param set 이 조용히 무시됐다.
// 이제 매 프레임 다시 읽어, 바뀌었으면 즉시 한 번 발행한다.
// param set 은 "다시 노드가 주인" 이라는 명시적 의사표시이므로
// 외부 양보 상태도 같이 푼다.
// ================================================================

void KauLaneDetectionNode::refreshCameraTilt()
{
    if (!camera_tilt_enable_)
    {
        return;
    }


    const double tilt_deg =
        this->get_parameter(
            "camera_tilt_deg"
        ).as_double();

    if (std::abs(tilt_deg - camera_tilt_deg_) < 1e-9)
    {
        return;
    }


    camera_tilt_deg_ = tilt_deg;

    // param set 은 "다시 이 노드가 주인" 이라는 명시적 의사표시다.
    camera_tilt_yielded_ = false;

    camera_tilt_external_deg_ =
        std::numeric_limits<double>::quiet_NaN();


    std_msgs::msg::Float64 msg;

    msg.data = camera_tilt_deg_ * M_PI / 180.0;

    camera_tilt_last_sent_rad_ = msg.data;

    camera_tilt_publisher_->publish(msg);


    double van = 0.0;

    double top = 0.0;

    double bot = 0.0;

    if (tiltDerivedBevRows(tilt_deg, van, top, bot))
    {
        RCLCPP_INFO(
            this->get_logger(),
            "camera_tilt_deg -> %+.2f deg 발행. 이 각 기준 BEV 유도값은 "
            "vanishing %.2f / top %.2f / bottom %.2f 다 "
            "(현재 세 행은 %+.2f deg 기준: %.2f / %.2f / %.2f).",
            tilt_deg,
            van,
            top,
            bot,
            camera_tilt_bev_ref_deg_,
            bev_vanishing_y_,
            bev_src_top_y_,
            bev_src_bottom_y_
        );
    }
    else
    {
        RCLCPP_INFO(
            this->get_logger(),
            "camera_tilt_deg -> %+.2f deg 발행.",
            tilt_deg
        );
    }
}


// ================================================================
// 틸트 -> BEV 세 행
//
//   alpha = tilt + MOUNT_PITCH_BIAS_DEG        (아래가 +)
//   y(d)  = cy + fy*tan(atan(h/d) - alpha)     지면거리 d 인 행
//   van   = cy - fy*tan(alpha)                 지평선 행
//
// 지면 밴드 d 는 **현재 세 행에서 역산**해 보존한다. 즉 look-ahead
// 는 그대로 두고 틸트가 바뀐 만큼만 세 행을 함께 옮긴다.
//
//   d(y) = h / tan(atan((y - cy)/fy) + alpha_ref)
//
// MOUNT_PITCH_BIAS_DEG 0.284 는 마운트 실측편차다 (CLAUDE.md §3 —
// 옛 bev_vanishing_y 179.0 이 당시 cy 180.0 보다 1 px 위였던 것).
// ================================================================

bool KauLaneDetectionNode::tiltDerivedBevRows(
    double tilt_deg,
    double & vanishing_y,
    double & top_y,
    double & bottom_y) const
{
    static constexpr double MOUNT_PITCH_BIAS_DEG = 0.284;

    if (
        !camera_calibrated_ ||
        camera_matrix_.empty() ||
        camera_height_cm_ <= 1e-6
    )
    {
        return false;
    }


    const double fy = camera_matrix_.at<double>(1, 1);

    const double cy = camera_matrix_.at<double>(1, 2);

    if (fy <= 1e-6)
    {
        return false;
    }


    const double h = camera_height_cm_;

    const double a_ref =
        (camera_tilt_bev_ref_deg_ + MOUNT_PITCH_BIAS_DEG) * M_PI / 180.0;

    const double a_new =
        (tilt_deg + MOUNT_PITCH_BIAS_DEG) * M_PI / 180.0;


    // 현재 행 -> 지면거리 (기준 틸트로 역산)
    const auto ground_cm =
        [&](double y)
        {
            const double depression =
                std::atan((y - cy) / fy) + a_ref;

            const double t = std::tan(depression);

            return (t > 1e-6) ? (h / t) : -1.0;
        };

    // 지면거리 -> 행 (새 틸트로 재유도)
    const auto row_of =
        [&](double d_cm)
        {
            return cy + fy * std::tan(std::atan(h / d_cm) - a_new);
        };


    const double d_top = ground_cm(bev_src_top_y_);

    const double d_bot = ground_cm(bev_src_bottom_y_);

    if (d_top <= 0.0 || d_bot <= 0.0)
    {
        return false;
    }


    vanishing_y = cy - fy * std::tan(a_new);

    top_y = row_of(d_top);

    bottom_y = row_of(d_bot);

    return true;
}
