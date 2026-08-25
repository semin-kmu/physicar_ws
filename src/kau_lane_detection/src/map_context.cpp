// ====================================================================
// map_context.cpp
//
// map 프레임 근거. 측위 TF, 전역경로 곡률, 장애물 판정.
//
// 설계 근거와 실측표는 CLAUDE.md 참고.
// ====================================================================

#include "kau_lane_detection/kau_lane_detection_node.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/exceptions.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <functional>

namespace geom = kau_lane::geom;


// ================================================================
// /path/global, /perception/obstacles 수신
//
// 둘 다 판정에만 쓰므로 최신(또는 latched) 한 장만 들고 있으면
// 된다. 단일 스레드 executor 라 콜백이 직렬화되므로 잠금은 없다.
// ================================================================

void KauLaneDetectionNode::globalPathCallback(
    const kau_msgs::msg::KauPath::SharedPtr msg)
{
    global_path_ = msg;
}


void KauLaneDetectionNode::obstaclesCallback(
    const kau_msgs::msg::ObstacleCircleArray::SharedPtr msg)
{
    latest_obstacles_ = msg;
}


// ================================================================
// map <- base_link 조회
//
// 설계 근거는 선언부(헤더) 주석 참고. 실패는 흔한 일이다 —
// 측위가 아직 안 떴거나, pure localization 초기화 전이거나,
// TF 버퍼가 그 시각을 못 채운 순간일 수 있다. 그때마다 로그를
// 쏟으면 소음이 되므로 THROTTLE 한다.
// ================================================================

KauLaneDetectionNode::VehiclePose KauLaneDetectionNode::lookupVehiclePose(
    const rclcpp::Time & frame_stamp) const
{
    VehiclePose pose;

    if (!tf_buffer_)
    {
        return pose;
    }

    geometry_msgs::msg::TransformStamped transform;

    try
    {
        transform = tf_buffer_->lookupTransform("map", "base_link", frame_stamp);
    }
    catch (const tf2::ExtrapolationException &)
    {
        // frame_stamp 가 TF 버퍼 최신값보다 살짝 앞서는 흔한 경우.
        // 몇 ms 오차는 이 판정 용도에 무관하므로 최신 가용 TF로 대체.
        try
        {
            transform = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
        }
        catch (const tf2::TransformException & e)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "map <- base_link TF 조회 실패: %s. 경로 map 변환 / pan 트리거 판정을 건너뛴다.", e.what());
            return pose;
        }
    }
    catch (const tf2::TransformException & e)
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
            "map <- base_link TF 조회 실패: %s. 경로 map 변환 / pan 트리거 판정을 건너뛴다.", e.what());
        return pose;
    }

    pose.x_m = transform.transform.translation.x;
    pose.y_m = transform.transform.translation.y;

    // yaw 만 필요하다 (평면 주행이므로 roll/pitch 는 버린다).
    const auto & q = transform.transform.rotation;

    const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);

    pose.yaw_rad = std::atan2(siny_cosp, cosy_cosp);
    pose.valid = true;

    return pose;
}


// ================================================================
// 자차 위치에서 /path/global 위 전방 곡률 룩어헤드
//
// 설계 근거는 선언부(헤더) 주석 참고.
// ================================================================

double KauLaneDetectionNode::curvatureAheadKappa(
    const VehiclePose & pose) const
{
    if (!pose.valid || !global_path_)
    {
        return -1.0;
    }

    const std::size_t nctrl =
        static_cast<std::size_t>(global_path_->degree) + 1;

    if (
        nctrl < 2 ||
        global_path_->ctrl_x.size() != global_path_->ctrl_y.size() ||
        global_path_->ctrl_x.empty() ||
        global_path_->ctrl_x.size() % nctrl != 0
    )
    {
        return -1.0;
    }

    const std::size_t nseg = global_path_->ctrl_x.size() / nctrl;

    if (
        nseg == 0 ||
        global_path_->seg_length.size() != nseg ||
        global_path_->seg_kappa_max.size() != nseg
    )
    {
        return -1.0;
    }


    // 자차 위치 [cm] (map 프레임, KauPath 단위와 맞춘다)
    const kau::bezier::Point2 ego{
        pose.x_m * 100.0,
        pose.y_m * 100.0
    };


    // 조각 s 의 제어점을 꺼낸다.
    const auto seg_ctrl =
        [&](std::size_t s)
        {
            kau::bezier::Ctrl ctrl(nctrl);

            for (
                std::size_t j = 0;
                j < nctrl;
                ++j
            )
            {
                const std::size_t idx = s * nctrl + j;

                ctrl[j].x = global_path_->ctrl_x[idx];
                ctrl[j].y = global_path_->ctrl_y[idx];
            }

            return ctrl;
        };


    // 주어진 조각 목록에서 최근접점. 조각 하나는 8.1 해석적 해법.
    const auto scan =
        [&](const std::vector<std::size_t> & segs,
            std::size_t & best_seg, double & best_u, double & best_dist)
        {
            best_dist = std::numeric_limits<double>::max();

            for (const std::size_t s : segs)
            {
                const kau::bezier::Nearest near =
                    kau::bezier::nearestOnSeg(seg_ctrl(s), ego);

                if (near.dist < best_dist)
                {
                    best_dist = near.dist;

                    best_seg  = s;

                    best_u    = near.u;
                }
            }
        };


    // 8.4 window: 직전 해의 조각을 중심으로 호길이 [-BACK, +FWD] 에
    // 걸치는 조각만 모은다. 폐곡선은 양쪽으로 순환한다.
    const auto window_segs =
        [&](std::size_t center)
        {
            std::vector<std::size_t> out{center};

            double back = 0.0;

            for (std::size_t k = 1; k < nseg && back < kTrackBackCm; ++k)
            {
                const std::size_t i =
                    (center >= k) ? (center - k)
                                  : (global_path_->is_closed
                                         ? (nseg + center - k) : nseg);

                if (i >= nseg)
                {
                    break;
                }

                out.push_back(i);

                back += global_path_->seg_length[i];
            }

            double fwd = 0.0;

            for (std::size_t k = 1; k < nseg && fwd < kTrackFwdCm; ++k)
            {
                std::size_t i = center + k;

                if (i >= nseg)
                {
                    if (!global_path_->is_closed)
                    {
                        break;
                    }

                    i -= nseg;
                }

                out.push_back(i);

                fwd += global_path_->seg_length[i];
            }

            return out;
        };


    std::size_t best_seg = 0;

    double best_u = 0.0;

    double best_dist = std::numeric_limits<double>::max();


    // 경로가 바뀌면 조각 인덱스의 뜻이 달라진다. 상태를 버린다.
    if (kappa_track_.valid && kappa_track_.seg >= nseg)
    {
        kappa_track_.valid = false;
    }

    bool used_window = false;

    if (kappa_track_.valid)
    {
        scan(window_segs(kappa_track_.seg), best_seg, best_u, best_dist);

        used_window = true;

        if (best_dist > kTrackGateCm)
        {
            // GATE 이탈. 연속 FAIL_LIMIT 회면 전역으로 되돌린다
            // (측위 점프 · 경로 교체 · 물리적 이탈).
            if (++kappa_track_.fails >= kTrackFailMax)
            {
                kappa_track_.valid = false;

                used_window = false;
            }
        }
        else
        {
            kappa_track_.fails = 0;
        }
    }

    if (!used_window)
    {
        // 8.3 전역 탐색. 첫 프레임과 GATE 연속 이탈 뒤에만 온다.
        std::vector<std::size_t> all(nseg);

        for (std::size_t s = 0; s < nseg; ++s)
        {
            all[s] = s;
        }

        scan(all, best_seg, best_u, best_dist);

        kappa_track_.fails = 0;
    }

    kappa_track_.seg   = best_seg;
    kappa_track_.u     = best_u;
    kappa_track_.valid = true;


    // best_seg, best_u 에서 시작해 lookahead 만큼 호길이로 걸으며
    // 지나는 구간들의 seg_kappa_max 중 최댓값을 취한다.
    //
    // u 는 균일 매개변수라 호길이 비례를 정확히 보장하지 않지만,
    // "구간이 얼마나 남았나" 근사로는 충분하다 — 트리거 판정용이지
    // 제어에 쓰지 않는다.
    double remaining =
        curvature_lookahead_m_ * 100.0 -
        global_path_->seg_length[best_seg] * (1.0 - best_u);

    double kappa_max =
        static_cast<double>(global_path_->seg_kappa_max[best_seg]);

    std::size_t s = best_seg;


    for (
        std::size_t step = 0;
        step < nseg && remaining > 0.0;
        ++step
    )
    {
        std::size_t next = s + 1;

        if (next >= nseg)
        {
            if (!global_path_->is_closed)
            {
                break;
            }

            next = 0;
        }

        if (next == best_seg)
        {
            // 폐곡선을 한 바퀴 다 돌았다 (lookahead 가 전체 길이보다 김)
            break;
        }

        s = next;

        kappa_max =
            std::max(
                kappa_max,
                static_cast<double>(global_path_->seg_kappa_max[s])
            );

        remaining -= global_path_->seg_length[s];
    }


    return kappa_max;
}


// ================================================================
// 소실된 쪽 전방에 장애물이 있는가 (map 프레임)
//
// 설계 근거는 선언부(헤더) 주석 참고.
// ================================================================

bool KauLaneDetectionNode::obstacleAheadOnSide(
    int dir,
    const VehiclePose & pose,
    const rclcpp::Time & frame_stamp) const
{
    if (
        !obstacle_pan_block_enable_ ||
        dir == 0 ||
        !pose.valid ||
        !latest_obstacles_
    )
    {
        return false;
    }


    if (
        latest_obstacles_->status !=
        kau_msgs::msg::ObstacleCircleArray::STATUS_OK
    )
    {
        // non-OK 프레임은 obstacles 가 항상 비어 있다 (msg 규약).
        // "장애물 없음" 이 아니라 "판단 불가" 이므로 막지 않는다.
        return false;
    }


    // 기준은 node clock 이 아니라 지금 처리 중인 영상의 시각이다
    // (선언부 주석 참고).
    const double age =
        (frame_stamp -
         rclcpp::Time(latest_obstacles_->header.stamp)).seconds();

    if (
        age < 0.0 ||
        age > obstacle_scan_timeout_s_
    )
    {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            5000,
            "/perception/obstacles 가 %.2fs 됐다 (상한 %.2f). "
            "장애물 가림 판정을 건너뛴다.",
            age,
            obstacle_scan_timeout_s_
        );

        return false;
    }


    const double cos_yaw = std::cos(pose.yaw_rad);

    const double sin_yaw = std::sin(pose.yaw_rad);


    for (const auto & c : latest_obstacles_->obstacles)
    {
        const double dx = static_cast<double>(c.center_x) - pose.x_m;

        const double dy = static_cast<double>(c.center_y) - pose.y_m;

        // map -> 차량 기준 (전방 +x, 좌측 +y).
        const double fwd = dx * cos_yaw + dy * sin_yaw;

        const double left = -dx * sin_yaw + dy * cos_yaw;


        // 반지름만큼 안쪽으로 당겨 "가장자리까지" 로 판정한다.
        const double edge_fwd =
            fwd - std::copysign(
                static_cast<double>(c.radius),
                fwd
            );

        if (
            edge_fwd < 0.0 ||
            edge_fwd > obstacle_ahead_max_m_
        )
        {
            continue;
        }


        // 반대쪽은 무시. dir > 0 (왼쪽) 이면 left > 0 이어야 한다.
        const double side = (dir > 0) ? left : -left;

        if (side < 0.0)
        {
            continue;
        }


        // 차로 폭 밖(트랙 경계벽 등)은 무시.
        if (
            std::abs(left) >
            obstacle_lane_lateral_m_ + static_cast<double>(c.radius)
        )
        {
            continue;
        }


        return true;
    }


    return false;
}
