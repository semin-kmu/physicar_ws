// ====================================================================
// pan_aim.cpp
//
// Pan 조준각 계산 (feedforward). 전역경로 근거와 차선 근거 두 가지.
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
// 조준 결과 정규화 — 두 소스의 공통 후미
// ================================================================

double KauLaneDetectionNode::panAimResultDeg(
    double bearing_rel,
    bool * out_valid) const
{
    while (bearing_rel > M_PI)
    {
        bearing_rel -= 2.0 * M_PI;
    }

    while (bearing_rel < -M_PI)
    {
        bearing_rel += 2.0 * M_PI;
    }


    if (out_valid)
    {
        *out_valid = true;
    }


    return pan_aim_gain_ * bearing_rel * 180.0 / M_PI;
}


// ================================================================
// Pan 조준각 (feedforward)
//
// 설계 근거는 선언부(헤더) 주석 참고.
//
// 1. /path/global 전 구간에서 자차 최근접점을 찾는다
// 2. 거기서 호길이로 pan_aim_lookahead_m 만큼 전진한 점을 구한다
// 3. 그 점이 base_link 기준 몇 도에 있는지(방위각) 잰다
// 4. pan_aim_gain 을 곱해 목표각으로 낸다 (클램프는 commandPan)
//
// 호길이는 정확히 잰다. 구간 전체는 seg_length(사전계산값), 마지막
// 부분 구간만 segLength() 이분탐색이다. curvatureAheadKappa 의
// u 균일 근사를 재사용하면 안 된다 — 그쪽 주석대로 "트리거 판정용
// 이지 제어에 쓰지 않는" 값이다.
// ================================================================

double KauLaneDetectionNode::panAimGoalDeg(
    const VehiclePose & pose,
    bool * out_valid) const
{
    if (out_valid)
    {
        *out_valid = false;
    }


    if (!pose.valid || !global_path_)
    {
        return 0.0;
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
        return 0.0;
    }


    const std::size_t nseg = global_path_->ctrl_x.size() / nctrl;

    if (
        nseg == 0 ||
        global_path_->seg_length.size() != nseg
    )
    {
        return 0.0;
    }


    // 구간 k 의 제어점을 꺼내는 helper.
    const auto seg_ctrl =
        [&](std::size_t k)
        {
            kau::bezier::Ctrl c(nctrl);

            for (
                std::size_t j = 0;
                j < nctrl;
                ++j
            )
            {
                const std::size_t idx = k * nctrl + j;

                c[j].x = global_path_->ctrl_x[idx];
                c[j].y = global_path_->ctrl_y[idx];
            }

            return c;
        };


    // 자차 위치 [cm] (map 프레임, KauPath 단위와 맞춘다)
    const kau::bezier::Point2 ego{
        pose.x_m * 100.0,
        pose.y_m * 100.0
    };


    // ------------------------------------------------------------
    // 1. 최근접점 (curvatureAheadKappa 와 같은 선형 탐색)
    // ------------------------------------------------------------

    std::size_t best_seg = 0;

    double best_d2 = std::numeric_limits<double>::max();

    double best_u = 0.0;


    for (
        std::size_t k = 0;
        k < nseg;
        ++k
    )
    {
        const kau::bezier::Ctrl ctrl = seg_ctrl(k);

        const kau::bezier::Nearest near =
            kau::bezier::nearestOnSeg(ctrl, ego);

        const kau::bezier::Point2 foot =
            kau::bezier::evalSeg(ctrl, near.u);

        const double dx = foot.x - ego.x;

        const double dy = foot.y - ego.y;

        const double d2 = dx * dx + dy * dy;

        if (d2 < best_d2)
        {
            best_d2 = d2;

            best_seg = k;

            best_u = near.u;
        }
    }


    // ------------------------------------------------------------
    // 2. 호길이로 lookahead 만큼 전진
    //
    // 부분 구간 길이는 segLength(ctrl, u0, u1) 가 정확히 낸다.
    // 목표 길이에 닿는 u 는 단조증가라 이분탐색이면 충분하다.
    // ------------------------------------------------------------

    double remaining = pan_aim_lookahead_m_ * 100.0;

    if (!(remaining > 0.0))
    {
        // 룩어헤드가 0 이하면 조준할 대상이 없다.
        return 0.0;
    }


    std::size_t cur_seg = best_seg;

    double cur_u = best_u;

    kau::bezier::Ctrl cur_ctrl = seg_ctrl(cur_seg);

    // 전 구간을 다 돌아도 못 채우면 마지막 점을 쓴다 (개곡선 종단).
    bool reached = false;


    for (
        std::size_t step = 0;
        step <= nseg;
        ++step
    )
    {
        const double rest =
            kau::bezier::segLength(cur_ctrl, cur_u, 1.0);

        if (remaining <= rest)
        {
            // 이 구간 안에서 끝난다. u 를 이분탐색으로 찾는다.
            double lo = cur_u;

            double hi = 1.0;

            for (
                int it = 0;
                it < 32;
                ++it
            )
            {
                const double mid = 0.5 * (lo + hi);

                if (
                    kau::bezier::segLength(cur_ctrl, cur_u, mid) <
                    remaining
                )
                {
                    lo = mid;
                }
                else
                {
                    hi = mid;
                }
            }

            cur_u = 0.5 * (lo + hi);

            reached = true;

            break;
        }


        remaining -= rest;


        std::size_t next = cur_seg + 1;

        if (next >= nseg)
        {
            if (!global_path_->is_closed)
            {
                // 개곡선 종단. 남은 거리를 못 채웠으므로 끝점을 쓴다.
                cur_u = 1.0;

                reached = true;

                break;
            }

            next = 0;
        }

        if (next == best_seg)
        {
            // 폐곡선을 한 바퀴 다 돌았다 (lookahead 가 전체 길이보다 김)
            cur_u = 1.0;

            reached = true;

            break;
        }

        cur_seg = next;

        cur_u = 0.0;

        cur_ctrl = seg_ctrl(cur_seg);
    }


    if (!reached)
    {
        return 0.0;
    }


    // ------------------------------------------------------------
    // 3. 방위각 [deg] (base_link 기준, 왼쪽이 +)
    //
    // camera_pan_joint 는 axis +Z 라 +rad 이 왼쪽이므로 이 부호가
    // 그대로 pan 명령의 부호가 된다 (§8 회전 부호).
    // ------------------------------------------------------------

    const kau::bezier::Point2 target =
        kau::bezier::evalSeg(cur_ctrl, cur_u);

    const double bearing_map =
        std::atan2(
            target.y - ego.y,
            target.x - ego.x
        );

    double bearing_rel = bearing_map - pose.yaw_rad;

    // [-pi, pi] 로 정규화. 폐곡선을 한 바퀴 돈 경우 등에서
    // 뒤쪽 점이 잡히면 여기서 접힌다.
    return panAimResultDeg(bearing_rel, out_valid);
}


// ================================================================
// Pan 조준 — 차선 기반 (기본 소스)
//
// 설계 근거는 선언부(헤더) 주석 참고. 요약하면 근거가 전역경로가
// 아니라 이 노드가 방금 만든 경로이고, 그 경로가 이미 base_link
// 기준이라 방위각이 적분되지 않는다는 것이다.
// ================================================================

double KauLaneDetectionNode::panAimGoalDegFromLane(
    const LanePath & path,
    bool * out_valid) const
{
    if (out_valid)
    {
        *out_valid = false;
    }


    // 게이트를 통과하지 못한 경로는 근거로 쓰지 않는다. 기각된
    // 경로도 built 로 남아 있지만 그건 debug 화면용이다.
    if (
        !path.valid ||
        path.ctrl.size() < 2
    )
    {
        return 0.0;
    }


    // ------------------------------------------------------------
    // 1. 근거 길이 — "가려지지 않은 부분" 은 여기서 정해진다
    //
    // valid_length_cm 은 근거 점이 끊기기 전까지의 호길이다.
    // 가림이 생기면 이 값이 저절로 줄어든다.
    // ------------------------------------------------------------

    const double evidence_cm = path.valid_length_cm;

    if (
        !std::isfinite(evidence_cm) ||
        evidence_cm < pan_aim_min_evidence_cm_
    )
    {
        return 0.0;
    }


    // ------------------------------------------------------------
    // 2. 조준 거리
    //
    // pan_aim_lookahead_m 은 자차 기준 거리로 읽히는 이름이다.
    // 경로는 카메라 앞 x_base_cm 부터 시작하므로 그만큼 빼야
    // 전역경로 소스와 같은 뜻이 된다.
    //
    // 축척을 못 구하면(CameraInfo 이전) 빼지 않는다 — 그 구간은
    // imageCallback 이 이미 조기 반환하므로 여기까지 오지 않는다.
    // ------------------------------------------------------------

    double sx = 0.0;

    double sy = 0.0;

    double x_base_cm = 0.0;

    const double start_offset_cm =
        bevScale(sx, sy, x_base_cm)
            ? (x_base_cm + path_x_offset_cm_)
            : 0.0;


    const double want_cm =
        pan_aim_lookahead_m_ * 100.0 - start_offset_cm;

    // 근거보다 멀리 조준하지 않는다. 그 뒤는 외삽이다.
    const double d_cm = std::min(want_cm, evidence_cm);


    if (!(d_cm > 1e-6))
    {
        // 룩어헤드가 경로 시작점보다 가깝다. 조준할 대상이 없다.
        return 0.0;
    }


    // ------------------------------------------------------------
    // 3. 호길이 d_cm 인 u (이분탐색)
    //
    // 호길이는 u 에 대해 단조증가라 이분탐색이면 충분하다.
    // panAimGoalDeg 2단계와 같은 방법이고, 조각이 하나라 구간
    // 순회가 없다.
    // ------------------------------------------------------------

    double lo = 0.0;

    double hi = 1.0;

    for (
        int it = 0;
        it < 32;
        ++it
    )
    {
        const double mid = 0.5 * (lo + hi);

        if (
            kau::bezier::segLength(path.ctrl, 0.0, mid) < d_cm
        )
        {
            lo = mid;
        }
        else
        {
            hi = mid;
        }
    }


    const double u = 0.5 * (lo + hi);


    // ------------------------------------------------------------
    // 4. 방위각 [deg] (base_link 기준, 왼쪽이 +)
    //
    // ctrl 이 이미 base_link 기준이고 자차가 원점이므로 뺄 것이
    // 없다. camera_pan_joint 는 axis +Z 라 +rad 이 왼쪽이므로
    // 이 부호가 그대로 pan 명령의 부호가 된다 (§8 회전 부호).
    // ------------------------------------------------------------

    const kau::bezier::Point2 target =
        kau::bezier::evalSeg(path.ctrl, u);


    if (
        !std::isfinite(target.x) ||
        !std::isfinite(target.y)
    )
    {
        return 0.0;
    }


    // 조준점이 자차 원점과 겹치면 방위가 정의되지 않는다.
    // 뒤쪽 점(x <= 0)도 조준 대상이 아니다 — 경로가 뒤집혔다는
    // 뜻이라 그 각으로 돌리면 정반대를 본다.
    if (
        target.x <= 1e-6 ||
        std::hypot(target.x, target.y) < 1e-6
    )
    {
        return 0.0;
    }


    const double bearing_rel =
        std::atan2(
            target.y,
            target.x
        );


    return panAimResultDeg(bearing_rel, out_valid);
}


// ================================================================
// 조준각 상한 [deg]
//
// 설계 근거는 선언부(헤더) 주석 참고. 지금 광학(반각 50도)에서는
// pan_max_deg(30) 가 항상 작아 이 함수가 걸리지 않는다.
// ================================================================

double KauLaneDetectionNode::panAimLimitDeg() const
{
    if (
        !camera_calibrated_ ||
        camera_matrix_.empty()
    )
    {
        return pan_max_deg_;
    }


    const double fx = camera_matrix_.at<double>(0, 0);

    const double cx = camera_matrix_.at<double>(0, 2);


    if (
        !std::isfinite(fx) ||
        fx <= 1e-6 ||
        !std::isfinite(cx) ||
        cx <= 0.0
    )
    {
        return pan_max_deg_;
    }


    const double half_fov_deg =
        std::atan(cx / fx) * 180.0 / M_PI;

    const double fov_limit_deg =
        half_fov_deg - pan_aim_fov_margin_deg_;


    if (!(fov_limit_deg > 0.0))
    {
        // 여유가 반각보다 크다. 설정 실수로 보고 막지 않는다
        // (판단 불가면 fail-open — obstacleAheadOnSide 와 같은 방침).
        RCLCPP_WARN_ONCE(
            this->get_logger(),
            "pan_aim_fov_margin_deg(%.1f) 가 수평 반각(%.1f) 이상이다. "
            "FOV 상한을 적용하지 않는다.",
            pan_aim_fov_margin_deg_,
            half_fov_deg
        );

        return pan_max_deg_;
    }


    return std::min(pan_max_deg_, fov_limit_deg);
}
