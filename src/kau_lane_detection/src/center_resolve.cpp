// ====================================================================
// center_resolve.cpp
//
// 도로 단면 판정. 어느 선인지 고르고 중앙선 대비 자차 횡거리를 정한다.
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
// 자차 종축 폴리라인
//
// src 사다리꼴의 옆변은 소실점 (bev_src_center_x_,
// bev_vanishing_y_) 을 지나도록 만들어져 있다. 요각 0 인
// 카메라에서 지면 평행선의 소실점 열은 주점 cx 이므로,
// bev_src_center_x_ == cx 일 때 사다리꼴의 대칭축은 지면에서
// 횡거리 0 인 직선, 즉 차량 종축과 같다.
//
// 아래(가까움) -> 위(멂) 순서라 진행방향과 같고, 이 폴리라인을
// 기준선으로 쓰면 lateralOffsets 의 부호가 그대로 "차량 오른쪽"
// 이 된다.
// ================================================================

std::vector<cv::Point2d> KauLaneDetectionNode::egoAxis(
    int bev_height) const
{
    double ex = 0.5 * static_cast<double>(bev_out_width_);


    if (bev_dst_points_.size() == 4)
    {
        ex =
            0.5 *
            static_cast<double>(
                bev_dst_points_[0].x + bev_dst_points_[1].x);
    }


    return {
        cv::Point2d(ex, static_cast<double>(bev_height - 1)),
        cv::Point2d(ex, 0.0)
    };
}


// ================================================================
// 관측선 -> 도로 단면의 자리
//
// 선이 하나만 보일 때 그것이 왼쪽 도로 경계인지 오른쪽
// 경계인지는 관측만으로는 알 수 없다. 히스토그램이 화면
// 반쪽 중 어디서 찾았는지는 근거가 못 된다. 코너에서 차선이
// 가로로 누우면 화면 왼쪽에 오른쪽 선이 온다.
//
// 맵 단면(road_map.hpp)이 있으면 답이 나온다. 자차가 중앙선
// 에서 몇 cm 떨어져 있는지 알면 각 선이 자차 종축에서 몇 cm
// 떨어져 있어야 하는지가 정해진다.
//
//   자차가 중앙선 위에 있다고 두면 ego = 0cm
//
//     왼쪽 흰선 (-31.25)  기대 횡거리 -31.25cm
//     중앙선    (  0.00)  기대 횡거리   0.00cm
//     오른쪽흰선(+31.25)  기대 횡거리 +31.25cm
//
// 흰선 두 후보가 62.5cm 떨어져 있으므로 판정 여유가 넉넉하다.
// 실측이 어느 후보에서도 0.75W 밖이면 도로의 선이 아니다
// (연석, 체커보드, 다른 구간 차선). 그때는 고르지 않는다.
//
// 기준점 ego_offset_cm 은 centerOffsetPriorCm() 이 준다. 노란선
// 이나 흰선 두 개로 잰 값이 아직 싱싱하면 그것을 쓰고, 없으면
// 중앙선 위에 있다고 둔다.
//
// 흰선 하나로 얻은 값(src 3)은 절대 기준점이 되지 않는다. 그
// 값은 이 기준점으로 자리를 골라서 나온 것이라, 되먹이면 오차가
// 스스로를 키운다.
// ================================================================

double KauLaneDetectionNode::centerOffsetPriorCm() const
{
    // 30 프레임 = 14Hz 에서 약 2 초.
    if (center_prior_age_ <= 30)
    {
        return center_prior_offset_cm_;
    }


    // 근거가 없을 때의 최선의 가정. 차로 개념이 없으므로
    // "지정 차선 중심" 같은 기본값은 없다 — 중앙선 자체가
    // 목표이니 근거가 없을 때도 중앙선 위에 있다고 둔다.
    return 0.0;
}


double KauLaneDetectionNode::lineOffsetCm(std::size_t i) const
{
    if (i >= kau_road::LINE_COUNT)
    {
        return 0.0;
    }


    return kau_road::LINES[i].offset_cm *
           (lane_width_cm_ / kau_road::LANE_WIDTH_CM);
}


double KauLaneDetectionNode::roadHalfWidthCm() const
{
    return kau_road::roadHalfWidthCm() *
           (lane_width_cm_ / kau_road::LANE_WIDTH_CM);
}


int KauLaneDetectionNode::identifyLine(
    const std::vector<cv::Point2d> & track,
    kau_road::LineType type,
    double ego_offset_cm,
    double sx,
    int bev_height) const
{
    if (track.size() < 2 || sx <= 1e-9)
    {
        return -1;
    }


    const double d_cm =
        geom::medianLateralOffset(
            egoAxis(bev_height),
            track
        ) * sx;


    const double ego_cm = ego_offset_cm;


    int best = -1;

    double best_res = 0.75 * lane_width_cm_;


    for (
        std::size_t i = 0;
        i < kau_road::LINE_COUNT;
        ++i
    )
    {
        if (kau_road::LINES[i].type != type)
        {
            continue;
        }


        const double res =
            std::abs(
                d_cm -
                (lineOffsetCm(i) - ego_cm));


        if (res < best_res)
        {
            best_res = res;

            best = static_cast<int>(i);
        }
    }


    return best;
}


// ================================================================
// 중앙선 대비 자차 위치 갱신
//
// 원리는 한 줄이다. 중앙선을 기준선으로 두고 자차 위치의 부호
// 있는 횡거리를 재면, 그게 곧 자차가 중앙선을 얼마나 잘 따라가고
// 있는지다.
//
// 차로 개념이 없으므로 이 값으로 "차로 번호"를 매기거나
// "지정 차선 이탈"을 판정하지 않는다. 순수하게 진단값이고,
// identifyLine 이 흰선 하나를 disambiguate 할 때 쓰는 기준점이다.
//
// 세 가지를 조심해야 한다.
//
// 1. 기준선은 반드시 "관측된" 선이어야 한다.
//    16-c 소실 복원은 남은 선에서 차로 폭만큼 평행이동해
//    없는 선을 만들어 낸다. 그 선으로 이 값을 재면
//    "차로 폭만큼 옆에 있을 것" 이라는 가정을 관측처럼
//    되돌려 읽는 순환이 된다. 그래서 16-b 에서, 복원 전에
//    부른다.
//
// 2. 자차 위치는 BEV 안에 없다. BEV 아랫변은 카메라 앞
//    x_base_cm(약 32cm) 지점이고 차량은 그보다 뒤에 있다.
//    관측 가능한 가장 가까운 점(BEV 맨 아랫줄의 중앙 열)을
//    자차 대신 쓴다. 횡방향으로는 같은 직선 위이므로 영향이
//    없고, 요각 오차만큼만 앞선 값이 된다.
//
// 3. 근거(노란선 또는 흰선 두 개)가 끊겨도 몇 프레임은 직전
//    값을 들고 간다. 점선 공백과 순간 가림 대응.
// ================================================================

KauLaneDetectionNode::CenterFix KauLaneDetectionNode::resolveCenterOffset(
    const LaneDetectionResult & left,
    const LaneDetectionResult & yellow,
    const LaneDetectionResult & right,
    int bev_height)
{
    // prior 나이 먹이기. 아래에서 확실한 근거가 나오면 0 으로
    // 되돌아간다. 포화시켜 두면 오래 굶어도 넘치지 않는다.
    if (center_prior_age_ < 1000000)
    {
        ++center_prior_age_;
    }


    double sx = 0.0;

    double sy = 0.0;

    double x_base = 0.0;


    const bool have_scale =
        bevScale(sx, sy, x_base) &&
        lane_width_px_ > 1e-6 &&
        lane_width_cm_ > 1e-6 &&
        bev_dst_points_.size() == 4;


    // ------------------------------------------------------------
    // 자차 종축 = BEV 중앙 열 (egoAxis 참고)
    // ------------------------------------------------------------

    if (have_scale && !camera_matrix_.empty())
    {
        const double cx = camera_matrix_.at<double>(0, 2);

        if (std::abs(bev_src_center_x_ - cx) > 2.0)
        {
            RCLCPP_WARN_ONCE(
                this->get_logger(),
                "bev_src_center_x(%.1f) 가 주점 cx(%.1f) 와 %.1fpx "
                "다릅니다. BEV 중앙 열이 차량 종축과 어긋나므로 "
                "중앙선 대비 위치 판정에 그만큼 편향이 실립니다.",
                bev_src_center_x_,
                cx,
                std::abs(bev_src_center_x_ - cx)
            );
        }
    }


    const auto observed =
        [](const LaneDetectionResult & l)
        {
            return l.valid &&
                   l.found_count > 0 &&
                   l.track_px.size() >= 2;
        };


    // ------------------------------------------------------------
    // 기준선 = 중앙선
    // ------------------------------------------------------------

    std::vector<cv::Point2d> center_ref;

    int src = 0;


    if (have_scale && observed(yellow))
    {
        center_ref = yellow.track_px;

        src = 1;
    }
    else if (have_scale && observed(left) && observed(right))
    {
        // 노란선이 점선 공백에 걸려 안 보여도, 흰선 두 개가
        // 도로 양 가장자리라면 중앙선은 그 한가운데다.
        //
        // 간격이 도로 폭(2W)에 맞을 때만 인정한다. 좌/우가 같은
        // 선을 물었거나 배경 차선을 물었으면 간격이 어긋난다.
        const double gap =
            geom::medianLateralOffset(
                left.track_px,
                right.track_px
            );

        const double want = 2.0 * lane_width_px_;


        if (
            std::abs(gap) > 0.75 * want &&
            std::abs(gap) < 1.25 * want
        )
        {
            center_ref =
                geom::offsetTrack(
                    left.track_px,
                    0.5 * gap
                );

            src = 2;
        }
    }
    else if (
        have_scale &&
        (observed(left) != observed(right))
    )
    {
        // ------------------------------------------------------
        // 흰선이 하나뿐
        //
        // 이 경우가 코너에서 제일 흔하다. 안쪽 차선이 BEV 옆으로
        // 먼저 빠져나가고 바깥 차선만 남는다.
        //
        // 관측만으로는 남은 그 선이 도로의 왼쪽 가장자리인지
        // 오른쪽 가장자리인지 알 수 없다. 맵 단면이 필요하다.
        // identifyLine 이 자차 종축 대비 실측 횡거리를 단면의
        // 기대 위치와 맞춰 자리를 고른다.
        //
        // 자리가 정해지면 그만큼 되밀어 중앙선을 세운다.
        // ------------------------------------------------------

        const LaneDetectionResult & one =
            observed(left) ? left : right;

        const int k =
            identifyLine(
                one.track_px,
                kau_road::LineType::White,
                centerOffsetPriorCm(),
                sx,
                bev_height
            );


        if (
            k >= 0 &&
            std::abs(lineOffsetCm(k)) > 1e-9
        )
        {
            center_ref =
                geom::offsetTrack(
                    one.track_px,
                    -lineOffsetCm(k) / sx
                );

            src = 3;
        }
    }


    // ------------------------------------------------------------
    // 근거가 없으면 직전 판정을 hold_frames 까지 끌고 간다
    // ------------------------------------------------------------

    if (center_ref.size() < 2)
    {
        if (
            center_fix_.valid &&
            center_fix_miss_ < center_hold_frames_
        )
        {
            ++center_fix_miss_;

            center_fix_.source = 4;

            return center_fix_;
        }


        center_fix_ = CenterFix();

        return center_fix_;
    }


    // ------------------------------------------------------------
    // 부호 있는 횡거리
    // ------------------------------------------------------------

    const cv::Point2d ego_px =
        egoAxis(bev_height).front();

    const double d_cm =
        geom::lateralOffsetAt(center_ref, ego_px) * sx;


    // 도로 폭 밖이면 기준선이 중앙선이 아니다.
    const double road_half_cm =
        roadHalfWidthCm();


    if (std::abs(d_cm) > road_half_cm + 0.5 * lane_width_cm_)
    {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            5000,
            "자차가 중앙선에서 %+.1fcm 로 나옵니다. 도로 반폭 "
            "%.1fcm 를 넘으므로 기준선이 중앙선이 아닙니다. "
            "판정을 보류합니다.",
            d_cm,
            road_half_cm
        );


        if (
            center_fix_.valid &&
            center_fix_miss_ < center_hold_frames_
        )
        {
            ++center_fix_miss_;

            center_fix_.source = 4;

            return center_fix_;
        }


        center_fix_ = CenterFix();

        return center_fix_;
    }


    center_fix_miss_ = 0;


    // ------------------------------------------------------------
    // 결과
    // ------------------------------------------------------------

    CenterFix fix;

    fix.valid = true;

    fix.offset_cm = d_cm;

    fix.source = src;


    // 확실한 근거로 잰 값만 다음 프레임의 기준점이 된다.
    if (src == 1 || src == 2)
    {
        center_prior_offset_cm_ = d_cm;

        center_prior_age_ = 0;
    }


    center_fix_ = fix;

    return center_fix_;
}
