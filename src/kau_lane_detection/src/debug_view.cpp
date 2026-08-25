// ====================================================================
// debug_view.cpp
//
// 디버그 오버레이 렌더링.
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
// 발행한 Bezier 를 BEV 위에 되돌려 그린다.
//
// sampling 은 시각화 전용 (대전제: 연산에 이산 좌표 금지).
// ================================================================

void KauLaneDetectionNode::drawPathOverlay(
    const LanePath & path,
    double camera_yaw_deg,
    cv::Mat & image)
{
    double sx = 0.0;

    double sy = 0.0;

    double x_base = 0.0;


    if (!path.built || !bevScale(sx, sy, x_base))
    {
        return;
    }


    // 통과 = 마젠타, 기각 = 붉은 회색.
    // 기각된 경로도 그려야 무엇이 막혔는지 눈으로 보인다.
    const cv::Scalar curve_color =
        path.valid
            ? cv::Scalar(255, 0, 255)
            : cv::Scalar(60, 60, 200);


    const double h_px = static_cast<double>(image.rows);

    const double cx_bev = 0.5 * image.cols;


    // path.ctrl 은 base_link 기준이다 (buildCenterlinePath 6-b 절
    // 회전 보정 후). 이 BEV 이미지는 "지금 카메라가 보는 방향"
    // 기준이므로, 그리기 전에 6-b 의 역회전으로 되돌린다.
    const double yaw_rad = -camera_yaw_deg * M_PI / 180.0;

    const double cos_yaw = std::cos(yaw_rad);

    const double sin_yaw = std::sin(yaw_rad);


    const auto to_px =
        [&](const kau::bezier::Point2 & p)
        {
            const double x_rel = p.x - pan_pivot_offset_x_cm_;

            const double y_rel = p.y - pan_pivot_offset_y_cm_;

            const double x_cam =
                pan_pivot_offset_x_cm_ +
                x_rel * cos_yaw - y_rel * sin_yaw;

            const double y_cam =
                pan_pivot_offset_y_cm_ +
                x_rel * sin_yaw + y_rel * cos_yaw;

            return cv::Point(
                static_cast<int>(std::lround(
                    cx_bev - (y_cam - path_y_offset_cm_) / sx)),
                static_cast<int>(std::lround(
                    h_px -
                    (x_cam - x_base - path_x_offset_cm_) / sy))
            );
        };


    // 제어 다각형
    for (
        std::size_t i = 1;
        i < path.ctrl.size();
        ++i
    )
    {
        cv::line(
            image,
            to_px(path.ctrl[i - 1]),
            to_px(path.ctrl[i]),
            cv::Scalar(120, 120, 120),
            1
        );
    }


    // 곡선
    const std::vector<kau::bezier::Point2> pts =
        kau::bezier::sample(path.ctrl, 60);


    for (
        std::size_t i = 1;
        i < pts.size();
        ++i
    )
    {
        cv::line(
            image,
            to_px(pts[i - 1]),
            to_px(pts[i]),
            cv::Scalar(255, 0, 255),
            2
        );
    }


    // 제어점
    for (const kau::bezier::Point2 & p : path.ctrl)
    {
        cv::circle(
            image,
            to_px(p),
            4,
            curve_color,
            -1
        );
    }
}
