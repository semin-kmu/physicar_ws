// ====================================================================
// bev_geometry.cpp
//
// BEV 원근변환 기하와 축척. src 사다리꼴, cm/px, CameraInfo.
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
// CameraInfo Callback
// ================================================================

void KauLaneDetectionNode::cameraInfoCallback(
    const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
    // ------------------------------------------------------------
    // 유효성 검사 — 캘리브레이션이 실제로 되어 있는가
    //
    // 실차의 camera_ros(libcamera) 는 캘리브 파일
    // ~/.ros/camera_info/$NAME.yaml 이 없으면 경고 한 줄만 내고
    // 0 으로 채운 intrinsic 을 그대로 발행한다 (그쪽 README 의
    // "Calibration" 절에 명시되어 있다).
    //
    // 그걸 그대로 받으면 어디서도 예외가 나지 않는다:
    //   - camera_calibrated_ 가 서므로 "Waiting for CameraInfo"
    //     경고가 사라져 정상처럼 보인다
    //   - deriveScale 은 fx 가드에 걸려 조용히 false 를 반환하고
    //     축척은 자리표시값에 머문다
    //   - initUndistortRectifyMap 은 특이행렬을 받아 inf/nan 맵을
    //     만들고, remap 결과가 통째로 깨진다
    //   - countVisibleAtZeroPan 의 atan(cx/fx) 는 0 으로 나눈다
    //
    // 즉 "영상만 안 나오는데 로그는 멀쩡한" 상태가 된다. 그래서
    // 여기서 막는다. 이미 받아 둔 정상 intrinsic 이 있으면 그것을
    // 유지한다 (뒤늦게 온 불량 메시지가 덮어쓰지 못하게).
    // ------------------------------------------------------------

    const double k_fx = msg->k[0];

    const double k_fy = msg->k[4];

    const double k_cx = msg->k[2];

    const double k_cy = msg->k[5];


    const bool intrinsic_ok =
        std::isfinite(k_fx) && std::isfinite(k_fy) &&
        std::isfinite(k_cx) && std::isfinite(k_cy) &&
        // 초점거리가 1 px 미만인 카메라는 없다. 0 초기화를 잡는다.
        k_fx > 1.0 && k_fy > 1.0 &&
        msg->width > 0 && msg->height > 0 &&
        // 주점은 영상 안에 있어야 한다.
        k_cx > 0.0 && k_cx < static_cast<double>(msg->width) &&
        k_cy > 0.0 && k_cy < static_cast<double>(msg->height);


    if (!intrinsic_ok)
    {
        RCLCPP_ERROR_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            5000,
            "CameraInfo 의 intrinsic 이 유효하지 않습니다 "
            "(fx=%.3f fy=%.3f cx=%.3f cy=%.3f, 영상 %ux%u). "
            "카메라 캘리브레이션 파일이 없어 드라이버가 0 을 "
            "발행하고 있을 가능성이 큽니다 — "
            "~/.ros/camera_info/ 를 확인하고 camera_calibration "
            "으로 캘리브하십시오. 이 메시지는 무시합니다.",
            k_fx,
            k_fy,
            k_cx,
            k_cy,
            msg->width,
            msg->height
        );

        return;
    }


    // 왜곡 모델 확인.
    //
    // 아래 undistort 는 cv::initUndistortRectifyMap 을 쓴다.
    // 이것은 Brown-Conrady(plumb_bob 계열) 전제다. 광각 렌즈를
    // equidistant/fisheye 로 캘리브했다면 cv::fisheye 쪽을 써야
    // 하고, 그대로 두면 가장자리가 크게 어긋난다.
    if (
        !msg->distortion_model.empty() &&
        msg->distortion_model != "plumb_bob" &&
        msg->distortion_model != "rational_polynomial"
    )
    {
        RCLCPP_WARN_ONCE(
            this->get_logger(),
            "왜곡 모델이 '%s' 입니다. undistort 는 Brown-Conrady "
            "(plumb_bob / rational_polynomial) 전제로 짜여 있어 "
            "이 모델에는 맞지 않습니다. 영상 가장자리가 어긋납니다.",
            msg->distortion_model.c_str()
        );
    }


    // ------------------------------------------------------------
    // Camera Matrix K
    // ------------------------------------------------------------

    camera_matrix_ = cv::Mat(
        3,
        3,
        CV_64F,
        const_cast<double*>(msg->k.data())
    ).clone();


    // ------------------------------------------------------------
    // Distortion Coefficients
    // ------------------------------------------------------------

    distortion_coefficients_ = cv::Mat(
        static_cast<int>(msg->d.size()),
        1,
        CV_64F,
        const_cast<double*>(msg->d.data())
    ).clone();


    camera_calibrated_ = true;


    // ------------------------------------------------------------
    // 축척이 intrinsic 에 의존하므로 유도값을 다시 계산한다
    // ------------------------------------------------------------

    refreshDerivedScale();


    // ------------------------------------------------------------
    // Log
    // ------------------------------------------------------------

    RCLCPP_INFO_ONCE(
        this->get_logger(),
        "CameraInfo received."
    );

    RCLCPP_INFO_ONCE(
        this->get_logger(),
        "Image size: %u x %u",
        msg->width,
        msg->height
    );

    RCLCPP_INFO_ONCE(
        this->get_logger(),
        "Distortion model: %s",
        msg->distortion_model.c_str()
    );


    RCLCPP_INFO_ONCE(
        this->get_logger(),
        "Camera Matrix K:"
    );

    RCLCPP_INFO_ONCE(
        this->get_logger(),
        "[ %.6f  %.6f  %.6f ]",
        camera_matrix_.at<double>(0, 0),
        camera_matrix_.at<double>(0, 1),
        camera_matrix_.at<double>(0, 2)
    );

    RCLCPP_INFO_ONCE(
        this->get_logger(),
        "[ %.6f  %.6f  %.6f ]",
        camera_matrix_.at<double>(1, 0),
        camera_matrix_.at<double>(1, 1),
        camera_matrix_.at<double>(1, 2)
    );

    RCLCPP_INFO_ONCE(
        this->get_logger(),
        "[ %.6f  %.6f  %.6f ]",
        camera_matrix_.at<double>(2, 0),
        camera_matrix_.at<double>(2, 1),
        camera_matrix_.at<double>(2, 2)
    );


    RCLCPP_INFO_ONCE(
        this->get_logger(),
        "Distortion coefficients: %zu values",
        msg->d.size()
    );
}


// ================================================================
// BEV Geometry
//
// src 사다리꼴 -> dst 사각형 변환 행렬을 만든다.
//
// 핵심:
//
//   src의 좌/우 변을 위로 연장했을 때 만나는 점(소실점)이
//   실제 도로 차선의 소실점과 같아야
//   BEV에서 차선이 "평행한 수직선"이 된다.
//
//   그래서 윗변 폭을 직접 주는 대신
//   소실점에서 역산한다.
// ================================================================

void KauLaneDetectionNode::updateBevGeometry()
{
    // ------------------------------------------------------------
    // src 사다리꼴
    // ------------------------------------------------------------

    const float center_x =
        static_cast<float>(
            bev_src_center_x_
        );

    const float half_bottom =
        static_cast<float>(
            bev_src_bottom_width_ * 0.5
        );

    const float top_y =
        static_cast<float>(
            bev_src_top_y_
        );

    const float bottom_y =
        static_cast<float>(
            bev_src_bottom_y_
        );


    // ------------------------------------------------------------
    // 윗변 폭 = 소실점을 지나도록 역산
    //
    //   half_top = half_bottom
    //              * (top_y    - vanishing_y)
    //              / (bottom_y - vanishing_y)
    // ------------------------------------------------------------

    const double depth_bottom =
        bev_src_bottom_y_ - bev_vanishing_y_;

    const double depth_top =
        bev_src_top_y_ - bev_vanishing_y_;


    if (
        depth_bottom <= 1.0 ||
        depth_top <= 1.0
    )
    {
        RCLCPP_ERROR(
            this->get_logger(),
            "Invalid BEV geometry: "
            "bev_vanishing_y (%.1f) must be above "
            "bev_src_top_y (%.1f) and "
            "bev_src_bottom_y (%.1f). "
            "Keeping previous transform.",
            bev_vanishing_y_,
            bev_src_top_y_,
            bev_src_bottom_y_
        );

        return;
    }


    const float half_top =
        static_cast<float>(
            half_bottom *
            depth_top /
            depth_bottom
        );


    bev_src_top_width_ =
        2.0 *
        static_cast<double>(
            half_top
        );


    bev_src_points_ =
    {
        cv::Point2f(
            center_x - half_top,
            top_y
        ),                                  // top-left

        cv::Point2f(
            center_x + half_top,
            top_y
        ),                                  // top-right

        cv::Point2f(
            center_x + half_bottom,
            bottom_y
        ),                                  // bottom-right

        cv::Point2f(
            center_x - half_bottom,
            bottom_y
        )                                   // bottom-left
    };


    // ------------------------------------------------------------
    // dst 사각형
    //
    // 좌우에 여백을 주어 차선이 화면 끝에 붙지 않고
    // 가운데로 모이게 한다.
    // ------------------------------------------------------------

    const float out_w =
        static_cast<float>(
            bev_out_width_
        );

    const float out_h =
        static_cast<float>(
            bev_out_height_
        );

    const float margin =
        out_w *
        static_cast<float>(
            std::clamp(
                bev_dst_margin_ratio_,
                0.0,
                0.45
            )
        );

    const float x_left = margin;

    const float x_right = out_w - margin;


    bev_dst_points_ =
    {
        cv::Point2f(
            x_left,
            0.0f
        ),

        cv::Point2f(
            x_right,
            0.0f
        ),

        cv::Point2f(
            x_right,
            out_h
        ),

        cv::Point2f(
            x_left,
            out_h
        )
    };


    perspective_matrix_ =
        cv::getPerspectiveTransform(
            bev_src_points_,
            bev_dst_points_
        );

    // 사다리꼴이 바뀌었으므로 유효영역도 다시 만들어야 한다.
    bev_valid_dirty_ = true;


    RCLCPP_INFO(
        this->get_logger(),
        "BEV src: top(%.1f ~ %.1f @ y=%.1f, width %.1f) "
        "bottom(%.1f ~ %.1f @ y=%.1f, width %.1f)",
        center_x - half_top,
        center_x + half_top,
        top_y,
        bev_src_top_width_,
        center_x - half_bottom,
        center_x + half_bottom,
        bottom_y,
        bev_src_bottom_width_
    );

    RCLCPP_INFO(
        this->get_logger(),
        "BEV vanishing y = %.1f "
        "-> look-ahead ratio %.1fx",
        bev_vanishing_y_,
        depth_bottom / depth_top
    );

    RCLCPP_INFO(
        this->get_logger(),
        "BEV dst: x %.1f ~ %.1f "
        "(margin ratio %.2f) size %d x %d",
        x_left,
        x_right,
        bev_dst_margin_ratio_,
        bev_out_width_,
        bev_out_height_
    );
}


// ================================================================
// Parameter Refresh
//
// ros2 param set 으로 값이 바뀌면 변환 행렬을 다시 만든다.
// ================================================================

void KauLaneDetectionNode::refreshBevParameters()
{
    const double center_x =
        this->get_parameter(
            "bev_src_center_x"
        ).as_double();

    const double top_y =
        this->get_parameter(
            "bev_src_top_y"
        ).as_double();

    const double bottom_y =
        this->get_parameter(
            "bev_src_bottom_y"
        ).as_double();

    const double vanishing_y =
        this->get_parameter(
            "bev_vanishing_y"
        ).as_double();

    const double bottom_width =
        this->get_parameter(
            "bev_src_bottom_width"
        ).as_double();

    const double margin_ratio =
        this->get_parameter(
            "bev_dst_margin_ratio"
        ).as_double();

    // 축척을 정하는 물리 파라미터. 바뀌면 lane_width_px 도 따라간다.
    const double cam_height =
        this->get_parameter(
            "camera_height_cm"
        ).as_double();

    const double lane_width =
        this->get_parameter(
            "lane_width_cm"
        ).as_double();

    const int out_width =
        static_cast<int>(
            this->get_parameter(
                "bev_out_width"
            ).as_int()
        );

    const int out_height =
        static_cast<int>(
            this->get_parameter(
                "bev_out_height"
            ).as_int()
        );


    window_margin_ =
        static_cast<int>(
            this->get_parameter(
                "window_margin"
            ).as_int()
        );

    // 창 면적과 스텝 길이. BEV 기하에는 안 걸리므로 changed 검사
    // 밖에서 매 프레임 그대로 반영한다.
    min_pixels_ =
        static_cast<int>(
            this->get_parameter(
                "min_pixels"
            ).as_int()
        );

    track_step_px_ =
        this->get_parameter(
            "track_step_px"
        ).as_double();

    // /lane/center 근거 선택. 실차에서 A/B 하려고 런타임 재독한다.
    center_source_ =
        this->get_parameter(
            "center_source"
        ).as_string();

    publish_roi_overlay_ =
        this->get_parameter(
            "publish_roi_overlay"
        ).as_bool();


    const bool changed =
        center_x      != bev_src_center_x_    ||
        top_y         != bev_src_top_y_       ||
        bottom_y      != bev_src_bottom_y_    ||
        vanishing_y   != bev_vanishing_y_     ||
        bottom_width  != bev_src_bottom_width_ ||
        margin_ratio  != bev_dst_margin_ratio_ ||
        out_width     != bev_out_width_       ||
        out_height    != bev_out_height_      ||
        cam_height    != camera_height_cm_    ||
        lane_width    != lane_width_cm_;


    if (!changed)
    {
        return;
    }


    bev_src_center_x_ = center_x;

    bev_src_top_y_ = top_y;

    bev_src_bottom_y_ = bottom_y;

    bev_vanishing_y_ = vanishing_y;

    bev_src_bottom_width_ = bottom_width;

    bev_dst_margin_ratio_ = margin_ratio;

    bev_out_width_ = out_width;

    bev_out_height_ = out_height;

    camera_height_cm_ = cam_height;

    lane_width_cm_ = lane_width;


    updateBevGeometry();

    // 사다리꼴이나 축척이 바뀌었으므로 lane_width_px 를 다시 유도
    refreshDerivedScale();
}


// ================================================================
// BEV 축척 [cm/px]
//
// 가로 축척은 실 차로 폭으로 정한다.
//
//   cm_per_px_x = lane_width_cm / lane_width_px
//
// 세로 축척은 따로 잴 필요가 없다. 카메라 intrinsic 과
// 소실점만으로 세로/가로 비가 결정되고, 카메라 높이 h 는
// 분자 분모에서 약분되기 때문이다.
//
// 유도 (카메라 좌표 x 우 / y 아래 / z 전방, roll 0):
//
//   a    = (v - cy) / fy                 행 v 의 정규화 y
//   a_h  = (vanishing_y - cy) / fy       지평선
//
//   지평선 광선은 지면과 평행하므로 지면 법선은
//   n = (0, -1, a_h) / sqrt(1 + a_h^2).
//
//   광선 P = t (b, a, 1) 이 지면(n·P = -h)과 만나는 점:
//
//     t(a)/h = sqrt(1 + a_h^2) / (a - a_h)        광축 depth
//     D(a)/h = (1 + a a_h)   / (a - a_h)          지면 전방거리
//
//   src 사다리꼴의 좌/우 변은 소실점을 지나도록 만들어졌으므로
//   (updateBevGeometry 참고) 지면에서는 서로 평행하다.
//   즉 src 사다리꼴의 지면 대응 도형은 정확히 직사각형이고,
//   그 폭 W 와 길이 L 은
//
//     W/h = 2 * (bottom_width/2) * t(a_bot)/h / fx
//     L/h = D(a_top)/h - D(a_bot)/h
//
//   dst 사각형이 W 를 dst_w px, L 을 dst_h px 로 펴므로
//
//     cm_per_px_y = cm_per_px_x * (L/W) * (dst_w / dst_h)
//
// 반환하는 x_base_cm 은 BEV 하단 모서리의 지면 전방거리.
// camera_height_cm 이 0 이면 (미지정) 0 을 준다.
// ================================================================

bool KauLaneDetectionNode::bevScale(
    double & sx,
    double & sy,
    double & x_base_cm) const
{
    if (
        !camera_calibrated_ ||
        camera_matrix_.empty() ||
        bev_dst_points_.size() != 4 ||
        camera_height_cm_ <= 1e-6 ||
        lane_width_cm_ <= 1e-6
    )
    {
        return false;
    }


    const double fx = camera_matrix_.at<double>(0, 0);

    const double fy = camera_matrix_.at<double>(1, 1);

    const double cy = camera_matrix_.at<double>(1, 2);


    if (fx <= 1e-6 || fy <= 1e-6)
    {
        return false;
    }


    const double a_h =
        (bev_vanishing_y_ - cy) / fy;

    const double a_bot =
        (bev_src_bottom_y_ - cy) / fy;

    const double a_top =
        (bev_src_top_y_ - cy) / fy;


    // 두 행 모두 지평선 아래(지면 위)여야 한다
    if (
        a_bot - a_h < 1e-6 ||
        a_top - a_h < 1e-6
    )
    {
        return false;
    }


    const auto ground_d =
        [&](double a)
        {
            return (1.0 + a * a_h) / (a - a_h);
        };

    const auto ray_t =
        [&](double a)
        {
            return std::sqrt(1.0 + a_h * a_h) / (a - a_h);
        };


    // h = 1 로 둔 무차원 값 (비에서 약분된다)
    const double w_ground =
        bev_src_bottom_width_ * ray_t(a_bot) / fx;

    const double l_ground =
        ground_d(a_top) - ground_d(a_bot);


    if (w_ground <= 1e-9 || l_ground <= 1e-9)
    {
        return false;
    }


    const double dst_w =
        static_cast<double>(
            bev_dst_points_[1].x - bev_dst_points_[0].x);

    const double dst_h =
        static_cast<double>(
            bev_dst_points_[2].y - bev_dst_points_[1].y);


    if (dst_w <= 1e-6 || dst_h <= 1e-6)
    {
        return false;
    }


    // w_ground / l_ground 는 h = 1 로 둔 무차원 값이므로
    // 실 카메라 높이를 곱하면 cm 가 된다.
    //
    //   sx = h * w_ground / dst_w      하단 모서리 지면 폭 / dst 폭
    //   sy = h * l_ground / dst_h      사다리꼴 지면 깊이 / dst 높이

    sx = camera_height_cm_ * w_ground / dst_w;

    sy = camera_height_cm_ * l_ground / dst_h;


    x_base_cm = camera_height_cm_ * ground_d(a_bot);


    // ------------------------------------------------------------
    // 로그
    //
    // 축척은 camera_height_cm 하나에서 나오므로 역산할 게 없다.
    // 대신 lane_width_cm 이 몇 px 에 해당하는지를 찍는다.
    // 이 값이 실제 BEV 상의 노란선-흰선 간격과 다르면
    // camera_height_cm 또는 lane_width_cm 이 틀린 것이다.
    // (imageCallback 이 매 프레임 실측과 대조해 경고한다)
    // ------------------------------------------------------------

    RCLCPP_INFO_ONCE(
        this->get_logger(),
        "BEV scale: 가로 %.4f cm/px, 세로 %.4f cm/px "
        "(카메라 높이 %.2f cm + intrinsic 유도, 비등방 %.2f배) "
        "-> 범위 %.0f x %.0f cm",
        sx,
        sy,
        camera_height_cm_,
        sx / sy,
        sx * dst_w,
        sy * dst_h
    );

    RCLCPP_INFO_ONCE(
        this->get_logger(),
        "BEV ground: 카메라앞 %.1f ~ %.1f cm "
        "(경로 X 원점 = %.1f cm). 차로폭 %.1f cm = %.1f BEV px",
        x_base_cm,
        camera_height_cm_ * ground_d(a_top),
        x_base_cm + path_x_offset_cm_,
        lane_width_cm_,
        lane_width_cm_ / sx
    );


    return true;
}


// ================================================================
// 유도 축척 갱신
//
// lane_width_px_ 는 설정값이 아니라 물리량에서 나온다.
//
//   lane_width_px = lane_width_cm / sx
//
// sx 는 카메라 높이와 intrinsic, BEV 사다리꼴에서 유도되므로
// CameraInfo 가 오거나 BEV 기하가 바뀌면 다시 계산해야 한다.
//
// 유도에 실패하면 (CameraInfo 미수신 등) 이전 값을 유지한다.
// imageCallback 이 CameraInfo 없이는 조기 반환하므로
// 자리표시 값이 실제 검출에 쓰이지는 않는다.
// ================================================================

void KauLaneDetectionNode::refreshDerivedScale()
{
    double sx = 0.0;

    double sy = 0.0;

    double x_base = 0.0;


    if (!bevScale(sx, sy, x_base))
    {
        return;
    }


    const double derived =
        lane_width_cm_ / sx;


    if (
        derived < 4.0 ||
        derived > 4.0 * static_cast<double>(bev_out_width_)
    )
    {
        RCLCPP_ERROR(
            this->get_logger(),
            "유도된 차로 폭 %.1f px 가 비정상입니다. "
            "camera_height_cm(%.2f) / lane_width_cm(%.1f) 확인. "
            "이전 값 %.1f px 유지.",
            derived,
            camera_height_cm_,
            lane_width_cm_,
            lane_width_px_
        );

        return;
    }


    if (std::abs(derived - lane_width_px_) > 0.05)
    {
        RCLCPP_INFO(
            this->get_logger(),
            "차로 폭 유도: %.1f cm / %.4f cm/px = %.1f BEV px "
            "(이전 %.1f px)",
            lane_width_cm_,
            sx,
            derived,
            lane_width_px_
        );
    }


    lane_width_px_ = derived;
}


// ================================================================
// BEV 유효영역 마스크
//
// 원본과 같은 크기의 흰 이미지를 같은 변환으로 warp 하면, 원본이
// 닿는 곳만 흰색으로 남는다. INTER_NEAREST 라 경계가 0/255 로
// 딱 떨어진다.
//
// 그 뒤 조금 깎는다 — 본 영상은 INTER_CUBIC 으로 warp 하므로
// 경계에서 검정과 지면 색이 2~3 px 섞이고, 그 중간색이 임계를
// 통과해 버릴 수 있다. 깎아내면 그 띠가 통째로 빠진다.
// ================================================================

void KauLaneDetectionNode::refreshBevValidMask(const cv::Size & src_size)
{
    if (
        !bev_valid_dirty_ &&
        bev_valid_src_size_ == src_size &&
        bev_valid_mask_.rows == bev_out_height_ &&
        bev_valid_mask_.cols == bev_out_width_
    )
    {
        return;
    }


    if (src_size.width <= 0 || src_size.height <= 0)
    {
        return;
    }


    const cv::Mat ones(
        src_size,
        CV_8UC1,
        cv::Scalar(255)
    );

    cv::warpPerspective(
        ones,
        bev_valid_mask_,
        perspective_matrix_,
        cv::Size(
            bev_out_width_,
            bev_out_height_
        ),
        cv::INTER_NEAREST
    );


    static const cv::Mat kernel =
        cv::getStructuringElement(
            cv::MORPH_RECT,
            cv::Size(3, 3)
        );

    cv::erode(
        bev_valid_mask_,
        bev_valid_mask_,
        kernel,
        cv::Point(-1, -1),
        2
    );


    bev_valid_src_size_ = src_size;

    bev_valid_dirty_ = false;


    const double valid_ratio =
        static_cast<double>(cv::countNonZero(bev_valid_mask_)) /
        static_cast<double>(bev_valid_mask_.total());

    RCLCPP_INFO(
        this->get_logger(),
        "BEV 유효영역 %.1f%% (나머지는 원본이 닿지 않는 곳이라 "
        "검출에서 뺀다)",
        100.0 * valid_ratio
    );
}
