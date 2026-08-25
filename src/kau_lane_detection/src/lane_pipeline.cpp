// ====================================================================
// lane_pipeline.cpp
//
// 프레임 파이프라인. imageCallback 이 단계들을 순서대로 부른다.
//
// 설계 근거와 실측표는 CLAUDE.md 참고.
// ====================================================================

#include "kau_lane_detection/kau_lane_detection_node.hpp"

#include <sensor_msgs/image_encodings.hpp>

#include <cv_bridge/cv_bridge.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <functional>

namespace geom = kau_lane::geom;


// ================================================================
// Image Callback
// ================================================================


// ================================================================
// 1-2. ROS Image -> undistort. CameraInfo 없으면 프레임을 버린다.
// ================================================================

bool KauLaneDetectionNode::stagePrepareFrame(
    const sensor_msgs::msg::Image::SharedPtr & msg,
    FrameContext & ctx)
{
    cv::Mat & undistorted_frame = ctx.undistorted;

        // 1. ROS Image -> OpenCV
        // ========================================================

        cv::Mat frame =
            cv_bridge::toCvCopy(
                msg,
                sensor_msgs::image_encodings::BGR8
            )->image;


        if (frame.empty())
        {
            return false;
        }


        // ========================================================
        // CameraInfo 대기
        // ========================================================

        if (!camera_calibrated_)
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "Waiting for CameraInfo..."
            );

            return false;
        }


        // ========================================================
        // 2. Camera Undistortion
        // ========================================================

        // cv::undistort() 는 부를 때마다 맵을 새로 만든다.
        // intrinsic 은 고정이므로 맵은 한 번만 만들고 remap 만 돈다.
        if (
            undistort_map1_.empty() ||
            undistort_map_size_ != frame.size()
        )
        {
            cv::initUndistortRectifyMap(
                camera_matrix_,
                distortion_coefficients_,
                cv::Mat(),
                camera_matrix_,
                frame.size(),
                CV_16SC2,
                undistort_map1_,
                undistort_map2_
            );

            undistort_map_size_ = frame.size();

            RCLCPP_INFO(
                this->get_logger(),
                "undistort 맵 생성 (%dx%d). 이후 프레임은 remap 만 돈다.",
                frame.cols,
                frame.rows
            );
        }





        cv::remap(
            frame,
            undistorted_frame,
            undistort_map1_,
            undistort_map2_,
            cv::INTER_LINEAR
        );


        // --------------------------------------------------------
        // Publish Undistorted Image
        // --------------------------------------------------------

        // 구독자가 없으면 만들지 않는다.
        //
        // sensor_msgs/Image 하나를 만드는 데 480x360 BGR8 기준
        // 518 KB 복사 + 직렬화 + DDS 전송이 든다. 이 노드는
        // 그런 토픽을 6개 발행하므로 프레임당 약 2.4 MB,
        // 14 Hz 면 초당 34 MB 다. 아무도 안 보고 있을 때 그
        // 전부가 순수 낭비였다.
        if (undistorted_image_publisher_->get_subscription_count() > 0)
        {
        auto undistorted_msg =
            cv_bridge::CvImage(
                msg->header,
                sensor_msgs::image_encodings::BGR8,
                undistorted_frame
            ).toImageMsg();


        undistorted_image_publisher_->publish(
            *undistorted_msg
        );
        }


        // ========================================================

    return true;
}



// ================================================================
// 3. BEV 원근변환. 파라미터 재독도 여기서 한다.
// ================================================================

void KauLaneDetectionNode::stageBuildBev(
    const sensor_msgs::msg::Image::SharedPtr & msg,
    FrameContext & ctx)
{
    cv::Mat & bev_frame = ctx.bev;
    cv::Mat & undistorted_frame = ctx.undistorted;

        // 3. BEV
        //
        // src 사다리꼴 / dst 사각형은 파라미터로 관리
        // ========================================================

        refreshBevParameters();

        // camera_tilt_deg 도 같은 자리에서 다시 읽는다 (ros2 param set).
        refreshCameraTilt();


        // --------------------------------------------------------
        // src ROI Overlay
        //
        // 원본 위에 사다리꼴을 그려서
        // 실제 차선과 좌/우 변이 나란한지 눈으로 확인한다.
        // --------------------------------------------------------

        if (
            publish_roi_overlay_ &&
            roi_overlay_publisher_->get_subscription_count() > 0
        )
        {
            cv::Mat roi_overlay =
                undistorted_frame.clone();


            std::vector<cv::Point> roi_polygon;


            for (const auto & point : bev_src_points_)
            {
                roi_polygon.push_back(
                    cv::Point(
                        static_cast<int>(point.x),
                        static_cast<int>(point.y)
                    )
                );
            }


            cv::polylines(
                roi_overlay,
                roi_polygon,
                true,
                cv::Scalar(
                    0,
                    255,
                    0
                ),
                2
            );


            auto roi_msg =
                cv_bridge::CvImage(
                    msg->header,
                    sensor_msgs::image_encodings::BGR8,
                    roi_overlay
                ).toImageMsg();


            roi_overlay_publisher_->publish(
                *roi_msg
            );
        }





        // INTER_CUBIC.
        //
        // src 사다리꼴은 원본의 아주 작은 조각이다.
        //   위쪽 변 84.4px -> dst 240px  (2.84배 확대)
        //   세로   75px    -> dst 360px  (4.80배 확대)
        // 즉 원거리 1픽셀이 BEV 에서 13.6픽셀 면적이 된다.
        //
        // 없는 정보가 생기지는 않지만, 보간이 부드러워지면
        // 확대된 마킹 가장자리의 색이 덜 뭉개져 HLS 임계를
        // 통과하는 픽셀이 늘어난다.
        //
        // 실패 77장 실측:
        //   LINEAR : 노랑 중앙 880px  노란선 추적 스텝합 420
        //   CUBIC  : 노랑 중앙1045px  노란선 추적 스텝합 469
        cv::warpPerspective(
            undistorted_frame,
            bev_frame,
            perspective_matrix_,
            cv::Size(
                bev_out_width_,
                bev_out_height_
            ),
            cv::INTER_CUBIC
        );


        // --------------------------------------------------------
        // Publish BEV
        // --------------------------------------------------------

        if (bev_image_publisher_->get_subscription_count() > 0)
        {
        auto bev_msg =
            cv_bridge::CvImage(
                msg->header,
                sensor_msgs::image_encodings::BGR8,
                bev_frame
            ).toImageMsg();


        bev_image_publisher_->publish(
            *bev_msg
        );
        }


        // ========================================================
}



// ================================================================
// 4-7. HLS 변환과 노랑/흰 마스크, 형태학 정리.
// ================================================================

void KauLaneDetectionNode::stageBuildMasks(
    FrameContext & ctx)
{
    cv::Mat & bev_frame = ctx.bev;
    cv::Mat & white_mask = ctx.white_mask;
    cv::Mat & yellow_mask = ctx.yellow_mask;

        // 4. HLS 변환
        // ========================================================

        cv::Mat hls;


        cv::cvtColor(
            bev_frame,
            hls,
            cv::COLOR_BGR2HLS
        );


        // ========================================================
        // 5. Yellow Mask
        // ========================================================




        cv::inRange(
            hls,
            geom::toScalar(yellow_hls_lo_),
            geom::toScalar(yellow_hls_hi_),
            yellow_mask
        );


        // ========================================================
        // 6. White Mask
        // ========================================================




        cv::inRange(
            hls,
            geom::toScalar(white_hls_lo_),
            geom::toScalar(white_hls_hi_),
            white_mask
        );


        // ========================================================
        // 7. 작은 Noise 제거
        // ========================================================

        // 3x3 고정이라 매 프레임 만들 이유가 없다.
        static const cv::Mat kernel =
            cv::getStructuringElement(
                cv::MORPH_RECT,
                cv::Size(
                    3,
                    3
                )
            );


        // --------------------------------------------------------
        // 슬라이딩 윈도우가 쓰는 건 yellow_mask / white_mask 이므로
        // 정리도 그 두 마스크에 직접 건다.
        //
        // OPEN  : 점 노이즈 제거
        // CLOSE : 점선 차선의 작은 끊김 메우기 (곡선 끊김 완화)
        // dilate: 살짝 두껍게 -> 창별 픽셀 수 안정
        // --------------------------------------------------------

        for (
            cv::Mat * mask_ptr :
            { &yellow_mask, &white_mask }
        )
        {
            cv::morphologyEx(
                *mask_ptr,
                *mask_ptr,
                cv::MORPH_OPEN,
                kernel
            );

            cv::morphologyEx(
                *mask_ptr,
                *mask_ptr,
                cv::MORPH_CLOSE,
                kernel
            );

            cv::dilate(
                *mask_ptr,
                *mask_ptr,
                kernel
            );
        }


        // ========================================================
        // 8-a. 원본이 닿지 않는 화소를 뺀다
        //
        // dilate 뒤에 거는 것이 중요하다 — dilate 는 경계 쪽으로
        // 마스크를 넓히므로 먼저 걸면 다시 새어 나온다.
        // ========================================================

        refreshBevValidMask(ctx.undistorted.size());

        if (
            !bev_valid_mask_.empty() &&
            bev_valid_mask_.size() == yellow_mask.size()
        )
        {
            cv::bitwise_and(yellow_mask, bev_valid_mask_, yellow_mask);

            cv::bitwise_and(white_mask, bev_valid_mask_, white_mask);
        }
}



// ================================================================
// 8. 디버그 캔버스. 구독자가 없으면 만들지 않는다.
// ================================================================

void KauLaneDetectionNode::stagePrepareDebugCanvas(
    FrameContext & ctx)
{

        // 8. Debug Image
        // ========================================================

        // 구독자가 없으면 만들지도 그리지도 않는다.
        //
        // 발행만 막아 놨더니 정작 비싼 쪽이 그대로 남아 있었다:
        // 480x360 BGR8 clone(518 KB) + 창 테두리 cv::line 약 180회
        // (창 45개 x 4변) + 추적선 폴리라인 + putText 4~5줄 +
        // drawPathOverlay 를 아무도 안 볼 때도 매 프레임 돌렸다.
        // putText 는 글리프를 매번 래스터라이즈하므로 특히 비싸다.
        //
        // 빈 Mat 을 sentinel 로 쓴다 — 아래 그리기 지점과
        // detectLaneSlidingWindow / drawPathOverlay 가 empty() 를 보고
        // 건너뛰므로 시그니처를 바꿀 필요가 없다.
        ctx.want_debug =
            debug_image_publisher_->get_subscription_count() > 0;

        ctx.debug_image =
            ctx.want_debug ? ctx.bev.clone() : cv::Mat();


        // ========================================================
}



// ================================================================
// 12-16. 히스토그램 시작점 + 세 차선 슬라이딩 윈도우.
// ================================================================

void KauLaneDetectionNode::stageDetectLanes(
    FrameContext & ctx)
{
    LaneDetectionResult & left_lane = ctx.left;
    LaneDetectionResult & right_lane = ctx.right;
    LaneDetectionResult & yellow_lane = ctx.yellow;
    cv::Mat & debug_image = ctx.debug_image;
    cv::Mat & white_mask = ctx.white_mask;
    cv::Mat & yellow_mask = ctx.yellow_mask;
    double & yellow_band = ctx.yellow_band;
    int & left_base = ctx.left_base;
    int & right_base = ctx.right_base;
    int & yellow_base = ctx.yellow_base;

        // 12. Histogram Base Detection
        //
        // 노란 중앙선을 앵커로 삼는다.
        //
        // 흰선을 화면 반쪽 전체에서 찾으면, 곡선에서 한쪽
        // 흰선이 프레임 밖으로 나갔을 때 멀리 있는 노이즈를
        // 차선으로 잡는다. 노랑 기준 [0.4W ~ 1.6W] 밴드
        // 안에서만 찾는다.
        // ========================================================

        // 좁은 밴드가 점선 공백에 걸려 넓힌 경우 band > 기본값이 된다.
        // 그때의 base 는 곡선에서 부정확하므로 상태로 내보낸다.


        ctx.yellow_base =
            findHistogramPeak(
                yellow_mask,
                0,
                yellow_mask.cols,
                &yellow_band
            );







        if (yellow_base >= 0)
        {
            left_base =
                findHistogramPeak(
                    white_mask,
                    static_cast<int>(
                        yellow_base - 1.6 * lane_width_px_
                    ),
                    static_cast<int>(
                        yellow_base - 0.4 * lane_width_px_
                    )
                );

            right_base =
                findHistogramPeak(
                    white_mask,
                    static_cast<int>(
                        yellow_base + 0.4 * lane_width_px_
                    ),
                    static_cast<int>(
                        yellow_base + 1.6 * lane_width_px_
                    )
                );
        }
        else
        {
            // 노랑 소실 -> 반쪽 전체 탐색으로 폴백
            left_base =
                findHistogramPeak(
                    white_mask,
                    0,
                    white_mask.cols / 2
                );

            right_base =
                findHistogramPeak(
                    white_mask,
                    white_mask.cols / 2,
                    white_mask.cols
                );
        }


        // ========================================================
        // 13. Lane Detection Result
        // ========================================================








        // ========================================================
        // 14. Left White Sliding Window
        // ========================================================

        if (left_base >= 0)
        {
            left_lane =
                detectLaneSlidingWindow(
                    white_mask,
                    left_base,
                    debug_image,
                    cv::Scalar(
                        255,
                        0,
                        0
                    )
                );
        }


        // ========================================================
        // 15. Center Yellow Sliding Window
        // ========================================================

        if (yellow_base >= 0)
        {
            yellow_lane =
                detectLaneSlidingWindow(
                    yellow_mask,
                    yellow_base,
                    debug_image,
                    cv::Scalar(
                        0,
                        255,
                        255
                    )
                );
        }


        // ========================================================
        // 16. Right White Sliding Window
        // ========================================================

        if (right_base >= 0)
        {
            right_lane =
                detectLaneSlidingWindow(
                    white_mask,
                    right_base,
                    debug_image,
                    cv::Scalar(
                        0,
                        0,
                        255
                    )
                );
        }


        // ========================================================
}



// ================================================================
// 16-a. 흰선 좌/우 검증과 회랑 절단.
// ================================================================

void KauLaneDetectionNode::stageValidateSides(
    FrameContext & ctx)
{
    LaneDetectionResult & left_lane = ctx.left;
    LaneDetectionResult & right_lane = ctx.right;
    LaneDetectionResult & yellow_lane = ctx.yellow;
    cv::Mat & bev_frame = ctx.bev;

        // 16-a. 흰선 쪽 검증
        //
        // 흰선 base 는 x 구간 히스토그램으로 찾는다. 차선이
        // 세로로 서 있을 때만 "x 로 왼쪽" = "차로의 왼쪽" 이다.
        // 90도 코너에서 차선이 가로로 누우면 좌/우 흰선이
        // 노란선의 위/아래에 쌓이므로, x 구간이 반대쪽 선이나
        // 같은 선을 물어 온다.
        //
        // 추적이 끝난 뒤 노란선 기준으로 어느 쪽에 있는지 재보고
        // 반대쪽이면 버린다. 버린 자리는 아래 16-c 복원이 채운다.
        // ========================================================

        if (
            yellow_lane.valid &&
            yellow_lane.track_px.size() >= 2
        )
        {
            const double side_min =
                lane_side_min_ratio_ * lane_width_px_;


            // 검증을 통과한 흰선의 실측 간격.
            // 유도된 lane_width_px_ 와 대조해 축척을 자가 점검한다.
            std::vector<double> measured_width_px;


            const auto usable =
                [](const LaneDetectionResult & l)
                {
                    return l.valid &&
                           l.found_count > 0 &&
                           l.track_px.size() >= 2;
                };


            // ----------------------------------------------------
            // (1) 부호로 자리 재배정
            //
            // x 구간 히스토그램이 좌우를 바꿔 무는 일이 있다.
            // 예전에는 그냥 버렸는데, 그건 멀쩡한 검출을 이름만
            // 잘못 붙인 것이라 버릴 이유가 없다. 노란선 기준
            // 부호가 가리키는 자리로 옮긴다.
            //
            // 주행 로그 18.5분 실측 (경고 16건의 내역):
            //   완전 뒤바뀜 4건(25%)  <- 여기서 회수
            //   같은 선     5건(31%)  <- 아래 (2) 에서 정리
            //   중간 표류   7건(44%)  <- 아래 (3) 회랑에서 절단
            // ----------------------------------------------------

            {
                struct Seen
                {
                    LaneDetectionResult lane;

                    double off;
                };


                std::vector<Seen> seen;


                for (LaneDetectionResult * l : { &left_lane, &right_lane })
                {
                    if (usable(*l))
                    {
                        seen.push_back(
                            Seen{
                                *l,
                                geom::medianLateralOffset(
                                    yellow_lane.track_px,
                                    l->track_px
                                )
                            }
                        );
                    }
                }


                left_lane = LaneDetectionResult();

                right_lane = LaneDetectionResult();


                // 같은 쪽을 문 것끼리는 근거가 많은 하나만 남는다
                for (const Seen & s : seen)
                {
                    LaneDetectionResult * slot =
                        (s.off <= -side_min) ? &left_lane :
                        (s.off >=  side_min) ? &right_lane : nullptr;


                    if (slot == nullptr)
                    {
                        RCLCPP_WARN_THROTTLE(
                            this->get_logger(),
                            *this->get_clock(),
                            2000,
                            "흰선이 노란선과 겹칩니다 "
                            "(대비 %+.0fpx, 기대 %s%.0fpx). 버립니다.",
                            s.off,
                            "+-",
                            side_min
                        );

                        continue;
                    }


                    if (slot->found_count >= s.lane.found_count &&
                        slot->track_px.size() >= 2)
                    {
                        continue;
                    }


                    *slot = s.lane;
                }
            }


            // ----------------------------------------------------
            // (3) 회랑 이탈 지점에서 절단
            //
            // 차로 폭은 고정이다. 노란선에서 그만큼 떨어져 있지
            // 않은 점은 우리 차로의 흰선이 아니라 체커보드 연석
            // 이거나 급커브에서 보이는 다른 구간 차선이다.
            //
            // 통째로 버리지 않고 갈아탄 지점에서 자른다.
            // ----------------------------------------------------

            const double lo_px = lane_corridor_lo_ * lane_width_px_;

            const double hi_px = lane_corridor_hi_ * lane_width_px_;


            const auto clip_side =
                [&](
                    LaneDetectionResult & lane,
                    double want_sign,
                    const char * name)
                {
                    if (!usable(lane))
                    {
                        return;
                    }


                    const std::size_t before =
                        lane.track_px.size();


                    lane.track_px =
                        geom::truncateAtCorridor(
                            lane.track_px,
                            yellow_lane.track_px,
                            want_sign,
                            lo_px,
                            hi_px
                        );


                    if (lane.track_px.size() == before)
                    {
                        measured_width_px.push_back(
                            std::abs(
                                geom::medianLateralOffset(
                                    yellow_lane.track_px,
                                    lane.track_px
                                )
                            )
                        );

                        return;
                    }


                    RCLCPP_WARN_THROTTLE(
                        this->get_logger(),
                        *this->get_clock(),
                        2000,
                        "%s 흰선이 %zu번째 스텝에서 차로를 벗어났습니다 "
                        "(회랑 %.0f~%.0fpx). 그 지점에서 자릅니다.",
                        name,
                        lane.track_px.size(),
                        lo_px,
                        hi_px
                    );


                    lane.found_count =
                        static_cast<int>(lane.track_px.size());


                    if (lane.track_px.size() < MIN_VALID_WINDOWS)
                    {
                        lane = LaneDetectionResult();
                    }
                };


            clip_side(left_lane,  -1.0, "왼쪽");

            clip_side(right_lane, +1.0, "오른쪽");


            // ----------------------------------------------------
            // 축척 자가 점검
            //
            // lane_width_px_ 는 camera_height_cm 과 intrinsic 에서
            // 유도한 값이다. 실측 간격이 여기서 크게 벗어나면
            // 카메라 높이나 차로 폭 설정이 틀린 것이고, 발행되는
            // 모든 좌표가 같은 비율로 틀어진다.
            //
            // 곡선 구간에서는 재지 않는다.
            //
            // 실측은 흰선의 각 점을 노란선 "폴리라인" 에 투영해
            // 잰다. 폴리라인은 호를 현으로 근사한 것이라 곡선
            // 바깥쪽에서는 실제 호까지의 거리보다 멀게 나온다.
            // 스텝 25px 짜리 현이면 곡률반경 100cm 구간에서
            // 몇 % 가 부풀려지고, 그것이 lane_width_cm 을 35.0
            // 으로 잘못 잡게 만든 원인이었다 (실측 중앙값 108.3
            // px, 직선에서 다시 재면 97.5 px).
            //
            // 현/호 비가 0.98 이상, 즉 거의 직선일 때만 신뢰한다.
            // ----------------------------------------------------

            double yellow_straightness = 0.0;

            {
                double arc = 0.0;

                for (
                    std::size_t i = 1;
                    i < yellow_lane.track_px.size();
                    ++i
                )
                {
                    arc +=
                        cv::norm(
                            yellow_lane.track_px[i] -
                            yellow_lane.track_px[i - 1]);
                }


                if (arc > 1e-6)
                {
                    yellow_straightness =
                        cv::norm(
                            yellow_lane.track_px.back() -
                            yellow_lane.track_px.front()) / arc;
                }
            }


            // 점이 2개뿐이면 현/호 비가 무조건 1.0 이라 직선
            // 판정이 공짜로 통과한다. 점 수 하한을 같이 건다.
            if (
                !measured_width_px.empty() &&
                lane_width_px_ > 1e-6 &&
                yellow_lane.track_px.size() >= 6 &&
                yellow_straightness > 0.98
            )
            {
                std::sort(
                    measured_width_px.begin(),
                    measured_width_px.end()
                );

                const double measured =
                    measured_width_px[measured_width_px.size() / 2];

                const double ratio =
                    measured / lane_width_px_;


                if (ratio < 0.85 || ratio > 1.15)
                {
                    RCLCPP_WARN_THROTTLE(
                        this->get_logger(),
                        *this->get_clock(),
                        10000,
                        "축척 불일치: 차로 폭 실측 %.1f px vs "
                        "유도 %.1f px (%.0f%%). "
                        "camera_height_cm(%.2f) 또는 "
                        "lane_width_cm(%.1f) 을 확인하십시오. "
                        "발행 좌표가 같은 비율로 틀어집니다.",
                        measured,
                        lane_width_px_,
                        100.0 * ratio,
                        camera_height_cm_,
                        lane_width_cm_
                    );
                }
            }
        }
        else if (
            left_lane.valid &&
            right_lane.valid &&
            left_lane.found_count > 0 &&
            right_lane.found_count > 0 &&
            left_lane.track_px.size() >= 2 &&
            right_lane.track_px.size() >= 2
        )
        {
            // 노란선이 없으면 기준이 없다. 이때 흰선 base 는
            // 화면 반쪽 탐색으로 떨어지는데, 코너에서는 좌/우가
            // 같은 흰선을 무는 일이 잦다. 서로 겹치면 근거가
            // 많은 쪽만 남긴다.
            const double gap =
                std::abs(
                    geom::medianLateralOffset(
                        left_lane.track_px,
                        right_lane.track_px
                    )
                );


            if (gap < 0.5 * lane_width_px_)
            {
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    2000,
                    "좌/우 흰선이 같은 선을 물었습니다 "
                    "(간격 %.0fpx). 근거가 적은 쪽을 버립니다.",
                    gap
                );


                if (left_lane.found_count >= right_lane.found_count)
                {
                    right_lane = LaneDetectionResult();
                }
                else
                {
                    left_lane = LaneDetectionResult();
                }
            }
        }
        else
        {
            // ----------------------------------------------------
            // 노란선도 없고 흰선도 하나뿐
            //
            // 여기가 코너에서 제일 흔한 상태다. 안쪽 차선이 BEV
            // 옆으로 먼저 빠져나가고 바깥 차선만 남는다.
            //
            // 이때 그 선이 어느 쪽 선인지를 히스토그램이 화면
            // 반쪽 중 어디서 찾았는지로 정하고 있었다. 차선이
            // 세로로 서 있을 때만 맞는 규칙이다. 90도 코너에서
            // 차선이 가로로 누우면 화면 왼쪽에 오른쪽 선이 온다.
            // 그러면 아래 16-c 복원이 반대 방향으로 차로 폭만큼
            // 밀어서, 있지도 않은 자리에 중심선을 세운다.
            //
            // 맵 단면(road_map.hpp)으로 자리를 고른다.
            // ----------------------------------------------------

            const auto observed_one =
                [](const LaneDetectionResult & l)
                {
                    return l.valid &&
                           l.found_count > 0 &&
                           l.track_px.size() >= 2;
                };

            const bool have_left  = observed_one(left_lane);

            const bool have_right = observed_one(right_lane);


            double sx_one = 0.0;

            double sy_one = 0.0;

            double xb_one = 0.0;


            if (
                (have_left != have_right) &&
                bevScale(sx_one, sy_one, xb_one)
            )
            {
                const LaneDetectionResult one =
                    have_left ? left_lane : right_lane;

                const int k =
                    identifyLine(
                        one.track_px,
                        kau_road::LineType::White,
                        centerOffsetPriorCm(),
                        sx_one,
                        bev_frame.rows
                    );


                if (k < 0)
                {
                    RCLCPP_WARN_THROTTLE(
                        this->get_logger(),
                        *this->get_clock(),
                        2000,
                        "흰선 하나를 잡았는데 맵 단면의 어느 선에도 "
                        "맞지 않습니다 (자차 종축 대비 %+.1fcm). "
                        "도로 밖의 선으로 보고 버립니다.",
                        geom::medianLateralOffset(
                            egoAxis(bev_frame.rows),
                            one.track_px) * sx_one
                    );

                    left_lane = LaneDetectionResult();

                    right_lane = LaneDetectionResult();
                }
                else
                {
                    const bool want_left =
                        lineOffsetCm(k) < 0.0;


                    if (want_left != have_left)
                    {
                        RCLCPP_INFO_THROTTLE(
                            this->get_logger(),
                            *this->get_clock(),
                            2000,
                            "흰선 하나를 %s 선으로 잡았는데 맵 단면상 "
                            "%s 입니다. 자리를 옮깁니다.",
                            have_left ? "왼쪽" : "오른쪽",
                            kau_road::LINES[k].name
                        );


                        left_lane = LaneDetectionResult();

                        right_lane = LaneDetectionResult();


                        if (want_left)
                        {
                            left_lane = one;
                        }
                        else
                        {
                            right_lane = one;
                        }
                    }
                }
            }
        }


        // ========================================================
}



// ================================================================
// 16-b. 중앙선 대비 자차 횡거리 갱신.
// ================================================================

void KauLaneDetectionNode::stageResolveCenter(
    FrameContext & ctx)
{
    LaneDetectionResult & left_lane = ctx.left;
    LaneDetectionResult & right_lane = ctx.right;
    LaneDetectionResult & yellow_lane = ctx.yellow;
    cv::Mat & bev_frame = ctx.bev;

        // 16-b. 중앙선 대비 자차 위치 갱신
        //
        // 반드시 아래 16-c 복원 앞이다. 복원된 선은 관측이
        // 아니라 "차로 폭만큼 옆에 있을 것" 이라는 가정이라,
        // 그것으로 이 값을 재면 가정을 관측처럼 되돌려 읽는
        // 순환이 된다.
        // ========================================================

        ctx.center_fix =
            resolveCenterOffset(
                left_lane,
                yellow_lane,
                right_lane,
                bev_frame.rows
            );


        // ========================================================
}



// ================================================================
// 16-c. 한쪽 흰선 소실 복원.
// ================================================================

void KauLaneDetectionNode::stageRestoreMissing(
    FrameContext & ctx)
{
    LaneDetectionResult & left_lane = ctx.left;
    LaneDetectionResult & right_lane = ctx.right;
    LaneDetectionResult & yellow_lane = ctx.yellow;

        // 16-c. 한쪽 소실 복원
        //
        // 곡선에서는 안쪽 차선이 BEV 프레임 옆으로 먼저
        // 빠져나가고 바깥 차선만 남는다. 이때 남은 차선에서
        // lane_width_px 만큼 평행이동해 사라진 쪽을 복원한다.
        //
        // 노란 중앙선이 기준이라 좌/우가 뒤바뀔 여지가 없어
        // 부호만 맞추면 끝이다.
        //
        // 노란선이 없고 흰선도 하나뿐인 경우가 문제였다. 그
        // 선이 어느 쪽 선인지 틀리면 여기서 차로 폭만큼 반대로
        // 밀어 완전히 다른 중심선을 만들어 낸다. 16-a 마지막
        // 분기가 맵 단면(road_map.hpp)으로 자리를 먼저 확정하고
        // 내려오므로, 여기서는 부호만 맞추면 된다.
        //
        // 오프셋은 맵 단면의 인접 선 간격 그대로다.
        //   흰선 <-> 중앙선  1W
        //   흰선 <-> 반대 흰선 2W
        // ========================================================

        const auto restore_from =
            [&](
                const LaneDetectionResult & source,
                double offset)
            {
                LaneDetectionResult restored;

                restored.detected = true;

                restored.valid = true;

                // 복원된 선은 실제로 본 것이 아니므로
                // found_count 는 물려주지 않는다.
                restored.found_count = 0;

                // 평행이동은 반드시 법선 방향.
                // x 방향 offset 은 차선이 세로일 때만 맞다.
                restored.track_px =
                    geom::offsetTrack(
                        source.track_px,
                        offset
                    );


                return restored;
            };


        if (yellow_lane.valid)
        {
            if (!left_lane.valid)
            {
                left_lane =
                    restore_from(
                        yellow_lane,
                        -lane_width_px_
                    );
            }

            if (!right_lane.valid)
            {
                right_lane =
                    restore_from(
                        yellow_lane,
                        lane_width_px_
                    );
            }
        }
        else if (left_lane.valid && !right_lane.valid)
        {
            yellow_lane =
                restore_from(
                    left_lane,
                    lane_width_px_
                );

            right_lane =
                restore_from(
                    left_lane,
                    2.0 * lane_width_px_
                );
        }
        else if (right_lane.valid && !left_lane.valid)
        {
            yellow_lane =
                restore_from(
                    right_lane,
                    -lane_width_px_
                );

            left_lane =
                restore_from(
                    right_lane,
                    -2.0 * lane_width_px_
                );
        }


        // ========================================================
}



// ================================================================
// 16-d. 중심선 -> quintic Bezier, EMA 강건화, pan 조준.
// ================================================================

void KauLaneDetectionNode::stageBuildPath(
    const sensor_msgs::msg::Image::SharedPtr & msg,
    FrameContext & ctx)
{
    LaneDetectionResult & left_lane = ctx.left;
    LaneDetectionResult & right_lane = ctx.right;
    LaneDetectionResult & yellow_lane = ctx.yellow;
    LanePath & lane_path = ctx.path;
    bool & pan_angle_clamped = ctx.pan_angle_clamped;
    const double & camera_yaw_deg = ctx.camera_yaw_deg;
    cv::Mat & bev_frame = ctx.bev;

        // 16-d. 중심선 -> quintic Bezier 제어점 (공통 경로 형식)
        //
        // 이 노드의 최종 산출물. 이산 좌표가 아니라 제어점 6개다.
        // 차로를 묻지 않고 중앙선(황색 점선)만 추종한다.
        //
        // pan != 0 이어도 짓는다. buildCenterlinePath 가 pan 각만큼
        // 회전 보정한다 (6-b 절, 선언부 헤더 §8 도입부 주석 참고).
        //
        // 각도는 **이 영상이 찍힌 시각**의 값을 쓴다. 예전에는
        // actual_pan_deg_(= 처리 시각의 값)를 그냥 읽었는데, 카메라가
        // 도는 중이면 그 사이 각도가 달라져 있어 경로 전체가 잘못된
        // 각도로 회전했다. 77cm 경로 기준 Δθ 3도가 끝단 4.0cm,
        // 15도가 19.9cm 다 — pan 중에만 나타나는 튐의 유력 원인.
        // panAngleAt() 이 스탬프로 보간해 그 어긋남을 없앤다.
        // ========================================================

        std::vector<cv::Point2d> center_pts_px;



        ctx.camera_yaw_deg =
            panAngleAt(
                rclcpp::Time(msg->header.stamp),
                &pan_angle_clamped
            );

        // 보정량 — 이 수정이 실제로 얼마나 일했는지의 지표.
        // status 의 pdfix 로 나간다. 0 에 가까우면 이 프레임에서는
        // 카메라가 사실상 정지해 있었다는 뜻이다.
        ctx.pan_angle_fix_deg =
            camera_yaw_deg - actual_pan_deg_;

        ctx.path =
            buildCenterlinePath(
                left_lane,
                yellow_lane,
                right_lane,
                bev_frame.cols,
                bev_frame.rows,
                camera_yaw_deg,
                center_pts_px
            );


        // ========================================================
        // 16-d-2. 시간축 강건화 (EMA)
        //
        // 게이트를 통과한 관측을 추정치에 섞는다. 여기서
        // lane_path.ctrl / confidence / valid_length 가 관측값
        // 에서 추정치로 바뀐다. 아래 발행과 debug 그리기는
        // 둘 다 이 추정치를 본다.
        // ========================================================

        robustifyPath(lane_path, rclcpp::Time(msg->header.stamp));


        // ========================================================
        // 16-d-3. Pan 탐색 / 조준 (설계 문서 ⑤)
        //
        // 좌/우 검출 상태는 16-a 정리가 끝난 뒤의 값을 본다. 16-c
        // 복원과는 무관하다 — 복원선은 found_count 가 항상 0 이라
        // 여기서 읽어도 16-c 이전과 같은 값이다.
        //
        // ★ 예전에는 이 호출이 16-c-2, 즉 경로를 짓기 전에 있었다.
        //   pan_aim_source: "lane" 이 이 프레임의 경로를 근거로
        //   삼으므로 robustifyPath(EMA) 뒤로 내렸다. 직전 프레임
        //   경로를 쓰면 14Hz 에서 70ms 지연이고, v_max 요구 각속도
        //   143deg/s 기준 10도라 못 쓴다.
        //
        //   16-c-2 와 여기 사이에는 조기 반환이 없으므로 pan 이
        //   갱신되지 않는 프레임은 생기지 않는다. buildCenterlinePath
        //   가 쓰는 camera_yaw_deg 는 panAngleAt() 에서 따로 나오므로
        //   여기서 pan 을 갱신해도 순환이 생기지 않는다.
        // ========================================================

        updatePanSearch(
            left_lane,
            yellow_lane,
            right_lane,
            lane_path,
            rclcpp::Time(msg->header.stamp)
        );


        // ========================================================
}



// ================================================================
// 16-e. 경로 발행.
// ================================================================

void KauLaneDetectionNode::stagePublishPath(
    const sensor_msgs::msg::Image::SharedPtr & msg,
    FrameContext & ctx)
{
    LaneDetectionResult & left_lane = ctx.left;
    LaneDetectionResult & right_lane = ctx.right;
    LanePath & lane_path = ctx.path;
    const double & camera_yaw_deg = ctx.camera_yaw_deg;
    cv::Mat & bev_frame = ctx.bev;
    cv::Mat & debug_image = ctx.debug_image;

        // 16-e. 경로 발행
        //
        // 게이트를 통과한 경로만 나간다. 기각된 경로는 발행하지
        // 않고 debug 화면에만 붉게 남긴다.
        //
        // stamp 는 경로 생성 시각이 아니라 근거가 된 영상의
        // 시각이다. 수신측이 지연을 보상할 수 있어야 한다.
        // ========================================================

        publishLanePath(
            lane_path,
            msg->header.stamp
        );


        // ========================================================
        // 좌/우 흰선 경로 (/lane/left, /lane/right)
        //
        // 중심선과 같은 프레임의 같은 추적 결과에서 만든다. 하류가
        // 이 둘을 corridor 물리 경계로 쓰므로 중심선과 시간 정합이
        // 맞아야 한다 — 그래서 여기서 같이 낸다.
        //
        // camera_yaw_deg 는 중심선이 쓴 값을 그대로 넘긴다 (영상
        // 시각의 pan 각). 셋이 같은 회전을 받아야 base_link 기준
        // 좌표계가 일치한다.
        //
        // 한쪽이 안 잡히면 buildEdgePath 가 valid=false 를 내고
        // publishLanePath 가 조용히 건너뛴다. 하류는 그 토픽이
        // 잠시 끊기는 것으로 본다.
        // ========================================================

        publishLanePath(
            buildEdgePath(
                left_lane,
                bev_frame.cols,
                bev_frame.rows,
                camera_yaw_deg
            ),
            msg->header.stamp,
            lane_left_publisher_
        );

        publishLanePath(
            buildEdgePath(
                right_lane,
                bev_frame.cols,
                bev_frame.rows,
                camera_yaw_deg
            ),
            msg->header.stamp,
            lane_right_publisher_
        );



        if (draw_path_overlay_)
        {
            drawPathOverlay(
                lane_path,
                camera_yaw_deg,
                debug_image
            );
        }


        // ========================================================
}



// ================================================================
// 17-21. 오버레이 / status 문자열 / 디버그 이미지 발행.
// ================================================================

void KauLaneDetectionNode::stageRender(
    const sensor_msgs::msg::Image::SharedPtr & msg,
    FrameContext & ctx)
{
    LaneDetectionResult & left_lane = ctx.left;
    LaneDetectionResult & right_lane = ctx.right;
    LaneDetectionResult & yellow_lane = ctx.yellow;
    LanePath & lane_path = ctx.path;
    bool & pan_angle_clamped = ctx.pan_angle_clamped;
    const CenterFix & center_fix = ctx.center_fix;
    const bool want_debug = ctx.want_debug;
    const double & pan_angle_fix_deg = ctx.pan_angle_fix_deg;
    cv::Mat & debug_image = ctx.debug_image;
    double & yellow_band = ctx.yellow_band;
    int & left_base = ctx.left_base;
    int & right_base = ctx.right_base;
    int & yellow_base = ctx.yellow_base;

        // ========================================================
        // 여기서부터 20절 직전까지는 전부 debug_image 에 그리는
        // 일뿐이다. 구독자가 없으면 통째로 건너뛴다.
        //
        // status 문자열 조립(std::string 연결 3회)과 putText 4~5줄이
        // 여기 들어 있다. 기계가 읽을 값은 21절의 status 토픽이
        // 따로 내므로 이 블록을 건너뛰어도 잃는 정보가 없다.
        // ========================================================

        if (want_debug)
        {


        // 17. Base Point Visualization
        // ========================================================

        if (left_base >= 0)
        {
            cv::circle(
                debug_image,
                cv::Point(
                    left_base,
                    debug_image.rows - 10
                ),
                6,
                cv::Scalar(
                    255,
                    0,
                    0
                ),
                -1
            );
        }


        if (yellow_base >= 0)
        {
            cv::circle(
                debug_image,
                cv::Point(
                    yellow_base,
                    debug_image.rows - 10
                ),
                6,
                cv::Scalar(
                    0,
                    255,
                    255
                ),
                -1
            );
        }


        if (right_base >= 0)
        {
            cv::circle(
                debug_image,
                cv::Point(
                    right_base,
                    debug_image.rows - 10
                ),
                6,
                cv::Scalar(
                    0,
                    0,
                    255
                ),
                -1
            );
        }


        // ========================================================
        // 18. Image Center
        // ========================================================

        cv::line(
            debug_image,
            cv::Point(
                debug_image.cols / 2,
                0
            ),
            cv::Point(
                debug_image.cols / 2,
                debug_image.rows
            ),
            cv::Scalar(
                255,
                255,
                255
            ),
            1
        );


        // ========================================================
        // 19. Status
        // ========================================================

        // 어디서 끊기는지 보이도록 창 검출 수까지 표시한다.
        //
        //   n/9  : 실제로 n 개 창에서 픽셀을 잡음
        //   REST : 반대편에서 lane_width_px 로 복원
        //   FAIL : 복원도 불가

        const auto lane_label =
            [](const LaneDetectionResult & lane)
            {
                if (!lane.valid)
                {
                    return std::string("FAIL");
                }


                if (lane.found_count == 0)
                {
                    return std::string("REST");
                }


                // 스텝 수는 더 이상 NUM_WINDOWS 로 고정이 아니다.
                // (코너에서는 같은 거리를 더 많은 스텝으로 돈다)
                return std::to_string(lane.found_count);
            };


        const std::string status =
            "L " + lane_label(left_lane) +
            " | Y " + lane_label(yellow_lane) +
            " | R " + lane_label(right_lane);


        // 경로 요약: 길이 / 최대 곡률 반경 / 횡오차
        char path_text[128];

        if (lane_path.valid)
        {
            const double r =
                (lane_path.kappa_max > 1e-9)
                    ? 1.0 / lane_path.kappa_max
                    : 0.0;

            std::snprintf(
                path_text,
                sizeof(path_text),
                "path %.0fcm | R %.0fcm | cte %+.1fcm | "
                "yaw %+.1fdeg",
                lane_path.length_cm,
                r,
                lane_path.cte_cm,
                lane_path.heading_err * 180.0 / M_PI
            );
        }
        else if (lane_path.built)
        {
            std::snprintf(
                path_text,
                sizeof(path_text),
                "REJECT %s | win %d | R %.0fcm",
                gateLabel(lane_path.reject).ascii,
                lane_path.source_windows,
                (lane_path.kappa_max > 1e-9)
                    ? 1.0 / lane_path.kappa_max : 0.0
            );
        }
        else
        {
            std::snprintf(
                path_text,
                sizeof(path_text),
                "path --"
            );
        }


        cv::putText(
            debug_image,
            path_text,
            cv::Point(
                10,
                45
            ),
            cv::FONT_HERSHEY_SIMPLEX,
            0.45,
            lane_path.valid
                ? cv::Scalar(255, 0, 255)
                : cv::Scalar(60, 60, 255),
            1
        );


        cv::putText(
            debug_image,
            status,
            cv::Point(
                10,
                25
            ),
            cv::FONT_HERSHEY_SIMPLEX,
            0.5,
            cv::Scalar(
                255,
                255,
                255
            ),
            1
        );


        // --------------------------------------------------------
        // 중앙선 대비 자차 위치
        //
        // 자차 종축(BEV 중앙 열)을 아래쪽에 짧게 그어 두면,
        // 판정된 횡거리가 화면에서 바로 읽힌다. 차로 개념은
        // 없으므로 순수하게 "중앙선에서 얼마나 떨어져 있는가"
        // 만 보여준다.
        // --------------------------------------------------------

        {
            const int axis_x =
                (bev_dst_points_.size() == 4)
                    ? static_cast<int>(
                          std::lround(
                              0.5 *
                              (bev_dst_points_[0].x +
                               bev_dst_points_[1].x)))
                    : debug_image.cols / 2;


            cv::line(
                debug_image,
                cv::Point(axis_x, debug_image.rows - 1),
                cv::Point(axis_x, debug_image.rows - 30),
                cv::Scalar(200, 200, 200),
                1
            );


            char center_text[128];

            if (center_fix.valid)
            {
                std::snprintf(
                    center_text,
                    sizeof(center_text),
                    "CENTER off %+.1fcm | src %d",
                    center_fix.offset_cm,
                    center_fix.source
                );
            }
            else
            {
                std::snprintf(
                    center_text,
                    sizeof(center_text),
                    "CENTER --"
                );
            }


            cv::putText(
                debug_image,
                center_text,
                cv::Point(
                    10,
                    65
                ),
                cv::FONT_HERSHEY_SIMPLEX,
                0.45,
                center_fix.valid
                    ? cv::Scalar(0, 255, 0)
                    : cv::Scalar(120, 120, 120),
                1
            );
        }


        // --------------------------------------------------------
        // Pan 탐색 상태 (설계 문서 ⑤)
        //
        // Idle 이 아닌 동안은 16-d 에서 경로를 새로 짓지 않으므로,
        // 화면에서 "path -- " 가 보이면서 아래 이 줄이 IDLE 이
        // 아니면 pan 탐색 때문이라는 걸 바로 알 수 있게 한다.
        // --------------------------------------------------------

        if (pan_search_enable_)
        {
            const char * pan_state_name =
                (pan_state_ == PanSearchState::Searching) ? "SEARCH" :
                (pan_state_ == PanSearchState::Holding)   ? "HOLD"   :
                (pan_state_ == PanSearchState::Returning) ? "RETURN" :
                                                             "IDLE";

            char pan_text[96];

            std::snprintf(
                pan_text,
                sizeof(pan_text),
                "PAN %s goal %+.1f cmd %+.1f actual %+.1fdeg",
                pan_state_name,
                pan_goal_deg_,
                pan_cmd_deg_,
                actual_pan_deg_
            );

            cv::putText(
                debug_image,
                pan_text,
                cv::Point(10, 85),
                cv::FONT_HERSHEY_SIMPLEX,
                0.45,
                (pan_state_ == PanSearchState::Idle)
                    ? cv::Scalar(120, 120, 120)
                    : cv::Scalar(0, 165, 255),
                1
            );
        }

        }   // if (want_debug)


        // ========================================================
        // 20. Publish Debug Image
        // ========================================================

        if (want_debug)
        {
        auto debug_msg =
            cv_bridge::CvImage(
                msg->header,
                sensor_msgs::image_encodings::BGR8,
                debug_image
            ).toImageMsg();


        debug_image_publisher_->publish(
            *debug_msg
        );
        }


        // ========================================================
        // 21. 상태 토픽
        //
        // 로그 파싱 대신 기계가 읽을 수 있는 형태로 낸다.
        //   lw/yw/rw : 실제로 픽셀을 잡은 창 수 (복원선은 0)
        //   lv/yv/rv : 그 차선을 쓸 수 있는 상태인지
        //   rad      : 경로 최소 곡률반경 [cm]. 0 이면 직선
        //   coff     : 중앙선 -> 자차 횡거리 [cm]. 오른쪽이 +
        //   csrc     : coff 판정 근거 (0 없음 / 1 노란선 /
        //              2 흰선 두 개 / 3 흰선 하나+맵 단면 /
        //              4 직전 유지)
        //   paim     : pan 조준 목표각 [deg]. 왼쪽이 +
        //   pav      : 이 프레임에 조준이 유효했는가 (0 이면 탐색 폴백)
        //   pasrc    : 조준 근거 (0 차선 / 1 전역경로)
        //   pahold   : 근거 부족으로 직전 조준각을 유지한 연속 프레임
        //              수. 0 이 아닌 값이 오래 이어지면 근거가 계속
        //              모자란다는 뜻이다 (pan_aim_min_evidence_cm 확인)
        // ========================================================

        {
            // 필드를 늘리면 여기도 같이 늘릴 것. snprintf 는 넘치면
            // 조용히 자르고, lane_failure_logger.py 는 잘린 줄에서
            // 뒤쪽 필드를 못 찾는다.
            char rec[448];

            // 제어점 최대 횡편차. 게이트3(시야이탈)이 보는 값 그대로.
            double max_lat = 0.0;

            for (const kau::bezier::Point2 & cp : lane_path.ctrl)
            {
                max_lat =
                    std::max(
                        max_lat,
                        std::abs(cp.y - path_y_offset_cm_));
            }


            const double radius =
                (lane_path.valid && lane_path.kappa_max > 1e-9)
                    ? 1.0 / lane_path.kappa_max
                    : 0.0;

            std::snprintf(
                rec,
                sizeof(rec),
                "lw=%d yw=%d rw=%d lv=%d yv=%d rv=%d "
                "path=%d gate=%d win=%d band=%.2f "
                "len=%.1f rad=%.1f maxlat=%.1f cte=%.2f yaw=%.2f "
                "coff=%.1f csrc=%d "
                "pan=%d pdeg=%.1f padeg=%.1f pgdeg=%.1f "
                "pdfix=%.2f pdext=%d "
                "conf=%.2f edev=%.1f egate=%.1f erej=%d "
                "pbend=%.1f pvis=%d "
                "paim=%.1f pav=%d pasrc=%d pahold=%d",
                left_lane.found_count,
                yellow_lane.found_count,
                right_lane.found_count,
                left_lane.valid   ? 1 : 0,
                yellow_lane.valid ? 1 : 0,
                right_lane.valid  ? 1 : 0,
                lane_path.valid   ? 1 : 0,
                lane_path.reject,
                lane_path.source_windows,
                yellow_band,
                lane_path.length_cm,
                radius,
                max_lat,
                lane_path.cte_cm,
                lane_path.heading_err * 180.0 / M_PI,
                center_fix.offset_cm,
                center_fix.source,
                static_cast<int>(pan_state_),
                pan_cmd_deg_,
                actual_pan_deg_,
                pan_goal_deg_,
                pan_angle_fix_deg,
                pan_angle_clamped ? 1 : 0,
                lane_path.confidence,
                last_path_dev_cm_,
                last_path_gate_cm_,
                path_ema_reject_streak_,
                last_bend_deg_,
                last_visible_at_zero_,
                last_pan_aim_deg_,
                last_pan_aim_valid_ ? 1 : 0,
                (pan_aim_source_ == "global") ? 1 : 0,
                pan_aim_hold_streak_
            );

            std_msgs::msg::String status_msg;

            status_msg.data = rec;

            status_publisher_->publish(status_msg);
        }
}



// ================================================================
// 프레임 파이프라인
//
// 단계 순서만 정한다. 각 단계의 내용은 위 함수들에 있고,
// 서로는 FrameContext 를 통해서만 이어진다.
// ================================================================

void KauLaneDetectionNode::imageCallback(
    const sensor_msgs::msg::Image::SharedPtr msg)
{
    try
    {
        FrameContext ctx;

        if (!stagePrepareFrame(msg, ctx))
        {
            return;
        }

        stageBuildBev(msg, ctx);

        stageBuildMasks(ctx);

        stagePrepareDebugCanvas(ctx);

        stageDetectLanes(ctx);

        stageValidateSides(ctx);

        stageResolveCenter(ctx);

        stageRestoreMissing(ctx);

        stageBuildPath(msg, ctx);

        stagePublishPath(msg, ctx);

        stageRender(msg, ctx);
    }


    // ============================================================
    // cv_bridge Exception
    // ============================================================

    catch (const cv_bridge::Exception & e)
    {
        RCLCPP_ERROR(
            this->get_logger(),
            "cv_bridge exception: %s",
            e.what()
        );
    }


    // ============================================================
    // OpenCV Exception
    // ============================================================

    catch (const cv::Exception & e)
    {
        RCLCPP_ERROR(
            this->get_logger(),
            "OpenCV exception: %s",
            e.what()
        );
    }


    // ============================================================
    // Standard Exception
    // ============================================================

    catch (const std::exception & e)
    {
        RCLCPP_ERROR(
            this->get_logger(),
            "Exception: %s",
            e.what()
        );
    }
}
