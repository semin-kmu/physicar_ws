// ====================================================================
// lane_detect.cpp
//
// 마스크에서 차선을 뽑는 단계. 히스토그램 시작점 + 방향성 슬라이딩 윈도우.
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
// Histogram Peak
//
// 첨부 코드의 방식:
//
// roi_hist = edges.rowRange(height * 0.6, height)
//
// 즉 BEV 하단 40%만 이용해서 histogram을 계산
// ================================================================

int KauLaneDetectionNode::findHistogramPeak(
    const cv::Mat & mask,
    int x_start,
    int x_end,
    double * band_used)
{
    x_start = std::max(
        0,
        x_start
    );

    x_end = std::min(
        mask.cols,
        x_end
    );


    if (band_used != nullptr)
    {
        *band_used = 0.0;
    }


    if (x_start >= x_end || mask.rows <= 0)
    {
        return -1;
    }


    // ------------------------------------------------------------
    // 좁은 밴드부터 시도하고, 비면 넓혀서 재시도.
    //
    // 노란 중앙선이 점선이라 좁은 밴드가 통째로 점선 공백에 걸릴 수
    // 있다. 그때는 넓혀서라도 찾는 편이 아무것도 못 찾는 것보다 낫다.
    // 대신 넓힌 밴드의 peak 는 곡선에서 부정확하므로, 어느 밴드가
    // 쓰였는지 호출측에 알려 준다.
    // ------------------------------------------------------------

    const double base =
        std::clamp(
            histogram_band_ratio_,
            0.02,
            1.0
        );

    const double ratios[3] =
    {
        base,
        std::min(base * 2.0, 1.0),
        std::min(base * 4.0, 1.0)
    };


    for (const double ratio : ratios)
    {
        const int start_y =
            std::clamp(
                static_cast<int>(mask.rows * (1.0 - ratio)),
                0,
                mask.rows - 1
            );


        std::vector<int> histogram(
            mask.cols,
            0
        );


        for (
            int y = start_y;
            y < mask.rows;
            ++y
        )
        {
            const uchar * row = mask.ptr<uchar>(y);

            for (
                int x = x_start;
                x < x_end;
                ++x
            )
            {
                if (row[x] > 0)
                {
                    histogram[x]++;
                }
            }
        }


        int max_x = -1;

        int max_value = 0;


        for (
            int x = x_start;
            x < x_end;
            ++x
        )
        {
            if (histogram[x] > max_value)
            {
                max_value = histogram[x];

                max_x = x;
            }
        }


        if (max_value > 0)
        {
            if (band_used != nullptr)
            {
                *band_used = ratio;
            }

            return max_x;
        }
    }


    return -1;
}


// ================================================================
// 방향성 슬라이딩 윈도우
//
// 예전 방식은 창을 y 격자에 못 박아 두고 아래에서 위로만
// 올렸다. 그 구조에는 세 가지 한계가 동시에 있었다.
//
//   1. 결과가 x = f(y) 였다.
//      차선이 가로로 누우면 dx/dy = 무한대라 표현 불가.
//   2. 가로 차선은 y 밴드 하나(360/9 = 40px)에 통째로 들어간다.
//      잡히는 창이 1개뿐이라 MIN_VALID_WINDOWS(3) 를 못 넘긴다.
//   3. 가로 구간의 창 무게중심은 창 중앙 그대로다.
//      창이 옆으로 따라가질 못하니 다음 창은 그냥 빈 하늘이다.
//
// 그래서 90도 코너에서 차선이 통째로 끊겼다.
//
// 지금 방식은 창을 진행 방향 theta 로 회전시키고, 그 방향으로
// 한 스텝씩 전진한다.
//
//   - 창 안 픽셀의 무게중심으로 위치를 보정
//   - 직전 점 -> 새 무게중심 벡터로 theta 갱신
//     (한 스텝 회전량은 track_max_turn_deg 로 제한 — 되꺾임 방지)
//   - 비면 관성으로 직진 (점선 공백 통과), 연속 miss 가
//     track_max_miss 를 넘으면 종료
//
// 결과는 y 격자에 묶이지 않은 "순서 있는 점열"이다.
// 방향에 아무 전제가 없으므로 90도든 180도든 따라간다.
// ================================================================

KauLaneDetectionNode::LaneDetectionResult KauLaneDetectionNode::detectLaneSlidingWindow(
    const cv::Mat & mask,
    int base_x,
    cv::Mat & debug_image,
    const cv::Scalar & window_color)
{
    LaneDetectionResult result;


    if (base_x < 0)
    {
        return result;
    }


    const int height = mask.rows;

    const int width = mask.cols;


    // 진행 방향 창 길이 = 한 스텝 전진 거리.
    //
    // 예전에는 height / NUM_WINDOWS (= 40px) 에 묶여 있었다.
    // 그 값으로는 90도 코너의 급전환 지점에서 창이 차선을
    // 지나쳐 버린다 (파라미터 선언부 주석 참고).
    const double step =
        std::max(4.0, track_step_px_);

    const double half = 0.5 * step;

    const double margin =
        static_cast<double>(window_margin_);

    const double max_turn =
        track_max_turn_deg_ * M_PI / 180.0;


    // 스캔 범위는 창을 감싸는 정사각형(반경 hypot(half, margin)) 이
    // 아니라 회전 사각형의 **밀착** bounding box 다. dx = u·ct − v·st 이므로
    // |u|<=half, |v|<=margin 인 점은 반드시
    //   |dx| <= half|ct| + margin|st|,  |dy| <= half|st| + margin|ct|
    // 를 만족한다 — 밀착 박스는 창의 진부분집합이 아니라 상위집합이라
    // 결과가 비트 단위로 같다.
    //
    // 정사각형은 th = -pi/2 (직진) 에서 84x84 = 7056 px 를 훑는데
    // 실제 창은 80x25 = 2000 px 뿐이다. 3.5배를 헛돌았다. 창 하나당
    // 5천 px 씩, 프레임당 창 40~50개면 20만 px 이 순수 낭비였다.


    double cx = static_cast<double>(base_x);

    double cy = static_cast<double>(height) - half;

    // 화면 위(-y) = 차량 전방
    double th = -M_PI / 2.0;

    int miss = 0;


    for (
        int s = 0;
        s < track_max_steps_;
        ++s
    )
    {
        const double ct = std::cos(th);

        const double st = std::sin(th);


        // 회전 사각형의 밀착 bounding box (위 reach 주석의 근거)
        const double ext_x =
            half * std::abs(ct) + margin * std::abs(st);

        const double ext_y =
            half * std::abs(st) + margin * std::abs(ct);


        const int x0 =
            std::max(0, static_cast<int>(std::floor(cx - ext_x)));

        const int x1 =
            std::min(width, static_cast<int>(std::ceil(cx + ext_x)) + 1);

        const int y0 =
            std::max(0, static_cast<int>(std::floor(cy - ext_y)));

        const int y1 =
            std::min(height, static_cast<int>(std::ceil(cy + ext_y)) + 1);


        // --------------------------------------------------------
        // 회전 창 내부 픽셀의 무게중심
        //
        //   u =  dx cos + dy sin   진행 방향  (|u| <= half)
        //   v = -dx sin + dy cos   횡 방향    (|v| <= margin)
        // --------------------------------------------------------

        double sum_x = 0.0;

        double sum_y = 0.0;

        int pixel_count = 0;


        for (
            int y = y0;
            y < y1;
            ++y
        )
        {
            const uchar * row = mask.ptr<uchar>(y);

            const double dy =
                static_cast<double>(y) - cy;


            for (
                int x = x0;
                x < x1;
                ++x
            )
            {
                if (row[x] == 0)
                {
                    continue;
                }


                const double dx =
                    static_cast<double>(x) - cx;

                const double u =  dx * ct + dy * st;

                const double v = -dx * st + dy * ct;


                if (
                    std::abs(u) <= half &&
                    std::abs(v) <= margin
                )
                {
                    sum_x += x;

                    sum_y += y;

                    pixel_count++;
                }
            }
        }


        const bool found =
            pixel_count > min_pixels_;


        if (found)
        {
            const double mx = sum_x / pixel_count;

            const double my = sum_y / pixel_count;


            // ----------------------------------------------------
            // 진행 방향 갱신
            //
            // 회전량을 제한하지 않으면 옆 차선이나 노이즈를
            // 물었을 때 창이 되꺾여 왔던 길을 되돌아간다.
            // ----------------------------------------------------

            if (!result.track_px.empty())
            {
                const cv::Point2d & prev =
                    result.track_px.back();

                const double vx = mx - prev.x;

                const double vy = my - prev.y;


                if (std::hypot(vx, vy) > 1.0)
                {
                    const double target =
                        std::atan2(vy, vx);

                    const double delta =
                        std::remainder(
                            target - th,
                            2.0 * M_PI
                        );


                    th +=
                        std::clamp(
                            delta,
                            -max_turn,
                            max_turn
                        );
                }
            }


            cx = mx;

            cy = my;

            result.track_px.push_back(
                cv::Point2d(cx, cy)
            );

            result.found_count++;

            miss = 0;


            // ----------------------------------------------------
            // 시각화: 실제로 잡은 창만 그린다.
            // 회전 사각형이라 차선 방향이 눈으로 바로 보인다.
            //
            // debug_image 가 비어 있으면 (= 구독자 없음) 통째로
            // 건너뛴다. 창 하나당 RotatedRect + cv::line 4회라
            // 프레임당 180회쯤 되고, 전부 아무도 안 보는 그림이었다.
            // ----------------------------------------------------

            if (!debug_image.empty())
            {

            cv::Point2f corners[4];

            cv::RotatedRect(
                cv::Point2f(
                    static_cast<float>(cx),
                    static_cast<float>(cy)
                ),
                cv::Size2f(
                    static_cast<float>(step),
                    static_cast<float>(2.0 * margin)
                ),
                static_cast<float>(th * 180.0 / M_PI)
            ).points(corners);


            for (
                int i = 0;
                i < 4;
                ++i
            )
            {
                cv::line(
                    debug_image,
                    corners[i],
                    corners[(i + 1) % 4],
                    window_color,
                    1
                );
            }

            }
        }
        else
        {
            miss++;


            if (miss > track_max_miss_)
            {
                break;
            }
        }


        // --------------------------------------------------------
        // 다음 창으로 전진 (방향은 theta)
        // --------------------------------------------------------

        cx += step * std::cos(th);

        cy += step * std::sin(th);


        if (
            cx < -margin ||
            cx > width + margin ||
            cy < -half ||
            cy > height + half
        )
        {
            break;
        }
    }


    result.detected =
        result.found_count >= MIN_VALID_WINDOWS;

    result.valid = result.detected;


    if (!result.valid)
    {
        return result;
    }


    // ============================================================
    // 추적 궤적 시각화
    //
    // 다항식으로 다시 그리지 않는다. 점열 자체가 결과다.
    // ============================================================

    if (debug_image.empty())
    {
        return result;
    }


    for (
        std::size_t i = 1;
        i < result.track_px.size();
        ++i
    )
    {
        cv::line(
            debug_image,
            cv::Point(
                static_cast<int>(result.track_px[i - 1].x),
                static_cast<int>(result.track_px[i - 1].y)
            ),
            cv::Point(
                static_cast<int>(result.track_px[i].x),
                static_cast<int>(result.track_px[i].y)
            ),
            window_color,
            2
        );
    }


    return result;
}
