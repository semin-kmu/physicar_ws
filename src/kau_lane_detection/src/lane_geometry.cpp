// ====================================================================
// lane_geometry.cpp
//
// lane_geometry.hpp 구현. 근거는 CLAUDE.md.
// ====================================================================

#include "kau_lane_detection/lane_geometry.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>

#include <Eigen/Dense>

namespace kau_lane
{
namespace geom
{

// ================================================================
// 파라미터(int 배열) -> cv::Scalar
// ================================================================

cv::Scalar toScalar(
    const std::vector<int64_t> & v)
{
    return cv::Scalar(
        v.size() > 0 ? static_cast<double>(v[0]) : 0.0,
        v.size() > 1 ? static_cast<double>(v[1]) : 0.0,
        v.size() > 2 ? static_cast<double>(v[2]) : 0.0
    );
}


// ================================================================
// 가중 최소제곱 다항식 적합  v = f(t)
//
// 중심선을 매개변수 s (누적 현길이) 로 적합하기 위한 도구다.
// x(s), y(s) 를 각각 한 번씩 부른다.
//
// 예전에는 x = f(y) 하나만 적합했다. 그 표현은 차선이 가로로
// 누우면 dx/dy = 무한대라 어떤 다항식으로도 표현할 수 없다.
// 90도 코너에서 차선이 끊기던 근본 원인이 여기였다.
//
// s 로 매개화하면 방향에 무관해진다. x(s) 도 y(s) 도 완만한
// 다항식이고, 코너에서도 발산하지 않는다.
//
// OpenCV 에 polyfit 이 없어 Vandermonde 행렬을 만들어
// cv::solve(DECOMP_QR) 로 직접 푼다. 가중치는 각 행에
// sqrt(w) 를 곱해서 넣는다 (가중 최소제곱의 표준 변형).
//
// coeffs[0]*t^order + ... + coeffs[order]
// ================================================================

bool polyFitW(
    const std::vector<double> & ts,
    const std::vector<double> & vs,
    const std::vector<double> & weights,
    int order,
    std::vector<double> & coeffs)
{
    const int n =
        static_cast<int>(ts.size());


    if (
        order < 1 ||
        n < order + 1 ||
        n != static_cast<int>(vs.size())
    )
    {
        return false;
    }


    const bool use_w =
        weights.size() == ts.size();


    // t 범위가 없으면 (모든 점이 같은 자리) 적합 불가
    const auto t_range =
        std::minmax_element(
            ts.begin(),
            ts.end()
        );


    if (*t_range.second - *t_range.first < 1e-6)
    {
        return false;
    }


    cv::Mat a(
        n,
        order + 1,
        CV_64F
    );

    cv::Mat b(
        n,
        1,
        CV_64F
    );


    for (
        int i = 0;
        i < n;
        ++i
    )
    {
        const double rw =
            std::sqrt(
                use_w ? std::max(0.0, weights[i]) : 1.0
            );


        double power = 1.0;


        for (
            int j = order;
            j >= 0;
            --j
        )
        {
            a.at<double>(i, j) = rw * power;

            power *= ts[i];
        }


        b.at<double>(i, 0) = rw * vs[i];
    }


    cv::Mat solution;


    if (
        !cv::solve(
            a,
            b,
            solution,
            cv::DECOMP_QR
        )
    )
    {
        return false;
    }


    coeffs.assign(
        order + 1,
        0.0
    );


    for (
        int j = 0;
        j <= order;
        ++j
    )
    {
        coeffs[j] = solution.at<double>(j, 0);
    }


    return true;
}


double polyEval(
    const std::vector<double> & coeffs,
    double y)
{
    double value = 0.0;


    for (const double coefficient : coeffs)
    {
        value = value * y + coefficient;
    }


    return value;
}


  


// ================================================================
// 폴리라인 위로 투영
//
// 세 차선 후보를 하나의 매개변수 s 로 묶기 위한 도구다.
//
// 후보끼리 s 를 각자 0 부터 세면 안 된다. 세 차선은 서로 다른
// 지점에서 추적이 시작되므로 같은 s 가 같은 위치를 뜻하지
// 않는다. 실패 65장 실측 잔차 중앙값이 24.6px 였고, 그 대부분이
// 이 어긋남에서 나왔다 (기준선 투영 시 10.4px).
//
// 양 끝 구간에서는 t 를 [0,1] 로 자르지 않는다. 기준선보다
// 앞/뒤로 나간 후보 점을 끝에 뭉치게 두면 그 자리만 과대
// 가중되기 때문이다. s < 0 이 나올 수 있고, 호출측에서
// 전체를 s_min 만큼 평행이동해 [0, L] 로 되돌린다.
// ================================================================

void projectOnPolyline(
    const std::vector<cv::Point2d> & poly,
    const std::vector<double> & cum,
    const cv::Point2d & p,
    double & s_out,
    double & d_out)
{
    s_out = 0.0;

    d_out = std::numeric_limits<double>::max();


    const int nseg =
        static_cast<int>(poly.size()) - 1;


    for (
        int j = 0;
        j < nseg;
        ++j
    )
    {
        const double dx = poly[j + 1].x - poly[j].x;

        const double dy = poly[j + 1].y - poly[j].y;

        const double len2 = dx * dx + dy * dy;


        if (len2 < 1e-12)
        {
            continue;
        }


        double t =
            ((p.x - poly[j].x) * dx +
             (p.y - poly[j].y) * dy) / len2;


        // 양 끝 구간만 외삽 허용
        const double lo = (j == 0)        ? -1e9 : 0.0;

        const double hi = (j == nseg - 1) ?  1e9 : 1.0;

        t = std::clamp(t, lo, hi);


        const double qx = poly[j].x + t * dx;

        const double qy = poly[j].y + t * dy;

        const double d =
            std::hypot(p.x - qx, p.y - qy);


        if (d < d_out)
        {
            d_out = d;

            s_out = cum[j] + t * std::sqrt(len2);
        }
    }
}


// ================================================================
// 기준선 대비 부호 있는 횡거리의 중앙값
//
// "왼쪽 흰선이 정말 왼쪽에 있는가" 를 확인하는 데 쓴다.
//
// 히스토그램 base 탐색은 x 구간
//
//   왼쪽  [yellow - 1.6W, yellow - 0.4W]
//   오른쪽 [yellow + 0.4W, yellow + 1.6W]
//
// 으로 되어 있다. 차선이 세로로 서 있을 때만 "x 로 왼쪽" 이
// "차로의 왼쪽" 과 같다. 90도 코너에서 차선이 가로로 누우면
// 좌/우 흰선은 노란선의 위/아래에 쌓이므로, x 구간 탐색이
// 반대쪽 선이나 같은 선을 물어 온다.
//
// 실패 프레임 중 노란선 추적이 된 55장 실측:
//   왼쪽 흰선이 왼쪽에 없음      11장
//   오른쪽 흰선이 오른쪽에 없음   5장
//   좌/우가 같은 선을 물음        8장
//
// base 탐색 자체를 법선 방향으로 바꿔 봐도 검출 수만 크게
// 줄고 (L 40 -> 17장) 이득이 없었다. 그래서 탐색은 그대로 두고,
// 추적 결과를 노란선 기준으로 검증해 반대쪽이면 버린다.
// ================================================================

std::vector<double> lateralOffsets(
    const std::vector<cv::Point2d> & ref,
    const std::vector<cv::Point2d> & pts)
{
    std::vector<double> offs;


    if (ref.size() < 2 || pts.empty())
    {
        return offs;
    }


    offs.reserve(pts.size());


    for (const cv::Point2d & p : pts)
    {
        double best_d = std::numeric_limits<double>::max();

        double best_o = 0.0;


        for (
            std::size_t j = 0;
            j + 1 < ref.size();
            ++j
        )
        {
            const double dx = ref[j + 1].x - ref[j].x;

            const double dy = ref[j + 1].y - ref[j].y;

            const double len2 = dx * dx + dy * dy;


            if (len2 < 1e-12)
            {
                continue;
            }


            const double t =
                std::clamp(
                    ((p.x - ref[j].x) * dx +
                     (p.y - ref[j].y) * dy) / len2,
                    0.0,
                    1.0
                );

            const double qx = ref[j].x + t * dx;

            const double qy = ref[j].y + t * dy;

            const double d = std::hypot(p.x - qx, p.y - qy);


            if (d < best_d)
            {
                const double len = std::sqrt(len2);

                // 진행방향 +90도 = 차량 오른쪽
                best_o =
                    (p.x - qx) * (-dy / len) +
                    (p.y - qy) * ( dx / len);

                best_d = d;
            }
        }


        offs.push_back(best_o);
    }


    return offs;
}


// ================================================================
// 부호 있는 횡거리의 중앙값
// ================================================================

double medianLateralOffset(
    const std::vector<cv::Point2d> & ref,
    const std::vector<cv::Point2d> & pts)
{
    std::vector<double> offs =
        lateralOffsets(ref, pts);


    if (offs.empty())
    {
        return 0.0;
    }


    std::nth_element(
        offs.begin(),
        offs.begin() + offs.size() / 2,
        offs.end()
    );


    return offs[offs.size() / 2];
}


// ================================================================
// 기준선 대비 부호 있는 횡거리 하나 (외삽 허용)
//
// lateralOffsets 는 각 구간의 투영 매개변수 t 를 [0,1] 로 자른다.
// 추적점끼리 재는 데에는 맞다. 하지만 자차 위치는 추적 시작점
// 보다 뒤(BEV 아래쪽)에 있어서, 자르면 첫 점까지의 직선거리가
// 나와 값이 부풀려지고 방향도 어긋난다.
//
// 양 끝 구간에서만 t 를 밖으로 열어 둔다. 중간 구간까지 열면
// 폴리라인이 꺾이는 곳에서 엉뚱한 구간에 붙는다.
// ================================================================

double lateralOffsetAt(
    const std::vector<cv::Point2d> & ref,
    const cv::Point2d & p)
{
    if (ref.size() < 2)
    {
        return 0.0;
    }


    const std::size_t nseg = ref.size() - 1;

    double best_d = std::numeric_limits<double>::max();

    double best_o = 0.0;


    for (
        std::size_t j = 0;
        j < nseg;
        ++j
    )
    {
        const double dx = ref[j + 1].x - ref[j].x;

        const double dy = ref[j + 1].y - ref[j].y;

        const double len2 = dx * dx + dy * dy;


        if (len2 < 1e-12)
        {
            continue;
        }


        const double lo = (j == 0)        ? -1e9 : 0.0;

        const double hi = (j == nseg - 1) ?  1e9 : 1.0;


        const double t =
            std::clamp(
                ((p.x - ref[j].x) * dx +
                 (p.y - ref[j].y) * dy) / len2,
                lo,
                hi
            );

        const double qx = ref[j].x + t * dx;

        const double qy = ref[j].y + t * dy;

        const double d = std::hypot(p.x - qx, p.y - qy);


        if (d < best_d)
        {
            const double len = std::sqrt(len2);

            // 진행방향 +90도 = 차량 오른쪽
            best_o =
                (p.x - qx) * (-dy / len) +
                (p.y - qy) * ( dx / len);

            best_d = d;
        }
    }


    return best_o;
}


// ================================================================
// 회랑 이탈 지점에서 절단
//
// 흰 마스크에는 차선만 있는 게 아니다. 체커보드 연석, 정지선,
// 그리고 급커브에서는 트랙의 다른 구간 차선까지 들어온다.
// 방향성 창은 그것들을 구분하지 못하고 그대로 갈아탄다.
//
// 실측 (실패 프레임 63장, 노란선 추적 성공분):
//   흰선 검출 L 34 / R 53 중 회랑을 벗어난 것 L 7 / R 3
//   추적점 1309개 중 68개(5%)만 잘리고, 잘린 뒤 3점 미만은 2건
//   그 7 프레임에서 중심선이 최대 117cm 달라졌다
//
// 차로 폭은 물리적으로 고정이므로, 노란선에서 그만큼 떨어져
// 있지 않은 점은 우리 차로의 흰선이 아니다. 통째로 버리면
// 정상이던 앞부분까지 잃으므로 이탈 지점에서 자른다.
// ================================================================

std::vector<cv::Point2d> truncateAtCorridor(
    const std::vector<cv::Point2d> & track,
    const std::vector<cv::Point2d> & ref,
    double want_sign,
    double lo_px,
    double hi_px)
{
    if (ref.size() < 2 || track.empty())
    {
        return track;
    }


    const std::vector<double> offs =
        lateralOffsets(ref, track);


    if (offs.size() != track.size())
    {
        return track;
    }


    std::size_t keep = 0;


    while (keep < track.size())
    {
        const double o = offs[keep] * want_sign;

        if (o < lo_px || o > hi_px)
        {
            break;
        }

        ++keep;
    }


    return std::vector<cv::Point2d>(
        track.begin(),
        track.begin() + static_cast<long>(keep)
    );
}


// ================================================================
// 점열을 국소 법선 방향으로 평행이동
//
// 흰선에서 중심선을 만들 때 쓴다.
//
// 예전에는 x 좌표에 +- lane_width_px 를 더했다. 차선이 세로로
// 서 있을 때만 맞는 식이다. 90도 코너에서 차선이 가로로 누우면
// 중심선은 x 가 아니라 y 로 밀려야 하므로 완전히 빗나간다.
//
// 국소 접선 t 를 +90도 회전한 법선
//
//   t = (0,-1) (화면 위 = 차량 전방)  ->  n = (1,0) (차량 오른쪽)
//
// 을 쓰면 방향에 무관하게 항상 옳다.
// ================================================================

std::vector<cv::Point2d> offsetTrack(
    const std::vector<cv::Point2d> & pts,
    double offset_px)
{
    std::vector<cv::Point2d> out;


    const std::size_t n = pts.size();


    if (n < 2)
    {
        return out;
    }


    out.reserve(n);


    for (
        std::size_t i = 0;
        i < n;
        ++i
    )
    {
        // 중앙차분. 끝점은 한쪽 차분.
        const std::size_t a = (i == 0) ? 0 : i - 1;

        const std::size_t b = (i + 1 < n) ? i + 1 : i;


        double tx = pts[b].x - pts[a].x;

        double ty = pts[b].y - pts[a].y;


        const double len = std::hypot(tx, ty);


        if (len < 1e-9)
        {
            tx = 0.0;

            ty = -1.0;
        }
        else
        {
            tx /= len;

            ty /= len;
        }


        out.push_back(
            cv::Point2d(
                pts[i].x + offset_px * (-ty),
                pts[i].y + offset_px * ( tx)
            )
        );
    }


    return out;
}


// ================================================================
// 점열의 꺾임각 [deg]
//
// 앞 절반의 현과 뒤 절반의 현이 이루는 각. 점이 모자라거나
// 현이 퇴화하면 -1 (판정 불가).
//
// 호모그래피가 직선을 직선으로 보내므로 BEV 좌표로 재도
// "직선인가" 는 성립한다. 크기는 보존되지 않는다 (헤더 주석).
// ================================================================

double trackBendDeg(
    const std::vector<cv::Point2d> & pts)
{
    // 절반씩 갈라 현을 두 개 만들려면 최소 4점.
    if (pts.size() < 4)
    {
        return -1.0;
    }


    const std::size_t mid = pts.size() / 2;

    const cv::Point2d d1 = pts[mid] - pts.front();

    const cv::Point2d d2 = pts.back() - pts[mid];


    if (
        cv::norm(d1) < 1e-6 ||
        cv::norm(d2) < 1e-6
    )
    {
        return -1.0;
    }


    double diff =
        std::atan2(d2.y, d2.x) - std::atan2(d1.y, d1.x);

    while (diff > M_PI)
    {
        diff -= 2.0 * M_PI;
    }

    while (diff < -M_PI)
    {
        diff += 2.0 * M_PI;
    }


    return std::abs(diff) * 180.0 / M_PI;
}

}  // namespace geom
}  // namespace kau_lane
