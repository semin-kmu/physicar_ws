// ====================================================================
// bezier_ext.hpp
//
// kau_control/bezier.hpp 는 Lane Detection 소비 범위(적합/평가/발행 전
// 검사)만 포팅했다. Local Path Planner 는 그 위에 "Hermite 경계조건으로
// segment 를 구성"하고 "구성 전 저렴한 곡률 상한으로 걸러내는" 두 가지가
// 추가로 필요한데, 이건 소비자(lane detection/controller) 쪽에는 없는
// **생산자** 전용 연산이라 여기(우리 패키지)에 둔다.
//
// 원본: KAU_AMET_Test / src/sim_common/curve.py 의
//       state_derivs / hermite_to_bezier / kappa_bound (문서 6.2, 6.6)
//
// kau::bezier 의 hodograph/split/distPointHull/binom 위에서만 동작하므로
// kau_control 을 건드리지 않고 순수 추가로 짤 수 있다.
// ====================================================================

#ifndef KAU_LOCAL_PATH_PLANNER__BEZIER_EXT_HPP_
#define KAU_LOCAL_PATH_PLANNER__BEZIER_EXT_HPP_

#include <algorithm>
#include <cmath>
#include <optional>
#include <vector>

#include "kau_control/bezier.hpp"
#include "kau_control/curve.hpp"

namespace kau
{
namespace local_path_planner
{

using kau::bezier::Ctrl;
using kau::bezier::Point2;

// ---------------------------------------------------------------------
// (theta, kappa, sigma, dsigma) -> (P', P'').  문서 6.2.
//     T = (cos t, sin t)   N = (-sin t, cos t)
//     P'  = sigma * T
//     P'' = dsigma * T + sigma^2 * kappa * N
// ---------------------------------------------------------------------
inline void stateDerivs(
    double theta, double kappa, double sigma, double dsigma,
    Point2 & d1_out, Point2 & d2_out)
{
    const Point2 t{std::cos(theta), std::sin(theta)};
    const Point2 n{-std::sin(theta), std::cos(theta)};

    d1_out = sigma * t;
    d2_out = dsigma * t + Point2{sigma * sigma * kappa * n.x,
                                sigma * sigma * kappa * n.y};
}

// ---------------------------------------------------------------------
// quintic Hermite -> Bezier 제어점 (정확한 기저 변환). 문서 6.2.
//
//   B0 = P0
//   B1 = P0 + P'(0)/5
//   B2 = P0 + 2P'(0)/5 + P''(0)/20
//   B3 = P1 - 2P'(1)/5 + P''(1)/20
//   B4 = P1 - P'(1)/5
//   B5 = P1
// ---------------------------------------------------------------------
inline Ctrl hermiteToBezier(
    const Point2 & p0, double th0, double k0,
    const Point2 & p1, double th1, double k1,
    double sig0, double sig1, double dsig0 = 0.0, double dsig1 = 0.0)
{
    Point2 d1_0, d2_0, d1_1, d2_1;
    stateDerivs(th0, k0, sig0, dsig0, d1_0, d2_0);
    stateDerivs(th1, k1, sig1, dsig1, d1_1, d2_1);

    return Ctrl{
        p0,
        p0 + 0.2 * d1_0,
        p0 + 0.4 * d1_0 + 0.05 * d2_0,
        p1 - 0.4 * d1_1 + 0.05 * d2_1,
        p1 - 0.2 * d1_1,
        p1,
    };
}

// ---------------------------------------------------------------------
// convex hull 기반 |kappa| 상한. 문서 6.6. 항상 참값 이상 -> 안전측 판정.
//
//   |kappa| <= max|w_k| / dist(0, conv{hodograph 제어점})^3
//   w = cross(P', P'') 의 Bernstein 계수 (7차)
//
// de Casteljau 로 depth 회 분할한 조각마다 계산해 최댓값을 취한다
// (분할할수록 과대율이 줄어듦, depth=2 채택 -- 과대율 1.15).
// ---------------------------------------------------------------------
inline double kappaBound(const Ctrl & ctrl, int depth = 2)
{
    const auto one = [](const Ctrl & c) -> double
    {
        const Ctrl v = kau::bezier::hodograph(c);      // 4차 (5 제어점)
        const double d = kau::bezier::distPointHull(Point2{0.0, 0.0}, v);
        if (d < kau::bezier::EPS_DEGEN)
        {
            return std::numeric_limits<double>::infinity();
        }
        const Ctrl a = kau::bezier::hodograph(v);       // 3차 (4 제어점)

        std::vector<double> w(8, 0.0);
        for (int i = 0; i < 5; ++i)
        {
            for (int j = 0; j < 4; ++j)
            {
                const double b = kau::bezier::binom(4, i) *
                                 kau::bezier::binom(3, j) /
                                 kau::bezier::binom(7, i + j);
                w[static_cast<std::size_t>(i + j)] +=
                    b * (v[static_cast<std::size_t>(i)].x *
                             a[static_cast<std::size_t>(j)].y -
                         v[static_cast<std::size_t>(i)].y *
                             a[static_cast<std::size_t>(j)].x);
            }
        }
        double wmax = 0.0;
        for (double x : w)
        {
            wmax = std::max(wmax, std::abs(x));
        }
        return wmax / (d * d * d);
    };

    std::vector<Ctrl> pieces{ctrl};
    for (int level = 0; level < depth; ++level)
    {
        std::vector<Ctrl> next;
        next.reserve(pieces.size() * 2);
        for (const Ctrl & c : pieces)
        {
            Ctrl left, right;
            kau::bezier::split(c, 0.5, left, right);
            next.push_back(std::move(left));
            next.push_back(std::move(right));
        }
        pieces = std::move(next);
    }

    double best = 0.0;
    for (const Ctrl & c : pieces)
    {
        best = std::max(best, one(c));
    }
    return best;
}

// ---------------------------------------------------------------------
// (point, heading, kappa) 를 호길이 s 에서 한 번의 locate() 로 평가.
// Python Curve.frame(s) 대응. kau_control::Curve 는 point/heading/kappa
// 를 각각 제공하지만 매번 locate() 를 다시 하므로, 셋을 동시에 쓸 때는
// 이 helper 로 한 번만 locate() 한다.
// ---------------------------------------------------------------------
struct Frame
{
    Point2 point;
    double heading = 0.0;
    double kappa   = 0.0;
};

inline Frame referenceFrame(const kau::control::Curve & cv, double s)
{
    int seg = 0;
    double u = 0.0;
    cv.locate(s, seg, u);

    const Ctrl & c = cv.seg(seg);
    Frame f;
    f.point   = kau::bezier::evalSeg(c, u);
    f.heading = kau::bezier::heading(c, u);
    f.kappa   = kau::bezier::curvature(c, u);
    return f;
}

}  // namespace local_path_planner
}  // namespace kau

#endif  // KAU_LOCAL_PATH_PLANNER__BEZIER_EXT_HPP_
