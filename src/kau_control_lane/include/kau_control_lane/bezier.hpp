// ====================================================================
// bezier.hpp
//
// quintic Bezier 경로 핵심 연산 (C++ 포팅).
// 원본: KAU_AMET_Test / src/sim_common/curve.py  (docs/경로_형식.md 구현)
// 출처: kau_lane_detection/include/kau_lane_detection/bezier.hpp 에서 복사.
//       제어기가 lane detection 에 의존하지 않도록 의도적으로 중복 보유한다.
//       8.3 전역 탐색 / 8.4 window 추적은 소비자 기능이라 curve.hpp 가 담당.
//
// 단위: 길이 cm / 각도 rad / 곡률 1/cm
//
// 전 연산이 해석적. 경로를 이산 좌표로 저장하지 않는다 (대전제).
// 곡선 sampling(sample) 은 시각화에서만 쓴다.
//
// 포팅 범위 (lane detection 이 실제로 쓰는 것 + 발행 전 검사)
//     5.4 기저 변환    to_power / power_to_bernstein / elevate
//     8.1 최근접점     9차 다항식 실근 전체 + 단부, 반복법 미사용
//     8.2 근 탐색      동반행렬 고유값 (Eigen::EigenSolver = numpy.roots)
//     8.5 호길이       Gauss-Legendre 10점
//     8.6 곡률         14차 근 정확값
//     8.7 퇴화 검사    hodograph convex hull 의 원점 배제
//
// 전역 탐색(8.3) / window 추적(8.4) 은 경로 소비자 쪽 기능이라
// 생산자인 lane detection 에는 포팅하지 않았다.
// ====================================================================

#ifndef KAU_CONTROL_LANE__BEZIER_HPP_
#define KAU_CONTROL_LANE__BEZIER_HPP_

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>


namespace kau
{
namespace bezier
{

// ====================================================================
// 상수
// ====================================================================

constexpr int    DEGREE     = 5;

constexpr int    NCTRL      = DEGREE + 1;

// 퇴화 판정 |r'| 하한
constexpr double EPS_DEGEN  = 1e-6;


// ====================================================================
// 평면 점
// ====================================================================

struct Point2
{
    double x = 0.0;

    double y = 0.0;
};


inline Point2 operator+(const Point2 & a, const Point2 & b)
{
    return Point2{a.x + b.x, a.y + b.y};
}


inline Point2 operator-(const Point2 & a, const Point2 & b)
{
    return Point2{a.x - b.x, a.y - b.y};
}


inline Point2 operator*(double s, const Point2 & a)
{
    return Point2{s * a.x, s * a.y};
}


using Ctrl = std::vector<Point2>;


// ====================================================================
// 이항계수
// ====================================================================

inline double binom(int n, int k)
{
    if (k < 0 || k > n)
    {
        return 0.0;
    }

    double result = 1.0;

    for (int i = 0; i < k; ++i)
    {
        result = result * (n - i) / (i + 1);
    }

    return std::round(result);
}


// ====================================================================
// 5.4 기저 변환
// ====================================================================

// 제어점 -> 멱기저 계수 (낮은 차수부터).
//
//   M[j][i] = C(n,j) * C(j,i) * (-1)^(j-i)
//   poly_coef = M @ ctrl
inline std::vector<Point2> toPower(const Ctrl & ctrl)
{
    const int n = static_cast<int>(ctrl.size()) - 1;

    std::vector<Point2> coef(n + 1, Point2{});


    for (int j = 0; j <= n; ++j)
    {
        for (int i = 0; i <= j; ++i)
        {
            const double m =
                binom(n, j) * binom(j, i) *
                (((j - i) % 2 == 0) ? 1.0 : -1.0);

            coef[j].x += m * ctrl[i].x;

            coef[j].y += m * ctrl[i].y;
        }
    }


    return coef;
}


// 멱기저 계수 -> 제어점.
//
//   b_i = sum_{j<=i} [ C(i,j) / C(n,j) ] * c_j
inline Ctrl powerToBernstein(const std::vector<Point2> & coef)
{
    const int n = static_cast<int>(coef.size()) - 1;

    Ctrl b(n + 1, Point2{});


    for (int i = 0; i <= n; ++i)
    {
        for (int j = 0; j <= i; ++j)
        {
            const double w = binom(i, j) / binom(n, j);

            b[i].x += w * coef[j].x;

            b[i].y += w * coef[j].y;
        }
    }


    return b;
}


// degree elevation. 무손실.
//
//   B'_i = (i/(n+1)) * B_{i-1} + (1 - i/(n+1)) * B_i
inline Ctrl elevate(const Ctrl & ctrl, int target = DEGREE)
{
    Ctrl c = ctrl;


    while (static_cast<int>(c.size()) - 1 < target)
    {
        const int n = static_cast<int>(c.size()) - 1;

        Ctrl out(n + 2, Point2{});

        out.front() = c.front();

        out.back()  = c.back();


        for (int i = 1; i <= n; ++i)
        {
            const double a =
                static_cast<double>(i) / (n + 1);

            out[i] = a * c[i - 1] + (1.0 - a) * c[i];
        }


        c = out;
    }


    return c;
}


// ====================================================================
// segment 평가
// ====================================================================

// de Casteljau. 수치적으로 가장 안정.
inline Point2 evalSeg(const Ctrl & ctrl, double u)
{
    Ctrl p = ctrl;

    const int n = static_cast<int>(p.size()) - 1;


    for (int r = 0; r < n; ++r)
    {
        for (int i = 0; i < n - r; ++i)
        {
            p[i] = (1.0 - u) * p[i] + u * p[i + 1];
        }
    }


    return p[0];
}


// 도함수 제어점.  H_i = n * (B_{i+1} - B_i)
inline Ctrl hodograph(const Ctrl & ctrl)
{
    const int n = static_cast<int>(ctrl.size()) - 1;

    Ctrl h;

    h.reserve(n);


    for (int i = 0; i < n; ++i)
    {
        h.push_back(
            static_cast<double>(n) * (ctrl[i + 1] - ctrl[i]));
    }


    return h;
}


// kappa = (x'y'' - y'x'') / |P'|^3.  좌회전 +
inline double curvature(const Ctrl & ctrl, double u)
{
    const Ctrl d1 = hodograph(ctrl);

    const Ctrl d2 = hodograph(d1);

    const Point2 v1 = evalSeg(d1, u);

    const Point2 v2 = evalSeg(d2, u);

    const double den =
        std::pow(v1.x * v1.x + v1.y * v1.y, 1.5);


    if (den < 1e-12)
    {
        return 0.0;
    }


    return (v1.x * v2.y - v1.y * v2.x) / den;
}


inline double heading(const Ctrl & ctrl, double u)
{
    const Point2 v = evalSeg(hodograph(ctrl), u);

    return std::atan2(v.y, v.x);
}


// de Casteljau 분할 -> (좌, 우). 정확.
inline void split(
    const Ctrl & ctrl,
    double u,
    Ctrl & left,
    Ctrl & right)
{
    Ctrl p = ctrl;

    const int n = static_cast<int>(p.size()) - 1;

    left.assign(1, p.front());

    right.assign(1, p.back());


    for (int r = 0; r < n; ++r)
    {
        for (int i = 0; i < n - r; ++i)
        {
            p[i] = (1.0 - u) * p[i] + u * p[i + 1];
        }

        left.push_back(p[0]);

        right.push_back(p[n - r - 1]);
    }


    std::reverse(right.begin(), right.end());
}


// ====================================================================
// 8.5 호길이 — Gauss-Legendre 10점
//
// 구간 [-1,1] 10점. 100 cm 당 절대오차 27 um.
// ====================================================================

inline const double * glNodes()
{
    static const double x[10] = {
        -0.9739065285171717, -0.8650633666889845,
        -0.6794095682990244, -0.4333953941292472,
        -0.1488743389816312,  0.1488743389816312,
         0.4333953941292472,  0.6794095682990244,
         0.8650633666889845,  0.9739065285171717
    };

    return x;
}


inline const double * glWeights()
{
    static const double w[10] = {
        0.0666713443086881, 0.1494513491505806,
        0.2190863625159820, 0.2692667193099963,
        0.2955242247147529, 0.2955242247147529,
        0.2692667193099963, 0.2190863625159820,
        0.1494513491505806, 0.0666713443086881
    };

    return w;
}


inline double segLength(
    const Ctrl & ctrl,
    double u0 = 0.0,
    double u1 = 1.0)
{
    const double half = 0.5 * (u1 - u0);

    const double mid  = 0.5 * (u1 + u0);

    const Ctrl d1 = hodograph(ctrl);

    double total = 0.0;


    for (int i = 0; i < 10; ++i)
    {
        const Point2 v =
            evalSeg(d1, mid + half * glNodes()[i]);

        total += glWeights()[i] * std::hypot(v.x, v.y);
    }


    return total * half;
}


// ====================================================================
// 다항식 유틸 (낮은 차수부터 저장)
// ====================================================================

inline std::vector<double> polyMul(
    const std::vector<double> & a,
    const std::vector<double> & b)
{
    if (a.empty() || b.empty())
    {
        return {};
    }


    std::vector<double> out(a.size() + b.size() - 1, 0.0);


    for (std::size_t i = 0; i < a.size(); ++i)
    {
        for (std::size_t j = 0; j < b.size(); ++j)
        {
            out[i + j] += a[i] * b[j];
        }
    }


    return out;
}


inline std::vector<double> polyAdd(
    const std::vector<double> & a,
    const std::vector<double> & b)
{
    std::vector<double> out(std::max(a.size(), b.size()), 0.0);


    for (std::size_t i = 0; i < a.size(); ++i)
    {
        out[i] += a[i];
    }

    for (std::size_t i = 0; i < b.size(); ++i)
    {
        out[i] += b[i];
    }


    return out;
}


inline std::vector<double> polyScale(
    const std::vector<double> & a,
    double s)
{
    std::vector<double> out = a;

    for (double & v : out)
    {
        v *= s;
    }

    return out;
}


inline std::vector<double> polyDer(const std::vector<double> & a)
{
    if (a.size() <= 1)
    {
        return {0.0};
    }


    std::vector<double> out(a.size() - 1, 0.0);


    for (std::size_t i = 1; i < a.size(); ++i)
    {
        out[i - 1] = a[i] * static_cast<double>(i);
    }


    return out;
}


// ====================================================================
// 8.2 근 탐색 — 동반행렬 고유값
//
// numpy.roots 와 동일한 알고리즘. Newton 등 반복법을 쓰지 않으므로
// 초기값 의존 / 미수렴 / 발산이 구조적으로 불가능하다.
// ====================================================================

inline std::vector<double> realRootsInUnit(
    const std::vector<double> & coef,
    double tol = 1e-9)
{
    // 최고차 0 계수 제거 (numpy 의 trim_zeros(c, "b"))
    std::vector<double> c = coef;

    while (c.size() > 1 && std::abs(c.back()) == 0.0)
    {
        c.pop_back();
    }


    if (c.size() <= 1)
    {
        return {};
    }


    const int n = static_cast<int>(c.size()) - 1;


    // 동반행렬 (numpy.roots 와 동일한 배치)
    //   A[0, :] = -p[1:] / p[0]     (p 는 높은 차수부터)
    //   부대각선 = 1
    Eigen::MatrixXd a =
        Eigen::MatrixXd::Zero(n, n);


    for (int i = 0; i < n; ++i)
    {
        a(0, i) = -c[n - 1 - i] / c[n];
    }

    for (int i = 1; i < n; ++i)
    {
        a(i, i - 1) = 1.0;
    }


    Eigen::EigenSolver<Eigen::MatrixXd> solver(a, false);


    if (solver.info() != Eigen::Success)
    {
        return {};
    }


    const Eigen::VectorXcd ev = solver.eigenvalues();


    // 실근 판정 기준을 numpy 포팅과 맞춘다.
    double scale = 1.0;

    for (int i = 0; i < ev.size(); ++i)
    {
        scale = std::max(scale, std::abs(ev(i).real()));
    }


    std::vector<double> roots;


    for (int i = 0; i < ev.size(); ++i)
    {
        if (std::abs(ev(i).imag()) >= tol * scale)
        {
            continue;
        }


        const double t = ev(i).real();


        if (t > 0.0 && t < 1.0)
        {
            roots.push_back(t);
        }
    }


    return roots;
}


// ====================================================================
// 8.1 최근접점 — 9차 다항식
//
// g(t) = (r(t) - p) . r'(t) 는 quintic 에서 정확히 9차.
// 후보 = { g=0 의 (0,1) 실근 } U {0, 1} -> 전역 최소 포함 보장.
// ====================================================================

struct Nearest
{
    double u    = 0.0;

    double dist = 0.0;
};


inline Nearest nearestOnSeg(const Ctrl & ctrl, const Point2 & p)
{
    const std::vector<Point2> c = toPower(ctrl);

    std::vector<double> cx(c.size());

    std::vector<double> cy(c.size());


    for (std::size_t i = 0; i < c.size(); ++i)
    {
        cx[i] = c[i].x;

        cy[i] = c[i].y;
    }


    const std::vector<double> dx = polyDer(cx);

    const std::vector<double> dy = polyDer(cy);


    std::vector<double> gx = cx;

    std::vector<double> gy = cy;

    gx[0] -= p.x;

    gy[0] -= p.y;


    const std::vector<double> g =
        polyAdd(polyMul(gx, dx), polyMul(gy, dy));


    std::vector<double> cand = realRootsInUnit(g);

    cand.push_back(0.0);

    cand.push_back(1.0);


    Nearest best;

    best.dist = std::numeric_limits<double>::infinity();


    for (const double u : cand)
    {
        const Point2 q = evalSeg(ctrl, u);

        const double d = std::hypot(q.x - p.x, q.y - p.y);


        if (d < best.dist)
        {
            best.dist = d;

            best.u    = u;
        }
    }


    return best;
}


// ====================================================================
// 8.6 곡률 — 14차 근 정확값
//
// kappa = N / D^{3/2} 의 극값 조건:  2*N'*D - 3*N*D' = 0
// ====================================================================

inline double kappaMaxExact(const Ctrl & ctrl)
{
    const std::vector<Point2> c = toPower(ctrl);

    std::vector<double> px(c.size());

    std::vector<double> py(c.size());


    for (std::size_t i = 0; i < c.size(); ++i)
    {
        px[i] = c[i].x;

        py[i] = c[i].y;
    }


    const std::vector<double> x1 = polyDer(px);

    const std::vector<double> y1 = polyDer(py);

    const std::vector<double> x2 = polyDer(x1);

    const std::vector<double> y2 = polyDer(y1);


    const std::vector<double> n =
        polyAdd(polyMul(x1, y2), polyScale(polyMul(y1, x2), -1.0));

    const std::vector<double> d =
        polyAdd(polyMul(x1, x1), polyMul(y1, y1));


    const std::vector<double> poly =
        polyAdd(
            polyScale(polyMul(polyDer(n), d),  2.0),
            polyScale(polyMul(n, polyDer(d)), -3.0));


    std::vector<double> cand = realRootsInUnit(poly);

    cand.push_back(0.0);

    cand.push_back(1.0);


    double best = 0.0;


    for (const double u : cand)
    {
        best = std::max(best, std::abs(curvature(ctrl, u)));
    }


    return best;
}


// ====================================================================
// 8.7 퇴화 검사
// ====================================================================

// monotone chain. 반시계 방향 정점 열.
inline std::vector<Point2> convexHull(const std::vector<Point2> & pts)
{
    std::vector<Point2> q = pts;


    // 중복 제거 후 x -> y 오름차순 정렬 (numpy unique + lexsort 와 동일)
    const auto round12 =
        [](double v)
        {
            return std::round(v * 1e12) / 1e12;
        };


    for (Point2 & v : q)
    {
        v.x = round12(v.x);

        v.y = round12(v.y);
    }


    std::sort(
        q.begin(),
        q.end(),
        [](const Point2 & a, const Point2 & b)
        {
            return (a.x != b.x) ? (a.x < b.x) : (a.y < b.y);
        });


    q.erase(
        std::unique(
            q.begin(),
            q.end(),
            [](const Point2 & a, const Point2 & b)
            {
                return a.x == b.x && a.y == b.y;
            }),
        q.end());


    if (q.size() <= 2)
    {
        return q;
    }


    const auto half =
        [](const std::vector<Point2> & seq)
        {
            std::vector<Point2> out;


            for (const Point2 & pt : seq)
            {
                while (out.size() >= 2)
                {
                    const Point2 & a = out[out.size() - 2];

                    const Point2 & b = out[out.size() - 1];

                    const double cross =
                        (b.x - a.x) * (pt.y - a.y) -
                        (b.y - a.y) * (pt.x - a.x);


                    if (cross > 0.0)
                    {
                        break;
                    }


                    out.pop_back();
                }


                out.push_back(pt);
            }


            return out;
        };


    std::vector<Point2> rev(q.rbegin(), q.rend());

    std::vector<Point2> lower = half(q);

    std::vector<Point2> upper = half(rev);

    lower.pop_back();

    upper.pop_back();

    lower.insert(lower.end(), upper.begin(), upper.end());


    return lower;
}


// 점-볼록껍질 거리. 점이 껍질 내부면 0.
inline double distPointHull(
    const Point2 & p,
    const std::vector<Point2> & pts)
{
    const std::vector<Point2> h = convexHull(pts);


    if (h.empty())
    {
        return std::numeric_limits<double>::infinity();
    }


    if (h.size() == 1)
    {
        return std::hypot(h[0].x - p.x, h[0].y - p.y);
    }


    double best = std::numeric_limits<double>::infinity();

    bool inside = h.size() > 2;

    const std::size_t m = h.size();


    for (std::size_t i = 0; i < m; ++i)
    {
        const Point2 & a = h[i];

        const Point2 & b = h[(i + 1) % m];

        const Point2 ab = b - a;

        const double l2 = ab.x * ab.x + ab.y * ab.y;

        double t = 0.0;


        if (l2 >= 1e-18)
        {
            t = ((p.x - a.x) * ab.x + (p.y - a.y) * ab.y) / l2;

            t = std::clamp(t, 0.0, 1.0);
        }


        best =
            std::min(
                best,
                std::hypot(
                    a.x + t * ab.x - p.x,
                    a.y + t * ab.y - p.y));


        // 반시계 껍질이므로 모든 변의 좌측이면 내부
        if (ab.x * (p.y - a.y) - ab.y * (p.x - a.x) < 0.0)
        {
            inside = false;
        }
    }


    return inside ? 0.0 : best;
}


// min |r'(u)|.  |r'|^2 도함수 근으로 정확 산출.
inline double minSpeed(const Ctrl & ctrl)
{
    const std::vector<Point2> c = toPower(ctrl);

    std::vector<double> px(c.size());

    std::vector<double> py(c.size());


    for (std::size_t i = 0; i < c.size(); ++i)
    {
        px[i] = c[i].x;

        py[i] = c[i].y;
    }


    const std::vector<double> x1 = polyDer(px);

    const std::vector<double> y1 = polyDer(py);

    const std::vector<double> d =
        polyAdd(polyMul(x1, x1), polyMul(y1, y1));


    std::vector<double> cand = realRootsInUnit(polyDer(d));

    cand.push_back(0.0);

    cand.push_back(1.0);


    const Ctrl v = hodograph(ctrl);

    double best = std::numeric_limits<double>::infinity();


    for (const double u : cand)
    {
        const Point2 s = evalSeg(v, u);

        best = std::min(best, std::hypot(s.x, s.y));
    }


    return best;
}


// 퇴화(cusp) 부재 여부. 발행 직전 검사.
inline bool isRegular(
    const Ctrl & ctrl,
    double eps = EPS_DEGEN)
{
    // (1) hodograph convex hull 이 원점을 배제하면 즉시 확정
    if (distPointHull(Point2{0.0, 0.0}, hodograph(ctrl)) > eps)
    {
        return true;
    }


    // (2) 정밀 확인
    return minSpeed(ctrl) > eps;
}


// ====================================================================
// 시각화 전용
//
// 연산에 사용 금지 (대전제: 경로를 이산 좌표로 저장하지 않는다).
// ====================================================================

inline std::vector<Point2> sample(const Ctrl & ctrl, int n = 40)
{
    std::vector<Point2> out;

    out.reserve(n);


    for (int i = 0; i < n; ++i)
    {
        out.push_back(
            evalSeg(
                ctrl,
                static_cast<double>(i) / (n - 1)));
    }


    return out;
}

}  // namespace bezier
}  // namespace kau

#endif  // KAU_CONTROL_LANE__BEZIER_HPP_
