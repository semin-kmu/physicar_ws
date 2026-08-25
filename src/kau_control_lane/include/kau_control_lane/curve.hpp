// ====================================================================
// curve.hpp
//
// quintic Bezier segment 열 = 경로. 경로 **소비자** 쪽 연산.
// 원본: KAU_AMET_Test / src/sim_common/curve.py 의 Curve / TrackState
//       (docs/경로_형식.md 8.3 / 8.4 / 8.5 구현)
//
// 단위: 길이 cm / 각도 rad / 곡률 1/cm
//
// bezier.hpp 가 segment 1개 연산(8.1 최근접점, 8.5 호길이, 8.6 곡률)을 담고
// 이 파일이 그 위에 segment **열** 연산을 얹는다.
//     8.3 전역 최근접점   AABB 하한 branch-and-bound. 주행 시작 / 복구 시 1회
//     8.4 국소 window     호길이 window + 최소표현 delta_s. 주행 중 매 tick
//     8.5 LookAhead       호길이 기준 전진
//
// 경로를 이산 좌표로 저장하지 않는다 (대전제). sample() 은 시각화 전용.
// ====================================================================

#ifndef KAU_CONTROL_LANE__CURVE_HPP_
#define KAU_CONTROL_LANE__CURVE_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <numeric>
#include <vector>

#include "kau_control_lane/bezier.hpp"


namespace kau
{
namespace control_lane
{

using bezier::Ctrl;
using bezier::Point2;

// segment 열 상수 (curve.py 상단과 동일값)
constexpr int NCTRL = bezier::NCTRL;

// 국소 window 추적 parameter (문서 8.4)
constexpr double FWD        = 100.0;   // cm, 전방 window
constexpr double BACK       = 50.0;    // cm, 후방 window
constexpr double GATE       = 250.0;   // cm, 경로 이탈 판정 거리
constexpr int    FAIL_LIMIT = 3;       // 연속 실패 -> 전역 재탐색


inline double wrapPi(double a)
{
    const double two_pi = 2.0 * M_PI;

    double r = std::fmod(a + M_PI, two_pi);

    if (r < 0.0)
    {
        r += two_pi;
    }

    return r - M_PI;
}


// ====================================================================
// 국소 window 추적 상태 (문서 8.4)
//
// valid=false 로 두고 nearest() 를 호출하면 전역 탐색으로 시작한다.
// ====================================================================

struct TrackState
{
    int    seg   = 0;

    double u     = 0.0;

    double s     = 0.0;      // cm, 경로 시작점부터의 호길이

    double dist  = 0.0;      // cm, 질의점까지 거리

    int    fails = 0;

    bool   valid = false;
};


// ====================================================================
// 경로
// ====================================================================

class Curve
{
public:
    Curve() = default;

    // ctrl: segment 열. 각 원소가 제어점 NCTRL 개.
    Curve(std::vector<Ctrl> ctrl, bool closed)
    : ctrl_(std::move(ctrl)),
      closed_(closed)
    {
        rebuild();
    }

    // ---------- 기본 ----------

    bool empty() const
    {
        return ctrl_.empty();
    }

    int nseg() const
    {
        return static_cast<int>(ctrl_.size());
    }

    bool closed() const
    {
        return closed_;
    }

    double length() const
    {
        return cum_s_.empty() ? 0.0 : cum_s_.back();
    }

    const std::vector<Ctrl> & ctrl() const
    {
        return ctrl_;
    }

    const Ctrl & seg(int i) const
    {
        return ctrl_[static_cast<std::size_t>(i)];
    }

    double segLen(int i) const
    {
        return seg_len_[static_cast<std::size_t>(i)];
    }

    // 개곡선은 단부 clamp, 폐곡선은 순환.
    double wrapS(double s) const
    {
        const double len = length();

        if (len <= 0.0)
        {
            return 0.0;
        }

        if (!closed_)
        {
            return std::min(std::max(s, 0.0), len);
        }

        double r = std::fmod(s, len);

        return r < 0.0 ? r + len : r;
    }

    // a -> b 최소표현 부호거리. 폐곡선 wrap 오판 방지 (문서 8.4).
    //
    // 성립 조건: 비교하는 두 호길이의 차가 L/2 미만.
    // segment 1개가 L/2 이상이면 window 판정이 뒤집힌다 (windowSafe 참조).
    double deltaS(double a, double b) const
    {
        if (!closed_)
        {
            return b - a;
        }

        const double len = length();

        double r = std::fmod(b - a, len);

        if (r < 0.0)
        {
            r += len;
        }

        return r > 0.5 * len ? r - len : r;
    }

    // 국소 window 추적이 성립하는 구성인가 (문서 8.4).
    //
    // 폐곡선에서 segment 1개가 전장의 절반 이상이면 deltaS 최소표현이 뒤집혀
    // window 판정이 무너진다. 레퍼런스 실측: nseg=2 에서 전역 탐색 대비
    // 불일치 29%, nseg >= 3 에서 0%.
    bool windowSafe() const
    {
        if (!closed_ || seg_len_.empty())
        {
            return true;
        }

        const double mx =
            *std::max_element(seg_len_.begin(), seg_len_.end());

        return mx < 0.5 * length();
    }

    // ---------- 좌표 ----------

    // 호길이 -> (segment index, u). 역산은 safeguarded Newton (문서 8.5).
    void locate(double s, int & seg_out, double & u_out) const
    {
        s = wrapS(s);

        // cum_s_ 에서 s 를 담는 구간을 찾는다 (searchsorted right - 1)
        std::size_t i =
            static_cast<std::size_t>(
                std::upper_bound(cum_s_.begin(), cum_s_.end(), s) -
                cum_s_.begin());

        i = (i == 0) ? 0 : i - 1;

        i = std::min(i, static_cast<std::size_t>(nseg() - 1));

        seg_out = static_cast<int>(i);

        u_out = invertArclen(seg_out, s - cum_s_[i]);
    }

    // segment i 내에서 s_local 에 해당하는 u.
    //
    // h(u) = s(u) - s_target, h'(u) = |r'(u)| > 0 -> h 순증가 -> 발산 불가.
    // bracket 을 유지하고 이탈 시 bisection 으로 폴백.
    double invertArclen(int i, double s_local) const
    {
        const Ctrl & c = seg(i);

        const double len = segLen(i);

        if (len < 1e-12)
        {
            return 0.0;
        }

        s_local = std::min(std::max(s_local, 0.0), len);

        double lo = 0.0;

        double hi = 1.0;

        double u = s_local / len;

        const Ctrl d1 = bezier::hodograph(c);


        for (int it = 0; it < 5; ++it)
        {
            const double h = bezier::segLength(c, 0.0, u) - s_local;

            if (std::abs(h) < 1e-9)
            {
                break;
            }

            if (h > 0.0)
            {
                hi = u;
            }
            else
            {
                lo = u;
            }

            const Point2 v = bezier::evalSeg(d1, u);

            const double sp = std::hypot(v.x, v.y);

            const double u_new =
                sp > 1e-12 ? u - h / sp : 0.5 * (lo + hi);

            u = (u_new > lo && u_new < hi) ? u_new : 0.5 * (lo + hi);
        }


        return std::min(std::max(u, 0.0), 1.0);
    }

    Point2 point(double s) const
    {
        int i = 0;

        double u = 0.0;

        locate(s, i, u);

        return bezier::evalSeg(seg(i), u);
    }

    double heading(double s) const
    {
        int i = 0;

        double u = 0.0;

        locate(s, i, u);

        return bezier::heading(seg(i), u);
    }

    double kappa(double s) const
    {
        int i = 0;

        double u = 0.0;

        locate(s, i, u);

        return bezier::curvature(seg(i), u);
    }

    double sOf(int seg_i, double u) const
    {
        return cum_s_[static_cast<std::size_t>(seg_i)] +
               bezier::segLength(seg(seg_i), 0.0, u);
    }

    // ---------- 8.3 전역 최근접점 ----------

    // AABB 하한 branch-and-bound. 주행 시작 / 복구 시 1회.
    TrackState nearestGlobal(const Point2 & p) const
    {
        TrackState best;

        best.dist  = std::numeric_limits<double>::infinity();

        best.valid = true;

        if (ctrl_.empty())
        {
            best.valid = false;

            return best;
        }


        // segment 별 AABB 하한 -> 오름차순 -> 하한이 현재 최적 이상이면 중단.
        // 제어점 볼록포가 곡선을 포함하므로 AABB 거리는 진하한이다.
        const std::size_t n = ctrl_.size();

        std::vector<double> lb(n);

        for (std::size_t i = 0; i < n; ++i)
        {
            const double dx =
                std::max({aabb_lo_[i].x - p.x, p.x - aabb_hi_[i].x, 0.0});

            const double dy =
                std::max({aabb_lo_[i].y - p.y, p.y - aabb_hi_[i].y, 0.0});

            lb[i] = std::hypot(dx, dy);
        }

        std::vector<std::size_t> order(n);

        std::iota(order.begin(), order.end(), std::size_t{0});

        std::sort(
            order.begin(), order.end(),
            [&lb](std::size_t a, std::size_t b)
            {
                return lb[a] < lb[b];
            });


        for (const std::size_t i : order)
        {
            if (lb[i] >= best.dist)
            {
                break;      // 진하한 -> 잔여 segment 배제 가능
            }

            const bezier::Nearest nr =
                bezier::nearestOnSeg(ctrl_[i], p);

            if (nr.dist < best.dist)
            {
                best.seg   = static_cast<int>(i);

                best.u     = nr.u;

                best.s     = sOf(best.seg, nr.u);

                best.dist  = nr.dist;

                best.valid = true;
            }
        }


        return best;
    }

    // ---------- 8.4 국소 window 추적 ----------

    // 주행 중 최근접점. state.valid=false 면 전역 탐색.
    //
    // window 반경(FWD/BACK)이 경로 최소 자기근접 거리보다 작아야
    // 반대편 분기 진입이 구조적으로 배제된다.
    TrackState nearest(const Point2 & p, const TrackState & state) const
    {
        if (ctrl_.empty())
        {
            TrackState out;

            out.valid = false;

            return out;
        }

        if (!state.valid)
        {
            return nearestGlobal(p);
        }


        TrackState best = state;

        best.dist  = std::numeric_limits<double>::infinity();

        best.valid = true;


        for (const int i : windowSegments(state.s))
        {
            const bezier::Nearest nr =
                bezier::nearestOnSeg(seg(i), p);

            if (nr.dist < best.dist)
            {
                best.seg  = i;

                best.u    = nr.u;

                best.s    = sOf(i, nr.u);

                best.dist = nr.dist;
            }
        }


        if (best.dist > GATE)
        {
            best.fails = state.fails + 1;

            if (best.fails >= FAIL_LIMIT)
            {
                TrackState out = nearestGlobal(p);

                out.fails = 0;

                return out;
            }
        }
        else
        {
            best.fails = 0;
        }


        return best;
    }

    // 호길이 window [s-BACK, s+FWD] 에 걸치는 segment index.
    std::vector<int> windowSegments(double s) const
    {
        std::vector<int> out;

        const double lo = s - BACK;

        const double hi = s + FWD;


        for (int i = 0; i < nseg(); ++i)
        {
            const double a = cum_s_[static_cast<std::size_t>(i)];

            const double b = cum_s_[static_cast<std::size_t>(i) + 1];

            if (closed_)
            {
                // 최소표현 거리로 겹침 판정
                if (deltaS(s, b) >= -BACK && deltaS(s, a) <= FWD)
                {
                    out.push_back(i);
                }
            }
            else if (b >= lo && a <= hi)
            {
                out.push_back(i);
            }
        }


        if (out.empty())
        {
            int i = 0;

            double u = 0.0;

            locate(s, i, u);

            out.push_back(i);
        }


        return out;
    }

    // ---------- 8.5 LookAhead ----------

    // 최근접점에서 경로를 따라 ld 전진한 호길이. 개곡선은 단부 clamp.
    double lookahead(double s, double ld) const
    {
        return wrapS(s + ld);
    }

    // ---------- 오차 ----------

    // (cross track error [cm], heading error [rad]). 좌측 +.
    void errors(
        double x,
        double y,
        double yaw,
        const TrackState & state,
        double & cte_out,
        double & head_err_out) const
    {
        const Ctrl & c = seg(state.seg);

        const Point2 pt = bezier::evalSeg(c, state.u);

        const double th = bezier::heading(c, state.u);

        cte_out = -std::sin(th) * (x - pt.x) + std::cos(th) * (y - pt.y);

        head_err_out = wrapPi(yaw - th);
    }

    // 개곡선 종점 도달 판정. 폐곡선은 끝이 없으므로 항상 false.
    bool finished(const TrackState & state, double tol) const
    {
        return !closed_ && (length() - state.s) <= tol;
    }

    // ---------- 곡률 ----------

    double kappaMax() const
    {
        double best = 0.0;

        for (const Ctrl & c : ctrl_)
        {
            best = std::max(best, bezier::kappaMaxExact(c));
        }

        return best;
    }

    // segment i 의 [u0, u1] 조각 제어점. de Casteljau 분할이라 정확.
    Ctrl subSegment(int i, double u0, double u1) const
    {
        if (u1 <= u0)
        {
            return seg(i);
        }

        Ctrl left;

        Ctrl right;

        bezier::split(seg(i), u0, left, right);

        if (u1 >= 1.0 - 1e-12)
        {
            return right;
        }

        Ctrl l2;

        Ctrl r2;

        bezier::split(right, (u1 - u0) / (1.0 - u0), l2, r2);

        return l2;
    }

    // 호길이 구간 [s0, s1] 의 max|kappa|. Speed Controller 용.
    //
    // 구간을 de Casteljau 로 잘라낸 뒤 조각마다 14차 근 정확값 (문서 8.6).
    // 분할이 정확하므로 구간 밖 곡률이 섞이지 않는다.
    double kappaMaxOver(double s0, double s1) const
    {
        if (ctrl_.empty())
        {
            return 0.0;
        }

        double span = s1 - s0;

        if (!closed_)
        {
            s0 = std::min(std::max(s0, 0.0), length());

            s1 = std::min(std::max(s1, s0), length());

            span = s1 - s0;
        }

        if (span <= 0.0)
        {
            return std::abs(kappa(s0));
        }

        span = std::min(span, length());


        int cur = 0;

        double u_start = 0.0;

        locate(s0, cur, u_start);

        double best = 0.0;

        double remain = span;


        while (remain > 1e-9)
        {
            const double seg_rest = segLen(cur) * (1.0 - u_start);

            if (seg_rest >= remain)
            {
                const double u_end =
                    invertArclen(cur, segLen(cur) * u_start + remain);

                best = std::max(
                    best,
                    bezier::kappaMaxExact(subSegment(cur, u_start, u_end)));

                break;
            }

            best = std::max(
                best,
                bezier::kappaMaxExact(subSegment(cur, u_start, 1.0)));

            remain -= seg_rest;

            u_start = 0.0;

            ++cur;

            if (cur >= nseg())
            {
                if (!closed_)
                {
                    break;
                }

                cur = 0;
            }
        }


        return best;
    }

    // ---------- 검사 ----------

    bool isRegular() const
    {
        for (const Ctrl & c : ctrl_)
        {
            if (!bezier::isRegular(c))
            {
                return false;
            }
        }

        return true;
    }

    // 이음매 (theta, kappa) 불일치 최대값. 폐곡선은 wrap 포함.
    void g2Error(double & dtheta_out, double & dkappa_out) const
    {
        dtheta_out = 0.0;

        dkappa_out = 0.0;

        const int n = nseg();

        const int pairs = closed_ ? n : n - 1;


        for (int k = 0; k < pairs; ++k)
        {
            const int a = k;

            const int b = (k + 1) % n;

            dtheta_out = std::max(
                dtheta_out,
                std::abs(wrapPi(
                    bezier::heading(seg(a), 1.0) -
                    bezier::heading(seg(b), 0.0))));

            dkappa_out = std::max(
                dkappa_out,
                std::abs(
                    bezier::curvature(seg(a), 1.0) -
                    bezier::curvature(seg(b), 0.0)));
        }
    }

    // ---------- 시각화 전용 ----------

    // 표시용 폴리라인. 제어 연산에 사용 금지 (대전제).
    std::vector<Point2> sample(int per_seg = 40) const
    {
        std::vector<Point2> out;

        out.reserve(static_cast<std::size_t>(nseg()) *
                    static_cast<std::size_t>(per_seg));

        for (const Ctrl & c : ctrl_)
        {
            for (int k = 0; k < per_seg; ++k)
            {
                const double u =
                    per_seg > 1
                        ? static_cast<double>(k) / (per_seg - 1)
                        : 0.0;

                out.push_back(bezier::evalSeg(c, u));
            }
        }

        return out;
    }

private:
    void rebuild()
    {
        const std::size_t n = ctrl_.size();

        seg_len_.assign(n, 0.0);

        cum_s_.assign(n + 1, 0.0);

        aabb_lo_.assign(n, Point2{});

        aabb_hi_.assign(n, Point2{});


        for (std::size_t i = 0; i < n; ++i)
        {
            seg_len_[i] = bezier::segLength(ctrl_[i]);

            cum_s_[i + 1] = cum_s_[i] + seg_len_[i];

            double lo_x = ctrl_[i][0].x;

            double lo_y = ctrl_[i][0].y;

            double hi_x = lo_x;

            double hi_y = lo_y;

            for (const Point2 & q : ctrl_[i])
            {
                lo_x = std::min(lo_x, q.x);

                lo_y = std::min(lo_y, q.y);

                hi_x = std::max(hi_x, q.x);

                hi_y = std::max(hi_y, q.y);
            }

            aabb_lo_[i] = Point2{lo_x, lo_y};

            aabb_hi_[i] = Point2{hi_x, hi_y};
        }
    }

    std::vector<Ctrl>   ctrl_;

    bool                closed_ = false;

    std::vector<double> seg_len_;

    std::vector<double> cum_s_;      // 길이 nseg+1

    std::vector<Point2> aabb_lo_;

    std::vector<Point2> aabb_hi_;
};

}  // namespace control_lane
}  // namespace kau

#endif  // KAU_CONTROL_LANE__CURVE_HPP_
