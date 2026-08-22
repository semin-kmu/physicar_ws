#include "kau_local_path_planner/boundary_checker.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "kau_control/bezier.hpp"

namespace kau
{
namespace local_path_planner
{

namespace
{

constexpr double kMPerCm = 0.01;

// 시뮬레이터 world(m) -> map(cm). kau_global_path 가 쓰는 변환과 동일
// (map_x = ox - sim_y, map_y = sim_x - oy), rot_deg=0 만 지원한다 -- 실제
// 배치가 회전 0 으로 확인됐고, 회전이 생기면 이 함수를 먼저 확장할 것.
Point2 simToMapCm(double sim_x_m, double sim_y_m, const SimToMap & t)
{
    if (std::abs(t.rot_deg) > 1e-9)
    {
        throw std::runtime_error(
            "boundary_checker: SimToMap.rot_deg != 0 은 아직 미구현");
    }
    const double map_x_m = t.ox - sim_y_m;
    const double map_y_m = sim_x_m - t.oy;
    return Point2{map_x_m / kMPerCm, map_y_m / kMPerCm};
}

double distanceToSegment(const Point2 & a, const Point2 & b, const Point2 & p)
{
    const double abx = b.x - a.x;
    const double aby = b.y - a.y;
    const double len2 = abx * abx + aby * aby;
    if (len2 <= 0.0)
    {
        return std::hypot(p.x - a.x, p.y - a.y);
    }
    double t = ((p.x - a.x) * abx + (p.y - a.y) * aby) / len2;
    t = std::clamp(t, 0.0, 1.0);
    const double cx = a.x + t * abx;
    const double cy = a.y + t * aby;
    return std::hypot(p.x - cx, p.y - cy);
}

}  // namespace

RoadBoundary makeRoadBoundary(
    const std::vector<double> & outer_x_m, const std::vector<double> & outer_y_m,
    const std::vector<double> & inner_x_m, const std::vector<double> & inner_y_m,
    const SimToMap & transform)
{
    RoadBoundary rb;

    if (outer_x_m.size() != outer_y_m.size() ||
        inner_x_m.size() != inner_y_m.size())
    {
        throw std::invalid_argument(
            "makeRoadBoundary: x/y 배열 길이 불일치");
    }

    rb.outer.reserve(outer_x_m.size());
    for (std::size_t i = 0; i < outer_x_m.size(); ++i)
    {
        rb.outer.push_back(simToMapCm(outer_x_m[i], outer_y_m[i], transform));
    }

    rb.inner.reserve(inner_x_m.size());
    for (std::size_t i = 0; i < inner_x_m.size(); ++i)
    {
        rb.inner.push_back(simToMapCm(inner_x_m[i], inner_y_m[i], transform));
    }

    return rb;
}

bool pointInPolygon(const std::vector<Point2> & polygon, const Point2 & p)
{
    if (polygon.size() < 3)
    {
        return false;
    }

    bool inside = false;
    const std::size_t n = polygon.size();
    for (std::size_t i = 0, prev = n - 1; i < n; prev = i++)
    {
        const Point2 & cur = polygon[i];
        const Point2 & pre = polygon[prev];

        const bool spans = (cur.y > p.y) != (pre.y > p.y);
        if (!spans)
        {
            continue;
        }
        const double dy = pre.y - cur.y;
        if (dy == 0.0)
        {
            continue;
        }
        const double crossing_x = cur.x + (p.y - cur.y) * (pre.x - cur.x) / dy;
        if (p.x < crossing_x)
        {
            inside = !inside;
        }
    }
    return inside;
}

double distanceToPolygonBoundary(
    const std::vector<Point2> & polygon, const Point2 & p)
{
    if (polygon.size() < 2)
    {
        return std::numeric_limits<double>::infinity();
    }
    double best = std::numeric_limits<double>::infinity();
    const std::size_t n = polygon.size();
    for (std::size_t i = 0; i < n; ++i)
    {
        const Point2 & a = polygon[i];
        const Point2 & b = polygon[(i + 1) % n];
        best = std::min(best, distanceToSegment(a, b, p));
    }
    return best;
}

double pointClearance(
    const RoadBoundary & boundary, const Point2 & p, double footprint_cm)
{
    const double outer_dist = distanceToPolygonBoundary(boundary.outer, p);
    const double outer_margin =
        pointInPolygon(boundary.outer, p) ? outer_dist : -outer_dist;

    const double inner_dist = distanceToPolygonBoundary(boundary.inner, p);
    const double inner_margin =
        !pointInPolygon(boundary.inner, p) ? inner_dist : -inner_dist;

    return std::min(outer_margin, inner_margin) - footprint_cm;
}

double marginAlongNormal(
    const RoadBoundary & boundary, const Point2 & origin, const Point2 & normal,
    double sign, double footprint_cm, double max_search_cm)
{
    const auto at = [&](double t) -> double
    {
        const Point2 p{origin.x + sign * t * normal.x,
                       origin.y + sign * t * normal.y};
        return pointClearance(boundary, p, footprint_cm);
    };

    // origin 자체가 이미 위반이면(예: 후보가 이미 경계 밖) 여유 0.
    if (at(0.0) < 0.0)
    {
        return 0.0;
    }

    // 단순 이분탐색은 clearance(t) 가 단조라고 가정하는데, inner 가 원형
    // 섬이 아니라 도로 폭보다 좁은 "구멍" 모양이면 안전 -> 위반 -> 안전으로
    // 두 번 부호가 바뀔 수 있다. 그 경우 이분탐색은 가까운 위반을 건너뛰고
    // 더 먼 경계를 오탐할 수 있다 (실제 gtest 로 발견). 그래서 먼저 성긴
    // step 으로 훑어 "가장 가까운" 부호 반전 구간을 찾고, 그 구간 안에서만
    // 이분탐색으로 정밀화한다 -- 이러면 단조 여부와 무관하게 항상 가장
    // 가까운 위반 지점을 찾는다.
    constexpr double kScanStep = 2.0;   // cm, 정밀도와 비용의 절충
    double prev_t = 0.0;
    double prev_clear = at(0.0);
    double lo = -1.0;
    double hi = -1.0;
    for (double t = kScanStep; t <= max_search_cm; t += kScanStep)
    {
        const double clear = at(t);
        if (clear < 0.0)
        {
            lo = prev_t;
            hi = t;
            break;
        }
        prev_t = t;
        prev_clear = clear;
    }
    (void)prev_clear;

    if (lo < 0.0)
    {
        return max_search_cm;   // 탐색 범위 안에서 위반 없음 -> 사실상 무제한
    }

    for (int it = 0; it < 24; ++it)
    {
        const double mid = 0.5 * (lo + hi);
        if (at(mid) >= 0.0)
        {
            lo = mid;
        }
        else
        {
            hi = mid;
        }
    }
    return lo;
}

double roadClearance(
    const RoadBoundary & boundary, const Curve & cv, double footprint_cm,
    double sample_interval_cm)
{
    double best = std::numeric_limits<double>::infinity();
    const int nseg = cv.nseg();
    for (int i = 0; i < nseg; ++i)
    {
        const int count = std::max(
            3, static_cast<int>(std::ceil(cv.segLen(i) / sample_interval_cm)) + 1);
        // Python: np.linspace(0, 1, count, endpoint=(i==nseg-1)) -- 마지막
        // segment 만 u=1.0 포함, 그 외엔 다음 segment 의 u=0 과 중복이라 제외.
        const bool include_end = (i == nseg - 1);
        const double step = include_end
            ? 1.0 / static_cast<double>(count - 1)
            : 1.0 / static_cast<double>(count);
        for (int k = 0; k < count; ++k)
        {
            const double u = static_cast<double>(k) * step;
            const Point2 point = kau::bezier::evalSeg(cv.seg(i), u);
            best = std::min(best, pointClearance(boundary, point, footprint_cm));
        }
    }
    return best;
}

}  // namespace local_path_planner
}  // namespace kau
