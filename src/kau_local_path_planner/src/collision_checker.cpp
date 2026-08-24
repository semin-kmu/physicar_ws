#include "kau_local_path_planner/collision_checker.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace kau
{
namespace local_path_planner
{

namespace
{

// 곡선 전체 제어점의 AABB (Python 의 cv.aabb_lo.min(axis=0)/aabb_hi.max(axis=0)
// 와 동일 -- per-segment AABB 들의 전체 min/max는 곡선 전체 AABB 와 같다).
void curveAabb(const Curve & cv, Point2 & lo_out, Point2 & hi_out)
{
    lo_out = Point2{std::numeric_limits<double>::infinity(),
                    std::numeric_limits<double>::infinity()};
    hi_out = Point2{-std::numeric_limits<double>::infinity(),
                    -std::numeric_limits<double>::infinity()};
    for (int i = 0; i < cv.nseg(); ++i)
    {
        for (const Point2 & q : cv.seg(i))
        {
            lo_out.x = std::min(lo_out.x, q.x);
            lo_out.y = std::min(lo_out.y, q.y);
            hi_out.x = std::max(hi_out.x, q.x);
            hi_out.y = std::max(hi_out.y, q.y);
        }
    }
}

}  // namespace

std::vector<Obstacle> obstaclesNear(
    const Curve & cv, const std::vector<Obstacle> & obstacles, double reach_cm)
{
    if (cv.empty())
    {
        return {};
    }
    Point2 lo, hi;
    curveAabb(cv, lo, hi);
    lo.x -= reach_cm; lo.y -= reach_cm;
    hi.x += reach_cm; hi.y += reach_cm;

    std::vector<Obstacle> out;
    for (const Obstacle & o : obstacles)
    {
        if (o.center.x >= lo.x && o.center.x <= hi.x &&
            o.center.y >= lo.y && o.center.y <= hi.y)
        {
            out.push_back(o);
        }
    }
    return out;
}

double minDistToPoint(const Curve & cv, const Point2 & p)
{
    double best = std::numeric_limits<double>::infinity();
    for (int i = 0; i < cv.nseg(); ++i)
    {
        best = std::min(best, kau::bezier::nearestOnSeg(cv.seg(i), p).dist);
    }
    return best;
}

double clearance(
    const Curve & cv, const std::vector<Obstacle> & obstacles,
    double body_radius_cm, double clear_target_cm, double clear_cap_cm)
{
    double max_radius = 0.0;
    for (const Obstacle & o : obstacles)
    {
        max_radius = std::max(max_radius, o.radius);
    }
    const double reach = body_radius_cm + clear_target_cm + max_radius;

    const std::vector<Obstacle> near = obstaclesNear(cv, obstacles, reach);
    if (near.empty())
    {
        return clear_cap_cm;
    }

    double best = std::numeric_limits<double>::infinity();
    for (const Obstacle & o : near)
    {
        best = std::min(
            best, minDistToPoint(cv, o.center) - o.radius - body_radius_cm);
    }
    return best;
}

double clearanceRect(
    const Curve & cv, const std::vector<Obstacle> & obstacles,
    const VehicleFootprint & body, double clear_target_cm,
    double sample_interval_cm, double clear_cap_cm)
{
    double max_radius = 0.0;
    for (const Obstacle & o : obstacles)
    {
        max_radius = std::max(max_radius, o.radius);
    }
    // 사각형 어느 꼭짓점에서도 station 중심까지의 최대거리 (근사와 무관한
    // 실제 대각선 절반) 를 prefilter reach 로 써서 놓치는 장애물이 없게 함.
    // rear_axle_offset 보정 후의 실제 종방향 최대 편차를 쓴다.
    const double body_reach = std::hypot(
        std::max(std::abs(body.frontEdge()), std::abs(body.rearEdge())),
        body.half_width_cm);
    const double reach = body_reach + clear_target_cm + max_radius;

    const std::vector<Obstacle> near = obstaclesNear(cv, obstacles, reach);
    if (near.empty())
    {
        return clear_cap_cm;
    }

    double best = std::numeric_limits<double>::infinity();
    const int nseg = cv.nseg();
    for (int i = 0; i < nseg; ++i)
    {
        const int count = std::max(
            3, static_cast<int>(std::ceil(cv.segLen(i) / sample_interval_cm)) + 1);
        const bool include_end = (i == nseg - 1);
        const double step = include_end
            ? 1.0 / static_cast<double>(count - 1)
            : 1.0 / static_cast<double>(count);
        const kau::bezier::Ctrl & seg = cv.seg(i);
        const kau::bezier::Ctrl d1 = kau::bezier::hodograph(seg);
        for (int k = 0; k < count; ++k)
        {
            const double u = static_cast<double>(k) * step;
            const Point2 center = kau::bezier::evalSeg(seg, u);
            const Point2 tangent = kau::bezier::evalSeg(d1, u);
            const double heading = std::atan2(tangent.y, tangent.x);
            const double ch = std::cos(heading);
            const double sh = std::sin(heading);
            for (const Obstacle & o : near)
            {
                const double dx = o.center.x - center.x;
                const double dy = o.center.y - center.y;
                const double local_x = dx * ch + dy * sh;
                const double local_y = -dx * sh + dy * ch;
                const double clamped_x = std::clamp(
                    local_x, body.rearEdge(), body.frontEdge());
                const double clamped_y = std::clamp(
                    local_y, -body.half_width_cm, body.half_width_cm);
                const double dist = std::hypot(local_x - clamped_x, local_y - clamped_y);
                best = std::min(best, dist - o.radius);
            }
        }
    }
    return best;
}

double previewClear(
    const Curve & global_path, const std::vector<ObstacleStation> & stations,
    double d, double end_ratio, double s0, double l_plan, double preview,
    double body_radius_cm, double clear_cap_cm)
{
    if (stations.empty() || preview <= 0.0)
    {
        return clear_cap_cm;
    }
    const double end = s0 + l_plan;
    const double d_end = end_ratio * d;

    double out = clear_cap_cm;
    for (const ObstacleStation & st : stations)
    {
        const double advance = global_path.deltaS(end, st.station_s);
        if (advance >= 0.0 && advance <= preview)
        {
            out = std::min(
                out, std::abs(d_end - st.lateral) - st.obstacle.radius -
                         body_radius_cm);
        }
    }
    return out;
}

}  // namespace local_path_planner
}  // namespace kau
