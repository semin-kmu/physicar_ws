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
