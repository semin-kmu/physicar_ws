#include "kau_local_path_planner_lane/boundary_checker.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace kau
{
namespace local_path_planner_lane
{

namespace
{

// edge 에 대한 부호 있는 margin. sign=+1(left 방향) 이면 edge=left, sign=-1
// 이면 edge=right. edge 가 없으면 kLaneWidthCm 로 근사(반대쪽 edge 관측점을
// 기준으로 -- 없으면 origin 자체를 기준으로 절반 폭).
double edgeMargin(
    const std::optional<Curve> & edge, const Point2 & origin,
    const Point2 & normal, double sign, double max_search_cm)
{
    if (!edge.has_value() || edge->nseg() == 0)
    {
        return max_search_cm;
    }
    const kau::control::TrackState st = edge->nearestGlobal(origin);
    if (!st.valid)
    {
        return max_search_cm;
    }
    const Point2 q = edge->point(st.s);
    const double d = sign * ((q.x - origin.x) * normal.x + (q.y - origin.y) * normal.y);
    return d;
}

}  // namespace

double marginAlongNormal(
    const RoadBoundary & boundary, const Point2 & origin, const Point2 & normal,
    double sign, double footprint_cm, double max_search_cm)
{
    double d;
    if (sign > 0.0)
    {
        d = edgeMargin(boundary.left, origin, normal, 1.0, max_search_cm);
        if (!boundary.left.has_value())
        {
            // 반대쪽만 있으면 LANE_WIDTH 로 근사(Python _corridor_bounds 동일).
            const double r = edgeMargin(boundary.right, origin, normal, -1.0, max_search_cm);
            if (boundary.right.has_value())
            {
                d = kLaneWidthCm - r;
            }
        }
    }
    else
    {
        d = edgeMargin(boundary.right, origin, normal, -1.0, max_search_cm);
        if (!boundary.right.has_value())
        {
            const double l = edgeMargin(boundary.left, origin, normal, 1.0, max_search_cm);
            if (boundary.left.has_value())
            {
                d = kLaneWidthCm - l;
            }
        }
    }
    d = std::clamp(d, 0.0, max_search_cm);
    return std::max(0.0, d - footprint_cm);
}

namespace
{

// 한 station(위치+heading)에서 좌/우 edge 까지의 안전 여유 최솟값.
// margin 이 이미 footprint 를 뺀 값이므로 min 이 곧 그 station 의 clearance.
double stationClearance(
    const RoadBoundary & boundary, const Point2 & center, double heading,
    double footprint_cm)
{
    const Point2 normal{-std::sin(heading), std::cos(heading)};
    const double left_m = marginAlongNormal(
        boundary, center, normal, 1.0, footprint_cm, 200.0);
    const double right_m = marginAlongNormal(
        boundary, center, normal, -1.0, footprint_cm, 200.0);
    return std::min(left_m, right_m);
}

}  // namespace

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
            best = std::min(best, stationClearance(boundary, center, heading, footprint_cm));
        }
    }
    return std::isfinite(best) ? best : 0.0;
}

double roadClearanceRect(
    const RoadBoundary & boundary, const Curve & cv,
    const VehicleFootprint & body, double road_safety_margin_cm,
    double sample_interval_cm)
{
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
            for (double dx : {body.rearEdge(), body.frontEdge()})
            {
                for (double dy : {-body.half_width_cm, body.half_width_cm})
                {
                    const Point2 corner{
                        center.x + dx * ch - dy * sh,
                        center.y + dx * sh + dy * ch};
                    best = std::min(
                        best, stationClearance(boundary, corner, heading, road_safety_margin_cm));
                }
            }
        }
    }
    return std::isfinite(best) ? best : 0.0;
}

RoadReport roadReportWheels(
    const RoadBoundary & boundary, const Curve & cv, const WheelFootprint & wheels,
    double road_safety_margin_cm, double sample_interval_cm, double s_max_cm)
{
    RoadReport rep;
    bool reached_limit = false;
    rep.min_clear_cm = std::numeric_limits<double>::infinity();

    const double y_outer = wheels.outerY();
    const double y_inner = wheels.innerY();
    const double x_rear = wheels.rearAxleX();
    const double x_front = wheels.frontAxleX();

    double station = 0.0;
    const int nseg = cv.nseg();
    for (int i = 0; i < nseg; ++i)
    {
        const double seg_len = cv.segLen(i);
        const int count = std::max(
            3, static_cast<int>(std::ceil(seg_len / sample_interval_cm)) + 1);
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

            const auto corner = [&](double dx, double dy) -> Point2
            {
                return Point2{center.x + dx * ch - dy * sh,
                              center.y + dx * sh + dy * ch};
            };

            int wheels_on = 0;
            for (double wx : {x_rear, x_front})
            {
                for (double sy : {-1.0, 1.0})
                {
                    rep.min_clear_cm = std::min(
                        rep.min_clear_cm,
                        stationClearance(boundary, corner(wx, sy * y_outer), heading,
                                         road_safety_margin_cm));
                    if (stationClearance(boundary, corner(wx, sy * y_inner), heading,
                                         road_safety_margin_cm) >= 0.0)
                    {
                        ++wheels_on;
                    }
                }
            }

            rep.min_wheels_on = std::min(rep.min_wheels_on, wheels_on);
            const double s_here = station + u * seg_len;
            if (wheels_on < 4)
            {
                if (rep.first_viol_s_cm < 0.0)
                {
                    rep.first_viol_s_cm = s_here;
                }
                rep.off_integral_cm +=
                    static_cast<double>(4 - wheels_on) / 4.0 * step * seg_len;
            }
            if (s_here >= s_max_cm)
            {
                reached_limit = true;
                break;
            }
        }
        if (reached_limit)
        {
            break;
        }
        station += seg_len;
    }

    if (!std::isfinite(rep.min_clear_cm))
    {
        rep.min_clear_cm = 0.0;
    }
    return rep;
}

}  // namespace local_path_planner_lane
}  // namespace kau
