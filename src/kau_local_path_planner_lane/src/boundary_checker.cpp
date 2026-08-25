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
    // nearestGlobal 이 (seg, u) 를 이미 확정해 줬는데 point(st.s) 는 그걸
    // 호길이로 되돌린 뒤 locate()+뉴턴으로 다시 (seg, u) 를 푼다. 왕복을
    // 없애고 제어점에서 바로 평가한다 -- 같은 점이고(실측 차이 1e-10 cm),
    // 역산 오차가 없는 만큼 오히려 정확하다.
    const Point2 q = kau::bezier::evalSeg(edge->seg(st.seg), st.u);
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

// ------------------------------------------------------------------
// 2026-08-25 (성능 응급수정): roadReportWheels 가 바퀴 코너(station 당 최대
// 8개) 마다 station 별 여유 -> marginAlongNormal ->
// edge->nearestGlobal 을 **독립적으로** 호출해, plan() 한 번에 수만 번의
// nearestGlobal(내부적으로 다항식 실근 탐색)이 실행됐다 (실측: 경로 1개
// 생성에 18초!). polygon+공간인덱스 기반이던 옛 boundary_checker 는 이
// 문제가 없었는데(인덱스가 O(1) 근사), edge Curve 기반으로 바꾸면서
// "코너마다 재탐색"을 그대로 옮긴 게 원인이다.
//
// 고정: station 의 **중심점 1개만** 좌/우 edge 에 최근접 투영(nearestGlobal
// 2회, edge 당 1회)하고, 바퀴 코너들은 그 결과 위에서 순수 벡터 연산
// (내적)으로 얻는다 -- KAU_AMET_Test Python 세션에서 이미 검증된
// "centerline 점 1개만 탐색 + 나머지는 평행이동" 원리와 동일. 코너가
// 중심에서 최대 ~23cm 떨어져 있을 뿐이라 근사 오차는 무시할 수준이고,
// nearestGlobal 호출이 station 당 8~16회 -> 2회로 줄어든다.
// ------------------------------------------------------------------

struct EdgeProjection
{
    std::optional<Point2> left_pt;
    std::optional<Point2> right_pt;
};

EdgeProjection projectStation(const RoadBoundary & boundary, const Point2 & center)
{
    EdgeProjection proj;
    if (boundary.left.has_value() && boundary.left->nseg() > 0)
    {
        const kau::control::TrackState st = boundary.left->nearestGlobal(center);
        if (st.valid)
        {
            // edgeMargin 과 같은 이유로 point(st.s) 왕복을 쓰지 않는다.
            proj.left_pt = kau::bezier::evalSeg(boundary.left->seg(st.seg), st.u);
        }
    }
    if (boundary.right.has_value() && boundary.right->nseg() > 0)
    {
        const kau::control::TrackState st = boundary.right->nearestGlobal(center);
        if (st.valid)
        {
            proj.right_pt = kau::bezier::evalSeg(boundary.right->seg(st.seg), st.u);
        }
    }
    return proj;
}

double marginFromProjection(
    const EdgeProjection & proj, const Point2 & point, const Point2 & normal,
    double sign, double footprint_cm, double max_search_cm = 200.0)
{
    double d;
    if (sign > 0.0)
    {
        if (proj.left_pt)
        {
            d = (proj.left_pt->x - point.x) * normal.x + (proj.left_pt->y - point.y) * normal.y;
        }
        else if (proj.right_pt)
        {
            const double r = -((proj.right_pt->x - point.x) * normal.x
                               + (proj.right_pt->y - point.y) * normal.y);
            d = kLaneWidthCm - r;
        }
        else
        {
            d = max_search_cm;
        }
    }
    else
    {
        if (proj.right_pt)
        {
            d = -((proj.right_pt->x - point.x) * normal.x
                 + (proj.right_pt->y - point.y) * normal.y);
        }
        else if (proj.left_pt)
        {
            const double l = (proj.left_pt->x - point.x) * normal.x
                            + (proj.left_pt->y - point.y) * normal.y;
            d = kLaneWidthCm - l;
        }
        else
        {
            d = max_search_cm;
        }
    }
    // 판정용 규약 -- 상한만 자르고 **부호는 살린다.** 여기서 하한 0 clamp 를
    // 걸면(= marginAlongNormal 의 폭 질의 규약) 여유가 절대 음수가 될 수 없어
    // 도로 이탈 판정이 통째로 무력해진다 (2026-08-25 실측: 경로를 도로 밖
    // 50m 로 보내도 min_wheels_on=4, off_integral=0, roadOkWheels=통과).
    //
    // 부호 규약: 좌 edge 는 +normal 쪽, 우 edge 는 -normal 쪽에 있다고 보고
    // 각각 "안쪽이면 양수" 가 되게 잰다. 도로 안이면 둘 다 양수, 왼쪽으로
    // 벗어나면 left 항이 음수가 된다.
    d = std::min(d, max_search_cm);
    return d - footprint_cm;
}

double stationClearanceFromProjection(
    const EdgeProjection & proj, const Point2 & point, const Point2 & normal,
    double footprint_cm)
{
    return std::min(
        marginFromProjection(proj, point, normal, 1.0, footprint_cm),
        marginFromProjection(proj, point, normal, -1.0, footprint_cm));
}

}  // namespace

RoadReport roadReportWheels(
    const RoadBoundary & boundary, const Curve & cv, const WheelFootprint & wheels,
    double road_safety_margin_cm, double sample_interval_cm, double s_max_cm,
    double report_horizon_cm)
{
    RoadReport rep;
    bool reached_limit = false;
    bool past_horizon = false;
    rep.min_clear_cm = std::numeric_limits<double>::infinity();
    rep.min_clear_upto_cm = std::numeric_limits<double>::infinity();

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
            const Point2 normal{-std::sin(heading), std::cos(heading)};
            const EdgeProjection proj = projectStation(boundary, center);

            // 이 스테이션이 report 구간에 드는가. 아래 s_max_cm 절단과 같은
            // 규약이다 -- 경계를 넘긴 첫 스테이션까지는 포함한다(그래야
            // s_max_cm 로 끊어 두 번 도는 것과 값이 정확히 일치한다).
            const double s_here = station + u * seg_len;
            const bool in_report_span = !past_horizon;

            int wheels_on = 0;
            double station_min_clear = std::numeric_limits<double>::infinity();
            for (double wx : {x_rear, x_front})
            {
                for (double sy : {-1.0, 1.0})
                {
                    station_min_clear = std::min(
                        station_min_clear,
                        stationClearanceFromProjection(
                            proj, corner(wx, sy * y_outer), normal, road_safety_margin_cm));
                    if (stationClearanceFromProjection(
                            proj, corner(wx, sy * y_inner), normal, road_safety_margin_cm) >= 0.0)
                    {
                        ++wheels_on;
                    }
                }
            }

            rep.min_clear_cm = std::min(rep.min_clear_cm, station_min_clear);
            if (in_report_span)
            {
                rep.min_clear_upto_cm =
                    std::min(rep.min_clear_upto_cm, station_min_clear);
            }
            if (s_here >= report_horizon_cm)
            {
                past_horizon = true;
            }

            rep.min_wheels_on = std::min(rep.min_wheels_on, wheels_on);
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
    if (!std::isfinite(rep.min_clear_upto_cm))
    {
        rep.min_clear_upto_cm = 0.0;
    }
    return rep;
}

}  // namespace local_path_planner_lane
}  // namespace kau
