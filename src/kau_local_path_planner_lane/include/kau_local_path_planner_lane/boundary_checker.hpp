// ====================================================================
// boundary_checker.hpp (lane-only 재설계, 2026-08-25)
//
// 도로 물리 경계를 map 폴리곤이 아니라 /lane/left, /lane/right(KauPath,
// base_link) 곡선 자체로 판정한다 (map/global path/localization 배제).
// 공개 API(marginAlongNormal/roadClearance*/roadReportWheels/roadOk*)는
// 옛 버전과 최대한 같게 유지해 candidate_generator/local_planner/
// path_evaluator 를 거의 안 건드리게 했다.
// ====================================================================

#ifndef KAU_LOCAL_PATH_PLANNER_LANE__BOUNDARY_CHECKER_HPP_
#define KAU_LOCAL_PATH_PLANNER_LANE__BOUNDARY_CHECKER_HPP_

#include <limits>
#include <optional>

#include "kau_control/curve.hpp"
#include "kau_local_path_planner_lane/types.hpp"

namespace kau
{
namespace local_path_planner_lane
{

using kau::bezier::Point2;
using kau::control::Curve;

// 검출된 좌/우 lane edge. 둘 다 base_link 상대, 없으면 nullopt(hold-last
// 만료 등). 한쪽만 있어도 동작한다(반대쪽 margin 은 LANE_WIDTH 근사).
struct RoadBoundary
{
    std::optional<Curve> left;
    std::optional<Curve> right;
};

inline constexpr double kLaneWidthCm = 70.0;   // cm, 한쪽 edge 만 있을 때 반대쪽 근사

// origin 에서 normal 방향(sign=+1 좌/-1 우)으로 얼마나 더 갈 수 있는지 [cm].
// edge 곡선에 대한 최근접 투영으로 **정확히** 계산한다(폴리곤 시절의 이분
// 탐색과 달리 반복 불필요). 그 방향 edge 가 없으면 max_search_cm 반환
// (제한 없음으로 취급, Python `track.LANE_WIDTH` 폴백과 동일 성격).
double marginAlongNormal(
    const RoadBoundary & boundary, const Point2 & origin, const Point2 & normal,
    double sign, double footprint_cm, double max_search_cm = 200.0);

// 곡선 전체에서 안전 여유의 최솟값 [cm]. 음수면 침범. 후보 자신의 접선을
// 법선 기준으로 써서 좌/우 edge 까지 거리를 잰다(3분할 원 근사, 옛 API 유지).
double roadClearance(
    const RoadBoundary & boundary, const Curve & cv, double footprint_cm,
    double sample_interval_cm);

inline bool roadOk(
    const RoadBoundary & boundary, const Curve & cv, double footprint_cm,
    double sample_interval_cm)
{
    return roadClearance(boundary, cv, footprint_cm, sample_interval_cm) >= 0.0;
}

// 회전 사각형 차체(후륜축 기준) 버전.
double roadClearanceRect(
    const RoadBoundary & boundary, const Curve & cv,
    const VehicleFootprint & body, double road_safety_margin_cm,
    double sample_interval_cm);

inline bool roadOkRect(
    const RoadBoundary & boundary, const Curve & cv,
    const VehicleFootprint & body, double road_safety_margin_cm,
    double sample_interval_cm)
{
    return roadClearanceRect(boundary, cv, body, road_safety_margin_cm,
                             sample_interval_cm) >= 0.0;
}

struct RoadReport
{
    int    min_wheels_on   = 4;
    double min_clear_cm    = 0.0;
    double first_viol_s_cm = -1.0;
    double off_integral_cm = 0.0;
};

RoadReport roadReportWheels(
    const RoadBoundary & boundary, const Curve & cv, const WheelFootprint & wheels,
    double road_safety_margin_cm, double sample_interval_cm,
    double s_max_cm = std::numeric_limits<double>::infinity());

inline bool roadOkWheels(
    const RoadBoundary & boundary, const Curve & cv, const WheelFootprint & wheels,
    double road_safety_margin_cm, double sample_interval_cm,
    double s_max_cm = std::numeric_limits<double>::infinity())
{
    return roadReportWheels(boundary, cv, wheels, road_safety_margin_cm,
                            sample_interval_cm, s_max_cm).min_wheels_on
           >= wheels.min_wheels_on;
}

}  // namespace local_path_planner_lane
}  // namespace kau

#endif  // KAU_LOCAL_PATH_PLANNER_LANE__BOUNDARY_CHECKER_HPP_
