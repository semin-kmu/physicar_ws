// ====================================================================
// boundary_checker.hpp (lane-only 재설계, 2026-08-25)
//
// 도로 물리 경계를 map 폴리곤이 아니라 /lane/left, /lane/right(KauPath,
// base_link) 곡선 자체로 판정한다 (map/global path/localization 배제).
//
// 공개 API 는 두 개뿐이다:
//   marginAlongNormal  코리도 폭 질의 ("이 방향으로 얼마나 더 갈 수 있나").
//                      비음수 -- 이미 밖이면 0 이다.
//   roadReportWheels   이탈 판정 (바퀴 4점). 부호를 살린다 -- 밖이면 음수.
// 둘을 섞어 쓰지 말 것. 폭 질의의 0 clamp 를 판정에 쓰면 "여유가 절대
// 음수가 될 수 없어" 이탈 판정이 통째로 무력해진다 (2026-08-25 실측 사고).
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

struct RoadReport
{
    int    min_wheels_on   = 4;
    double min_clear_cm    = 0.0;
    double first_viol_s_cm = -1.0;
    double off_integral_cm = 0.0;

    // cm, report_horizon_cm 이내 구간만의 min_clear_cm. 진단이 "전 구간" 과
    // "committed 구간" 두 값을 같이 쓰는데, 후자를 얻으려고 같은 스캔을 한
    // 번 더 도는 게 비쌌다 (스테이션당 곡선 최근접 투영 16 회). 한 번의
    // 스캔에서 접두 구간 최솟값을 같이 누적한다 -- 전체 스캔의 접두부라
    // 값은 두 번 도는 것과 정확히 같다.
    double min_clear_upto_cm = 0.0;
};

// report_horizon_cm: min_clear_upto_cm 을 채울 구간 상한. 기본(무한대) 이면
// min_clear_upto_cm == min_clear_cm 이다. s_max_cm(스캔 자체를 끊는 값) 과
// 달리 계산량을 줄이지 않고 누적 대상만 가른다.
RoadReport roadReportWheels(
    const RoadBoundary & boundary, const Curve & cv, const WheelFootprint & wheels,
    double road_safety_margin_cm, double sample_interval_cm,
    double s_max_cm = std::numeric_limits<double>::infinity(),
    double report_horizon_cm = std::numeric_limits<double>::infinity());

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
