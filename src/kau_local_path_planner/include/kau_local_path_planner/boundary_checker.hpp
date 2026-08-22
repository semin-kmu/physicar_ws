// ====================================================================
// boundary_checker.hpp
//
// 실제 도로 inner/outer boundary hard constraint.
//
// 원본: KAU_AMET_Test / src/kau_local_path_planner/test/planner.py 의
//       LocalPlanner._road_ok / _road_clearance / _road_coordinates
//       (원본은 sim_common/track.py 의 divider-relative 사전계산 corridor
//       를 썼으나, 그 divider 정렬 데이터가 ROS2 쪽에는 없어 이 포팅에서는
//       point-in-polygon + 거리 방식으로 재설계했다 -- kau_object_detection
//       /include/kau_object_detection/track_geometry.hpp 와 동일한 기하
//       알고리즘을 그대로 재사용 (구현은 Point2 타입 차이로 이 파일에 복제).
//
// 경계 데이터 출처: kau_object_detection/config/amet_2026_track.yaml
// (track_outer_x/y, track_inner_x/y) -- 토픽 아님, 노드 시작 시 1회 로드.
//
// 주의: 이 yaml 좌표는 map frame 이 아니라 Gazebo 시뮬레이터 world
// 프레임(m)이다. TF(map->base_footprint) 기준 pose 와 비교하려면 로드
// 시점에 kau_global_path 의 sim->map 변환 + m->cm 변환을 적용해야 한다:
//     map_x = ox - sim_y,  map_y = sim_x - oy   (ox=3.68, oy=1.39, rot=0)
// ====================================================================

#ifndef KAU_LOCAL_PATH_PLANNER__BOUNDARY_CHECKER_HPP_
#define KAU_LOCAL_PATH_PLANNER__BOUNDARY_CHECKER_HPP_

#include <string>
#include <vector>

#include "kau_control/curve.hpp"

namespace kau
{
namespace local_path_planner
{

using kau::bezier::Point2;
using kau::control::Curve;

// 드라이버블 링 = outer 내부 AND inner 외부. 둘 다 cm, map frame, 닫힌 폴리곤
// (마지막 점이 첫 점과 같지 않아도 되며, 내부에서 wrap 처리한다).
struct RoadBoundary
{
    std::vector<Point2> outer;
    std::vector<Point2> inner;
};

struct SimToMap
{
    double ox = 3.68;         // m
    double oy = 1.39;         // m
    double rot_deg = 0.0;     // 현재 배치는 회전 없음 (kau_global_path 확인)
};

// amet_2026_track.yaml (ROS2 파라미터 형식) 을 읽어 RoadBoundary 로 변환.
// track_outer_x/y, track_inner_x/y 파라미터를 이미 declare/load 한 노드에서
// 그 값을 그대로 넘긴다 (파일 파싱 자체는 노드 launch 가 표준 파라미터
// 메커니즘으로 처리하므로 여기서는 좌표 변환만 담당).
RoadBoundary makeRoadBoundary(
    const std::vector<double> & outer_x_m, const std::vector<double> & outer_y_m,
    const std::vector<double> & inner_x_m, const std::vector<double> & inner_y_m,
    const SimToMap & transform);

// ---------------------------------------------------------------------
// 기하 primitive (kau_object_detection/track_geometry.cpp 와 동일 알고리즘)
// ---------------------------------------------------------------------

bool pointInPolygon(const std::vector<Point2> & polygon, const Point2 & p);

double distanceToPolygonBoundary(
    const std::vector<Point2> & polygon, const Point2 & p);

// ---------------------------------------------------------------------
// 한 점의 안전 여유 [cm]. 드라이버블 링 안쪽이면 +, 벗어나면 -.
// footprint 는 이미 반영된 상태로 반환 (즉 0 이 hard 한계).
// ---------------------------------------------------------------------
double pointClearance(
    const RoadBoundary & boundary, const Point2 & p, double footprint_cm);

// ---------------------------------------------------------------------
// origin 에서 normal 방향(부호 sign = +1.0 또는 -1.0)으로 이분탐색해
// pointClearance 가 0 이 되는 거리를 구한다. corridor 폭 계산용
// (Python _road_coordinates 의 lower/upper 대응, 단 origin 자신을
// 기준(=0)으로 측정하므로 progress.lateral_distance 보정이 불필요하다).
//
// max_search_cm 안에서 부호 반전을 못 찾으면(직선 도로 등 매우 넓은
// 경우) max_search_cm 을 그대로 반환한다 -- 실제 트랙 폭보다 훨씬 큰
// 값으로 두면 사실상 "이 방향은 제한 없음"과 동일하게 동작한다.
// ---------------------------------------------------------------------
double marginAlongNormal(
    const RoadBoundary & boundary, const Point2 & origin, const Point2 & normal,
    double sign, double footprint_cm, double max_search_cm = 200.0);

// ---------------------------------------------------------------------
// 곡선 전체에서 안전 여유의 최솟값 [cm]. 음수면 침범.
// ---------------------------------------------------------------------
double roadClearance(
    const RoadBoundary & boundary, const Curve & cv, double footprint_cm,
    double sample_interval_cm);

inline bool roadOk(
    const RoadBoundary & boundary, const Curve & cv, double footprint_cm,
    double sample_interval_cm)
{
    return roadClearance(boundary, cv, footprint_cm, sample_interval_cm) >= 0.0;
}

}  // namespace local_path_planner
}  // namespace kau

#endif  // KAU_LOCAL_PATH_PLANNER__BOUNDARY_CHECKER_HPP_
