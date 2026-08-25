// ====================================================================
// collision_checker.hpp
//
// 장애물 충돌 판정 (문서 6.8). 원본: KAU_AMET_Test /
// src/kau_local_path_planner_lane/test/planner.py 의 LocalPlanner._clearance /
// _near / _preview_clear.
//
// 장애물은 /perception/obstacles (kau_msgs/ObstacleCircleArray, m 단위,
// 물리 반지름만 -- 차체/안전마진 미포함) 에서 온다. cm 로 변환하고
// body_radius_cm + obs_margin_cm 을 더하는 건 이 파일의 함수들이 맡는다
// (Python 의 self.body / self.p.obs_margin 위치와 동일).
// ====================================================================

#ifndef KAU_LOCAL_PATH_PLANNER_LANE__COLLISION_CHECKER_HPP_
#define KAU_LOCAL_PATH_PLANNER_LANE__COLLISION_CHECKER_HPP_

#include <vector>

#include "kau_control/curve.hpp"
#include "kau_local_path_planner_lane/types.hpp"

namespace kau
{
namespace local_path_planner_lane
{

using kau::bezier::Point2;
using kau::control::Curve;

// map frame, cm. radius 는 물리 반지름 (margin 미포함) -- ObstacleCircle 과 동일 규약.
struct Obstacle
{
    Point2 center;
    double radius = 0.0;
};

// Python: LocalPlanner._near. AABB(제어점 전체 bounding box) 사전 filtering
// 으로 reach 안에 들 수 있는 장애물만 추림.
std::vector<Obstacle> obstaclesNear(
    const Curve & cv, const std::vector<Obstacle> & obstacles, double reach_cm);

// Python: Curve.min_dist_to -- kau_control::Curve 에는 없어서 여기서
// 세그먼트별 bezier::nearestOnSeg 로 직접 구현.
double minDistToPoint(const Curve & cv, const Point2 & p);

// Python: LocalPlanner._clearance. 차체 반폭 포함 최소 여유 [cm]. 음수면 충돌.
// near 장애물이 없으면 clear_cap 반환 (근처에 장애물 없음을 나타내는 상한).
double clearance(
    const Curve & cv, const std::vector<Obstacle> & obstacles,
    double body_radius_cm, double clear_target_cm, double clear_cap_cm = 999.0);

// 2026-08-25: 3분할 원 근사(`clearance`) 대신 실제 차체(회전 사각형,
// 후륜축 기준)와 장애물(원) 간 최단거리. 각 station 에서 장애물 중심을
// heading 기준 로컬프레임으로 회전한 뒤 사각형 반폭으로 클램프하는 표준
// rectangle-point 최단거리 공식 (KAU_AMET_Test Python 포팅과 동일 원리).
double clearanceRect(
    const Curve & cv, const std::vector<Obstacle> & obstacles,
    const VehicleFootprint & body, double clear_target_cm,
    double sample_interval_cm, double clear_cap_cm = 999.0);

// Python: LocalPlanner._preview_clear. l_plan 종점 너머 preview 구간의
// 장애물 여유 [cm]. 비용에만 반영, hard gate 아님.
//
// stations: 장애물별 (reference 호길이, 횡위치) -- Python 의 self.stations,
// gp.delta_s/nearest_global 로 사전계산해서 전달한다 (매 후보마다 재계산할
// 필요 없음, 장애물이 바뀔 때만 갱신).
struct ObstacleStation
{
    double station_s = 0.0;      // reference(global path) 호길이
    double lateral    = 0.0;     // 그 지점 기준 횡위치 [cm]
    Obstacle obstacle;
};

double previewClear(
    const Curve & global_path, const std::vector<ObstacleStation> & stations,
    double d, double end_ratio, double s0, double l_plan, double preview,
    double body_radius_cm, double clear_cap_cm = 999.0);

}  // namespace local_path_planner_lane
}  // namespace kau

#endif  // KAU_LOCAL_PATH_PLANNER_LANE__COLLISION_CHECKER_HPP_
