// ====================================================================
// candidate_generator.hpp
//
// 후보 quintic Bezier 경로 생성 -- 기본 corridor 7개 + 실패 시 fallback
// (obstacle_primitives, direct_family).
//
// 원본: KAU_AMET_Test / src/kau_local_path_planner/test/planner.py 의
//       LocalPlanner._candidate / _corridor_offsets / _obstacle_primitives
//       / _direct_family / _direct_target_indices / _direct_candidate /
//       _primitive_candidate, 그리고 fit_segment / _station (module 함수)
//
// 차이점 (boundary_checker 재설계에 따른 것, KAU_AMET_ROS 세션 결정):
//   - Python 은 corridor 폭을 track.py 의 divider-relative 사전계산으로
//     얻었다. 우리는 그 divider 정렬 데이터가 없어 point-in-polygon+거리
//     기반으로 재설계했고 (boundary_checker.hpp), corridor offset 은
//     기준점에서 법선 방향 이분탐색(marginAlongNormal)으로 구한다.
//   - _direct_target_indices 의 "장애물이 어느 쪽에 있는가" 판정은 원래
//     divider 기준 절대 lateral 좌표(track.road_coordinates)를 다시 쓰는데,
//     우리는 이미 station 계산 시점에 구해둔 global-path 기준 lateral
//     (ObstacleStation.lateral, 원본의 _station() 과 동일 정의) 을 그대로
//     재사용한다 -- 어느 쪽이든 "부호(좌/우)"만 쓰므로 실질적으로 동등하다.
// ====================================================================

#ifndef KAU_LOCAL_PATH_PLANNER__CANDIDATE_GENERATOR_HPP_
#define KAU_LOCAL_PATH_PLANNER__CANDIDATE_GENERATOR_HPP_

#include <array>
#include <optional>
#include <utility>
#include <vector>

#include "kau_control/curve.hpp"
#include "kau_local_path_planner/bezier_ext.hpp"
#include "kau_local_path_planner/boundary_checker.hpp"
#include "kau_local_path_planner/collision_checker.hpp"
#include "kau_local_path_planner/types.hpp"

namespace kau
{
namespace local_path_planner
{

using kau::control::Curve;
using kau::bezier::Ctrl;

// Python: CORRIDOR_FRACTIONS
inline constexpr std::array<double, 7> kCorridorFractions{
    0.0, 0.1625, 0.3875, 0.50, 0.6125, 0.72, 1.0};

inline constexpr double kRoadSafetyMarginCm = 2.0;
inline constexpr double kRoadSampleIntervalCm = 4.0;
inline constexpr double kEndRatio = 0.80;

// knot 1 개 (위치, heading, 곡률). Python: curve.Knot.
struct Knot
{
    Point2 p;
    double theta = 0.0;
    double kappa = 0.0;
};

// Python: fit_segment(a, b, depth). sigma 를 chord 배수 3점 격자로 탐색해
// max|kappa| 최소인 제어점을 고른다 (fit_sigma 의 golden-section 대비 저렴).
// 전 후보가 퇴화면 (nullopt, inf).
std::pair<std::optional<Ctrl>, double> fitSegment(
    const Knot & a, const Knot & b, int bound_depth = 2);

// Python: _station(gp, obstacle). 장애물의 (reference 호길이, 횡위치).
// obstacle.center 는 이미 cm/map frame 이어야 한다.
ObstacleStation makeStation(const Curve & global_path, const Obstacle & obstacle);

class CandidateGenerator
{
public:
    CandidateGenerator(
        const Curve & global_path, const RoadBoundary & boundary,
        const std::vector<Obstacle> & obstacles,
        const std::vector<ObstacleStation> & stations,
        const PlannerParams & params, double kappa_lim,
        double kappa_max_vehicle, double body_radius_cm);

    // 참조 멤버(global_path_ 등)를 들고 있어 복사/이동하면 댕글링된다.
    CandidateGenerator(const CandidateGenerator &) = delete;
    CandidateGenerator & operator=(const CandidateGenerator &) = delete;
    CandidateGenerator(CandidateGenerator &&) = delete;
    CandidateGenerator & operator=(CandidateGenerator &&) = delete;

    // Python: _corridor_offsets(frames). frames[0]=0.25*l_plan 지점,
    // frames[1]=l_plan(종점) 지점. 반환은 7개 (offset0, offset1) 쌍.
    std::array<std::pair<double, double>, 7> corridorOffsets(
        const std::array<Frame, 2> & frames) const;

    // Python: _candidate(p, yaw, kappa0, frames, offsets).
    Candidate candidate(
        const Point2 & p, double yaw, double kappa0,
        const std::array<Frame, 2> & frames,
        const std::pair<double, double> & offsets,
        double s0, const Curve * previous_path) const;

    // Python: _obstacle_primitives(start, yaw, kappa0, terminal_frame).
    // 기본 7개가 전멸했을 때만 호출. 가장 가까운 장애물 앞에 apex knot 을
    // 두고 좌/우 두 후보를 만든다.
    std::vector<Candidate> obstaclePrimitives(
        const Point2 & start, double yaw, double kappa0,
        const Frame & terminal_frame, double s0,
        const Curve * previous_path) const;

    // Python: _direct_family(p, yaw, kappa0, terminal_frame, offset_pairs,
    // base_candidates). obstacle_primitives 도 전멸했을 때만 호출.
    std::vector<Candidate> directFamily(
        const Point2 & p, double yaw, double kappa0,
        const Frame & terminal_frame,
        const std::array<std::pair<double, double>, 7> & offset_pairs,
        std::vector<Candidate> base_candidates, double s0,
        const Curve * previous_path);

private:
    Candidate primitiveCandidate(
        const std::vector<Knot> & knots, double terminal_offset, double peak,
        double s0, const Curve * previous_path) const;

    // Python 은 장애물이 preview 구간에 없으면 길이 1, 있으면 길이 3인
    // 튜플을 반환한다 -- 고정 배열 대신 vector 로 그 가변성을 그대로 둔다.
    std::vector<int> directTargetIndices(
        double s0,
        const std::array<std::pair<double, double>, 7> & offset_pairs) const;

    Candidate directCandidate(
        const Point2 & start, double yaw, double kappa0,
        const Point2 & terminal, double reference_heading, double target,
        double s0, const Curve * previous_path);

    const Curve & global_path_;
    const RoadBoundary & boundary_;
    const std::vector<Obstacle> & obstacles_;
    const std::vector<ObstacleStation> & stations_;
    const PlannerParams & params_;
    double kappa_lim_;
    double kappa_max_vehicle_;
    double body_radius_cm_;

    // Python: self._direct_seed -- 직전 성공 direct_candidate 조합 warm-start.
    mutable std::optional<std::array<double, 4>> direct_seed_;
};

}  // namespace local_path_planner
}  // namespace kau

#endif  // KAU_LOCAL_PATH_PLANNER__CANDIDATE_GENERATOR_HPP_
