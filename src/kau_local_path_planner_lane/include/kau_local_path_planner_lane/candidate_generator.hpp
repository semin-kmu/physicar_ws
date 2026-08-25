// ====================================================================
// candidate_generator.hpp
//
// 후보 quintic Bezier 경로 생성 -- 기본 corridor 7개 + obstacle_offset
// (Smart K1) 2개 + 실패 시 fallback (obstacle_primitives, direct_family).
//
// 원본: KAU_AMET_Test / src/kau_local_path_planner_lane/test/planner.py 의
//       LocalPlanner._candidate / _corridor_offsets / _obstacle_offset_
//       candidates / _obstacle_primitives / _direct_family /
//       _direct_target_indices / _direct_candidate / _primitive_candidate,
//       그리고 fit_segment / _station (module 함수)
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
//
// 2026-08-24 KAU_AMET_Test 세션 알고리즘 반영 (이번 포팅 세션):
//   - corridor 가 2-frame 에서 3-frame 으로 확장됐다 -- 중간 knot 이 하나
//     늘었다 (현재 위치는 아래 `corridorOffsets` 주석 참고). 긴 2-segment
//     chord 가 도로 굴곡을 한 번에 가로질러 road_boundary 를 자주 위반하던
//     문제 완화다 (실측: road-valid 전멸 100->45).
//     `corridorOffsets`/`candidate()` 를 N-frame(가변 길이)
//     지원하도록 일반화했다 -- Python 의 `zip(*offsets)`/`zip(offsets,
//     frames)` 제네릭 패턴과 동일 원리. obstacle_offsets/primitives/
//     direct_family/K2 는 기존 2-frame 그대로 (corridor 만 변경).
//   - `_obstacle_offset_candidates`("Smart K1") 를 새로 포팅했다: 장애물
//     회피 시 middle knot(K1) 의 longitudinal 위치를 장애물 station 기준
//     lead/ratio 4-combo 격자(2026-08-24 세션에 6->4 로 축소, 계산량
//     -33% 이면서 결과 동일함을 실측 확인)로 탐색해 hard-check 통과하는
//     cost 최소 후보를 고른다. corridor 7개 + 이 2개 = 9개로 여전히
//     "새 candidate family" 가 아니라 기존 후보군에 합류하는 것.
// ====================================================================

#ifndef KAU_LOCAL_PATH_PLANNER_LANE__CANDIDATE_GENERATOR_HPP_
#define KAU_LOCAL_PATH_PLANNER_LANE__CANDIDATE_GENERATOR_HPP_

#include <array>
#include <optional>
#include <utility>
#include <vector>

#include "kau_control/curve.hpp"
#include "kau_local_path_planner_lane/bezier_ext.hpp"
#include "kau_local_path_planner_lane/boundary_checker.hpp"
#include "kau_local_path_planner_lane/collision_checker.hpp"
#include "kau_local_path_planner_lane/types.hpp"

namespace kau
{
namespace local_path_planner_lane
{

using kau::control::Curve;
using kau::bezier::Ctrl;

// Python: CORRIDOR_FRACTIONS
inline constexpr std::array<double, 7> kCorridorFractions{
    0.0, 0.1625, 0.3875, 0.50, 0.6125, 0.72, 1.0};

inline constexpr double kRoadSafetyMarginCm = 2.0;
inline constexpr double kRoadSampleIntervalCm = 4.0;

// 2026-08-24 (committed/prediction horizon, KAU_AMET_Test 알고리즘 반영):
// 1-lap dynamic+EMA 실측(`diagnose_receding_horizon.py`) 결과, 실제
// replanning 주기당 ego 이동거리는 mean=14.2cm, p95=21.5cm, max=26.211cm
// 였다(plan_hz=5Hz). 이 max 값의 2배(재계획이 한 사이클 지연/스킵돼도
// 커버)를 "반드시 안전해야 하는" committed horizon 으로 삼는다 -- 임의
// 상수가 아니라 실측값 기반. 같은 진단에서 degraded 148개 중 84개
// (56.8%)가 "실행 구간(committed horizon)은 완전 valid, 400cm 끝의 먼
// 미래 구간 때문에만 전체 invalid"였다. committed horizon 안의 road/
// curvature 위반은 그대로 hard reject 하되, 그 이후(먼 미래, prediction
// 구간)에서만의 위반은 즉시 폐기하지 않고 cost 페널티만 부여한다.
// obstacle clearance 는 이 구분과 무관하게 항상 엄격하게 유지한다.
inline constexpr double kCommittedHorizonCm = 52.422;
inline constexpr double kPredictionViolationPenalty = 200.0;

// Python: _committed_segment_count(segs). `kCommittedHorizonCm` 이내를
// 포함하는 앞쪽 segment 개수(segment 단위 근사 -- station 이 horizon 을
// 살짝 넘는 segment 하나까지 통째로 포함시켜, 정밀 절단 없이도 committed
// 판정 범위가 항상 실제보다 넓거나 같도록 보수적으로 정의한다). 최소 1.
int committedSegmentCount(const std::vector<Ctrl> & segs);

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
        double kappa_max_vehicle, double body_radius_cm,
        VehicleFootprint body_footprint = VehicleFootprint{},
        WheelFootprint wheels = WheelFootprint{});

    // 참조 멤버(global_path_ 등)를 들고 있어 복사/이동하면 댕글링된다.
    CandidateGenerator(const CandidateGenerator &) = delete;
    CandidateGenerator & operator=(const CandidateGenerator &) = delete;
    CandidateGenerator(CandidateGenerator &&) = delete;
    CandidateGenerator & operator=(CandidateGenerator &&) = delete;

    // 2026-08-25 (P0 앵커): P0 를 이전 경로에 앵커하면 발행 경로가 자차에서
    // 최대 이만큼 떨어져 있다. road/obstacle 검사는 **경로를 따라** 수행되므로
    // 그대로 두면 마진이 그만큼 잠식된다 (kRoadSafetyMarginCm 은 2cm 뿐이다).
    // 그래서 이 값을 마진에 더해 "계획에서 벗어나 있는 만큼 더 보수적으로"
    // 검사한다. 게이트뿐 아니라 corridor 폭/장애물 회피 목표값에도 같이
    // 적용해야 한다 -- 게이트만 조이면 후보가 옛 마진에 딱 붙어 생성돼
    // 전멸한다. LocalPlanner::plan() 이 매 틱 설정한다.
    void setAnchorOffset(double offset_cm)
    {
        anchor_offset_cm_ = offset_cm > 0.0 ? offset_cm : 0.0;
    }

    // Python: _corridor_offsets(frames), N-frame 일반화(2026-08-24).
    // frames 는 LocalPlanner::plan() 이 0.6 / 0.8 / 1.0 * l_plan 에서 뜬다
    // (l_plan=180 이면 108 / 144 / 180cm -- 셋 다 lane 관측 168cm 안쪽).
    // 반환은 7개 (offset_at_frame0, offset_at_frame1, offset_at_frame2) 쌍
    // -- 마지막 원소만 direct_family/direct_target_indices 가 terminal
    // 값으로 읽는다 (Python 의 pair[-1] 과 동일 원리).
    std::array<std::array<double, 3>, 7> corridorOffsets(
        const std::array<Frame, 3> & frames) const;

    // Python: _candidate(p, yaw, kappa0, frames, offsets). N-knot 일반형
    // (corridor 의 3-frame 호출에 쓰인다).
    Candidate candidate(
        const Point2 & p, double yaw, double kappa0,
        const std::vector<Frame> & frames, const std::vector<double> & offsets,
        double s0, const Curve * previous_path) const;

    // 2-frame 전용 오버로드 (obstacle_offsets/primitives 등 기존 경로).
    // 내부적으로 위 N-knot 버전에 위임한다.
    Candidate candidate(
        const Point2 & p, double yaw, double kappa0,
        const std::array<Frame, 2> & frames,
        const std::pair<double, double> & offsets,
        double s0, const Curve * previous_path) const;

    // Python: _obstacle_offset_candidates(p, yaw, kappa0, frames) --
    // "Smart K1". 가장 가까운 장애물의 station 기준 lead/ratio 4-combo
    // 격자로 middle knot(K1) longitudinal 위치를 정해 side(근접/원접)당
    // 완성 후보 1개씩, 총 2개(빈 벡터면 장애물이 preview 밖). corridor
    // 후보군에 합류할 뿐 새 candidate family 가 아니다.
    std::vector<Candidate> obstacleOffsetCandidates(
        const Point2 & p, double yaw, double kappa0,
        const std::array<Frame, 2> & frames, double s0,
        const Curve * previous_path) const;

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
        const std::array<std::array<double, 3>, 7> & offset_pairs,
        std::vector<Candidate> base_candidates, double s0,
        const Curve * previous_path);

private:
    // 앵커 오프셋을 반영한 실효 마진. 이 두 개만 쓰고 kRoadSafetyMarginCm /
    // params_.obs_margin 을 직접 쓰지 말 것 (마진이 갈리면 후보가 전멸한다).
    double roadMargin() const { return kRoadSafetyMarginCm + anchor_offset_cm_; }
    double obsMargin() const { return params_.obs_margin + anchor_offset_cm_; }

    Candidate primitiveCandidate(
        const std::vector<Knot> & knots, double terminal_offset, double peak,
        double s0, const Curve * previous_path) const;

    // Python 은 장애물이 preview 구간에 없으면 길이 1, 있으면 길이 3인
    // 튜플을 반환한다 -- 고정 배열 대신 vector 로 그 가변성을 그대로 둔다.
    std::vector<int> directTargetIndices(
        double s0,
        const std::array<std::array<double, 3>, 7> & offset_pairs) const;

    Candidate directCandidate(
        const Point2 & start, double yaw, double kappa0,
        const Point2 & terminal, double reference_heading, double target,
        double s0, const Curve * previous_path);

    const Curve * global_path_;
    const RoadBoundary * boundary_;
    const std::vector<Obstacle> & obstacles_;
    const std::vector<ObstacleStation> & stations_;
    const PlannerParams & params_;
    double kappa_lim_;
    double kappa_max_vehicle_;
    double body_radius_cm_;
    VehicleFootprint body_footprint_;   // 장애물 충돌 판정 (범퍼 포함)
    WheelFootprint   wheels_;           // 도로 이탈 판정 (바퀴 4개)

    // cm, 발행 경로와 자차의 최대 이격 (= (1-alpha)*e). setAnchorOffset() 참조.
    double anchor_offset_cm_ = 0.0;

    // Python: self._direct_seed -- 직전 성공 direct_candidate 조합 warm-start.
    mutable std::optional<std::array<double, 4>> direct_seed_;
};

}  // namespace local_path_planner_lane
}  // namespace kau

#endif  // KAU_LOCAL_PATH_PLANNER_LANE__CANDIDATE_GENERATOR_HPP_
