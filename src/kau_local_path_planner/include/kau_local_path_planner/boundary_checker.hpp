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

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "kau_control/curve.hpp"
#include "kau_local_path_planner/types.hpp"

namespace kau
{
namespace local_path_planner
{

using kau::bezier::Point2;
using kau::control::Curve;

// pointClearance 용 공간 인덱스. 구현은 boundary_checker.cpp 안에만 있다
// (호출자는 존재를 알 필요가 없다 -- RoadBoundary 를 그냥 값으로 들고 다니면
// 인덱스도 shared_ptr 로 따라온다).
//
// ★ 그래서 인덱스는 폴리곤을 **복사해서** 갖는다. 원본을 가리키기만 하면
//   복사본이 원본보다 오래 살 때 dangling 이 된다 (실제로 그 버그가 있었다.
//   boundary_checker.cpp 의 poly_ 선언 주석 참조).
struct BoundaryIndex;

// 드라이버블 링 = outer 내부 AND inner 외부. 둘 다 cm, map frame, 닫힌 폴리곤
// (마지막 점이 첫 점과 같지 않아도 되며, 내부에서 wrap 처리한다).
struct RoadBoundary
{
    std::vector<Point2> outer;
    std::vector<Point2> inner;

    // ------------------------------------------------------------------
    // 2026-08-25 (성능): pointClearance 가속용 공간 인덱스.
    //
    // 실측 배경 -- 실제 트랙(outer 388 / inner 388 정점)에서 pointClearance 는
    // 한 번 부를 때마다 776 개 선분에 거리 계산 + 두 번의 ray-cast 를 돌았다.
    // plan() 한 틱이 이 함수를 4,100~8,600 번 부르므로 (roadReportWheels 가
    // 경로 샘플마다 바퀴 4 개 x 2 점을 찍는다) 틱당 폴리곤 선분 연산이
    // 640 만~1,340 만 회였고, 그게 plan() 전체 시간의 약 95% 였다
    // (평균 26~50ms / 최대 210ms @ 10Hz. 예산 100ms 를 회피 상황에서 초과).
    //
    // 인덱스는 근사가 아니라 **후보 축소**다 -- 아래 ensureIndex() 주석의
    // 포함 증명 참조. 같은 입력에 같은 double 이 나오므로 게이트 판정이
    // 바뀌지 않는다 (200 틱 회귀에서 status/offset/kappa/clearance 전부
    // 비트 단위 동일 확인).
    //
    // 최초 질의 때 lazy 로 만들고 이후 재사용한다. makeRoadBoundary() 는
    // 미리 만들어 둔다 (첫 plan() 틱이 빌드 비용을 물지 않게).
    //
    // ★ outer/inner 를 직접 바꿨으면 resetIndex() 를 부를 것. 안 불러도
    //   정점 개수가 달라지면 자동으로 다시 만들지만, 개수가 같은 채로
    //   좌표만 바뀌면 낡은 인덱스를 쓰게 된다.
    mutable std::shared_ptr<const BoundaryIndex> index;

    void resetIndex() const { index.reset(); }
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

// 아래 둘은 인덱스를 쓰지 않는 전수 탐색 참조 구현이다. pointClearance 의
// 정답 기준(단위테스트)이자, 인덱스 격자 밖 질의의 폴백 경로다.
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

// ---------------------------------------------------------------------
// 2026-08-25: 3분할 원 근사(`roadClearance`, 경로 위 점 + 상수 반경) 대신
// 실제 차체(회전 사각형, 후륜축 기준)로 도로 경계를 검사한다.
//
// KAU_AMET_Test 세션에서 이 아이디어를 먼저 Python 으로 검증했다. 거기서는
// 회전 사각형의 4 꼭짓점을 각각 독립적으로 (divider 기준) nearest-point
// 재탐색에 넣었더니, 곡률이 큰 구간에서 꼭짓점(centerline 밖 오프셋 점)의
// 재탐색이 진짜 위치와 다른 arc-length 로 튀는 Frenet 근사 왜곡이 생겨
// vehicle boundary violation 실측이 오히려 악화됐다 (그래서 Python 쪽은
// "centerline 점만 1 번 탐색 + 사각형은 그 결과 위에 평행이동으로 얹는"
// 방식으로 우회했다).
//
// 여기(C++)는 그 문제가 원천적으로 없다: `pointClearance`/
// `distanceToPolygonBoundary` 가 임의의 (x,y) 점을 실제 폴리곤과 직접
// 비교하는 절대 기하 계산이라, Frenet 근사나 nearest-point 재탐색을 전혀
// 거치지 않는다. 그래서 사각형 4 꼭짓점을 그대로(정확한 heading 회전으로
// 계산해) 각각 `pointClearance` 에 넣으면 된다 -- 우회 없이 Python 대비
// 더 정확하고 더 단순한 구현.
// ---------------------------------------------------------------------
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

// ---------------------------------------------------------------------
// 2026-08-24: 도로 이탈 판정은 차체가 아니라 **바퀴** 로 한다.
//
// 대회 규정이 "흰 실선은 밟아도 되고, 네 바퀴가 전부 노면 밖으로 나가야
// 감점" 이다. 흰 실선의 바깥 모서리가 곧 아스팔트 끝이라 track 폴리곤이
// 그대로 규정상의 경계선이 된다 -- 폴리곤은 손대지 않는다.
//
// 위의 `roadClearanceRect`(차체 사각형)는 이 규정보다 훨씬 엄격하다. 차체
// 앞모서리는 base_footprint 에서 25.1cm 인데 앞바퀴 외측은 13.3cm 라, 코너
// 에서 12cm 바깥을 보고 멀쩡한 후보를 떨어뜨린다. 그래서 도로 게이트는 이
// 함수로 바꾸고, `roadClearanceRect` 는 차체 외곽이 필요한 곳(단위테스트,
// 장애물 계열과의 대조)에만 남긴다.
//
// 바퀴 하나를 두 점으로 본다:
//   - 바깥 모서리 (outerY): 여유 측정용. 음수면 그 바퀴가 경계를 물었다
//     (= 선을 밟기 시작). 규정상 감점은 아니라 거부 사유가 아니다.
//   - 안쪽 모서리 (innerY): 여기까지 밖이면 그 바퀴는 **완전히** 나갔다.
//     wheels_on 은 이 기준으로 센다.
// ---------------------------------------------------------------------

struct RoadReport
{
    int    min_wheels_on   = 4;     // 전 구간에서 가장 적게 남은 "노면 위" 바퀴 수
    double min_clear_cm    = 0.0;   // 바퀴 바깥 모서리 기준 최소 여유 (음수 = 물음)
    double first_viol_s_cm = -1.0;  // 바퀴가 처음 완전히 나간 호길이. 없으면 음수
    double off_integral_cm = 0.0;   // ∫ (4 - wheels_on)/4 ds. 비용/진단용
};

// s_max_cm: 검사할 호길이 상한. 기본값(무한)이면 곡선 전체.
//
// 2026-08-25: 게이트를 "검증 구간"으로 좁히기 위해 추가했다. 예전에는 곡선을
// 세그먼트 단위로 잘라 새 Curve 를 만들어 넘겼는데, 세그먼트가 75/112.5/112.5cm
// 라 검증 구간이 75cm 아니면 187.5cm 둘 중 하나로만 떨어졌다 (중간값 불가).
// 여기서 호길이로 끊으면 임의 길이를 정확히 검사할 수 있고 Curve 생성도 없다.
//
// 경계 처리는 보수적이다 -- s_max 를 넘어서는 첫 샘플까지 포함하고 멈춘다.
// (u 가 호길이 매개변수가 아니라 station 이 근사값이므로, 모자라게 끊는 것보다
// 넘치게 끊는 쪽이 안전하다.)
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

}  // namespace local_path_planner
}  // namespace kau

#endif  // KAU_LOCAL_PATH_PLANNER__BOUNDARY_CHECKER_HPP_
