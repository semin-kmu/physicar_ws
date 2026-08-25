#include "kau_local_path_planner/boundary_checker.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>

#include "kau_control/bezier.hpp"

namespace kau
{
namespace local_path_planner
{

namespace
{

constexpr double kMPerCm = 0.01;

// 시뮬레이터 world(m) -> map(cm). kau_global_path 가 쓰는 변환과 동일
// (map_x = ox - sim_y, map_y = sim_x - oy), rot_deg=0 만 지원한다 -- 실제
// 배치가 회전 0 으로 확인됐고, 회전이 생기면 이 함수를 먼저 확장할 것.
Point2 simToMapCm(double sim_x_m, double sim_y_m, const SimToMap & t)
{
    if (std::abs(t.rot_deg) > 1e-9)
    {
        throw std::runtime_error(
            "boundary_checker: SimToMap.rot_deg != 0 은 아직 미구현");
    }
    const double map_x_m = t.ox - sim_y_m;
    const double map_y_m = sim_x_m - t.oy;
    return Point2{map_x_m / kMPerCm, map_y_m / kMPerCm};
}

double distanceToSegment(const Point2 & a, const Point2 & b, const Point2 & p)
{
    const double abx = b.x - a.x;
    const double aby = b.y - a.y;
    const double len2 = abx * abx + aby * aby;
    if (len2 <= 0.0)
    {
        return std::hypot(p.x - a.x, p.y - a.y);
    }
    double t = ((p.x - a.x) * abx + (p.y - a.y) * aby) / len2;
    t = std::clamp(t, 0.0, 1.0);
    const double cx = a.x + t * abx;
    const double cy = a.y + t * aby;
    return std::hypot(p.x - cx, p.y - cy);
}

// ====================================================================
// 공간 인덱스 (2026-08-25 성능)
//
// 트랙 폴리곤은 노드 기동 시 1 회 로드하고 주행 내내 바뀌지 않는데,
// pointClearance 는 매 질의마다 776 개 선분 전체를 다시 훑고 있었다.
// 한 번 만들어 두고 질의마다 "볼 필요가 있는 변"만 남긴다.
//
// 두 질의가 필요로 하는 것이 달라 인덱스도 둘이다:
//   거리       -> 격자 셀별 후보 edge 리스트 (cells_)
//   안/밖 판정 -> y-버킷별 edge 리스트     (rows_)
//
// 어느 쪽도 근사가 아니다. 버리는 변은 "정답이 될 수 없음이 증명되는"
// 변뿐이라, 남은 후보에 대한 min/parity 는 전수 탐색과 같은 double 이
// 나온다 (아래 각 build 주석의 근거 참조).
// ====================================================================

class PolygonIndex
{
public:
    // 격자 셀 하나가 물고 있을 변 개수의 목표치. 셀을 잘게 쪼갤수록 후보는
    // 줄지만 빌드 비용/메모리가 는다. 실제 트랙(388 정점, 평균 변 길이
    // 8.3cm, 530x1022cm)에서 셀 25cm 근방이 후보 5~20 개로 떨어지는 지점.
    static constexpr double kTargetCellsPerEdge = 8.0;
    static constexpr double kMinCellCm  = 5.0;
    static constexpr std::size_t kMaxCells = 200000;

    // 격자 밖 질의는 전수 탐색으로 폴백한다. 경로가 트랙에서 이만큼 벗어나면
    // 어차피 후보로 살아남지 못하므로 성능상 의미가 없는 경로다.
    static constexpr double kPadCm = 100.0;

    // y-버킷 높이. 변 평균 길이와 같은 정도면 줄당 변이 몇 개로 떨어진다.
    static constexpr double kRowHeightCm = 10.0;

    bool valid() const { return n_ >= 3; }

    std::size_t vertexCount() const { return static_cast<std::size_t>(n_); }

    void build(const std::vector<Point2> & polygon)
    {
        poly_ = polygon;   // 복사본을 소유한다. 아래 poly_ 선언 주석 참조
        n_ = static_cast<int>(polygon.size());
        cells_.clear();
        rows_.clear();
        if (n_ < 3)
        {
            return;   // 폴리곤이라 부를 수 없는 입력 -- 폴백만 쓴다
        }

        double lo_x = kInfLocal, lo_y = kInfLocal;
        double hi_x = -kInfLocal, hi_y = -kInfLocal;
        for (const Point2 & q : polygon)
        {
            lo_x = std::min(lo_x, q.x); hi_x = std::max(hi_x, q.x);
            lo_y = std::min(lo_y, q.y); hi_y = std::max(hi_y, q.y);
        }
        x0_ = lo_x - kPadCm; y0_ = lo_y - kPadCm;
        const double w = (hi_x + kPadCm) - x0_;
        const double h = (hi_y + kPadCm) - y0_;

        // 셀 크기는 폴리곤 크기에 맞춰 정한다 (트랙이 바뀌어도 후보 개수가
        // 비슷하게 유지되도록). 셀 수 상한은 메모리 폭주 방지용.
        double target_cells = kTargetCellsPerEdge * static_cast<double>(n_);
        target_cells = std::min(target_cells, static_cast<double>(kMaxCells));
        cell_ = std::max(kMinCellCm, std::sqrt(w * h / target_cells));
        nx_ = std::max(1, static_cast<int>(std::ceil(w / cell_)));
        ny_ = std::max(1, static_cast<int>(std::ceil(h / cell_)));
        while (static_cast<std::size_t>(nx_) * static_cast<std::size_t>(ny_) >
               kMaxCells)
        {
            cell_ *= 2.0;
            nx_ = std::max(1, static_cast<int>(std::ceil(w / cell_)));
            ny_ = std::max(1, static_cast<int>(std::ceil(h / cell_)));
        }

        buildCells();
        buildRows(lo_y, hi_y);
    }

    // distanceToPolygonBoundary 와 같은 값. 격자 밖이면 전수 탐색.
    double distance(const Point2 & p) const
    {
        const std::vector<Point2> & poly = poly_;
        const int cx = static_cast<int>(std::floor((p.x - x0_) / cell_));
        const int cy = static_cast<int>(std::floor((p.y - y0_) / cell_));
        if (cx < 0 || cy < 0 || cx >= nx_ || cy >= ny_)
        {
            return distanceToPolygonBoundary(poly, p);
        }

        double best = kInfLocal;
        for (const int j : cells_[cellIndex(cx, cy)])
        {
            // 전수 탐색과 같은 (a, b) 순서로 넘겨야 같은 double 이 나온다.
            best = std::min(
                best, distanceToSegment(poly[j], poly[nextIndex(j)], p));
        }
        return best;
    }

    // pointInPolygon 과 같은 값.
    bool inside(const Point2 & p) const
    {
        const std::vector<Point2> & poly = poly_;
        const int r = static_cast<int>(std::floor((p.y - y0_) / kRowHeightCm));
        if (r < 0 || r >= nrow_)
        {
            return false;   // 폴리곤 y 범위 + pad 밖 -> 확실히 바깥
        }

        bool inside_flag = false;
        for (const int i : rows_[static_cast<std::size_t>(r)])
        {
            // pointInPolygon 의 (cur=polygon[i], pre=polygon[i-1]) 짝을
            // 그대로 유지한다 -- crossing_x 가 cur 기준으로 계산되므로
            // 순서를 바꾸면 마지막 비트가 달라질 수 있다.
            const Point2 & cur = poly[i];
            const Point2 & pre = poly[prevIndex(i)];

            const bool spans = (cur.y > p.y) != (pre.y > p.y);
            if (!spans)
            {
                continue;
            }
            const double dy = pre.y - cur.y;
            if (dy == 0.0)
            {
                continue;
            }
            const double crossing_x = cur.x + (p.y - cur.y) * (pre.x - cur.x) / dy;
            if (p.x < crossing_x)
            {
                inside_flag = !inside_flag;
            }
        }
        return inside_flag;
    }

private:
    static constexpr double kInfLocal = std::numeric_limits<double>::infinity();

    int nextIndex(int i) const { return (i + 1 == n_) ? 0 : i + 1; }
    int prevIndex(int i) const { return (i == 0) ? n_ - 1 : i - 1; }

    std::size_t cellIndex(int cx, int cy) const
    {
        return static_cast<std::size_t>(cy) * static_cast<std::size_t>(nx_) +
               static_cast<std::size_t>(cx);
    }

    // 변 j 와 셀 사각형 사이 거리의 **진하한**. 변을 자기 AABB 로 대신
    // 재므로 실제보다 작거나 같다 -- 후보를 더 넣을지언정 빠뜨리지 않는다.
    static double edgeRectLowerBound(
        const std::vector<Point2> & poly, int j, double lx, double ly,
        double hx, double hy)
    {
        const Point2 & a = poly[j];
        const Point2 & b = poly[(j + 1 == static_cast<int>(poly.size())) ? 0 : j + 1];
        const double elx = std::min(a.x, b.x);
        const double ely = std::min(a.y, b.y);
        const double ehx = std::max(a.x, b.x);
        const double ehy = std::max(a.y, b.y);
        // 두 AABB 사이 거리 = 축별 간격의 hypot (겹치면 0).
        const double gx = std::max({lx - ehx, elx - hx, 0.0});
        const double gy = std::max({ly - ehy, ely - hy, 0.0});
        return std::hypot(gx, gy);
    }

    // ----------------------------------------------------------------
    // 셀별 후보 edge 리스트.
    //
    // 포함 증명 -- 셀 C 안의 임의의 점 q 와 그 최근접 변 e* 에 대해:
    //   R  = min_e  maxDist(C, e)          (셀에서 가장 멀어야 하는 거리들의 최소)
    //   lb(e) = C 와 e 의 AABB 사이 거리   (진하한: lb(e) <= dist(q, e))
    // 이면 dist(q, e*) <= dist(q, e_R) <= maxDist(C, e_R) = R 이고,
    // 따라서 lb(e*) <= dist(q, e*) <= R. 즉 e* 는 항상 후보에 들어온다.
    //
    // maxDist 는 사각형 꼭짓점에서 재면 정확하다 (점-선분 거리는 볼록
    // 함수라 볼록집합 위 최댓값이 꼭짓점에서 잡힌다).
    // ----------------------------------------------------------------
    void buildCells()
    {
        const std::vector<Point2> & poly = poly_;
        cells_.assign(static_cast<std::size_t>(nx_) * static_cast<std::size_t>(ny_),
                     std::vector<int>{});
        const double half_diag = 0.5 * std::hypot(cell_, cell_);

        for (int cy = 0; cy < ny_; ++cy)
        {
            for (int cx = 0; cx < nx_; ++cx)
            {
                const double lx = x0_ + cx * cell_;
                const double ly = y0_ + cy * cell_;
                const double hx = lx + cell_;
                const double hy = ly + cell_;
                const Point2 corners[4]{
                    Point2{lx, ly}, Point2{hx, ly}, Point2{hx, hy}, Point2{lx, hy}};
                const double ccx = 0.5 * (lx + hx);
                const double ccy = 0.5 * (ly + hy);

                // 1) 값싼 상한 먼저. 셀 중심에서 가장 가까운 **정점** 까지의
                //    거리 + 셀 반대각이면 "셀 안 어느 점에서든 최근접 변까지"
                //    의 상한이 된다 (삼각부등식). 정점 거리라 hypot 만 돈다.
                double nearest_vertex = kInfLocal;
                for (int j = 0; j < n_; ++j)
                {
                    nearest_vertex = std::min(
                        nearest_vertex,
                        std::hypot(poly[j].x - ccx, poly[j].y - ccy));
                }
                const double r_upper = nearest_vertex + half_diag;

                // 2) 그 상한으로 1 차 추림. 여기서 걸러지는 변은 최근접이 될
                //    수 없다 (lb 가 진하한이므로).
                std::vector<int> & list = cells_[cellIndex(cx, cy)];
                list.clear();
                for (int j = 0; j < n_; ++j)
                {
                    if (edgeRectLowerBound(poly, j, lx, ly, hx, hy) <= r_upper)
                    {
                        list.push_back(j);
                    }
                }

                // 3) 추린 것들로만 정확한 R 을 구한다 (min over e of maxDist).
                //    전수로 돌면 셀당 n_ x 4 번 점-선분 거리라 기동이 느려진다.
                double r_bound = kInfLocal;
                for (const int j : list)
                {
                    const Point2 & a = poly[j];
                    const Point2 & b = poly[nextIndex(j)];
                    double far = 0.0;
                    for (const Point2 & c : corners)
                    {
                        far = std::max(far, distanceToSegment(a, b, c));
                    }
                    r_bound = std::min(r_bound, far);
                }

                // 4) 확정. r_bound <= r_upper 이므로 list 밖은 볼 필요 없다.
                list.erase(
                    std::remove_if(
                        list.begin(), list.end(),
                        [&](int j)
                        {
                            return edgeRectLowerBound(poly, j, lx, ly, hx, hy) >
                                   r_bound;
                        }),
                    list.end());
            }
        }
    }

    // ----------------------------------------------------------------
    // y-버킷. ray-cast 는 점의 y 를 가로지르는 변만 의미가 있으므로
    // (spans 조건), 변을 자기 y 구간이 걸치는 줄에 전부 등록해 둔다.
    // 질의는 자기 줄만 보면 되고, 빠뜨리는 변이 없으므로 parity 가 같다.
    // ----------------------------------------------------------------
    void buildRows(double lo_y, double hi_y)
    {
        const std::vector<Point2> & poly = poly_;
        nrow_ = std::max(
            1, static_cast<int>(
                   std::ceil(((hi_y + kPadCm) - y0_) / kRowHeightCm)));
        rows_.assign(static_cast<std::size_t>(nrow_), std::vector<int>{});
        (void)lo_y;

        for (int i = 0; i < n_; ++i)
        {
            // distance 쪽과 달리 (i, i-1) 짝이다 (pointInPolygon 규약).
            const Point2 & cur = poly[i];
            const Point2 & pre = poly[prevIndex(i)];
            int r0 = static_cast<int>(
                std::floor((std::min(cur.y, pre.y) - y0_) / kRowHeightCm));
            int r1 = static_cast<int>(
                std::floor((std::max(cur.y, pre.y) - y0_) / kRowHeightCm));
            r0 = std::clamp(r0, 0, nrow_ - 1);
            r1 = std::clamp(r1, 0, nrow_ - 1);
            for (int r = r0; r <= r1; ++r)
            {
                rows_[static_cast<std::size_t>(r)].push_back(i);
            }
        }
    }

    // 폴리곤을 **복사해서 갖는다**. 가리키기만 하면 안 된다 --
    // RoadBoundary 는 index(shared_ptr) 를 기본 복사자로 그대로 넘기므로
    // (헤더 주석 "인덱스도 shared_ptr 로 따라온다"), 원본을 가리키던
    // 포인터가 원본이 사라진 뒤에도 남아 dangling 이 된다.
    // 실제로 `boundary_ = loadBoundary()` 의 임시 객체와
    // `LocalPlanner(RoadBoundary boundary)` 의 값 전달에서 그렇게 됐고,
    // 첫 pointClearance 질의에서 SIGSEGV 로 노드가 즉사했다.
    // 388 정점 x 2 = 약 12KB 라 복사 비용은 무시할 수 있다.
    std::vector<Point2> poly_;
    int n_ = 0;

    double x0_ = 0.0, y0_ = 0.0, cell_ = 25.0;
    int nx_ = 0, ny_ = 0;
    std::vector<std::vector<int>> cells_;

    int nrow_ = 0;
    std::vector<std::vector<int>> rows_;
};

}  // namespace

// RoadBoundary 가 shared_ptr 로 들고 다니는 실체. 헤더에는 전방 선언만 있다.
struct BoundaryIndex
{
    PolygonIndex outer;
    PolygonIndex inner;
};

namespace
{

// 인덱스를 보장한다. 없으면 만들고, 정점 개수가 달라졌으면 (폴리곤이
// 교체된 것이므로) 다시 만든다. RoadBoundary::resetIndex() 도 참조.
const BoundaryIndex & ensureIndex(const RoadBoundary & boundary)
{
    if (boundary.index &&
        boundary.index->outer.vertexCount() == boundary.outer.size() &&
        boundary.index->inner.vertexCount() == boundary.inner.size())
    {
        return *boundary.index;
    }

    auto built = std::make_shared<BoundaryIndex>();
    built->outer.build(boundary.outer);
    built->inner.build(boundary.inner);
    boundary.index = built;
    return *built;
}

}  // namespace

RoadBoundary makeRoadBoundary(
    const std::vector<double> & outer_x_m, const std::vector<double> & outer_y_m,
    const std::vector<double> & inner_x_m, const std::vector<double> & inner_y_m,
    const SimToMap & transform)
{
    RoadBoundary rb;

    if (outer_x_m.size() != outer_y_m.size() ||
        inner_x_m.size() != inner_y_m.size())
    {
        throw std::invalid_argument(
            "makeRoadBoundary: x/y 배열 길이 불일치");
    }

    rb.outer.reserve(outer_x_m.size());
    for (std::size_t i = 0; i < outer_x_m.size(); ++i)
    {
        rb.outer.push_back(simToMapCm(outer_x_m[i], outer_y_m[i], transform));
    }

    rb.inner.reserve(inner_x_m.size());
    for (std::size_t i = 0; i < inner_x_m.size(); ++i)
    {
        rb.inner.push_back(simToMapCm(inner_x_m[i], inner_y_m[i], transform));
    }

    // 공간 인덱스를 여기서 미리 만든다. lazy 로 둬도 동작은 같지만, 그러면
    // 첫 plan() 틱 하나가 빌드 비용을 통째로 문다 (실측 트랙에서 수 ms).
    ensureIndex(rb);

    return rb;
}

bool pointInPolygon(const std::vector<Point2> & polygon, const Point2 & p)
{
    if (polygon.size() < 3)
    {
        return false;
    }

    bool inside = false;
    const std::size_t n = polygon.size();
    for (std::size_t i = 0, prev = n - 1; i < n; prev = i++)
    {
        const Point2 & cur = polygon[i];
        const Point2 & pre = polygon[prev];

        const bool spans = (cur.y > p.y) != (pre.y > p.y);
        if (!spans)
        {
            continue;
        }
        const double dy = pre.y - cur.y;
        if (dy == 0.0)
        {
            continue;
        }
        const double crossing_x = cur.x + (p.y - cur.y) * (pre.x - cur.x) / dy;
        if (p.x < crossing_x)
        {
            inside = !inside;
        }
    }
    return inside;
}

double distanceToPolygonBoundary(
    const std::vector<Point2> & polygon, const Point2 & p)
{
    if (polygon.size() < 2)
    {
        return std::numeric_limits<double>::infinity();
    }
    double best = std::numeric_limits<double>::infinity();
    const std::size_t n = polygon.size();
    for (std::size_t i = 0; i < n; ++i)
    {
        const Point2 & a = polygon[i];
        const Point2 & b = polygon[(i + 1) % n];
        best = std::min(best, distanceToSegment(a, b, p));
    }
    return best;
}

double pointClearance(
    const RoadBoundary & boundary, const Point2 & p, double footprint_cm)
{
    // 공간 인덱스 경유. 아래 전수 탐색 버전과 **같은 double** 이 나온다
    // (RoadBoundary::index 주석의 포함 증명 참조). 폴리곤이 폴리곤이라
    // 부를 수 없을 만큼 작으면(단위테스트의 더미 inner 등) 인덱스가
    // valid()=false 로 남고 참조 구현으로 폴백한다.
    const BoundaryIndex & ix = ensureIndex(boundary);

    const double outer_dist = ix.outer.valid()
        ? ix.outer.distance(p) : distanceToPolygonBoundary(boundary.outer, p);
    const bool outer_in = ix.outer.valid()
        ? ix.outer.inside(p) : pointInPolygon(boundary.outer, p);
    const double outer_margin = outer_in ? outer_dist : -outer_dist;

    const double inner_dist = ix.inner.valid()
        ? ix.inner.distance(p) : distanceToPolygonBoundary(boundary.inner, p);
    const bool inner_in = ix.inner.valid()
        ? ix.inner.inside(p) : pointInPolygon(boundary.inner, p);
    const double inner_margin = !inner_in ? inner_dist : -inner_dist;

    return std::min(outer_margin, inner_margin) - footprint_cm;
}

double marginAlongNormal(
    const RoadBoundary & boundary, const Point2 & origin, const Point2 & normal,
    double sign, double footprint_cm, double max_search_cm)
{
    const auto at = [&](double t) -> double
    {
        const Point2 p{origin.x + sign * t * normal.x,
                       origin.y + sign * t * normal.y};
        return pointClearance(boundary, p, footprint_cm);
    };

    // origin 자체가 이미 위반이면(예: 후보가 이미 경계 밖) 여유 0.
    if (at(0.0) < 0.0)
    {
        return 0.0;
    }

    // 단순 이분탐색은 clearance(t) 가 단조라고 가정하는데, inner 가 원형
    // 섬이 아니라 도로 폭보다 좁은 "구멍" 모양이면 안전 -> 위반 -> 안전으로
    // 두 번 부호가 바뀔 수 있다. 그 경우 이분탐색은 가까운 위반을 건너뛰고
    // 더 먼 경계를 오탐할 수 있다 (실제 gtest 로 발견). 그래서 먼저 성긴
    // step 으로 훑어 "가장 가까운" 부호 반전 구간을 찾고, 그 구간 안에서만
    // 이분탐색으로 정밀화한다 -- 이러면 단조 여부와 무관하게 항상 가장
    // 가까운 위반 지점을 찾는다.
    constexpr double kScanStep = 2.0;   // cm, 정밀도와 비용의 절충
    double prev_t = 0.0;
    double prev_clear = at(0.0);
    double lo = -1.0;
    double hi = -1.0;
    for (double t = kScanStep; t <= max_search_cm; t += kScanStep)
    {
        const double clear = at(t);
        if (clear < 0.0)
        {
            lo = prev_t;
            hi = t;
            break;
        }
        prev_t = t;
        prev_clear = clear;
    }
    (void)prev_clear;

    if (lo < 0.0)
    {
        return max_search_cm;   // 탐색 범위 안에서 위반 없음 -> 사실상 무제한
    }

    for (int it = 0; it < 24; ++it)
    {
        const double mid = 0.5 * (lo + hi);
        if (at(mid) >= 0.0)
        {
            lo = mid;
        }
        else
        {
            hi = mid;
        }
    }
    return lo;
}

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
        // Python: np.linspace(0, 1, count, endpoint=(i==nseg-1)) -- 마지막
        // segment 만 u=1.0 포함, 그 외엔 다음 segment 의 u=0 과 중복이라 제외.
        const bool include_end = (i == nseg - 1);
        const double step = include_end
            ? 1.0 / static_cast<double>(count - 1)
            : 1.0 / static_cast<double>(count);
        for (int k = 0; k < count; ++k)
        {
            const double u = static_cast<double>(k) * step;
            const Point2 point = kau::bezier::evalSeg(cv.seg(i), u);
            best = std::min(best, pointClearance(boundary, point, footprint_cm));
        }
    }
    return best;
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
                        best, pointClearance(boundary, corner, road_safety_margin_cm));
                }
            }
        }
    }
    return best;
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
        // roadClearanceRect 와 같은 규약: 마지막 segment 만 u=1.0 을 포함한다
        // (그 외엔 다음 segment 의 u=0 과 중복).
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
                        pointClearance(boundary, corner(wx, sy * y_outer),
                                      road_safety_margin_cm));
                    if (pointClearance(boundary, corner(wx, sy * y_inner),
                                      road_safety_margin_cm) >= 0.0)
                    {
                        ++wheels_on;
                    }
                }
            }

            rep.min_wheels_on = std::min(rep.min_wheels_on, wheels_on);
            // u 는 호길이 매개변수가 아니라 station 은 근사다 (아래 주석 참조).
            const double s_here = station + u * seg_len;
            if (wheels_on < 4)
            {
                // u 는 호길이 매개변수가 아니라 Bezier 매개변수라, u*seg_len 은
                // 근사다. 진단 표시용이므로 이 정도면 충분하다 (sample_interval
                // 4cm 격자 위의 값이다).
                if (rep.first_viol_s_cm < 0.0)
                {
                    rep.first_viol_s_cm = s_here;
                }
                rep.off_integral_cm +=
                    static_cast<double>(4 - wheels_on) / 4.0 * step * seg_len;
            }

            // s_max 를 넘긴 첫 샘플까지 본 뒤 멈춘다 (보수적 절단).
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
        rep.min_clear_cm = 0.0;   // 빈 곡선
    }
    return rep;
}

}  // namespace local_path_planner
}  // namespace kau
