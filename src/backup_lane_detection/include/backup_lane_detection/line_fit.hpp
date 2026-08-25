// 지면 직선 IRLS 피팅 + 변곡점 분할.
//
// RANSAC 을 쓰지 않는다. 같은 입력에 다른 해가 나와 프레임 간 무상관
// 잡음이 되고, 그게 지터의 3위 원인이다. 이전 프레임 해를 초기값으로 한
// Huber IRLS 는 결정적이고 시간적으로 연속이다.
//
// 좌표: x 전방(후륜축 원점), y 좌 +. 모델 y = a*x + b.

#ifndef BACKUP_LANE_DETECTION__LINE_FIT_HPP_
#define BACKUP_LANE_DETECTION__LINE_FIT_HPP_

#include <algorithm>
#include <cmath>
#include <vector>

#include <backup_common/bezier.hpp>

namespace backup_lane_detection
{

using backup_common::Point2;

struct Line
{
  double a = 0.0;
  double b = 0.0;
  double x0 = 0.0;             // 실제 검출점의 x 범위
  double x1 = 0.0;
  double rms = 0.0;            // [m] 잔차 RMS
  int n = 0;
  float confidence = 0.0f;
  bool valid = false;

  double at(double x) const {return a * x + b;}
  double heading() const {return std::atan(a);}
};

struct FitParams
{
  int iterations = 4;
  double huber_delta = 0.02;
  int min_points = 8;
  bool seed_from_previous = true;
};

// 가중 최소제곱 누산기. prefix-sum 분할 탐색에도 그대로 쓴다.
struct LsSums
{
  double n = 0.0;
  double sx = 0.0;
  double sy = 0.0;
  double sxx = 0.0;
  double sxy = 0.0;
  double syy = 0.0;

  void add(double x, double y, double w)
  {
    n += w;
    sx += w * x;
    sy += w * y;
    sxx += w * x * x;
    sxy += w * x * y;
    syy += w * y * y;
  }

  LsSums operator-(const LsSums & o) const
  {
    LsSums r;
    r.n = n - o.n;
    r.sx = sx - o.sx;
    r.sy = sy - o.sy;
    r.sxx = sxx - o.sxx;
    r.sxy = sxy - o.sxy;
    r.syy = syy - o.syy;
    return r;
  }
};

// 최적해에서 SSE = Syy - a*Sxy - b*Sy.
inline bool solveLs(const LsSums & s, double & a, double & b, double & sse)
{
  const double det = s.n * s.sxx - s.sx * s.sx;
  if (std::fabs(det) < 1e-12) {return false;}
  a = (s.n * s.sxy - s.sx * s.sy) / det;
  b = (s.sxx * s.sy - s.sx * s.sxy) / det;
  sse = std::max(0.0, s.syy - a * s.sxy - b * s.sy);
  return true;
}

// [begin, end) 구간을 IRLS 로 피팅. seed 가 있으면 그것을 초기값으로 쓴다.
inline bool fitIRLS(
  const std::vector<Point2> & pts, size_t begin, size_t end,
  const FitParams & fp, const Line * seed, Line & out)
{
  const size_t n = (end > begin) ? (end - begin) : 0;
  if (n < static_cast<size_t>(std::max(2, fp.min_points))) {return false;}

  double a = 0.0;
  double b = 0.0;

  if (fp.seed_from_previous && seed != nullptr && seed->valid) {
    a = seed->a;
    b = seed->b;
  } else {
    LsSums s;
    for (size_t i = begin; i < end; ++i) {s.add(pts[i].x, pts[i].y, 1.0);}
    double sse = 0.0;
    if (!solveLs(s, a, b, sse)) {return false;}
  }

  for (int it = 0; it < fp.iterations; ++it) {
    LsSums s;
    for (size_t i = begin; i < end; ++i) {
      const double r = std::fabs(pts[i].y - (a * pts[i].x + b));
      const double w = (r <= fp.huber_delta) ? 1.0 : fp.huber_delta / std::max(r, 1e-12);
      s.add(pts[i].x, pts[i].y, w);
    }
    double na = 0.0;
    double nb = 0.0;
    double sse = 0.0;
    if (!solveLs(s, na, nb, sse)) {break;}
    a = na;
    b = nb;
  }

  double sq = 0.0;
  int inliers = 0;
  double xmin = pts[begin].x;
  double xmax = pts[begin].x;
  for (size_t i = begin; i < end; ++i) {
    const double r = pts[i].y - (a * pts[i].x + b);
    sq += r * r;
    if (std::fabs(r) <= fp.huber_delta) {++inliers;}
    xmin = std::min(xmin, pts[i].x);
    xmax = std::max(xmax, pts[i].x);
  }

  out.a = a;
  out.b = b;
  out.x0 = xmin;
  out.x1 = xmax;
  out.n = static_cast<int>(n);
  out.rms = std::sqrt(sq / static_cast<double>(n));
  // 점 개수 하한은 min_points 가 이미 거른다. 남는 지표는 inlier 비율뿐이다.
  out.confidence = static_cast<float>(static_cast<double>(inliers) / static_cast<double>(n));
  out.valid = true;
  return true;
}

inline bool fitIRLS(
  const std::vector<Point2> & pts, const FitParams & fp, const Line * seed, Line & out)
{
  return fitIRLS(pts, 0, pts.size(), fp, seed, out);
}

struct Split
{
  bool ok = false;
  Line a;                  // 근거리 (진입)
  Line b;                  // 원거리 (탈출)
  double x = 0.0;          // 두 직선 교점 [m]
  double y = 0.0;
  double delta_psi = 0.0;  // [rad] 좌회전 +
  double arc_length = 0.0; // [m] 전환구간 호길이. 신뢰 불가면 0
  int n_transition = 0;
};

// x 오름차순 정렬된 [begin, end) 를 [직선A][전환][직선B] 로 나눈다.
// 분할점 탐색은 prefix-sum LS 로 O(n), 확정 뒤 양쪽만 IRLS 로 다듬는다.
// (모든 후보마다 IRLS 를 돌리면 O(n * iters) 라 예산을 넘긴다.)
inline bool splitAtInflection(
  const std::vector<Point2> & pts, size_t begin, size_t end,
  const FitParams & fp, double residual_thresh, int min_transition_points, Split & out)
{
  const size_t n = (end > begin) ? (end - begin) : 0;
  const size_t need = static_cast<size_t>(std::max(2, fp.min_points));
  if (n < 2 * need) {return false;}

  // prefix[i] = [begin, begin+i) 누산
  std::vector<LsSums> prefix(n + 1);
  for (size_t i = 0; i < n; ++i) {
    prefix[i + 1] = prefix[i];
    prefix[i + 1].add(pts[begin + i].x, pts[begin + i].y, 1.0);
  }

  double best_cost = -1.0;
  size_t best_k = 0;
  for (size_t k = need; k + need <= n; ++k) {
    double a1 = 0.0;
    double b1 = 0.0;
    double s1 = 0.0;
    double a2 = 0.0;
    double b2 = 0.0;
    double s2 = 0.0;
    if (!solveLs(prefix[k], a1, b1, s1)) {continue;}
    if (!solveLs(prefix[n] - prefix[k], a2, b2, s2)) {continue;}
    const double cost = s1 + s2;
    if (best_cost < 0.0 || cost < best_cost) {
      best_cost = cost;
      best_k = k;
    }
  }
  if (best_cost < 0.0) {return false;}

  Line la;
  Line lb;
  if (!fitIRLS(pts, begin, begin + best_k, fp, nullptr, la)) {return false;}
  if (!fitIRLS(pts, begin + best_k, end, fp, nullptr, lb)) {return false;}

  // 거의 평행하면 교점이 발산한다 -- 변곡점이 아니다.
  const double da = la.a - lb.a;
  if (std::fabs(da) < 1e-9) {return false;}

  out.a = la;
  out.b = lb;
  out.x = (lb.b - la.b) / da;
  out.y = la.at(out.x);
  out.delta_psi = lb.heading() - la.heading();

  // 전환구간 = 양쪽 직선 모두에서 벗어난 점.
  double arc = 0.0;
  int cnt = 0;
  bool have_prev = false;
  Point2 prev;
  for (size_t i = begin; i < end; ++i) {
    const double ra = std::fabs(pts[i].y - la.at(pts[i].x));
    const double rb = std::fabs(pts[i].y - lb.at(pts[i].x));
    if (ra <= residual_thresh || rb <= residual_thresh) {continue;}
    ++cnt;
    if (have_prev) {arc += std::hypot(pts[i].x - prev.x, pts[i].y - prev.y);}
    prev = pts[i];
    have_prev = true;
  }
  out.n_transition = cnt;
  // 점이 모자라면 호길이를 신뢰할 수 없다. 하류가 쓰지 않도록 0 을 낸다.
  out.arc_length = (cnt >= min_transition_points) ? arc : 0.0;

  out.ok = true;
  return true;
}

}  // namespace backup_lane_detection

#endif  // BACKUP_LANE_DETECTION__LINE_FIT_HPP_
