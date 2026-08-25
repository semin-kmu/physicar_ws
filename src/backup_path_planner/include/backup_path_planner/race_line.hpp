// out-in-out 레이스 라인 기하. ROS 비의존, 헤더 온리. 단위 m/rad, 좌측 +.
//
// 중앙선 추종은 이 트랙에서 원리적으로 불가능하다 (중앙선 R_min 0.151 <
// 차량 R_min 0.4945). 코너를 통과하는 수단은 레이스 라인뿐이다.

#ifndef BACKUP_PATH_PLANNER__RACE_LINE_HPP_
#define BACKUP_PATH_PLANNER__RACE_LINE_HPP_

#include <algorithm>
#include <cmath>

#include <backup_common/bezier.hpp>
#include <backup_common/vehicle.hpp>

namespace backup_path_planner
{

using backup_common::Point2;
using backup_common::Segment;
using backup_common::VehicleParams;

inline Point2 normalized(const Point2 & u)
{
  const double n = std::hypot(u.x, u.y);
  return (n > 1e-9) ? Point2{u.x / n, u.y / n} : Point2{1.0, 0.0};
}

inline Point2 rotated(const Point2 & u, double a)
{
  const double c = std::cos(a);
  const double s = std::sin(a);
  return {u.x * c - u.y * s, u.x * s + u.y * c};
}

// 좌측 법선.
inline Point2 leftNormal(const Point2 & u)
{
  return {-u.y, u.x};
}

// 레이스 라인 형상 파라미터. 전부 yaml 에서 온다.
struct RaceLineConfig
{
  double d_sight_m = 1.60;
  double t_lat_s = 0.10;
  double r_max_m = 5.0;
  double track_width_m = 0.700;
  double lane_margin_m = 0.05;
  int width_iterations = 2;
};

struct RaceLine
{
  double radius = 0.0;       // 채택 R (R_min 클램프 후)
  double tangent_len = 0.0;  // T = R*tan(|dpsi|/2). 꼭짓점 <-> 접점
  double apex_offset = 0.0;  // 꼭짓점 -> 정점 = R*(sec(dpsi/2)-1)
  Point2 entry{};            // 진입 접점 A
  Point2 apex{};             // 정점 M
  Point2 exit{};             // 탈출 접점 B
  Point2 u_in{1.0, 0.0};
  Point2 u_out{1.0, 0.0};
};

// R = min( W/(1-cos(dpsi/2)), (d_sight - v*t_lat)/tan(dpsi/2), r_max )
// W = track_width - sweptWidth(R) - 2*lane_margin
//
// W 가 R 에 의존하므로 반복해서 푼다. 초기값은 R 비의존인 R_sight.
// dpsi 가 작으면 R_sight 가 발산하므로 r_max 로 상한.
inline double solveRaceRadius(
  double dpsi_abs, double v, const RaceLineConfig & cfg, const VehicleParams & veh)
{
  const double half = 0.5 * dpsi_abs;
  const double tan_half = std::tan(half);
  const double one_minus_cos = 1.0 - std::cos(half);

  const double sight = std::max(cfg.d_sight_m - v * cfg.t_lat_s, 0.0);
  double r_sight = (tan_half > 1e-6) ? (sight / tan_half) : cfg.r_max_m;
  r_sight = std::min(r_sight, cfg.r_max_m);

  double r = std::max(r_sight, 1e-3);
  const int iters = std::max(cfg.width_iterations, 1);
  for (int i = 0; i < iters; ++i) {
    const double w = cfg.track_width_m - veh.sweptWidth(r) - 2.0 * cfg.lane_margin_m;
    const double r_width =
      (one_minus_cos > 1e-9) ? (std::max(w, 0.0) / one_minus_cos) : cfg.r_max_m;
    r = std::min({r_width, r_sight, cfg.r_max_m});
    r = std::max(r, 1e-3);  // sweptWidth(0) 발산 방지
  }
  return r;
}

// 꼭짓점 C, 진입 방향 u_in, 편각 dpsi, 반경 R 로 접점/정점을 놓는다.
inline RaceLine raceLineFromRadius(
  const Point2 & corner, const Point2 & u_in, double dpsi, double radius)
{
  RaceLine rl;
  const double half = 0.5 * std::fabs(dpsi);
  const double cos_half = std::cos(half);

  rl.radius = radius;
  rl.tangent_len = radius * std::tan(half);
  rl.apex_offset = (cos_half > 1e-9) ? radius * (1.0 / cos_half - 1.0) : 0.0;

  rl.u_in = normalized(u_in);
  rl.u_out = rotated(rl.u_in, dpsi);

  rl.entry = corner - rl.u_in * rl.tangent_len;
  rl.exit = corner + rl.u_out * rl.tangent_len;
  // 정점 방향 = 방향차 이등분. 선회 안쪽을 가리킨다.
  rl.apex = corner + normalized(rl.u_out - rl.u_in) * rl.apex_offset;
  return rl;
}

// quintic 의 시작 곡률은 kappa(0) = 0.8 * d_perp / h^2 이다
// (d_perp = P2 의 P0P1 접선 수직거리, h = |P1-P0|).
// tangent_epsilon_m 0.05 를 그대로 쓰면 d_perp 6 mm 만 넘어도 조향 한계를 넘는다.
// 그래서 eps 는 하한으로만 두고, kappa(0) 이 설계 곡률 1/R 을 넘지 않는
// 최소 간격 h = sqrt(0.8 * d_perp * R) 을 쓴다. 접선 방향 고정은 h 와 무관하다.
inline double tangentSpacing(double d_perp, double design_radius, double tangent_eps)
{
  return std::max(tangent_eps, std::sqrt(0.8 * std::fabs(d_perp) * std::fabs(design_radius)));
}

// 제어점 6 개.
//   P0/P1  자차 원점 + 진행방향 접선 (anchor 시)
//   P2     진입 접점 A
//   P3     꼭짓점 C  -- 탈출 직선 위에서 B 보다 T 앞이라 곡선을 정점 쪽으로 당기면서
//                      P3,P4,P5 가 공선이 되어 끝 곡률이 정확히 0 이다
//   P4/P5  탈출 접점 B 와 그 T 뒤
// zero_end_curvature 가 false 면 P3 를 실제 정점에 둔다 (끝 곡률 != 0).
//
// 진입 접점이 이미 자차 뒤면 (T > 꼭짓점까지 거리 -- 접근 중에는 흔하다)
// P2 를 P0,P1 연장선에 둔다. 자차가 이미 호 위에 있다는 뜻이라
// 남은 선회는 P3,P4,P5 가 맡고 시작 곡률은 0 이 된다.
inline Segment raceLineSegment(
  const RaceLine & rl, const Point2 & corner,
  double tangent_eps, bool anchor_at_vehicle, bool zero_end_curvature)
{
  Segment s;
  const Point2 dir = anchor_at_vehicle ? Point2{1.0, 0.0} : rl.u_in;
  const double d_perp = rl.entry.y * dir.x - rl.entry.x * dir.y;
  const double h = tangentSpacing(d_perp, rl.radius, tangent_eps);

  s.p[0] = {0.0, 0.0};
  s.p[1] = dir * h;
  s.p[2] = (rl.entry.x > h) ? rl.entry : (s.p[1] + (s.p[1] - s.p[0]));
  s.p[3] = zero_end_curvature ? corner : rl.apex;
  s.p[4] = rl.exit;
  s.p[5] = rl.exit + rl.u_out * rl.tangent_len;
  return s;
}

// 중앙 직선을 quintic 으로. P2~P5 를 직선 위 균일 배치하면 P3,P4,P5 가
// 공선이라 끝 곡률이 0 이고, 자차가 직선 위에 있으면 경로가 그대로 직선이 된다.
// foot 은 자차 원점을 중앙선에 내린 발.
inline Segment centerLineSegment(
  const Point2 & foot, const Point2 & u_c, double length,
  double tangent_eps, bool anchor_at_vehicle)
{
  Segment s;
  // 균일 간격. 자차가 중앙선 위에 있으면 6 점이 등간격 공선이라 정확히 직선이 된다.
  const double g = std::max(length, 1e-3) / static_cast<double>(backup_common::kDegree);
  const double h = std::max(tangent_eps, g);
  s.p[0] = {0.0, 0.0};
  s.p[1] = anchor_at_vehicle ? Point2{h, 0.0} : Point2{0.0, 0.0} + u_c * h;
  for (int i = 2; i < backup_common::kCtrlPerSeg; ++i) {
    s.p[i] = foot + u_c * (g * static_cast<double>(i));
  }
  return s;
}

// 횡성분을 beta 배로 줄여 자차 진행방향 직선 쪽으로 섞는다. beta 1 = 원본.
inline Segment blendToStraight(const Segment & src, double beta)
{
  Segment o = src;
  for (int i = 0; i < backup_common::kCtrlPerSeg; ++i) {
    o.p[i].y = src.p[i].y * beta;
  }
  return o;
}

// 연속 코너 퇴화 모드. 실현 가능 최대 곡률까지 이분법으로 낮춘다.
// 12 회는 수치 반복 횟수이지 트랙 튜닝값이 아니다 (beta 해상도 2^-12).
inline Segment clampToFeasibleCurvature(const Segment & src, double kappa_max, double & beta_out)
{
  beta_out = 1.0;
  if (src.maxAbsCurvature() <= kappa_max) {return src;}

  double lo = 0.0, hi = 1.0;
  for (int i = 0; i < 12; ++i) {
    const double mid = 0.5 * (lo + hi);
    if (blendToStraight(src, mid).maxAbsCurvature() <= kappa_max) {lo = mid;} else {hi = mid;}
  }
  beta_out = lo;
  return blendToStraight(src, lo);
}

// 표본 nearest 로 점 q 까지의 호길이와 그 지점의 t.
inline double arcLengthToPoint(const Segment & s, const Point2 & q, double & t_out, int n = 64)
{
  double acc = 0.0, best = 1e18, best_s = -1.0;
  t_out = 0.0;
  Point2 prev = s.eval(0.0);
  for (int i = 0; i <= n; ++i) {
    const double t = static_cast<double>(i) / n;
    const Point2 c = s.eval(t);
    acc += std::hypot(c.x - prev.x, c.y - prev.y);
    const double d = (c.x - q.x) * (c.x - q.x) + (c.y - q.y) * (c.y - q.y);
    if (d < best) {best = d; best_s = acc; t_out = t;}
    prev = c;
  }
  return best_s;
}

// [t0, 1] 구간의 |kappa| 최대.
inline double maxAbsCurvatureFrom(const Segment & s, double t0, int n = 21)
{
  double m = 0.0;
  for (int i = 0; i < n; ++i) {
    const double t = t0 + (1.0 - t0) * static_cast<double>(i) / (n - 1);
    m = std::max(m, std::fabs(s.curvature(t)));
  }
  return m;
}

// 종방향 x 에서 경로의 횡좌표. x 가 단조인 구간에서만 의미가 있다.
inline double lateralAtStation(const Segment & s, double x, int n = 32)
{
  Point2 prev = s.eval(0.0);
  for (int i = 1; i <= n; ++i) {
    const Point2 c = s.eval(static_cast<double>(i) / n);
    if ((prev.x - x) * (c.x - x) <= 0.0 && std::fabs(c.x - prev.x) > 1e-9) {
      const double w = (x - prev.x) / (c.x - prev.x);
      return prev.y + (c.y - prev.y) * w;
    }
    prev = c;
  }
  return prev.y;
}

}  // namespace backup_path_planner

#endif  // BACKUP_PATH_PLANNER__RACE_LINE_HPP_
