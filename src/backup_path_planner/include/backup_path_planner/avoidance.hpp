// 장애물 회피 -- 기준 경로에 횡 offset 을 중첩한다. ROS 비의존, 헤더 온리.
//
// 콘 6 개가 전부 직선 구간에 있어 코너 로직과 겹치지 않는다.
// 차선 이탈이 실격이므로 장애물 여유를 만족하는 선에서 차선 여유를 최대로 잡는다.

#ifndef BACKUP_PATH_PLANNER__AVOIDANCE_HPP_
#define BACKUP_PATH_PLANNER__AVOIDANCE_HPP_

#include <algorithm>
#include <cmath>
#include <vector>

#include <backup_common/bezier.hpp>
#include <backup_common/vehicle.hpp>

#include "backup_path_planner/race_line.hpp"

namespace backup_path_planner
{

struct AvoidConfig
{
  double obstacle_radius_m = 0.142;
  double clearance_m = 0.10;
  double trigger_m = 1.40;
  double lateral_gate_m = 0.30;
  double vehicle_width_m = 0.205;
  double track_width_m = 0.700;
  double lane_margin_m = 0.05;
};

struct AvoidTarget
{
  double x = 0.0;
  double y = 0.0;
};

struct AvoidDecision
{
  bool active = false;
  double offset = 0.0;       // 기준 경로 대비 목표 횡 offset. 좌측 +
  bool clamped = false;      // 차선 여유에 걸렸다
  bool stop_request = false; // 클램프 후에도 장애물 여유 미달
  AvoidTarget target{};
};

// 차체 중심이 장애물 중심에서 떨어져 있어야 하는 최소 횡거리.
inline double requiredSeparation(const AvoidConfig & cfg)
{
  return cfg.obstacle_radius_m + 0.5 * cfg.vehicle_width_m + cfg.clearance_m;
}

// 차선 중앙에서 허용되는 편측 최대 횡변위.
inline double lateralLimit(const AvoidConfig & cfg)
{
  return 0.5 * cfg.track_width_m - 0.5 * cfg.vehicle_width_m - cfg.lane_margin_m;
}

// 전방 trigger 이내 & 경로 횡거리 |dy| < gate 인 장애물 중 가장 가까운 것.
// obstacles 는 base_link 좌표.
inline bool selectObstacle(
  const Segment & ref, const std::vector<AvoidTarget> & obstacles,
  const AvoidConfig & cfg, AvoidTarget & out, double & path_y_out)
{
  bool found = false;
  double best_x = cfg.trigger_m + 1.0;
  for (const auto & o : obstacles) {
    if (o.x <= 0.0 || o.x > cfg.trigger_m) {continue;}
    const double py = lateralAtStation(ref, o.x);
    if (std::fabs(o.y - py) >= cfg.lateral_gate_m) {continue;}
    if (o.x < best_x) {best_x = o.x; out = o; path_y_out = py; found = true;}
  }
  return found;
}

// 종방향 x 에서의 차선 중앙 횡좌표. foot/u_c 는 중앙선.
inline double laneCenterAtStation(const Point2 & foot, const Point2 & u_c, double x)
{
  if (std::fabs(u_c.x) < 1e-3) {return foot.y;}
  return foot.y + u_c.y * (x - foot.x) / u_c.x;
}

// |offset| = (r_obs + 차폭/2 + clearance) - |장애물-경로 횡거리|,  부호는 장애물 반대편.
// 실측 검산: 콘이 중앙선에서 0.18, r 0.142, 차폭 0.205, clearance 0.10
//            -> offset -0.1645, 차선 가장자리까지 0.083 남는다.
inline AvoidDecision decideAvoidance(
  const Segment & ref, const std::vector<AvoidTarget> & obstacles,
  const Point2 & foot, const Point2 & u_c, const AvoidConfig & cfg)
{
  AvoidDecision d;
  AvoidTarget obs;
  double path_y = 0.0;
  if (!selectObstacle(ref, obstacles, cfg, obs, path_y)) {return d;}

  const double d_req = requiredSeparation(cfg);
  const double dy = obs.y - path_y;
  if (std::fabs(dy) >= d_req) {return d;}

  d.active = true;
  d.target = obs;
  d.offset = -((dy >= 0.0) ? 1.0 : -1.0) * (d_req - std::fabs(dy));

  // 차선 여유 클램프. 이탈이 실격이라 장애물 여유보다 우선한다.
  const double lane_center_y_at_obstacle = laneCenterAtStation(foot, u_c, obs.x);
  const double limit = lateralLimit(cfg);
  const double dev = (path_y + d.offset) - lane_center_y_at_obstacle;
  if (std::fabs(dev) > limit) {
    d.offset = lane_center_y_at_obstacle + backup_common::clampAbs(dev, limit) - path_y;
    d.clamped = true;
    if (std::fabs(obs.y - (path_y + d.offset)) < d_req) {d.stop_request = true;}
  }
  return d;
}

// 중간 제어점만 횡이동한다. 양 끝 접선이 보존되어 S 자가 자동으로 이어진다.
inline void applyLateralOffset(Segment & s, double offset)
{
  s.p[2].y += offset;
  s.p[3].y += offset;
}

// 레이트 리밋. 계단 offset 은 조향 계단이 된다.
inline double rateLimitOffset(double current, double target, double rate, double dt)
{
  if (dt <= 0.0) {return current;}
  return current + backup_common::clampAbs(target - current, rate * dt);
}

}  // namespace backup_path_planner

#endif  // BACKUP_PATH_PLANNER__AVOIDANCE_HPP_
