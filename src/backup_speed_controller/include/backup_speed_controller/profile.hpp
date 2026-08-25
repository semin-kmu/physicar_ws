// 속도 프로파일 순수 함수. ROS 비의존.
//
// 기하 판단은 없다 -- 경로 곡률과 코너 진입거리만 본다.

#ifndef BACKUP_SPEED_CONTROLLER__PROFILE_HPP_
#define BACKUP_SPEED_CONTROLLER__PROFILE_HPP_

#include <algorithm>
#include <cmath>

namespace backup_speed_controller
{

struct ProfileParams
{
  double v_max = 1.40;
  double v_min = 0.50;
  double a_lat_max = 3.0;
  double a_decel = 0.5;
  double t_lat_s = 0.10;
};

// v = sqrt(a_lat_max / |kappa|). kappa 0 (직선) 이면 제한 없음 -> v_max.
inline double curveSpeed(double abs_kappa, const ProfileParams & p)
{
  if (abs_kappa <= 0.0) {return p.v_max;}
  return std::sqrt(p.a_lat_max / abs_kappa);
}

inline double clampSpeed(double v, const ProfileParams & p)
{
  return std::max(p.v_min, std::min(v, p.v_max));
}

// d = (v^2 - v_corner^2)/(2*a_decel) + v*t_lat.
// t_lat 항은 이미지 stamp -> 조향 반영까지의 지연 동안 진행한 거리다.
inline double brakeDistance(double v, double v_corner, const ProfileParams & p)
{
  const double a = std::max(p.a_decel, 1e-6);
  return (v * v - v_corner * v_corner) / (2.0 * a) + v * p.t_lat_s;
}

// tick 당 상승/하강 폭 제한. up_step / down_step 은 모두 양수.
inline double rateLimit(double cur, double target, double up_step, double down_step)
{
  const double d = target - cur;
  if (d > 0.0) {return cur + std::min(d, std::max(up_step, 0.0));}
  return cur - std::min(-d, std::max(down_step, 0.0));
}

}  // namespace backup_speed_controller

#endif  // BACKUP_SPEED_CONTROLLER__PROFILE_HPP_
