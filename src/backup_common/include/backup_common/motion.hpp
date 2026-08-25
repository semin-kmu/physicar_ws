// 데드레커닝. 상대좌표 메시지를 stamp 시점 body frame 에서 현재 body frame 으로 옮긴다.
//
// 백업 스택 지터의 최대 원인이 이 보상의 부재다.
// 경로는 base_link 상대좌표인데 ~12 Hz 로 오고 제어는 50 Hz 다.
// v=1.43 m/s 에서 83 ms 동안 11.9 cm 어긋나 조향에 1.70 deg 톱니파가 실린다.
// 측정 잡음(0.5 cm -> 0.07 deg)의 24 배다.

#ifndef BACKUP_COMMON__MOTION_HPP_
#define BACKUP_COMMON__MOTION_HPP_

#include <cmath>

namespace backup_common
{

struct Pose2
{
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
};

// 등속·등조향으로 dt 동안 움직였을 때, 옛 body frame 원점에서 본 새 body frame 의 pose.
// bicycle model: R = L/tan(delta), 호를 그린다.
inline Pose2 integrateBicycle(double v, double steer_rad, double wheelbase, double dt)
{
  Pose2 d;
  const double t = std::tan(steer_rad);
  if (std::fabs(t) < 1e-9) {
    d.x = v * dt;
    d.y = 0.0;
    d.yaw = 0.0;
    return d;
  }
  const double radius = wheelbase / t;
  d.yaw = v * dt / radius;
  d.x = radius * std::sin(d.yaw);
  d.y = radius * (1.0 - std::cos(d.yaw));
  return d;
}

// 옛 body frame 좌표 (x, y) 를 새 body frame 으로 옮긴다.
//   p_new = Rot(-dyaw) * (p_old - t)
inline void transformPoint(const Pose2 & motion, double & x, double & y)
{
  const double dx = x - motion.x;
  const double dy = y - motion.y;
  const double c = std::cos(motion.yaw);
  const double s = std::sin(motion.yaw);
  x = c * dx + s * dy;
  y = -s * dx + c * dy;
}

// 각도(헤딩)도 같이 돌린다.
inline double transformHeading(const Pose2 & motion, double heading)
{
  return heading - motion.yaw;
}

// 편의 함수: v/steer/dt 를 받아 점 하나를 바로 옮긴다.
inline void deadReckonPoint(
  double v, double steer_rad, double wheelbase, double dt,
  double & x, double & y)
{
  const Pose2 m = integrateBicycle(v, steer_rad, wheelbase, dt);
  transformPoint(m, x, y);
}

}  // namespace backup_common

#endif  // BACKUP_COMMON__MOTION_HPP_
