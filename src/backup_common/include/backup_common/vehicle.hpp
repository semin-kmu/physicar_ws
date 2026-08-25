// 차량 제원과 조향/곡률 변환. 헤더 온리.
//
// 단위: 길이 m, 각 rad (파라미터 입력만 deg).
// 종방향 거리 원점은 후륜축 -- Pure Pursuit 기준점이 후륜축이기 때문이다.

#ifndef BACKUP_COMMON__VEHICLE_HPP_
#define BACKUP_COMMON__VEHICLE_HPP_

#include <algorithm>
#include <cmath>

namespace backup_common
{

inline constexpr double kDeg2Rad = M_PI / 180.0;
inline constexpr double kRad2Deg = 180.0 / M_PI;

struct VehicleParams
{
  double wheelbase = 0.18;               // [m]
  double width = 0.205;                  // [m]
  double length = 0.280;                 // [m]
  double rear_axle_to_front_bumper = 0.230;  // [m]
  double max_steer_deg = 20.0;           // 하드웨어 한계

  double maxSteerRad() const { return max_steer_deg * kDeg2Rad; }

  // 조향 한계에서 나오는 최소 선회반경. 이보다 급한 곡률은 만들 수 없다.
  double rMin() const { return wheelbase / std::tan(maxSteerRad()); }

  double kappaMax() const { return std::tan(maxSteerRad()) / wheelbase; }

  // kappa -> 조향각. Ackermann bicycle.
  double steerOfKappa(double kappa) const
  {
    return std::atan(wheelbase * kappa);
  }

  double kappaOfSteer(double steer_rad) const
  {
    return std::tan(steer_rad) / wheelbase;
  }

  // 반경 R 로 선회할 때 직사각 차체가 쓸고 가는 횡폭.
  // 정지 폭보다 넓다 (R 0.296 에서 1.30 배).
  double sweptWidth(double radius) const
  {
    const double r = std::max(radius, 1e-6);
    const double r_out = std::hypot(r + width * 0.5, rear_axle_to_front_bumper);
    const double r_in = r - width * 0.5;
    return r_out - r_in;
  }

  // 곡률 R 에서 차선 안에 남는 편측 여유.
  double lateralMargin(double radius, double track_width) const
  {
    return (track_width - sweptWidth(radius)) * 0.5;
  }
};

inline double clampAbs(double v, double limit)
{
  return std::max(-limit, std::min(limit, v));
}

// [-pi, pi) 로 정규화.
inline double wrapAngle(double a)
{
  while (a >= M_PI) {a -= 2.0 * M_PI;}
  while (a < -M_PI) {a += 2.0 * M_PI;}
  return a;
}

}  // namespace backup_common

#endif  // BACKUP_COMMON__VEHICLE_HPP_
