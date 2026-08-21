// ====================================================================
// pure_pursuit.hpp
//
// Pure Pursuit 조향 제어기.
// 원본: KAU_AMET_Test / src/kau_controller/test/controller.py
//
// 시뮬 상태에 의존하지 않는 순수 함수로 유지한다 (원본의 설계 의도).
// ROS 노드는 이 함수들을 호출만 하고, 여기에는 rclcpp 를 들이지 않는다.
//
// 단위: 길이 cm / 각도 deg / 속도 m/s
// ====================================================================

#ifndef KAU_CONTROL__PURE_PURSUIT_HPP_
#define KAU_CONTROL__PURE_PURSUIT_HPP_

#include <algorithm>
#include <cmath>

#include "kau_control/params.hpp"


namespace kau
{
namespace control
{
namespace pure_pursuit
{

// Ld = clamp(k_v * v, ld_min, ld_max) [cm]
inline double lookaheadDistance(double v_ms, const ControllerParams & p)
{
    const double ld = p.k_v * v_ms * CM_PER_M;

    return std::min(std::max(ld, p.ld_min), p.ld_max);
}


// 후륜축 기준 Pure Pursuit 조향각 [deg].
//
//     delta = atan(2 * L * sin(alpha) / Ld)
//     alpha = 차량 헤딩과 LookAhead 점 방향 사이의 각
//
// x, y, target_* 는 cm, yaw 는 rad, 반환은 deg (좌회전 +).
inline double steerCommand(
    double x,
    double y,
    double yaw,
    double target_x,
    double target_y,
    double ld,
    const VehicleParams & vehicle)
{
    if (ld < 1e-6)
    {
        return 0.0;
    }

    double alpha = std::atan2(target_y - y, target_x - x) - yaw;

    alpha = std::fmod(alpha + M_PI, 2.0 * M_PI);

    if (alpha < 0.0)
    {
        alpha += 2.0 * M_PI;
    }

    alpha -= M_PI;

    const double delta =
        std::atan2(2.0 * vehicle.wheelbase * std::sin(alpha), ld);

    return delta * 180.0 / M_PI;
}

}  // namespace pure_pursuit
}  // namespace control
}  // namespace kau

#endif  // KAU_CONTROL__PURE_PURSUIT_HPP_
