// ====================================================================
// params.hpp
//
// 차량 제원 / 제어기 / 속도 제어 parameter.
// 원본: KAU_AMET_Test / src/sim_common/config.py
//
// 단위: 길이 cm / 각도 deg / 속도 m/s / 곡률 1/cm
//       속도만 m/s 이므로 계산 시 cm/s 로 변환 (CM_PER_M)
// 좌표계: 2D 직교, yaw 는 +x 축 기준 CCW +, 조향각 좌회전 +
//
// 기본값은 시뮬 검증본과 동일. 실제 운용값은 config/*_controller.yaml 에서
// 덮어쓴다 (팀 규칙: 튜닝값은 YAML).
// ====================================================================

#ifndef KAU_CONTROL__PARAMS_HPP_
#define KAU_CONTROL__PARAMS_HPP_

#include <algorithm>
#include <cmath>


namespace kau
{
namespace control
{

constexpr double CM_PER_M = 100.0;


inline double deg2rad(double d)
{
    return d * M_PI / 180.0;
}


inline double rad2deg(double r)
{
    return r * 180.0 / M_PI;
}


// ====================================================================
// 차량 제원
//
// 휠베이스 18 cm / 최대 조향각 20 deg / 최대 조향 각속도 600 deg/s
//     -> 대회측 회신 (확정). physicar driver_params.yaml 의 wheelbase 0.18 m 와 일치.
// 조향 응답 시상수 0.1 s -> 미회신, 잠정값 (제어기는 쓰지 않고 참고용)
// ====================================================================

struct VehicleParams
{
    double wheelbase      = 18.0;    // cm, 확정

    double max_steer      = 20.0;    // deg, 확정

    double max_steer_rate = 600.0;   // deg/s, 확정

    // Pure Pursuit 기준점(후륜축 중심)이 base_frame 원점에서 떨어진 종방향 거리 [cm].
    //
    // PhysiCar URDF 는 base_link/base_footprint 를 **휠베이스 중앙**에 둔다
    // (뒷바퀴 x=-wheelbase/2, 앞바퀴 x=+wheelbase/2). 따라서 기본값이
    // -wheelbase/2 = -9 cm 이며, 이 보정을 빼먹으면 9 cm 앞을 후륜축으로
    // 착각해 코너에서 계속 안쪽으로 파고든다.
    double rear_axle_offset = -9.0;  // cm, base_frame 원점 기준 (뒤쪽이 음수)

    double r_min() const
    {
        return wheelbase / std::tan(deg2rad(max_steer));
    }

    double kappa_max() const
    {
        return 1.0 / r_min();
    }

    // delta = atan(L * kappa) [deg]
    double steer_of_kappa(double kappa) const
    {
        return rad2deg(std::atan(wheelbase * kappa));
    }

    // kappa = tan(delta) / L [1/cm]
    double kappa_of_steer(double steer_deg) const
    {
        return std::tan(deg2rad(steer_deg)) / wheelbase;
    }
};


// ====================================================================
// Pure Pursuit
//
// 기준점은 후륜축 중심 (cross track error 도 동일 기준점에서 측정).
// 시뮬에서 얻은 레퍼런스 튜닝값이 기본값이다.
// ====================================================================

struct ControllerParams
{
    double k_v    = 0.5;      // sec, Ld = k_v * v

    double ld_min = 30.0;     // cm

    double ld_max = 175.0;    // cm
};


// ====================================================================
// Speed Controller
//
// 전방 lookahead 구간의 최대 곡률로 목표 속도 결정.
//
//     window = [s0 + look_min, s0 + look_k * v]      look_k = look_max / v_max
//     v_ref  = law(kappa_win) 을 [v_min, v_max] 로 clamp -> 가감속 제한
//
// 실제 발행값은 speed_controller 가 v_ref 에 PID 보정을 더해 만든다.
//
// 기준점 2 개로 정의
//     kappa = 0                      -> v_max   (직선)
//     kappa = kappa_ref = m * k_max  -> v_min   (기하 최소회전반경의 margin m)
//
// law
//     sqrt   : v = sqrt(a_lat / kappa). 등횡가속도, 물리적 근거 있음.
//              a_lat 는 위 기준점에서 역산 (실측 확보 시 그 값으로 교체)
//     linear : v = v_max - (v_max - v_min) * kappa / kappa_ref
// ====================================================================

struct SpeedParams
{
    double v_min    = 0.5;    // m/s

    double v_max    = 2.0;    // m/s

    double look_min = 20.0;   // cm, window 하한 (고정)

    double look_max = 150.0;  // cm, v_max 에서의 window 상한

    double min_span = 20.0;   // cm, window 최소 폭 (저속 퇴화 방지)

    double margin   = 0.8;    // v_min 기준 곡률 = margin * kappa_max

    bool   use_sqrt = true;   // true: sqrt law, false: linear law

    double accel_max = 1.0;   // m/s^2, 목표속도 증가율 상한

    double decel_max = 2.0;   // m/s^2, 목표속도 감소율 상한 (감속은 넉넉히)

    // cm per (m/s). v_max 에서 look_max 가 되도록.
    double look_k() const
    {
        return look_max / v_max;
    }

    // v_min 이 나오는 곡률 [1/cm]. 기하 최소회전반경의 margin.
    double kappa_ref(const VehicleParams & vp) const
    {
        return margin * vp.kappa_max();
    }

    // [m/s^2]. 기준점에서 역산한 잠정값. kappa 는 1/m 로 환산.
    double a_lat(const VehicleParams & vp) const
    {
        return v_min * v_min * kappa_ref(vp) * CM_PER_M;
    }

    // (하한, 상한) [cm]. 차량 최근접점 기준 전방 호길이.
    void window(double v_ms, double & lo_out, double & hi_out) const
    {
        lo_out = look_min;

        hi_out = std::max(look_k() * v_ms, look_min + min_span);
    }

    // 목표속도 급변 방지. prev 에서 want 로 dt 동안 갈 수 있는 만큼만 간다.
    double rateLimit(double prev, double want, double dt) const
    {
        const double up = accel_max * dt;

        const double dn = decel_max * dt;

        return std::min(std::max(want, prev - dn), prev + up);
    }

    // 구간 최대 곡률 [1/cm] -> 목표 속도 [m/s].
    double target(double kappa_win, const VehicleParams & vp) const
    {
        const double k = std::abs(kappa_win);

        if (k < 1e-12)
        {
            return v_max;
        }

        double v = 0.0;

        if (use_sqrt)
        {
            v = std::sqrt(a_lat(vp) / (k * CM_PER_M));
        }
        else
        {
            const double r = k / kappa_ref(vp);

            v = v_max - (v_max - v_min) * r;
        }

        return std::min(std::max(v, v_min), v_max);
    }
};

}  // namespace control
}  // namespace kau

#endif  // KAU_CONTROL__PARAMS_HPP_
