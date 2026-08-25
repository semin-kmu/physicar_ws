// ====================================================================
// steer_limiter.hpp -- 조향 슬루(변화율) 제한. ROS 의존성 없음.
//
// 왜 필요한가:
//
//  1. `VehicleParams::max_steer_rate` (600 deg/s) 는 kau_control 의
//     params.hpp 에 **차량 확정값으로 선언되어 있는데 어느 조향 제어기도
//     쓰고 있지 않다** (kau_control/src/steer_controller_node.cpp,
//     kau_control_lane/src/lane_steer_controller_node.cpp 둘 다 grep 으로
//     확인). 서보가 못 따라가는 명령을 내면 실제 조향각이 명령과 달라지고,
//     그만큼 Pure Pursuit 의 전제가 깨진다.
//
//  2. /path/drive 는 장애물 유무에 따라 차선(14Hz)과 회피 경로(5Hz)를
//     오간다. 두 경로는 서로 다른 곡선이라 전환 순간 lookahead 점이 튀고,
//     그대로 두면 조향이 계단처럼 꺾인다. 전환 직후 잠깐 상한을 더 조여
//     그 계단을 시간축으로 펴 준다.
//
// 2번은 근본 해법이 아니다 (경로 자체를 블렌딩하는 것이 맞다). 전환
// **횟수**는 kau_path_arbiter 의 히스테리시스가 줄이고, 여기서는 남은
// 전환의 **날카로움**만 줄인다.
// ====================================================================

#ifndef KAU_CONTROL_DRIVE__STEER_LIMITER_HPP_
#define KAU_CONTROL_DRIVE__STEER_LIMITER_HPP_

#include <algorithm>
#include <cmath>

namespace kau
{
namespace control_drive
{

struct SteerLimiterParams
{
    double max_steer_deg = 20.0;          // 확정값 (VehicleParams::max_steer)
    double max_steer_rate_dps = 600.0;    // 확정값 (VehicleParams::max_steer_rate)

    // 소스 전환 직후 적용할 더 조인 상한과 그 지속 시간.
    // 150 deg/s 는 0.4 초 동안 최대 60deg -- 전 범위(-20~+20, 40deg)를
    // 훑고도 남으므로 정상 조향을 막지 않으면서 계단만 편다.
    double transition_rate_dps = 150.0;
    double transition_sec = 0.4;
};

class SteerLimiter
{
public:
    explicit SteerLimiter(SteerLimiterParams p)
    : params_(p)
    {
    }

    // want_deg: Pure Pursuit 이 낸 원 명령. dt: 제어 주기 [s].
    // source_changed: 이번 tick 에 경로 소스가 바뀌었는가.
    // now_sec: 단조 증가 시각 [s].
    double apply(double want_deg, double dt, bool source_changed, double now_sec)
    {
        if (source_changed)
        {
            transition_until_ = now_sec + params_.transition_sec;
        }
        const double want = std::clamp(
            want_deg, -params_.max_steer_deg, params_.max_steer_deg);

        if (!(dt > 0.0))
        {
            return current_;
        }
        const bool in_transition = now_sec < transition_until_;
        const double rate = in_transition
            ? std::min(params_.transition_rate_dps, params_.max_steer_rate_dps)
            : params_.max_steer_rate_dps;

        const double step = rate * dt;
        current_ = std::clamp(want, current_ - step, current_ + step);
        return current_;
    }

    // 추종 실패 등으로 안전값(0)을 낼 때. 슬루를 거치지 않고 즉시 0 이다 --
    // 조향을 푸는 방향은 늦출 이유가 없고, 늦추면 위험한 쪽으로 남는다.
    double releaseToZero()
    {
        current_ = 0.0;
        transition_until_ = 0.0;
        return current_;
    }

    double current() const { return current_; }
    bool inTransition(double now_sec) const { return now_sec < transition_until_; }

private:
    SteerLimiterParams params_;
    double current_ = 0.0;
    double transition_until_ = 0.0;
};

}  // namespace control_drive
}  // namespace kau

#endif  // KAU_CONTROL_DRIVE__STEER_LIMITER_HPP_
