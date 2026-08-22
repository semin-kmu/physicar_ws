// ====================================================================
// pid.hpp
//
// 속도 제어용 PID. 상태 3 개(적분·직전오차·초기화여부)뿐인 최소 구현.
//
// 적분 windup 은 적분항을 i_max 로 자르는 것으로 막는다. 속도 명령은
// 0 ~ v_max 로 좁게 clamp 되므로 이 정도면 충분하다.
//
// 단위: 오차 m/s, 출력 m/s (속도 명령 보정량)
// ====================================================================

#ifndef KAU_CONTROL__PID_HPP_
#define KAU_CONTROL__PID_HPP_

#include <algorithm>


namespace kau
{
namespace control
{

struct PidParams
{
    double kp      = 0.0;

    double ki      = 0.0;

    double kd      = 0.0;

    double i_max   = 0.5;   // 적분항 절대값 상한 [m/s]

    double out_max = 0.5;   // 보정량 절대값 상한 [m/s]
};


class Pid
{
public:
    void configure(const PidParams & p)
    {
        p_ = p;

        i_ = std::clamp(i_, -p_.i_max, p_.i_max);
    }

    void reset()
    {
        i_ = 0.0;

        prev_ = 0.0;

        have_prev_ = false;
    }

    double step(double err, double dt)
    {
        if (dt <= 0.0)
        {
            return 0.0;
        }

        i_ = std::clamp(i_ + err * dt, -p_.i_max, p_.i_max);

        const double d = have_prev_ ? (err - prev_) / dt : 0.0;

        prev_ = err;

        have_prev_ = true;

        const double u = p_.kp * err + p_.ki * i_ + p_.kd * d;

        return std::clamp(u, -p_.out_max, p_.out_max);
    }

private:
    PidParams p_;

    double i_ = 0.0;

    double prev_ = 0.0;

    bool have_prev_ = false;
};

}  // namespace control
}  // namespace kau

#endif  // KAU_CONTROL__PID_HPP_
